using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Policies;
using System.Collections.Generic;
using Unity.MLAgents.Sensors;
using System.Collections;
using System.Linq;

public enum Team
{
    Blue = 0,
    Purple = 1
}

public class AgentSoccer : Agent
{
    // Note that that the detectable tags are different for the blue and Red teams. The order is
    // * ball
    // * own goal
    // * opposing goal
    // * wall
    // * own teammate
    // * opposing player

    public enum Position
    {
        Striker,
        Goalie,
        Generic
    }

    [HideInInspector]
    public Team team;
    float m_KickPower;
    // The coefficient for the reward for colliding with a ball. Set using curriculum.
    float m_BallTouch;
    public Position position;

    const float k_Power = 2000f;
    float m_Existential;
    float m_LateralSpeed;
    float m_ForwardSpeed;


    [HideInInspector]
    public Rigidbody agentRb;
    SoccerSettings m_SoccerSettings;
    BehaviorParameters m_BehaviorParameters;
    public Vector3 initialPos;
    public float rotSign;

    EnvironmentParameters m_ResetParams;


    public float visionAngle; // Added vision angle

    private Queue<float[]> rayObservationMemory; //buffer for the past ray traces
    public const int memoryFrames = 8; // More frames to remember

    private Vector3? lastSeenBallPosition = null; // Nullable to indicate no data
    private float memoryDecayTime = 5f; // Time to remember ball position
    private float ballMemoryTimer = 0f; // Timer for how long ball memory is valid

    private MemoryRaySensor memorySensor;

    private Vector3? previousPosition;
    private const float MEMORY_REWARD_MULTIPLIER = 0.05f;
    private const float MEMORY_MOVEMENT_REWARD_MULTIPLIER = 0.02f;

    private readonly RaycastHit[] raycastHits = new RaycastHit[1]; // Cache RaycastHit array
    private List<float> combinedObservations; // Pre-allocate list
    private Vector3[] rayDirections; // Cache ray directions
    public const int NUM_RAYS = 11;

    // Add field for action buffers
    private ActionBuffers m_StoredActionBuffers;

    private const float RAY_LENGTH = 20f; // Longer sight distance

    // Add new constants for memory-based rewards
    private const float LOOK_AT_BALL_REWARD = 0.01f;  // Reward for looking towards remembered ball
    private const float VISION_ANGLE_THRESHOLD = 30f;  // Degrees within which we consider "looking at" the ball

    // Memory structure to store object positions
    private class DynamicObjectMemory
    {
        public Vector3 position;        // Current position
        public Vector3 velocity;        // Current velocity
        public Vector3 predictedPosition; // Position + (velocity * time)
        public string tag;             // Object type
        public float lastSeenTime;     // When we last saw it
        public float confidence;       // How confident we are in the prediction
    }

    private List<DynamicObjectMemory> dynamicObjects = new List<DynamicObjectMemory>();

    private float[] GenerateRayObservations()
    {
        float[] rayObservations = new float[NUM_RAYS];
        bool ballSeen = false;
        
        // Update existing memories
        for (int i = dynamicObjects.Count - 1; i >= 0; i--)
        {
            dynamicObjects[i].confidence -= Time.deltaTime / memoryDecayTime;
            if (dynamicObjects[i].confidence <= 0)
            {
                dynamicObjects.RemoveAt(i);
            }
        }
        
        for (int i = 0; i < NUM_RAYS; i++)
        {
            Vector3 direction = rayDirections[i];
            Vector3 rayDirection = transform.TransformDirection(direction);
            
            int hitCount = Physics.RaycastNonAlloc(
                transform.position,
                rayDirection,
                raycastHits,
                RAY_LENGTH);

            if (hitCount > 0)
            {
                rayObservations[i] = raycastHits[0].distance / RAY_LENGTH;
                
                string tag = raycastHits[0].collider.tag;
                Vector3 hitPoint = raycastHits[0].point;
                
                // Update memory
                bool memoryExists = false;
                foreach (var memory in dynamicObjects)
                {
                    if (memory.tag == tag && Vector3.Distance(memory.position, hitPoint) < 1f)
                    {
                        memory.position = hitPoint;
                        memory.confidence = 1.0f;
                        memoryExists = true;
                        break;
                    }
                }

                if (!memoryExists && (tag == "ball" || tag.Contains("Agent")))
                {
                    dynamicObjects.Add(new DynamicObjectMemory 
                    { 
                        position = hitPoint,
                        tag = tag,
                        confidence = 1.0f
                    });
                }

                // Draw the ray with appropriate color based on what was hit
                DrawRayWithMemory(transform.position, rayDirection, raycastHits[0].distance, tag);

                if (tag == "ball")
                {
                    ballSeen = true;
                    lastSeenBallPosition = hitPoint;
                    ballMemoryTimer = memoryDecayTime;
                }
            }
            else
            {
                rayObservations[i] = 1.0f;
                // Draw ray that didn't hit anything
                DrawRayWithMemory(transform.position, rayDirection, RAY_LENGTH, "none");
            }
        }

        // Update ball memory if we didn't see the ball this frame
        if (!ballSeen && lastSeenBallPosition.HasValue)
        {
            ballMemoryTimer -= Time.deltaTime;
            if (ballMemoryTimer <= 0f)
            {
                lastSeenBallPosition = null;
            }
        }

        return rayObservations;
    }

    #if UNITY_EDITOR
    private void DrawWireSphere(Vector3 pos, float radius, Color color)
    {
        int segments = 16;
        float angle = 360f / segments;
        
        // Draw three circles in different planes
        for (int i = 0; i < 3; i++)
        {
            Vector3 lastPoint = Vector3.zero;
            for (int j = 0; j <= segments; j++)
            {
                float rad = Mathf.Deg2Rad * angle * j;
                Vector3 point;
                switch (i)
                {
                    case 0: // XY plane
                        point = new Vector3(Mathf.Cos(rad), Mathf.Sin(rad), 0);
                        break;
                    case 1: // XZ plane
                        point = new Vector3(Mathf.Cos(rad), 0, Mathf.Sin(rad));
                        break;
                    default: // YZ plane
                        point = new Vector3(0, Mathf.Cos(rad), Mathf.Sin(rad));
                        break;
                }
                point = pos + point * radius;
                
                if (j > 0)
                    Debug.DrawLine(lastPoint, point, color, 0.1f);
                lastPoint = point;
            }
        }
    }
    #endif

    public override void Initialize()
    {
        // Get BehaviorParameters
        m_BehaviorParameters = GetComponent<BehaviorParameters>();
        if (m_BehaviorParameters == null)
        {
            Debug.LogError("BehaviorParameters not found!");
            return;
        }

        // Important: Set UseChildSensors to true to allow sensor components to be found
        m_BehaviorParameters.UseChildSensors = true;

        // Initialize action buffers first
        try
        {
            var discreteActionSpec = m_BehaviorParameters.BrainParameters.ActionSpec.BranchSizes;
            m_StoredActionBuffers = new ActionBuffers(
                new float[0],  // Continuous actions (none in this case)
                new int[discreteActionSpec.Length]  // Discrete actions
            );
        }
        catch (System.Exception e)
        {
            Debug.LogError($"Failed to initialize action buffers: {e}");
            m_StoredActionBuffers = new ActionBuffers(
                new float[0],
                new int[3]  // Default size for discrete actions
            );
        }

        // Pre-calculate ray directions
        rayDirections = new Vector3[NUM_RAYS];
        float angleStep = 120f / (NUM_RAYS - 1);
        float startAngle = -60f;
        for (int i = 0; i < NUM_RAYS; i++)
        {
            float angle = startAngle + (i * angleStep);
            rayDirections[i] = Quaternion.Euler(0, angle, 0) * Vector3.forward;
        }

        // Initialize memory
        rayObservationMemory = new Queue<float[]>(memoryFrames);
        float[] initialObservation = GenerateRayObservations();
        for (int i = 0; i < memoryFrames; i++)
        {
            rayObservationMemory.Enqueue(initialObservation);
        }

        // Initialize MemoryRaySensorComponent
        var sensorComponent = GetComponent<MemoryRaySensorComponent>();
        if (sensorComponent == null)
        {
            // If the component doesn't exist, add it
            sensorComponent = gameObject.AddComponent<MemoryRaySensorComponent>();
            Debug.Log("Added MemoryRaySensorComponent to agent");
        }
        
        // Calculate correct observation size
        int rayObsSize = NUM_RAYS;  // 11
        int totalRayMemorySize = rayObsSize * (memoryFrames + 1);  // 11 * 9 = 99
        int visionAngleSize = 1;  // 1
        int maxMemories = 5;  // Changed back to 5 memories
        int memoryFeatures = 5;  // Changed back to 5 features (including tag)
        int totalObservations = totalRayMemorySize + visionAngleSize + (maxMemories * memoryFeatures);  // 99 + 1 + 25 = 125

        // Initialize the sensor component with correct size
        sensorComponent.Init(this, totalObservations);
        
        // Force sensor creation
        var sensors = sensorComponent.CreateSensors();
        Debug.Log($"Created {sensors.Length} sensors");

        // Initialize other components
        SoccerEnvController envController = GetComponentInParent<SoccerEnvController>();
        if (envController != null)
        {
            m_Existential = 1f / envController.MaxEnvironmentSteps;
        }
        else
        {
            m_Existential = 1f / MaxStep;
        }

        // Set team-specific parameters
        if (m_BehaviorParameters.TeamId == (int)Team.Blue)
        {
            team = Team.Blue;
            initialPos = new Vector3(transform.position.x - 5f, .5f, transform.position.z);
            rotSign = 1f;
        }
        else
        {
            team = Team.Purple;
            initialPos = new Vector3(transform.position.x + 5f, .5f, transform.position.z);
            rotSign = -1f;
        }

        // Set position-specific parameters
        if (position == Position.Goalie)
        {
            m_LateralSpeed = 1.0f;
            m_ForwardSpeed = 1.0f;
        }
        else if (position == Position.Striker)
        {
            m_LateralSpeed = 0.3f;
            m_ForwardSpeed = 1.3f;
        }
        else
        {
            m_LateralSpeed = 0.3f;
            m_ForwardSpeed = 1.0f;
        }

        // Initialize remaining fields
        m_SoccerSettings = FindObjectOfType<SoccerSettings>();
        agentRb = GetComponent<Rigidbody>();
        agentRb.maxAngularVelocity = 500;

        m_ResetParams = Academy.Instance.EnvironmentParameters;
        if (m_ResetParams == null)
        {
            Debug.LogWarning("EnvironmentParameters not found, using default ball_touch value of 0");
        }

        visionAngle = 0f;

        // Pre-allocate combined observations list
        combinedObservations = new List<float>((NUM_RAYS * (memoryFrames + 1)) + 1 + 4);
    }

    protected override void OnDisable()
    {
        try 
        {
            // Only try to clear if we have valid action buffers
            if (m_StoredActionBuffers.DiscreteActions.Array != null)
            {
                m_StoredActionBuffers.Clear();
            }
        }
        catch (System.Exception e)
        {
            Debug.LogWarning($"Error during OnDisable: {e}");
        }
        finally 
        {
            base.OnDisable();
        }
    }

    // Add this method to ensure action buffers are always valid
    private void EnsureActionBuffersInitialized()
    {
        if (m_StoredActionBuffers.DiscreteActions.Array == null)
        {
            if (m_BehaviorParameters != null)
            {
                var discreteActionSpec = m_BehaviorParameters.BrainParameters.ActionSpec.BranchSizes;
                m_StoredActionBuffers = new ActionBuffers(
                    new float[0],
                    new int[discreteActionSpec.Length]
                );
            }
            else
            {
                m_StoredActionBuffers = new ActionBuffers(
                    new float[0],
                    new int[3]  // Default size for discrete actions
                );
            }
        }
    }

    // Helper methods for the sensor
    public Queue<float[]> GetRayMemory()
    {
        return rayObservationMemory;
    }

    public float[] GenerateCurrentRayObservations()
    {
        return GenerateRayObservations();
    }

    public float GetVisionAngle()
    {
        return visionAngle;
    }

    public List<(Vector3 position, Vector3 velocity, Vector3 prediction, string tag, float confidence)> GetDynamicObjects()
    {
        float currentTime = Time.time;
        return dynamicObjects.ConvertAll(m => 
        {
            float timeSinceLastSeen = currentTime - m.lastSeenTime;
            // Update prediction based on time passed
            Vector3 currentPrediction = m.position + (m.velocity * timeSinceLastSeen);
            // Confidence decreases with time
            float currentConfidence = Mathf.Max(0, 1 - (timeSinceLastSeen / memoryDecayTime));
            
            return (m.position, m.velocity, currentPrediction, m.tag, currentConfidence);
        });
    }

    public void MoveAgent(ActionSegment<int> act)
    {
        var dirToGo = Vector3.zero;
        var rotateDir = Vector3.zero;

        m_KickPower = 0f;

        var forwardAxis = act[0];
        var rightAxis = act[1];
        var rotateAxis = act[2];

        switch (forwardAxis)
        {
            case 1:
                dirToGo = transform.forward * m_ForwardSpeed;
                m_KickPower = 1f;
                break;
            case 2:
                dirToGo = transform.forward * -m_ForwardSpeed;
                break;
        }

        switch (rightAxis)
        {
            case 1:
                dirToGo = transform.right * m_LateralSpeed;
                break;
            case 2:
                dirToGo = transform.right * -m_LateralSpeed;
                break;
        }

        switch (rotateAxis)
        {
            case 1:
                rotateDir = transform.up * -1f;
                break;
            case 2:
                rotateDir = transform.up * 1f;
                break;
        }

        // Add reward for looking towards remembered ball position
        if (lastSeenBallPosition.HasValue && ballMemoryTimer > 0)
        {
            Vector3 directionToBall = (lastSeenBallPosition.Value - transform.position).normalized;
            float angleToMemory = Vector3.Angle(transform.forward, directionToBall);
            
            // If we're turning towards the ball
            if (angleToMemory < VISION_ANGLE_THRESHOLD)
            {
                // More reward the more directly we look at it
                float alignmentReward = 1f - (angleToMemory / VISION_ANGLE_THRESHOLD);
                AddReward(LOOK_AT_BALL_REWARD * alignmentReward * (ballMemoryTimer / memoryDecayTime));
                
                #if UNITY_EDITOR
                // Visual debug - yellow ray when looking at remembered position
                Debug.DrawRay(transform.position, transform.forward * 2f, Color.yellow, 0.1f);
                #endif
            }
        }

        // Apply movement
        agentRb.AddForce(dirToGo * m_SoccerSettings.agentRunSpeed,
            ForceMode.VelocityChange);
        transform.Rotate(rotateDir, Time.fixedDeltaTime * 100f);
    }

    // Change from private to public
    public class PositionHistory
    {
        public Vector3 currentPosition;
        public Vector3 previousPosition;
        public string tag;  // For identifying team
        public float lastUpdateTime;

        public Vector3 GetVelocity()
        {
            float deltaTime = Time.time - lastUpdateTime;
            if (deltaTime > 0)
            {
                return (currentPosition - previousPosition) / deltaTime;
            }
            return Vector3.zero;
        }

        public void UpdatePosition(Vector3 newPosition)
        {
            previousPosition = currentPosition;
            currentPosition = newPosition;
            lastUpdateTime = Time.time;
        }
    }

    private Dictionary<GameObject, PositionHistory> positionHistories = new Dictionary<GameObject, PositionHistory>();
    private const int HISTORY_BUFFER_SIZE = 30;  // Store 30 frames of history

    private void UpdatePositionHistory(GameObject obj)
    {
        if (!positionHistories.ContainsKey(obj))
        {
            positionHistories[obj] = new PositionHistory
            {
                currentPosition = obj.transform.position,
                previousPosition = obj.transform.position,
                tag = obj.tag,
                lastUpdateTime = Time.time
            };
        }
        else
        {
            positionHistories[obj].UpdatePosition(obj.transform.position);
        }
    }

    private void CleanupOldHistories()
    {
        float currentTime = Time.time;
        var keysToRemove = positionHistories.Keys
            .Where(k => currentTime - positionHistories[k].lastUpdateTime > 5f)
            .ToList();
        
        foreach (var key in keysToRemove)
        {
            positionHistories.Remove(key);
        }
    }

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        // Move agent based on actions
        MoveAgent(actionBuffers.DiscreteActions);

        // Update position histories
        UpdatePositionHistory(gameObject);  // Self
        
        // Ball
        GameObject ball = GameObject.FindGameObjectWithTag("ball");
        if (ball != null)
        {
            UpdatePositionHistory(ball);
        }

        // Other players
        var players = GameObject.FindGameObjectsWithTag("blueAgent")
            .Concat(GameObject.FindGameObjectsWithTag("purpleAgent"))
            .Where(p => p != gameObject);  // Exclude self
        
        foreach (var player in players)
        {
            UpdatePositionHistory(player);
        }

        // Clean up old histories periodically
        CleanupOldHistories();

        // Only reward for kicking towards goal and goalie position
        if (position == Position.Goalie)
        {
            // Reward goalie for staying near goal
            GameObject ownGoal = GameObject.FindGameObjectWithTag(team == Team.Blue ? "blueGoal" : "purpleGoal");
            if (ownGoal != null)
            {
                float distanceToGoal = Vector3.Distance(transform.position, ownGoal.transform.position);
                float optimalDistance = 5f;
                float positionReward = Mathf.Exp(-(distanceToGoal - optimalDistance) * (distanceToGoal - optimalDistance) / 10f) / 5000f;
                AddReward(positionReward);
            }
        }
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var discreteActionsOut = actionsOut.DiscreteActions;
        //forward
        if (Input.GetKey(KeyCode.W))
        {
            discreteActionsOut[0] = 1;
        }
        if (Input.GetKey(KeyCode.S))
        {
            discreteActionsOut[0] = 2;
        }
        //rotate
        if (Input.GetKey(KeyCode.A))
        {
            discreteActionsOut[2] = 1;
        }
        if (Input.GetKey(KeyCode.D))
        {
            discreteActionsOut[2] = 2;
        }
        //right
        if (Input.GetKey(KeyCode.E))
        {
            discreteActionsOut[1] = 1;
        }
        if (Input.GetKey(KeyCode.Q))
        {
            discreteActionsOut[1] = 2;
        }
    }
    /// <summary>
    /// Used to provide a "kick" to the ball.
    /// </summary>
    void OnCollisionEnter(Collision c)
    {
        var force = k_Power * m_KickPower;
        if (position == Position.Goalie)
        {
            force = k_Power;
        }
        if (c.gameObject.CompareTag("ball"))
        {
            // Get kick direction
            var dir = c.contacts[0].point - transform.position;
            dir = dir.normalized;
            
            // Get goal positions
            GameObject ownGoal = GameObject.FindGameObjectWithTag(team == Team.Blue ? "blueGoal" : "purpleGoal");
            GameObject opponentGoal = GameObject.FindGameObjectWithTag(team == Team.Blue ? "purpleGoal" : "blueGoal");
            
            if (opponentGoal != null && ownGoal != null)
            {
                if (position != Position.Goalie)  // Strikers
                {
                    // Direction to opponent goal
                    Vector3 dirToGoal = (opponentGoal.transform.position - c.contacts[0].point).normalized;
                    float kickAlignment = Vector3.Dot(dir, dirToGoal);  // 1 = towards opponent goal
                    
                    // Reward for kicking towards opponent goal (0.001 max)
                    AddReward(kickAlignment * 0.001f);
                }
                else  // Goalies
                {
                    // Direction AWAY from own goal
                    Vector3 dirAwayFromGoal = (c.contacts[0].point - ownGoal.transform.position).normalized;
                    float kickAlignment = Vector3.Dot(dir, dirAwayFromGoal);  // 1 = away from own goal
                    
                    // Reward for kicking away from own goal (0.001 max)
                    AddReward(kickAlignment * 0.001f);
                }
            }

            // Apply the kick force
            c.gameObject.GetComponent<Rigidbody>().AddForce(dir * force);
        }
    }

    public override void OnEpisodeBegin()
    {
        if (m_ResetParams != null)
        {
            m_BallTouch = m_ResetParams.GetWithDefault("ball_touch", 0);
        }
        else
        {
            m_BallTouch = 0;
            Debug.LogWarning("m_ResetParams is null in OnEpisodeBegin, using default ball_touch value of 0");
        }

        // Ensure action buffers are initialized
        EnsureActionBuffersInitialized();
    }

    // Override CollectObservations to prevent default vector observations
    public override void CollectObservations(VectorSensor sensor)
    {
        // Empty - all observations are handled by MemoryRaySensor
    }

    private void UpdateDynamicObject(string tag, Vector3 newPosition, float deltaTime)
    {
        var memory = dynamicObjects.Find(m => m.tag == tag);
        
        if (memory != null)
        {
            // Update velocity
            Vector3 newVelocity = (newPosition - memory.position) / deltaTime;
            // Use exponential smoothing for velocity to reduce noise
            memory.velocity = Vector3.Lerp(memory.velocity, newVelocity, 0.5f);
            
            // Update position
            memory.position = newPosition;
            
            // Update prediction
            memory.predictedPosition = newPosition + (memory.velocity * deltaTime);
            
            memory.lastSeenTime = Time.time;
            memory.confidence = 1.0f;
        }
        else if (tag == "ball" || tag.Contains("Agent")) // Only track dynamic objects
        {
            dynamicObjects.Add(new DynamicObjectMemory
            {
                position = newPosition,
                velocity = Vector3.zero,
                predictedPosition = newPosition,
                tag = tag,
                lastSeenTime = Time.time,
                confidence = 1.0f
            });
        }
    }

    #if UNITY_EDITOR
    private void OnDrawGizmos()
    {
        // Draw current field of view
        Gizmos.color = new Color(1f, 1f, 1f, 0.1f);
        DrawFieldOfView();

        // Draw dynamic object tracking
        foreach (var obj in dynamicObjects)
        {
            // Current position
            Gizmos.color = GetObjectColor(obj.tag, 1.0f);
            Gizmos.DrawWireSphere(obj.position, 0.2f);

            // Predicted position
            Gizmos.color = GetObjectColor(obj.tag, obj.confidence * 0.5f);
            Gizmos.DrawWireSphere(obj.predictedPosition, 0.3f);

            // Velocity vector
            Gizmos.color = Color.yellow * obj.confidence;
            Gizmos.DrawLine(obj.position, obj.position + obj.velocity);

            // Historical positions (trail)
            if (positionHistories.TryGetValue(GameObject.FindGameObjectWithTag(obj.tag), out var history))
            {
                Gizmos.color = GetObjectColor(obj.tag, 0.3f);
                Gizmos.DrawLine(history.previousPosition, history.currentPosition);
            }
        }
    }

    private Color GetObjectColor(string tag, float alpha)
    {
        switch (tag)
        {
            case "ball":
                return new Color(1f, 0.5f, 0f, alpha); // Orange
            case "blueAgent":
                return new Color(0f, 0.5f, 1f, alpha); // Blue
            case "purpleAgent":
                return new Color(0.8f, 0f, 0.8f, alpha); // Purple
            default:
                return new Color(0.5f, 0.5f, 0.5f, alpha); // Gray
        }
    }

    private void DrawFieldOfView()
    {
        float fovAngle = 120f;
        float viewDistance = 20f;
        int segments = 20;

        Vector3 forward = transform.forward;
        Vector3 position = transform.position;

        float startAngle = -fovAngle / 2;
        float angleStep = fovAngle / segments;

        for (int i = 0; i <= segments; i++)
        {
            float angle = startAngle + i * angleStep;
            Vector3 direction = Quaternion.Euler(0, angle, 0) * forward;
            Gizmos.DrawLine(position, position + direction * viewDistance);
        }
    }
    #endif

    // Add this method to get position history
    public PositionHistory GetPositionHistory(GameObject obj)
    {
        if (positionHistories.TryGetValue(obj, out var history))
        {
            return history;
        }
        return null;
    }

    private void DrawRayWithMemory(Vector3 origin, Vector3 direction, float distance, string hitTag)
    {
        #if UNITY_EDITOR
        // Current ray (solid colors)
        Color rayColor;
        switch (hitTag)
        {
            case "ball":
                rayColor = new Color(0f, 0f, 1f, 1f);  // Solid blue for current ball hit
                break;
            case "blueAgent":
            case "purpleAgent":
                // Green for teammates, red for opponents
                if ((hitTag == "blueAgent" && team == Team.Blue) || 
                    (hitTag == "purpleAgent" && team == Team.Purple))
                {
                    rayColor = new Color(0f, 1f, 0f, 1f);  // Solid green for current teammate hit
                }
                else
                {
                    rayColor = new Color(1f, 0f, 0f, 1f);  // Solid red for current opponent hit
                }
                break;
            default:
                rayColor = new Color(1f, 1f, 1f, 0.1f);  // Faint white for no hit or other objects
                break;
        }
        Debug.DrawRay(origin, direction * distance, rayColor, 0.1f);

        // Memory trails (faint colors)
        foreach (var memory in dynamicObjects)
        {
            if (memory.tag == hitTag)
            {
                float memoryAlpha = memory.confidence * 0.3f;  // Faint for memories
                Color memoryColor;
                
                switch (memory.tag)
                {
                    case "ball":
                        memoryColor = new Color(0f, 0f, 1f, memoryAlpha);  // Faint blue for remembered ball
                        break;
                    case "blueAgent":
                    case "purpleAgent":
                        // Faint green for teammates, faint red for opponents
                        if ((memory.tag == "blueAgent" && team == Team.Blue) || 
                            (memory.tag == "purpleAgent" && team == Team.Purple))
                        {
                            memoryColor = new Color(0f, 1f, 0f, memoryAlpha);  // Faint green
                        }
                        else
                        {
                            memoryColor = new Color(1f, 0f, 0f, memoryAlpha);  // Faint red
                        }
                        break;
                    default:
                        continue;  // Skip other objects
                }
                
                Debug.DrawLine(origin, memory.position, memoryColor, 0.1f);
            }
        }
        #endif
    }

}

