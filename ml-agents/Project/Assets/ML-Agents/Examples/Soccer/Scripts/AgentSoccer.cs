using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Policies;
using System.Collections.Generic;
using Unity.MLAgents.Sensors;
using System.Collections;

public enum Team
{
    Blue = 0,
    Purple = 1
}

// Add this class at the top of the file, outside the AgentSoccer class
public class MemoryRaySensor : ISensor
{
    private string name;
    private AgentSoccer agent;
    private int expectedObservations;

    public MemoryRaySensor(string name, AgentSoccer agent, int expectedObservations)
    {
        this.name = name;
        this.agent = agent;
        this.expectedObservations = expectedObservations;
    }

    public string GetName()
    {
        return name;
    }

    public ObservationSpec GetObservationSpec()
    {
        return ObservationSpec.Vector(expectedObservations);
    }

    public byte[] GetCompressedObservation()
    {
        return null;
    }

    public int Write(ObservationWriter writer)
    {
        var index = 0;
        int frameCount = 0;
        
        // Write ray observations from memory
        foreach (var pastObservation in agent.GetRayMemory())
        {
            frameCount++;
            float sum = 0;
            foreach (float obs in pastObservation)
            {
                writer[index++] = obs;
                sum += obs;
            }
            Debug.Log($"Memory Frame {frameCount}: Average distance = {sum/11:F2}");
        }

        // Write current observations
        var currentObs = agent.GenerateCurrentRayObservations();
        float currentSum = 0;
        foreach (float obs in currentObs)
        {
            writer[index++] = obs;
            currentSum += obs;
        }
        Debug.Log($"Current Frame: Average distance = {currentSum/11:F2}");

        // Write vision angle
        writer[index++] = agent.GetVisionAngle() / 360.0f;
        Debug.Log($"Vision Angle: {agent.GetVisionAngle():F2}°");

        // Write ball information
        var ballInfo = agent.GetBallInfo();
        writer[index++] = ballInfo.position.x;
        writer[index++] = ballInfo.position.y;
        writer[index++] = ballInfo.position.z;
        writer[index++] = ballInfo.memoryTimer;
        Debug.Log($"Ball Position: {ballInfo.position}, Memory Timer: {ballInfo.memoryTimer:F2}");

        return index;
    }

    public void Update() {}
    public void Reset() {}

    public CompressionSpec GetCompressionSpec()
    {
        return CompressionSpec.Default();
    }
}

// Add this class before AgentSoccer class
public class MemoryRaySensorComponent : SensorComponent 
{
    private MemoryRaySensor sensor;
    private AgentSoccer agent;
    private int observationSize;

    public void Init(AgentSoccer agent, int observationSize)
    {
        this.agent = agent;
        this.observationSize = observationSize;
    }

    public override ISensor[] CreateSensors()
    {
        sensor = new MemoryRaySensor("MemoryRaySensor", agent, observationSize);
        return new ISensor[] { sensor };
    }
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
    private const int memoryFrames = 5; //the number of frames to remember

    private Vector3? lastSeenBallPosition = null; // Nullable to indicate no data
    private float memoryDecayTime = 2f; // Time to remember ball position
    private float ballMemoryTimer = 0f; // Timer for how long ball memory is valid

    private MemoryRaySensor memorySensor;

    private float[] GenerateRayObservations()
    {
        int numRays = 11;
        float[] rayObservations = new float[numRays];
        float angleStep = 120f / (numRays - 1);
        float startAngle = -60f;

        bool ballSeen = false;
        float closestDistance = 10f;

        for (int i = 0; i < numRays; i++)
        {
            float angle = startAngle + (i * angleStep) + visionAngle;
            Vector3 direction = Quaternion.Euler(0, angle, 0) * transform.forward;

            if (Physics.Raycast(transform.position, direction, out RaycastHit hit, 10f))
            {
                rayObservations[i] = hit.distance;
                if (hit.collider.CompareTag("ball"))
                {
                    ballSeen = true;
                    closestDistance = Mathf.Min(closestDistance, hit.distance);
                    lastSeenBallPosition = hit.point;
                    ballMemoryTimer = memoryDecayTime;
                    Debug.Log($"Ball detected at ray {i}, distance: {hit.distance:F2}");
                }
            }
            else
            {
                rayObservations[i] = 10f;
            }

            // Make rays visible longer for debugging
            Color rayColor = ballSeen ? Color.green : Color.red;
            Debug.DrawRay(transform.position, direction * rayObservations[i], rayColor, 0.1f);
        }

        if (ballSeen)
        {
            Debug.Log($"Ball in view! Closest distance: {closestDistance:F2}");
        }
        else
        {
            ballMemoryTimer -= Time.deltaTime;
            if (ballMemoryTimer <= 0f)
            {
                lastSeenBallPosition = null;
                Debug.Log("Ball memory expired");
            }
            else if (lastSeenBallPosition.HasValue)
            {
                Debug.Log($"Using ball memory. Time remaining: {ballMemoryTimer:F2}s");
            }
        }

        return rayObservations;
    }



    public override void Initialize()
    {
        m_BehaviorParameters = gameObject.GetComponent<BehaviorParameters>();
        if (m_BehaviorParameters == null)
        {
            Debug.LogError("BehaviorParameters component not found!");
            return;
        }

        Debug.Log($"Initial observation size: {m_BehaviorParameters.BrainParameters.VectorObservationSize}");

        int expectedObsSize = (11 * (memoryFrames + 1)) + 1 + 4; // rays * frames + vision angle + ball info
        if (m_BehaviorParameters.BrainParameters.VectorObservationSize != expectedObsSize)
        {
            Debug.LogWarning($"Updating observation size from {m_BehaviorParameters.BrainParameters.VectorObservationSize} to {expectedObsSize}");
            m_BehaviorParameters.BrainParameters.VectorObservationSize = expectedObsSize;
        }

        rayObservationMemory = new Queue<float[]>(memoryFrames);
        float[] initialObservation = GenerateRayObservations();
        for (int i = 0; i < memoryFrames; i++)
        {
            rayObservationMemory.Enqueue(initialObservation);
        }

        SoccerEnvController envController = GetComponentInParent<SoccerEnvController>();
        if (envController != null)
        {
            m_Existential = 1f / envController.MaxEnvironmentSteps;
        }
        else
        {
            m_Existential = 1f / MaxStep;
        }

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
        m_SoccerSettings = FindObjectOfType<SoccerSettings>();
        agentRb = GetComponent<Rigidbody>();
        agentRb.maxAngularVelocity = 500;

        m_ResetParams = Academy.Instance.EnvironmentParameters;


        visionAngle = 0f; // Initialize vision angle
        
        // Create and add our custom sensor component
        var sensorComponent = gameObject.AddComponent<MemoryRaySensorComponent>();
        sensorComponent.Init(this, expectedObsSize);

        Debug.Log($"Added custom sensor with {expectedObsSize} observations");
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

    public (Vector3 position, float memoryTimer) GetBallInfo()
    {
        return (
            lastSeenBallPosition ?? Vector3.zero,
            lastSeenBallPosition.HasValue ? ballMemoryTimer / memoryDecayTime : 0f
        );

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

        transform.Rotate(rotateDir, Time.deltaTime * 100f);
        agentRb.AddForce(dirToGo * m_SoccerSettings.agentRunSpeed,
            ForceMode.VelocityChange);
    }

    public override void OnActionReceived(ActionBuffers actionBuffers)

    {

        if (position == Position.Goalie)
        {
            // Existential bonus for Goalies.
            AddReward(m_Existential);
        }
        else if (position == Position.Striker)
        {
            // Existential penalty for Strikers
            AddReward(-m_Existential);
        }
        MoveAgent(actionBuffers.DiscreteActions);

        float[] currentObservations = GenerateRayObservations();

        //below is what I added, when action received, it should store the frames while dequeuing the older frames
        if (rayObservationMemory.Count >= memoryFrames)
        {
            rayObservationMemory.Dequeue(); //removing the oldest frame from the stack
        }
        rayObservationMemory.Enqueue(currentObservations);//new frame

        //this could be wrong 
        List<float> combinedObservations = new List<float>();
        foreach (var pastObservation in rayObservationMemory)
        {
            combinedObservations.AddRange(pastObservation);
        }
        combinedObservations.AddRange(currentObservations);

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
            AddReward(.2f * m_BallTouch);
            var dir = c.contacts[0].point - transform.position;
            dir = dir.normalized;
            c.gameObject.GetComponent<Rigidbody>().AddForce(dir * force);
        }
    }

    public override void OnEpisodeBegin()
    {
        m_BallTouch = m_ResetParams.GetWithDefault("ball_touch", 0);
    }

}
