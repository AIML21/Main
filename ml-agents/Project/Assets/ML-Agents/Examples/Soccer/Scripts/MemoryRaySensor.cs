using UnityEngine;
using Unity.MLAgents.Sensors;
using System.Collections.Generic;
using System.Linq;

public class MemoryRaySensor : ISensor
{
    private const float fieldSize = 50f;  // Size of the soccer field
    private const float maxBallSpeed = 30f;  // Maximum possible ball speed
    private GameObject ownGoal;
    private GameObject opponentGoal;
    private Transform transform;

    public struct RayPerceptionSettings
    {
        public int RaysPerDirection;
        public float MaxRayDegrees;
        public float RayLength;
        public List<string> DetectableTags;
        public LayerMask RayLayerMask;
    }

    private string name;
    private AgentSoccer agent;
    private int expectedObservations;
    private RayPerceptionSettings raySettings;
    private Vector3[] rayDirections;

    public MemoryRaySensor(
        string name, 
        AgentSoccer agent, 
        int expectedObservations, 
        RayPerceptionSettings raySettings)
    {
        this.name = name;
        this.agent = agent;
        this.expectedObservations = expectedObservations;
        this.raySettings = raySettings;
        this.transform = agent.transform;

        // Get goal references based on team
        string ownGoalTag = agent.team == Team.Blue ? "blueGoal" : "purpleGoal";
        string opponentGoalTag = agent.team == Team.Blue ? "purpleGoal" : "blueGoal";
        
        ownGoal = GameObject.FindGameObjectWithTag(ownGoalTag);
        opponentGoal = GameObject.FindGameObjectWithTag(opponentGoalTag);

        if (ownGoal == null || opponentGoal == null)
        {
            Debug.LogError("Could not find goal objects!");
        }

        // Pre-calculate ray directions
        rayDirections = new Vector3[raySettings.RaysPerDirection];
        float angleStep = raySettings.MaxRayDegrees / (raySettings.RaysPerDirection - 1);
        float startAngle = -raySettings.MaxRayDegrees / 2f;
        for (int i = 0; i < raySettings.RaysPerDirection; i++)
        {
            float angle = startAngle + (i * angleStep);
            rayDirections[i] = Quaternion.Euler(0, angle, 0) * Vector3.forward;
        }
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

    private (float angle, float distance) ToPolar(Vector3 position, Transform reference)
    {
        Vector3 direction = position - reference.position;
        float distance = direction.magnitude / fieldSize;  // Normalized distance
        
        // Get angle relative to agent's forward direction (not global)
        float angle = Vector3.SignedAngle(reference.forward, direction, Vector3.up) / 180f;  
        // angle = 0 when target is directly in front
        // angle = 1 when target is 180° to the right
        // angle = -1 when target is 180° to the left
        
        return (angle, distance);
    }

    public int Write(ObservationWriter writer)
    {
        var index = 0;
        Debug.Log($"Writing observations for agent {agent.name} (Team: {agent.team})");

        // 1. Self Information (7 values)
        writer[index++] = transform.forward.x;
        writer[index++] = transform.forward.z;
        writer[index++] = agent.team == Team.Blue ? 1f : -1f;
        
        GameObject ownGoal = GameObject.FindGameObjectWithTag(agent.team == Team.Blue ? "blueGoal" : "purpleGoal");
        GameObject enemyGoal = GameObject.FindGameObjectWithTag(agent.team == Team.Blue ? "purpleGoal" : "blueGoal");
        
        if (ownGoal != null && enemyGoal != null)
        {
            var (ownGoalAngle, ownGoalDist) = ToPolar(ownGoal.transform.position, transform);
            var (enemyGoalAngle, enemyGoalDist) = ToPolar(enemyGoal.transform.position, transform);
            
            writer[index++] = ownGoalAngle;
            writer[index++] = ownGoalDist;
            writer[index++] = enemyGoalAngle;
            writer[index++] = enemyGoalDist;

            Debug.Log($"Goals: Own({ownGoalAngle:F2}°, {ownGoalDist:F2}m) Enemy({enemyGoalAngle:F2}°, {enemyGoalDist:F2}m)");
        }
        else
        {
            Debug.LogWarning("Could not find goals!");
            for (int i = 0; i < 4; i++) writer[index++] = 0f;
        }

        // 2. Ball Information (4 values)
        GameObject ball = GameObject.FindGameObjectWithTag("ball");
        if (ball != null)
        {
            var (angle, distance) = ToPolar(ball.transform.position, transform);
            writer[index++] = angle;
            writer[index++] = distance;
            
            var ballRb = ball.GetComponent<Rigidbody>();
            var ballVelocityAngle = Vector3.SignedAngle(transform.forward, ballRb.velocity, Vector3.up) / 180f;
            var ballSpeed = ballRb.velocity.magnitude / maxBallSpeed;
            writer[index++] = ballVelocityAngle;
            writer[index++] = ballSpeed;

            Debug.Log($"Ball: Angle={angle:F2}°, Distance={distance:F2}m, Velocity={ballSpeed:F2} at {ballVelocityAngle:F2}°");
            
            // Draw debug ray to ball
            Debug.DrawLine(transform.position, ball.transform.position, Color.yellow, 0.1f);
        }
        else
        {
            Debug.LogWarning("Could not find ball!");
            for (int i = 0; i < 4; i++) writer[index++] = 0f;
        }

        // 3. Other Players
        var players = GameObject.FindGameObjectsWithTag("blueAgent")
            .Concat(GameObject.FindGameObjectsWithTag("purpleAgent"))
            .Where(p => p != agent.gameObject)
            .Take(3)
            .ToList();

        Debug.Log($"Found {players.Count} other players");

        foreach (var player in players)
        {
            var (angle, distance) = ToPolar(player.transform.position, transform);
            bool isTeammate = player.CompareTag(agent.team == Team.Blue ? "blueAgent" : "purpleAgent");
            
            writer[index++] = angle;
            writer[index++] = distance;
            writer[index++] = isTeammate ? 1f : -1f;
            
            // Draw debug ray to player
            Color rayColor = isTeammate ? Color.green : Color.red;
            Debug.DrawLine(transform.position, player.transform.position, rayColor, 0.1f);

            if (ball != null)
            {
                var playerTransform = player.transform;
                var (ballAngle, ballDist) = ToPolar(ball.transform.position, playerTransform);
                writer[index++] = ballAngle;
                writer[index++] = ballDist;
                
                var (ownGoalAngle, _) = ToPolar(ownGoal.transform.position, playerTransform);
                var (enemyGoalAngle, _) = ToPolar(enemyGoal.transform.position, playerTransform);
                writer[index++] = ownGoalAngle;
                writer[index++] = enemyGoalAngle;
                writer[index++] = playerTransform.forward.x;

                Debug.Log($"Player {(isTeammate ? "Teammate" : "Opponent")}: " +
                         $"Angle={angle:F2}°, Distance={distance:F2}m, " +
                         $"Ball(angle={ballAngle:F2}°, dist={ballDist:F2}m)");
            }
            else
            {
                for (int i = 0; i < 6; i++) writer[index++] = 0f;
            }
        }

        // Pad missing players
        for (int i = players.Count; i < 3; i++)
        {
            for (int j = 0; j < 8; j++) writer[index++] = 0f;
        }

        Debug.Log($"Total observations written: {index}");
        return index;
    }

    public void Update() {}
    public void Reset() {}

    public CompressionSpec GetCompressionSpec()
    {
        return CompressionSpec.Default();
    }
} 