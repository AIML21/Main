using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Policies;
using System.Linq;

public enum Team
{
    Blue = 0,
    Purple = 1
}

public class AgentSoccer : Agent
{
    // Note that that the detectable tags are different for the blue and purple teams. The order is
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

    private GameObject m_OwnGoal;
    private GameObject m_OpponentGoal;
    private const float k_GoalieDistancePenalty = 0.1f;
    private const float k_DirectionalKickReward = 0.5f;

    public override void Initialize()
    {
        SoccerEnvController envController = GetComponentInParent<SoccerEnvController>();
        if (envController != null)
        {
            m_Existential = 1f / envController.MaxEnvironmentSteps;
        }
        else
        {
            m_Existential = 1f / MaxStep;
        }

        m_BehaviorParameters = gameObject.GetComponent<BehaviorParameters>();
        
        // Assign position based on object name - if it has (1) it's a goalie, otherwise striker
        if (gameObject.name.Contains("(1)"))
        {
            position = Position.Goalie;
            Debug.Log($"Agent {gameObject.name} assigned as Goalie");
        }
        else
        {
            position = Position.Striker;
            Debug.Log($"Agent {gameObject.name} assigned as Striker");
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

        // Set speeds based on position
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

        m_SoccerSettings = FindObjectOfType<SoccerSettings>();
        agentRb = GetComponent<Rigidbody>();
        agentRb.maxAngularVelocity = 500;

        m_ResetParams = Academy.Instance.EnvironmentParameters;

        if (team == Team.Blue)
        {
            m_OwnGoal = envController.transform.Find("BlueGoal")?.gameObject;
            m_OpponentGoal = envController.transform.Find("PurpleGoal")?.gameObject;
            
            if (m_OwnGoal == null)
            {
                m_OwnGoal = envController.GetComponentsInChildren<Transform>()
                    .FirstOrDefault(t => t.CompareTag("blueGoal"))?.gameObject;
                m_OpponentGoal = envController.GetComponentsInChildren<Transform>()
                    .FirstOrDefault(t => t.CompareTag("purpleGoal"))?.gameObject;
            }
        }
        else
        {
            m_OwnGoal = envController.transform.Find("PurpleGoal")?.gameObject;
            m_OpponentGoal = envController.transform.Find("BlueGoal")?.gameObject;
            
            if (m_OwnGoal == null)
            {
                m_OwnGoal = envController.GetComponentsInChildren<Transform>()
                    .FirstOrDefault(t => t.CompareTag("purpleGoal"))?.gameObject;
                m_OpponentGoal = envController.GetComponentsInChildren<Transform>()
                    .FirstOrDefault(t => t.CompareTag("blueGoal"))?.gameObject;
            }
        }

        if (m_OwnGoal == null || m_OpponentGoal == null)
        {
            Debug.LogError($"Could not find goals for agent {gameObject.name} in environment {envController.name}");
        }
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
            
            // Add distance-based penalty for goalies
            float distanceFromGoal = Vector3.Distance(transform.position, m_OwnGoal.transform.position);
            float penaltyMultiplier = Mathf.Clamp01(distanceFromGoal / 10f); // Adjust 10f based on field size
            AddReward(-penaltyMultiplier * k_GoalieDistancePenalty);
            
            // Debug visualization of distance
            Debug.DrawLine(transform.position, m_OwnGoal.transform.position, Color.red);
        }
        else if (position == Position.Striker)
        {
            // Existential penalty for Strikers
            AddReward(-m_Existential);
        }
        MoveAgent(actionBuffers.DiscreteActions);
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
            // Base reward for touching the ball
            AddReward(.2f * m_BallTouch);
            
            var dir = c.contacts[0].point - transform.position;
            dir = dir.normalized;
            
            // Calculate direction to opponent's goal
            Vector3 ballToGoal = m_OpponentGoal.transform.position - c.gameObject.transform.position;
            ballToGoal.y = 0; // Ignore vertical component
            ballToGoal.Normalize();
            
            // Calculate angle between kick direction and ideal direction
            float dotProduct = Vector3.Dot(dir, ballToGoal);
            // Convert dot product to angle (0 to 180 degrees)
            float angle = Mathf.Acos(dotProduct) * Mathf.Rad2Deg;
            
            // Calculate direction reward (1.0 when angle is 0, 0.0 when angle is 180)
            float directionReward = Mathf.Max(0, (180f - angle) / 180f);
            
            // Calculate kick strength reward (based on m_KickPower which is 0-1)
            float strengthReward = m_KickPower;
            
            // Combine direction and strength rewards
            float kickReward = directionReward * strengthReward * k_DirectionalKickReward;
            AddReward(kickReward);
            
            // Debug visualization - make lines thicker and last longer
            Debug.DrawRay(c.gameObject.transform.position, dir * 10f, Color.blue, 3f); // Actual kick direction
            Debug.DrawRay(c.gameObject.transform.position, ballToGoal * 10f, Color.green, 3f); // Ideal direction
            
            // Debug.Log for monitoring rewards
            Debug.Log($"Kick Reward: {kickReward:F2} (Direction: {directionReward:F2}, Strength: {strengthReward:F2}, Angle: {angle:F1}°)");
            
            // Apply force to ball
            c.gameObject.GetComponent<Rigidbody>().AddForce(dir * force);
        }
    }

    public override void OnEpisodeBegin()
    {
        m_BallTouch = m_ResetParams.GetWithDefault("ball_touch", 0);
    }

    private void OnDrawGizmos()
    {
        if (!Application.isPlaying || m_OpponentGoal == null || m_OwnGoal == null) return;

        // Draw lines for all agents
        Gizmos.color = Color.yellow;
        Gizmos.DrawLine(transform.position, m_OpponentGoal.transform.position);

        //print own position
        Debug.Log("Own position: " + position);
        
        // For goalies, also draw the line to their own goal
        if (position == Position.Goalie)
        {

            //log that we are drawing the line
            Debug.Log("Drawing line to own goal");
            // Make the line bright red and thicker using Debug.DrawLine instead
            Debug.DrawLine(
                transform.position, 
                m_OwnGoal.transform.position, 
                new Color(1f, 0f, 0f, 1f), // Bright red
                0f, // Duration (0 means single frame)
                false // Depth testing
            );
            
            // Also draw a thicker line with Gizmos as backup
            Gizmos.color = Color.red;
            Gizmos.DrawLine(transform.position, m_OwnGoal.transform.position);
            
        }
    }

}
