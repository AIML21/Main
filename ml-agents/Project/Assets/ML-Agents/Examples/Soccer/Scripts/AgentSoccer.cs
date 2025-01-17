using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Policies;

public enum Team
{
    Blue = 0,
    Purple = 1
}

public class AgentSoccer : Agent
{
    public enum Position
    {
        Striker,
        Goalie,
        Generic
    }

    [HideInInspector]
    public Team team;
    private float m_KickPower;
    private float m_BallTouch;
    public Position position;

    private const float k_Power = 2000f;
    private float m_Existential;
    private float m_LateralSpeed;
    private float m_ForwardSpeed;

    [HideInInspector]
    public Rigidbody agentRb;
    private SoccerSettings m_SoccerSettings;
    private BehaviorParameters m_BehaviorParameters;
    private SoccerEnvController envController;
    public Vector3 initialPos;
    public float rotSign;

    private EnvironmentParameters m_ResetParams;

    public float visionAngle; // Vision angle for directional awareness

    public override void Initialize()
    {
        envController = GetComponentInParent<SoccerEnvController>();
        if (envController != null)
        {
            m_Existential = 1f / envController.MaxEnvironmentSteps;
        }
        else
        {
            m_Existential = 1f / MaxStep;
        }

        m_BehaviorParameters = GetComponent<BehaviorParameters>();
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
        switch (position)
        {
            case Position.Goalie:
                m_LateralSpeed = 1.0f;
                m_ForwardSpeed = 1.0f;
                break;
            case Position.Striker:
                m_LateralSpeed = 0.3f;
                m_ForwardSpeed = 1.3f;
                break;
            default:
                m_LateralSpeed = 0.3f;
                m_ForwardSpeed = 1.0f;
                break;
        }

        m_SoccerSettings = FindObjectOfType<SoccerSettings>();
        agentRb = GetComponent<Rigidbody>();
        agentRb.maxAngularVelocity = 500;

        m_ResetParams = Academy.Instance.EnvironmentParameters;

        visionAngle = 0f; // Initialize vision angle
    }

    public void MoveAgent(ActionSegment<int> act)
    {
        var dirToGo = Vector3.zero;
        var rotateDir = Vector3.zero;

        m_KickPower = 0f;

        var forwardAxis = act[0];
        var rightAxis = act[1];
        var rotateAxis = act[2];
        var visionAxis = act.Length > 3 ? act[3] : 0; // Check for vision axis availability

        // Forward/backward movement
        switch (forwardAxis)
        {
            case 1:
                dirToGo = transform.forward * m_ForwardSpeed;
                break;
            case 2:
                dirToGo = transform.forward * -m_ForwardSpeed;
                break;
        }

        // Lateral movement
        switch (rightAxis)
        {
            case 1:
                dirToGo = transform.right * m_LateralSpeed;
                break;
            case 2:
                dirToGo = transform.right * -m_LateralSpeed;
                break;
        }

        // Rotation
        switch (rotateAxis)
        {
            case 1:
                rotateDir = transform.up * -1f;
                break;
            case 2:
                rotateDir = transform.up * 1f;
                break;
        }

        // Adjust vision angle for awareness
        if (visionAxis > 0)
        {
            switch (visionAxis)
            {
                case 1:
                    visionAngle -= 20f; // Look left
                    break;
                case 2:
                    visionAngle += 20f; // Look right
                    break;
            }
        }

        visionAngle = Mathf.Repeat(visionAngle, 360f); // Clamp vision angle to [0, 360]

        // Apply movement and rotation
        transform.Rotate(rotateDir, Time.deltaTime * 150f);
        agentRb.AddForce(dirToGo * m_SoccerSettings.agentRunSpeed, ForceMode.VelocityChange);
    }

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        // Existential reward: encourage agents to persist in the environment
        AddReward(m_Existential);

        Vector3 ballPosition = envController.getBallPosition();

        // Positional rewards (specific to roles)
        if (position == Position.Goalie)
        {
            // Add specific goalie rewards, if necessary
        }
        else if (position == Position.Striker)
        {
            // Add specific striker rewards, if necessary
        }

        // Move the agent based on actions
        MoveAgent(actionBuffers.DiscreteActions);

        // Penalize inactivity near the wall
        PenalizeInactivityNearWall();

        // Reward movement away from the wall
        //RewardMovementAwayFromWall();
    }

    // private void RewardMovementAwayFromWall()
    // {
    //     float wallDistanceX = Mathf.Abs(transform.position.x);
    //     float wallDistanceZ = Mathf.Abs(transform.position.z);

    //     // If the agent is near the wall, reward them for moving further away
    //     if (wallDistanceX > 4.5f || wallDistanceZ > 4.5f)
    //     {
    //         AddReward(0.05f); // Reward magnitude can be adjusted based on impact
    //     }
    // }

    private void PenalizeInactivityNearWall()
    {
        float wallDistance = Mathf.Min(Mathf.Abs(transform.position.x), Mathf.Abs(transform.position.z));

        // Penalize if the agent is too close to the wall and not moving
        if (wallDistance < 1.0f && agentRb.velocity.magnitude < 0.5f)
        {
            AddReward(-0.1f);  // Adjust the penalty as needed
        }
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var discreteActionsOut = actionsOut.DiscreteActions;

        // Forward/backward
        if (Input.GetKey(KeyCode.W)) discreteActionsOut[0] = 1;
        if (Input.GetKey(KeyCode.S)) discreteActionsOut[0] = 2;

        // Right/left
        if (Input.GetKey(KeyCode.D)) discreteActionsOut[1] = 1;
        if (Input.GetKey(KeyCode.A)) discreteActionsOut[1] = 2;

        // Rotation
        if (Input.GetKey(KeyCode.E)) discreteActionsOut[2] = 1;
        if (Input.GetKey(KeyCode.Q)) discreteActionsOut[2] = 2;

        // Vision adjustment
        if (Input.GetKey(KeyCode.LeftArrow)) discreteActionsOut[3] = 1;
        if (Input.GetKey(KeyCode.RightArrow)) discreteActionsOut[3] = 2;
    }

    private void EvaluateKickDirection(Collision collision)
    {
        if (collision.gameObject.CompareTag("ball"))
        {
            // Get the ball's Rigidbody to analyze its velocity
            Rigidbody ballRb = collision.gameObject.GetComponent<Rigidbody>();
            Vector3 ballVelocity = ballRb.velocity;

            // Determine if the ball is moving towards the agent's own side
            bool isMovingTowardsOwnSide = (team == Team.Blue && ballVelocity.x < 0) ||
                                          (team == Team.Purple && ballVelocity.x > 0);

            // Determine if the ball is moving towards the opponent's side
            bool isMovingTowardsOpponentSide = (team == Team.Blue && ballVelocity.x > 0) ||
                                               (team == Team.Purple && ballVelocity.x < 0);

            // Penalize for kicking towards own side
            if (isMovingTowardsOwnSide)
            {
                AddReward(-0.2f); // Penalize the agent
            }

            // Reward for kicking towards the opponent's side
            if (isMovingTowardsOpponentSide)
            {
                AddReward(0.3f); // Reward the agent
            }
        }
    }

    private void OnCollisionEnter(Collision collision)
    {
        var force = k_Power * m_KickPower;
        if (collision.gameObject.CompareTag("ball"))
        {
            AddReward(0.2f); // Reward for ball contact
            m_BallTouch += 0.1f; // Increment ball touch metric

            var dir = collision.contacts[0].point - transform.position;
            dir = dir.normalized;

            collision.gameObject.GetComponent<Rigidbody>().AddForce(dir * force);

            // Evaluate the direction of the kick
            EvaluateKickDirection(collision);
        }
    }

    public override void OnEpisodeBegin()
    {
        m_BallTouch = m_ResetParams.GetWithDefault("ball_touch", 0.1f); // Initialize ball touch reward scaling
    }
}
