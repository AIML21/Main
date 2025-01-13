using System.Collections.Generic;
using Unity.MLAgents;
using UnityEngine;

public class SoccerEnvController : MonoBehaviour
{
    [System.Serializable]
    public class PlayerInfo
    {
        public AgentSoccer Agent;
        [HideInInspector]
        public Vector3 StartingPos;
        [HideInInspector]
        public Quaternion StartingRot;
        [HideInInspector]
        public Rigidbody Rb;
    }

    [Tooltip("Max Environment Steps")] public int MaxEnvironmentSteps = 25000;

    public GameObject ball;
    [HideInInspector]
    public Rigidbody ballRb;
    private Vector3 m_BallStartingPos;

    public List<PlayerInfo> AgentsList = new List<PlayerInfo>();

    private SoccerSettings m_SoccerSettings;
    private SimpleMultiAgentGroup m_BlueAgentGroup;
    private SimpleMultiAgentGroup m_PurpleAgentGroup;

    private int m_ResetTimer;

    void Start()
    {
        m_SoccerSettings = FindObjectOfType<SoccerSettings>();
        m_BlueAgentGroup = new SimpleMultiAgentGroup();
        m_PurpleAgentGroup = new SimpleMultiAgentGroup();
        ballRb = ball.GetComponent<Rigidbody>();
        m_BallStartingPos = ball.transform.position;

        foreach (var item in AgentsList)
        {
            item.StartingPos = item.Agent.transform.position;
            item.StartingRot = item.Agent.transform.rotation;
            item.Rb = item.Agent.GetComponent<Rigidbody>();
            if (item.Agent.team == Team.Blue)
            {
                m_BlueAgentGroup.RegisterAgent(item.Agent);
            }
            else
            {
                m_PurpleAgentGroup.RegisterAgent(item.Agent);
            }
        }

        ResetScene();
    }

    void FixedUpdate()
    {
        m_ResetTimer += 1;

        // Check for timeout
        if (m_ResetTimer >= MaxEnvironmentSteps && MaxEnvironmentSteps > 0)
        {
            foreach (var item in AgentsList)
            {
                item.Agent.AddReward(-0.01f); // Small penalty for timeout
            }
            ResetScene();
        }

        // Periodic updates to rewards
        UpdateAgentRewards();

        // Penalize clumping near ball and wall
        PenalizeClumpingNearBallAndWall();
    }

    public Vector3 getBallPosition()
    {
        return ball.transform.position;
    }

    public void ResetBall()
    {
        var randomPosX = Random.Range(-2.5f, 2.5f);
        var randomPosZ = Random.Range(-2.5f, 2.5f);

        ball.transform.position = m_BallStartingPos + new Vector3(randomPosX, 0f, randomPosZ);
        ballRb.velocity = Vector3.zero;
        ballRb.angularVelocity = Vector3.zero;
    }

    public void GoalTouched(Team scoredTeam)
    {
        foreach (var item in AgentsList)
        {
            if (item.Agent.team == scoredTeam)
            {
                item.Agent.AddReward(1.0f); // Large reward for scoring
            }
            else
            {
                item.Agent.AddReward(-1.0f); // Large penalty for conceding
            }
        }

        m_BlueAgentGroup.EndGroupEpisode();
        m_PurpleAgentGroup.EndGroupEpisode();

        ResetScene();
    }

    public void ResetScene()
    {
        m_ResetTimer = 0;

        // Reset Agents
        foreach (var item in AgentsList)
        {
            var randomPosX = Random.Range(-3f, 3f); // Adjusted randomness for agent positions
            var newStartPos = item.Agent.initialPos + new Vector3(randomPosX, 0f, 0f);
            var rot = item.Agent.rotSign * Random.Range(80.0f, 100.0f);
            var newRot = Quaternion.Euler(0, rot, 0);
            item.Agent.transform.SetPositionAndRotation(newStartPos, newRot);

            item.Rb.velocity = Vector3.zero;
            item.Rb.angularVelocity = Vector3.zero;
        }

        // Reset Ball
        ResetBall();
    }

    public void UpdateAgentRewards()
    {
        // Reward/penalize based on ball proximity
        foreach (var item in AgentsList)
        {
            float distanceToBall = Vector3.Distance(item.Agent.transform.position, ball.transform.position);
            if (distanceToBall < 1.5f) // Close to the ball
            {
                item.Agent.AddReward(0.01f); // Small positive reward for proximity
            }
            else
            {
                item.Agent.AddReward(-0.005f); // Small penalty for being too far from the ball
            }
        }
    }

    // Penalize agents for clumping near the ball and near the wall
    public void PenalizeClumpingNearBallAndWall()
    {
        float clumpDistanceThreshold = 1.5f; // Distance considered "clumped"
        float wallProximityThreshold = 4.5f; // Distance from wall to trigger penalization
        float penalty = -0.1f; // Penalty for clumping

        // Get the position of the ball
        Vector3 ballPosition = ball.transform.position;

        foreach (var agentInfo in AgentsList)
        {
            int clumpCount = 0;
            Vector3 agentPosition = agentInfo.Agent.transform.position;

            // Check if agent is near the ball
            if (Vector3.Distance(agentPosition, ballPosition) < clumpDistanceThreshold)
            {
                // Check proximity to other agents
                foreach (var otherAgentInfo in AgentsList)
                {
                    if (agentInfo != otherAgentInfo) // Skip self
                    {
                        if (Vector3.Distance(agentPosition, otherAgentInfo.Agent.transform.position) < clumpDistanceThreshold)
                        {
                            clumpCount++;
                        }
                    }
                }

                // Check if agent is near the wall
                bool nearWall = Mathf.Abs(agentPosition.x) > wallProximityThreshold || Mathf.Abs(agentPosition.z) > wallProximityThreshold;

                // Apply penalty if clumping occurs near the ball and the wall
                if (clumpCount >= 2 && nearWall) // 3 agents (including self) clumped
                {
                    agentInfo.Agent.AddReward(penalty);
                }
            }
        }
    }
}
