using UnityEngine;
using Unity.MLAgents.Sensors;
using System.Collections.Generic;

[AddComponentMenu("ML-Agents/Memory Ray Sensor Component")]
public class MemoryRaySensorComponent : SensorComponent
{
    [Header("Ray Settings")]
    [Tooltip("List of tags that the rays should detect")]
    public string[] detectableTags = new string[] { 
        "ball", 
        "wall", 
        "blueGoal", 
        "purpleGoal", 
        "blueAgent", 
        "purpleAgent" 
    };

    [Tooltip("Number of rays to cast in each direction")]
    public int raysPerDirection = 11;

    [Tooltip("Maximum angle for rays (in degrees)")]
    public float maxRayDegrees = 120f;

    [Tooltip("Maximum length of rays")]
    public float rayLength = 20f;

    [Tooltip("Layer mask for ray casts")]
    public LayerMask rayLayerMask = Physics.DefaultRaycastLayers;

    [Header("Sensor Settings")]
    [Tooltip("Unique name for this sensor")]
    public string sensorName = "MemoryRaySensor";

    private MemoryRaySensor sensor;
    private AgentSoccer agent;
    private int observationSize;

    void Awake()
    {
        // Get the AgentSoccer component
        agent = GetComponent<AgentSoccer>();
        if (agent == null)
        {
            Debug.LogError("MemoryRaySensorComponent requires an AgentSoccer component");
            return;
        }

        // Calculate observation size
        int framesStored = 31;  // Current frame + 30 history frames
        
        // Per frame data:
        // Self (7):
        //   - Forward direction (2)
        //   - Team ID (1)
        //   - Angles/distances to goals (4)
        int selfData = 7;

        // Ball (4):
        //   - Angle/distance to ball (2)
        //   - Ball velocity angle/speed (2)
        int ballData = 4;

        // Other Players (3 players × 8 values = 24):
        //   - Angle/distance to player (2)
        //   - Team ID (1)
        //   - Player's angle/distance to ball (2)
        //   - Player's angles to goals (2)
        //   - Player's forward direction (1)
        int otherPlayersData = 24;

        // Total per frame
        int dataPerFrame = selfData + ballData + otherPlayersData;  // 35

        // Total for all frames
        observationSize = dataPerFrame * framesStored;  // 35 × 31 = 1,085
        
        Debug.Log($"Initialized MemoryRaySensorComponent with:" +
                  $"\n - Data per frame: {dataPerFrame}" +
                  $"\n - Frames stored: {framesStored}" +
                  $"\n - Total observation size: {observationSize}");
    }

    private void CreateOrUpdateSensor()
    {
        if (agent == null)
        {
            agent = GetComponent<AgentSoccer>();
            if (agent == null)
            {
                Debug.LogError("Cannot create sensor - agent is null");
                return;
            }
        }

        var raySettings = new MemoryRaySensor.RayPerceptionSettings
        {
            RaysPerDirection = raysPerDirection,
            MaxRayDegrees = maxRayDegrees,
            RayLength = rayLength,
            DetectableTags = new List<string>(detectableTags),
            RayLayerMask = rayLayerMask
        };

        sensor = new MemoryRaySensor(sensorName, agent, observationSize, raySettings);
        
        Debug.Log($"Created MemoryRaySensor with settings:" +
                 $"\n - Observation Size: {observationSize}" +
                 $"\n - Rays Per Direction: {raysPerDirection}" +
                 $"\n - Max Ray Degrees: {maxRayDegrees}" +
                 $"\n - Ray Length: {rayLength}");
    }

    public override ISensor[] CreateSensors()
    {
        if (sensor == null)
        {
            CreateOrUpdateSensor();
        }

        if (sensor == null)
        {
            Debug.LogError("Failed to create MemoryRaySensor - sensor is null");
            return new ISensor[0];
        }

        return new ISensor[] { sensor };
    }

    void OnValidate()
    {
        if (raysPerDirection < 1) raysPerDirection = 1;
        if (maxRayDegrees < 0) maxRayDegrees = 0;
        if (maxRayDegrees > 360) maxRayDegrees = 360;
        if (rayLength <= 0) rayLength = 0.1f;
    }
} 