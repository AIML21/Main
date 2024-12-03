using UnityEngine;
using UnityEngine.Serialization;
using System.Collections.Generic;

namespace Unity.MLAgents.Sensors
{
    /// <summary>
    /// A component for 3D Ray Perception with memory of previous observations.
    /// </summary>
    [AddComponentMenu("ML Agents/Ray Perception Sensor 3D", (int)MenuGroup.Sensors)]
    public class RayPerceptionSensorComponent3D : RayPerceptionSensorComponentBase
    {
        public const int MEMORY_FRAMES = 30;
        private const int DEFAULT_NUM_RAYS = 11;
        private Queue<RayPerceptionOutput> m_RayMemory;
        private Queue<Dictionary<GameObject, RelativeObjectData>> m_ObjectMemory;
        private bool m_Initialized = false;

        public struct RelativeObjectData
        {
            public Vector3 RelativePosition;
            public Vector3 RelativeVelocity;
        }

        [HideInInspector, SerializeField, FormerlySerializedAs("startVerticalOffset")]
        [Range(-10f, 10f)]
        [Tooltip("Ray start is offset up or down by this amount.")]
        float m_StartVerticalOffset;

        /// <summary>
        /// Ray start is offset up or down by this amount.
        /// </summary>
        public float StartVerticalOffset
        {
            get => m_StartVerticalOffset;
            set { m_StartVerticalOffset = value; UpdateSensor(); }
        }

        [HideInInspector, SerializeField, FormerlySerializedAs("endVerticalOffset")]
        [Range(-10f, 10f)]
        [Tooltip("Ray end is offset up or down by this amount.")]
        float m_EndVerticalOffset;

        /// <summary>
        /// Ray end is offset up or down by this amount.
        /// </summary>
        public float EndVerticalOffset
        {
            get => m_EndVerticalOffset;
            set { m_EndVerticalOffset = value; UpdateSensor(); }
        }

        void Awake()
        {
            EnsureInitialized();
        }

        private void EnsureInitialized()
        {
            if (m_Initialized) return;

            m_RayMemory = new Queue<RayPerceptionOutput>();
            m_ObjectMemory = new Queue<Dictionary<GameObject, RelativeObjectData>>();

            // Initialize with empty frames
            for (int i = 0; i < MEMORY_FRAMES; i++)
            {
                var emptyRayOutput = new RayPerceptionOutput();
                emptyRayOutput.RayOutputs = new RayPerceptionOutput.RayOutput[DEFAULT_NUM_RAYS];
                for (int j = 0; j < DEFAULT_NUM_RAYS; j++)
                {
                    emptyRayOutput.RayOutputs[j] = new RayPerceptionOutput.RayOutput
                    {
                        HasHit = false,
                        HitFraction = 1.0f,
                        HitTaggedObject = false,
                        HitTagIndex = -1,
                    };
                }
                m_RayMemory.Enqueue(emptyRayOutput);
                m_ObjectMemory.Enqueue(new Dictionary<GameObject, RelativeObjectData>());
            }

            m_Initialized = true;
        }

        public void UpdateMemory(RayPerceptionOutput rayOutput, GameObject[] trackedObjects)
        {
            EnsureInitialized();

            if (rayOutput?.RayOutputs == null || trackedObjects == null) return;

            // Update ray memory
            if (m_RayMemory.Count >= MEMORY_FRAMES)
            {
                m_RayMemory.Dequeue();
            }
            m_RayMemory.Enqueue(rayOutput);

            // Update object memory
            if (m_ObjectMemory.Count >= MEMORY_FRAMES)
            {
                m_ObjectMemory.Dequeue();
            }

            var newObjectFrame = new Dictionary<GameObject, RelativeObjectData>();
            foreach (var obj in trackedObjects)
            {
                if (obj == null) continue;

                var objRb = obj.GetComponent<Rigidbody>();
                if (objRb == null) continue;

                var relativePos = transform.InverseTransformPoint(obj.transform.position);
                var relativeVel = transform.InverseTransformDirection(objRb.velocity);

                newObjectFrame[obj] = new RelativeObjectData
                {
                    RelativePosition = relativePos,
                    RelativeVelocity = relativeVel
                };
            }
            m_ObjectMemory.Enqueue(newObjectFrame);
        }

        public float[] GetMemoryObservations()
        {
            EnsureInitialized();

            var observations = new List<float>();

            // Add ray memory
            foreach (var rayFrame in m_RayMemory)
            {
                if (rayFrame?.RayOutputs != null)
                {
                    foreach (var rayOutput in rayFrame.RayOutputs)
                    {
                        observations.Add(rayOutput.HitFraction);
                    }
                }
                else
                {
                    // Add default values if frame is null
                    for (int i = 0; i < DEFAULT_NUM_RAYS; i++)
                    {
                        observations.Add(1.0f);
                    }
                }
            }

            // Add object memory
            foreach (var objectFrame in m_ObjectMemory)
            {
                foreach (var objData in objectFrame.Values)
                {
                    // Normalize positions and velocities
                    observations.Add(objData.RelativePosition.x / 50f);
                    observations.Add(objData.RelativePosition.y / 10f);
                    observations.Add(objData.RelativePosition.z / 50f);
                    observations.Add(objData.RelativeVelocity.x / 20f);
                    observations.Add(objData.RelativeVelocity.z / 20f);
                }
            }

            return observations.ToArray();
        }

        public override RayPerceptionCastType GetCastType()
        {
            return RayPerceptionCastType.Cast3D;
        }

        public override float GetStartVerticalOffset()
        {
            return StartVerticalOffset;
        }

        public override float GetEndVerticalOffset()
        {
            return EndVerticalOffset;
        }

        void OnEnable()
        {
            EnsureInitialized();
        }

        void OnDisable()
        {
            if (m_RayMemory != null)
            {
                m_RayMemory.Clear();
                m_Initialized = false;
            }
            if (m_ObjectMemory != null)
            {
                m_ObjectMemory.Clear();
            }
        }
    }
}
