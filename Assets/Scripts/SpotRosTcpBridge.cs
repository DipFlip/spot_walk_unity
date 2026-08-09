using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.IO;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEngine;

[DisallowMultipleComponent]
[RequireComponent(typeof(SpotKinematicDrive))]
public sealed class SpotRosTcpBridge : MonoBehaviour
{
    [Serializable]
    private sealed class BridgeMessage
    {
        public string type;
        public string id;
        public string command;
        public bool success;
        public bool has_lease;
        public bool powered_on;
        public bool standing;
        public string message;
        public float vx;
        public float vy;
        public float wz;
        public float x;
        public float y;
        public float z;
        public float yaw;
        public float qx;
        public float qy;
        public float qz;
        public float qw;
        public float vision_x;
        public float vision_y;
        public float vision_z;
        public float vision_qx;
        public float vision_qy;
        public float vision_qz;
        public float vision_qw;
        public uint sequence;
        public float background_rate_per_detector;
        public RadiationSourceMessage[] sources;
    }

    [Serializable]
    private sealed class RadiationSourceMessage
    {
        public string id;
        public string isotope;
        public float activity_bq;
        public float x;
        public float y;
        public float z;
        public float qx;
        public float qy;
        public float qz;
        public float qw;
    }

    private struct RosPose
    {
        public readonly Vector3 Position;
        public readonly Quaternion Rotation;

        public RosPose(Vector3 position, Quaternion rotation)
        {
            Position = position;
            Rotation = rotation;
        }
    }

    [Header("Bridge")]
    [SerializeField] private string host = "127.0.0.1";
    [SerializeField] private int port = 50052;
    [SerializeField, Min(0.1f)] private float reconnectDelaySeconds = 2f;
    [SerializeField, Range(1f, 100f)] private float poseRateHz = 30f;
    [SerializeField, Min(0.05f)] private float velocityCommandTimeoutSeconds = 0.35f;

    [Header("Runtime")]
    [SerializeField, Range(15, 240)] private int targetFrameRate = 30;

    [Header("Radiation Simulation")]
    [SerializeField, Range(0.5f, 30f)] private float radiationSourceRateHz = 5f;
    [SerializeField, Min(0f)] private float backgroundRatePerDetector = 1000f;

    [Header("Initial Robot State")]
    [SerializeField] private bool initiallyPoweredOn;
    [SerializeField] private bool initiallyStanding;

    [Header("Diagnostics")]
    [SerializeField] private bool logConnectionChanges = true;

    private readonly ConcurrentQueue<string> inboundMessages = new ConcurrentQueue<string>();
    private readonly object socketLock = new object();
    private readonly object writerLock = new object();
    private readonly ManualResetEvent stopRequested = new ManualResetEvent(false);

    private SpotKinematicDrive drive;
    private SpotProceduralTrot proceduralTrot;
    private TcpClient activeClient;
    private StreamWriter activeWriter;
    private Thread networkThread;
    private Vector3 initialPosition;
    private Quaternion initialRotation;
    private Vector3 previousPosition;
    private Quaternion previousRotation;
    private float previousPoseTime;
    private float nextPoseTime;
    private float lastVelocityCommandTime = float.NegativeInfinity;
    private bool hasLease;
    private bool poweredOn;
    private bool standing;
    private int connectionState;
    private int reportedConnectionState = int.MinValue;
    private int previousTargetFrameRate;
    private readonly List<RadiationSource> radiationSources = new List<RadiationSource>();
    private float nextRadiationSourceTime;
    private uint radiationSequence;

    public bool IsConnected => Volatile.Read(ref connectionState) == 1;
    public bool HasLease => hasLease;
    public bool IsPoweredOn => poweredOn;
    public bool IsStanding => standing;

    private void Awake()
    {
        drive = GetComponent<SpotKinematicDrive>();
        proceduralTrot = GetComponent<SpotProceduralTrot>();
    }

    private void OnEnable()
    {
        poseRateHz = Mathf.Clamp(poseRateHz, 1f, 100f);
        velocityCommandTimeoutSeconds = Mathf.Max(0.05f, velocityCommandTimeoutSeconds);
        targetFrameRate = Mathf.Clamp(targetFrameRate, 15, 240);
        radiationSourceRateHz = Mathf.Clamp(radiationSourceRateHz, 0.5f, 30f);
        backgroundRatePerDetector = Mathf.Max(0f, backgroundRatePerDetector);
        previousTargetFrameRate = Application.targetFrameRate;
        Application.targetFrameRate = targetFrameRate;
        initialPosition = transform.position;
        initialRotation = transform.rotation;
        previousPosition = initialPosition;
        previousRotation = initialRotation;
        previousPoseTime = Time.unscaledTime;
        nextPoseTime = previousPoseTime;
        nextRadiationSourceTime = previousPoseTime;
        radiationSequence = 0;
        hasLease = false;
        poweredOn = initiallyPoweredOn;
        standing = initiallyStanding && poweredOn;
        drive.SetExternalControlEnabled(false);
        RefreshMotionState();
        proceduralTrot?.SetSitting(!standing);

        stopRequested.Reset();
        Volatile.Write(ref connectionState, 0);
        networkThread = new Thread(NetworkLoop)
        {
            IsBackground = true,
            Name = "Spot ROS control TCP client"
        };
        networkThread.Start();
    }

    private void Update()
    {
        ReportConnectionChange();
        while (inboundMessages.TryDequeue(out string json))
        {
            HandleMessage(json);
        }

        if (Time.unscaledTime - lastVelocityCommandTime > velocityCommandTimeoutSeconds)
        {
            drive.SetExternalControlEnabled(false);
        }

        if (Time.unscaledTime >= nextPoseTime)
        {
            PublishPose();
            nextPoseTime = Time.unscaledTime + 1f / poseRateHz;
        }

        if (Time.unscaledTime >= nextRadiationSourceTime)
        {
            PublishRadiationSources();
            nextRadiationSourceTime = Time.unscaledTime + 1f / radiationSourceRateHz;
        }
    }

    private void HandleMessage(string json)
    {
        BridgeMessage request;
        try
        {
            request = JsonUtility.FromJson<BridgeMessage>(json);
        }
        catch (ArgumentException error)
        {
            Debug.LogWarning($"Ignoring malformed ROS bridge message: {error.Message}", this);
            return;
        }

        if (request == null)
        {
            return;
        }

        if (request.type == "velocity")
        {
            ApplyVelocity(request.vx, request.vy, request.wz);
            return;
        }

        if (request.type == "command")
        {
            HandleCommand(request);
        }
    }

    private void ApplyVelocity(float forward, float left, float yaw)
    {
        lastVelocityCommandTime = Time.unscaledTime;
        if (!hasLease || !poweredOn || !standing)
        {
            drive.SetExternalControlEnabled(false);
            return;
        }

        drive.SetExternalVelocity(forward, left, yaw);
    }

    private void HandleCommand(BridgeMessage request)
    {
        bool success = ExecuteStateCommand(request.command, out string responseMessage);

        Send(new BridgeMessage
        {
            type = "response",
            id = request.id,
            command = request.command,
            success = success,
            message = responseMessage
        });
    }

    public bool ClaimLease(out string message)
    {
        return ExecuteStateCommand("claim", out message);
    }

    public bool ReleaseLease(out string message)
    {
        return ExecuteStateCommand("release", out message);
    }

    public bool PowerOn(out string message)
    {
        return ExecuteStateCommand("power_on", out message);
    }

    public bool Stand(out string message)
    {
        return ExecuteStateCommand("stand", out message);
    }

    public bool Sit(out string message)
    {
        return ExecuteStateCommand("sit", out message);
    }

    private bool ExecuteStateCommand(string command, out string responseMessage)
    {
        bool success;
        switch (command)
        {
            case "claim":
                hasLease = true;
                success = true;
                responseMessage = "Lease acquired";
                break;
            case "release":
                hasLease = false;
                StopMotion();
                success = true;
                responseMessage = "Lease released";
                break;
            case "power_on":
                success = hasLease;
                poweredOn = success || poweredOn;
                responseMessage = success ? "Motors powered on" : "Claim the lease before powering on";
                break;
            case "power_off":
                success = hasLease;
                if (success)
                {
                    poweredOn = false;
                    standing = false;
                    StopMotion();
                    proceduralTrot?.SetSitting(true);
                }
                responseMessage = success ? "Motors powered off" : "Claim the lease before powering off";
                break;
            case "stand":
                success = hasLease && poweredOn;
                if (success)
                {
                    standing = true;
                    proceduralTrot?.SetSitting(false);
                }
                responseMessage = success ? "Robot standing" : "A lease and motor power are required to stand";
                break;
            case "sit":
                success = hasLease && poweredOn;
                if (success)
                {
                    standing = false;
                    StopMotion();
                    proceduralTrot?.SetSitting(true);
                }
                responseMessage = success ? "Robot sitting" : "A lease and motor power are required to sit";
                break;
            case "stop":
                success = hasLease;
                StopMotion();
                responseMessage = success ? "Robot stopped" : "Claim the lease before stopping";
                break;
            default:
                success = false;
                responseMessage = $"Unsupported command: {command}";
                break;
        }

        RefreshMotionState();
        return success;
    }

    private void StopMotion()
    {
        lastVelocityCommandTime = float.NegativeInfinity;
        drive.SetExternalControlEnabled(false);
    }

    private void RefreshMotionState()
    {
        drive.SetMovementEnabled(hasLease && poweredOn && standing);
    }

    private void PublishPose()
    {
        float now = Time.unscaledTime;
        float dt = Mathf.Max(now - previousPoseTime, 0.0001f);
        Vector3 relativePosition = Quaternion.Inverse(initialRotation) * (transform.position - initialPosition);
        Quaternion relativeRotation = Quaternion.Inverse(initialRotation) * transform.rotation;
        float unityYaw = Vector3.SignedAngle(Vector3.forward, relativeRotation * Vector3.forward, Vector3.up);
        RosPose bodyPose = ToRosPose(relativePosition, relativeRotation);

        Vector3 worldVelocity = (transform.position - previousPosition) / dt;
        Vector3 bodyVelocity = Quaternion.Inverse(transform.rotation) * worldVelocity;
        float unityYawRate = Vector3.SignedAngle(
            previousRotation * Vector3.forward,
            transform.rotation * Vector3.forward,
            Vector3.up) * Mathf.Deg2Rad / dt;

        Send(new BridgeMessage
        {
            type = "pose",
            x = bodyPose.Position.x,
            y = bodyPose.Position.y,
            z = bodyPose.Position.z,
            yaw = -unityYaw * Mathf.Deg2Rad,
            qx = bodyPose.Rotation.x,
            qy = bodyPose.Rotation.y,
            qz = bodyPose.Rotation.z,
            qw = bodyPose.Rotation.w,
            vision_x = 0f,
            vision_y = 0f,
            vision_z = 0f,
            vision_qx = 0f,
            vision_qy = 0f,
            vision_qz = 0f,
            vision_qw = 1f,
            vx = bodyVelocity.z,
            vy = -bodyVelocity.x,
            wz = -unityYawRate,
            has_lease = hasLease,
            powered_on = poweredOn,
            standing = this.standing
        });

        previousPosition = transform.position;
        previousRotation = transform.rotation;
        previousPoseTime = now;
    }

    private void PublishRadiationSources()
    {
        RadiationSource.CopyActiveSourcesTo(radiationSources);
        radiationSources.Sort((left, right) =>
            string.CompareOrdinal(left.SourceId, right.SourceId));
        var sourceIds = new HashSet<string>();
        var sourceMessages = new RadiationSourceMessage[radiationSources.Count];
        for (int index = 0; index < radiationSources.Count; index++)
        {
            RadiationSource source = radiationSources[index];
            if (!sourceIds.Add(source.SourceId))
            {
                Debug.LogError($"Duplicate RadiationSource id '{source.SourceId}'.", source);
                return;
            }

            Vector3 relativePosition = Quaternion.Inverse(initialRotation) *
                (source.transform.position - initialPosition);
            Quaternion relativeRotation = Quaternion.Inverse(initialRotation) *
                source.transform.rotation;
            RosPose pose = ToRosPose(relativePosition, relativeRotation);
            sourceMessages[index] = new RadiationSourceMessage
            {
                id = source.SourceId,
                isotope = source.Isotope.ToString(),
                activity_bq = (float)source.ActivityBecquerels,
                x = pose.Position.x,
                y = pose.Position.y,
                z = pose.Position.z,
                qx = pose.Rotation.x,
                qy = pose.Rotation.y,
                qz = pose.Rotation.z,
                qw = pose.Rotation.w
            };
        }

        Send(new BridgeMessage
        {
            type = "radiation_sources",
            sequence = radiationSequence++,
            background_rate_per_detector = backgroundRatePerDetector,
            sources = sourceMessages
        });
    }

    private static RosPose ToRosPose(Vector3 unityPosition, Quaternion unityRotation)
    {
        Vector3 rosPosition = UnityVectorToRos(unityPosition);
        Vector3 rosForward = UnityVectorToRos(unityRotation * Vector3.forward);
        Vector3 rosLeft = UnityVectorToRos(unityRotation * Vector3.left);
        Vector3 rosUp = UnityVectorToRos(unityRotation * Vector3.up);
        Quaternion rosRotation = QuaternionFromAxes(rosForward, rosLeft, rosUp);
        return new RosPose(rosPosition, rosRotation);
    }

    private static Vector3 UnityVectorToRos(Vector3 unityVector)
    {
        return new Vector3(unityVector.z, -unityVector.x, unityVector.y);
    }

    private static Quaternion QuaternionFromAxes(Vector3 xAxis, Vector3 yAxis, Vector3 zAxis)
    {
        xAxis.Normalize();
        yAxis.Normalize();
        zAxis.Normalize();

        float m00 = xAxis.x;
        float m01 = yAxis.x;
        float m02 = zAxis.x;
        float m10 = xAxis.y;
        float m11 = yAxis.y;
        float m12 = zAxis.y;
        float m20 = xAxis.z;
        float m21 = yAxis.z;
        float m22 = zAxis.z;
        float trace = m00 + m11 + m22;

        Quaternion q;
        if (trace > 0f)
        {
            float s = Mathf.Sqrt(trace + 1f) * 2f;
            q = new Quaternion(
                (m21 - m12) / s,
                (m02 - m20) / s,
                (m10 - m01) / s,
                0.25f * s);
        }
        else if (m00 > m11 && m00 > m22)
        {
            float s = Mathf.Sqrt(1f + m00 - m11 - m22) * 2f;
            q = new Quaternion(
                0.25f * s,
                (m01 + m10) / s,
                (m02 + m20) / s,
                (m21 - m12) / s);
        }
        else if (m11 > m22)
        {
            float s = Mathf.Sqrt(1f + m11 - m00 - m22) * 2f;
            q = new Quaternion(
                (m01 + m10) / s,
                0.25f * s,
                (m12 + m21) / s,
                (m02 - m20) / s);
        }
        else
        {
            float s = Mathf.Sqrt(1f + m22 - m00 - m11) * 2f;
            q = new Quaternion(
                (m02 + m20) / s,
                (m12 + m21) / s,
                0.25f * s,
                (m10 - m01) / s);
        }

        q.Normalize();
        return q;
    }

    private void NetworkLoop()
    {
        while (!stopRequested.WaitOne(0))
        {
            try
            {
                using (var client = new TcpClient())
                {
                    client.NoDelay = true;
                    lock (socketLock)
                    {
                        activeClient = client;
                    }
                    client.Connect(host, port);

                    using (NetworkStream stream = client.GetStream())
                    using (var reader = new StreamReader(stream, Encoding.UTF8, false, 4096, true))
                    using (var writer = new StreamWriter(stream, new UTF8Encoding(false), 4096, true) { AutoFlush = true })
                    {
                        lock (writerLock)
                        {
                            activeWriter = writer;
                        }
                        Volatile.Write(ref connectionState, 1);

                        while (!stopRequested.WaitOne(0))
                        {
                            string line = reader.ReadLine();
                            if (line == null)
                            {
                                break;
                            }
                            inboundMessages.Enqueue(line);
                        }
                    }
                }
            }
            catch (SocketException)
            {
                Volatile.Write(ref connectionState, 0);
            }
            catch (IOException)
            {
                Volatile.Write(ref connectionState, 0);
            }
            catch (ObjectDisposedException)
            {
                Volatile.Write(ref connectionState, 0);
            }
            finally
            {
                lock (writerLock)
                {
                    activeWriter = null;
                }
                lock (socketLock)
                {
                    activeClient = null;
                }
                Volatile.Write(ref connectionState, 0);
            }

            stopRequested.WaitOne(TimeSpan.FromSeconds(reconnectDelaySeconds));
        }
    }

    private void Send(BridgeMessage message)
    {
        string json = JsonUtility.ToJson(message);
        lock (writerLock)
        {
            if (activeWriter == null)
            {
                return;
            }

            try
            {
                activeWriter.WriteLine(json);
            }
            catch (IOException)
            {
                Volatile.Write(ref connectionState, 0);
            }
            catch (ObjectDisposedException)
            {
                Volatile.Write(ref connectionState, 0);
            }
        }
    }

    private void ReportConnectionChange()
    {
        int currentState = Volatile.Read(ref connectionState);
        if (currentState == reportedConnectionState)
        {
            return;
        }

        reportedConnectionState = currentState;
        if (currentState != 1)
        {
            hasLease = false;
            StopMotion();
            RefreshMotionState();
        }

        if (!logConnectionChanges)
        {
            return;
        }

        if (currentState == 1)
        {
            Debug.Log($"Spot ROS control connected to {host}:{port}.", this);
        }
        else
        {
            Debug.LogWarning($"Spot ROS control waiting for TCP bridge at {host}:{port}.", this);
        }
    }

    private void OnDisable()
    {
        stopRequested.Set();
        lock (socketLock)
        {
            activeClient?.Close();
        }
        if (networkThread != null && networkThread.IsAlive)
        {
            networkThread.Join(2000);
        }
        networkThread = null;
        StopMotion();
        drive.SetMovementEnabled(true);
        Application.targetFrameRate = previousTargetFrameRate;
        Volatile.Write(ref connectionState, -1);
    }

    private void OnDestroy()
    {
        stopRequested.Dispose();
    }
}
