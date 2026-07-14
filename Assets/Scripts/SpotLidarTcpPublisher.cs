using System;
using System.IO;
using System.Net.Sockets;
using System.Runtime.InteropServices;
using System.Threading;
using Unity.Collections;
using Unity.Jobs;
using UnityEngine;

[DisallowMultipleComponent]
public sealed class SpotLidarTcpPublisher : MonoBehaviour
{
    private const int PacketHeaderBytes = 68;
    private const ushort ProtocolVersion = 1;

    [StructLayout(LayoutKind.Explicit)]
    private struct FloatBits
    {
        [FieldOffset(0)] public float Float;
        [FieldOffset(0)] public uint UInt;
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
    [SerializeField] private int port = 50053;
    [SerializeField, Min(0.1f)] private float reconnectDelaySeconds = 2f;

    [Header("Scan")]
    [SerializeField, Range(1f, 360f)] private float horizontalFieldOfViewDegrees = 270f;
    [SerializeField, Range(0.1f, 10f)] private float horizontalResolutionDegrees = 1.5f;
    [SerializeField, Range(0f, 90f)] private float verticalFieldOfViewDegrees = 25f;
    [SerializeField, Range(1, 32)] private int verticalChannels = 3;
    [SerializeField, Range(1f, 30f)] private float scansPerSecond = 5f;
    [SerializeField, Min(0.01f)] private float minimumRange = 0.25f;
    [SerializeField, Min(0.1f)] private float maximumRange = 20f;
    [SerializeField] private LayerMask environmentLayers = ~0;

    [Header("Measurement Model")]
    [SerializeField, Min(0f)] private float noiseBaseSigmaMeters = 0.01f;
    [SerializeField, Min(0f)] private float noiseRangeSigmaFraction = 0.002f;
    [SerializeField, Range(0f, 0.25f)] private float dropoutProbability = 0.005f;
    [SerializeField] private int randomSeed = 1337;

    [Header("Diagnostics")]
    [SerializeField] private bool logConnectionChanges = true;

    private readonly object packetLock = new object();
    private readonly object clientLock = new object();
    private readonly AutoResetEvent packetAvailable = new AutoResetEvent(false);
    private readonly ManualResetEvent stopRequested = new ManualResetEvent(false);

    private NativeArray<RaycastCommand> commands;
    private NativeArray<RaycastHit> hits;
    private Vector3[] localDirections;
    private Transform robotRoot;
    private System.Random noiseRandom;
    private byte[] pendingPacket;
    private TcpClient activeClient;
    private Thread senderThread;
    private int horizontalCount;
    private int rayCount;
    private float horizontalMin;
    private float horizontalIncrement;
    private float verticalMin;
    private float verticalIncrement;
    private uint sequence;
    private float nextScanTime;
    private int connectionState;
    private int reportedConnectionState = int.MinValue;

    public bool IsConnected => Volatile.Read(ref connectionState) == 1;

    private void OnEnable()
    {
        if (maximumRange <= minimumRange)
        {
            throw new InvalidOperationException("Lidar maximumRange must exceed minimumRange.");
        }

        horizontalCount = Mathf.Max(
            2,
            Mathf.RoundToInt(horizontalFieldOfViewDegrees / horizontalResolutionDegrees) + 1);
        verticalChannels = Mathf.Max(1, verticalChannels);
        rayCount = horizontalCount * verticalChannels;
        horizontalMin = -0.5f * horizontalFieldOfViewDegrees * Mathf.Deg2Rad;
        horizontalIncrement = horizontalFieldOfViewDegrees * Mathf.Deg2Rad /
            (horizontalCount - 1);
        verticalMin = -0.5f * verticalFieldOfViewDegrees * Mathf.Deg2Rad;
        verticalIncrement = verticalChannels > 1
            ? verticalFieldOfViewDegrees * Mathf.Deg2Rad / (verticalChannels - 1)
            : 0f;
        commands = new NativeArray<RaycastCommand>(rayCount, Allocator.Persistent);
        hits = new NativeArray<RaycastHit>(rayCount, Allocator.Persistent);
        localDirections = new Vector3[rayCount];
        robotRoot = transform.parent;
        BuildLocalDirections();
        noiseRandom = new System.Random(randomSeed);
        sequence = 0;
        nextScanTime = Time.unscaledTime;

        stopRequested.Reset();
        Volatile.Write(ref connectionState, 0);
        senderThread = new Thread(SenderLoop)
        {
            IsBackground = true,
            Name = "Spot lidar TCP sender"
        };
        senderThread.Start();
    }

    private void Update()
    {
        ReportConnectionChange();
        if (!IsConnected || Time.unscaledTime < nextScanTime)
        {
            return;
        }

        nextScanTime = Time.unscaledTime + 1f / scansPerSecond;
        CaptureScan();
    }

    private void CaptureScan()
    {
        Vector3 worldOrigin = transform.position;
        Quaternion worldRotation = transform.rotation;

        for (int rayIndex = 0; rayIndex < rayCount; rayIndex++)
        {
            commands[rayIndex] = new RaycastCommand(
                worldOrigin,
                worldRotation * localDirections[rayIndex],
                maximumRange,
                environmentLayers,
                1);
        }

        JobHandle handle = RaycastCommand.ScheduleBatch(commands, hits, 32);
        handle.Complete();

        byte[] packet = new byte[PacketHeaderBytes + rayCount * sizeof(float)];
        int offset = 0;
        packet[offset++] = (byte)'U';
        packet[offset++] = (byte)'L';
        packet[offset++] = (byte)'D';
        packet[offset++] = (byte)'R';
        WriteUInt16LittleEndian(packet, ref offset, ProtocolVersion);
        WriteUInt16LittleEndian(packet, ref offset, 0);
        WriteUInt32LittleEndian(packet, ref offset, sequence++);
        WriteUInt16LittleEndian(packet, ref offset, checked((ushort)horizontalCount));
        WriteUInt16LittleEndian(packet, ref offset, checked((ushort)verticalChannels));
        WriteSingleLittleEndian(packet, ref offset, horizontalMin);
        WriteSingleLittleEndian(packet, ref offset, horizontalIncrement);
        WriteSingleLittleEndian(packet, ref offset, verticalMin);
        WriteSingleLittleEndian(packet, ref offset, verticalIncrement);
        WriteSingleLittleEndian(packet, ref offset, minimumRange);
        WriteSingleLittleEndian(packet, ref offset, maximumRange);
        RosPose mountPose = ToRosPose(transform.localPosition, transform.localRotation);
        WriteSingleLittleEndian(packet, ref offset, mountPose.Position.x);
        WriteSingleLittleEndian(packet, ref offset, mountPose.Position.y);
        WriteSingleLittleEndian(packet, ref offset, mountPose.Position.z);
        WriteSingleLittleEndian(packet, ref offset, mountPose.Rotation.x);
        WriteSingleLittleEndian(packet, ref offset, mountPose.Rotation.y);
        WriteSingleLittleEndian(packet, ref offset, mountPose.Rotation.z);
        WriteSingleLittleEndian(packet, ref offset, mountPose.Rotation.w);

        for (int index = 0; index < rayCount; index++)
        {
            RaycastHit hit = hits[index];
            float distance = float.PositiveInfinity;
            if (hit.collider != null &&
                (robotRoot == null || !hit.collider.transform.IsChildOf(robotRoot)) &&
                noiseRandom.NextDouble() >= dropoutProbability)
            {
                float sigma = noiseBaseSigmaMeters + hit.distance * noiseRangeSigmaFraction;
                distance = Mathf.Clamp(
                    hit.distance + sigma * NextGaussian(),
                    minimumRange,
                    maximumRange);
            }
            WriteSingleLittleEndian(packet, ref offset, distance);
        }

        lock (packetLock)
        {
            pendingPacket = packet;
        }
        packetAvailable.Set();
    }

    private void BuildLocalDirections()
    {
        int rayIndex = 0;
        for (int verticalIndex = 0; verticalIndex < verticalChannels; verticalIndex++)
        {
            float elevation = verticalMin + verticalIndex * verticalIncrement;
            float cosElevation = Mathf.Cos(elevation);
            float sinElevation = Mathf.Sin(elevation);
            for (int horizontalIndex = 0; horizontalIndex < horizontalCount; horizontalIndex++)
            {
                float azimuth = horizontalMin + horizontalIndex * horizontalIncrement;
                localDirections[rayIndex++] = new Vector3(
                    -cosElevation * Mathf.Sin(azimuth),
                    sinElevation,
                    cosElevation * Mathf.Cos(azimuth));
            }
        }
    }

    private float NextGaussian()
    {
        double u1 = 1.0 - noiseRandom.NextDouble();
        double u2 = 1.0 - noiseRandom.NextDouble();
        return (float)(Math.Sqrt(-2.0 * Math.Log(u1)) * Math.Cos(2.0 * Math.PI * u2));
    }

    private void SenderLoop()
    {
        while (!stopRequested.WaitOne(0))
        {
            try
            {
                using (var client = new TcpClient())
                {
                    client.NoDelay = true;
                    lock (clientLock)
                    {
                        activeClient = client;
                    }
                    client.Connect(host, port);
                    Volatile.Write(ref connectionState, 1);
                    using (NetworkStream stream = client.GetStream())
                    {
                        SendPackets(stream);
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
                lock (clientLock)
                {
                    activeClient = null;
                }
                Volatile.Write(ref connectionState, 0);
            }

            stopRequested.WaitOne(TimeSpan.FromSeconds(reconnectDelaySeconds));
        }
    }

    private void SendPackets(NetworkStream stream)
    {
        while (!stopRequested.WaitOne(0))
        {
            int signaled = WaitHandle.WaitAny(
                new WaitHandle[] { stopRequested, packetAvailable },
                TimeSpan.FromMilliseconds(250));
            if (signaled == 0)
            {
                return;
            }
            if (signaled == WaitHandle.WaitTimeout)
            {
                continue;
            }

            byte[] packet;
            lock (packetLock)
            {
                packet = pendingPacket;
                pendingPacket = null;
            }
            if (packet == null)
            {
                continue;
            }

            byte[] lengthHeader =
            {
                (byte)(packet.Length >> 24),
                (byte)(packet.Length >> 16),
                (byte)(packet.Length >> 8),
                (byte)packet.Length
            };
            stream.Write(lengthHeader, 0, lengthHeader.Length);
            stream.Write(packet, 0, packet.Length);
        }
    }

    private void ReportConnectionChange()
    {
        if (!logConnectionChanges)
        {
            return;
        }
        int currentState = Volatile.Read(ref connectionState);
        if (currentState == reportedConnectionState)
        {
            return;
        }
        reportedConnectionState = currentState;
        if (currentState == 1)
        {
            Debug.Log($"Spot lidar connected to {host}:{port} ({rayCount} rays at {scansPerSecond:F1} Hz).", this);
        }
        else
        {
            Debug.LogWarning($"Spot lidar waiting for TCP bridge at {host}:{port}.", this);
        }
    }

    private static void WriteUInt16LittleEndian(byte[] destination, ref int offset, ushort value)
    {
        destination[offset++] = (byte)value;
        destination[offset++] = (byte)(value >> 8);
    }

    private static void WriteUInt32LittleEndian(byte[] destination, ref int offset, uint value)
    {
        destination[offset++] = (byte)value;
        destination[offset++] = (byte)(value >> 8);
        destination[offset++] = (byte)(value >> 16);
        destination[offset++] = (byte)(value >> 24);
    }

    private static void WriteSingleLittleEndian(byte[] destination, ref int offset, float value)
    {
        FloatBits bits = new FloatBits { Float = value };
        WriteUInt32LittleEndian(destination, ref offset, bits.UInt);
    }

    private static RosPose ToRosPose(Vector3 unityPosition, Quaternion unityRotation)
    {
        Vector3 rosPosition = UnityVectorToRos(unityPosition);
        Vector3 rosForward = UnityVectorToRos(unityRotation * Vector3.forward);
        Vector3 rosLeft = UnityVectorToRos(unityRotation * Vector3.left);
        Vector3 rosUp = UnityVectorToRos(unityRotation * Vector3.up);
        return new RosPose(rosPosition, QuaternionFromAxes(rosForward, rosLeft, rosUp));
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

    private void OnDisable()
    {
        stopRequested.Set();
        packetAvailable.Set();
        lock (clientLock)
        {
            activeClient?.Close();
        }
        if (senderThread != null && senderThread.IsAlive)
        {
            senderThread.Join(2000);
        }
        senderThread = null;
        if (commands.IsCreated)
        {
            commands.Dispose();
        }
        if (hits.IsCreated)
        {
            hits.Dispose();
        }
        localDirections = null;
        robotRoot = null;
        Volatile.Write(ref connectionState, -1);
    }

    private void OnDestroy()
    {
        packetAvailable.Dispose();
        stopRequested.Dispose();
    }
}
