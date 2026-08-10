using System;
using System.Collections;
using System.Net.Sockets;
using System.Runtime.InteropServices;
using System.Threading;
using UnityEngine;

[DisallowMultipleComponent]
[RequireComponent(typeof(Camera))]
public sealed class SpotCameraTcpPublisher : MonoBehaviour
{
    private const int PacketHeaderBytes = 76;
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
    [SerializeField] private int port = 50051;
    [SerializeField, Min(0.1f)] private float reconnectDelaySeconds = 2f;

    [Header("Image")]
    [SerializeField] private string cameraName = "frontleft";
    [SerializeField, Min(16)] private int width = 640;
    [SerializeField, Min(16)] private int height = 480;
    [SerializeField, Range(1f, 5f)] private float framesPerSecond = 5f;
    [SerializeField, Range(1, 100)] private int jpegQuality = 75;

    [Header("Depth")]
    [Tooltip("Trace one ray per N x N output pixels, then fill that block. Four gives 160x120 depth samples for a 640x480 image.")]
    [SerializeField, Range(1, 16)] private int depthDownsample = 4;

    [Header("Diagnostics")]
    [SerializeField] private bool logConnectionChanges = true;

    private readonly object frameLock = new object();
    private readonly object clientLock = new object();
    private readonly AutoResetEvent frameAvailable = new AutoResetEvent(false);
    private readonly ManualResetEvent stopRequested = new ManualResetEvent(false);

    private Camera sensorCamera;
    private RenderTexture renderTexture;
    private Texture2D readbackTexture;
    private RenderTexture originalTargetTexture;
    private bool originalCameraEnabled;
    private Transform robotRoot;
    private byte[] pendingPacket;
    private TcpClient activeClient;
    private Thread senderThread;
    private Coroutine captureCoroutine;
    private int connectionState;
    private int reportedConnectionState = int.MinValue;
    private uint sequence;

    public bool IsConnected => Volatile.Read(ref connectionState) == 1;

    private void Awake()
    {
        sensorCamera = GetComponent<Camera>();
    }

    private void OnEnable()
    {
        width = Mathf.Max(16, width);
        height = Mathf.Max(16, height);
        framesPerSecond = Mathf.Clamp(framesPerSecond, 1f, 5f);
        jpegQuality = Mathf.Clamp(jpegQuality, 1, 100);

        originalTargetTexture = sensorCamera.targetTexture;
        originalCameraEnabled = sensorCamera.enabled;
        robotRoot = transform.root;
        sequence = 0;
        sensorCamera.allowHDR = false;
        sensorCamera.allowMSAA = false;
        renderTexture = new RenderTexture(
            width,
            height,
            24,
            RenderTextureFormat.ARGB32,
            RenderTextureReadWrite.sRGB)
        {
            name = "Spot Camera Stream",
            antiAliasing = 1,
            useMipMap = false,
            autoGenerateMips = false
        };
        renderTexture.Create();

        readbackTexture = new Texture2D(width, height, TextureFormat.RGB24, false);
        sensorCamera.targetTexture = renderTexture;
        sensorCamera.enabled = false;

        stopRequested.Reset();
        Volatile.Write(ref connectionState, 0);
        senderThread = new Thread(SenderLoop)
        {
            IsBackground = true,
            Name = "Spot camera TCP sender"
        };
        senderThread.Start();
        captureCoroutine = StartCoroutine(CaptureLoop());
    }

    private void Update()
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
            Debug.Log($"Spot camera `{cameraName}` connected to {host}:{port}.", this);
        }
        else if (currentState == 0)
        {
            Debug.LogWarning($"Spot camera `{cameraName}` waiting for TCP bridge at {host}:{port}.", this);
        }
    }

    private IEnumerator CaptureLoop()
    {
        var endOfFrame = new WaitForEndOfFrame();
        float nextCaptureTime = 0f;

        while (true)
        {
            yield return endOfFrame;

            if (Time.unscaledTime < nextCaptureTime)
            {
                continue;
            }

            nextCaptureTime = Time.unscaledTime + 1f / framesPerSecond;
            CaptureFrame();
        }
    }

    private void CaptureFrame()
    {
        RenderTexture previousActive = RenderTexture.active;
        try
        {
            sensorCamera.targetTexture = renderTexture;
            sensorCamera.Render();
            RenderTexture.active = renderTexture;
            readbackTexture.ReadPixels(new Rect(0f, 0f, width, height), 0, 0, false);
            readbackTexture.Apply(false, false);

            byte[] jpeg = readbackTexture.EncodeToJPG(jpegQuality);
            byte[] depthMillimeters = CaptureRegisteredDepth();

            byte[] packet = BuildPacket(jpeg, depthMillimeters);
            lock (frameLock)
            {
                pendingPacket = packet;
            }
            frameAvailable.Set();
        }
        finally
        {
            RenderTexture.active = previousActive;
        }
    }

    private byte[] CaptureRegisteredDepth()
    {
        byte[] output = new byte[width * height * sizeof(ushort)];
        int step = Mathf.Clamp(depthDownsample, 1, 16);
        float farClip = sensorCamera.farClipPlane;
        Vector3 cameraForward = transform.forward;

        // ROS Image row zero is the top row. Unity viewport Y grows upward, so
        // sample each output block at its vertically flipped center.
        for (int top = 0; top < height; top += step)
        {
            int blockHeight = Mathf.Min(step, height - top);
            float viewportY = 1f - (top + 0.5f * blockHeight) / height;
            for (int left = 0; left < width; left += step)
            {
                int blockWidth = Mathf.Min(step, width - left);
                float viewportX = (left + 0.5f * blockWidth) / width;
                Ray ray = sensorCamera.ViewportPointToRay(new Vector3(viewportX, viewportY, 0f));

                ushort millimeters = 0;
                if (Physics.Raycast(
                    ray,
                    out RaycastHit hit,
                    farClip,
                    sensorCamera.cullingMask,
                    QueryTriggerInteraction.Ignore))
                {
                    // Registered depth is optical-axis Z, not radial ray range.
                    float opticalDepth = hit.distance * Vector3.Dot(ray.direction, cameraForward);
                    millimeters = (ushort)Mathf.Clamp(
                        Mathf.RoundToInt(opticalDepth * 1000f),
                        1,
                        ushort.MaxValue);
                }

                for (int y = top; y < top + blockHeight; y++)
                {
                    int byteOffset = (y * width + left) * sizeof(ushort);
                    for (int x = 0; x < blockWidth; x++)
                    {
                        output[byteOffset++] = (byte)millimeters;
                        output[byteOffset++] = (byte)(millimeters >> 8);
                    }
                }
            }
        }
        return output;
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
                        SendFrames(stream);
                    }
                }
            }
            catch (SocketException)
            {
                Volatile.Write(ref connectionState, 0);
            }
            catch (System.IO.IOException)
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
            }

            if (!stopRequested.WaitOne(TimeSpan.FromSeconds(reconnectDelaySeconds)))
            {
                continue;
            }
        }
    }

    private void SendFrames(NetworkStream stream)
    {
        while (!stopRequested.WaitOne(0))
        {
            int signaled = WaitHandle.WaitAny(
                new WaitHandle[] { stopRequested, frameAvailable },
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
            lock (frameLock)
            {
                packet = pendingPacket;
                pendingPacket = null;
            }
            if (packet == null || packet.Length == 0)
            {
                continue;
            }

            byte[] header =
            {
                (byte)(packet.Length >> 24),
                (byte)(packet.Length >> 16),
                (byte)(packet.Length >> 8),
                (byte)packet.Length
            };
            stream.Write(header, 0, header.Length);
            stream.Write(packet, 0, packet.Length);
        }
    }

    private byte[] BuildPacket(byte[] jpeg, byte[] depthMillimeters)
    {
        byte[] packet = new byte[PacketHeaderBytes + jpeg.Length + depthMillimeters.Length];
        int offset = 0;
        packet[offset++] = (byte)'U';
        packet[offset++] = (byte)'C';
        packet[offset++] = (byte)'A';
        packet[offset++] = (byte)'M';
        WriteUInt16LittleEndian(packet, ref offset, ProtocolVersion);
        WriteUInt16LittleEndian(packet, ref offset, 0);
        WriteUInt32LittleEndian(packet, ref offset, sequence++);
        WriteUInt16LittleEndian(packet, ref offset, checked((ushort)width));
        WriteUInt16LittleEndian(packet, ref offset, checked((ushort)height));

        float fy = 0.5f * height / Mathf.Tan(0.5f * sensorCamera.fieldOfView * Mathf.Deg2Rad);
        float fx = fy;
        WriteSingleLittleEndian(packet, ref offset, fx);
        WriteSingleLittleEndian(packet, ref offset, fy);
        WriteSingleLittleEndian(packet, ref offset, 0.5f * (width - 1));
        WriteSingleLittleEndian(packet, ref offset, 0.5f * (height - 1));
        WriteSingleLittleEndian(packet, ref offset, sensorCamera.nearClipPlane);
        WriteSingleLittleEndian(packet, ref offset, sensorCamera.farClipPlane);

        Vector3 mountPosition = robotRoot.InverseTransformPoint(transform.position);
        Quaternion mountRotation = Quaternion.Inverse(robotRoot.rotation) * transform.rotation;
        RosPose mountPose = ToRosOpticalPose(mountPosition, mountRotation);
        WriteSingleLittleEndian(packet, ref offset, mountPose.Position.x);
        WriteSingleLittleEndian(packet, ref offset, mountPose.Position.y);
        WriteSingleLittleEndian(packet, ref offset, mountPose.Position.z);
        WriteSingleLittleEndian(packet, ref offset, mountPose.Rotation.x);
        WriteSingleLittleEndian(packet, ref offset, mountPose.Rotation.y);
        WriteSingleLittleEndian(packet, ref offset, mountPose.Rotation.z);
        WriteSingleLittleEndian(packet, ref offset, mountPose.Rotation.w);
        WriteUInt32LittleEndian(packet, ref offset, checked((uint)jpeg.Length));
        WriteUInt32LittleEndian(packet, ref offset, checked((uint)depthMillimeters.Length));

        Buffer.BlockCopy(jpeg, 0, packet, offset, jpeg.Length);
        offset += jpeg.Length;
        Buffer.BlockCopy(depthMillimeters, 0, packet, offset, depthMillimeters.Length);
        return packet;
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

    private static RosPose ToRosOpticalPose(Vector3 unityPosition, Quaternion unityRotation)
    {
        Vector3 rosPosition = UnityVectorToRos(unityPosition);
        Vector3 opticalRight = UnityVectorToRos(unityRotation * Vector3.right);
        Vector3 opticalDown = UnityVectorToRos(unityRotation * Vector3.down);
        Vector3 opticalForward = UnityVectorToRos(unityRotation * Vector3.forward);
        return new RosPose(
            rosPosition,
            QuaternionFromAxes(opticalRight, opticalDown, opticalForward));
    }

    private static Vector3 UnityVectorToRos(Vector3 unityVector)
    {
        return new Vector3(unityVector.z, -unityVector.x, unityVector.y);
    }

    private static Quaternion QuaternionFromAxes(Vector3 xAxis, Vector3 yAxis, Vector3 zAxis)
    {
        Matrix4x4 rotation = Matrix4x4.identity;
        rotation.SetColumn(0, new Vector4(xAxis.x, xAxis.y, xAxis.z, 0f));
        rotation.SetColumn(1, new Vector4(yAxis.x, yAxis.y, yAxis.z, 0f));
        rotation.SetColumn(2, new Vector4(zAxis.x, zAxis.y, zAxis.z, 0f));
        return rotation.rotation;
    }

    private void OnDisable()
    {
        if (captureCoroutine != null)
        {
            StopCoroutine(captureCoroutine);
            captureCoroutine = null;
        }

        stopRequested.Set();
        frameAvailable.Set();
        lock (clientLock)
        {
            activeClient?.Close();
        }
        if (senderThread != null && senderThread.IsAlive)
        {
            senderThread.Join(2000);
        }
        senderThread = null;

        lock (frameLock)
        {
            pendingPacket = null;
        }

        if (sensorCamera != null)
        {
            sensorCamera.targetTexture = originalTargetTexture;
            sensorCamera.enabled = originalCameraEnabled;
        }
        if (renderTexture != null)
        {
            renderTexture.Release();
            Destroy(renderTexture);
            renderTexture = null;
        }
        if (readbackTexture != null)
        {
            Destroy(readbackTexture);
            readbackTexture = null;
        }

        Volatile.Write(ref connectionState, -1);
    }

    private void OnDestroy()
    {
        frameAvailable.Dispose();
        stopRequested.Dispose();
    }
}
