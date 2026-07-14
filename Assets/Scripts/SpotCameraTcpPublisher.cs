using System;
using System.Collections;
using System.Net.Sockets;
using System.Threading;
using UnityEngine;

[DisallowMultipleComponent]
[RequireComponent(typeof(Camera))]
public sealed class SpotCameraTcpPublisher : MonoBehaviour
{
    [Header("Bridge")]
    [SerializeField] private string host = "127.0.0.1";
    [SerializeField] private int port = 50051;
    [SerializeField, Min(0.1f)] private float reconnectDelaySeconds = 2f;

    [Header("Image")]
    [SerializeField, Min(16)] private int width = 640;
    [SerializeField, Min(16)] private int height = 480;
    [SerializeField, Range(1f, 30f)] private float framesPerSecond = 5f;
    [SerializeField, Range(1, 100)] private int jpegQuality = 75;

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
    private byte[] pendingFrame;
    private TcpClient activeClient;
    private Thread senderThread;
    private Coroutine captureCoroutine;
    private int connectionState;
    private int reportedConnectionState = int.MinValue;

    public bool IsConnected => Volatile.Read(ref connectionState) == 1;

    private void Awake()
    {
        sensorCamera = GetComponent<Camera>();
    }

    private void OnEnable()
    {
        width = Mathf.Max(16, width);
        height = Mathf.Max(16, height);
        framesPerSecond = Mathf.Clamp(framesPerSecond, 1f, 30f);
        jpegQuality = Mathf.Clamp(jpegQuality, 1, 100);

        originalTargetTexture = sensorCamera.targetTexture;
        originalCameraEnabled = sensorCamera.enabled;
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
            Debug.Log($"Spot camera connected to {host}:{port}.", this);
        }
        else if (currentState == 0)
        {
            Debug.LogWarning($"Spot camera waiting for TCP bridge at {host}:{port}.", this);
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
            lock (frameLock)
            {
                pendingFrame = jpeg;
            }
            frameAvailable.Set();
        }
        finally
        {
            RenderTexture.active = previousActive;
        }
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

            byte[] frame;
            lock (frameLock)
            {
                frame = pendingFrame;
                pendingFrame = null;
            }
            if (frame == null || frame.Length == 0)
            {
                continue;
            }

            byte[] header =
            {
                (byte)(frame.Length >> 24),
                (byte)(frame.Length >> 16),
                (byte)(frame.Length >> 8),
                (byte)frame.Length
            };
            stream.Write(header, 0, header.Length);
            stream.Write(frame, 0, frame.Length);
        }
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
            pendingFrame = null;
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
