using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using UnityEditor;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.SceneManagement;

public static class BarrelDatasetCapture
{
    private const int ImageWidth = 640;
    private const int ImageHeight = 480;
    private const int ViewsPerBarrel = 5;
    private const int NegativeViewsPerBarrel = 3;
    private const int MaximumAttemptsPerBarrel = 40;
    private const int MinimumFocusPixels = 750;

    [Serializable]
    private sealed class MaskRecord
    {
        public int instance_id;
        public string instance_name;
        public string path;
        public int pixels;
    }

    [Serializable]
    private sealed class FrameRecord
    {
        public string image;
        public int focus_instance_id;
        public string focus_instance_name;
        public Vector3 camera_position;
        public Quaternion camera_rotation;
        public MaskRecord[] masks;
    }

    private sealed class BarrelInstance
    {
        public int Id;
        public GameObject Root;
        public Renderer[] Renderers;
        public Bounds Bounds;
    }

    private sealed class CameraState
    {
        public Vector3 Position;
        public Quaternion Rotation;
        public RenderTexture TargetTexture;
        public CameraClearFlags ClearFlags;
        public Color BackgroundColor;
        public bool AllowHdr;
        public bool AllowMsaa;
    }

    [MenuItem("LAMP/Training/Capture Barrel Dataset")]
    public static void CaptureDataset()
    {
        Scene scene = SceneManager.GetActiveScene();
        Camera camera = Camera.main != null
            ? Camera.main
            : UnityEngine.Object.FindAnyObjectByType<Camera>();
        if (camera == null)
        {
            throw new InvalidOperationException("The active scene has no camera.");
        }

        List<BarrelInstance> barrels = FindBarrels(scene);
        if (barrels.Count == 0)
        {
            throw new InvalidOperationException(
                "No active scene objects beginning with `barrell_plastic` or " +
                "`barrell_yellow` were found.");
        }

        string projectRoot = Directory.GetParent(Application.dataPath).FullName;
        string timestamp = DateTime.Now.ToString("yyyyMMdd_HHmmss");
        string datasetRoot = Path.Combine(
            projectRoot,
            "TrainingData",
            "barrell",
            scene.name.Replace(' ', '_') + "_" + timestamp);
        string imagesDirectory = Path.Combine(datasetRoot, "images");
        string masksDirectory = Path.Combine(datasetRoot, "instance_masks");
        Directory.CreateDirectory(imagesDirectory);
        Directory.CreateDirectory(masksDirectory);

        string manifestPath = Path.Combine(datasetRoot, "manifest.jsonl");
        File.WriteAllText(
            Path.Combine(datasetRoot, "classes.txt"),
            "barrell" + Environment.NewLine);

        Renderer[] sceneRenderers = UnityEngine.Object
            .FindObjectsByType<Renderer>(FindObjectsInactive.Exclude)
            .Where(renderer => renderer.gameObject.scene == scene)
            .ToArray();
        var cameraState = new CameraState
        {
            Position = camera.transform.position,
            Rotation = camera.transform.rotation,
            TargetTexture = camera.targetTexture,
            ClearFlags = camera.clearFlags,
            BackgroundColor = camera.backgroundColor,
            AllowHdr = camera.allowHDR,
            AllowMsaa = camera.allowMSAA,
        };

        Shader unlitShader = Shader.Find("Hidden/LAMP/DatasetMask");
        if (unlitShader == null)
        {
            throw new InvalidOperationException(
                "Could not find the Hidden/LAMP/DatasetMask shader.");
        }

        Material blackMaterial = CreateMaskMaterial(unlitShader, Color.black, "Dataset Mask Black");
        Material whiteMaterial = CreateMaskMaterial(unlitShader, Color.white, "Dataset Mask White");
        RenderTexture rgbTarget = CreateRenderTexture("Dataset RGB", RenderTextureReadWrite.sRGB);
        RenderTexture maskTarget = CreateRenderTexture("Dataset Mask", RenderTextureReadWrite.Linear);
        Texture2D rgbReadback = new Texture2D(ImageWidth, ImageHeight, TextureFormat.RGB24, false);
        Texture2D maskReadback = new Texture2D(ImageWidth, ImageHeight, TextureFormat.RGB24, false, true);

        int frameNumber = 0;
        int rejectedCandidates = 0;
        var random = new System.Random(1701);

        try
        {
            camera.allowHDR = false;
            camera.allowMSAA = false;

            for (int focusIndex = 0; focusIndex < barrels.Count; focusIndex++)
            {
                BarrelInstance focus = barrels[focusIndex];
                int acceptedViews = 0;
                int attempts = 0;
                while (acceptedViews < ViewsPerBarrel && attempts < MaximumAttemptsPerBarrel)
                {
                    attempts++;
                    PositionCamera(camera, focus.Bounds, acceptedViews, attempts, random);
                    Physics.SyncTransforms();
                    if (!HasClearLineOfSight(camera, focus.Bounds))
                    {
                        rejectedCandidates++;
                        continue;
                    }
                    int focusPixels = RenderMask(
                        camera,
                        sceneRenderers,
                        focus.Renderers,
                        blackMaterial,
                        whiteMaterial,
                        maskTarget,
                        maskReadback);
                    if (focusPixels < MinimumFocusPixels)
                    {
                        rejectedCandidates++;
                        continue;
                    }

                    float progress = (float)(focusIndex * ViewsPerBarrel + acceptedViews) /
                        (barrels.Count * ViewsPerBarrel);
                    if (EditorUtility.DisplayCancelableProgressBar(
                        "Capturing barrel dataset",
                        $"{focus.Root.name}: view {acceptedViews + 1}/{ViewsPerBarrel}",
                        progress))
                    {
                        throw new OperationCanceledException("Dataset capture canceled.");
                    }

                    string frameStem = $"frame_{frameNumber:D5}";
                    string imageRelativePath = Path.Combine("images", frameStem + ".png");
                    RenderRgb(
                        camera,
                        rgbTarget,
                        rgbReadback,
                        Path.Combine(datasetRoot, imageRelativePath));

                    var maskRecords = new List<MaskRecord>();
                    foreach (BarrelInstance barrel in barrels)
                    {
                        int pixels = RenderMask(
                            camera,
                            sceneRenderers,
                            barrel.Renderers,
                            blackMaterial,
                            whiteMaterial,
                            maskTarget,
                            maskReadback);
                        if (pixels > 0)
                        {
                            string maskRelativePath = Path.Combine(
                                "instance_masks",
                                $"{frameStem}_instance_{barrel.Id:D3}.png");
                            File.WriteAllBytes(
                                Path.Combine(datasetRoot, maskRelativePath),
                                maskReadback.EncodeToPNG());
                            maskRecords.Add(new MaskRecord
                            {
                                instance_id = barrel.Id,
                                instance_name = barrel.Root.name,
                                path = maskRelativePath.Replace('\\', '/'),
                                pixels = pixels,
                            });
                        }
                    }

                    var frameRecord = new FrameRecord
                    {
                        image = imageRelativePath.Replace('\\', '/'),
                        focus_instance_id = focus.Id,
                        focus_instance_name = focus.Root.name,
                        camera_position = camera.transform.position,
                        camera_rotation = camera.transform.rotation,
                        masks = maskRecords.ToArray(),
                    };
                    File.AppendAllText(
                        manifestPath,
                        JsonUtility.ToJson(frameRecord) + Environment.NewLine);
                    frameNumber++;
                    acceptedViews++;
                }

                if (acceptedViews < ViewsPerBarrel)
                {
                    Debug.LogWarning(
                        $"Only captured {acceptedViews}/{ViewsPerBarrel} usable views " +
                        $"for `{focus.Root.name}` after {attempts} attempts.");
                }
            }

            int requestedNegativeFrames = barrels.Count * NegativeViewsPerBarrel;
            int acceptedNegativeFrames = 0;
            int negativeAttempts = 0;
            Renderer[] allBarrelRenderers = barrels
                .SelectMany(barrel => barrel.Renderers)
                .Distinct()
                .ToArray();
            while (acceptedNegativeFrames < requestedNegativeFrames &&
                   negativeAttempts < requestedNegativeFrames * 50)
            {
                negativeAttempts++;
                BarrelInstance anchor = barrels[negativeAttempts % barrels.Count];
                PositionCamera(camera, anchor.Bounds, 0, negativeAttempts, random);
                RotateCameraAwayFromTarget(camera, anchor.Bounds, random);
                Physics.SyncTransforms();
                if (!IsCameraPositionClear(camera))
                {
                    rejectedCandidates++;
                    continue;
                }

                int barrelPixels = RenderMask(
                    camera,
                    sceneRenderers,
                    allBarrelRenderers,
                    blackMaterial,
                    whiteMaterial,
                    maskTarget,
                    maskReadback);
                if (barrelPixels > 20)
                {
                    rejectedCandidates++;
                    continue;
                }

                float progress = (float)acceptedNegativeFrames / requestedNegativeFrames;
                if (EditorUtility.DisplayCancelableProgressBar(
                    "Capturing barrel dataset negatives",
                    $"Empty warehouse view {acceptedNegativeFrames + 1}/{requestedNegativeFrames}",
                    progress))
                {
                    throw new OperationCanceledException("Dataset capture canceled.");
                }

                string frameStem = $"frame_{frameNumber:D5}";
                string imageRelativePath = Path.Combine("images", frameStem + ".png");
                RenderRgb(
                    camera,
                    rgbTarget,
                    rgbReadback,
                    Path.Combine(datasetRoot, imageRelativePath));
                var frameRecord = new FrameRecord
                {
                    image = imageRelativePath.Replace('\\', '/'),
                    focus_instance_id = -1,
                    focus_instance_name = "negative",
                    camera_position = camera.transform.position,
                    camera_rotation = camera.transform.rotation,
                    masks = Array.Empty<MaskRecord>(),
                };
                File.AppendAllText(
                    manifestPath,
                    JsonUtility.ToJson(frameRecord) + Environment.NewLine);
                frameNumber++;
                acceptedNegativeFrames++;
            }

            if (acceptedNegativeFrames < requestedNegativeFrames)
            {
                Debug.LogWarning(
                    $"Only captured {acceptedNegativeFrames}/{requestedNegativeFrames} " +
                    $"barrel-free negative views after {negativeAttempts} attempts.");
            }
        }
        finally
        {
            EditorUtility.ClearProgressBar();
            camera.transform.SetPositionAndRotation(cameraState.Position, cameraState.Rotation);
            camera.targetTexture = cameraState.TargetTexture;
            camera.clearFlags = cameraState.ClearFlags;
            camera.backgroundColor = cameraState.BackgroundColor;
            camera.allowHDR = cameraState.AllowHdr;
            camera.allowMSAA = cameraState.AllowMsaa;
            RenderTexture.active = null;
            rgbTarget.Release();
            maskTarget.Release();
            UnityEngine.Object.DestroyImmediate(rgbTarget);
            UnityEngine.Object.DestroyImmediate(maskTarget);
            UnityEngine.Object.DestroyImmediate(rgbReadback);
            UnityEngine.Object.DestroyImmediate(maskReadback);
            UnityEngine.Object.DestroyImmediate(blackMaterial);
            UnityEngine.Object.DestroyImmediate(whiteMaterial);
        }

        Debug.Log(
            $"Captured {frameNumber} barrel training frames for {barrels.Count} " +
            $"instances ({rejectedCandidates} rejected camera candidates) at `{datasetRoot}`.");
        EditorUtility.RevealInFinder(datasetRoot);
    }

    private static List<BarrelInstance> FindBarrels(Scene scene)
    {
        GameObject[] candidates = Resources.FindObjectsOfTypeAll<GameObject>()
            .Where(gameObject =>
                gameObject.scene == scene &&
                gameObject.activeInHierarchy &&
                IsBarrelName(gameObject.name) &&
                (gameObject.transform.parent == null ||
                 !IsBarrelName(gameObject.transform.parent.name)))
            .OrderBy(gameObject => gameObject.name, StringComparer.Ordinal)
            .ThenBy(gameObject => gameObject.transform.position.x)
            .ThenBy(gameObject => gameObject.transform.position.z)
            .ToArray();

        var barrels = new List<BarrelInstance>();
        for (int index = 0; index < candidates.Length; index++)
        {
            Renderer[] renderers = candidates[index].GetComponentsInChildren<Renderer>(false);
            if (renderers.Length == 0)
            {
                Debug.LogWarning($"Skipping `{candidates[index].name}` because it has no renderers.");
                continue;
            }
            Bounds bounds = renderers[0].bounds;
            for (int rendererIndex = 1; rendererIndex < renderers.Length; rendererIndex++)
            {
                bounds.Encapsulate(renderers[rendererIndex].bounds);
            }
            BarrelInstance overlapping = barrels.FirstOrDefault(existing =>
                Vector3.Distance(existing.Bounds.center, bounds.center) < 0.001f &&
                Vector3.Distance(existing.Bounds.size, bounds.size) < 0.001f);
            if (overlapping != null)
            {
                Debug.LogWarning(
                    $"Treating exact-overlap scene objects `{overlapping.Root.name}` and " +
                    $"`{candidates[index].name}` as one barrel training instance.");
                continue;
            }
            barrels.Add(new BarrelInstance
            {
                Id = barrels.Count,
                Root = candidates[index],
                Renderers = renderers,
                Bounds = bounds,
            });
        }
        return barrels;
    }

    private static bool IsBarrelName(string objectName)
    {
        return objectName.StartsWith("barrell_plastic", StringComparison.OrdinalIgnoreCase) ||
            objectName.StartsWith("barrell_yellow", StringComparison.OrdinalIgnoreCase);
    }

    private static void PositionCamera(
        Camera camera,
        Bounds target,
        int acceptedView,
        int attempt,
        System.Random random)
    {
        float largestExtent = Mathf.Max(target.extents.x, target.extents.y, target.extents.z);
        float distanceScale = 2.8f + (float)random.NextDouble() * 3.2f;
        float distance = Mathf.Max(2.0f, largestExtent * distanceScale);
        float baseAngle = acceptedView * (360f / ViewsPerBarrel);
        float angle = baseAngle + attempt * 37f + Mathf.Lerp(-22f, 22f, (float)random.NextDouble());
        float radians = angle * Mathf.Deg2Rad;
        float heightOffset = Mathf.Lerp(
            -0.25f * target.size.y,
            0.75f * target.size.y,
            (float)random.NextDouble());
        Vector3 position = target.center + new Vector3(
            Mathf.Cos(radians) * distance,
            heightOffset,
            Mathf.Sin(radians) * distance);
        position.y = Mathf.Max(position.y, 0.35f);

        Vector3 aimPoint = target.center + new Vector3(
            Mathf.Lerp(-0.15f, 0.15f, (float)random.NextDouble()) * target.size.x,
            Mathf.Lerp(-0.1f, 0.15f, (float)random.NextDouble()) * target.size.y,
            Mathf.Lerp(-0.15f, 0.15f, (float)random.NextDouble()) * target.size.z);
        camera.transform.SetPositionAndRotation(
            position,
            Quaternion.LookRotation(aimPoint - position, Vector3.up));
    }

    private static bool HasClearLineOfSight(Camera camera, Bounds target)
    {
        if (!IsCameraPositionClear(camera))
        {
            return false;
        }

        Vector3 cameraToTarget = target.center - camera.transform.position;
        float targetRadius = Mathf.Max(0.1f, target.extents.magnitude * 0.45f);
        float obstructionDistance = Mathf.Max(0f, cameraToTarget.magnitude - targetRadius);
        return !Physics.Raycast(
            camera.transform.position,
            cameraToTarget.normalized,
            obstructionDistance,
            ~0,
            QueryTriggerInteraction.Ignore);
    }

    private static bool IsCameraPositionClear(Camera camera)
    {
        return !Physics.CheckSphere(
            camera.transform.position,
            Mathf.Max(0.1f, camera.nearClipPlane),
            ~0,
            QueryTriggerInteraction.Ignore);
    }

    private static void RotateCameraAwayFromTarget(
        Camera camera,
        Bounds target,
        System.Random random)
    {
        Vector3 towardTarget = target.center - camera.transform.position;
        float targetYaw = Mathf.Atan2(towardTarget.x, towardTarget.z) * Mathf.Rad2Deg;
        float yawOffset = Mathf.Lerp(75f, 285f, (float)random.NextDouble());
        float pitch = Mathf.Lerp(-15f, 12f, (float)random.NextDouble());
        camera.transform.rotation = Quaternion.Euler(pitch, targetYaw + yawOffset, 0f);
    }

    private static RenderTexture CreateRenderTexture(string name, RenderTextureReadWrite readWrite)
    {
        var renderTexture = new RenderTexture(
            ImageWidth,
            ImageHeight,
            24,
            RenderTextureFormat.ARGB32,
            readWrite)
        {
            name = name,
            antiAliasing = 1,
            useMipMap = false,
            autoGenerateMips = false,
        };
        renderTexture.Create();
        return renderTexture;
    }

    private static Material CreateMaskMaterial(Shader shader, Color color, string name)
    {
        var material = new Material(shader) { name = name };
        material.SetColor("_Color", color);
        return material;
    }

    private static void RenderRgb(
        Camera camera,
        RenderTexture target,
        Texture2D readback,
        string outputPath)
    {
        RenderTexture previous = RenderTexture.active;
        camera.targetTexture = target;
        camera.Render();
        RenderTexture.active = target;
        readback.ReadPixels(new Rect(0f, 0f, ImageWidth, ImageHeight), 0, 0, false);
        readback.Apply(false, false);
        File.WriteAllBytes(outputPath, readback.EncodeToPNG());
        RenderTexture.active = previous;
    }

    private static int RenderMask(
        Camera camera,
        IEnumerable<Renderer> sceneRenderers,
        IEnumerable<Renderer> targetRenderers,
        Material blackMaterial,
        Material whiteMaterial,
        RenderTexture target,
        Texture2D readback)
    {
        RenderTexture previous = RenderTexture.active;
        using (var commands = new CommandBuffer { name = "Barrel instance mask" })
        {
            commands.SetRenderTarget(target);
            commands.ClearRenderTarget(true, true, Color.black);
            commands.SetViewProjectionMatrices(
                camera.worldToCameraMatrix,
                camera.projectionMatrix);
            DrawRenderers(commands, sceneRenderers, blackMaterial);
            DrawRenderers(commands, targetRenderers, whiteMaterial);
            Graphics.ExecuteCommandBuffer(commands);
        }
        RenderTexture.active = target;
        readback.ReadPixels(new Rect(0f, 0f, ImageWidth, ImageHeight), 0, 0, false);
        readback.Apply(false, false);
        Color32[] pixels = readback.GetPixels32();
        int whitePixels = pixels.Count(pixel => pixel.r > 200 && pixel.g > 200 && pixel.b > 200);
        RenderTexture.active = previous;
        return whitePixels;
    }

    private static void DrawRenderers(
        CommandBuffer commands,
        IEnumerable<Renderer> renderers,
        Material material)
    {
        foreach (Renderer renderer in renderers)
        {
            if (renderer == null || !renderer.enabled || !renderer.gameObject.activeInHierarchy)
            {
                continue;
            }
            int submeshCount = Mathf.Max(1, renderer.sharedMaterials.Length);
            for (int submeshIndex = 0; submeshIndex < submeshCount; submeshIndex++)
            {
                commands.DrawRenderer(renderer, material, submeshIndex, 0);
            }
        }
    }
}
