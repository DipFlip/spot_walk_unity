using UnityEngine;
#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem;
#endif

public sealed class SpotKinematicDrive : MonoBehaviour
{
    [Header("Movement")]
    [SerializeField] private float moveSpeed = 1.2f;
    [SerializeField] private float reverseSpeedMultiplier = 0.6f;
    [SerializeField] private float turnSpeedDegrees = 100f;
    [SerializeField] private bool moveInLocalForward = true;

    [Header("Visual Motion")]
    [SerializeField] private Transform visualRoot;
    [SerializeField] private float walkBobHeight = 0.025f;
    [SerializeField] private float walkBobFrequency = 4f;
    [SerializeField] private float turnLeanDegrees = 2f;

    [Header("Imported URDF Cleanup")]
    [SerializeField] private bool freezeImportedPhysics = true;
    [SerializeField] private bool disableImportedArticulationBodies = true;
    [SerializeField] private bool disableUrdfDemoControllers = true;

    private Vector3 visualBaseLocalPosition;
    private Quaternion visualBaseLocalRotation;
    private float gaitTime;

    private void Awake()
    {
        if (visualRoot != null)
        {
            visualBaseLocalPosition = visualRoot.localPosition;
            visualBaseLocalRotation = visualRoot.localRotation;
        }

        if (freezeImportedPhysics)
        {
            FreezePhysicsBodies();
        }

        if (disableUrdfDemoControllers)
        {
            DisableUrdfDemoControllers();
        }
    }

    private void Update()
    {
        Vector2 input = ReadMoveInput();
        float forward = Mathf.Clamp(input.y, -1f, 1f);
        float turn = Mathf.Clamp(input.x, -1f, 1f);
        float strafe = Mathf.Clamp(ReadStrafeInput(), -1f, 1f);

        float dt = Time.deltaTime;
        transform.Rotate(0f, turn * turnSpeedDegrees * dt, 0f, Space.World);

        float speed = forward >= 0f ? moveSpeed : moveSpeed * reverseSpeedMultiplier;
        Vector3 direction = moveInLocalForward ? transform.forward : Vector3.forward;
        transform.position += direction * (forward * speed * dt);
        transform.position += transform.right * (strafe * moveSpeed * dt);

        AnimateVisualRoot(forward, turn, dt);
    }

    private float ReadStrafeInput()
    {
#if ENABLE_INPUT_SYSTEM
        Keyboard keyboard = Keyboard.current;
        if (keyboard == null)
        {
            return 0f;
        }

        float strafe = 0f;
        if (keyboard.qKey.isPressed)
        {
            strafe -= 1f;
        }

        if (keyboard.eKey.isPressed)
        {
            strafe += 1f;
        }

        return strafe;
#else
        float strafe = 0f;
        if (Input.GetKey(KeyCode.Q))
        {
            strafe -= 1f;
        }

        if (Input.GetKey(KeyCode.E))
        {
            strafe += 1f;
        }

        return strafe;
#endif
    }

    private Vector2 ReadMoveInput()
    {
#if ENABLE_INPUT_SYSTEM
        Keyboard keyboard = Keyboard.current;
        if (keyboard == null)
        {
            return Vector2.zero;
        }

        float forward = 0f;
        float turn = 0f;

        if (keyboard.wKey.isPressed || keyboard.upArrowKey.isPressed)
        {
            forward += 1f;
        }

        if (keyboard.sKey.isPressed || keyboard.downArrowKey.isPressed)
        {
            forward -= 1f;
        }

        if (keyboard.aKey.isPressed || keyboard.leftArrowKey.isPressed)
        {
            turn -= 1f;
        }

        if (keyboard.dKey.isPressed || keyboard.rightArrowKey.isPressed)
        {
            turn += 1f;
        }

        return new Vector2(turn, forward);
#else
        float forward = 0f;
        float turn = 0f;

        if (Input.GetKey(KeyCode.W) || Input.GetKey(KeyCode.UpArrow))
        {
            forward += 1f;
        }

        if (Input.GetKey(KeyCode.S) || Input.GetKey(KeyCode.DownArrow))
        {
            forward -= 1f;
        }

        if (Input.GetKey(KeyCode.A) || Input.GetKey(KeyCode.LeftArrow))
        {
            turn -= 1f;
        }

        if (Input.GetKey(KeyCode.D) || Input.GetKey(KeyCode.RightArrow))
        {
            turn += 1f;
        }

        return new Vector2(turn, forward);
#endif
    }

    private void AnimateVisualRoot(float forward, float turn, float dt)
    {
        if (visualRoot == null || visualRoot == transform)
        {
            return;
        }

        float motion = Mathf.Max(Mathf.Abs(forward), Mathf.Abs(turn) * 0.5f);
        if (motion > 0.01f)
        {
            gaitTime += dt * walkBobFrequency * Mathf.Lerp(0.5f, 1f, motion);
        }

        float bob = motion > 0.01f ? Mathf.Sin(gaitTime * Mathf.PI * 2f) * walkBobHeight : 0f;
        float lean = -turn * turnLeanDegrees;

        visualRoot.localPosition = visualBaseLocalPosition + new Vector3(0f, bob, 0f);
        visualRoot.localRotation = visualBaseLocalRotation * Quaternion.Euler(0f, 0f, lean);
    }

    private void FreezePhysicsBodies()
    {
        foreach (Rigidbody body in GetComponentsInChildren<Rigidbody>(true))
        {
            body.useGravity = false;
            body.isKinematic = true;
            body.linearVelocity = Vector3.zero;
            body.angularVelocity = Vector3.zero;
        }

        foreach (ArticulationBody body in GetComponentsInChildren<ArticulationBody>(true))
        {
            body.useGravity = false;
            body.linearVelocity = Vector3.zero;
            body.angularVelocity = Vector3.zero;

            if (IsRootArticulationBody(body))
            {
                body.immovable = true;
            }

            if (disableImportedArticulationBodies)
            {
                body.enabled = false;
            }
        }
    }

    private static bool IsRootArticulationBody(ArticulationBody body)
    {
        Transform parent = body.transform.parent;
        while (parent != null)
        {
            if (parent.GetComponent<ArticulationBody>() != null)
            {
                return false;
            }

            parent = parent.parent;
        }

        return true;
    }

    private void DisableUrdfDemoControllers()
    {
        foreach (MonoBehaviour behaviour in GetComponentsInChildren<MonoBehaviour>(true))
        {
            if (behaviour == null || behaviour == this)
            {
                continue;
            }

            string typeName = behaviour.GetType().FullName;
            if (typeName == "Unity.Robotics.UrdfImporter.Control.Controller" ||
                typeName == "Unity.Robotics.UrdfImporter.Control.FKRobot" ||
                typeName == "Unity.Robotics.UrdfImporter.Control.IKRobot" ||
                typeName == "Unity.Robotics.UrdfImporter.Control.JointControl")
            {
                behaviour.enabled = false;
            }
        }
    }
}
