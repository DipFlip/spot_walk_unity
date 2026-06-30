using UnityEngine;
using UnityEngine.AI;
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

    [Header("NavMesh Restriction")]
    [SerializeField] private bool restrictMovementToNavMesh = true;
    [SerializeField] private int navMeshAreaMask = NavMesh.AllAreas;
    [SerializeField] private float navMeshSampleDistance = 0.35f;
    [SerializeField] private bool slideAlongNavMeshEdges = true;
    [SerializeField] private int navMeshSlideIterations = 2;
    [SerializeField] private float navMeshSlideMinDistance = 0.001f;
    [SerializeField] private bool pushAwayFromNavMeshEdgesOnYaw = true;
    [SerializeField] private int navMeshYawPushIterations = 4;
    [SerializeField] private float navMeshYawPushMaxDistance = 0.04f;
    [SerializeField] private float navMeshYawPushSmoothing = 0.45f;
    [SerializeField] private bool useNavMeshHeight;
    [SerializeField] private Transform frontNavProbe;
    [SerializeField] private Transform rearNavProbe;
    [SerializeField] private Vector3 frontNavProbeLocalOffset = new Vector3(0f, 0f, 0.45f);
    [SerializeField] private Vector3 rearNavProbeLocalOffset = new Vector3(0f, 0f, -0.45f);

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
        RotateWithNavMeshRestriction(turn * turnSpeedDegrees * dt);

        float speed = forward >= 0f ? moveSpeed : moveSpeed * reverseSpeedMultiplier;
        Vector3 direction = moveInLocalForward ? transform.forward : Vector3.forward;
        Vector3 movement = direction * (forward * speed * dt);
        movement += transform.right * (strafe * moveSpeed * dt);
        MoveWithNavMeshRestriction(movement);

        AnimateVisualRoot(forward, turn, dt);
    }

    private void RotateWithNavMeshRestriction(float yawDegrees)
    {
        if (Mathf.Abs(yawDegrees) <= Mathf.Epsilon)
        {
            return;
        }

        Quaternion currentRotation = transform.rotation;
        Quaternion targetRotation = Quaternion.Euler(0f, yawDegrees, 0f) * currentRotation;
        if (!restrictMovementToNavMesh ||
            TryMoveFootprintOnNavMesh(transform.position, currentRotation, transform.position, targetRotation, out _, out _))
        {
            transform.rotation = targetRotation;
            return;
        }

        if (pushAwayFromNavMeshEdgesOnYaw &&
            TryPushYawFootprintOnNavMesh(transform.position, currentRotation, targetRotation, out Vector3 pushedPosition))
        {
            Vector3 smoothedPosition = SmoothYawPushPosition(transform.position, pushedPosition);
            if (TryMoveFootprintOnNavMesh(
                transform.position,
                currentRotation,
                smoothedPosition,
                targetRotation,
                out Vector3 constrainedPosition,
                out _))
            {
                transform.SetPositionAndRotation(constrainedPosition, targetRotation);
            }
            else
            {
                transform.SetPositionAndRotation(pushedPosition, targetRotation);
            }
        }
    }

    private bool TryPushYawFootprintOnNavMesh(
        Vector3 currentPosition,
        Quaternion currentRotation,
        Quaternion targetRotation,
        out Vector3 pushedPosition)
    {
        pushedPosition = currentPosition;
        int iterations = Mathf.Max(1, navMeshYawPushIterations);
        for (int i = 0; i < iterations; i++)
        {
            if (TryMoveFootprintOnNavMesh(
                currentPosition,
                currentRotation,
                pushedPosition,
                targetRotation,
                out Vector3 constrainedPosition,
                out Vector3 blockingNormal))
            {
                pushedPosition = constrainedPosition;
                return true;
            }

            if (!TryGetFootprintNavMeshPush(pushedPosition, targetRotation, out Vector3 push))
            {
                if (blockingNormal.sqrMagnitude <= Mathf.Epsilon)
                {
                    return false;
                }

                push = blockingNormal.normalized * Mathf.Max(navMeshSampleDistance * 0.5f, navMeshSlideMinDistance);
            }

            pushedPosition += push;
        }

        return TryMoveFootprintOnNavMesh(
            currentPosition,
            currentRotation,
            pushedPosition,
            targetRotation,
            out pushedPosition,
            out _);
    }

    private Vector3 SmoothYawPushPosition(Vector3 currentPosition, Vector3 pushedPosition)
    {
        Vector3 push = pushedPosition - currentPosition;
        float maxPushDistance = Mathf.Max(0.001f, navMeshYawPushMaxDistance);
        if (push.magnitude > maxPushDistance)
        {
            push = push.normalized * maxPushDistance;
        }

        return currentPosition + push * Mathf.Clamp01(navMeshYawPushSmoothing);
    }

    private void MoveWithNavMeshRestriction(Vector3 movement)
    {
        if (movement.sqrMagnitude <= Mathf.Epsilon)
        {
            return;
        }

        Vector3 currentPosition = transform.position;
        Vector3 targetPosition = currentPosition + movement;
        Quaternion currentRotation = transform.rotation;
        if (!restrictMovementToNavMesh)
        {
            transform.position = targetPosition;
            return;
        }

        if (!TryConstrainToNavMesh(currentPosition, currentRotation, targetPosition, movement, out Vector3 constrainedPosition))
        {
            return;
        }

        transform.position = constrainedPosition;
    }

    private bool TryConstrainToNavMesh(
        Vector3 currentPosition,
        Quaternion currentRotation,
        Vector3 targetPosition,
        Vector3 movement,
        out Vector3 constrainedPosition)
    {
        constrainedPosition = currentPosition;

        if (TryMoveFootprintOnNavMesh(
            currentPosition,
            currentRotation,
            targetPosition,
            currentRotation,
            out constrainedPosition,
            out Vector3 blockingNormal))
        {
            return true;
        }

        if (!slideAlongNavMeshEdges)
        {
            return false;
        }

        Vector3 remainingMovement = movement;
        int iterations = Mathf.Max(1, navMeshSlideIterations);
        for (int i = 0; i < iterations; i++)
        {
            if (blockingNormal.sqrMagnitude <= Mathf.Epsilon)
            {
                return false;
            }

            Vector3 slideMovement = Vector3.ProjectOnPlane(remainingMovement, blockingNormal);
            if (slideMovement.sqrMagnitude < navMeshSlideMinDistance * navMeshSlideMinDistance)
            {
                return false;
            }

            Vector3 slideTarget = currentPosition + slideMovement;
            if (TryMoveFootprintOnNavMesh(
                currentPosition,
                currentRotation,
                slideTarget,
                currentRotation,
                out constrainedPosition,
                out blockingNormal))
            {
                return true;
            }

            remainingMovement = slideMovement;
        }

        return false;
    }

    private bool TryMoveFootprintOnNavMesh(
        Vector3 currentPosition,
        Quaternion currentRotation,
        Vector3 targetPosition,
        Quaternion targetRotation,
        out Vector3 constrainedPosition,
        out Vector3 blockingNormal)
    {
        constrainedPosition = currentPosition;
        blockingNormal = Vector3.zero;

        Vector3 frontOffset = GetNavProbeLocalOffset(frontNavProbe, frontNavProbeLocalOffset);
        Vector3 rearOffset = GetNavProbeLocalOffset(rearNavProbe, rearNavProbeLocalOffset);

        if (!TryMoveProbeOnNavMesh(
            currentPosition,
            currentRotation,
            targetPosition,
            targetRotation,
            frontOffset,
            out Vector3 frontTarget,
            out blockingNormal))
        {
            return false;
        }

        if (!TryMoveProbeOnNavMesh(
            currentPosition,
            currentRotation,
            targetPosition,
            targetRotation,
            rearOffset,
            out Vector3 rearTarget,
            out blockingNormal))
        {
            return false;
        }

        float navMeshHeight = (frontTarget.y + rearTarget.y) * 0.5f;
        constrainedPosition = useNavMeshHeight
            ? new Vector3(targetPosition.x, navMeshHeight, targetPosition.z)
            : targetPosition;
        return true;
    }

    private bool TryMoveProbeOnNavMesh(
        Vector3 currentPosition,
        Quaternion currentRotation,
        Vector3 targetPosition,
        Quaternion targetRotation,
        Vector3 localOffset,
        out Vector3 targetHitPosition,
        out Vector3 blockingNormal)
    {
        Vector3 currentProbe = currentPosition + currentRotation * localOffset;
        Vector3 targetProbe = targetPosition + targetRotation * localOffset;
        targetHitPosition = targetProbe;
        blockingNormal = Vector3.zero;

        if (!NavMesh.SamplePosition(currentProbe, out NavMeshHit currentHit, navMeshSampleDistance, navMeshAreaMask))
        {
            return false;
        }

        if (!NavMesh.SamplePosition(targetProbe, out NavMeshHit targetHit, navMeshSampleDistance, navMeshAreaMask))
        {
            if (NavMesh.FindClosestEdge(currentHit.position, out NavMeshHit edgeHit, navMeshAreaMask))
            {
                blockingNormal = edgeHit.normal;
            }

            return false;
        }

        if (NavMesh.Raycast(currentHit.position, targetHit.position, out NavMeshHit raycastHit, navMeshAreaMask))
        {
            blockingNormal = raycastHit.normal;
            return false;
        }

        targetHitPosition = targetHit.position;
        return true;
    }

    private Vector3 GetNavProbeLocalOffset(Transform probe, Vector3 fallbackOffset)
    {
        return probe != null ? transform.InverseTransformPoint(probe.position) : fallbackOffset;
    }

    private bool TryGetFootprintNavMeshPush(Vector3 position, Quaternion rotation, out Vector3 push)
    {
        push = Vector3.zero;
        Vector3 frontOffset = GetNavProbeLocalOffset(frontNavProbe, frontNavProbeLocalOffset);
        Vector3 rearOffset = GetNavProbeLocalOffset(rearNavProbe, rearNavProbeLocalOffset);

        bool pushedFront = TryGetProbeNavMeshPush(position, rotation, frontOffset, out Vector3 frontPush);
        bool pushedRear = TryGetProbeNavMeshPush(position, rotation, rearOffset, out Vector3 rearPush);
        if (!pushedFront || !pushedRear)
        {
            return false;
        }

        push = frontPush.sqrMagnitude >= rearPush.sqrMagnitude ? frontPush : rearPush;
        push.y = 0f;
        return push.sqrMagnitude > Mathf.Epsilon;
    }

    private bool TryGetProbeNavMeshPush(
        Vector3 position,
        Quaternion rotation,
        Vector3 localOffset,
        out Vector3 push)
    {
        Vector3 probePosition = position + rotation * localOffset;
        if (!NavMesh.SamplePosition(probePosition, out NavMeshHit hit, navMeshSampleDistance, navMeshAreaMask))
        {
            push = Vector3.zero;
            return false;
        }

        push = hit.position - probePosition;
        push.y = 0f;
        return true;
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
