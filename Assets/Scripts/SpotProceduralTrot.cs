using System;
using UnityEngine;

public sealed class SpotProceduralTrot : MonoBehaviour
{
    [Serializable]
    private sealed class Leg
    {
        public string hipName;
        public string upperLegName;
        public string lowerLegName;
        public int diagonalGroup;
        public float sideSign = 1f;

        [NonSerialized] public Transform hip;
        [NonSerialized] public Transform upperLeg;
        [NonSerialized] public Transform lowerLeg;
        [NonSerialized] public Quaternion hipBase;
        [NonSerialized] public Quaternion upperBase;
        [NonSerialized] public Quaternion lowerBase;
        [NonSerialized] public Vector3 homeLocal;
        [NonSerialized] public Vector3 plantedWorld;
        [NonSerialized] public Vector3 swingStartWorld;
        [NonSerialized] public Vector3 swingTargetWorld;
        [NonSerialized] public bool wasSwinging;
    }

    [Header("Neutral Pose")]
    [SerializeField] private float neutralUpperLegDegrees = -12f;
    [SerializeField] private float neutralKneeDegrees = 24f;
    [SerializeField] private float hipSwayDegrees = 3f;

    [Header("Stepping")]
    [SerializeField] private float cycleFrequency = 1.7f;
    [SerializeField] private float strideLength = 0.34f;
    [SerializeField] private float stepHeight = 0.09f;
    [SerializeField] private float minimumMovingStrideScale = 0.35f;
    [SerializeField] private float backwardStrideMultiplier = 0.55f;
    [SerializeField] private float strafeStrideMultiplier = 0.5f;
    [SerializeField] private float backwardFootBias = 0.16f;
    [SerializeField] private float strafeFootBias = 0.12f;
    [SerializeField] private float stepWidth = 0.02f;
    [SerializeField] private float minStepDistance = 0.08f;
    [SerializeField] private float turnStrideLength = 0.22f;
    [SerializeField] private float turnReplantDistance = 0.06f;
    [SerializeField] private float footPlantSharpness = 24f;
    [SerializeField] private Vector3 footLocalOffset = new Vector3(0f, -0.36f, 0f);

    [Header("IK")]
    [SerializeField] private int ikIterations = 8;
    [SerializeField] private float ikWeight = 0.9f;
    [SerializeField] private float maxJointDegreesPerIteration = 18f;
    [SerializeField] private Vector3 hipAxis = Vector3.forward;
    [SerializeField] private Vector3 upperLegAxis = Vector3.left;
    [SerializeField] private Vector3 lowerLegAxis = Vector3.left;

    [Header("Body Motion")]
    [SerializeField] private string bodyLinkName = "body";
    [SerializeField] private float bodyBobHeight = 0.025f;
    [SerializeField] private float bodyPitchDegrees = 1.5f;
    [SerializeField] private float bodyRollDegrees = 1.2f;
    [SerializeField] private float returnToStandSpeed = 8f;

    [Header("Movement Detection")]
    [SerializeField] private float movementThreshold = 0.001f;
    [SerializeField] private float turnThresholdDegrees = 0.05f;

    [Header("Debug")]
    [SerializeField] private bool drawFootTargets;

    [SerializeField]
    private Leg[] legs =
    {
        new Leg { hipName = "fl_hip", upperLegName = "fl_uleg", lowerLegName = "fl_lleg", diagonalGroup = 0, sideSign = 1f },
        new Leg { hipName = "hr_hip", upperLegName = "hr_uleg", lowerLegName = "hr_lleg", diagonalGroup = 0, sideSign = -1f },
        new Leg { hipName = "fr_hip", upperLegName = "fr_uleg", lowerLegName = "fr_lleg", diagonalGroup = 1, sideSign = -1f },
        new Leg { hipName = "hl_hip", upperLegName = "hl_uleg", lowerLegName = "hl_lleg", diagonalGroup = 1, sideSign = 1f },
    };

    private Vector3 previousPosition;
    private Quaternion previousRotation;
    private Transform bodyLink;
    private Vector3 bodyBaseLocalPosition;
    private Quaternion bodyBaseLocalRotation;
    private float gaitTime;
    private float motionBlend;
    private float stepBlend;
    private Vector3 velocityWorld;
    private float turnVelocity;
    private float signedTurnVelocity;

    private void Awake()
    {
        MigrateOldSerializedValues();
        BindLegs();
        PoseNeutral();
        CacheHomeFeet();
        CacheBody();

        previousPosition = transform.position;
        previousRotation = transform.rotation;
    }

    private void LateUpdate()
    {
        float dt = Mathf.Max(Time.deltaTime, 0.0001f);
        Vector3 positionDelta = transform.position - previousPosition;
        float turnDelta = Quaternion.Angle(transform.rotation, previousRotation);
        velocityWorld = positionDelta / dt;
        turnVelocity = turnDelta / dt;
        signedTurnVelocity = GetSignedYawDelta(previousRotation, transform.rotation) / dt;

        bool isMoving = positionDelta.sqrMagnitude > movementThreshold * movementThreshold || turnDelta > turnThresholdDegrees;
        motionBlend = Mathf.MoveTowards(motionBlend, isMoving ? 1f : 0f, returnToStandSpeed * dt);
        stepBlend = isMoving ? 1f : Mathf.MoveTowards(stepBlend, 0f, returnToStandSpeed * dt);

        if (stepBlend > 0.001f)
        {
            float planarSpeed = Vector3.ProjectOnPlane(velocityWorld, Vector3.up).magnitude;
            float cadenceScale = Mathf.Lerp(0.75f, 1.2f, Mathf.Clamp01(planarSpeed / 1.2f));
            gaitTime += dt * cycleFrequency * cadenceScale;
        }

        PoseNeutral();
        AnimateBody();
        UpdateFootTargets(isMoving, dt);
        SolveLegs();

        previousPosition = transform.position;
        previousRotation = transform.rotation;
    }

    private void BindLegs()
    {
        foreach (Leg leg in legs)
        {
            leg.hip = FindDeepChild(leg.hipName);
            leg.upperLeg = FindDeepChild(leg.upperLegName);
            leg.lowerLeg = FindDeepChild(leg.lowerLegName);

            if (leg.hip != null)
            {
                leg.hipBase = leg.hip.localRotation;
            }

            if (leg.upperLeg != null)
            {
                leg.upperBase = leg.upperLeg.localRotation;
            }

            if (leg.lowerLeg != null)
            {
                leg.lowerBase = leg.lowerLeg.localRotation;
            }
        }
    }

    private void CacheHomeFeet()
    {
        foreach (Leg leg in legs)
        {
            if (leg.lowerLeg == null)
            {
                continue;
            }

            Vector3 footWorld = GetFootWorld(leg);
            leg.homeLocal = transform.InverseTransformPoint(footWorld);
            leg.plantedWorld = footWorld;
            leg.swingStartWorld = footWorld;
            leg.swingTargetWorld = footWorld;
        }
    }

    private void CacheBody()
    {
        bodyLink = FindDeepChild(bodyLinkName, false);
        if (bodyLink != null && bodyLink != transform)
        {
            bodyBaseLocalPosition = bodyLink.localPosition;
            bodyBaseLocalRotation = bodyLink.localRotation;
        }
    }

    private void PoseNeutral()
    {
        foreach (Leg leg in legs)
        {
            if (leg.hip != null)
            {
                leg.hip.localRotation = leg.hipBase;
            }

            if (leg.upperLeg != null)
            {
                leg.upperLeg.localRotation = leg.upperBase * Quaternion.AngleAxis(neutralUpperLegDegrees, upperLegAxis.normalized);
            }

            if (leg.lowerLeg != null)
            {
                leg.lowerLeg.localRotation = leg.lowerBase * Quaternion.AngleAxis(neutralKneeDegrees, lowerLegAxis.normalized);
            }
        }
    }

    private void UpdateFootTargets(bool isMoving, float dt)
    {
        float groupPhase = Mathf.Repeat(gaitTime, 1f);
        Vector3 localVelocity = transform.InverseTransformDirection(Vector3.ProjectOnPlane(velocityWorld, Vector3.up));
        float forwardAmount = Mathf.Clamp(localVelocity.z / 1.2f, -1f, 1f);
        float lateralAmount = Mathf.Clamp(localVelocity.x / 1.2f, -1f, 1f);
        float turnAmount = Mathf.Clamp(signedTurnVelocity / 120f, -1f, 1f);
        float speedScale = Mathf.Clamp01(Mathf.Max(new Vector2(forwardAmount, lateralAmount).magnitude, Mathf.Abs(turnAmount) * 0.7f));
        float strideScale = Mathf.Lerp(minimumMovingStrideScale, 1f, speedScale);
        if (forwardAmount < -0.01f)
        {
            strideScale *= backwardStrideMultiplier;
        }

        Vector3 forwardStep = transform.forward * forwardAmount;
        Vector3 strafeStep = transform.right * lateralAmount * strafeStrideMultiplier;
        Vector3 travelStep = (forwardStep + strafeStep) * strideLength * strideScale * stepBlend;
        Vector3 backwardBias = forwardAmount < -0.01f
            ? -transform.forward * (Mathf.Abs(forwardAmount) * backwardFootBias * stepBlend)
            : Vector3.zero;
        Vector3 strafeBias = Mathf.Abs(lateralAmount) > 0.01f
            ? transform.right * (lateralAmount * strafeFootBias * stepBlend)
            : Vector3.zero;
        float stepLift = stepHeight * Mathf.Lerp(0.55f, 1f, speedScale) * stepBlend;

        foreach (Leg leg in legs)
        {
            if (leg.lowerLeg == null)
            {
                continue;
            }

            bool shouldSwing = isMoving && stepBlend > 0.1f && IsGroupSwinging(leg.diagonalGroup, groupPhase);
            Vector3 homeWorld = transform.TransformPoint(leg.homeLocal);
            Vector3 turnStep = GetTurnStep(leg, turnAmount);
            Vector3 desiredPlant = homeWorld + travelStep * 0.5f + backwardBias + strafeBias + turnStep + transform.right * (leg.sideSign * stepWidth * stepBlend);

            if (shouldSwing && !leg.wasSwinging)
            {
                leg.swingStartWorld = leg.plantedWorld;
                float catchUp = Mathf.Clamp01(Vector3.Distance(leg.plantedWorld, homeWorld) / Mathf.Max(strideLength, 0.001f));
                leg.swingTargetWorld = Vector3.Lerp(desiredPlant, homeWorld + travelStep + backwardBias + strafeBias + turnStep, catchUp);
            }

            if (shouldSwing)
            {
                float swingT = GetSwingProgress(leg.diagonalGroup, groupPhase);
                Vector3 footWorld = Vector3.Lerp(leg.swingStartWorld, leg.swingTargetWorld, SmoothStep(swingT));
                footWorld += Vector3.up * (Mathf.Sin(swingT * Mathf.PI) * stepLift);
                leg.plantedWorld = footWorld;
            }
            else
            {
                float drift = Vector3.Distance(leg.plantedWorld, homeWorld);
                float allowedDrift = Mathf.Lerp((strideLength * strideScale) + minStepDistance, turnReplantDistance, Mathf.Abs(turnAmount));
                if (!isMoving || stepBlend < 0.1f || drift > allowedDrift)
                {
                    leg.plantedWorld = Vector3.Lerp(leg.plantedWorld, homeWorld, 1f - Mathf.Exp(-footPlantSharpness * dt));
                }
            }

            leg.wasSwinging = shouldSwing;
        }
    }

    private Vector3 GetTurnStep(Leg leg, float turnAmount)
    {
        if (Mathf.Abs(turnAmount) < 0.001f)
        {
            return Vector3.zero;
        }

        Vector3 home = leg.homeLocal;
        Vector3 radial = new Vector3(home.x, 0f, home.z);
        if (radial.sqrMagnitude < 0.0001f)
        {
            radial = new Vector3(leg.sideSign, 0f, 0f);
        }

        Vector3 tangentLocal = Vector3.Cross(Vector3.up, radial).normalized * Mathf.Sign(turnAmount);
        return transform.TransformDirection(tangentLocal) * (Mathf.Abs(turnAmount) * turnStrideLength * stepBlend);
    }

    private void SolveLegs()
    {
        foreach (Leg leg in legs)
        {
            if (leg.hip == null || leg.upperLeg == null || leg.lowerLeg == null)
            {
                continue;
            }

            float phase = (gaitTime + leg.diagonalGroup * 0.5f) * Mathf.PI * 2f;
            leg.hip.localRotation *= Quaternion.AngleAxis(
                Mathf.Sin(phase) * hipSwayDegrees * leg.sideSign * motionBlend,
                hipAxis.normalized);

            for (int i = 0; i < ikIterations; i++)
            {
                ApplyHingeIK(leg.lowerLeg, lowerLegAxis, leg, leg.plantedWorld);
                ApplyHingeIK(leg.upperLeg, upperLegAxis, leg, leg.plantedWorld);
                ApplyHingeIK(leg.hip, hipAxis, leg, leg.plantedWorld);
            }
        }
    }

    private void ApplyHingeIK(Transform joint, Vector3 localAxis, Leg leg, Vector3 targetWorld)
    {
        Vector3 axisWorld = joint.TransformDirection(localAxis.normalized);
        Vector3 jointToFoot = Vector3.ProjectOnPlane(GetFootWorld(leg) - joint.position, axisWorld);
        Vector3 jointToTarget = Vector3.ProjectOnPlane(targetWorld - joint.position, axisWorld);

        if (jointToFoot.sqrMagnitude < 0.000001f || jointToTarget.sqrMagnitude < 0.000001f)
        {
            return;
        }

        float angle = Vector3.SignedAngle(jointToFoot, jointToTarget, axisWorld);
        angle = Mathf.Clamp(angle * ikWeight, -maxJointDegreesPerIteration, maxJointDegreesPerIteration);
        joint.localRotation *= Quaternion.AngleAxis(angle, localAxis.normalized);
    }

    private Vector3 GetFootWorld(Leg leg)
    {
        return leg.lowerLeg.TransformPoint(footLocalOffset);
    }

    private static float GetSignedYawDelta(Quaternion from, Quaternion to)
    {
        Vector3 fromForward = Vector3.ProjectOnPlane(from * Vector3.forward, Vector3.up);
        Vector3 toForward = Vector3.ProjectOnPlane(to * Vector3.forward, Vector3.up);
        return Vector3.SignedAngle(fromForward, toForward, Vector3.up);
    }

    private static bool IsGroupSwinging(int group, float phase)
    {
        return group == 0 ? phase < 0.5f : phase >= 0.5f;
    }

    private static float GetSwingProgress(int group, float phase)
    {
        return group == 0 ? Mathf.InverseLerp(0f, 0.5f, phase) : Mathf.InverseLerp(0.5f, 1f, phase);
    }

    private static float SmoothStep(float value)
    {
        value = Mathf.Clamp01(value);
        return value * value * (3f - 2f * value);
    }

    private void AnimateBody()
    {
        if (bodyLink == null || bodyLink == transform)
        {
            return;
        }

        float phase = gaitTime * Mathf.PI * 2f;
        float bob = Mathf.Abs(Mathf.Sin(phase)) * bodyBobHeight * motionBlend;
        float pitch = Mathf.Sin(phase) * bodyPitchDegrees * motionBlend;
        float roll = Mathf.Cos(phase) * bodyRollDegrees * motionBlend;

        bodyLink.localPosition = bodyBaseLocalPosition + new Vector3(0f, bob, 0f);
        bodyLink.localRotation = bodyBaseLocalRotation * Quaternion.Euler(pitch, 0f, roll);
    }

    private void MigrateOldSerializedValues()
    {
        if (Mathf.Approximately(strideLength, 0f))
        {
            strideLength = 0.34f;
        }

        if (Mathf.Approximately(stepHeight, 0f))
        {
            stepHeight = 0.09f;
        }

        if (Mathf.Approximately(minimumMovingStrideScale, 0f))
        {
            minimumMovingStrideScale = 0.35f;
        }

        if (Mathf.Approximately(backwardStrideMultiplier, 0f))
        {
            backwardStrideMultiplier = 0.55f;
        }

        if (Mathf.Approximately(strafeStrideMultiplier, 0f))
        {
            strafeStrideMultiplier = 0.5f;
        }

        if (Mathf.Approximately(backwardFootBias, 0f))
        {
            backwardFootBias = 0.16f;
        }

        if (Mathf.Approximately(strafeFootBias, 0f))
        {
            strafeFootBias = 0.12f;
        }

        if (Mathf.Approximately(footPlantSharpness, 0f))
        {
            footPlantSharpness = 24f;
        }

        if (Mathf.Approximately(turnStrideLength, 0f))
        {
            turnStrideLength = 0.22f;
        }

        if (Mathf.Approximately(turnReplantDistance, 0f))
        {
            turnReplantDistance = 0.06f;
        }

        if (footLocalOffset == Vector3.zero)
        {
            footLocalOffset = new Vector3(0f, -0.36f, 0f);
        }

        if (ikIterations <= 0)
        {
            ikIterations = 8;
        }

        if (Mathf.Approximately(ikWeight, 0f))
        {
            ikWeight = 0.9f;
        }

        if (Mathf.Approximately(maxJointDegreesPerIteration, 0f))
        {
            maxJointDegreesPerIteration = 18f;
        }

        bool groupsNeedMigration = true;
        foreach (Leg leg in legs)
        {
            if (leg.diagonalGroup != 0)
            {
                groupsNeedMigration = false;
                break;
            }
        }

        if (groupsNeedMigration)
        {
            foreach (Leg leg in legs)
            {
                if (leg.hipName.StartsWith("fr_", StringComparison.Ordinal) ||
                    leg.hipName.StartsWith("hl_", StringComparison.Ordinal))
                {
                    leg.diagonalGroup = 1;
                }
                else
                {
                    leg.diagonalGroup = 0;
                }
            }
        }

        if (hipAxis == Vector3.right)
        {
            hipAxis = Vector3.forward;
        }

        if (upperLegAxis == Vector3.up)
        {
            upperLegAxis = Vector3.left;
        }

        if (lowerLegAxis == Vector3.up)
        {
            lowerLegAxis = Vector3.left;
        }
    }

    private Transform FindDeepChild(string childName, bool warnIfMissing = true)
    {
        Transform[] children = GetComponentsInChildren<Transform>(true);
        foreach (Transform child in children)
        {
            if (child.name == childName)
            {
                return child;
            }
        }

        if (warnIfMissing)
        {
            Debug.LogWarning($"SpotProceduralTrot could not find child transform '{childName}'.", this);
        }

        return null;
    }

    private void OnDrawGizmosSelected()
    {
        if (!drawFootTargets || legs == null)
        {
            return;
        }

        Gizmos.color = Color.cyan;
        foreach (Leg leg in legs)
        {
            if (leg == null)
            {
                continue;
            }

            Gizmos.DrawSphere(leg.plantedWorld, 0.025f);
        }
    }
}
