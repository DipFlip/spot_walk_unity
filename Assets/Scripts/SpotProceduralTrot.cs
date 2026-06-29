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
        public float phaseOffset;
        public float sideSign = 1f;

        [NonSerialized] public Transform hip;
        [NonSerialized] public Transform upperLeg;
        [NonSerialized] public Transform lowerLeg;
        [NonSerialized] public Quaternion hipBase;
        [NonSerialized] public Quaternion upperBase;
        [NonSerialized] public Quaternion lowerBase;
    }

    [Header("Gait")]
    [SerializeField] private float cycleFrequency = 2.1f;
    [SerializeField] private float neutralUpperLegDegrees = -12f;
    [SerializeField] private float neutralKneeDegrees = 24f;
    [SerializeField] private float upperLegSwingDegrees = 22f;
    [SerializeField] private float kneeSwingDegrees = 42f;
    [SerializeField] private float hipSwayDegrees = 4f;
    [SerializeField] private float returnToStandSpeed = 8f;

    [Header("Body Motion")]
    [SerializeField] private string bodyLinkName = "body";
    [SerializeField] private float bodyBobHeight = 0.025f;
    [SerializeField] private float bodyPitchDegrees = 1.5f;
    [SerializeField] private float bodyRollDegrees = 1.2f;

    [Header("Joint Axes")]
    [SerializeField] private Vector3 hipAxis = Vector3.forward;
    [SerializeField] private Vector3 upperLegAxis = Vector3.left;
    [SerializeField] private Vector3 lowerLegAxis = Vector3.left;

    [Header("Movement Detection")]
    [SerializeField] private float movementThreshold = 0.001f;
    [SerializeField] private float turnThresholdDegrees = 0.05f;

    [SerializeField]
    private Leg[] legs =
    {
        new Leg { hipName = "fl_hip", upperLegName = "fl_uleg", lowerLegName = "fl_lleg", phaseOffset = 0f, sideSign = 1f },
        new Leg { hipName = "hr_hip", upperLegName = "hr_uleg", lowerLegName = "hr_lleg", phaseOffset = 0f, sideSign = -1f },
        new Leg { hipName = "fr_hip", upperLegName = "fr_uleg", lowerLegName = "fr_lleg", phaseOffset = 0.5f, sideSign = -1f },
        new Leg { hipName = "hl_hip", upperLegName = "hl_uleg", lowerLegName = "hl_lleg", phaseOffset = 0.5f, sideSign = 1f },
    };

    private Vector3 previousPosition;
    private Quaternion previousRotation;
    private Transform bodyLink;
    private Vector3 bodyBaseLocalPosition;
    private Quaternion bodyBaseLocalRotation;
    private float gaitTime;
    private float motionBlend;

    private void Awake()
    {
        MigrateOldDefaultAxes();

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

        bodyLink = FindDeepChild(bodyLinkName, false);
        if (bodyLink != null && bodyLink != transform)
        {
            bodyBaseLocalPosition = bodyLink.localPosition;
            bodyBaseLocalRotation = bodyLink.localRotation;
        }

        previousPosition = transform.position;
        previousRotation = transform.rotation;
    }

    private void LateUpdate()
    {
        float distance = Vector3.Distance(transform.position, previousPosition);
        float turnDegrees = Quaternion.Angle(transform.rotation, previousRotation);
        bool isMoving = distance > movementThreshold || turnDegrees > turnThresholdDegrees;

        float targetBlend = isMoving ? 1f : 0f;
        motionBlend = Mathf.MoveTowards(motionBlend, targetBlend, returnToStandSpeed * Time.deltaTime);

        if (motionBlend > 0.001f)
        {
            gaitTime += Time.deltaTime * cycleFrequency;
        }

        foreach (Leg leg in legs)
        {
            AnimateLeg(leg);
        }

        AnimateBody();

        previousPosition = transform.position;
        previousRotation = transform.rotation;
    }

    private void AnimateLeg(Leg leg)
    {
        float phase = (gaitTime + leg.phaseOffset) * Mathf.PI * 2f;
        float stride = Mathf.Sin(phase);
        float lift = Mathf.Max(0f, stride);
        float plant = Mathf.Max(0f, -stride);
        float upperAngle = neutralUpperLegDegrees + upperLegSwingDegrees * stride * motionBlend;
        float kneeAngle = neutralKneeDegrees + (0.35f + 0.65f * lift - 0.2f * plant) * kneeSwingDegrees * motionBlend;

        Quaternion hipRotation = Quaternion.AngleAxis(hipSwayDegrees * leg.sideSign * stride * motionBlend, hipAxis.normalized);
        Quaternion upperRotation = Quaternion.AngleAxis(upperAngle, upperLegAxis.normalized);
        Quaternion lowerRotation = Quaternion.AngleAxis(kneeAngle, lowerLegAxis.normalized);

        if (leg.hip != null)
        {
            leg.hip.localRotation = leg.hipBase * hipRotation;
        }

        if (leg.upperLeg != null)
        {
            leg.upperLeg.localRotation = leg.upperBase * upperRotation;
        }

        if (leg.lowerLeg != null)
        {
            leg.lowerLeg.localRotation = leg.lowerBase * lowerRotation;
        }
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

    private void MigrateOldDefaultAxes()
    {
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
}
