using System;
using System.Collections.Generic;
using UnityEngine;

public enum RadiationIsotope
{
    Cs137,
    Co60,
    Na22,
    Ba133,
    Am241,
    Co57,
    Mn54,
    K40
}

[DisallowMultipleComponent]
public sealed class RadiationSource : MonoBehaviour
{
    private static readonly HashSet<RadiationSource> ActiveSources = new HashSet<RadiationSource>();

    [SerializeField] private string sourceId;
    [SerializeField] private RadiationIsotope isotope = RadiationIsotope.Cs137;
    [SerializeField, Min(0f)] private float activityMicroCuries = 1000f;

    public string SourceId => sourceId;
    public RadiationIsotope Isotope => isotope;
    public double ActivityBecquerels => activityMicroCuries * 3.7e4;

    public static void CopyActiveSourcesTo(List<RadiationSource> destination)
    {
        destination.Clear();
        foreach (RadiationSource source in ActiveSources)
        {
            if (source != null && source.isActiveAndEnabled)
            {
                destination.Add(source);
            }
        }
    }

    private void Reset()
    {
        EnsureSourceId();
    }

    private void OnValidate()
    {
        activityMicroCuries = Mathf.Max(0f, activityMicroCuries);
        EnsureSourceId();
    }

    private void OnEnable()
    {
        EnsureSourceId();
        ActiveSources.Add(this);
    }

    private void OnDisable()
    {
        ActiveSources.Remove(this);
    }

    private void EnsureSourceId()
    {
        if (string.IsNullOrWhiteSpace(sourceId))
        {
            sourceId = Guid.NewGuid().ToString("N");
        }
    }
}
