using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SceneManagement;

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

    public static void CopyActiveSourcesTo(List<RadiationSource> destination, Scene scene)
    {
        destination.Clear();
        foreach (RadiationSource source in ActiveSources)
        {
            if (source != null &&
                source.isActiveAndEnabled &&
                source.gameObject.scene == scene)
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
        EnsureUniqueActiveSourceId();
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

    private void EnsureUniqueActiveSourceId()
    {
        if (!Application.isPlaying)
        {
            return;
        }

        foreach (RadiationSource source in ActiveSources)
        {
            if (source == null ||
                source == this ||
                source.gameObject.scene != gameObject.scene ||
                !string.Equals(source.sourceId, sourceId, StringComparison.Ordinal))
            {
                continue;
            }

            string duplicateId = sourceId;
            sourceId = Guid.NewGuid().ToString("N");
            Debug.LogWarning(
                $"RadiationSource id '{duplicateId}' was duplicated; " +
                $"assigned runtime id '{sourceId}' to '{name}'.",
                this);
            return;
        }
    }
}
