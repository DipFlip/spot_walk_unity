using DistributionCenter;
using Unity.AI.Navigation;
using UnityEditor;
using UnityEngine;
using UnityEngine.AI;

public static class SurvivalistPatrolSetup
{
    private const string AnimatorControllerPath =
        "Assets/Survivalist/StarterAssets/ThirdPersonController/Character/Animations/StarterAssetsThirdPerson.controller";

    [MenuItem("Tools/Survivalist/Set Up Waypoint Patrol", false, 10)]
    private static void SetUpWaypointPatrol()
    {
        GameObject character = Selection.activeGameObject;
        if (character == null)
        {
            EditorUtility.DisplayDialog("Waypoint Patrol", "Select the Survivalist character in the Hierarchy first.", "OK");
            return;
        }

        NavMeshAgent agent = character.GetComponent<NavMeshAgent>();
        if (agent == null)
            agent = Undo.AddComponent<NavMeshAgent>(character);

        Undo.RecordObject(agent, "Configure patrol agent");
        agent.radius = 0.3f;
        agent.height = 1.8f;
        agent.speed = 2f;
        agent.acceleration = 8f;
        agent.angularSpeed = 240f;
        agent.stoppingDistance = 0f;
        agent.autoBraking = false;

        WaypointPatrol patrol = character.GetComponent<WaypointPatrol>();
        if (patrol == null)
            patrol = Undo.AddComponent<WaypointPatrol>(character);

        Animator animator = character.GetComponentInChildren<Animator>();
        if (animator != null)
        {
            Undo.RecordObject(animator, "Configure patrol animator");
            animator.applyRootMotion = false;

            if (animator.runtimeAnimatorController == null)
            {
                RuntimeAnimatorController controller =
                    AssetDatabase.LoadAssetAtPath<RuntimeAnimatorController>(AnimatorControllerPath);
                if (controller != null)
                    animator.runtimeAnimatorController = controller;
            }

            EditorUtility.SetDirty(animator);
        }

        Transform route = patrol.WaypointRoot;
        if (route == null)
        {
            GameObject routeObject = new GameObject(GameObjectUtility.GetUniqueNameForSibling(null, "Survivalist Patrol Route"));
            Undo.RegisterCreatedObjectUndo(routeObject, "Create patrol route");
            route = routeObject.transform;

            Vector3[] offsets =
            {
                new Vector3(0f, 0f, 2f),
                new Vector3(2f, 0f, 2f),
                new Vector3(2f, 0f, -2f),
                new Vector3(0f, 0f, -2f)
            };

            for (int i = 0; i < offsets.Length; i++)
            {
                GameObject waypoint = new GameObject($"Waypoint {i + 1}");
                Undo.RegisterCreatedObjectUndo(waypoint, "Create patrol waypoint");
                waypoint.transform.SetParent(route, true);
                waypoint.transform.position = SnapToNavMesh(character.transform.position + offsets[i]);
            }

            Undo.RecordObject(patrol, "Assign patrol route");
            patrol.WaypointRoot = route;
            EditorUtility.SetDirty(patrol);
        }

        if (Object.FindAnyObjectByType<NavMeshSurface>() == null)
            Debug.LogWarning("No NavMeshSurface was found. Add and bake one before using waypoint patrol.", character);

        Selection.activeTransform = route;
        EditorGUIUtility.PingObject(route);
        Debug.Log("Waypoint patrol is ready. Move, add, delete, or reorder the route's child waypoints, then enter Play mode.", character);
    }

    [MenuItem("Tools/Survivalist/Set Up Waypoint Patrol", true)]
    private static bool ValidateSetUpWaypointPatrol()
    {
        return Selection.activeGameObject != null && !EditorApplication.isPlayingOrWillChangePlaymode;
    }

    private static Vector3 SnapToNavMesh(Vector3 position)
    {
        return NavMesh.SamplePosition(position, out NavMeshHit hit, 3f, NavMesh.AllAreas)
            ? hit.position
            : position;
    }
}
