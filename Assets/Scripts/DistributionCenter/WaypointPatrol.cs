using UnityEngine;
using UnityEngine.AI;

namespace DistributionCenter
{
    [DisallowMultipleComponent]
    [RequireComponent(typeof(NavMeshAgent))]
    public sealed class WaypointPatrol : MonoBehaviour
    {
        public enum RouteMode
        {
            Loop,
            PingPong
        }

        [Header("Route")]
        [SerializeField] private Transform waypointRoot;
        [SerializeField] private RouteMode routeMode = RouteMode.Loop;
        [SerializeField, Min(0.05f)] private float waypointReachDistance = 0.75f;

        [Header("Navigation")]
        [SerializeField, Min(0.1f)] private float navMeshSnapDistance = 2f;

        [Header("Animation")]
        [SerializeField] private Animator animator;
        [SerializeField, Min(0f)] private float animationDampTime = 0.1f;

        private static readonly int SpeedHash = Animator.StringToHash("Speed");
        private static readonly int MotionSpeedHash = Animator.StringToHash("MotionSpeed");
        private static readonly int GroundedHash = Animator.StringToHash("Grounded");
        private static readonly int FreeFallHash = Animator.StringToHash("FreeFall");

        private NavMeshAgent agent;
        private int waypointIndex;
        private int direction = 1;
        private bool hasSpeedParameter;
        private bool hasMotionSpeedParameter;
        private bool hasGroundedParameter;
        private bool hasFreeFallParameter;

        public Transform WaypointRoot
        {
            get => waypointRoot;
            set => waypointRoot = value;
        }

        private int WaypointCount => waypointRoot == null ? 0 : waypointRoot.childCount;

        private void Awake()
        {
            agent = GetComponent<NavMeshAgent>();
            if (animator == null)
                animator = GetComponentInChildren<Animator>();

            // Movement tuning belongs to NavMeshAgent.  The patrol only controls the route.
            agent.autoBraking = false;
            CacheAnimatorParameters();
        }

        private void Start()
        {
            if (!PlaceOnNavMesh())
            {
                Debug.LogWarning($"{name} could not find a NavMesh within {navMeshSnapDistance:0.##} metres.", this);
                enabled = false;
                return;
            }

            GoToCurrentWaypoint();
        }

        private void Update()
        {
            UpdateAnimation();

            if (WaypointCount == 0 || agent == null || !agent.enabled || !agent.isOnNavMesh)
                return;

            if (agent.pathPending || float.IsInfinity(agent.remainingDistance))
                return;

            if (agent.remainingDistance <= Mathf.Max(agent.stoppingDistance, waypointReachDistance))
            {
                AdvanceWaypoint();
                GoToCurrentWaypoint();
            }
        }

        private void OnDisable()
        {
            if (agent != null && agent.enabled && agent.isOnNavMesh)
                agent.ResetPath();

            SetAnimationValues(0f);
        }

        private bool PlaceOnNavMesh()
        {
            if (agent.isOnNavMesh)
                return true;

            if (!NavMesh.SamplePosition(transform.position, out NavMeshHit hit, navMeshSnapDistance, agent.areaMask))
                return false;

            return agent.Warp(hit.position);
        }

        private void GoToCurrentWaypoint()
        {
            if (WaypointCount == 0)
                return;

            waypointIndex = Mathf.Clamp(waypointIndex, 0, WaypointCount - 1);
            Transform waypoint = waypointRoot.GetChild(waypointIndex);

            if (NavMesh.SamplePosition(waypoint.position, out NavMeshHit hit, navMeshSnapDistance, agent.areaMask))
                agent.SetDestination(hit.position);
            else
                Debug.LogWarning($"Waypoint '{waypoint.name}' is not close enough to the NavMesh.", waypoint);
        }

        private void AdvanceWaypoint()
        {
            if (WaypointCount <= 1)
                return;

            if (routeMode == RouteMode.Loop)
            {
                waypointIndex = (waypointIndex + 1) % WaypointCount;
                return;
            }

            waypointIndex += direction;
            if (waypointIndex >= WaypointCount)
            {
                waypointIndex = WaypointCount - 2;
                direction = -1;
            }
            else if (waypointIndex < 0)
            {
                waypointIndex = 1;
                direction = 1;
            }
        }

        private void CacheAnimatorParameters()
        {
            if (animator == null || animator.runtimeAnimatorController == null)
                return;

            foreach (AnimatorControllerParameter parameter in animator.parameters)
            {
                hasSpeedParameter |= parameter.nameHash == SpeedHash && parameter.type == AnimatorControllerParameterType.Float;
                hasMotionSpeedParameter |= parameter.nameHash == MotionSpeedHash && parameter.type == AnimatorControllerParameterType.Float;
                hasGroundedParameter |= parameter.nameHash == GroundedHash && parameter.type == AnimatorControllerParameterType.Bool;
                hasFreeFallParameter |= parameter.nameHash == FreeFallHash && parameter.type == AnimatorControllerParameterType.Bool;
            }

            if (hasGroundedParameter)
                animator.SetBool(GroundedHash, true);
            if (hasFreeFallParameter)
                animator.SetBool(FreeFallHash, false);
        }

        private void UpdateAnimation()
        {
            float speed = agent != null && agent.enabled && agent.isOnNavMesh ? agent.velocity.magnitude : 0f;
            SetAnimationValues(speed);
        }

        private void SetAnimationValues(float speed)
        {
            if (animator == null || animator.runtimeAnimatorController == null)
                return;

            if (hasSpeedParameter)
                animator.SetFloat(SpeedHash, speed, animationDampTime, Time.deltaTime);
            if (hasMotionSpeedParameter)
                animator.SetFloat(MotionSpeedHash, speed > 0.05f ? 1f : 0f, animationDampTime, Time.deltaTime);
        }

        private void OnDrawGizmos()
        {
            if (waypointRoot == null || waypointRoot.childCount == 0)
                return;

            Gizmos.color = new Color(0.1f, 0.85f, 1f, 0.9f);
            for (int i = 0; i < waypointRoot.childCount; i++)
            {
                Transform current = waypointRoot.GetChild(i);
                Gizmos.DrawSphere(current.position, 0.18f);

                if (i + 1 < waypointRoot.childCount)
                    Gizmos.DrawLine(current.position, waypointRoot.GetChild(i + 1).position);
            }

            if (routeMode == RouteMode.Loop && waypointRoot.childCount > 1)
                Gizmos.DrawLine(waypointRoot.GetChild(waypointRoot.childCount - 1).position, waypointRoot.GetChild(0).position);
        }
    }
}
