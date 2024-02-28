using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.AI;

namespace IVI
{
    public class SFAgent : SEAN.Scenario.Agents.Base
    {

        private const int OBSTACLE_ANGLE_BINS = 6;

        private SphereCollider perceptionSphereAgent;
        private SphereCollider perceptionSphereWall;

        private GameObject ttc_parent;

        //NEIGHBORS
        private HashSet<GameObject> neighbors = new HashSet<GameObject>();
        private HashSet<GameObject> obstacles = new HashSet<GameObject>();

        // Neighbor Computed Values
        Dictionary<int, Vector3> closestPoints;

        // Static Fields
        public static Dictionary<GameObject, SEAN.Scenario.Agents.Base> GO2Agent = new Dictionary<GameObject, SEAN.Scenario.Agents.Base>();

        //ROBOT REPULSION
        private float robotRepulsion;

        protected override void Start()
        {
            base.Start();

            if (SEAN.SEAN.instance != null)
            {
                var robot = SEAN.SEAN.instance.robot.gameObject;

                if (!GO2Agent.ContainsKey(robot))
                {
                    GO2Agent.Add(robot, null);
                }
                //neighbors.Add(robot);
            }

            GO2Agent.Add(gameObject, this);

            perceptionSphereAgent = gameObject.AddComponent<SphereCollider>();
            perceptionSphereAgent.isTrigger = true;
            perceptionSphereAgent.radius = perception;

            perceptionSphereWall = gameObject.AddComponent<SphereCollider>();
            perceptionSphereWall.isTrigger = true;
            perceptionSphereWall.radius = PERCEPTION_RADIUS_WALL;

            GameObject ttc = new GameObject("TTC");
            ttc.tag = "TTC";
            BoxCollider ttc_collider = ttc.AddComponent<BoxCollider>();
            ttc_collider.isTrigger = true;
            ttc_collider.center = new Vector3(0f, 0.1f, 5f);
            ttc_collider.size = new Vector3(0.2f, 0.2f, 10f);
            ttc.transform.parent = gameObject.transform;
            ttc.transform.localPosition = new Vector3(0f,0f,0f);

            robotRepulsion = Random.value * (Parameters.ROBOT_REPULSION_DAMPENING_MAX - Parameters.ROBOT_REPULSION_DAMPENING_MIN) + Parameters.ROBOT_REPULSION_DAMPENING_MIN;

            if(staticAgent)
            {
                GetComponent<Animator>().enabled = false;
            }

        }

        protected override Vector3 UpdateVelocity()
        {
            SEAN.Scenario.Agents.SocialForce totalForce = ComputeForce();
            var accel = totalForce.force / MASS;
            Vector3 nextVelocity = velocity + accel * Time.deltaTime;
            Debug.DrawLine(transform.position, transform.position + nextVelocity, Color.magenta);
            nextVelocity.y = 0;
            if (nextVelocity.sqrMagnitude > 0)
            {
                nextVelocity = nextVelocity.normalized * Mathf.Min(nextVelocity.magnitude, maxSpeed);
            }
            return nextVelocity;
        }

        void OnTriggerEnter(Collider other)
        {
            if (other.isTrigger)
            {
                return;
            }
            if (other.GetComponentInChildren<IVI.INavigable>() != null && !neighbors.Contains(other.gameObject))
            {
                neighbors.Add(other.gameObject);
            }
            else if (other.GetComponentInChildren<SEAN.TF.OdometryPublisher>() != null && !neighbors.Contains(SEAN.SEAN.instance.robot.gameObject))
            {
                neighbors.Add(SEAN.SEAN.instance.robot.gameObject);
            }
            else if (other.gameObject.GetComponent<BoxCollider>() != null && (SEAN.Util.Geometry.GroundPlaneDist(other.transform.position, gameObject.transform.position) - IVI.SFAgent.RADIUS) <= PERCEPTION_RADIUS_WALL && !obstacles.Contains(other.gameObject))
            {
                obstacles.Add(other.gameObject);
            }
        }

        void OnTriggerExit(Collider other)
        {
            if (other.isTrigger)
            {
                return;
            }

            if (other.GetComponentInChildren<SEAN.TF.OdometryPublisher>() != null && neighbors.Contains(SEAN.SEAN.instance.robot.gameObject))
                neighbors.Remove(SEAN.SEAN.instance.robot.gameObject);

            if (neighbors.Contains(other.gameObject))
            {
                neighbors.Remove(other.gameObject);
            }
            if (obstacles.Contains(other.gameObject))
            {
                obstacles.Remove(other.gameObject);
            }
        }

        #region Forces

        private SEAN.Scenario.Agents.SocialForce ComputeForce()
        {
            SEAN.Scenario.Agents.SocialForce totalForce = CalculateAgentForce();
            //Debug.DrawLine(transform.position, transform.position + CalculateAgentForce().force, Color.yellow);
            //print("AgentForce: '" + totalForce.force + "'");
            totalForce.force += CalculateGoalForce();
            //Debug.DrawLine(transform.position, transform.position + CalculateGoalForce(), Color.blue);
            //print("GoalForce: '" + CalculateGoalForce() + "'");
            totalForce.force += CalculateWallForce();
            //print("WallForce: '" + CalculateWallForce() + "'");

            #region Limit Backward/Lateral Motion

            var ang = Vector3.Dot(transform.forward, totalForce.force.normalized);
            if (ang < 0)
            {
                var projectOnForward = transform.forward * Vector3.Dot(transform.forward, totalForce.force);
                totalForce.force -= projectOnForward;
                //totalForce.force += projectOnForward / (Parameters.BACKWARD_DAMPENING * Mathf.Abs(ang));
            }
            var projectOnRight = transform.right * Vector3.Dot(transform.right, totalForce.force);
            totalForce.force -= projectOnRight;
            totalForce.force += projectOnRight / Parameters.LATERAL_DAMPENING;
            //ang = Mathf.Abs(Vector3.Dot(transform.right, totalForce.force.normalized));
            //if (ang > 0.5f)
            //{
            //    var projectOnRight = transform.right * Vector3.Dot(transform.right, totalForce.force);
            //    totalForce.force -= projectOnRight;
            //    totalForce.force += projectOnRight / (Parameters.LATERAL_DAMPENING * 2 * (ang - 0.5f));
            //}

            #endregion

            //if (totalForce.force.magnitude > 0)
            //{
            //    totalForce.force = totalForce.force.normalized * Mathf.Min(totalForce.force.magnitude, 200);
            //}
            //Debug.DrawLine(transform.position, transform.position + totalForce.force, Color.green);

            return totalForce;
        }

        private Vector3 CalculateGoalForce()
        {
            var temp = nearestGoalPoint - transform.position;
            temp.y = 0;
            var desiredVel = temp.normalized * desiredSpeed;
            return MASS * (desiredVel - velocity) / Parameters.T;
        }

        private SEAN.Scenario.Agents.SocialForce CalculateAgentForce()
        {
            SEAN.Scenario.Agents.SocialForce agentForce = new SEAN.Scenario.Agents.SocialForce();

            foreach (GameObject n in neighbors)
            {
                if (GO2Agent.ContainsKey(n))
                {
                    MonoBehaviour neighbor = GO2Agent[n];
                    Debug.DrawLine(transform.position, n.transform.position, Color.red);

                    Vector3 dir = Vector3.zero;
                    float overlap = 0;
                    float dampenFactor = 0;

                    if (neighbor != null)
                    {
                        dir = transform.position - neighbor.transform.position;
                        dir.y = 0;
                        overlap = 2 * RADIUS - dir.magnitude;
                        dir = dir.normalized;
                        dampenFactor = 1f;
                    }
                    else
                    {
                        SEAN.Scenario.Robot robot = n.GetComponent<SEAN.Scenario.Robot>();
                        dir = transform.position - robot.transform.position;
                        dir.y = 0;
                        overlap = (RADIUS + ROBOT_RADIUS) - dir.magnitude;
                        dir = dir.normalized;
                        neighbor = robot;
                        var robotRB = robot.GetComponentInChildren<Rigidbody>();
                        dampenFactor = robotRB.velocity.magnitude > 0.1f ? robotRepulsion : 1f;
                    }
                    Vector3 goalDir = (nearestGoalPoint - transform.position).normalized;
                    var neighborAvatar = neighbor.GetComponent<SEAN.Scenario.Agents.Base>();
                    var neighborDir = (neighborAvatar == null ? neighbor.transform.forward : neighborAvatar.velocity) - velocity;

                    overlap += 0.5f;
                    //if (neighborAvatar.velocity.magnitude == 0)
                    //    overlap += 1f;

                    agentForce.force += Parameters.A * Mathf.Exp(overlap / Parameters.B) * dir * dampenFactor;

                    //var neighborDir = neighborAvatar != null && neighborAvatar.path.Count == 0 ? neighbor.transform.forward : neighborAvatar.path[0] - neighborAvatar.transform.position;
                    bool inFront = Vector3.Dot(-dir, goalDir) >= 0.5;
                    bool approaching = Vector3.Dot(goalDir, neighborDir.normalized) < 0;
                    if (inFront && approaching)
                    {
                        //Debug.DrawLine(transform.position, n.transform.position, Color.yellow);
                        var sideStepScale = - Vector3.Dot(-dir, goalDir);
                        agentForce.force += sideStepScale * Parameters.A / 10 * SEAN.Util.Geometry.Tangent(goalDir) * dampenFactor;

                        agentForce.anyAgentInFront = inFront;
                        agentForce.anyAgentApproaching = approaching;
                    }
                }
            }

            return agentForce;
        }

        private Vector3 CalculateWallForce()
        {
            closestPoints = new Dictionary<int, Vector3>();
            foreach (var obstacle in obstacles)
            {
                var obsBounds = obstacle.GetComponent<BoxCollider>().bounds;
                var agentBounds = GetComponentInChildren<Renderer>().bounds;
                var boundVolume = (obsBounds.max.x - obsBounds.min.x) * (obsBounds.max.y - obsBounds.min.y) * (obsBounds.max.z - obsBounds.min.z);
                var invalid = obsBounds.max.y < agentBounds.min.y + 0.1f || obsBounds.min.y > agentBounds.max.y;
                if (invalid)
                {
                    continue;
                }

                Vector3 closestPoint = obstacle.GetComponent<BoxCollider>().ClosestPoint(transform.position);
                int bin = (int)((Vector3.SignedAngle(transform.forward, closestPoint - transform.position, Vector3.up) + 180) / (360 / OBSTACLE_ANGLE_BINS)) % OBSTACLE_ANGLE_BINS;

                if (closestPoints.ContainsKey(bin))
                {
                    if ((closestPoints[bin] - transform.position).sqrMagnitude > (closestPoint - transform.position).sqrMagnitude)
                        closestPoints[bin] = closestPoint;
                }
                else
                {
                    closestPoints[bin] = closestPoint;
                }
            }

            var wallForce = Vector3.zero;
            foreach (var closestPoint in closestPoints.Values)
            {
                //Debug.DrawLine(transform.position + Vector3.up, new Vector3() { x = closestPoint.x, y = transform.position.y, z = closestPoint.z } + Vector3.up, Color.green);

                var wallNorm = transform.position - closestPoint;
                wallNorm.y = 0;
                var overlap = RADIUS - wallNorm.magnitude;

                wallForce += Parameters.WALL_A * Mathf.Exp(overlap / Parameters.WALL_B) * wallNorm;

                //var tangent = new Vector3(-wallNorm.z, 0, wallNorm.x);
                //wallForce += Parameters.WALL_KAPPA * (overlap > 0f ? overlap : 0) * Vector3.Dot(GetVelocity(), tangent) * tangent;
            }

            return wallForce;
        }

        #endregion


        protected override void OnDrawGizmosSelected()
        {
            base.OnDrawGizmosSelected();
            if (!ShowDebug) { return; }
            // Display the explosion radius when selected
            // Red lines to people
            Gizmos.color = new Color(1, 0, 0, 0.75F);
            foreach (GameObject n in neighbors)
            {
                if (GO2Agent.ContainsKey(n))
                {
                    var neighbor = GO2Agent[n];
                    if (neighbor != null)
                    {
                        Gizmos.DrawLine(transform.position, neighbor.transform.position);
                    }
                }
            }
            // Green lines to walls
            Gizmos.color = new Color(0, 1, 0, 0.5F);
            if (closestPoints != null)
            {
                foreach (var closestPoint in closestPoints)
                {
                    Gizmos.DrawLine(transform.position, closestPoint.Value);
                }
            }
        }
    }
}
