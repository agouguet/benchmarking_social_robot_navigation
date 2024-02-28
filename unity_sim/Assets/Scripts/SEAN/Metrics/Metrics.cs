// Copyright (c) 2021, Members of Yale Interactive Machines Group, Yale University,
// Nathan Tsoi
// All rights reserved.
// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree. 

using System;
using System.Collections.Generic;
using UnityEngine;
using RosMessageTypes.BuiltinInterfaces;
using Unity.VisualScripting;
using System.Linq;

namespace SEAN.Metrics
{
    public class Metrics : MonoBehaviour
    {
        protected SEAN sean;

        #region parameters
        private bool ShowDebug = false;

        public float UpdateFrequencyHz = 1f;
        public float IntimateDistance = 0.45f;
        public float PersonalDistance = 1.2f;

        // How long of a window should we look at over which we check to see if the robot has moved?
        public float StuckWindowSeconds = 5f;
        // How far does the robot need to move over the stuck window to not be considered stuck?
        public float StuckWindowDist = 0.01f;
        #endregion

        private float lastUpdate = 0f;
        private float UpdateSec { get { return 1 / UpdateFrequencyHz; } }
        public TimeMsg StartTime { get; private set; }
        public uint ObjectCollisions { get; private set; }
        public double MinDistToPed { get; private set; }
        public double MinDistToTarget { get; private set; }

        public bool HasActors { get { return sean.pedestrianBehavior.agents.Length > 0; } }
        public List<Pose> RobotPoses { get; private set; }
        public List<TimeMsg> RobotPosesTS { get; private set; }

        public Vector3 LastVelocity { get; private set; }
        public Vector3 LastAcceleration { get; private set; }

        public uint RobotOnPersonCollisions { get; private set; }
        public uint PersonOnRobotCollisions { get; private set; }
        public uint RobotOnPersonIntimateDistViolations { get; private set; }
        public uint PersonOnRobotIntimateDistViolations { get; private set; }
        public uint RobotOnPersonPersonalDistViolations { get; private set; }
        public uint PersonOnRobotPersonalDistViolations { get; private set; }
        public double TimeInPersonalSpace {get; private set; }

        public void OnNewTask()
        {
            if (ShowDebug)
            {
                print("Metrics OnNewTask");
            }
            Reset();
        }

        public void Start()
        {
            sean = SEAN.instance;
            Reset();
            // Register the delegate for new task events
            if (ShowDebug)
            {
                print("Registering Metrics delegate");
            }
            sean.robotTask.onNewTask += OnNewTask;
            // Make sure our robot has a count collision component
            if (sean.robot.base_link.GetComponent<CountCollisions>() == null)
            {
                sean.robot.base_link.AddComponent<CountCollisions>();
            }
        }

        public void Update()
        {
            if (!sean.robotTask.isRunning) { return; }
            if (lastUpdate != 0 && Time.time - lastUpdate < UpdateSec)
            {
                return;
            }
            UpdateMetrics();
            lastUpdate = Time.time;
        }

        private void Reset()
        {
            if (ShowDebug)
            {
                print("Metrics Reset");
            }
            StartTime = sean.clock.Now();
            RobotPoses = new List<Pose>();
            RobotPosesTS = new List<TimeMsg>();
            ObjectCollisions = 0;
            MinDistToPed = double.MaxValue;
            MinDistToTarget = double.MaxValue;

            RobotOnPersonCollisions = 0;
            PersonOnRobotCollisions = 0;
            RobotOnPersonIntimateDistViolations = 0;
            PersonOnRobotIntimateDistViolations = 0;
            RobotOnPersonPersonalDistViolations = 0;
            PersonOnRobotPersonalDistViolations = 0;
        }


        private void UpdateMetrics()
        {
            // add a location
            RobotPoses.Add(new Pose(sean.robot.position, sean.robot.rotation));
            RobotPosesTS.Add(sean.clock.Now());
            MinDistToTarget = Math.Min(MinDistToTarget, TargetDist);
            MinDistToPed = Math.Min(MinDistToPed, NearestPedestrian);
        }

        public void IncrementObjectCollisions()
        {
            ObjectCollisions++;
        }

        public void IncrementPeopleCollisions(bool isRobotAtFault)
        {
            if (isRobotAtFault)
            {
                RobotOnPersonCollisions++;
            }
            else
            {
                PersonOnRobotCollisions++;
            }
        }

        public void IncrementIntimateSpaceViolations(bool isRobotAtFault)
        {
            if (isRobotAtFault)
            {
                RobotOnPersonIntimateDistViolations++;
            }
            else
            {
                PersonOnRobotIntimateDistViolations++;
            }
        }

        public void IncrementPersonalDistViolations(bool isRobotAtFault)
        {
            if (isRobotAtFault)
            {
                RobotOnPersonPersonalDistViolations++;
            }
            else
            {
                PersonOnRobotPersonalDistViolations++;
            }
        }

        public void IncrementTimeInPersonalSpace(double timeDelta)
        {
            TimeInPersonalSpace += timeDelta;
        }

        public double PathLength
        {
            get
            {
                double pathLength = 0;
                for (int i = 0; i < RobotPoses.Count - 1; i++)
                {
                    pathLength += Util.Geometry.GroundPlaneDist(RobotPoses[i].position, RobotPoses[i + 1].position);
                }
                return pathLength;
            }
        }

        public double PathIrregularity
        {
            get
            {
                double pathIrregularity = 0;
                for (int i = 0; i < RobotPoses.Count - 1; i++)
                {
                    pathIrregularity += Quaternion.Angle(RobotPoses[i].rotation, RobotPoses[i + 1].rotation);
                }
                return pathIrregularity;
            }
        }

        public double TimeNotMoving
        {
            get
            {
                double timeNotMoving = 0;
                if(RobotPoses.Count > 0)
                {
                    int second = RobotPosesTS[0].sec;
                    Pose currentPose = RobotPoses[0];
                    for (int i = 1; i < RobotPoses.Count - 1; i++)
                    {
                        if (second + 1 <= RobotPosesTS[i].sec)
                        {
                            if (Util.Geometry.GroundPlaneDist(currentPose.position, RobotPoses[i].position) <= StuckWindowDist)
                            {
                                timeNotMoving += RobotPosesTS[i].sec - second;
                            }
                            second = RobotPosesTS[i].sec;
                            currentPose = RobotPoses[i];
                        }
                    }
                }
                return timeNotMoving;
            }
        }

        public double TargetDist
        {
            get
            {
                return Util.Geometry.GroundPlaneDist(sean.robot.position, sean.robotTask.robotGoal.transform.position);
            }
        }

        public double TargetDistNorm
        {
            get
            {
                return TargetDist / PathLength;
            }
        }

        private double NearestPedestrian
        {
            get
            {
                double minDist = double.MaxValue;
                foreach (Scenario.Trajectory.TrackedAgent agents in sean.pedestrianBehavior.agents)
                {
                    double dist = Util.Geometry.GroundPlaneDist(sean.robot.position, agents.transform.position);
                    if (dist < minDist)
                    {
                        minDist = dist;
                    }
                }
                return minDist;
            }
        }

        public double TimeToCollision
        {
            get
            {
                double minTTC = double.MaxValue;
                foreach (Scenario.Trajectory.TrackedAgent agents in sean.pedestrianBehavior.agents)
                {
                    double TTC = TimeToCollisionWithAnAgent(agents);
                    if (TTC < minTTC)
                    {
                        minTTC = TTC;
                    }
                }
                return minTTC;
            }
        }

        private double TimeToCollisionWithAnAgent(Scenario.Trajectory.TrackedAgent agent)
        {
            Vector2 v1 = new Vector2(sean.robot.position.x, sean.robot.position.z);
            Vector2 v2 = new Vector2(sean.robot.transform.forward.x, sean.robot.transform.forward.z) * 10;
            Vector2 v3 = new Vector2(agent.transform.position.x, agent.transform.position.z);
            Vector2 v4 = new Vector2(agent.transform.forward.x, agent.transform.forward.z) * 10;
            UnityEngine.Vector3 result = Intersection(v1, v2, v3, v4);

            Debug.DrawLine(sean.robot.position, new Vector3(result.x, 0, result.y), new Color(0.0f, 0.0f, 1.0f));

            if(SeeThePoint(sean.robot.transform, result) && SeeThePoint(agent.transform, result)){
                float dist = Vector3.Distance(sean.robot.position, result);
                Vector3 speed3d = sean.robot.base_link.GetComponent<Rigidbody>().velocity;
                float speed = Mathf.Max(Mathf.Max(speed3d.x, speed3d.y), speed3d.z);
                // print(dist + "    " + speed + "    ");
                if(speed == 0)
                    return Mathf.Infinity;
                return dist/speed;
            }

            return Mathf.Infinity;
        }

        private Vector3 Intersection( UnityEngine.Vector3  linePoint1, UnityEngine.Vector3  lineDirection1, UnityEngine.Vector3  linePoint2, UnityEngine.Vector3  lineDirection2)
        {
            Vector3 intersection;
            UnityEngine.Vector3  lineVec3 = linePoint2 - linePoint1;
            UnityEngine.Vector3  crossVec1and2 = UnityEngine.Vector3 .Cross(lineDirection1, lineDirection2);
            UnityEngine.Vector3  crossVec3and2 = UnityEngine.Vector3 .Cross(lineVec3, lineDirection2);
            float planarFactor = UnityEngine.Vector3 .Dot(lineVec3, crossVec1and2);
    
            //is coplanar, and not parallel
            if (Mathf.Abs(planarFactor) < 0.0001f
                    && crossVec1and2.sqrMagnitude > 0.0001f)
            {
                float s = UnityEngine.Vector3 .Dot(crossVec3and2, crossVec1and2) / crossVec1and2.sqrMagnitude;
                intersection = linePoint1 + (lineDirection1 * s);
                return intersection;
            }
            else
            {
                intersection = UnityEngine.Vector3 .zero;
                return intersection;
            }
        }

        private bool SeeThePoint(Transform agent, Vector3 point){
            Vector3 facing = agent.forward;
            Vector3 toTarget = (point - agent.position).normalized;
            float dot = Vector3.Dot(facing, toTarget);
            float cosFovAngle = Mathf.Cos((180/2) * Mathf.Deg2Rad);
            return dot > cosFovAngle;
        }

        public double MovementJerk
        {
            get
            {
                Vector3 acc = Acceleration();
                Vector3 jerk = (acc - LastAcceleration) / Time.fixedDeltaTime;
                LastAcceleration = acc;
                return Mathf.Sqrt(Mathf.Pow(jerk.x, 2) + Mathf.Pow(jerk.y, 2) + Mathf.Pow(jerk.z, 2));
            }
        }

        private Vector3 Acceleration(){
            Vector3 vel = RobotVelocity();
            Vector3 acceleration = (vel - LastVelocity) / Time.fixedDeltaTime;
            LastVelocity = vel;
            return acceleration;
        }

        private Vector3 RobotVelocity(){
            return sean.robot.base_link.GetComponent<Rigidbody>().velocity;
        }

    }



}
