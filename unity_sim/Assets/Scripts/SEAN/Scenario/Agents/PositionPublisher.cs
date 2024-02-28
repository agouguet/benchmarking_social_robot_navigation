// Copyright (c) 2021, Members of Yale Interactive Machines Group, Yale University,
// Nathan Tsoi
// All rights reserved.
// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree. 

using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using SEAN.Scenario.Trajectory;

namespace SEAN.Scenario.Agents
{

    public class PositionPublisher : MonoBehaviour
    {
        private ROSConnection ros;
        private SEAN sean;
        private float timeElapsed;
        

        public string topicName = "/social_sim/agent_positions";
        public float publishMessageFrequency = 0.5f;
        public string frame = "map";


        public CameraAgentDetector detector;
        public bool haveGlobalInformation = false;

        void Start()
        {
            ros = ROSConnection.GetOrCreateInstance();
            ros.RegisterPublisher<RosMessageTypes.Geometry.PoseArrayMsg>(topicName);
            sean = SEAN.instance;
        }

        private void Update()
        {
            timeElapsed += Time.deltaTime;
            if (timeElapsed <= publishMessageFrequency)
            {
                return;
            }

            if(haveGlobalInformation)
                PublishGlobalInformation();
            else
                PublishLocalInformation();
            timeElapsed = 0;
        }

        private void PublishGlobalInformation()
        {
            RosMessageTypes.Geometry.PoseArrayMsg message = new RosMessageTypes.Geometry.PoseArrayMsg();
            message.header.frame_id = frame;
            message.header.stamp = sean.clock.LastPublishedTime();
            // Filter out the robot
            List<Scenario.Trajectory.TrackedAgent> people = new List<Scenario.Trajectory.TrackedAgent>();
            foreach (Trajectory.TrackedAgent person in sean.pedestrianBehavior.agents)
            {
                //if (person is IVI.RobotAgent) { continue; }
                people.Add(person);
            }
            message.poses = new RosMessageTypes.Geometry.PoseMsg[people.Count];
            int i = 0;
            foreach (Trajectory.TrackedAgent person in people)
            {
                message.poses[i++] = Util.Geometry.GetMPose(person.gameObject.transform);
            }
            ros.Publish(topicName, message);
        }

        private void PublishLocalInformation()
        {
            RosMessageTypes.Geometry.PoseArrayMsg message = new RosMessageTypes.Geometry.PoseArrayMsg();
            message.header.frame_id = frame;
            message.header.stamp = sean.clock.LastPublishedTime();

            List<Scenario.Trajectory.TrackedAgent> people = new List<Scenario.Trajectory.TrackedAgent>();
            foreach (var agent in detector.agentsView)
            {
                if(agent.Value)
                    people.Add(agent.Key.GetComponent<TrackedAgent>());
            }
            message.poses = new RosMessageTypes.Geometry.PoseMsg[people.Count];
            int i = 0;
            foreach (Trajectory.TrackedAgent person in people)
            {
                message.poses[i++] = Util.Geometry.GetMPose(person.gameObject.transform);
            }
            ros.Publish(topicName, message);
        }
    }
}
