// Copyright (c) 2021, Members of Yale Interactive Machines Group, Yale University,
// Nathan Tsoi
// All rights reserved.
// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree. 

using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using SEAN.Scenario.Trajectory;
using IVI;

namespace SEAN.Scenario.Agents
{

    public class Publisher : MonoBehaviour
    {
        private ROSConnection ros;
        private SEAN sean;
        private float timeElapsed;

        public string topicName = "/social_sim/agents";
        public string frame = "map";

        public float publishMessageFrequency = 0.5f;

        public CameraAgentDetector detector;

        void Start()
        {
            ros = ROSConnection.GetOrCreateInstance();
            ros.RegisterPublisher<RosMessageTypes.Simulation.AgentArrayMsg>(topicName);
            sean = SEAN.instance;
            // print("REGISTER AGENT" + topicName);
            // print(MessageRegistry.GetRosMessageName<RosMessageTypes.SocialSimRos.AgentArrayMsg>());
        }

        private void Update()
        {
            timeElapsed += Time.deltaTime;
            if (timeElapsed <= publishMessageFrequency)
            {
                return;
            }
            PublishInformation();
            timeElapsed = 0;
        }

        private void PublishInformation(){
            RosMessageTypes.Simulation.AgentArrayMsg message = new RosMessageTypes.Simulation.AgentArrayMsg();
            message.header.frame_id = frame;
            message.header.stamp = sean.clock.LastPublishedTime();

            List<Scenario.Trajectory.TrackedAgent> people = new List<Scenario.Trajectory.TrackedAgent>();
            foreach (var agent in detector.agentsView)
            {
                if(agent.Value)
                    people.Add(agent.Key.GetComponent<TrackedAgent>());
            }

            message.agents = new RosMessageTypes.Simulation.AgentMsg[sean.pedestrianBehavior.agents.Length];
            int i = 0;
            foreach (Trajectory.TrackedAgent person in sean.pedestrianBehavior.agents)
            {
                RosMessageTypes.Simulation.AgentMsg agent = new RosMessageTypes.Simulation.AgentMsg();
                agent.type = "person";
                agent.track_id = person.track_id;
                agent.pose = Util.Geometry.GetMPose(person.gameObject.transform);
                agent.twist = Util.Geometry.GetMTwist(person.gameObject.GetComponent<SFAgent>().velocity);
                agent.visible_by_robot = people.Contains(person);
                message.agents[i++] = agent;
            }
            ros.Publish(topicName, message);
        }
    }
}
