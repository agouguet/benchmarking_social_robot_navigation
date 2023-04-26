// Copyright (c) 2021, Members of Yale Interactive Machines Group, Yale University,
// Nathan Tsoi
// All rights reserved.
// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree. 

using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;

namespace SEAN.Scenario.Agents
{

    public class Publisher : MonoBehaviour
    {
        private ROSConnection ros;
        private SEAN sean;

        public string topicName = "/social_sim/agents";
        public string frame = "map";

        void Start()
        {
            ros = ROSConnection.instance;
            sean = SEAN.instance;
        }

        private void Update()
        {
            RosMessageTypes.SocialSimRos.MAgentArray message = new RosMessageTypes.SocialSimRos.MAgentArray();
            message.header.frame_id = frame;
            message.header.stamp = sean.clock.LastPublishedTime();
            message.agents = new RosMessageTypes.SocialSimRos.MAgent[sean.pedestrianBehavior.agents.Length];
            int i = 0;
            foreach (Trajectory.TrackedAgent person in sean.pedestrianBehavior.agents)
            {
                RosMessageTypes.SocialSimRos.MAgent agent = new RosMessageTypes.SocialSimRos.MAgent();
                agent.type = "person";
                agent.track_id = person.track_id;
                agent.pose = Util.Geometry.GetMPose(person.gameObject.transform);
                agent.twist = Util.Geometry.GetMTwist(person.trajectory);
                message.agents[i++] = agent;
            }
            ros.Send(topicName, message);
        }
    }
}
