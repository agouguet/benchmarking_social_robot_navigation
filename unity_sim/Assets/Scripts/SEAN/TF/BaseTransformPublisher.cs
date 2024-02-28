// Copyright (c) 2021, Members of Yale Interactive Machines Group, Yale University,
// Nathan Tsoi
// All rights reserved.
// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree. 

using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using System.Collections.Generic;
using RosMessageTypes.BuiltinInterfaces;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace SEAN.TF
{
    public class BaseTransformPublisher : MonoBehaviour
    {
        private ROSConnection ros;
        private TimeMsg LastHeader = new TimeMsg();

        protected void Start()
        {
            ros = ROSConnection.GetOrCreateInstance();
        }

        public class NamedTransform
        {
            public string name;
            public RosMessageTypes.Geometry.PoseStampedMsg pose;

            public NamedTransform(string name, RosMessageTypes.Geometry.PoseStampedMsg pose)
            {
                this.name = name;
                this.pose = pose;
            }
        }

        protected void PublishIfNew(NamedTransform transform)
        {
            List<NamedTransform> transforms = new List<NamedTransform>();
            transforms.Add(transform);
            PublishIfNew(transforms);
        }

        protected void PublishIfNew(List<NamedTransform> transforms)
        {
            foreach (NamedTransform transform in transforms)
            {
                SEAN.instance.clock.UpdateMHeader(transform.pose.header);
                if (LastHeader.sec >= transform.pose.header.stamp.sec && LastHeader.nanosec >= transform.pose.header.stamp.nanosec)
                {
                    return;
                }
                string name = transform.name;
                if (!name.StartsWith("/"))
                {
                    name = "/" + name;
                }
                if (! ros.GetOrCreateTopic(name, MessageRegistry.GetRosMessageName<RosMessageTypes.Geometry.PoseStampedMsg>()).IsPublisher)
                {
                    ros.RegisterPublisher<RosMessageTypes.Geometry.PoseStampedMsg>(name);
                }
                ros.Publish(name, transform.pose);
            }
            LastHeader.sec = transforms[0].pose.header.stamp.sec;
            LastHeader.nanosec = transforms[0].pose.header.stamp.nanosec;
        }
    }
}
