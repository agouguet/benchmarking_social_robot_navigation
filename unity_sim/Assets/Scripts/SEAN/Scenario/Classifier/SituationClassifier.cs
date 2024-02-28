// Copyright (c) 2021, Members of Yale Interactive Machines Group, Yale University,
// Nathan Tsoi
// All rights reserved.
// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree. 

using UnityEngine;

using Unity.Robotics.ROSTCPConnector;

using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace SEAN.Scenario.Classifier
{
    public abstract class SituationClassifier : MonoBehaviour
    {
        protected ROSConnection ros;
        protected SEAN sean;
        protected string BaseTopic = "/social_sim/situations/";

        protected abstract string ClassifierType { get; }

        public Scenario.Situation empty { get; private set; }
        public Scenario.Situation downPath { get; private set; }
        public Scenario.Situation crossPath { get; private set; }
        public Scenario.Situation leaveGroup { get; private set; }
        public Scenario.Situation joinGroup { get; private set; }

        public float lastUpdateTime { get; protected set; }

        virtual public void Start()
        {
            empty = Scenario.Situation.Empty;
            downPath = Scenario.Situation.DownPath;
            crossPath = Scenario.Situation.CrossPath;
            leaveGroup = Scenario.Situation.LeaveGroup;
            joinGroup = Scenario.Situation.JoinGroup;

            lastUpdateTime = 0f;
            ros = ROSConnection.GetOrCreateInstance();
            sean = SEAN.instance;
        }

        protected void Publish(Scenario.Situation situation)
        {
            //if (situation.name == "empty") {
            //    print("Publishing: " + situation.name + ": " + situation.val);
            //}
            //if (situation.name == "leave_group") {
            //    print("Publishing: " + situation.name + ": " + situation.val);
            //}
            RosMessageTypes.Std.Float32Msg msg = new RosMessageTypes.Std.Float32Msg();
            msg.data = situation.val;
            if (! ros.GetOrCreateTopic(BaseTopic + ClassifierType + situation.name, MessageRegistry.GetRosMessageName<RosMessageTypes.Std.Float32Msg>()).IsPublisher)
            {
                ros.RegisterPublisher<RosMessageTypes.Std.Float32Msg>(BaseTopic + ClassifierType + situation.name);
            }
            ros.Publish(BaseTopic + ClassifierType + situation.name, msg);
        }
    }
}