// Copyright (c) 2021, Members of Yale Interactive Machines Group, Yale University,
// Nathan Tsoi
// All rights reserved.
// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree. 

using UnityEngine;
using System;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.BuiltinInterfaces;

namespace SEAN.ROSClock
{
    public class ROSClockPublisher : MonoBehaviour
    {
        ROSConnection ros;
        public string topicName = "clock";

        private static ROSClockPublisher _instance;
        public static ROSClockPublisher instance { get { return _instance; } }

        public static DateTime UNIX_EPOCH = new DateTime(1970, 1, 1, 0, 0, 0, 0, DateTimeKind.Utc);
        public double StartMillis { get; private set; }


        public double LastMSecs { get; private set; }
        private TimeMsg lastPublishedTime;

        private void Awake()
        {
            if (_instance != null && _instance != this)
            {
                Destroy(this.gameObject);
            }
            else
            {
                //print("TIMESCALE: " + Time.timeScale);
                _instance = this;
                StartMillis = (DateTime.Now.ToUniversalTime() - UNIX_EPOCH).TotalMilliseconds;
                //print("startMillis: " + startMillis);
                lastPublishedTime = this.Now();
            }
        }

        void Start()
        {
            ros = ROSConnection.GetOrCreateInstance();
            ros.RegisterPublisher<RosMessageTypes.Rosgraph.ClockMsg >(topicName);
        }

        void FixedUpdate()
        {
            instance.Publish();
        }

        public TimeMsg Now()
        {
            return Util.Time.TimeMsg(Util.Time.Milliseconds(StartMillis));
        }

        public TimeMsg LastPublishedTime()
        {
            return lastPublishedTime;
        }

        public void Publish()
        {
            double msecs = Util.Time.Milliseconds(StartMillis);
            if (LastMSecs == msecs)
            {
                return;
            }
            LastMSecs = msecs;
            TimeMsg t = instance.Now();
            RosMessageTypes.Rosgraph.ClockMsg message = new RosMessageTypes.Rosgraph.ClockMsg();
            message.clock.sec = t.sec;
            message.clock.nanosec = t.nanosec;
            //print("clock: " + t.secs + "." + t.nsecs);
            ros.Publish(topicName, message);
            lastPublishedTime = t;
        }

        public void UpdateMHeader(RosMessageTypes.Std.HeaderMsg header)
        {
            UpdateMHeaders(new List<RosMessageTypes.Std.HeaderMsg>() { header });
        }

        public void UpdateMHeaders(List<RosMessageTypes.Std.HeaderMsg> headers)
        {
            foreach (RosMessageTypes.Std.HeaderMsg header in headers)
            {
                header.stamp.sec = lastPublishedTime.sec;
                header.stamp.nanosec = lastPublishedTime.nanosec;
            }
        }
    }
}