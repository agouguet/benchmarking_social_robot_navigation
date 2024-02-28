// Copyright (c) 2021, Members of Yale Interactive Machines Group, Yale University,
// Nathan Tsoi
// All rights reserved.
// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree. 

using UnityEngine;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using RosMessageTypes.Tf2;
using RosMessageTypes.Std;
using System;
using System.Linq;

namespace SEAN.TF
{
    public class TFPublishers : MonoBehaviour
    {
        ROSConnection m_ROS;

        const string k_TfTopic = "/tf";

        [SerializeField]
        List<string> m_GlobalFrameIds = new List<string> { "map", "odom" };
        public List<GameObject> tf_elements = new List<GameObject>();

        [SerializeField]
        GameObject m_RootGameObject;
        [SerializeField]
        double m_PublishRateHz = 20f;

        SEAN sean;
        GameObject MapToOdom;
        GameObject OdomToBaseFootprint;
        GameObject BasefootprintToBaseLink;
        private bool initialized = false;

        double PublishPeriodSeconds => 1.0f / m_PublishRateHz;
        double m_LastPublishTimeSeconds;
        bool ShouldPublishMessage => SEAN.instance.clock.LastMSecs > m_LastPublishTimeSeconds + PublishPeriodSeconds;

        private TFMessageMsg message;

        private void Start()
        {
            sean = SEAN.instance;

            m_ROS = ROSConnection.GetOrCreateInstance();
            m_ROS.RegisterPublisher<TFMessageMsg>(k_TfTopic);
        }

        private void Update()
        {   
            if (ShouldPublishMessage)
            {
                var tfMessageList = new List<TransformStampedMsg>();

                if (m_GlobalFrameIds.Count > 0)
                {
                    var tfRootToGlobal = new TransformStampedMsg(
                        new HeaderMsg(SEAN.instance.clock.Now(), m_GlobalFrameIds.Last()),
                        m_RootGameObject.name,
                        m_RootGameObject.transform.To<FLU>());
                    tfMessageList.Add(tfRootToGlobal);
                }
                else
                {
                    Debug.LogWarning($"No {m_GlobalFrameIds} specified, transform tree will be entirely local coordinates.");
                }
                
                // In case there are multiple "global" transforms that are effectively the same coordinate frame, 
                // treat this as an ordered list, first entry is the "true" global
                for (var i = 1; i < m_GlobalFrameIds.Count; ++i)
                {
                    var tfGlobalToGlobal = new TransformStampedMsg(
                        new HeaderMsg(SEAN.instance.clock.Now(), m_GlobalFrameIds[i - 1]),
                        m_GlobalFrameIds[i],
                        // Initializes to identity transform
                        new TransformMsg());
                    tfMessageList.Add(tfGlobalToGlobal);
                }

                for (var i = 0; i < tf_elements.Count; ++i)
                {
                    tfMessageList.Add(
                        new RosMessageTypes.Geometry.TransformStampedMsg(
                        new RosMessageTypes.Std.HeaderMsg(SEAN.instance.clock.Now(), tf_elements[i].transform.parent.gameObject.name),
                        tf_elements[i].transform.gameObject.name,
                        new RosMessageTypes.Geometry.TransformMsg(tf_elements[i].transform.localPosition.To<FLU>(),tf_elements[i].transform.localRotation.To<FLU>()))
                    );
                }

                message = new TFMessageMsg(tfMessageList.ToArray());
                m_ROS.Publish(k_TfTopic, message);
                m_LastPublishTimeSeconds = SEAN.instance.clock.LastMSecs;

            }
        }
    }
}
