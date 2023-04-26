// Copyright (c) 2021, Members of Yale Interactive Machines Group, Yale University,
// Nathan Tsoi
// All rights reserved.
// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree. 

using UnityEngine;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using System.Collections.Generic;

namespace SEAN.TF
{
    public class WorldTransformPublishers : BaseTransformPublisher
    {
        SEAN sean;
        GameObject MapToOdom;
        GameObject OdomToBaseFootprint;
        GameObject BasefootprintToBaseLink;
        private bool initialized = false;
        
        RosMessageTypes.Geometry.MPoseStamped mapToOdomPoseStamped = new RosMessageTypes.Geometry.MPoseStamped();
        RosMessageTypes.Geometry.MPoseStamped odomToBasefootprintPoseStamped = new RosMessageTypes.Geometry.MPoseStamped();
        RosMessageTypes.Geometry.MPoseStamped basefootprintToBaseLinkPoseStamped = new RosMessageTypes.Geometry.MPoseStamped();

        private void Start()
        {
            sean = SEAN.instance;
            base.Start();
            
            MapToOdom = new GameObject();
            OdomToBaseFootprint = new GameObject();
            BasefootprintToBaseLink = new GameObject();
            
            MapToOdom.name = "map_to_odom";
            OdomToBaseFootprint.name = "odom_to_base_footprint";
            BasefootprintToBaseLink.name = "base_footprint_to_base_link";

            mapToOdomPoseStamped.header.frame_id = "map";
            odomToBasefootprintPoseStamped.header.frame_id = "odom";
            basefootprintToBaseLinkPoseStamped.header.frame_id = "base_footprint";
        }

        private void Update()
        {

            if (!initialized)
            {
                MapToOdom.transform.position = new Vector3(0, 0, 0);
                MapToOdom.transform.rotation = new Quaternion(0, 0, 0, 1);
                mapToOdomPoseStamped.pose.position = Util.Geometry.GetGeometryPoint(MapToOdom.transform.position.To<FLU>());
                mapToOdomPoseStamped.pose.orientation = Util.Geometry.GetGeometryQuaternion(MapToOdom.transform.rotation.To<FLU>());
                initialized = true;
            }

            OdomToBaseFootprint.transform.position = new Vector3(sean.robot.position.x, 0, sean.robot.position.z);
            OdomToBaseFootprint.transform.rotation = sean.robot.rotation;
            odomToBasefootprintPoseStamped.pose.position = Util.Geometry.GetGeometryPoint(OdomToBaseFootprint.transform.position.To<FLU>());
            odomToBasefootprintPoseStamped.pose.orientation = Util.Geometry.GetGeometryQuaternion(OdomToBaseFootprint.transform.rotation.To<FLU>());

            BasefootprintToBaseLink.transform.position = new Vector3(0, sean.robot.position.y, 0);
            BasefootprintToBaseLink.transform.rotation = new Quaternion(0,0,0,1);
            basefootprintToBaseLinkPoseStamped.pose.position = Util.Geometry.GetGeometryPoint(BasefootprintToBaseLink.transform.position.To<FLU>());
            basefootprintToBaseLinkPoseStamped.pose.orientation = Util.Geometry.GetGeometryQuaternion(BasefootprintToBaseLink.transform.rotation.To<FLU>());

            List<NamedTransform> transforms = new List<NamedTransform>(){
                new NamedTransform("/" + MapToOdom.name, mapToOdomPoseStamped),
                new NamedTransform("/" + OdomToBaseFootprint.name, odomToBasefootprintPoseStamped),
                new NamedTransform("/" + BasefootprintToBaseLink.name, basefootprintToBaseLinkPoseStamped)
            };
            PublishIfNew(transforms);
        }
    }
}
