using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using Unity.Robotics.Core;

namespace SBSN.Robot{
    public class OdometryPublisher : MonoBehaviour
    {
        ROSConnection ros;
        public float publishMessageFrequency = 0.5f;
        private float timeElapsed;
        public string topicName;

        public Transform PublishedTransform;
        public string FrameId = "map";

        private RosMessageTypes.Nav.OdometryMsg message;

        private float previousRealTime;
        private Vector3 previousPosition = Vector3.zero;
        private Quaternion previousRotation = Quaternion.identity;

        private double[] identityMatrix = {1, 0, 0, 0, 0, 0,
                                            0, 1, 0, 0, 0, 0,
                                            0, 0, 1, 0, 0, 0,
                                            0, 0, 0, 1, 0, 0,
                                            0, 0, 0, 0, 1, 0,
                                            0, 0, 0, 0, 0, 1};

        void Start()
        {
            ros = ROSConnection.GetOrCreateInstance();
            ros.RegisterPublisher<RosMessageTypes.Nav.OdometryMsg>(topicName);
            InitializeMessage();

        }

        private void FixedUpdate()
        {
            UpdateMessage();
        }

        private void InitializeMessage()
        {
            message = new RosMessageTypes.Nav.OdometryMsg();
            message.pose.covariance = identityMatrix;
            message.twist.covariance = identityMatrix;
            message.child_frame_id = FrameId;
        }

        private void UpdateMessage()
        {
            float deltaTime = Time.realtimeSinceStartup - previousRealTime;
            timeElapsed += Time.deltaTime;

            if (timeElapsed <= publishMessageFrequency)
            {
                return;
            }

            Vector3 linearVelocity = (PublishedTransform.position - previousPosition) / deltaTime;
            Vector3 angularVelocity = (PublishedTransform.rotation.eulerAngles - previousRotation.eulerAngles) / deltaTime;

            previousRealTime = Time.realtimeSinceStartup;
            previousPosition = PublishedTransform.position;
            previousRotation = PublishedTransform.rotation;

            message.header = new RosMessageTypes.Std.HeaderMsg();
            message.header.stamp = new TimeStamp(Clock.time);
            message.twist.twist.linear = GetGeometryVector3(linearVelocity.To<FLU>());
            message.twist.twist.angular = GetGeometryVector3(-angularVelocity.To<FLU>());
            message.pose.pose.position = GetGeometryPoint(PublishedTransform.position.To<FLU>());
            message.pose.pose.orientation = GetGeometryQuaternion(PublishedTransform.rotation.To<FLU>());
            ros.Publish(topicName, message);
            timeElapsed = 0;
        }

        private static RosMessageTypes.Geometry.Vector3Msg GetGeometryVector3(Vector3<FLU> vector3)
        {
            RosMessageTypes.Geometry.Vector3Msg geometryVector3 = new RosMessageTypes.Geometry.Vector3Msg();
            geometryVector3.x = vector3.x;
            geometryVector3.y = vector3.y;
            geometryVector3.z = vector3.z;
            return geometryVector3;
        }

        private static RosMessageTypes.Geometry.PointMsg GetGeometryPoint(Vector3<FLU> position)
        {
            RosMessageTypes.Geometry.PointMsg geometryPoint = new RosMessageTypes.Geometry.PointMsg();
            geometryPoint.x = position.x;
            geometryPoint.y = position.y;
            geometryPoint.z = position.z;
            return geometryPoint;
        }

        private static RosMessageTypes.Geometry.QuaternionMsg GetGeometryQuaternion(Quaternion<FLU> quaternion)
        {
            RosMessageTypes.Geometry.QuaternionMsg geometryQuaternion = new RosMessageTypes.Geometry.QuaternionMsg();
            geometryQuaternion.x = quaternion.x;
            geometryQuaternion.y = quaternion.y;
            geometryQuaternion.z = quaternion.z;
            geometryQuaternion.w = quaternion.w;
            return geometryQuaternion;
        }
    }
}