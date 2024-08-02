using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.Core;
using RosMessageTypes.Geometry;

namespace SBSN.Scenario
{
    public class ScenarioPublisher : MonoBehaviour
    {
        public string topicName = "/social_sim/scene_info";
        ROSConnection ros;

        private RosMessageTypes.Simulation.SceneInfoMsg message;

        void Start()
        {
            ros = ROSConnection.GetOrCreateInstance();
            ros.RegisterPublisher<RosMessageTypes.Simulation.SceneInfoMsg>(topicName);
            message = new RosMessageTypes.Simulation.SceneInfoMsg();
        }

        private void FixedUpdate()
        {
            message.header = new RosMessageTypes.Std.HeaderMsg();
            message.header.stamp = new TimeStamp(Clock.time);
            message.scenario_name = "";
            message.robot_start_pose = new PoseMsg();
            message.robot_target_pose = new PoseMsg();
            message.num_people = 0;
            message.num_groups = 0;
            message.environment = "hospital";
            ros.Publish(topicName, message);
        }
    }
}
