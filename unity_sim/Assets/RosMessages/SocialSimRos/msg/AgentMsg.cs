//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;

namespace RosMessageTypes.SocialSimRos
{
    [Serializable]
    public class AgentMsg : Message
    {
        public const string k_RosMessageName = "unity_sim/Agent";
        public override string RosMessageName => k_RosMessageName;

        //  Message defining an entry of a agent
        public HeaderMsg header;
        //  Age of the track
        public ulong track_id;
        //  Unique ID for each agent
        //  Type of agent
        public string type;
        //  Pose of the track
        public Geometry.PoseMsg pose;
        //  Velocity of the track
        public Geometry.TwistMsg twist;

        public AgentMsg()
        {
            this.header = new HeaderMsg();
            this.track_id = 0;
            this.type = "";
            this.pose = new Geometry.PoseMsg();
            this.twist = new Geometry.TwistMsg();
        }

        public AgentMsg(HeaderMsg header, ulong track_id, string type, Geometry.PoseMsg pose, Geometry.TwistMsg twist)
        {
            this.header = header;
            this.track_id = track_id;
            this.type = type;
            this.pose = pose;
            this.twist = twist;
        }

        public static AgentMsg Deserialize(MessageDeserializer deserializer) => new AgentMsg(deserializer);

        private AgentMsg(MessageDeserializer deserializer)
        {
            this.header = HeaderMsg.Deserialize(deserializer);
            deserializer.Read(out this.track_id);
            deserializer.Read(out this.type);
            this.pose = Geometry.PoseMsg.Deserialize(deserializer);
            this.twist = Geometry.TwistMsg.Deserialize(deserializer);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.header);
            serializer.Write(this.track_id);
            serializer.Write(this.type);
            serializer.Write(this.pose);
            serializer.Write(this.twist);
        }

        public override string ToString()
        {
            return "AgentMsg: " +
            "\nheader: " + header.ToString() +
            "\ntrack_id: " + track_id.ToString() +
            "\ntype: " + type.ToString() +
            "\npose: " + pose.ToString() +
            "\ntwist: " + twist.ToString();
        }

#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [UnityEngine.RuntimeInitializeOnLoadMethod]
#endif
        public static void Register()
        {
            MessageRegistry.Register(k_RosMessageName, Deserialize);
        }
    }
}
