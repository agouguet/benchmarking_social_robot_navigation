//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Simulation
{
    [Serializable]
    public class AgentArrayMsg : Message
    {
        public const string k_RosMessageName = "simulation_msgs/AgentArray";
        public override string RosMessageName => k_RosMessageName;

        //  Message defining an array of all agent entries
        public Std.HeaderMsg header;
        //  Age of the track
        public AgentMsg[] agents;
        //  Array containing the entries for the N agents in the current environment

        public AgentArrayMsg()
        {
            this.header = new Std.HeaderMsg();
            this.agents = new AgentMsg[0];
        }

        public AgentArrayMsg(Std.HeaderMsg header, AgentMsg[] agents)
        {
            this.header = header;
            this.agents = agents;
        }

        public static AgentArrayMsg Deserialize(MessageDeserializer deserializer) => new AgentArrayMsg(deserializer);

        private AgentArrayMsg(MessageDeserializer deserializer)
        {
            this.header = Std.HeaderMsg.Deserialize(deserializer);
            deserializer.Read(out this.agents, AgentMsg.Deserialize, deserializer.ReadLength());
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.header);
            serializer.WriteLength(this.agents);
            serializer.Write(this.agents);
        }

        public override string ToString()
        {
            return "AgentArrayMsg: " +
            "\nheader: " + header.ToString() +
            "\nagents: " + System.String.Join(", ", agents.ToList());
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
