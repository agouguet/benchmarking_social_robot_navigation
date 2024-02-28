//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Simulation
{
    [Serializable]
    public class GraphNavMsg : Message
    {
        public const string k_RosMessageName = "simulation_msgs/GraphNav";
        public override string RosMessageName => k_RosMessageName;

        public Std.HeaderMsg header;
        public GraphEdgeMsg[] edges;
        public GraphNodeMsg[] nodes;

        public GraphNavMsg()
        {
            this.header = new Std.HeaderMsg();
            this.edges = new GraphEdgeMsg[0];
            this.nodes = new GraphNodeMsg[0];
        }

        public GraphNavMsg(Std.HeaderMsg header, GraphEdgeMsg[] edges, GraphNodeMsg[] nodes)
        {
            this.header = header;
            this.edges = edges;
            this.nodes = nodes;
        }

        public static GraphNavMsg Deserialize(MessageDeserializer deserializer) => new GraphNavMsg(deserializer);

        private GraphNavMsg(MessageDeserializer deserializer)
        {
            this.header = Std.HeaderMsg.Deserialize(deserializer);
            deserializer.Read(out this.edges, GraphEdgeMsg.Deserialize, deserializer.ReadLength());
            deserializer.Read(out this.nodes, GraphNodeMsg.Deserialize, deserializer.ReadLength());
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.header);
            serializer.WriteLength(this.edges);
            serializer.Write(this.edges);
            serializer.WriteLength(this.nodes);
            serializer.Write(this.nodes);
        }

        public override string ToString()
        {
            return "GraphNavMsg: " +
            "\nheader: " + header.ToString() +
            "\nedges: " + System.String.Join(", ", edges.ToList()) +
            "\nnodes: " + System.String.Join(", ", nodes.ToList());
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
