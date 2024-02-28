using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using SEAN;
using RosMessageTypes.Std;

public class GraphCreator : MonoBehaviour
{

    struct Edge {
        public GameObject n1;
        public GameObject n2;

        public Edge(GameObject n1, GameObject n2) : this()
        {
            this.n1 = n1;
            this.n2 = n2;
        }
    }

    public string topicName = "/graph";

    // Ros connection and Ros Messages
    ROSConnection ros;

    private GameObject graph;

    // Start is called before the first frame update
    void Start()
    {
        // Use ROSTCP connector
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<RosMessageTypes.Simulation.GraphNavMsg>(topicName);

        graph = GameObject.Find("CreateGraph/NavGraph");

        publish();
    }

    void publish()
    {
        RosMessageTypes.Simulation.GraphNavMsg message = new RosMessageTypes.Simulation.GraphNavMsg();

        // Header
        message.header.frame_id = "";
        message.header.stamp = SEAN.SEAN.instance.clock.LastPublishedTime();

        List<GameObject> nodes = new List<GameObject>();
        List<Edge> edges = new List<Edge>();

        foreach(Transform node_tf in graph.transform)
        {
            GameObject node = node_tf.gameObject;
            nodes.Add(node);

            List<GameObject> neighbor = node.GetComponent<NavNode>().neighbor;
            foreach(GameObject n in neighbor)
            {
                if(!(edges.Contains(new Edge(node, n)) || edges.Contains(new Edge(n, node))))
                    edges.Add(new Edge(node, n));
            }

        }

        // Nodes
        message.nodes = new RosMessageTypes.Simulation.GraphNodeMsg[nodes.Count];
        int i = 0;
        foreach(GameObject node in nodes)
        {
            RosMessageTypes.Simulation.GraphNodeMsg nodeMsg = new RosMessageTypes.Simulation.GraphNodeMsg();
            nodeMsg.id = Convert.ToUInt64(node.name.Split(" ")[1]);
            nodeMsg.x = node.transform.position.z;
            nodeMsg.y = -node.transform.position.x;
            message.nodes[i++] = nodeMsg;
        }

        // Edges
        message.edges = new RosMessageTypes.Simulation.GraphEdgeMsg[edges.Count];
        int j = 0;
        foreach(Edge edge in edges)
        {
            RosMessageTypes.Simulation.GraphEdgeMsg edgeMsg = new RosMessageTypes.Simulation.GraphEdgeMsg();
            edgeMsg.id_n1 = Convert.ToUInt64(edge.n1.name.Split(" ")[1]);
            edgeMsg.id_n2 = Convert.ToUInt64(edge.n2.name.Split(" ")[1]);
            message.edges[j++] = edgeMsg;
        }

        ros.Publish(topicName, message);
    }
}
