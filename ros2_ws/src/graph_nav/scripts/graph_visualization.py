#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, Point
import visualization_msgs
from simulation_msgs.msg import GraphNav
from simulation_msgs.msg import GraphEdge
from simulation_msgs.msg import GraphNode

import networkx as nx


class GraphVisualization(Node):

    def __init__(self):
        super().__init__('Test_graph_msg')
        self.subscription = self.create_subscription(GraphNav, 'graph', self.listener_callback, 10)
        self.nodes_publisher_ = self.create_publisher(MarkerArray, 'graph/visual/nodes', 10)
        self.edges_publisher_ = self.create_publisher(MarkerArray, 'graph/visual/edges', 10)
        self.i = 0

    def listener_callback(self, msg):
        G = msg
        E = G.edges
        N = G.nodes
        self.publish_nodes(N)
        self.publish_edges(E, N)

    def publish_nodes(self, nodes):
        marker_array = MarkerArray()
        markers = []

        for n in nodes:
            marker = Marker()

            marker.header = Header()
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.header.frame_id = "/map"

            marker.id = n.id
            marker.type = marker.SPHERE     

            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.color.a = 0.5
            marker.scale.x = 0.15
            marker.scale.y = 0.15
            marker.scale.z = 0.15

            p = Pose()
            p.position.x = float(n.x)
            p.position.y = float(n.y)
            p.position.z = 1.0
            marker.pose = p
            markers.append(marker)

        marker_array.markers = markers

        self.nodes_publisher_.publish(marker_array)

    def publish_edges(self, edges, nodes):
        marker_array = MarkerArray()
        markers = []

        for i in range(len(edges)):
            e = edges[i]
            marker = Marker()

            marker.header = Header()
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.header.frame_id = "/map"

            marker.id = i
            marker.type = marker.LINE_STRIP     

            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.1
            marker.scale.x = 0.03
            marker.scale.y = 0.03
            marker.scale.z = 0.03

            
            p1 = Point()
            p1.x = float(nodes[e.id_n1].x)
            p1.y = float(nodes[e.id_n1].y)

            p2 = Point()
            p2.x = float(nodes[e.id_n2].x)
            p2.y = float(nodes[e.id_n2].y)

            points = [p1, p2]

            marker.points = points
            markers.append(marker)

        marker_array.markers = markers

        self.edges_publisher_.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = GraphVisualization()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()