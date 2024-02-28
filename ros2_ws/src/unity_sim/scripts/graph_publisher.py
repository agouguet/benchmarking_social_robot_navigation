#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import pickle
from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, Point
import visualization_msgs
from simulation_msgs.msg import GraphNav
from simulation_msgs.msg import GraphEdge
from simulation_msgs.msg import GraphNode
from simulation_msgs.msg import SceneInfo

import networkx as nx
import os


PATH_TO_SAVE_GRAPH = "/home/adam/ws/my_unity_sim/ros2_ws/src/unity_sim/graphs/"
EXTENSION_FILE = ".pickle"

class GraphPublisher(Node):

    def __init__(self):
        super().__init__('GraphPublisher')
        self.scene_info_subscriber_ = self.create_subscription(SceneInfo, '/social_sim/scene_info', self.scene_info_callback, 10)
        self.graph_subscription = self.create_subscription(GraphNav, 'graph', self.graph_callback, 10)
        self.graph_publisher = self.create_publisher(GraphNav, 'graph', 10)
        timer_period = 10
        self.graph_publisher_timer = self.create_timer(timer_period, self.graph_publish_callback)

        self.scenario = None
        self.G = None

        

    def scene_info_callback(self, msg_scene_info):
        if self.scenario != msg_scene_info.environment.lower():
            self.scenario = msg_scene_info.environment.lower()
            print("Set new scenario...", self.scenario)
            if os.path.isfile(PATH_TO_SAVE_GRAPH + self.scenario + EXTENSION_FILE):
                print("Load graph", self.scenario + EXTENSION_FILE)
                self.G = pickle.load(open(PATH_TO_SAVE_GRAPH + self.scenario + EXTENSION_FILE, 'rb'))

    def graph_callback(self, msg):
        if self.G == None and self.scenario != None:
            self.G = nx.Graph()
            for n in msg.nodes:
                self.G.add_node(n.id, pos=(n.x, n.y))
            for e in msg.edges:
                self.G.add_edge(e.id_n1, e.id_n2)
            
            print("Save graph as ...", PATH_TO_SAVE_GRAPH + self.scenario + EXTENSION_FILE)
            pickle.dump(self.G, open(PATH_TO_SAVE_GRAPH + self.scenario + EXTENSION_FILE, 'wb'))


    def graph_publish_callback(self):
        if self.G != None:
            msg = GraphNav()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            edges = []
            nodes = []

            for n in self.G.nodes():
                node_msg = GraphNode()
                node_msg.id = n
                node_msg.x = float(self.G.nodes(data=True)[n]['pos'][0])
                node_msg.y = float(self.G.nodes(data=True)[n]['pos'][1])
                nodes.append(node_msg)

            for e, w in self.G.edges.items():
                edge = GraphEdge()
                edge.id_n1 = e[0]
                edge.id_n2 = e[1]
                edges.append(edge)

            msg.nodes = nodes
            msg.edges = edges

            print("Publish graph...")
            self.graph_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = GraphPublisher()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()