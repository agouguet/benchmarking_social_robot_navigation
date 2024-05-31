#!/usr/bin/env python3

import os, sys, math, random, time, copy, yaml, cv2
import rclpy
import numpy as np
from rclpy.node import Node

from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry
from agents_msgs.msg import AgentTrajectories
from graph_msgs.msg import GraphNav
from simulation_msgs.msg import SceneInfo
from sensor_msgs.msg import LaserScan

from mbsn.model.graph.MBSN import MBSN
from mbsn.solver.MCTS import MBSNAgentMCTS, MBSNAgentNode
from mbsn.solver.qtable import QTable
from mbsn.solver.multi_armed_bandit.ucb import UpperConfidenceBounds
from mbsn.utils.graph_visualisation import GraphVisualisation
# from mbsn.model.graph.GraphAction import GraphAction
# from mbsn.model.graph.GraphWorld import GraphWorld
# from mbsn.model.graph.GraphState import GraphState
from mbsn.solver.ValueIteration import ValueIteration
from mbsn.utils.visibility_graph import VisibilityGraph

from mbsn.interpreter.interpreter import Interpreter

from ament_index_python.packages import get_package_prefix

from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, Point, PointStamped
from graph_msgs.msg import GraphNav, GraphNode, GraphEdge
from nav_msgs.msg import OccupancyGrid

import tf2_geometry_msgs.tf2_geometry_msgs
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf2_ros import LookupException

import networkx as nx
from scipy.spatial import distance
	
import json
import pickle

EXTENSION_FILE_GRAPH = ".pickle"
EXTENSION_FILE_MAP = ".yaml"

AUTOMATIC = True

class Graph(Node):

    def __init__(self):
        super().__init__("graph")
        self.scenario = None
        self.G = None
        self.path_to_graphs = get_package_prefix('mbsn') + "/../../src/MBSN/graphs/"
        self.path_to_maps = get_package_prefix('unity_sim') + "/../../src/unity_sim/maps/"

        # PUBLISHER/SUBSCRIBER
        self.scene_info_subscriber_ = self.create_subscription(SceneInfo, '/social_sim/scene_info', self.scene_info_callback, 10)
        # self.map_subscription_ = self.create_subscription(OccupancyGrid, 'map', self.map_callback, 10)
        self.graph_subscription = self.create_subscription(GraphNav, 'graph', self.graph_callback, 10)
        self.graph_publisher = self.create_publisher(GraphNav, 'graph', 10)
        timer_period = 10
        self.graph_publisher_timer = self.create_timer(timer_period, self.graph_publish_callback)

        self.robot_pos = None
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        
    def scene_info_callback(self, msg_scene_info):
        if self.scenario != msg_scene_info.environment.lower():
            self.scenario = msg_scene_info.environment.lower()
            print("Set new scenario...", self.scenario)
            if not AUTOMATIC and os.path.isfile(self.path_to_graphs + self.scenario + EXTENSION_FILE_GRAPH):
                print("Load graph", self.scenario + EXTENSION_FILE_GRAPH)
                self.G = pickle.load(open(self.path_to_graphs + self.scenario + EXTENSION_FILE_GRAPH, 'rb'))
            else:
                if os.path.isfile(self.path_to_maps + self.scenario + "/map" + EXTENSION_FILE_MAP):
                    map_config = yaml.safe_load(open(self.path_to_maps + self.scenario + "/map" + EXTENSION_FILE_MAP))
                    image_map = cv2.imread(self.path_to_maps + self.scenario + "/" + map_config["image"])

    def graph_callback(self, msg):
        if self.G == None and self.scenario != None:
            self.G = nx.Graph()
            for n in msg.nodes:
                self.G.add_node(n.id, pos=(n.x, n.y))
            for e in msg.edges:
                self.G.add_edge(e.id_n1, e.id_n2)
            
            print("Save graph as ...", self.path_to_graphs + self.scenario + EXTENSION_FILE_GRAPH)
            pickle.dump(self.G, open(self.path_to_graphs + self.scenario + EXTENSION_FILE_GRAPH, 'wb'))

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

    mbsn_node = Graph()

    rclpy.spin(mbsn_node)

    mbsn_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()