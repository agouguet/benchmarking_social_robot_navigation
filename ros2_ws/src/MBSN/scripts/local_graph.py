#!/usr/bin/env python3
import os
import time
import math
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

import tf2_geometry_msgs.tf2_geometry_msgs
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf2_ros import LookupException

import networkx as nx
from scipy.spatial import distance
	
import json
import pickle

def dist(p1, p2):
    return (p1[0]-p2[0])**2 + (p1[1]-p2[1])**2

def merge(points, distance):
    ret = []
    n = len(points)
    taken = [False] * n
    for i in range(n):
        if not taken[i]:
            count = 1
            point = [points[i][0], points[i][1]]
            taken[i] = True
            for j in range(i+1, n):
                if dist(points[i], points[j]) < distance:
                    point[0] += points[j][0]
                    point[1] += points[j][1]
                    count+=1
                    taken[j] = True
            point[0] /= count
            point[1] /= count
            ret.append((int(point[0]), int(point[1])))
    return ret

def merge(points, distance=0.1):
    ret = []
    n = len(points)
    i = 0
    while i < n:
        if dist(points[i-1], points[i]) < distance:
            x = (points[i-1][0] + points[i][0]) / 2
            y = (points[i-1][1] + points[i][1]) / 2
            ret.append((x, y))
            i += 1
        else:
            ret.append(points[i-1])
        i+=1
    return ret

def infinite_merge(points, distance=0.01):
    if distance == 0:
        return points
    previous = points
    while 1:
        merged = merge(previous, distance)
        if merged == previous:
            return merged
        previous = merged


class LocalGraph(Node):

    def __init__(self):
        super().__init__("local_graph")
        self.robot_position_subscription_ = self.create_subscription(Odometry, 'odom', self.robot_odom_callback, 10)
        # self.laser_subscription_ = self.create_subscription(LaserScan, 'scan', self.laser_callback, 10)
        self.graph_publisher = self.create_publisher(GraphNav, 'localgraph', 10)
        self.nodes_publisher_ = self.create_publisher(MarkerArray, 'localgraph/visual/nodes', 10)
        self.edges_publisher_ = self.create_publisher(MarkerArray, 'localgraph/visual/edges', 10)
        self.robot_pos = None
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

    # def laser_callback(self, msg_scan):
    #     if self.robot_pos != None:

    #         trans = self._tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())

    #         ranges = msg_scan.ranges
    #         i = 0
    #         points = []
    #         for l in ranges:
    #             ang = math.radians(i)
    #             target_x = math.sin(ang) * l
    #             target_y = math.cos(ang) * l
    #             if (target_y, target_x) != (0.0, 0.0):
    #                 points.append((target_y, target_x))
    #             i += 1


    #         # print(dist(points[0], points[10]))
    #         # points = infinite_merge(points)
    #         points = merge(points)
    #         points = merge(points)
    #         points = merge(points)
    #         points = np.array(points)
    #         print(points)
    #         points = np.concatenate((points, points/2), axis=0)

    #         print(points)

    #         G = nx.Graph()
    #         i = 0
    #         for p in points:
    #             G.add_node(i, pos=(p[0], p[1]))
    #             i += 1

    #         msg = GraphNav()
    #         msg.header = Header()
    #         msg.header.stamp = self.get_clock().now().to_msg()
    #         edges = []
    #         nodes = []

    #         for n in G.nodes():
    #             p = Pose()
    #             p.position.x = float(G.nodes(data=True)[n]['pos'][0])
    #             p.position.y = float(G.nodes(data=True)[n]['pos'][1])
    #             p_transformed = tf2_geometry_msgs.do_transform_pose(p, trans)

    #             node_msg = GraphNode()
    #             node_msg.id = n
    #             node_msg.x = p_transformed.position.x
    #             node_msg.y = p_transformed.position.y
    #             nodes.append(node_msg)
                

    #         for e, w in G.edges.items():
    #             edge = GraphEdge()
    #             edge.id_n1 = e[0]
    #             edge.id_n2 = e[1]
    #             edges.append(edge)


    #         # test = PoseStamped()
    #         # test.header.frame_id = "base_link"
    #         # test.pose.position.x = 2.0
    #         # test.pose.position.y = 2.0

    #         # node_msg = GraphNode()
    #         # node_msg.id = 1000
    #         # p_transformed = self._tf_buffer.transform(test, "map")
    #         # node_msg.x = p_transformed.pose.position.x
    #         # node_msg.y = p_transformed.pose.position.y
    #         # nodes.append(node_msg)



    #         msg.nodes = nodes
    #         msg.edges = edges

    #         self.graph_publisher.publish(msg)
    #         self.publish_nodes(nodes)
    #         self.publish_edges(edges, nodes)

    def robot_odom_callback(self, msg_odom):
        self.robot_pos = msg_odom.pose.pose.position

        m, n = (5, 5)

        G = nx.grid_2d_graph(m, n, periodic=False)
        G.add_edges_from([
            ((x, y), (x+1, y+1))
            for x in range(m-1)
            for y in range(n-1)
        ] + [
            ((x+1, y), (x, y+1))
            for x in range(m-1)
            for y in range(n-1)
        ])


        div = 3
        c = nx.center(G)[0]
        G = nx.relabel_nodes(G, {(x,y):(x-c[0],y-c[1]) for x,y in G.nodes()})
        G = nx.relabel_nodes(G, {(x,y):(x/div,y/div) for x,y in G.nodes()})
        G = nx.relabel_nodes(G, {(x,y):(x+self.robot_pos.x,y+self.robot_pos.y) for x,y in G.nodes()})
        nx.set_node_attributes(G, {(x,y):(x,y) for x,y in G.nodes()}, 'pos')

        mapping = {}
        i = 0
        for x,y in G.nodes():
            mapping[(x, y)] = i
            i += 1

        msg = GraphNav()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        edges = []
        nodes = []

        
        for n in G.nodes():
            node_msg = GraphNode()
            node_msg.id = mapping[n]
            node_msg.x = float(G.nodes(data=True)[n]['pos'][0])
            node_msg.y = float(G.nodes(data=True)[n]['pos'][1])
            nodes.append(node_msg)
            

        for e, w in G.edges.items():
            edge = GraphEdge()
            edge.id_n1 = mapping[e[0]]
            edge.id_n2 = mapping[e[1]]
            edges.append(edge)

        msg.nodes = nodes
        msg.edges = edges

        # print("Publish graph...")
        self.graph_publisher.publish(msg)
        self.publish_nodes(nodes)
        self.publish_edges(edges, nodes)

    def publish_nodes(self, nodes):
        marker_array = MarkerArray()
        markers = []

        for n in nodes:
            marker = Marker()

            marker.header = Header()
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.header.frame_id = "map"

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
            marker.header.frame_id = "map"

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

    mbsn_node = LocalGraph()

    rclpy.spin(mbsn_node)

    mbsn_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()