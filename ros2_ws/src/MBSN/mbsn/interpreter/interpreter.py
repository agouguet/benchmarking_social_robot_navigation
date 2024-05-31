#!/usr/bin/env python3
import os
import time
import math
import rclpy
from rclpy.node import Node
from abc import ABC, abstractmethod

from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry
from agents_msgs.msg import AgentTrajectories
from graph_msgs.msg import GraphNav
from simulation_msgs.msg import SceneInfo

# from mbsn.model.graph.GraphWorld import GraphWorld
from mbsn.model.graph.GraphState import GraphState
from mbsn.solver.ValueIteration import ValueIteration
from mbsn.utils.visibility_graph import VisibilityGraph

from ament_index_python.packages import get_package_prefix

import networkx as nx
from scipy.spatial import distance
	
import json
import pickle

EXTENSION_FILE = ".pickle"

DISTANCE_LIMIT_OCCUPIED_NODE = 1.5
DISTANCE_LIMIT_OCCUPIED_NODE_TRAJECTORIES = 1.0
DISTANCE_LIMIT_ROBOT_NEAREST_NODE = 0.6
TIME_OCCUPANCY = 10

# DISTANCE_LIMIT_OCCUPIED_NODE = 0.5
# DISTANCE_LIMIT_OCCUPIED_NODE_TRAJECTORIES = 0.2
# DISTANCE_LIMIT_ROBOT_NEAREST_NODE = 0.1
# TIME_OCCUPANCY = 1

class Interpreter(Node):

    def __init__(self, publish_graph = True, save_policy = True):
        super().__init__('MBSN')
        self.graph = None
        self.problem = None
        self.policy = None
        self.robot_goal = None
        self.robot_node = None
        self.scenario = None
        self.current_state = None
        self.humans_position = []
        self.human_trajectories = []

        self.save_policy = save_policy

        # Paths
        self.path_to_policies = get_package_prefix('unity_sim') + "/../../src/unity_sim/policies/"
        self.path_to_maps = get_package_prefix('unity_sim') + "/../../src/unity_sim/maps/"

        # Graph
        self.graph_subscription = self.create_subscription(GraphNav, 'graph', self.graph_callback, 10)

        # Agents
        self.robot_position_subscription_ = self.create_subscription(Odometry, 'odom', self.robot_odom_callback, 10)
        self.humans_position_subscription_ = self.create_subscription(PoseArray, 'social_sim/agent_positions', self.humans_position_callback, 10)
        self.humans_trajectories_subscription_ = self.create_subscription(AgentTrajectories, 'agent/trajectories', self.humans_trajectories_callback, 10)

        # Goal
        self.goal_subscription_ = self.create_subscription(PoseStamped, 'global_goal', self.goal_callback, 10)
        self.goal_publisher_ = self.create_publisher(PoseStamped, 'goal_pose', 10)

        # Scene
        self.scene_info_subscriber_ = self.create_subscription(SceneInfo, '/social_sim/scene_info', self.scene_info_callback, 10)

    # INIT

    def scene_info_callback(self, msg_scene_info):
        if self.scenario != msg_scene_info.environment.lower():
            self.scenario = msg_scene_info.environment.lower()
            print("Get scenario:", self.scenario)

    def graph_callback(self, msg_graph):
        if self.graph == None:
            print("Creation Graph...")
            self.graph = nx.Graph()
            for n in msg_graph.nodes:
                self.graph.add_node(n.id, pos=(n.x, n.y))
            for e in msg_graph.edges:
                self.graph.add_edge(e.id_n1, e.id_n2)

    def goal_callback(self, msg_goal):
        pos = msg_goal.pose.position
        if self.scenario != None and self.graph != None:
            for n, p in self.graph.nodes.items():
                if(self.robot_goal == None or self.euclidean_distance_from_node(pos, n) < self.euclidean_distance_from_node(pos, self.robot_goal)):
                    self.robot_goal = n
                    print("Get New Global Goal:", self.robot_goal)

    # RUN

    def robot_odom_callback(self, msg_odom):
        if self.robot_goal != None:
            self.robot_node = self.get_closest_node_to_pos(msg_odom.pose.pose.position, distance_minimum=DISTANCE_LIMIT_ROBOT_NEAREST_NODE)
            if self.robot_node == None:
                return 
            occupied_node = self.get_current_state_humans([pos[0] for pos in self.humans_position], [traj[0] for traj in self.human_trajectories])
            new_state = GraphState(self.robot_node, self.robot_goal, occupied_node)

            if self.current_state == None or new_state != self.current_state:
                self.current_state = new_state
                self.update(new_state)

    def humans_position_callback(self, msg_position):
        for point in msg_position.poses:
            self.humans_position.append([point.position, time.time()])
        tmp = []
        for human_pos in self.humans_position:
            if human_pos[1] + TIME_OCCUPANCY >= time.time():
                tmp.append(human_pos)
        self.humans_position = tmp

    def humans_trajectories_callback(self, msg_agent_trajectories):
        self.human_trajectories = []
        for trajectory in msg_agent_trajectories.trajectories:
            for pose in trajectory.poses:
                self.human_trajectories.append([pose, time.time()])
        tmp = []
        for human_traj in self.human_trajectories:
            if human_traj[1] + TIME_OCCUPANCY >= time.time():
                tmp.append(human_traj)
        self.human_trajectories = tmp


    @abstractmethod
    def update(self, new_state):
        pass


    def get_closest_node_to_pos(self, position, distance_minimum = math.inf):
        state = 0
        for n, p in self.graph.nodes.items():
            if(self.euclidean_distance_from_node(position, n) < self.euclidean_distance_from_node(position, state)):
                state = n
        if self.euclidean_distance_from_node(position, state) < distance_minimum:
                return state
        return None

    def get_current_state_humans(self, human_position, traj_positions):
        state = ()
        for pos in human_position:
            closest_node = self.get_closest_node_to_pos(pos, distance_minimum=DISTANCE_LIMIT_OCCUPIED_NODE)
            if closest_node != None:
                state = state + (closest_node,)
        for pos in traj_positions:
            closest_node = self.get_closest_node_to_pos(pos, distance_minimum=DISTANCE_LIMIT_OCCUPIED_NODE_TRAJECTORIES)
            if closest_node != None:
                state = state + (closest_node,)

        occupied_node = [0 for n in self.graph.nodes]
        for i in state:
            occupied_node[i] = 1
        return occupied_node

    def publish_goal(self, action):
        n = action._node
        x = self.graph.nodes(data=True)[n]['pos'][0]
        y = self.graph.nodes(data=True)[n]['pos'][1]

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)

        self.goal_publisher_.publish(msg)
    
    def euclidean_distance_from_node(self, position_robot, node):
        return distance.euclidean((position_robot.x, position_robot.y), self.graph.nodes(data=True)[node]['pos'])

def main(args=None):
    rclpy.init(args=args)

    mbsn_node = Interpreter(save_policy=True)

    rclpy.spin(mbsn_node)

    mbsn_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()