#!/usr/bin/env python3
import os
import time
import math
import rclpy # type: ignore
from rclpy.node import Node # type: ignore
from abc import ABC, abstractmethod

from std_msgs.msg import Header # type: ignore
from geometry_msgs.msg import PoseStamped # type: ignore
from geometry_msgs.msg import PoseArray # type: ignore
from nav_msgs.msg import Odometry # type: ignore
from agents_msgs.msg import AgentArray # type: ignore
from graph_msgs.msg import GraphNav # type: ignore
from simulation_msgs.msg import SceneInfo # type: ignore

# from mbsn.model.graph.GraphWorld import GraphWorld
from mbsn.model.graph.GraphState import GraphState

from ament_index_python.packages import get_package_prefix # type: ignore

import networkx as nx
from scipy.spatial import distance
	

class EnvironmentInterpreter(Node):

    def __init__(self, name_node='EnvironmentInterpreter'):
        super().__init__(name_node)
        
        self.scenario = None
        self.goal = None
        self.current_state = None
        self.humans = []

        # Agents
        self.robot_position_subscription_ = self.create_subscription(Odometry, 'robot_odom', self.robot_odom_callback, 10)
        self.humans_position_subscription_ = self.create_subscription(AgentArray, 'social_sim/agents', self.humans_callback, 10)

        # Goal
        self.goal_subscription_ = self.create_subscription(PoseStamped, 'global_goal', self.goal_callback, 10)
        self.goal_publisher_ = self.create_publisher(PoseStamped, 'goal_pose', 10)

        # Scene
        self.scene_info_subscriber_ = self.create_subscription(SceneInfo, '/social_sim/scene_info', self.scene_info_callback, 10)

    # INIT

    def scene_info_callback(self, msg_scene_info):
        if self.scenario != msg_scene_info.environment.lower():
            self.scenario = msg_scene_info.environment.lower()

    def goal_callback(self, msg_goal):
        pos = msg_goal.pose.position
        self.goal = pos

    # RUN
    @abstractmethod
    def robot_odom_callback(self, msg_odom):
        pass

    @abstractmethod
    def humans_position_callback(self, msg_position):
        pass

    @abstractmethod
    def humans_callback(self, msg_agents):
        pass

    @abstractmethod
    def update(self, new_state):
        pass