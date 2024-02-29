#!/usr/bin/env python3
import os
import time
import rclpy
from rclpy.node import Node

from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry
from agents_msgs.msg import AgentTrajectories
from graph_msgs.msg import GraphNav
from simulation_msgs.msg import SceneInfo

from mbsn.model.graph.GraphWorld import GraphWorld
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

class MDPBasedSocialNavigation(Node):

    def __init__(self, publish_graph = True, save_policy = True):
        super().__init__('GraphNav')
        self.graph = None
        self.problem = None
        self.policy = None
        self.robot_goal = None
        self.scenario = None
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

    def scene_info_callback(self, msg_scene_info):
        if self.scenario != msg_scene_info.environment.lower():
            self.scenario = msg_scene_info.environment.lower()
            print("Set new scenario...", self.scenario)

    def graph_callback(self, msg_graph):
        if self.graph == None and self.scenario != None:
            print("Creation MDP...")
            self.graph = nx.Graph()
            for n in msg_graph.nodes:
                self.graph.add_node(n.id, pos=(n.x, n.y))
            for e in msg_graph.edges:
                self.graph.add_edge(e.id_n1, e.id_n2)

            path_policies_values = self.path_to_policies + self.scenario + "_values" + EXTENSION_FILE

            # if os.path.isfile(PATH_TO_SAVE_POLICY + self.scenario + EXTENSION_FILE):
            #     print("Load policy...", self.scenario + EXTENSION_FILE)
            #     self.policy = pickle.load(open(PATH_TO_SAVE_POLICY + self.scenario + EXTENSION_FILE, 'rb'))
            if os.path.isfile(path_policies_values):
                print("Load policy...", self.scenario + "_values" + EXTENSION_FILE)
                self.policy = pickle.load(open(path_policies_values, 'rb'))
            else:
                visibility_graph = VisibilityGraph(self.path_to_maps + self.scenario + "/", self.graph)

                self.problem = GraphWorld(self.graph, visibility_graph.visibility_graph, numbers_human = 3, debug_mode=True)
                print("Value Iteration...")
                self.solver = ValueIteration(self.problem, gamma=0.2)
                self.solver.train()
                self.policy = self.solver.full_values
                if self.save_policy:
                    print("Save policy as ...", self.path_to_policies + self.scenario + EXTENSION_FILE)
                    pickle.dump(self.policy, open(self.path_to_policies + self.scenario + EXTENSION_FILE, 'wb'))
                    print("Save policy full values as ...", self.path_to_policies + self.scenario + "_values" + EXTENSION_FILE)
                    pickle.dump(self.solver.full_values, open(self.path_to_policies + self.scenario + "_values" + EXTENSION_FILE, 'wb'))
                    print("Save astar dict as ...", self.path_to_policies + self.scenario + "_astar" + EXTENSION_FILE)
                    pickle.dump(self.problem.astar_dict, open(self.path_to_policies + self.scenario + "_astar" + EXTENSION_FILE, 'wb'))

            self.robot_state = 0
            self.humans_position = []
            self.human_trajectories = []
            self.robot_goal = None

            self.current_state = (self.robot_state, (), self.robot_goal)

    def euclidean_distance_from_node(self, position_robot, node):
        return distance.euclidean((position_robot.x, position_robot.y), self.graph.nodes(data=True)[node]['pos'])


    def get_current_state_robot(self, position):
        state = self.robot_state
        for n, p in self.graph.nodes.items():
            if(self.euclidean_distance_from_node(position, n) < self.euclidean_distance_from_node(position, state)):
                state = n
        if self.euclidean_distance_from_node(position, state) < DISTANCE_LIMIT_ROBOT_NEAREST_NODE:
                return state
        return self.robot_state

    def get_current_state_humans(self, human_position, traj_positions):
        state = ()
        for pos in human_position:
            state_of_pos = 0
            for n, p in self.graph.nodes.items():
                if(self.euclidean_distance_from_node(pos, n) < self.euclidean_distance_from_node(pos, state_of_pos)):
                    state_of_pos = n
            if self.euclidean_distance_from_node(pos, state_of_pos) < DISTANCE_LIMIT_OCCUPIED_NODE:
                state = state + (state_of_pos,)

        for pos in traj_positions:
            state_of_pos = 0
            for n, p in self.graph.nodes.items():
                if(self.euclidean_distance_from_node(pos, n) < self.euclidean_distance_from_node(pos, state_of_pos)):
                    state_of_pos = n
            if self.euclidean_distance_from_node(pos, state_of_pos) < DISTANCE_LIMIT_OCCUPIED_NODE_TRAJECTORIES:
                state = state + (state_of_pos,)
        return state

    def robot_odom_callback(self, msg_odom):
        if self.robot_goal != None:
            robot_position = msg_odom.pose.pose.position
            self.robot_state = self.get_current_state_robot(robot_position)
            occupied_node = self.get_current_state_humans([pos[0] for pos in self.humans_position], [traj[0] for traj in self.human_trajectories])
            occupied_node = tuple(sorted(set(occupied_node))) # remove duplicates and sort

            current_state = (self.robot_state, occupied_node, self.robot_goal)

            if(current_state != self.current_state):
                print("NEW STATE:", self.current_state, "->", current_state)
                self.current_state = current_state
                action_values = self.policy[GraphState(self.robot_state, occupied_node, self.robot_goal)]
                action = max(action_values, key=action_values.get)
                print("NEW GOAL:", action._node)
                self.publish_goal(action)

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

    def goal_callback(self, msg_goal):
        pos = msg_goal.pose.position
        if self.policy != None:
            for n, p in self.graph.nodes.items():
                if(self.robot_goal == None or self.euclidean_distance_from_node(pos, n) < self.euclidean_distance_from_node(pos, self.robot_goal)):
                    self.robot_goal = n

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

def main(args=None):
    rclpy.init(args=args)

    mbsn_node = MDPBasedSocialNavigation(save_policy=True)

    rclpy.spin(mbsn_node)

    mbsn_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()