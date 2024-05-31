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

from mbsn.model.graph.MBSN import MBSN
from mbsn.solver.MCTS import MBSNAgentMCTS, MBSNAgentNode
from mbsn.solver.qtable import QTable
from mbsn.solver.multi_armed_bandit.ucb import UpperConfidenceBounds
from mbsn.utils.graph_visualisation import GraphVisualisation
from mbsn.model.graph.GraphAction import GraphAction
# from mbsn.model.graph.GraphWorld import GraphWorld
from mbsn.model.graph.GraphState import GraphState
from mbsn.solver.ValueIteration import ValueIteration
from mbsn.utils.visibility_graph import VisibilityGraph

from mbsn.interpreter.interpreter import Interpreter

from ament_index_python.packages import get_package_prefix

import numpy as np
import networkx as nx
from scipy.spatial import distance
	
import json
import pickle

EXTENSION_FILE = ".pickle"

class MDPBasedSocialNavigationMCTS(Interpreter):

    def __init__(self, publish_graph = True, save_policy = True):
        super().__init__(publish_graph, save_policy)
        self.mcts = None

    def update(self, new_state):
        if self.graph != None and self.scenario != None:
            if self.mcts == None:
                visibility_graph = VisibilityGraph(self.path_to_maps + self.scenario + "/", self.graph).visibility_graph
                self.mdp = MBSN(self.graph, visibility_graph, 4)
                qfunction = QTable()
                self.mcts = MBSNAgentMCTS(self.mdp, qfunction, UpperConfidenceBounds())


            node = self.mcts.mcts(new_state, timeout=2)
            action, value = node.get_value()
            print("\n----------------")
            print("Update:")
            print("     Robot Node:", new_state.robot_node)
            print("     Humans Node:", np.where(np.array(new_state.humans_node) == 1)[0])
            print("     From possible actions:", self.mdp.get_actions(new_state))
            print("     Action:", action, value)
            
            self.publish_goal(action)

    # def calcul_policy(self):
    #     path_policies_values = self.path_to_policies + self.scenario + "_values" + EXTENSION_FILE

    #     if os.path.isfile(path_policies_values):
    #         print("Load policy...", self.scenario + "_values" + EXTENSION_FILE)
    #         self.policy = pickle.load(open(path_policies_values, 'rb'))
    #     else:
    #         visibility_graph = VisibilityGraph(self.path_to_maps + self.scenario + "/", self.graph)

    #         self.problem = GraphWorld(self.graph, visibility_graph.visibility_graph, self.robot_goal, numbers_human = 4, debug_mode=True)
    #         print("Value Iteration...")
    #         self.solver = ValueIteration(self.problem, gamma=0.9)
    #         self.solver.train()
    #         self.policy = self.solver.full_values
    #         if self.save_policy:
    #             print("Save policy as ...", self.path_to_policies + self.scenario + EXTENSION_FILE)
    #             pickle.dump(self.policy, open(self.path_to_policies + self.scenario + EXTENSION_FILE, 'wb'))
    #             print("Save policy full values as ...", self.path_to_policies + self.scenario + "_values" + EXTENSION_FILE)
    #             pickle.dump(self.solver.full_values, open(self.path_to_policies + self.scenario + "_values" + EXTENSION_FILE, 'wb'))
    #             print("Save astar dict as ...", self.path_to_policies + self.scenario + "_astar" + EXTENSION_FILE)
    #             pickle.dump(self.problem.astar_dict, open(self.path_to_policies + self.scenario + "_astar" + EXTENSION_FILE, 'wb'))

    #     self.robot_node = 0
    #     self.humans_position = []
    #     self.human_trajectories = []
    #     self.robot_goal = None

    #     self.current_state = (self.robot_node, self.robot_goal, ())

    
    def euclidean_distance_from_node(self, position_robot, node):
        return distance.euclidean((position_robot.x, position_robot.y), self.graph.nodes(data=True)[node]['pos'])

def main(args=None):
    rclpy.init(args=args)

    mbsn_node = MDPBasedSocialNavigationMCTS(save_policy=True)

    rclpy.spin(mbsn_node)

    mbsn_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()