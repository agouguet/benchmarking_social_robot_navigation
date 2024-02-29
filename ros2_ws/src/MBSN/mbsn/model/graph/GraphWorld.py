import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from time import time
import math
from mbsn.model.interfaces.IWorld import IWorld
from mbsn.model.graph.GraphAction import GraphAction
from mbsn.model.graph.GraphState import GraphState
from scipy.spatial import distance
from itertools import combinations 
import networkx as nx

PENALITY_DISTANCE_GOAL = 3
PENALITY_COLLISION_HUMAN = 50
PENALITY_PROXIMITY_HUMAN =0.1


class GraphWorld(IWorld):
    def __init__(self, networkx_graph, visibility_graph, debug_mode = False, time_limit=1000, numbers_human = None):
        self.graph = networkx_graph
        self.visibility_graph = visibility_graph
        self.time_limit = time_limit
        self.numbers_human = numbers_human
        self.astar_dict = self.astar_calculation()
        self.init(debug_mode)

    # |S| = N^2 * 2^N
    def get_state_model(self):
        S = []
        
        nodes = self.graph.nodes()
        numbers = len(nodes)+1 if self.numbers_human == None else self.numbers_human+1
        humans_presence_possibilities = []

        for count in range(0, numbers): # O(h)
            comb = combinations(nodes, count)   # O(n! / c! / (n-c)!)
            for t in comb:
                humans_presence_possibilities.append(t)

        for n in self.graph.nodes():
            for goal in self.graph.nodes():
                for human_presence in humans_presence_possibilities:
                    S.append(GraphState(n, tuple(sorted(set(human_presence))), goal))
        return S

    def get_action_model(self):
        A = {}
        for s in self.states:
            n = s.robot_node
            actions = [GraphAction(n)]
            for e in self.graph.edges([n]):
                actions.append(GraphAction(e[1]))
            A[s] = actions
        return A

    def get_state_from_pos(self, robot_node, human_node):
        for s in self.states:
            if s.robot_node == robot_node and s.human_node == human_node:
                return s
        return None

    def get_state_prim_model(self):
        S_prim = {}
        for s in self.states:
            S_prim[s] = {}
            for a in self.actions[s]:
                S_prim[s][a] = [GraphState(a._node, s.humans_node, s.goal)]
                for human_node in s.humans_node:
                    neighbor = [i for i in self.graph.neighbors(human_node)]
                    for n in neighbor:
                        occupied_nodes = tuple(sorted(set(s.humans_node + (n,))))
                        if len(occupied_nodes) <= self.numbers_human:
                            S_prim[s][a].append(GraphState(a._node, occupied_nodes, s.goal))
        return S_prim


    def get_reward_function(self):
        R = {}
        for s in self.states:
            R[s] = {}
            for a in self.actions[s]:
                R[s][a] = {}
                for s_prim in self.states_prim[s][a]:
                    R[s][a][s_prim] = - self.distance_cost(s, s_prim) - self.proximity_cost_to_humans(s_prim.robot_node, s_prim.humans_node) - self.proximity_cost_to_humans(s_prim.robot_node, s.humans_node)
                            
        return R

    def distance_cost(self, s, s_prim):
        cost = self.euclidean_distance_between_node(s.robot_node, s_prim.robot_node)
        if s_prim.robot_node in self.visibility_graph[s.goal]:
            cost += PENALITY_DISTANCE_GOAL * self.euclidean_distance_between_node(s_prim.robot_node, s.goal)
        else:
            cost += PENALITY_DISTANCE_GOAL * self.astar_dict[s_prim.robot_node][s.goal]
        
            
        return cost

    def proximity_cost_to_humans(self, robot_node, occupied_nodes):
        if len(occupied_nodes) == 0:
            return 0
        min_distance = math.inf
        for occupied_node in occupied_nodes:
            dist_to_robot = self.astar_dict[robot_node][occupied_node]
            if min_distance > dist_to_robot:
                min_distance = dist_to_robot
        if min_distance != 0: 
            return PENALITY_PROXIMITY_HUMAN * (PENALITY_COLLISION_HUMAN/min_distance)
        else:
            return PENALITY_COLLISION_HUMAN

    def get_transition_model(self):
        transitions = {}
        for s in self.states:
            transitions[s] = {}
            for a in self.actions[s]:
                transitions[s][a] = {}
                for s_prim in self.states_prim[s][a]:
                    transitions[s][a][s_prim] = 1/len(self.states_prim[s][a])
        return transitions

    def get_pos_of_node(self, node):
        return self.graph.nodes(data=True)[node]['pos']

    def euclidean_distance_between_node(self, node1, node2):
        return distance.euclidean(self.get_pos_of_node(node1), self.get_pos_of_node(node2))

    def astar_calculation(self):
        print("Calcul all astar distance ...")
        astardict = {}
        for n in self.graph.nodes():
            astardict[n] = {}
            for m in self.graph.nodes():
                if n != m:
                    path = nx.astar_path(self.graph, n, m)
                    sum_dist = 0
                    for i in range(1, len(path)):
                        sum_dist += self.euclidean_distance_between_node(path[i-1], path[i])
                    astardict[n][m] = sum_dist
                else:
                    astardict[n][m] = 0
        return astardict
