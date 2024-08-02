from collections import defaultdict

from shapely import Point
from mbsn.model.polygon.MDP import MDP
from scipy.spatial import distance
import networkx as nx
import numpy as np
import math

PENALITY_DISTANCE_GOAL = 1
PENALITY_COLLISION_HUMAN = 1000
PENALITY_PROXIMITY_HUMAN = 0.1
REWARD_GOAL = 100
PENALITY_WAIT = 1

class NavPolygonMDP(MDP):

    def __init__(
        self,
        polygons,
        goal=None,
        # number_of_detected_human = None,
        distance_factor=1.0,
        social_factor=1.0, 
        discount_factor = 0.99
    ):
        if isinstance(polygons, dict):
            self.polygons = polygons
        else:
            self.polygons = {poly.id:poly for poly in polygons}

        self.goal = goal

        # self.number_of_detected_human = number_of_detected_human
        self.discount_factor = discount_factor
        self._social_factor = social_factor
        self._distance_factor = distance_factor

    """Return all states of this MDP"""
    def get_states(self):
        for id in self.polygons.keys():
            yield id

    """ Return all actions with non-zero probability from this state """
    def get_actions(self, state):
        actions = ["find_goal"] + [neighbor.id for neighbor in self.polygons[state].neighbors if neighbor.id in self.polygons]
        return actions

    def get_next_states(self, state, action):
        return [action] if action != "find_goal" else [state]

    """ Return all non-zero probability transitions for this action
        from this state, as a list of (state, probability) pairs
    """
    def get_transitions(self, state, action):
        next_states = self.get_next_states(state, action)
        list_state_probability = [(nx, 1.0) for nx in next_states]
        # for next_state in next_states:
        #     list_state_probability.append((next_state, 1/len(next_states)))
        return list_state_probability

    """ Return the reward for transitioning from state to
        nextState via action
    """
    def get_reward(self, state, action, next_state):
        return REWARD_GOAL if self.is_terminal(next_state) else 0

    """ Return true if and only if state is a terminal state of this MDP """
    def is_terminal(self, state):
        if len(self.get_actions(state)) == 0:
            return True
        return self.polygons[state].intersects(self.goal.buffer(0.05))

    """ Return the discount factor for this MDP """
    def get_discount_factor(self):
        return self.discount_factor
    
    def get_state_from_continuous_position(self, position):
        if not isinstance(position, Point):
            position = Point(position)
        for id, polygon in self.polygons.items():
            if polygon.buffer(0.2).contains(position):
                return id
        return None

    def distance_cost(self, s, next_state):
        # return self.astar_dict[s.robot_node][next_state.robot_node]
        # cost = self.euclidean_distance_between_node(s.robot_node, next_state.robot_node)
        cost = self.astar_dict[s.robot_node][next_state.robot_node]
        # cost += PENALITY_DISTANCE_GOAL * self.astar_dict[next_state.robot_node][s.goal]
        return cost

    def proximity_cost_to_humans(self, robot_node, occupied_nodes):
        if not np.any(occupied_nodes,  where=1):
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

    def get_pos_of_node(self, node):
        return self.graph.nodes(data=True)[node]['pos']

    def euclidean_distance_between_node(self, node1, node2):
        return distance.euclidean(self.get_pos_of_node(node1), self.get_pos_of_node(node2))

    def astar_calculation(self):
        def dist(u, v):
            (x1, y1) = self.graph.nodes(data=True)[u]['pos']
            (x2, y2) = self.graph.nodes(data=True)[v]['pos']
            return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5

        astardict = {}
        for n in self.graph.nodes():
            astardict[n] = {}
            for m in self.graph.nodes():
                if n != m:
                    path = nx.astar_path(self.graph, n, m, heuristic=dist)
                    sum_dist = 0
                    for i in range(1, len(path)):
                        sum_dist += self.euclidean_distance_between_node(path[i-1], path[i])
                    astardict[n][m] = sum_dist
                else:
                    astardict[n][m] = 0
        return astardict