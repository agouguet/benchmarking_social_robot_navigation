from collections import defaultdict

from shapely import MultiPolygon, Point, coverage_union
from mbsn.model.polygon.MDP import MDP
from mbsn.model.polygon.Action import Action
from mbsn.model.polygon.State import State
from scipy.spatial import distance
from itertools import combinations 
import networkx as nx
import itertools
import numpy as np
import math
import random

PENALITY_DISTANCE_GOAL = 1
PENALITY_COLLISION_HUMAN = 1000
PENALITY_FUTURE_COLLISION_HUMAN = 100
PENALITY_PROXIMITY_HUMAN = 0.1
REWARD_GOAL = 1000
PENALITY_WAIT = 10

class NavRoomByVisibilityWithHumanMDP(MDP):

    def __init__(
        self,
        map_polygon,
        robot_position,
        human_trajectory_prediction_function=None,
        goal=None,
        global_goal=None,
        number_of_detected_human = 0,
        distance_factor=1.0,
        social_factor=1.0, 
        discount_factor = 0.99
    ):
        self.polygons = defaultdict(list)

        _, self.visibility_polygon = map_polygon.visibility_polygon.build(robot_position.x, robot_position.y, range=5.0)

        visible_rooms=[]

        # for id, room in map_polygon.rooms.items():
        #     if room.polygon.intersects(self.visibility_polygon):
        #         visible_rooms.append(room)

        # for room in visible_rooms:
        #     cells = room.get_cells_from_grid(map_polygon.test_grid)
        #     for cell in cells:
        #         if cell.polygon.intersects(self.visibility_polygon):
        #             self.polygons[cell.id].append(cell)

        for id, cell in map_polygon.test_grid.items():
            if cell.polygon.intersects(self.visibility_polygon):
                self.polygons[cell.id].append(cell)

        self.goal = goal
        self.global_goal = global_goal

        self.number_of_detected_human = number_of_detected_human
        self.discount_factor = discount_factor
        self._social_factor = social_factor
        self._distance_factor = distance_factor
        self.human_trajectory_prediction_function = human_trajectory_prediction_function

        # print(map_polygon.grid[113].neighbors)


    """Return all states of this MDP"""
    def get_states(self):
        for id in self.polygons.keys():
            for occupied_polygon in itertools.product([0, 1], repeat=len(self.polygons.keys())):
                if list(occupied_polygon).count(1) <= max(2, self.number_of_detected_human*2):
                    yield State(id, list(occupied_polygon))

    """ Return all actions with non-zero probability from this state """
    def get_actions(self, state):
        if self.get_state_from_continuous_position(self.global_goal) == state.robot:
            return ["find_goal"]
        actions = [state.robot]
        polygons = self.polygons[state.robot]
        for poly in polygons:
            for n in poly.neighbors:
                if n.id in self.polygons:
                    actions.append(n.id)
        actions = list(np.unique(actions))
        return actions

    def get_next_states(self, state, action):
        next_states = [State(action, state.humans)]
        return self.human_trajectory_prediction_function(self, state, action)

    """ Return all non-zero probability transitions for this action
        from this state, as a list of (state, probability) pairs
    """
    def get_transitions(self, state, action):
        if self.human_trajectory_prediction_function is None:
            next_states = self.get_next_states(state, action)
            list_state_probability = [(nx, 1.0/len(next_states)) for nx in next_states]
            return list_state_probability
        return self.human_trajectory_prediction_function(state, action)

    """ Return the reward for transitioning from state to
        nextState via action
    """
    def get_reward(self, state, action, next_state):
        reward = 0

        # Penalty Time
        reward -= 5

        # Penalty Stationary
        if state.robot == action:
            reward -= PENALITY_WAIT

        # Penalty Distance
        reward += self._distance_factor * (self.goal.distance(self.get_position_of_state(state.robot)) - self.goal.distance(self.get_position_of_state(next_state.robot)))

        # Penalty Human
        for human in state.humans:
            human_state = self.get_state_from_continuous_position(human.position)
            if state.robot == human_state or action == human_state:
                reward -= self._social_factor * PENALITY_COLLISION_HUMAN
                break

        # for human in state.humans:
        #     for traj in human.future_predicted_position:
        #         traj_state = self.get_state_from_continuous_position(traj)
        #         if state.robot == traj_state or action == traj_state:
        #             reward -= self._social_factor * PENALITY_FUTURE_COLLISION_HUMAN
        #             break

        for polygon in self.polygons[state.robot]:
            if polygon.intersects(self.goal.buffer(0.05)):
                reward += REWARD_GOAL
                break
        return reward

    """ Return true if and only if state is a terminal state of this MDP """
    def is_terminal(self, state):
        if len(self.get_actions(state)) == 0:
            return True

        if self.get_state_from_continuous_position(self.goal) == state.robot:
            return True
        for polygon in self.polygons[state.robot]:
            if polygon.intersects(self.goal.buffer(0.1)):
                return True
        return False

    """ Return the discount factor for this MDP """
    def get_discount_factor(self):
        return self.discount_factor
    
    def get_state_from_continuous_position(self, position):
        if not isinstance(position, Point):
            position = Point(position)

        if isinstance(self.polygons, dict):
            for id, polygon in self.polygons.items():
                # print(id, polygon[0].centroid, position)
                for poly in polygon:
                    if poly.buffer(0.1).contains(position) or poly.buffer(0.1).intersects(position):
                        return id
        return None
    

    def get_position_of_state(self, state):
        return MultiPolygon([p.polygon for p in self.polygons[state]]).centroid

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