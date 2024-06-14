from collections import defaultdict
from mbsn.model.interfaces.MDP import MDP
from mbsn.model.graph.GraphAction import GraphAction
from mbsn.model.graph.GraphState import GraphState
from scipy.spatial import distance
from itertools import combinations 
import networkx as nx
import itertools
import numpy as np
import math
import random


PENALITY_DISTANCE_GOAL = 3
PENALITY_COLLISION_HUMAN = 1000
PENALITY_PROXIMITY_HUMAN =0.1
REWARD_GOAL = 100

class MBSN(MDP):

    def __init__(
        self,
        networkx_graph,
        robot_goal_node = None,
        distance_factor=1.0,
        social_factor=1.0, 
        discount_factor = 0.99
    ):
        self.graph = networkx_graph
        self.robot_goal_node = robot_goal_node
        self.astar_dict = self.astar_calculation()
        self.discount_factor = discount_factor
        self._social_factor = social_factor
        self._distance_factor = distance_factor

    """Return all states of this MDP"""
    def get_states(self):
        N = len(self.graph.nodes)
        if self.robot_goal_node == None:
            for robot_node in range(N):
                for robot_goal_node in range(N):
                    for occupied_node in itertools.product([0, 1], repeat=N):
                        yield GraphState(robot_node, robot_goal_node, list(occupied_node))
        else:
            for robot_node in range(N):
                for occupied_node in itertools.product([0, 1], repeat=N):
                    yield GraphState(robot_node, self.robot_goal_node, list(occupied_node))

    """ Return all actions with non-zero probability from this state """
    def get_actions(self, state):
        n = state.robot_node
        actions = [GraphAction(n, n)]
        for e in self.graph.edges([n]):
            actions.append(GraphAction(n, e[1]))
        return actions

    def get_next_states(self, state, action):
        next_states = [GraphState(action._node, state.goal, state.humans_node)]
        for human_node in [idx for idx, value in enumerate(state.humans_node) if value == 1]:
            neighbor = [i for i in self.graph.neighbors(human_node)]
            for n in neighbor:
                occupied_nodes = state.humans_node.copy()
                # print(occupied_nodes, n)
                occupied_nodes[n] = 1
                next_state = GraphState(action._node, state.goal, occupied_nodes)
                if next_state not in next_states:
                    next_states.append(next_state)
        return next_states

    """ Return all non-zero probability transitions for this action
        from this state, as a list of (state, probability) pairs
    """
    def get_transitions(self, state, action):
        next_states = self.get_next_states(state, action)
        list_state_probability = []
        for next_state in next_states:
            list_state_probability.append((next_state, 1/len(next_states)))
        return list_state_probability

    """ Return the reward for transitioning from state to
        nextState via action
    """
    def get_reward(self, state, action, next_state):
        reward = 0
        reward -= self._distance_factor * self.distance_cost(state, next_state)
        if state.humans_node[state.robot_node] or state.humans_node[next_state.robot_node]:
            reward -= self._social_factor * PENALITY_COLLISION_HUMAN
        # if next_state.robot_node == state.goal:
        #     reward += REWARD_GOAL
        if state.robot_node == state.goal:
            reward += REWARD_GOAL
        # reward -= self._social_factor * (self.proximity_cost_to_humans(next_state.robot_node, next_state.humans_node) + self.proximity_cost_to_humans(next_state.robot_node, state.humans_node))
        return reward

    """ Return true if and only if state is a terminal state of this MDP """
    def is_terminal(self, state):
        if len(self.get_actions(state)) == 0:
            return True
        if state.robot_node == state.goal:
            return True
        return False

    """ Return the discount factor for this MDP """
    def get_discount_factor(self):
        return self.discount_factor

    # """ Return the initial state of this MDP """
    # @abstractmethod
    # def get_initial_state(self):

    # """ Return all goal states of this MDP """
    # @abstractmethod
    # def get_goal_states(self):

    """ Return a new state and a reward for executing action in state,
    based on the underlying probability. This can be used for
    model-free learning methods, but requires a model to operate.
    Override for simulation-based learning
    """
    def execute(self, state, action):
        rand = random.random()
        cumulative_probability = 0.0
        for (new_state, probability) in self.get_transitions(state, action):
            if cumulative_probability <= rand <= probability + cumulative_probability:
                reward = self.get_reward(state, action, new_state)
                return (new_state, reward)
            cumulative_probability += probability
            if cumulative_probability >= 1.0:
                raise (
                    "Cumulative probability >= 1.0 for action "
                    + str(action)
                    + " from "
                    + str(state)
                )

        raise BaseException(
            "No outcome state in simulation for action"
            + str(action)
            + " from "
            + str(state)
        )

    """ 
    Execute a policy on this mdp for a number of episodes.
    """
    def execute_policy(self, policy, episodes=100):
        cumulative_rewards = []
        states = set()
        for _ in range(episodes):
            cumulative_reward = 0.0
            state = self.get_initial_state()
            step = 0
            while not self.is_terminal(state):
                actions = self.get_actions(state)
                action = policy.select_action(state, actions)
                (next_state, reward) = self.execute(state, action)
                cumulative_reward += reward * (self.discount_factor ** step)
                state = next_state
                step += 1
            cumulative_rewards += [cumulative_reward]
        return cumulative_rewards



    def distance_cost(self, s, next_state):
        return self.astar_dict[s.robot_node][next_state.robot_node]
        cost = self.euclidean_distance_between_node(s.robot_node, next_state.robot_node)
        cost += PENALITY_DISTANCE_GOAL * self.astar_dict[next_state.robot_node][s.goal]
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