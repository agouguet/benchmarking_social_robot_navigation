import os, sys, math, random, copy, yaml, cv2, warnings, pickle, numpy as np, networkx as nx
from scipy.spatial import distance

HEURISTIC_FUNCTIONS = [
    "random_function",
    "closest_node_to_goal",
    "avoid_humans",
    "farthest_from_human",
]

DEFAULT_HEURISTIC_FUNCTION = "avoid_humans"

def random_function(mdp, state):
    return random.choice(mdp.get_actions(state))

def closest_node_to_goal(mdp, state):
    actions = mdp.get_actions(state)
    return min(actions, key=lambda a: (mdp.astar_dict[a._node][state.goal]))

def farthest_from_human(mdp, state, w1=0.0, w2=1.0):
    actions = mdp.get_actions(state)
    actions = sorted(actions, key=lambda a: (mdp.astar_dict[a._node][state.goal]))
    
    if not np.any(state.humans_node,  where=1):
        return actions[0]

    def min_dist_with_human(n):
        human_idx = [h for h in range(len(state.humans_node)) if state.humans_node[h] != 0]
        dist_from_each_human = [mdp.astar_dict[n][h] for h in human_idx]
        return min(dist_from_each_human)

    actions = sorted(actions, key=lambda a: (w1*mdp.astar_dict[a._node][state.goal] - w2*min_dist_with_human(a._node)))
    a = actions[0]
    print(state, a, w1*mdp.astar_dict[a._node][state.goal] + w2*min_dist_with_human(a._node))
    return a


def avoid_humans(mdp, state):
    actions = mdp.get_actions(state)
    actions = sorted(actions, key=lambda a: (mdp.astar_dict[a._node][state.goal]))

    def dist(u, v):
        (x1, y1) = mdp.graph.nodes(data=True)[u]['pos']
        (x2, y2) = mdp.graph.nodes(data=True)[v]['pos']
        return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5

    def min_dist_with_human(n):
        human_idx = [h for h in range(len(state.humans_node)) if state.humans_node[h] != 0]
        dist_from_each_human = [mdp.astar_dict[n][h] for h in human_idx]
        return min(dist_from_each_human)

    if not np.any(state.humans_node,  where=1):
        return actions[0]

    for action in actions:
        path = nx.astar_path(mdp.graph, action._node, state.goal, heuristic=dist)
        human_on_path = False
        for node in path:
            if min_dist_with_human(node) < 2.0:
                human_on_path=True
        if not human_on_path:
            return action
    

    distance_between_robot_and_goal = mdp.astar_dict[state.robot_node][state.goal]

    for action in actions:
        if not state.humans_node[action._node] and mdp.astar_dict[action._node][state.goal] < distance_between_robot_and_goal:
            return action

    for action in actions:
        if action._node == state.robot_node:
            return action

    return actions[0]