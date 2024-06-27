import os, sys, math, random, copy, yaml, cv2, warnings, pickle, numpy as np, networkx as nx
from scipy.spatial import distance

HEURISTIC_FUNCTIONS = [
    "random_function",
    "closest_node_to_goal",
    "avoid_humans",
    "furthest_from_human",
]



LIMIT_DISTANCE_TO_OTHER_AGENT = 1.5

def random_function(mdp, state):
    return random.choice(mdp.get_actions(state))

def closest_node_to_goal(mdp, state):
    actions = mdp.get_actions(state)
    return min(actions, key=lambda a: (mdp.astar_dict[a._node][state.goal]))

def furthest_from_human(mdp, state, w1=1.0, w2=1.5, w3=0.2, debug=False):
    actions = mdp.get_actions(state)
    actions = sorted(actions, key=lambda a: (mdp.astar_dict[a._node][state.goal]))

    if not np.any(state.humans_node,  where=1):
        return actions[0]

    def min_dist_with_human(n):
        human_idx = [h for h in range(len(state.humans_node)) if state.humans_node[h] != 0]
        if len(human_idx) == 0:
            return float("inf")
        dist_from_each_human = [mdp.astar_dict[n][h] for h in human_idx]
        return min(dist_from_each_human)

    dm_dict = {act:mdp.astar_dict[state.robot_node][act._node] for act in actions}
    min_dm = min(dm_dict.values())
    max_dm = max(dm_dict.values())

    dg_dict = {act:mdp.astar_dict[act._node][state.goal] for act in actions}
    min_dg = min(dg_dict.values())
    max_dg = max(dg_dict.values())

    dnear_dict = {act:min_dist_with_human(act._node) for act in actions}

    def score_of_movement(action):
        return 1 - (dm_dict[action]-min_dm)/(max_dm-min_dm) 

    def score_from_goal(action):
        return 1 - (dg_dict[action]-min_dg)/(max_dg-min_dg)

    def score_from_closest_agent(action, limit=LIMIT_DISTANCE_TO_OTHER_AGENT):
        return dnear_dict[action]/limit
        # return dnear_dict[action]/limit if dnear_dict[action]<=limit else 1.0

    def standard_deviation(action, k=0.5, alpha=0.1):
        dg = score_from_goal(action)
        # dnear = score_from_closest_agent(action, 10.0)
        dnear = 1-math.exp(-alpha*score_from_closest_agent(action, 1.5))
        dm = score_of_movement(action)

        sum_weighted_score = w1*dg + w2*dnear + w3*dm
        mean = sum_weighted_score/(w1+w2+w3)
        weighted_variance = (w1*(dg-mean)**2 + w2*(dnear-mean)**2 + w3*(dm-mean)**2)/(w1+w2+w3)
        weighted_standard_deviation = round(math.sqrt(weighted_variance), 3)
        score = round(mean - k*weighted_standard_deviation, 3)
        

        if debug:
            print("", action, score, "%.2f" % mean, "%.2f" % weighted_standard_deviation)
            print("     ", "%.2f" % dg, "%.2f" % dnear, "%.2f" % dm)

        return score

    max_actions = []
    max_value = float("-inf")
    for action in actions:
        value = round(standard_deviation(action), 3)
        if value > max_value:
            max_actions = [action]
            max_value = value
        elif value == max_value:
            max_actions += [action]
    result = random.choice(max_actions)
    return result


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

DEFAULT_HEURISTIC_FUNCTION_STR = "furthest_from_human"
DEFAULT_HEURISTIC_FUNCTION = globals()[DEFAULT_HEURISTIC_FUNCTION_STR]