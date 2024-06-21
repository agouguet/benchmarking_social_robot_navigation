import os, sys, math, random, copy, yaml, cv2, warnings, pickle, numpy as np, networkx as nx
from scipy.spatial import distance

HEURISTIC_FUNCTIONS = [
    "random_function",
    "closest_node_to_goal",
    "avoid_humans",
    "farthest_from_human",
]

DEFAULT_HEURISTIC_FUNCTION = "avoid_humans"

LIMIT_DISTANCE_TO_OTHER_AGENT = 1.5

def random_function(mdp, state):
    return random.choice(mdp.get_actions(state))

def closest_node_to_goal(mdp, state):
    actions = mdp.get_actions(state)
    return min(actions, key=lambda a: (mdp.astar_dict[a._node][state.goal]))

def farthest_from_human(mdp, state, w1=1.0, w2=1000.0, w3=1.0):
    actions = mdp.get_actions(state)
    actions = sorted(actions, key=lambda a: (mdp.astar_dict[a._node][state.goal]))
    
    if not np.any(state.humans_node,  where=1):
        return actions[0]

    def min_dist_with_human(n):
        human_idx = [h for h in range(len(state.humans_node)) if state.humans_node[h] != 0]
        dist_from_each_human = [mdp.astar_dict[n][h] for h in human_idx]
        return min(dist_from_each_human)

    
    d={a:(w1*mdp.astar_dict[a._node][state.goal] - w2*min_dist_with_human(a._node)) for a in actions}
    a = min(d, key=d.get)
    print(state, a, d)

    dm_dict = {act:mdp.astar_dict[state.robot_node][act._node] for act in actions}

    dm_norm = {act:(dm_dict[act]-min(dm_dict.values()))/(max(dm_dict.values())-min(dm_dict.values())) for act in actions}

    dg_dict = {act:mdp.astar_dict[act._node][state.goal] for act in actions}

    dg_norm = {act:(dg_dict[act]-min(dg_dict.values()))/(max(dg_dict.values())-min(dg_dict.values())) for act in actions}

    dnear_dict = {act:min_dist_with_human(act._node) for act in actions}

    dnear_norm = {act:(dnear_dict[act]-min(dnear_dict.values()))/(max(dnear_dict.values())-min(dnear_dict.values()) + 10e-6) for act in actions}

    dnear_pwnorm = {act:dnear_dict[act]/1.5 if dnear_dict[act]<= LIMIT_DISTANCE_TO_OTHER_AGENT else 1.0 for act in actions}


    print(dm_dict)

    print("               DM  DG  DNEAR")
    for act in actions:
        dm = dm_dict[act]
        dg = dg_dict[act]
        # dnear = dnear_dict[act]
        alpha=1.0
        dnear = math.exp(-alpha*(dnear_dict[act] + 10e-6))
        test = math.exp(-alpha*(dnear_pwnorm[act] + 10e-6))

        v = - w1*dg - w2*dnear - w3*dm
        v_norm = 1 - (w1*dg_norm[act] + w2*test + w3*dm_norm[act]) / (w1+w2+w3)

        print("     ", act)
        
        print("         v:", '%.2f' % v)
        print("             ", '%.2f' % dm, '%.2f' % dg, '%.2f' % dnear, '%.2f' % dnear_dict[act], '%.2f' % dnear_pwnorm[act])
        print("         v_norm:", '%.2f' % v_norm)
        print("             ", '%.2f' % dm_norm[act], '%.2f' % dg_norm[act], '%.2f' % test, '%.2f' % dnear)

    return a



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