import heapq
import time
from shapely import MultiPolygon
import os, sys, math, random, copy, yaml, cv2, warnings, pickle, numpy as np, networkx as nx
from scipy.spatial import distance

from mbsn.model.polygon.State import State

HEURISTIC_FUNCTIONS = [
    "random_function",
    "closest_to_goal",
    "avoid_humans",
    "social_heuristic",
]



LIMIT_DISTANCE_TO_OTHER_AGENT = 1.5

def random_function(mdp, state):
    return random.choice(mdp.get_actions(state))

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


def astar(start, goal, mdp, limited_action_of_start_state=None):

    def heuristic(cell1, cell2):
        # Utilisation de la distance de Manhattan comme heuristique
        a = mdp.polygons[cell1][0].centroid
        b = mdp.polygons[cell2][0].centroid
        return a.distance(b)
        return abs(a.x - b.x) + abs(a.y - b.y) + mdp.polygons[cell1][0].area

    open_set = []
    heapq.heappush(open_set, (0, start.robot))
    came_from = {}
    g_score = {start.robot: 0}
    f_score = {start.robot: heuristic(start.robot, goal)}

    while open_set:
        _, current = heapq.heappop(open_set)

        if current == goal:
            # Reconstruire le chemin
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start.robot)
            path.reverse()
            return path

        actions = mdp.get_actions(State(current,start.humans))
        if limited_action_of_start_state is not None and current == start.robot:
            actions = limited_action_of_start_state
        
        for neighbor in actions:
            tentative_g_score = g_score[current] + 1
            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heapq.heappush(open_set, (f_score[neighbor], neighbor))

    return None  # Aucun chemin trouvé

def closest_to_goal(mdp, state, limited_action_of_start_state=None):
    cell_goal = mdp.get_state_from_continuous_position(mdp.goal)
    astar_path = astar(state, cell_goal, mdp, limited_action_of_start_state=limited_action_of_start_state)
    if astar_path is not None:
        return astar_path[1]
    return state.robot

def social_heuristic(mdp, state, prev_actions=None, actions=None):
    if len(state.humans) == 0:
        # print("closest_to_goal : ", closest_to_goal(mdp, state))
        return closest_to_goal(mdp, state)

    cell_goal = mdp.get_state_from_continuous_position(mdp.goal)
    if actions is None:
        actions = mdp.get_actions(state)
    
    if prev_actions is not None:
        for a in prev_actions:
            if a is not None and a != state.robot and a in actions:
                actions.remove(a)
                
    for human in state.humans:
        for pos in human.future_predicted_position:
            pos_traj_state = mdp.get_state_from_continuous_position(pos)
            if pos_traj_state in actions:
                actions.remove(pos_traj_state)

    # sorted_actions = []

    # for action in actions:
    #     astar_path = astar(State(action, state.humans), cell_goal, mdp)
    #     if astar_path is None:
    #         continue
    #     dist = 0
    #     for i in range(1, len(astar_path)):
    #         cell = astar_path[i-1]
    #         next_cell = astar_path[i]
    #         dist += mdp.polygons[cell][0].centroid.distance(mdp.polygons[next_cell][0].centroid)
    #     sorted_actions.append((action, dist))

    # # actions = sorted(sorted_actions, key=lambda a: a[1])
    # actions = [a[0] for a in sorted(sorted_actions, key=lambda a:a[1])]

    # for action in actions:
    #     astar_path = astar(State(action, state.humans), cell_goal, mdp)
    #     human_on_path = False
    #     for cell in astar_path:
    #         for h in state.humans:
    #             if mdp.polygons[cell][0].centroid.distance(h.position) <= 1.2:
    #                 human_on_path = True
    #     if not human_on_path:
    #         return action

    #                     |||
    # NEED TO CHANGE THIS vvv
    #

    # print("S:", state, actions)

    for h in state.humans:
        if h.position.distance(mdp.polygons[state.robot][0].centroid) < 1.0:
            return state.robot
    
    

    actions = list(actions)
    if len(actions) == 0:
        return state.robot
    return closest_to_goal(mdp, state, limited_action_of_start_state=actions)
    
    _, max_value = mdp.execute(state, actions[0])
    best_actions = [actions[0]]

    for i in range(1, len(actions)):
        _, v = mdp.execute(state, actions[i])
        print("--->", actions[i], v)
        if v > max_value:
            max_value = v
            best_actions = [actions[i]]
        elif v == max_value:
            best_actions.append(actions[i])

    

    actions = sorted(best_actions, key=lambda a: -mdp.polygons[a][0].area)
    # print("BA --> ", actions)
    return actions[0]
    
    

DEFAULT_HEURISTIC_FUNCTION_STR = "social_heuristic"
DEFAULT_HEURISTIC_FUNCTION = globals()[DEFAULT_HEURISTIC_FUNCTION_STR]
