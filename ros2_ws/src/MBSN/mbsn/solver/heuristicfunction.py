import heapq
import time
from matplotlib import pyplot as plt
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

LIMIT_DISTANCE_TO_OTHER_AGENT = 1.0

def random_function(mdp, state):
    return random.choice(mdp.get_actions(state))

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

def heuristic_rules_based(mdp, state, prev_actions=None, actions=None):
    if state.robot == mdp.get_state_from_continuous_position(mdp.goal):
        return state.robot
    
    if len(state.humans) == 0:
        return closest_to_goal(mdp, state)

    if actions is None:
        actions = mdp.get_actions(state)
    
    # if prev_actions is not None:
    #     for a in prev_actions:
    #         if a is not None and a != state.robot and a in actions:
    #             actions.remove(a)
                
    for human in state.humans:
        for pos in human.future_predicted_position:
            pos_traj_state = mdp.get_state_from_continuous_position(pos)
            if pos_traj_state in actions:
                actions.remove(pos_traj_state)

    for h in state.humans:
        if h.position.distance(mdp.polygons[state.robot][0].centroid) < 0.7:
            return state.robot

    actions = list(actions)
    if len(actions) == 0:
        return state.robot
    return closest_to_goal(mdp, state, limited_action_of_start_state=actions)    




def heuristic_score_based(mdp, state, w1=1.0, w2=1.5, w3=0.1, w4=1.0, debug=False, prev_actions=None, actions=None):
    if state.robot == mdp.get_state_from_continuous_position(mdp.goal):
        return state.robot
    
    actions = mdp.get_actions(state)
    # actions = sorted(actions, key=lambda a: (mdp.astar_dict[a._node][state.goal]))
    robot_pos = mdp.polygons[state.robot][0].centroid

    if len(state.humans) == 0:
        return closest_to_goal(mdp, state)
    
    # for human in state.humans:
    #     for pos in human.future_predicted_position:
    #         pos_traj_state = mdp.get_state_from_continuous_position(pos)
    #         if pos_traj_state in actions:
    #             actions.remove(pos_traj_state)

    

    def score_of_movement(state, action):
        actions = mdp.get_actions(state)
        robot_pos = mdp.polygons[state.robot][0].centroid
        dm_dict = {act:robot_pos.distance(mdp.polygons[act][0].centroid) for act in actions}
        min_dm = min(dm_dict.values())
        max_dm = max(dm_dict.values())
        return 1 - (dm_dict[action]-min_dm)/(max_dm-min_dm) 
    
    def score_from_goal(state, action):
        actions = mdp.get_actions(state)
        dg_dict = {act:mdp.goal.distance(mdp.polygons[act][0].centroid) for act in actions}
        min_dg = min(dg_dict.values())
        max_dg = max(dg_dict.values())
        return 1 - (dg_dict[action]-min_dg)/(max_dg-min_dg)

    def min_dist_with_human(state, action):
        dist = math.inf
        action_pos = mdp.polygons[action][0].centroid
        for human in state.humans:
            dist_robot_human = human.position.distance(action_pos)
            if dist_robot_human < dist:
                dist = dist_robot_human

        for human in state.humans:
            for pos in human.future_predicted_position:
                dist_robot_future_pos_human = pos.distance(action_pos)
                if dist_robot_future_pos_human < dist:
                    dist = dist_robot_future_pos_human
        return dist

    def score_from_closest_agent(state, action, limit=LIMIT_DISTANCE_TO_OTHER_AGENT):
        mean_min_dist = 0
        for ns, p in mdp.get_transitions(state, action):
            # print("             ", ns, p)
            mean_min_dist += min_dist_with_human(ns, action) * p
        mean_min_dist /= len(mdp.get_transitions(state, action))
        dnear = min_dist_with_human(state, action) #+ mean_min_dist
        # return dnear_dict[action]/limit
        return dnear#/limit #LAST VERSION
        # return dnear_dict[action]/limit if dnear_dict[action]<=limit else 1.0

        return dnear

    def score_for_direction_passage_with_humans(state, action):
        if len(state.humans) == 0:
            return 0.0
        
        p1 = mdp.polygons[state.robot][0].centroid
        p3 = mdp.polygons[action][0].centroid

        closest_human = None
        dist = math.inf
        for human in state.humans:
            dist_robot_human = human.position.distance(p1)
            if dist_robot_human < dist:
                dist = dist_robot_human
                closest_human = human
        p2 = closest_human.position

        x1, y1 = p1.x, p1.y
        x2, y2 = p2.x, p2.y
        x3, y3 = p3.x, p3.y
        
        cross_product = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1)
        
        if cross_product == 0:
            return 0.0
        elif cross_product > 0:
            return 1.0
        elif cross_product < 0:
            return 0.99

    def standard_deviation(state, action, k=0.5, alpha=0.8):
        dg = score_from_goal(state, action)
        # dnear = 1-math.exp(-alpha*score_from_closest_agent(state, action))

        dist = score_from_closest_agent(state, action)
        limit = LIMIT_DISTANCE_TO_OTHER_AGENT
        dnear = np.where(dist >= limit, 1.0, dist/limit * np.exp(-alpha*(limit-dist)**2))
        # print("DNEAR=", dnear, dist)

        dm = score_of_movement(state, action)
        do = score_for_direction_passage_with_humans(state, action)

        sum_weighted_score = w1*dg + w2*dnear + w3*dm + w4*do
        mean = sum_weighted_score/(w1+w2+w3+w4)
        weighted_variance = (w1*(dg-mean)**2 + w2*(dnear-mean)**2 + w3*(dm-mean)**2 + w4*(do-mean)**2)/(w1+w2+w3+w4)
        weighted_standard_deviation = round(math.sqrt(weighted_variance), 3)
        score = round(mean - k*weighted_standard_deviation, 3)
        

        if debug:
            print("\n", state)
            print("", action, score, "%.2f" % mean, "%.2f" % weighted_standard_deviation)
            print("     ", "%.2f" % dg, "%.2f" % dnear, "%.2f" % dm, "%.2f" % do)
            print("score_from_closest_agent", score_from_closest_agent(state, action), dnear)

        return score

    max_actions = []
    max_value = float("-inf")
    # print(state)
    for action in actions:
        value = round(standard_deviation(state, action), 3)
        # print("     ", action, value)
        if value > max_value:
            max_actions = [action]
            max_value = value
        elif value == max_value:
            max_actions += [action]
    result = random.choice(max_actions)
    return result




DEFAULT_HEURISTIC_FUNCTION_STR = "heuristic_rules_based"
DEFAULT_HEURISTIC_FUNCTION = globals()[DEFAULT_HEURISTIC_FUNCTION_STR]