
from collections import defaultdict
import math
import time
from matplotlib import pyplot as plt
import matplotlib


import numpy as np
import random
from shapely import GeometryCollection, MultiPolygon, delaunay_triangles
from shapely.geometry import Point, Polygon, LineString, box
from shapely.plotting import plot_line, plot_points, plot_polygon
from shapely.ops import triangulate

import calfem.core as cfc
import calfem.geometry as cfg
import calfem.mesh as cfm
import calfem.vis_mpl as cfv
import calfem.utils as cfu

from mbsn.model.polygon.environment_state import EnvironmentState, get_cell_id_in_dict_from_continuous_position
from mbsn.model.trajectory_prediction.human_trajctory_prediction_model import simple_human_trajectory_prediction, pecnet_human_trajectory_prediction, simple_trajectory_prediction_for_one_human
from mbsn.solver.MCTS import MBSNAgentNode
from mbsn.solver.heuristicfunction import heuristic_score_based, heuristic_rules_based
from mbsn.utils.map_loader import load_map
from mbsn.model.polygon.NavPolygonMDP import NavPolygonMDP
from mbsn.model.polygon.human import Human
from mbsn.polygon.nav_map import NavMap
from mbsn.solver.ValueIteration import ValueIteration
from mbsn.utils.util import astar

matplotlib.use('Agg')

SCENARIO = "open"
TYPE_MAP = "hexagon"

# HEURISTIC_FUNCTION = heuristic_rules_based
HEURISTIC_FUNCTION = heuristic_score_based

PREDICTION_FUNCTION = simple_human_trajectory_prediction
# PREDICTION_FUNCTION = pecnet_human_trajectory_prediction
USE_MCTS = True
TIMEOUT_MCTS = 1.0
CORRECT_PREDICTED_GOAL = False
NUMBER_HUMAN = 50

NB_TIME = 1


def move_a_human(map_polygon, human, human_goal_detected_by_robot):
    h_pos = get_cell_id_in_dict_from_continuous_position(map_polygon.test_grid, human.position)
    g_pos = get_cell_id_in_dict_from_continuous_position(map_polygon.test_grid, human.goal[0])
    true_path = astar(h_pos, g_pos, map_polygon.test_grid)
    if len(true_path) > 1:
        human.position = map_polygon.test_grid[true_path[1]].centroid
    else:
        if len(human.goal) > 1:
            human.goal.pop(0)
            if human.goal != human_goal_detected_by_robot and len(human_goal_detected_by_robot) > 1:
                human_goal_detected_by_robot.pop(0)

    if PREDICTION_FUNCTION.__name__ == simple_human_trajectory_prediction.__name__:
        human.future_predicted_position = simple_trajectory_prediction_for_one_human(human)
    else:
        h_pos = get_cell_id_in_dict_from_continuous_position(map_polygon.test_grid, human.position)
        g_dbr_pos = get_cell_id_in_dict_from_continuous_position(map_polygon.test_grid, human_goal_detected_by_robot[0])
        detected_path = astar(h_pos, g_dbr_pos, map_polygon.test_grid)
        if len(detected_path) > 1:
            human.future_predicted_position = [map_polygon.test_grid[id].centroid for id in detected_path[1:6]]


def make_scenario_n_time(n):
    map_polygon = NavMap(SCENARIO, type=TYPE_MAP)

    all_minimum_distance = []
    all_nodes_visited = []

    for id_test in range(n):
        MBSNAgentNode.reset_visits()
        print(MBSNAgentNode.visits)
        pos_pool = [Point(-12.0, i) for i in np.linspace(-6.5, 6.5, num=50)] + [Point(12.0, i) for i in np.linspace(-6.5, 6.5, num=50)] + [Point(i, -6.5) for i in np.linspace(-12, 12, num=50)] + [Point(i, 6.5) for i in np.linspace(-12, 12, num=50)]

        humans_goal_predicted = {}
        robot_pos = random.choice(pos_pool)
        pos_pool.remove(robot_pos)

        goal_pos = random.choice(pos_pool)
        while goal_pos.x == robot_pos.x or goal_pos.y == robot_pos.y:
            goal_pos = random.choice(pos_pool)
        pos_pool.remove(goal_pos)

        humans = []

        for i in range(NUMBER_HUMAN):
            pos = random.choice(pos_pool)
            while pos.distance(robot_pos) < 1.5:
                pos = random.choice(pos_pool)
            pos_pool.remove(pos)

            g1 = random.choice(pos_pool)
            while g1.distance(goal_pos) < 1.5 or g1.x == pos.x or g1.y == pos.y:
                g1 = random.choice(pos_pool)
            pos_pool.remove(g1)

            gf1 = random.choice(pos_pool)
            ori = 180 if pos.x == 12 else 0
            human = Human(pos, ori, goal=[g1])
            humans_goal_predicted[human.id] = [gf1]
            humans.append(human)

        test = EnvironmentState(map_polygon, mcts=USE_MCTS, timeout_mcts=TIMEOUT_MCTS, heuristic=HEURISTIC_FUNCTION, prediction_function=PREDICTION_FUNCTION)
        test.start_navigation(goal_pos)

        action = test.robot_position_updated(robot_pos)
        next_cell = test.local_mdp.polygons[action]
        test.human_updated(humans)

        total_minimum_distance = math.inf

        i = 0
        while test.current_pos.distance(goal_pos) > 0.5  and i < 50:
            for h in humans:
                move_a_human(map_polygon, h, humans_goal_predicted[h.id])
            test.human_updated(humans)

            action = test.robot_position_updated(MultiPolygon([p.polygon for p in next_cell]).centroid)
            if action != "find_goal":
                next_cell = test.local_mdp.polygons[action]
            else:
                break
            

            minimum_distance = math.inf

            for h in humans:
                dist = h.position.distance(test.local_mdp.polygons[test.robot_path[-1]][0].centroid)
                if dist < minimum_distance:
                    minimum_distance = dist

            if minimum_distance < total_minimum_distance:
                total_minimum_distance = minimum_distance

            i+=1

        print("Node visited:", MBSNAgentNode.next_node_id)

        all_minimum_distance.append(total_minimum_distance)
        all_nodes_visited.append(MBSNAgentNode.next_node_id)

        dpi = 100
        px = 10
        fig, ax = plt.subplots(figsize=(1.9*px, 2*px))
        test.plot(ax, plot_circle_previous_pos=True)
        ax.axis('off')
        ax.set_aspect('equal', adjustable='box')
        ax.invert_yaxis()
        fig.tight_layout()
        # figManager = plt.get_current_fig_manager()
        # figManager.window.showMaximized()
        # plt.show()
        fig.savefig('./results/path/multi_test/'+str(id_test)+'.svg', format='svg', dpi=dpi)
    
    return all_minimum_distance, all_nodes_visited



all_minimum_distance, all_nodes_visited = make_scenario_n_time(NB_TIME)

print(all_minimum_distance, all_nodes_visited)

all_minimum_distance = np.array(all_minimum_distance)
all_nodes_visited = np.array(all_nodes_visited)

print(np.mean(all_minimum_distance), np.mean(all_nodes_visited))