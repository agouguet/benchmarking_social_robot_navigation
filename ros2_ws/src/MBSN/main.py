
from collections import defaultdict
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


SCENARIO = "open"
TYPE_MAP = "hexagon"

# HEURISTIC_FUNCTION = heuristic_rules_based
HEURISTIC_FUNCTION = heuristic_score_based

# PREDICTION_FUNCTION = simple_human_trajectory_prediction
PREDICTION_FUNCTION = pecnet_human_trajectory_prediction
USE_MCTS = True
TIMEOUT_MCTS = 1.0
CORRECT_PREDICTED_GOAL = True



def scenario_01():
    goal_pool = [
        Point(-3.0, -17.0),
        Point(-10.0, -2.0),
        Point(+1.0, +17.0),
        Point(+4.0, +16.0),
        Point(+10.0, -4.0),
        Point(-7.0, -2.0),
        Point(-10.0, +12.0)
    ]

    robot_pos = Point(-10.0, -11.0)
    goal_pos = Point(9.0, 15.0)
    humans_goal_predicted = {}

    g1 = random.choice(goal_pool)
    gf1 = random.choice(goal_pool)
    human1 = Human(Point(10.0, -14.0), 180, goal=g1)
    humans_goal_predicted[human1.id] = gf1
    goal_pool.remove(g1)

    g2 = random.choice(goal_pool)
    gf2 = random.choice(goal_pool)
    human2 = Human(Point(-10.0, 0.0), -90, goal=g2)
    humans_goal_predicted[human2.id] = gf2
    goal_pool.remove(g2)

    g3 = random.choice(goal_pool)
    gf3 = random.choice(goal_pool)
    human3 = Human(Point(-1.5, 18.0), 0, goal=g3)
    humans_goal_predicted[human3.id] = gf3
    goal_pool.remove(g3)

    humans = [human1, human2, human3]

    map_polygon = NavMap(SCENARIO, type=TYPE_MAP)

    return map_polygon, goal_pos, robot_pos, humans, humans_goal_predicted

def scenario_02():
    robot_pos = Point(-10.0, -11.0)
    goal_pos = Point(8.0, 23.0)
    humans_goal_predicted = {}

    g1 = Point(-1.0, -17.0)
    human1 = Human(Point(-1.0, 18.0), -90, goal=g1)
    humans_goal_predicted[human1.id] = g1

    g2 = Point(-3.0, -17.0)
    human2 = Human(Point(-2.0, 18.0), -90, goal=g2)
    humans_goal_predicted[human2.id] = g2

    humans = [human1, human2]

    map_polygon = NavMap(SCENARIO, type=TYPE_MAP)

    return map_polygon, goal_pos, robot_pos, humans, humans_goal_predicted

def scenario_03():
    robot_pos = Point(2.0, -10.0)
    goal_pos = Point(2.0, 10.0)
    humans_goal_predicted = {}

    g1 = Point(2.0, -10.0)
    human1 = Human(Point(2.0, 10.0), -90, goal=g1)
    humans_goal_predicted[human1.id] = g1

    humans = [human1]

    map_polygon = NavMap(SCENARIO, type=TYPE_MAP)

    return map_polygon, goal_pos, robot_pos, humans, humans_goal_predicted

def scenario_04():
    humans_goal_predicted = {}
    robot_pos = Point(2.0, -10.0)
    goal_pos = Point(2.0, 10.0)

    g1 = Point(2.0, -10.0)
    gf1 = Point(0.0, -5.0)

    human1 = Human(Point(2.0, 10.0), -90, goal=g1)
    humans_goal_predicted[human1.id] = gf1

    humans = [human1]

    map_polygon = NavMap(SCENARIO, type=TYPE_MAP)

    return map_polygon, goal_pos, robot_pos, humans, humans_goal_predicted

def scenario_t_corridor():
    humans_goal_predicted = {}
    robot_pos = Point(0.0, 11.0)
    goal_pos = Point(0.0, -11.0)

    g1 = Point(0.0, 11.0)
    gf1 = Point(-12.0, 0.0)

    human1 = Human(Point(0.0, -11.0), 90, goal=g1)
    humans_goal_predicted[human1.id] = gf1

    humans = [human1]

    map_polygon = NavMap("t_corridor", type=TYPE_MAP)

    return map_polygon, goal_pos, robot_pos, humans, humans_goal_predicted

def scenario_small():
    p1 = Point(0.0, 4.0)
    center_bot = Point(0.0, 3.5)
    p2 = Point(1.5, 2.5)
    p3 = Point(-2.5, 0.0)
    center = Point(0.0, 0.0)
    p4 = Point(1.5, -2.5)
    center_top = Point(0.0, -3.5)
    p5 = Point(0.0, -4.0)

    humans_goal_predicted = {}
    robot_pos = p1
    goal_pos = p5

    human_pos = p5

    gp1 = [p1]
    gp2 = [center, p2]
    gp3 = [center, p3]
    gp4 = [center_top, p4]

    list_of_gp = [gp1, gp2, gp3, gp4]

    if not CORRECT_PREDICTED_GOAL:
        # g1 = random.choice(list_of_gp)
        # list_of_gp.remove(g1)

        # gf1 = random.choice(list_of_gp)
        # list_of_gp.remove(gf1)

        g1 = gp2
        list_of_gp.remove(g1)
        gf1 = random.choice(list_of_gp)
    else:
        g1 = gp2
        gf1 = g1

    human1 = Human(human_pos, 90, goal=g1)
    humans_goal_predicted[human1.id] = g1 if CORRECT_PREDICTED_GOAL else gf1

    humans = [human1]

    map_polygon = NavMap("small", type=TYPE_MAP)

    return map_polygon, goal_pos, robot_pos, humans, humans_goal_predicted

def scenario_door_passing():
    humans_goal_predicted = {}
    robot_pos = Point(11.0, 0.0)
    goal_pos = Point(-11.0, 0.0)

    g1 = Point(11.0, 0.0)
    gf1 = Point(-0.5, -1.2)

    human1 = Human(Point(-11.0, 0.0), 0, goal=g1)
    humans_goal_predicted[human1.id] = gf1

    humans = [human1]

    map_polygon = NavMap("door_passing", type=TYPE_MAP)

    return map_polygon, goal_pos, robot_pos, humans, humans_goal_predicted

def scenario_open(number_of_human):

    pos_pool = [
        Point(-12.0, -0.0),
        Point(-12.0, -3.25),
        Point(-12.0, -6.5),
        Point(-9.0, -6.5),
        Point(-6.0, -6.5),
        Point(-3.0, -6.5),
        Point(0.0, -6.5),
        Point(3.0, -6.5),
        Point(6.0, -6.5),
        Point(9.0, -6.5),
        Point(12.0, -6.5),
        Point(12.0, -3.25),
        Point(12.0, 0.0),
        Point(12.0, 3.25),
        Point(12.0, 6.5),
        Point(9.0, 6.5),
        Point(6.0, 6.5),
        Point(3.0, 6.5),
        Point(0.0, 6.5),
        Point(-3.0, 6.5),
        Point(-6.0, 6.5),
        Point(-9.0, 6.5),
        Point(-12.0, 6.5),
        Point(-12.0, 3.25),
    ]

    pos_pool = [Point(-12.0, i) for i in np.linspace(-6.5, 6.5, num=50)] + [Point(12.0, i) for i in np.linspace(-6.5, 6.5, num=50)] + [Point(i, -6.5) for i in np.linspace(-12, 12, num=50)] + [Point(i, 6.5) for i in np.linspace(-12, 12, num=50)]

    humans_goal_predicted = {}
    robot_pos = random.choice(pos_pool)
    pos_pool.remove(robot_pos)

    goal_pos = random.choice(pos_pool)
    while goal_pos.x == robot_pos.x or goal_pos.y == robot_pos.y:
        goal_pos = random.choice(pos_pool)
    pos_pool.remove(goal_pos)

    humans = []

    for i in range(number_of_human):
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

    map_polygon = NavMap(SCENARIO, type=TYPE_MAP)

    return map_polygon, goal_pos, robot_pos, humans, humans_goal_predicted

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




# map_polygon, goal_pos, robot_pos, humans, humans_goal_predicted = scenario_open(5)
map_polygon, goal_pos, robot_pos, humans, humans_goal_predicted = scenario_small()



# matplotlib.use('Qt5Agg')
# fig, ax = plt.subplots()

# map_polygon.plot(ax)

# for id, cell in map_polygon.test_grid.items():
#     cell.plot(ax=ax, add_id=True)

# ax.axis('off')
# ax.set_aspect('equal', adjustable='box')
# ax.invert_yaxis()
# plt.show()


test = EnvironmentState(map_polygon, mcts=USE_MCTS, timeout_mcts=TIMEOUT_MCTS, heuristic=HEURISTIC_FUNCTION, prediction_function=PREDICTION_FUNCTION)
test.start_navigation(goal_pos)

# test.human_updated([human1, human2, human3])
action = test.robot_position_updated(robot_pos)
# action = test.robot_position_updated(Point(0, 4.0))
next_cell = test.local_mdp.polygons[action]
test.human_updated(humans)

# human1.move(-1.0, +0.0)
# human1.future_predicted_position = [Point(0.0, human1.position.y + i) for i in range(5)]
# test.human_updated([human1])

matplotlib.use('Qt5Agg')


i = 0
while test.current_pos.distance(goal_pos) > 0.5  and i < 50:
    # if i % 5 == 0 and i != 0:
    #     fig, ax = plt.subplots()
    #     test.plot(ax, plot_circle_previous_pos=True, debug=True)
    #     # ax.set_xlim(test.current_pos.x-8, test.current_pos.x+8)
    #     # ax.set_ylim(test.current_pos.y-8, test.current_pos.y+8)
    #     ax.axis('off')
    #     ax.set_aspect('equal', adjustable='box')
    #     ax.invert_yaxis()
    #     figManager = plt.get_current_fig_manager()
    #     figManager.window.showMaximized()
    #     plt.show()

    t = time.time()

    # human1.move(-0.0, +1.0)
    for h in humans:
        # print(h, humans_goal_predicted)
        move_a_human(map_polygon, h, humans_goal_predicted[h.id])
    # human1.move(-1.0, +0.0)
    # human1.future_predicted_position = [Point(human1.position.x - i, human1.position.y) for i in range(5)]
    test.human_updated(humans)

    action = test.robot_position_updated(MultiPolygon([p.polygon for p in next_cell]).centroid)
    if action != "find_goal":
        next_cell = test.local_mdp.polygons[action]
    else:
        fig, ax = plt.subplots()
        test.plot(ax)    
        ax.axis('off')
        ax.set_aspect('equal', adjustable='box')
        ax.invert_yaxis()
        figManager = plt.get_current_fig_manager()
        figManager.window.showMaximized()
        plt.show()
        break
    
    print("PATH=", test.robot_path)
    print(time.time() - t)
    i+=1

dpi = 100

px = 10
fig, ax = plt.subplots(figsize=(1.9*px, 2*px))
test.plot(ax, plot_circle_previous_pos=True)

# if PREDICTION_FUNCTION.__name__ == pecnet_human_trajectory_prediction.__name__:
#     for id, goals_predicted in humans_goal_predicted.items():
#         pos = goals_predicted[0]
#         circle = plt.Circle((pos.x, pos.y), radius=0.15, color="green", label="Predicted Goal")
#         ax.add_patch(circle)
#         label = ax.annotate("PG", xy=(pos.x, pos.y), fontsize=24, ha="center", color="white", verticalalignment="center", horizontalalignment="center")
# ax.set_xlim(test.current_pos.x-8, test.current_pos.x+8)
# ax.set_ylim(test.current_pos.y-8, test.current_pos.y+8)
ax.axis('off')
ax.set_aspect('equal', adjustable='box')
ax.invert_yaxis()
fig.tight_layout()
figManager = plt.get_current_fig_manager()
figManager.window.showMaximized()
# plt.savefig('./results/path/destination_path.eps', format='eps')
fig.savefig('./results/path/destination_path.svg', format='svg', dpi=dpi)
# fig.savefig('./results/path/destination_path.png', format='png', dpi=dpi)
# plt.show()
print("Node visited:", MBSNAgentNode.next_node_id)