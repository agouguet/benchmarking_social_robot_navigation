
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
from mbsn.utils.map_loader import load_map
from mbsn.model.polygon.NavPolygonMDP import NavPolygonMDP
from mbsn.model.polygon.human import Human
from mbsn.polygon.nav_map import NavMap
from mbsn.solver.ValueIteration import ValueIteration
from mbsn.utils.util import astar


SCENARIO = "openspace"
TYPE_MAP = "square"

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

    g1 = Point(-1.0, -17.0)
    human1 = Human(Point(-1.0, 18.0), -90, goal=g1)

    g2 = Point(-3.0, -17.0)
    human2 = Human(Point(-2.0, 18.0), -90, goal=g2)

    humans = [human1, human2]

    map_polygon = NavMap(SCENARIO, type=TYPE_MAP)

    return map_polygon, goal_pos, robot_pos, humans


def scenario_03():
    robot_pos = Point(2.0, -10.0)
    goal_pos = Point(2.0, 10.0)

    g1 = Point(2.0, -10.0)
    human1 = Human(Point(2.0, 10.0), -90, goal=g1)

    humans = [human1]

    map_polygon = NavMap(SCENARIO, type=TYPE_MAP)

    return map_polygon, goal_pos, robot_pos, humans


def scenario_04():
    humans_goal_predicted = {}
    robot_pos = Point(2.0, -10.0)
    goal_pos = Point(2.0, 10.0)

    g1 = Point(2.0, -10.0)
    gf1 = Point(0.0, -5.0)

    human1 = Human(Point(2.0, 10.0), -90, goal=g1)
    humans_goal_predicted[human1.id] = g1

    humans = [human1]

    map_polygon = NavMap(SCENARIO, type=TYPE_MAP)

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

    pos_pool = [Point(-12.0, i) for i in np.linspace(-6.5, 6.5, num=50)] + [Point(12.0, i) for i in np.linspace(-6.5, 6.5, num=50)]# + [Point(i, -6.5) for i in range(-12, 12, 0.5)] + [Point(i, 6.5) for i in range(-12, 12, 0.5)] + [Point(12.0, i) for i in range(-6.5, 6.5, 0.5)]

    def get_random_pos_in_pool(pool):
        g1 = random.choice(pool)
        pool.remove(g1)
        return g1


    humans_goal_predicted = {}
    robot_pos = get_random_pos_in_pool(pos_pool)
    goal_pos = get_random_pos_in_pool(pos_pool)
    while goal_pos.x == robot_pos.x:
        goal_pos = get_random_pos_in_pool(pos_pool)

    humans = []

    for i in range(number_of_human):
        pos = get_random_pos_in_pool(pos_pool)
        while pos.distance(robot_pos) < 1.5:
            pos = get_random_pos_in_pool(pos_pool)

        g1 = get_random_pos_in_pool(pos_pool)
        while g1.distance(goal_pos) < 1.5:
            g1 = get_random_pos_in_pool(pos_pool)

        gf1 = get_random_pos_in_pool(pos_pool)
        ori = 180 if pos.x == 12 else 0
        human = Human(pos, ori, goal=g1)
        humans_goal_predicted[human.id] = gf1
        humans.append(human)

    map_polygon = NavMap(SCENARIO, type=TYPE_MAP)

    return map_polygon, goal_pos, robot_pos, humans, humans_goal_predicted


def move_a_human(map_polygon, human, human_goal_detected_by_robot):
    h_pos = get_cell_id_in_dict_from_continuous_position(map_polygon.test_grid, human.position)
    g_pos = get_cell_id_in_dict_from_continuous_position(map_polygon.test_grid, human.goal)
    true_path = astar(h_pos, g_pos, map_polygon.test_grid)
    if len(true_path) > 1:
        human.position = map_polygon.test_grid[true_path[1]].centroid


    h_pos = get_cell_id_in_dict_from_continuous_position(map_polygon.test_grid, human.position)
    g_dbr_pos = get_cell_id_in_dict_from_continuous_position(map_polygon.test_grid, human_goal_detected_by_robot)
    detected_path = astar(h_pos, g_dbr_pos, map_polygon.test_grid)
    if len(detected_path) > 1:
        human.future_predicted_position = [map_polygon.test_grid[id].centroid for id in detected_path[1:6]]
    # print(h_pos, path)
    # human1.move(-1.0, +0.0)




map_polygon, goal_pos, robot_pos, humans, humans_goal_predicted = scenario_open(5)



# matplotlib.use('Qt5Agg')
# fig, ax = plt.subplots()

# map_polygon.plot(ax)

# for id, cell in map_polygon.test_grid.items():
#     cell.plot(ax=ax, add_id=True)

# ax.axis('off')
# ax.set_aspect('equal', adjustable='box')
# ax.invert_yaxis()
# plt.show()


test = EnvironmentState(map_polygon)
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
    # if i % 10 == 0 and i != 0:
    #     fig, ax = plt.subplots()
    #     test.plot(ax, plot_circle_previous_pos=True)
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

fig, ax = plt.subplots()
test.plot(ax, plot_circle_previous_pos=True)
# ax.set_xlim(test.current_pos.x-8, test.current_pos.x+8)
# ax.set_ylim(test.current_pos.y-8, test.current_pos.y+8)
ax.axis('off')
ax.set_aspect('equal', adjustable='box')
ax.invert_yaxis()
figManager = plt.get_current_fig_manager()
figManager.window.showMaximized()
plt.show()
