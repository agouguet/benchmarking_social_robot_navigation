
from collections import defaultdict
import time
from matplotlib import pyplot as plt
import matplotlib


import numpy as np
from shapely import GeometryCollection, MultiPolygon, delaunay_triangles
from shapely.geometry import Point, Polygon, LineString, box
from shapely.plotting import plot_line, plot_points, plot_polygon
from shapely.ops import triangulate

import calfem.core as cfc
import calfem.geometry as cfg
import calfem.mesh as cfm
import calfem.vis_mpl as cfv
import calfem.utils as cfu

from mbsn.model.polygon.environment_state import EnvironmentState
from mbsn.utils.map_loader import load_map
from mbsn.model.polygon.NavPolygonMDP import NavPolygonMDP
from mbsn.model.polygon.human import Human
from mbsn.polygon.nav_map import NavMap
from mbsn.solver.ValueIteration import ValueIteration


SCENARIO = "door_passing"

robot_pos = Point(12.0, 0.0)
print("ROBOT =", robot_pos)

human1 = Human(Point(-12.0, 0.0))
human2 = Human((2.0, -12.0))
human3 = Human((-0.0, -11.75))

goal_pos = Point(-12, 0.0)
print("GOAL =", goal_pos)


map_polygon = NavMap(SCENARIO, type="square")

matplotlib.use('Qt5Agg')
fig, ax = plt.subplots()

map_polygon.plot(ax)

for id, room in map_polygon.rooms.items():
    cells = room.get_cells_from_grid(map_polygon.grid)
    for cell in cells:
        cell.plot(ax=ax, add_id=True)

# for cell in map_polygon.grid.values():
#     cell.plot(ax=ax, add_id=True)

ax.axis('off')
ax.set_aspect('equal', adjustable='box')
ax.invert_yaxis()
plt.show()


test = EnvironmentState(map_polygon)
test.start_navigation(goal_pos)

# test.human_updated([human1, human2, human3])
action = test.robot_position_updated(Point(12, 0.0))
# action = test.robot_position_updated(Point(0, 4.0))
next_cell = test.local_mdp.polygons[action]

matplotlib.use('Qt5Agg')


i = 0
while 1:
    if i % 5 == 0 and i != 0:
        fig, ax = plt.subplots()
        test.plot(ax)
        # ax.set_xlim(test.current_pos.x-8, test.current_pos.x+8)
        # ax.set_ylim(test.current_pos.y-8, test.current_pos.y+8)
        ax.axis('off')
        ax.set_aspect('equal', adjustable='box')
        ax.invert_yaxis()
        figManager = plt.get_current_fig_manager()
        figManager.window.showMaximized()
        plt.show()

    t = time.time()

    human1.move(1.0, 0.0)
    human2.move(-0.5, 0.0)
    test.human_updated([human1, human2, human3])

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
