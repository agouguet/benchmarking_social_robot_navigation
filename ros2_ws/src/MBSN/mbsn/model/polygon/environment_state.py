
from collections import defaultdict
import random
import time
from matplotlib import pyplot as plt
import matplotlib
import numpy as np
from shapely import MultiPolygon, Point, Polygon
from mbsn.model.polygon.NavRoomByVisibilityWithHumanMDP import NavRoomByVisibilityWithHumanMDP
from mbsn.model.polygon.NavPolygonMDP import NavPolygonMDP
from mbsn.model.polygon.State import State
from mbsn.model.trajectory_prediction.human_trajctory_prediction_model import pecnet_human_trajectory_prediction, simple_human_trajectory_prediction
from mbsn.solver.MCTS import MBSNAgentMCTS, MBSNAgentNode
from mbsn.solver.ValueIteration import ValueIteration
from mbsn.solver.heuristicfunction import heuristic_score_based, heuristic_rules_based
from mbsn.solver.multi_armed_bandit.ucb import UpperConfidenceBounds
from mbsn.solver.qtable import QTable
from shapely.plotting import plot_line, plot_points, plot_polygon

from mbsn.utils.graph_visualisation import GraphVisualisation
from mbsn.utils.util import astar

colors = [plt.cm.hsv(i) for i in np.linspace(0, 1, 600)]
random.shuffle(colors)

class EnvironmentState():

    def __init__(self, map_polygon):
        self.map_polygon = map_polygon

        rooms = map_polygon.rooms
        self.global_mdp = NavPolygonMDP(rooms)
        self.global_solver = ValueIteration(self.global_mdp, 0.99)
        self.global_policy = None
        self.global_state = None
        self.global_goal = None

        self.local_mdp = None
        self.local_policy = None
        self.local_state = None
        self.local_goal = None
        self.local_qfunction = QTable(default=-1e6)
        self.local_bandit = UpperConfidenceBounds()

        self.current_pos = None
        self.humans = []
        self.robot_path = []

        self.plot_previous_position = []
        self.plot_previous_position_humans = defaultdict(list)

    def start_navigation(self, goal):
        self.global_goal_updated(goal)
        
    def robot_position_updated(self, pos):
        self.current_pos = pos
        self.plot_previous_position.append(self.current_pos)

        global_state = self.global_mdp.get_state_from_continuous_position(pos)
        
        if global_state != self.global_state:
            self.global_state = global_state
            self.global_state_updated()

        self.local_state_updated()

        return self.local_action

    def human_updated(self, humans):
        self.humans = humans
        for h in humans:
            self.plot_previous_position_humans[h.id].append(h.position)


    def global_goal_updated(self, goal):
        if goal != self.global_goal:
            self.global_goal = goal
            self.global_mdp.goal = self.global_goal
            self.global_solver.train()
            self.global_policy = self.global_solver.policy

    def global_state_updated(self):
        self.global_action = self.global_policy[self.global_state]
        
    def local_state_updated(self):
        
        self.local_mdp = NavRoomByVisibilityWithHumanMDP(self.map_polygon, 
                                                         self.current_pos, 
                                                        #  human_trajectory_prediction_function=simple_human_trajectory_prediction, 
                                                         human_trajectory_prediction_function=pecnet_human_trajectory_prediction,
                                                         global_goal=self.global_goal, 
                                                         distance_factor=3.0, 
                                                         social_factor=1.0)

        # A* to find the local goal
        # grid = self.map_polygon.grid
        grid = self.map_polygon.test_grid
        current_state = get_cell_id_in_dict_from_continuous_position(grid, self.current_pos)
        goal_state = get_cell_id_in_dict_from_continuous_position(grid, self.global_goal)
        # print(current_state)
        astar_path = astar(current_state, goal_state, grid)
        local_goal_id = astar_path[0]
        for cell_id in astar_path:
            if cell_id in self.local_mdp.polygons:
                local_goal_id = cell_id
        self.local_goal_id = local_goal_id
        self.local_mdp.goal = self.local_mdp.polygons[local_goal_id][0].centroid

        # Human
        # occupied_polygon = {poly_id:0 for poly_id in self.local_mdp.polygons.keys()}
        # for h in self.humans:
        #     human_state = self.local_mdp.get_state_from_continuous_position(h.position)
        #     if human_state is not None:
        #         occupied_polygon[human_state] = 1


        visible_human = []

        for h in self.humans:
            if h.position.distance(self.current_pos) <= 5.0:
                visible_human.append(h)

        # local_state = State(self.local_mdp.get_state_from_continuous_position(self.current_pos), occupied_polygon)
        local_state = State(self.local_mdp.get_state_from_continuous_position(self.current_pos), visible_human)

        # MCTS
        # self.local_solver = MBSNAgentMCTS(self.local_mdp, self.local_qfunction, self.local_bandit, heuristic_function=heuristic_score_based)
        
        # root_node, _ = self.local_solver.mcts(local_state, timeout=1.0)
        # self.local_action, _ = root_node.get_value()

        # print("Result ==== > ", furthest_from_human(self.local_mdp, local_state, debug=True))

        ### TEST ###
        self.local_action = heuristic_rules_based(self.local_mdp, local_state)
        # self.local_action = heuristic_score_based(self.local_mdp, local_state)

        
        self.robot_path.append(self.local_action)




    def update(self):
        return 
    

    def plot(self, ax, plot_circle_previous_pos=True):
        self.map_polygon.plot(ax=ax, add_points=False)

        # for id, cell in self.map_polygon.grid.items():
        #     cell.plot(ax=ax, add_id=False)
        #     plot_polygon(cell.polygon, ax=ax)

        # for id, room in self.global_mdp.polygons.items():
        #     room.plot(ax=ax)

        for poly in self.local_mdp.polygons.values():
            for p in poly:
                p.plot(ax=ax, add_id=False)

        if self.current_pos is not None:
            circle = plt.Circle((self.current_pos.x, self.current_pos.y), radius=0.15, color="orange")
            ax.add_patch(circle)
            label = ax.annotate("R", xy=(self.current_pos.x, self.current_pos.y), fontsize=10, ha="center", color="white", verticalalignment="center", horizontalalignment="center")
            # ax.plot(self.current_pos.x, self.current_pos.y, marker="o",  markersize=10)

        if self.global_goal is not None:
            circle = plt.Circle((self.global_goal.x, self.global_goal.y), radius=0.2, color="purple")
            ax.add_patch(circle)
            label = ax.annotate("G", xy=(self.global_goal.x, self.global_goal.y), fontsize=10, ha="center", color="white", verticalalignment="center", horizontalalignment="center")

        if self.local_mdp.goal is not None:
            circle = plt.Circle((self.local_mdp.goal.x, self.local_mdp.goal.y), radius=0.2, color="red")
            ax.add_patch(circle)
            label = ax.annotate("L", xy=(self.local_mdp.goal.x, self.local_mdp.goal.y), fontsize=10, ha="center", color="white", verticalalignment="center", horizontalalignment="center")

            if isinstance(self.local_mdp.visibility_polygon, Polygon):
                x, y = self.local_mdp.visibility_polygon.exterior.xy
                ax.fill(x, y, alpha=0.5, fc='g', ec='black')
            elif isinstance(self.local_mdp.visibility_polygon, MultiPolygon):
                for p in self.local_mdp.visibility_polygon.geoms:
                    x, y = p.exterior.xy
                    ax.fill(x, y, alpha=0.5, fc='g', ec='black')
            
        if plot_circle_previous_pos:
            for i in range(len(self.plot_previous_position)-1):
                p = self.plot_previous_position[i]
                circle = plt.Circle((p.x, p.y), radius=0.1, color="orange")
                ax.add_patch(circle)
                label = ax.annotate(str(i), xy=(p.x, p.y), fontsize=10, ha="center", color="white", verticalalignment="center", horizontalalignment="center")
                # ax.scatter(p.x, p.y, color="orange", zorder=1000)

        for i in range(1, len(self.plot_previous_position)-1):
            p1 = self.plot_previous_position[i-1]
            p2 = self.plot_previous_position[i]
            ax.plot((p1.x, p2.x), (p1.y, p2.y), color="orange", alpha=0.5, linewidth=4)


        for h in self.humans:
            h.plot(ax, color=colors[h.id])
            # for pos in h.future_predicted_position:
            #     circle = plt.Circle((pos.x, pos.y), radius=0.1, color=colors[h.id])
            #     ax.add_patch(circle)

        for id, positions in self.plot_previous_position_humans.items():
            if plot_circle_previous_pos:
                for i in range(len(positions)-1):
                    p = positions[i]
                    circle = plt.Circle((p.x, p.y), radius=0.1, color=colors[id], alpha=i/len(positions))
                    ax.add_patch(circle)
                    label = ax.annotate(str(i), xy=(p.x, p.y), fontsize=10, ha="center", color="white", verticalalignment="center", horizontalalignment="center", alpha=i/len(positions))
            
            for i in range(1, len(positions)-1):
                p1 = positions[i-1]
                p2 = positions[i]
                ax.plot((p1.x, p2.x), (p1.y, p2.y), color=colors[id], alpha=i/len(positions))
        
        handles, labels = ax.get_legend_handles_labels()
        ax.legend(loc='center left', bbox_to_anchor=(1, 0.5), handles=handles)

def get_cell_id_in_dict_from_continuous_position(grid_dict, position):
    if not isinstance(position, Point):
        position = Point(position)

    if isinstance(grid_dict, dict):
        for id, polygon in grid_dict.items():
            if polygon.buffer(0.1).contains(position):
                return id
    return None