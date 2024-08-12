#!/usr/bin/env python3
import os
import time

from matplotlib import pyplot as plt
from mbsn.solver.MCTS import MBSNAgentMCTS
from mbsn.solver.multi_armed_bandit.ucb import UpperConfidenceBounds
from mbsn.solver.qtable import QTable
import rclpy # type: ignore

from shapely import LineString, Point
from shapely.ops import nearest_points
from geometry_msgs.msg import Pose, Point as RosPoint # type: ignore
from mbsn.interpreter.environment_interpreter import EnvironmentInterpreter
from mbsn.model.polygon.NavPolygonMDP import NavPolygonMDP
from mbsn.model.polygon.NavRoomByVisibilityWithHumanMDP import NavRoomByVisibilityWithHumanMDP
from mbsn.model.polygon.State import State
from mbsn.polygon.nav_map import NavMap
from mbsn.solver.ValueIteration import ValueIteration
from mbsn.utils.util import astar, ros_point_to_shapely_point
from geometry_msgs.msg import PoseStamped # type: ignore
from visualization_msgs.msg import Marker, MarkerArray # type: ignore
from std_msgs.msg import Header # type: ignore

MIN_TIME_OCCUPANCY = 10
TIME_OCCUPANCY = 30

class EnvironmentInterpreterToPolygon(EnvironmentInterpreter):

    def __init__(self):
        super().__init__()

        self.map_polygon = None

        self.global_mdp = None
        self.global_policy = None
        self.global_state = None

        self.local_mdp = None
        self.local_state = None
        self.local_state_timed = time.time()
        self.local_qfunction = QTable(default=-1e6)
        self.local_bandit = UpperConfidenceBounds()
        self.can_publish_local_goal = True

        self.polygon_publisher_ = self.create_publisher(MarkerArray, 'polygon_map', 10)

    def scene_info_callback(self, msg_scene_info):
        if self.scenario != msg_scene_info.environment.lower():
            self.scenario = msg_scene_info.environment.lower()
            self.get_logger().info('Scenario received: ' + str(self.scenario))

            self.map_polygon = NavMap(self.scenario, type="hexagon")
            rooms = self.map_polygon.rooms
            self.global_mdp = NavPolygonMDP(rooms)
            self.global_solver = ValueIteration(self.global_mdp, 0.99)

    def goal_callback(self, msg_goal):
        pos = msg_goal.pose.position
        goal = ros_point_to_shapely_point(pos)
        if self.global_mdp is not None and self.goal != goal:
            self.goal = goal
            self.get_logger().info('Goal received: ' + str(self.goal))
            self.global_mdp.goal = self.goal
            self.global_solver.train()
            self.global_policy = self.global_solver.policy

    def robot_odom_callback(self, msg_odom):
        if self.global_policy is not None:
            robot_position = msg_odom.pose.pose.position
            robot_position = ros_point_to_shapely_point(robot_position)
            self.current_pos = robot_position

            global_state = self.global_mdp.get_state_from_continuous_position(self.current_pos)

            if global_state != self.global_state:
                self.global_state = global_state
                self.global_state_updated()

            if self.local_mdp is not None:
                occupied_polygon = self.get_occupied_polygon()
                local_state = State(self.local_mdp.get_state_from_continuous_position(self.current_pos), occupied_polygon)
                if local_state != self.local_state and self.can_publish_local_goal:
                    self.local_state_updated()
            else:
                self.local_state_updated()

    def global_state_updated(self):
        self.global_action = self.global_policy[self.global_state]

    def local_state_updated(self):
        self.can_publish_local_goal = False
        self.local_mdp = NavRoomByVisibilityWithHumanMDP(self.map_polygon, self.current_pos, discount_factor=1.0, social_factor=1.0)
        self.publish_polygon_map_vizualisation()

        # A* to find the local goal
        grid = self.map_polygon.grid
        current_state = get_cell_id_in_dict_from_continuous_position(grid, self.current_pos)
        goal_state = get_cell_id_in_dict_from_continuous_position(grid, self.global_mdp.goal)
        astar_path = astar(current_state, goal_state, grid)
        local_goal_id = astar_path[0]
        for cell_id in astar_path:
            if cell_id in self.local_mdp.polygons:
                local_goal_id = cell_id
        self.local_goal_id = local_goal_id
        self.local_mdp.goal = self.local_mdp.polygons[local_goal_id][0].centroid

        # Human
        occupied_polygon = self.get_occupied_polygon()

        local_state = State(self.local_mdp.get_state_from_continuous_position(self.current_pos), occupied_polygon)

        # print(self.map_polygon.grid[155].neighbors)

        # print(local_state)
        # for a in self.local_mdp.get_actions(local_state):
        #     print("    ", a)
        #     for (new_state, probability) in self.local_mdp.get_transitions(local_state, a):
        #         print("        ", new_state, probability, self.local_mdp.get_reward(local_state, a, new_state))


        self.local_state = local_state
        self.local_state_timed = time.time()
        self.get_logger().info('New local state: ' + str(self.local_state))

        # MCTS
        self.local_solver = MBSNAgentMCTS(self.local_mdp, self.local_qfunction, self.local_bandit)
        
        root_node, _ = self.local_solver.mcts(local_state, timeout=2.0)
        self.local_action, _ = root_node.get_value()
        # self.robot_path.append(self.local_action)

        # print(self.local_solver.qfunction.qtable)

        for (s, a), v in self.local_solver.qfunction.qtable.items():
            print(s, a, v)

        local_goal = self.local_mdp.polygons[self.local_action][0].centroid
        p1, p2 = nearest_points(self.local_mdp.polygons[self.local_action][0].polygon, self.local_mdp.polygons[local_goal_id][0].centroid)
        print(p1, p2)

        self.get_logger().info('New local goal: ' + str(self.local_mdp.polygons[self.local_action][0].id))
        self.publish_local_goal(p1)


    def get_occupied_polygon(self):
        occupied_polygon = {poly_id:0 for poly_id in self.local_mdp.polygons.keys()}
        for h in self.humans:
            human_state = self.local_mdp.get_state_from_continuous_position(h.position)
            if human_state is not None:
                occupied_polygon[human_state] = 1
        return occupied_polygon

    def humans_position_callback(self, msg_position):
        pass

    def humans_callback(self, msg_agents):
        pass
    
    def update(self, new_state):
        pass

    def publish_local_goal(self, position):
        
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        msg.pose.position.x = float(position.x)
        msg.pose.position.y = float(-position.y)

        self.goal_publisher_.publish(msg)
        self.can_publish_local_goal = True


    def publish_polygon_map_vizualisation(self):
        self.clear_polygon_map_markers()
        marker_array = MarkerArray()
        markers = []

        id = 0

        for poly in self.local_mdp.polygons.values():
        # poly = list(self.local_mdp.polygons.values())[0]
            for p in poly:
                b = p.boundary.coords
                linestrings = [LineString(b[k:k+2]) for k in range(len(b) - 1)]
                for l in linestrings:
                    marker = Marker()

                    marker.header = Header()
                    marker.header.stamp = self.get_clock().now().to_msg()
                    marker.header.frame_id = "map"

                    marker.id = id = id+1
                    marker.type = marker.LINE_STRIP

                    marker.color.r = 1.0
                    marker.color.g = 0.0
                    marker.color.b = 0.0
                    marker.color.a = 1.0
                    marker.scale.x = 0.03
                    marker.scale.y = 0.03
                    marker.scale.z = 0.03

                    a, b = l.boundary.geoms

                    p1 = RosPoint()
                    p1.x = float(a.x)
                    p1.y = float(-a.y)

                    p2 = RosPoint()
                    p2.x = float(b.x)
                    p2.y = float(-b.y)

                    points = [p1, p2]

                    marker.points = points
                    markers.append(marker)
                    id += 1
                
                # TEXT
                marker = Marker()
                marker.header = Header()
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.header.frame_id = "map"

                marker.id = id = id+1
                marker.type = marker.TEXT_VIEW_FACING     

                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 0.8
                marker.scale.x = 0.15
                marker.scale.y = 0.15
                marker.scale.z = 0.15

                pose = Pose()
                c = p.centroid
                pose.position.x = float(c.x)
                pose.position.y = float(-c.y) - 0.15
                pose.position.z = 1.0
                marker.pose = pose
                marker.text = str(p.id)
                markers.append(marker)
            # print([list(ls.coords) for ls in linestrings])

        marker_array.markers = markers

        self.polygon_publisher_.publish(marker_array)

    def clear_polygon_map_markers(self):
        markers = MarkerArray()
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.action = marker.DELETEALL
        markers.markers.append(marker)
        self.polygon_publisher_.publish(markers)

def get_cell_id_in_dict_from_continuous_position(grid_dict, position):
    if not isinstance(position, Point):
        position = Point(position)

    if isinstance(grid_dict, dict):
        for id, polygon in grid_dict.items():
            if polygon.buffer(0.1).contains(position):
                return id
    return None

def main(args=None):
    rclpy.init(args=args)

    mbsn_node = EnvironmentInterpreterToPolygon()

    rclpy.spin(mbsn_node)

    mbsn_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()