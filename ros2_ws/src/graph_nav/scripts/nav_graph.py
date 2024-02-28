#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseArray
from actionlib_msgs.msg import GoalStatusArray
from nav_msgs.msg import Odometry
from agent_trajectory_prediction.msg import AgentTrajectories

from graph_nav.solver.ValueIteration import ValueIteration

from graph_nav.model.graph.GraphWorld import GraphWorld
from graph_nav.model.graph_boolean.GraphWorldBoolean import GraphWorldBoolean

from graph_nav.msg import GraphNav
from graph_nav.msg import GraphEdge
from graph_nav.msg import GraphNode

from graph_nav.model.graph.GraphState import GraphState
from graph_nav.model.graph_boolean.GraphStateBoolean import GraphStateBoolean

import networkx as nx
from scipy.spatial import distance


"""
# -----------------------------------------------------
#       s1 - - - - - s5   |   s7 - - - - - s8
#                   |
       
s0            s3          s6        s9              s12

#                   |
#       s4 - - - - - s2   |   s10 - - - - - s11
# -----------------------------------------------------

"""
NODES_EXAMPLE_1 = { 0: (-11 , 0 , -1  , 0),
                1: (-8  , 1.2 , -1  , 0),
                2: (-3  , -1.2, -1  , 0),
                3: (-5  , 0 , -1  , 0),
                4: (-8  , -1.2, -1  , 0),
                5: (-3  , 1.2 , -1  , 0),
                6: (0   , 0 , -1  , 0),
                7: (3   , 1.2 , -1  , 0),
                8: (8   , 1.2 , -1  , 0),
                9: (5   , 0 , -1  , 0),
                10: (3  , -1.2, -1  , 0),
                11: (8  , -1.2, -1  , 0),
                12: (11 , 0 , 10  , 1)
        }

EDGES_EXAMPLE_1 = {
            (0, 1): 1    , (0, 3): 1.5 , (0, 4): 1,
            (1, 3): 1    , (1, 4): 1.5 , (1, 5): 1.5,
            (2, 3): 1    , (2, 4): 1.5 , (2, 5): 1.5 , (2, 6): 1.5,
            (3, 4): 1    , (3, 5): 1   , (3, 6): 1.5,
            (5, 6): 1.5,
            (6, 7): 1.5    , (6, 9): 1.5 , (6, 10): 1.5,
            (7, 8): 1.5  , (7, 9): 1   , (7, 10): 1.5,
            (8, 9): 1    , (8, 11): 1.5, (8, 12): 1,
            (9, 10): 1   , (9, 11): 1  , (9, 12): 1.5,
            (10, 11): 1.5,
            (11, 12): 1,
        }

NODES_EXAMPLE_2 = {       0: (-11 , 0 , -1  , 0),
                1: (-8  , 1.2 , -1  , 0),
                2: (-3  , -1.2, -1  , 0),
                3: (-5  , 0 , -1  , 0),
                4: (-8  , -1.2, -1  , 0),
        }

EDGES_EXAMPLE_2 = {
            (0, 1): 1    , (0, 3): 1.5 , (0, 4): 1,
            (1, 3): 1    , (1, 4): 1.5 ,
            (2, 3): 1    , (2, 4): 1.5 ,
            (3, 4): 1    ,
        }

HUMAN_PATH_EXAMPLE_1 = [12, 9, 6, 3, 0]

HUMAN_PATH = {
    12: 9,
    11: 9,
    8: 9,
    9: 6,
    10: 6,
    7: 6,
    6: 3,
    2: 3,
    5: 3,
    3: 0,
    1: 0,
    4: 0,
    0: 0
}

HUMAN_NUMBER = 3

DISTANCE_LIMIT_OCCUPIED_NODE = 0.8
DISTANCE_LIMIT_ROBOT_NEAREST_NODE = 1.0

def create_graph(nodes, edges):

        G = nx.Graph()
        for n, p in nodes.items():
            G.add_node(n, pos=(p[0], p[1]), weight=p[2], is_terminal=p[3])

        for e, w in edges.items():
            G.add_edge(e[0], e[1], weight=w)

        for n in nodes.keys():
            G.add_edge(n, n, weight=0.5)

        return G


class GraphNavNode(Node):

    def __init__(self, publish_graph = True):
        super().__init__('GraphNav')
        print("Creation MDP...")
        self.graph = create_graph(NODES_EXAMPLE_1, EDGES_EXAMPLE_1)
        #self.problem = GraphWorld(self.graph, HUMAN_PATH)
        self.problem = GraphWorldBoolean(self.graph, HUMAN_NUMBER)

        self.robot_position_subscription_ = self.create_subscription(Odometry, 'robot_odom', self.robot_odom_callback, 10)
        self.humans_position_subscription_ = self.create_subscription(PoseArray, 'social_sim/agent_positions', self.humans_position_callback, 10)
        self.humans_trajectories_subscription_ = self.create_subscription(AgentTrajectories, 'agent/trajectories', self.humans_trajectories_callback, 10)
        self.goal_status_subscription_ = self.create_subscription(GoalStatusArray, 'move_base/status', self.goal_status_callback, 10)
        self.goal_publisher_ = self.create_publisher(PoseStamped, 'goal_pose', 10)

        if publish_graph:
            self.graph_publisher_ = self.create_publisher(GraphNav, 'graph', 10)
            timer_period = 0.5
            self.graph_publisher_timer = self.create_timer(timer_period, self.graph_publish_callback)

        print("Value Iteration...")
        self.solver = ValueIteration(self.problem, gamma=0.9)
        self.solver.train()

        self.robot_state = 0
        self.humans_position = []
        self.human_trajectories = []

        self.current_state = (self.robot_state, ())

        self.policy = self.solver.policy

        self.publish_goal(self.policy[GraphStateBoolean(self.current_state[0], self.current_state[1])])

        #print(self.problem.__str__reward__())

        #print(self.policy)

    def euclidean_distance_from_node(self, position_robot, node):
        return distance.euclidean((position_robot.x, position_robot.y), self.graph.nodes(data=True)[node]['pos'])

    def get_current_state_robot(self, position):
        state = self.robot_state
        for n, p in self.graph.nodes.items():
            if(self.euclidean_distance_from_node(position, n) < self.euclidean_distance_from_node(position, state)):
                state = n
        if self.euclidean_distance_from_node(position, state) < DISTANCE_LIMIT_ROBOT_NEAREST_NODE:
                return state
        return self.robot_state

    def get_current_state_humans(self, positions):
        state = ()
        for pos in positions:
            state_of_pos = 0
            for n, p in self.graph.nodes.items():
                if(self.euclidean_distance_from_node(pos, n) < self.euclidean_distance_from_node(pos, state_of_pos)):
                    state_of_pos = n
            if self.euclidean_distance_from_node(pos, state_of_pos) < DISTANCE_LIMIT_OCCUPIED_NODE:
                state = state + (state_of_pos,)
        return state

    def robot_odom_callback(self, msg_odom):
        robot_position = msg_odom.pose.pose.position
        self.robot_state = self.get_current_state_robot(robot_position)
        humans_state = self.get_current_state_humans(self.humans_position)
        humans_traj = self.get_current_state_humans(self.human_trajectories)
        occupied_node = humans_state + humans_traj
        occupied_node = tuple(sorted(set(occupied_node))) # remove duplicates and sort

        current_state = (self.robot_state, occupied_node)

        if(current_state != self.current_state):
            print("NEW STATE:", self.current_state, "->", current_state)
            self.current_state = current_state
            action = self.policy[GraphStateBoolean(self.robot_state, occupied_node)]
            print("NEW GOAL:", action._node)
            self.publish_goal(action)

    def humans_position_callback(self, msg_position):
        self.humans_position = []
        for point in msg_position.poses:
            self.humans_position.append(point.position)

    def humans_trajectories_callback(self, msg_agent_trajectories):
        self.human_trajectories = []
        for trajectory in msg_agent_trajectories.trajectories:
            for pose in trajectory.poses:
                self.human_trajectories.append(pose)

    def goal_status_callback(self, msg_goal_status):
        # print(msg_goal_status)
        # self.publish_goal(list(self.solver.policy.values())[0])
        pass

    def publish_goal(self, action):
        n = action._node
        x = self.graph.nodes(data=True)[n]['pos'][0]
        y = self.graph.nodes(data=True)[n]['pos'][1]

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)

        self.goal_publisher_.publish(msg)

    def graph_publish_callback(self):
        msg = GraphNav()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        edges = []
        nodes = []

        for n in self.graph.nodes():
            node_msg = GraphNode()
            node_msg.id = n
            node_msg.x = float(self.graph.nodes(data=True)[n]['pos'][0])
            node_msg.y = float(self.graph.nodes(data=True)[n]['pos'][1])
            nodes.append(node_msg)

        for e, w in self.graph.edges.items():
            edge = GraphEdge()
            edge.id_n1 = e[0]
            edge.id_n2 = e[1]
            edges.append(edge)

        msg.nodes = nodes
        msg.edges = edges

        self.graph_publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = GraphNavNode()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()