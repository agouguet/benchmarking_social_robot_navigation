#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose
from agents_msgs.msg import AgentTrajectories

import networkx as nx


class AgentTrajectoryPredictionVisualization(Node):

    def __init__(self):
        super().__init__('Test_graph_msg')
        self.subscription = self.create_subscription(AgentTrajectories, 'agent/trajectories', self.listener_callback, 10)
        self.trajs_publisher_ = self.create_publisher(MarkerArray, 'agent/visual/trajectories', 10)
        self.i = 0

    def listener_callback(self, msg):
        trajs = msg.trajectories
        self.publish_trajectories(trajs)

    def publish_trajectories(self, trajs):
        marker_array = MarkerArray()
        markers = []
        i = 0
        for traj in trajs:
            for pose in traj.poses:
                marker = Marker()

                marker.header = Header()
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.header.frame_id = "map"

                marker.id = i
                marker.type = marker.SPHERE     

                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 1.0
                marker.color.a = 0.5
                marker.scale.x = 0.1
                marker.scale.y = 0.1
                marker.scale.z = 0.1

                p = Pose()
                p.position.x = pose.x
                p.position.y = pose.y
                p.position.z = 1.0
                marker.pose = p
                markers.append(marker)
                i += 1

        marker_array.markers = markers

        self.trajs_publisher_.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = AgentTrajectoryPredictionVisualization()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()