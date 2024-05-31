#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from simulation_msgs.msg import SceneInfo

import psutil
import subprocess
import shlex
import signal

class MapPublisherNode(Node):

    def __init__(self):
        super().__init__('MapPublisher')

        self.map = None
        self.pid = None

        self.scene_info_subscriber_ = self.create_subscription(SceneInfo, '/social_sim/scene_info', self.scene_info_callback, 10)
        self.get_logger().info("Ready to publish map", once=True)

    def scene_info_callback(self, msg_scene_info):
        _map = msg_scene_info.environment.lower()
        if self.map != _map:
            # self.stop_map()
            self.publish_map(_map)

    def ros_command(self, _map):
        cmd = "ros2 launch unity_sim map_publisher.launch.py map:={}".format(_map)
        return shlex.split(cmd)

    def publish_map(self, _map):
        process = subprocess.Popen(self.ros_command(_map))
        self.pid = process.pid
        self.map = _map
        self.get_logger().info("Started publishing map {}".format(self.map), once=False)

    def stop_map(self):
        if self.pid is None:
            return
        signal_process_and_children(self.pid, signal.SIGINT, wait=True)
        self.pid = None
        self.get_logger().info("Stopped publishing map", once=False)


def main(args=None):
    rclpy.init(args=args)

    map_publisher_node = MapPublisherNode()

    rclpy.spin(map_publisher_node)

    map_publisher_node.stop_map()
    map_publisher_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()