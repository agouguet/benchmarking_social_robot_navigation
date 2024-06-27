#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import Header
from geometry_msgs.msg import Point
from agents_msgs.msg import AgentArray, AgentTrajectories, AgentTrajectory

TIME_INTERVAL = 1.5 # seconds
FUTURE_POS_NUMBER = 3

class AgentTrajectoryPublisher(Node):

    def __init__(self):
        super().__init__('AgentTrajectoryPublisher')

        self.agents_subscription_ = self.create_subscription(AgentArray, 'social_sim/agents', self.agents_callback, 10)
        self.agents_trajectories_publisher_ = self.create_publisher(AgentTrajectories, 'agent/trajectories', 10)


    def agents_callback(self, msg_agents):
        trajectories_msg = AgentTrajectories()
        trajectories = []
        for agent in msg_agents.agents:
            if(agent.visible_by_robot):
                trajectory = AgentTrajectory()
                trajectory.header =  Header()
                trajectory.header.stamp = self.get_clock().now().to_msg()
                trajectory.id = agent.id
                vel = agent.velocity.linear
                poses = []
                for t in range(1, FUTURE_POS_NUMBER+1):
                    pose = Point()
                    pose.x = agent.pose.position.x + (t*TIME_INTERVAL) * vel.x
                    pose.y = agent.pose.position.y + (t*TIME_INTERVAL) * vel.y
                    pose.z = agent.pose.position.z + (t*TIME_INTERVAL) * vel.z
                    poses.append(pose)
                trajectory.poses = poses
                trajectories.append(trajectory)

        trajectories_msg.header =  Header()
        trajectories_msg.header.stamp = self.get_clock().now().to_msg()
        trajectories_msg.trajectories = trajectories
        self.agents_trajectories_publisher_.publish(trajectories_msg)



def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = AgentTrajectoryPublisher()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()