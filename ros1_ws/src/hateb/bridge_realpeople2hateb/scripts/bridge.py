#!/usr/bin/python3

import sys
import rospy
import time
import tf
import tf2_ros
import yaml
from cohan_msgs.msg import TrackedAgents, TrackedAgent, TrackedSegment, TrackedSegmentType, AgentType
from geometry_msgs.msg import PoseStamped, PoseArray, Twist
from agents_msgs.msg import AgentArray
from nav_msgs.msg import Odometry
import message_filters

PREFIX_AGENT = "Agent"

class StageAgents(object):

    def __init__(self):
        rospy.init_node('Stage_Agents', anonymous=True)
        self.tracked_agents_pub = []
        self.Segment_Type = TrackedSegmentType.TORSO
        self.agents = TrackedAgents()
        self.robot = TrackedAgent()
        self.listener = tf.TransformListener()
        time.sleep(3.0)

    def AgentsPub(self):
        # Subscibe to human agents
        agent_sub = message_filters.Subscriber("/agents", AgentArray)
        pose_msg = message_filters.TimeSynchronizer([agent_sub], 10)
        pose_msg.registerCallback(self.AgentsCB)

        self.tracked_agents_pub = rospy.Publisher("tracked_agents", TrackedAgents, queue_size=1)
        rospy.Timer(rospy.Duration(0.02), self.publishAgents)
        rospy.spin()

    def AgentsCB(self,msg):
        tracked_agents = TrackedAgents()
        i = 0
        for agent in msg.agents:
            i+=1
            p = PoseStamped()
            p.header = agent.header
            p.pose = agent.pose

            self.listener.waitForTransform("/map", "/camera_link", rospy.Time(), rospy.Duration(4.0))
            mpose = self.listener.transformPose("/map", p)

            agent_segment = TrackedSegment()
            agent_segment.type = self.Segment_Type
            agent_segment.pose.pose = mpose.pose
            agent_segment.twist.twist = agent.velocity
            tracked_agent = TrackedAgent()
            tracked_agent.type = AgentType.HUMAN
            tracked_agent.name = PREFIX_AGENT+"_"+str(i)
            tracked_agent.segments.append(agent_segment)
            tracked_agents.agents.append(tracked_agent)
        if(tracked_agents.agents):
            self.agents = tracked_agents

    def publishAgents(self, event):
        self.agents.header.stamp = rospy.Time.now()
        self.agents.header.frame_id = "map"
        for agent_id in range(0, len(self.agents.agents)):
            self.agents.agents[agent_id].track_id = agent_id+1
        self.tracked_agents_pub.publish(self.agents)


if __name__ == '__main__':
    agents = StageAgents()
    agents.AgentsPub()
