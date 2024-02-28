#!/usr/bin/python3

# Brief: This node subscribes to the robots published on /humani, i=1,2, .. and robot, and publishes /tracked_agents required for CoHAN
# Author: Phani Teja Singamaneni

import sys
import rospy
import time
from cohan_msgs.msg import TrackedAgents, TrackedAgent, TrackedSegment, TrackedSegmentType, AgentType
from simulation_msgs.msg import AgentArray
from nav_msgs.msg import Odometry
import message_filters

PREFIX_AGENT = "Agent"

class StageAgents(object):

    def __init__(self):
        self.tracked_agents_pub = []
        self.Segment_Type = TrackedSegmentType.TORSO
        self.agents = TrackedAgents()
        self.robot = TrackedAgent()

    def AgentsPub(self):
        rospy.init_node('Stage_Agents', anonymous=True)

        # Subscibe to human agents
        agent_sub = message_filters.Subscriber("/social_sim/agents", AgentArray)
        pose_msg = message_filters.TimeSynchronizer([agent_sub], 10)
        pose_msg.registerCallback(self.AgentsCB)

        self.tracked_agents_pub = rospy.Publisher("tracked_agents", TrackedAgents, queue_size=1)
        rospy.Timer(rospy.Duration(0.02), self.publishAgents)
        rospy.spin()

    def AgentsCB(self,*msg):
        tracked_agents = TrackedAgents()
        agent_array = msg[0]
        i = 0
        for agent in agent_array.agents:
            i+=1
            if (agent.visible_by_robot):
                agent_segment = TrackedSegment()
                agent_segment.type = self.Segment_Type
                agent_segment.pose.pose = agent.pose
                agent_segment.twist.twist = agent.twist
                tracked_agent = TrackedAgent()
                tracked_agent.type = AgentType.HUMAN
                tracked_agent.name = PREFIX_AGENT+"_"+str(agent.track_id)
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
