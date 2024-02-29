#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import csv
import time
import argparse
import numpy as np
import pathlib
import json

from nav_msgs.msg import Path
from std_msgs.msg import Float32MultiArray, Float32
from metric_msgs.msg import TrialInfo
from simulation_msgs.msg import SceneInfo
from agents_msgs.msg import AgentArray

from collections import defaultdict

PREFIX_PATH = "./analyse/data/"
CONDITION = "navstack"

class TrialInfoListener(Node):

    def __init__(self):
        super().__init__('trial_info_listener')

        self.declare_parameter('method', rclpy.Parameter.Type.STRING)

        self.method = self.get_parameter('method').value

        self.TRIAL_ID = 0 # used to disambiguate trials in the same csv (and their jsons)
        self.latest_robot_poses = None
        self.latest_robot_poses_ts = None

        ts = int(time.time() * 1000)

        self.prefix = PREFIX_PATH
        self.condition = CONDITION
        
        self.name = f"{self.condition}-{ts}"
        self.filename = None

        self.create_subscription(TrialInfo, 'social_sim/metrics', self.trial_info_callback, 10)
        self.create_subscription(AgentArray, 'social_sim/agents', self.human_position, 10)
        self.create_subscription(SceneInfo, 'social_sim/scene_info', self.scene_info_callback, 10)
        self.create_subscription(Path, 'move_base/GlobalPlanner/plan', self.global_plan_callback, 10)
        self.create_subscription(Float32MultiArray, 'lifecycle_learner/attention_l', self.attention_l_callback, 10)
        self.create_subscription(Float32MultiArray, 'lifecycle_learner/attention_cmd_vel', self.attention_cmd_vel_callback, 10)
        
        self.headers = False

        self.environment = ""
        self.global_plan = None
        self.l_n = 0
        self.attention_l = None
        self.cmd_vel_n = 0
        self.attention_cmd_vel = None

        self.scenario = None
        print("ready")

    def init_files(self):
        self.filename = pathlib.Path(f"{self.prefix}/{self.method.upper()}")
        self.filename.mkdir(exist_ok=True, parents=True)
        self.filename = self.filename.joinpath(f"{self.environment.lower()}.csv")
        print(f"Results will be written to: {self.filename}")

        # create and clear the file
        with open(self.filename, mode='w') as f:
            f.write('')

        self.json_file = pathlib.Path(f"{self.prefix}/{self.method.upper()}/json")
        self.json_file.mkdir(exist_ok=True, parents=True)
        self.json_file = self.json_file.joinpath(f"{self.environment.lower()}.json")

        self.json_file_human_pos = pathlib.Path(f"{self.prefix}/{self.method.upper()}/json")
        self.json_file_human_pos.mkdir(exist_ok=True, parents=True)
        self.json_file_human_pos = self.json_file_human_pos.joinpath(f"{self.environment.lower()}_human_pos.json")

        print(self.filename)
        print(self.json_file)
        print(self.json_file_human_pos)

        with open(self.json_file_human_pos, 'w') as outfile:
            poses = {'data':[]}
            json.dump(poses, outfile)

    def scene_info_callback(self, msg):
        self.environment = msg.environment
        self.scenario = msg.scenario_name
        if self.filename == None:
            self.init_files()

    def global_plan_callback(self, msg):

        # get the x,y,z for rotation and x,y,z,w for orientation
        def _format_pose(pose_stamped):
            x, y, z = pose_stamped.pose.position.x, pose_stamped.pose.position.y, pose_stamped.pose.position.z
            ox, oy, oz, ow = pose_stamped.pose.orientation.x, pose_stamped.pose.orientation.y, pose_stamped.pose.orientation.z, pose_stamped.pose.orientation.w

            return f"{x},{y},{z}<>{ox},{oy},{oz},{ow}"

        self.global_plan = ":".join([_format_pose(pose_stamped) for pose_stamped in msg.poses])

    def attention_l_callback(self, msg):
        data = np.array(msg.data)
        if self.attention_l is None:
            self.attention_l = data
            return
        self.l_n += 1
        self.attention_l = self.attention_l * (self.l_n-1)/self.l_n + data/self.l_n


    def attention_cmd_vel_callback(self, msg):
        data = np.array(msg.data)
        if self.attention_cmd_vel is None:
            self.attention_cmd_vel = data
            return
        self.cmd_vel_n += 1
        self.attention_cmd_vel = self.attention_cmd_vel * (self.cmd_vel_n-1)/self.cmd_vel_n + data/self.cmd_vel_n

    def trial_info_callback(self, msg):
        if self.scenario == None or self.filename == None:
            return
        # Increment the trial ID every time we get a new set of robot poses
        # which will be shorter than the latest list of robot poses
        if(self.latest_robot_poses is not None and len(self.latest_robot_poses) > len(msg.robot_poses)):
            self.TRIAL_ID += 1

        self.latest_robot_poses = msg.robot_poses
        self.latest_robot_poses_ts = msg.robot_poses_ts

        with open(self.json_file, 'w') as outfile:
            poses = {'data':[]}
            for pose, ts in zip(self.latest_robot_poses, self.latest_robot_poses_ts):
                poses['data'].append({
                    'secs':ts.sec,
                    'nsecs':ts.nanosec,
                    'position':{
                        'x':pose.position.x,
                        'y':pose.position.y,
                        'z':pose.position.z
                    },
                    'orientation':{
                        'x':pose.orientation.x,
                        'y':pose.orientation.y,
                        'z':pose.orientation.z,
                        'w':pose.orientation.w,
                    }
                })

            json.dump(poses, outfile)

        with open(self.filename, mode='a') as csv_file:
            row = {

                # Trial ID
                'trial_id': self.TRIAL_ID,

                # Fields from other callbacks
                'name': self.name,
                'condition': self.condition,
                'environment': self.environment,
                'scenario': self.scenario,
                'mean_attention_l': self.attention_l,
                'mean_attention_cmd_vel': self.attention_cmd_vel,

                # Header
                'secs': msg.header.stamp.sec,
                'nsecs': msg.header.stamp.nanosec,

                # Information about the current interaction
                'trial_start': msg.trial_start,
                'timeout_time': msg.timeout_time,
                'trial_name': msg.trial_name,
                'trial_number': msg.trial_number,
                'num_actors': msg.num_actors,

                # Robot start / goal locations
                'robot_start_position': f"{msg.robot_start.position.x},{msg.robot_start.position.y},{msg.robot_start.position.z}",
                'robot_start_orientation': f"{msg.robot_start.orientation.x},{msg.robot_start.orientation.y},{msg.robot_start.orientation.z},{msg.robot_start.orientation.w}",
                'robot_goal_position': f"{msg.robot_goal.position.x},{msg.robot_goal.position.y},{msg.robot_goal.position.z}",
                'robot_goal_orientation': f"{msg.robot_goal.orientation.x},{msg.robot_goal.orientation.y},{msg.robot_goal.orientation.z},{msg.robot_goal.orientation.w}",

                # Robot location / distance relative to start / goal
                'dist_to_target': msg.dist_to_target,
                'min_dist_to_target': msg.min_dist_to_target,

                # Robot location relative to pedestrians
                'min_dist_to_ped': msg.min_dist_to_ped,

                # Collisions between robots and people
                'robot_on_person_intimate_dist_violations': msg.robot_on_person_intimate_dist_violations,
                'person_on_robot_intimate_dist_violations': msg.person_on_robot_intimate_dist_violations,
                'robot_on_person_personal_dist_violations': msg.robot_on_person_personal_dist_violations,
                'person_on_robot_personal_dist_violations': msg.person_on_robot_personal_dist_violations,
                'robot_on_person_collisions': msg.robot_on_person_collisions,
                'person_on_robot_collisions': msg.person_on_robot_collisions,

                # Collisions w/ static objects
                'obj_collisions': msg.obj_collisions,

                'path_length': msg.path_length,
                'path_irregularity': msg.path_irregularity,
                'time_not_moving': msg.time_not_moving,
                'time_in_personal_space': msg.time_in_personal_space,
                'minimum_time_to_collision' : msg.minimum_time_to_collision,
                'movement_jerk' : msg.movement_jerk,

                # Current/latest global plan
                'global_plan': self.global_plan
            }
            #print(f"writing row: {row}")
            writer = csv.DictWriter(csv_file, fieldnames=list(row.keys()))
            if not self.headers:
                writer.writeheader()
                self.headers = True
            writer.writerow(row)
        #print(f"file: {self.filename}")
        #print(f"json file: {self.json_file}")


    def human_position(self, msg):
        if self.filename != None :
            if self.json_file_human_pos != None:
                with open(self.json_file_human_pos, "r+") as file:
                    data = json.load(file)
                    data_to_append = {}
                    data_to_append['secs'] = msg.header.stamp.sec
                    data_to_append['nsecs'] = msg.header.stamp.nanosec
                    humans = {}
                    for human_id in range(len(msg.agents)):
                        human = {}
                        position = {}
                        position['x'] = msg.agents[human_id].pose.position.x
                        position['y'] = msg.agents[human_id].pose.position.y
                        position['z'] = msg.agents[human_id].pose.position.z
                        human['position'] = position
                        orientation = {}
                        orientation['x'] = msg.agents[human_id].pose.orientation.x
                        orientation['y'] = msg.agents[human_id].pose.orientation.y
                        orientation['z'] = msg.agents[human_id].pose.orientation.z
                        orientation['w'] = msg.agents[human_id].pose.orientation.w
                        human['orientation'] = orientation

                        human['visible_by_robot'] = msg.agents[human_id].visible_by_robot
                        humans['human_'+str(human_id)] = human
                    data_to_append['human_position'] = humans
                    data['data'].append(data_to_append)
                    file.seek(0)
                    json.dump(data, file)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = TrialInfoListener()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()