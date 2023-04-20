#!/usr/bin/env python3
import rospy
import csv
import time
import argparse
import numpy as np
import pathlib
import json

from nav_msgs.msg import Path
from std_msgs.msg import Float32MultiArray, Float32
from geometry_msgs.msg import PoseArray
from social_sim_ros.msg import TrialInfo, SceneInfo

from collections import defaultdict
class TrialInfoListener():


    def __init__(self):
        rospy.init_node("trial_info_listener")

        self.TRIAL_ID = 0 # used to disambiguate trials in the same csv (and their jsons)
        self.latest_robot_poses = None
        self.latest_robot_poses_ts = None

        ts = int(time.time() * 1000)

        prefix = rospy.get_param("~prefix")
        condition = rospy.get_param("~condition")

        self.prefix = prefix
        self.condition = condition
        self.filename = pathlib.Path(f"{prefix}/{self.condition}")
        self.filename.mkdir(exist_ok=True, parents=True)
        self.name = f"{self.condition}-{ts}"

        rospy.Subscriber('/social_sim/metrics', TrialInfo, self.trial_info_callback)
        rospy.Subscriber('/social_sim/agent_positions', PoseArray, self.human_position)
        rospy.Subscriber('/social_sim/scene_info', SceneInfo, self.scene_info_callback)
        rospy.Subscriber('/move_base/GlobalPlanner/plan', Path, self.global_plan_callback)
        rospy.Subscriber('/lifecycle_learner/attention_l', Float32MultiArray, self.attention_l_callback)
        rospy.Subscriber('/lifecycle_learner/attention_cmd_vel', Float32MultiArray, self.attention_cmd_vel_callback)

        self.rule_based = defaultdict(int) # calls int() to default to 0
        rospy.Subscriber('/social_sim/situations/rule_based/cross_path', Float32, self.rule_based_callback, callback_args='cross_path')
        rospy.Subscriber('/social_sim/situations/rule_based/down_path', Float32, self.rule_based_callback, callback_args='down_path')
        rospy.Subscriber('/social_sim/situations/rule_based/empty', Float32, self.rule_based_callback, callback_args='empty')
        rospy.Subscriber('/social_sim/situations/rule_based/join_group', Float32, self.rule_based_callback, callback_args='join_group')
        rospy.Subscriber('/social_sim/situations/rule_based/leave_group', Float32, self.rule_based_callback, callback_args='leave_group')

        self.filename = self.filename.joinpath(f"trial_info_{self.condition}_{self.name}.csv")
        print(f"Results will be written to: {self.filename}")

        # create and clear the file
        with open(self.filename, mode='w') as f:
            f.write('')
        self.headers = False

        self.global_plan = None
        self.l_n = 0
        self.attention_l = None
        self.cmd_vel_n = 0
        self.attention_cmd_vel = None

        self.json_file_human_pos = f"{str(self.filename)[:-4]}_human_pos.json"
        with open(self.json_file_human_pos, 'w') as outfile:
            poses = {'data':[]}
            print(poses)
            json.dump(poses, outfile)

        print("ready")
        rospy.spin()


    def scene_info_callback(self, msg):
        self.environment = msg.environment
        self.scenario = msg.scenario_name

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

    def rule_based_callback(self, msg, situation_name):
        self.rule_based[situation_name] = msg.data


    def trial_info_callback(self, msg):

        # Compute post-hoc metrics:
        # 'completed': msg.completed,
        # 'targ_dist_norm': msg.targ_dist_norm,
        # 'path_length': msg.path_length,
        # 'mean_dist_to_target_not_moving': msg.mean_dist_to_target_not_moving,
        # 'time_not_moving': msg.time_not_moving,
        # 'episode_timed_out': msg.episode_timed_out,
        # 'path_irregularity': msg.path_irregularity,
        # 'path_efficiency': msg.path_efficiency

        # Increment the trial ID every time we get a new set of robot poses
        # which will be shorter than the latest list of robot poses
        if(self.latest_robot_poses is not None and len(self.latest_robot_poses) > len(msg.robot_poses)):
            self.TRIAL_ID += 1

        self.latest_robot_poses = msg.robot_poses
        self.latest_robot_poses_ts = msg.robot_poses_ts

        prefix = str(self.filename)[:-4]
        self.json_file = f"{prefix}_TRIAL_{self.TRIAL_ID}.json"
        with open(self.json_file, 'w') as outfile:
            poses = {'data':[]}
            for pose, ts in zip(self.latest_robot_poses, self.latest_robot_poses_ts):
                poses['data'].append({
                    'secs':ts.secs,
                    'nsecs':ts.nsecs,
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
                'secs': msg.header.stamp.secs,
                'nsecs': msg.header.stamp.nsecs,

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

                # Rule based social situations
                'cross_path': self.rule_based['cross_path'],
                'down_path': self.rule_based['down_path'],
                'empty': self.rule_based['empty'],
                'join_group': self.rule_based['join_group'],
                'leave_group': self.rule_based['leave_group'],

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
        with open(self.json_file_human_pos, "r+") as file:
            data = json.load(file)
            data_to_append = {}
            data_to_append['secs'] = msg.header.stamp.secs
            data_to_append['nsecs'] = msg.header.stamp.nsecs
            humans = {}
            for human_id in range(len(msg.poses)):
                human = {}
                position = {}
                position['x'] = msg.poses[human_id].position.x
                position['y'] = msg.poses[human_id].position.y
                position['z'] = msg.poses[human_id].position.z
                human['position'] = position
                orientation = {}
                orientation['x'] = msg.poses[human_id].orientation.x
                orientation['y'] = msg.poses[human_id].orientation.y
                orientation['z'] = msg.poses[human_id].orientation.z
                orientation['w'] = msg.poses[human_id].orientation.w
                human['orientation'] = orientation
                humans['human_'+str(human_id)] = human
            data_to_append['human_position'] = humans
            data['data'].append(data_to_append)
            file.seek(0)
            json.dump(data, file)


if __name__ == "__main__":
    try:
        TrialInfoListener()
    except rospy.ROSInterruptException:
        pass
