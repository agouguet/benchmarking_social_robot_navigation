import pandas as pd
import argparse
from datetime import datetime
import glob
import os
import sys
import math


ALL_METHODS = ["DWA", "TEB", "HATEB", "SARL*"]

ALL_SCENARIOS = ["static", "frontal", "overtaking", "corner", "intersection", "door_passing"]

##### UTILS #####

def get_first_row_value(df, column_name):
    return df.iloc[0][column_name]

def get_last_row_value(df, column_name):
    return df.iloc[-1][column_name]


def get_total_distance(df):
    return get_first_row_value(df, 'min_dist_to_target')

def average(df, column_name):
    return df[column_name].mean()

def minimum(df, column_name):
    return df[column_name].min()

def maximum(df, column_name):
    return df[column_name].max()

##### PERFORMANCE METRICS #####

def get_path_length(df):
    return get_last_row_value(df, 'path_length')

def get_duration(df):
    first_row = df.iloc[0]
    last_row = df.iloc[-1]

    time_first_row = datetime.fromtimestamp(first_row['secs'])
    time_last_row = datetime.fromtimestamp(last_row['secs'])
    duration = time_last_row - time_first_row
    duration_in_s = duration.total_seconds()
    return duration_in_s

def get_final_distance_to_target(df):
    return get_last_row_value(df, 'dist_to_target')

def get_total_rotation(df):
    return get_last_row_value(df, 'path_irregularity')

def get_time_not_moving(df):
    temp = df.loc[df['min_dist_to_target'] < get_total_distance(df)]
    return get_last_row_value(df, 'time_not_moving') - get_first_row_value(temp, 'time_not_moving')

def get_static_collision(df):
    return get_last_row_value(df, 'obj_collisions')

def get_df_robot(df_json):
    df = pd.DataFrame(columns=['secs','nsecs','pos_x','pos_y','pos_z','ori_x','ori_y','ori_z'])

    for n in range(df_json.shape[0]):
        d = dict(df_json["data"][n])
        df.loc[n] = pd.Series({'secs':d["secs"], 'nsecs':d["nsecs"], 'pos_x':d["position"]["x"], 'pos_y':d["position"]["y"], 'pos_z':d["position"]["z"], 'ori_x':d["orientation"]["x"],'ori_y':d["orientation"]["y"],'ori_z':d["orientation"]["z"]})
    
    return df

def get_robot_speed(df):
    df_speed = df
    df_speed['vel_x'] = df_speed['pos_x'].diff()
    df_speed['vel_y'] = df_speed['pos_y'].diff()
    df_speed['vel_z'] = df_speed['pos_z'].diff()
    return df_speed

def get_robot_acceleration(df):
    df_acc = get_robot_speed(df)
    df_acc['acc_x'] = df_acc['vel_x'].diff()
    df_acc['acc_y'] = df_acc['vel_y'].diff()
    df_acc['acc_z'] = df_acc['vel_z'].diff()
    return df_acc

def get_robot_movement_jerk(df):
    df_move_jerk = get_robot_acceleration(df)
    df_move_jerk['movement_jerk'] = (df_move_jerk['acc_x']**2 + df_move_jerk['acc_y']**2 + df_move_jerk['acc_z']**2) ** 0.5
    return df_move_jerk


##### SOCIAL METRICS #####

def get_violation_time_personal_space(df):
    return get_last_row_value(df, 'robot_on_person_personal_dist_violations')

def get_violation_time_personal_space_while_robot_stationary(df):
    return get_last_row_value(df, 'person_on_robot_personal_dist_violations')

def get_time_in_personal_space(df):
    return get_last_row_value(df, 'time_in_personal_space')

def get_minimum_distance_to_pedestrian(df):
    return get_last_row_value(df, 'min_dist_to_ped')

def get_human_collision(df):
    return get_last_row_value(df, 'robot_on_person_collisions') + get_last_row_value(df, 'person_on_robot_collisions')

def get_minimum_time_to_collision(df):
    df2 = df[df["minimum_time_to_collision"] > 0]
    return minimum(df2, "minimum_time_to_collision")


def analyze(filepath, scenario, method):
    df = pd.read_csv(filepath + scenario +".csv")
    df_robot = get_df_robot(pd.read_json(filepath+"json/"+scenario+".json"))
    df_humans = pd.read_json(filepath+"json/"+scenario+"_human_pos.json")

    results = "" 

    results += ("----- Performance Metrics -----")
    results += ("\nLength (m):" + str(get_path_length(df)))
    results += ("\nTime To Reach Goal (s):"+ str( get_duration(df)))
    results += ("\nError Distance to Target (m):"+ str(get_final_distance_to_target(df)))
    results += ("\nTotal Rotation (degres):"+ str(get_total_rotation(df)))
    results += ("\nTime Not Moving (s):"+ str(get_time_not_moving(df)))
    results += ("\nCollision with Static Obstacle :"+ str(get_static_collision(df)))
    results += ("\nMovement Jerk (m/s^3) :"+ str(minimum(get_robot_movement_jerk(df_robot), "movement_jerk")) + "/" + str(maximum(get_robot_movement_jerk(df_robot), "movement_jerk")) + "/" + str(average(get_robot_movement_jerk(df_robot), "movement_jerk")))

    results += ("\n")

    results += ("\n----- Social Metrics -----")
    results += ("\nViolation of Human Personal Space :"+ str(get_violation_time_personal_space(df)))
    results += ("\nViolation of Human Personal Space While the Robot is Stationary :"+ str(get_violation_time_personal_space_while_robot_stationary(df)))
    results += ("\nTime of Robot Violate Human Personal Space (s):"+ str(get_time_in_personal_space(df)))
    results += ("\nMinimum Distance Between the Robot and the Closest Human (m):"+ str(get_minimum_distance_to_pedestrian(df)))
    results += ("\nCollision with Human :"+ str(get_human_collision(df)))
    results += ("\nMinimum Time to Collision (m) :"+ str(get_minimum_time_to_collision(df)))


    if("follow" in filepath or "lead" in filepath):
        results += ("\n")

        results += ("\n----- Task Metrics -----")
        results += ("\nAverage Distance Between Robot and Leader :"+ str(average(df, "dist_to_leader")))
        results += ("\nMinimum Distance Between Robot and Leader :"+ str(minimum(df, "dist_to_leader")))
        results += ("\nMaximum Distance Between Robot and Leader :"+ str(maximum(df, "dist_to_leader")))
        results += ("\nAverage Distance Between Robot and Follower :"+ str(average(df, "dist_to_follower")))
        results += ("\nMinimum Distance Between Robot and Follower :"+ str(minimum(df, "dist_to_follower")))
        results += ("\nMaximum Distance Between Robot and Follower :"+ str(maximum(df, "dist_to_follower")))

    return results

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Analyse data given by social sim unity.')
    parser.add_argument('--method', help="name of robot navigation method.", required=False)
    parser.add_argument('--scenario', help="name of scenario/map.", required=False)
    args = parser.parse_args()

    method = args.method

    for scenario in ALL_SCENARIOS:
        print("\n\n++++++++++", scenario.upper(), "++++++++++")
        results = analyze("data/"+method+"/", scenario, method)
        print(results)
        write_file = "results/analyzes/results_" + method + "_" + scenario
        f = open(write_file+".txt","w+")
        f.write(results)
        f.close()

