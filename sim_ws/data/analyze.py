import pandas as pd
import argparse
from datetime import datetime


ALL_METHODS = ["DWA", "TEB", "HATEB", "SARL*"]

##### UTILS #####

def get_first_row_value(df, column_name):
    return df.iloc[0][column_name]

def get_last_row_value(df, column_name):
    return df.iloc[-1][column_name]


def get_total_distance(df):
    return get_first_row_value(df, 'min_dist_to_target')




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


def analyze(filename):
    df = pd.read_csv(filename)
    df_top = df.head()

    print("----- Performance Metrics -----")
    print("Length (m):", get_path_length(df))
    print("Time To Reach Goal (s):", get_duration(df))
    print("Error Distance to Target (m):", get_final_distance_to_target(df))
    print("Total Rotation (degres):", get_total_rotation(df))
    print("Time Not Moving (s):", get_time_not_moving(df))
    print("Collision with Static Obstacle :", get_static_collision(df))

    print("")

    print("----- Social Metrics -----")
    print("Violation of Human Personal Space :", get_violation_time_personal_space(df))
    print("Violation of Human Personal Space While the Robot is Stationary :", get_violation_time_personal_space_while_robot_stationary(df))
    print("Time of Robot Violate Human Personal Space (s):", get_time_in_personal_space(df))
    print("Minimum Distance Between the Robot and the Closest Human (m):", get_minimum_distance_to_pedestrian(df))
    print("Collision with Human :", get_human_collision(df))

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Analyse data given by social sim unity.')
    parser.add_argument('filename', help="filename of .csv file.")
    args = parser.parse_args()
    analyze(args.filename)
