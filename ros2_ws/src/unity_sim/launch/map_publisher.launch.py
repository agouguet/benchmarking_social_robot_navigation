import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
import launch_ros.actions


def launch_setup(context, *args, **kwargs):
    map_conf = LaunchConfiguration("map")

    # map file
    map_file_path = os.path.join(
        get_package_share_directory('unity_sim'),
        'maps',
        map_conf.perform(context),
        'map.yaml'
    )

    map_server_cmd = Node(
        package='nav2_map_server',
        executable='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_file_path}])


    lifecycle_nodes = ['map_server']
    use_sim_time = True
    autostart = True

    start_lifecycle_manager_cmd = launch_ros.actions.Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes}])

    timer_action = TimerAction(period=1.5, actions=[start_lifecycle_manager_cmd])

    set_map = launch_ros.actions.SetParameter(name="map", value=map_conf)

    return [
        set_map,
        map_server_cmd,
        timer_action,
    ]

def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("map", default_value="complete"),
            OpaqueFunction(function=launch_setup),
        ]
    )

# def generate_launch_description():

#     ld = LaunchDescription()

#     map_launch_arg = DeclareLaunchArgument(
#         'map',
#         default_value='complete'
#     )

#     map_value = LaunchConfiguration('map')

    

#     # map file
#     map_file_path = os.path.join(
#         get_package_share_directory('unity_sim'),
#         'maps',
#         map_value,
#         'map.yaml'
#     )

#     map_server_cmd = Node(
#         package='nav2_map_server',
#         executable='map_server',
#         output='screen',
#         parameters=[{'yaml_filename': map_file_path}])


#     lifecycle_nodes = ['map_server']
#     use_sim_time = True
#     autostart = True

#     start_lifecycle_manager_cmd = launch_ros.actions.Node(
#             package='nav2_lifecycle_manager',
#             executable='lifecycle_manager',
#             name='lifecycle_manager',
#             output='screen',
#             emulate_tty=True,  # https://github.com/ros2/launch/issues/188
#             parameters=[{'use_sim_time': use_sim_time},
#                         {'autostart': autostart},
#                         {'node_names': lifecycle_nodes}])

#     timer_action = TimerAction(period=1.5, actions=[start_lifecycle_manager_cmd])

#     ld.add_action(map_server_cmd)
#     ld.add_action(timer_action)

#     return LaunchDescription([
#         map_launch_arg,
#         map_server_cmd,
#         timer_action
#     ])