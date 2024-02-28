from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    base_path = os.path.realpath(get_package_share_directory('unity_sim')) # also tried without realpath
    rviz_path=base_path+'/rviz/config.rviz'

    launch_description = LaunchDescription()
    launch_description.add_action(
        Node(
          package='rviz2', executable='rviz2', name="rviz2", output='screen', arguments=['-d', str(rviz_path)]
        ),
    )

    return launch_description