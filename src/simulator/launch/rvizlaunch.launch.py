import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    sh_nav_path = os.path.join(get_package_share_directory('shelfino_navigation'), 'launch')
    sh_des_path = os.path.join(get_package_share_directory('shelfino_description'), 'launch')
    
    robot_id = LaunchConfiguration('robot_id', default='shelfino404')
    remote = LaunchConfiguration('use_rviz', default='false')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_id',
            default_value='false',
            description='Shelfino ID'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Use Rviz defaults to false'
        ),
    ])