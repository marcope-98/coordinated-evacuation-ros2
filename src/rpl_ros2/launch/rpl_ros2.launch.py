import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    world_descriptor_node = Node(
        package='rpl_ros2',
        executable='world_descriptor'
    )


    return LaunchDescription([
        world_descriptor_node,
    ])