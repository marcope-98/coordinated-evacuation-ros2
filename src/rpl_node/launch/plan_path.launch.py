import os
from launch import LaunchDescription

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='rpl_node',
            executable='rpl_node'
        ),
    ])