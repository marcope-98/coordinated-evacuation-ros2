import os
import sys
from launch import LaunchDescription, LaunchContext
from launch.actions import OpaqueFunction, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression

# cant find a better way of doing this
n_shelfinos = "0"
for arg in sys.argv:
    if arg.startswith("n:="):
        n_shelfinos = arg.split(":=")[1]


def generate_launch_description():

    world_descriptor_node = Node(
        package='rpl_ros2',
        executable='world_descriptor'
    )

    roadmap_publisher_node = Node(
        package='rpl_ros2',
        executable='roadmap_publisher'
    )

    n = LaunchConfiguration('n', default='0')

    shelfino1_planner = Node(
        package='rpl_ros2',
        condition=IfCondition(PythonExpression([n ,'>=1'])),
        executable='shelfino_planner',
        namespace='shelfino1',
        remappings=[('roadmap_topic', '/roadmap_topic'),
                    ('world_description', '/world_description')]
    )

    shelfino2_planner = Node(
        package='rpl_ros2',
        condition=IfCondition(PythonExpression([n ,'>=2'])),
        executable='shelfino_planner',
        namespace='shelfino2',
        remappings=[('roadmap_topic', '/roadmap_topic'),
                    ('world_description', '/world_description')]
    )

    shelfino3_planner = Node(
        package='rpl_ros2',
        condition=IfCondition(PythonExpression([n ,'>=3'])),
        executable='shelfino_planner',
        namespace='shelfino3',
        remappings=[('roadmap_topic', '/roadmap_topic'),
                    ('world_description', '/world_description')]

    )

    shelfino_delay_node = Node(
        package='rpl_ros2',
        executable='shelfino_delay',
        parameters=[{'n_shelfinos': n_shelfinos}]
    )

    shelfino1 = Node(
        package='rpl_ros2',
        condition=IfCondition(PythonExpression([n ,'>=1'])),
        executable='shelfino_path_executor',
        namespace='shelfino1'
    )

    shelfino2 = Node(
        package='rpl_ros2',
        condition=IfCondition(PythonExpression([n ,'>=2'])),
        executable='shelfino_path_executor',
        namespace='shelfino2'
    )

    shelfino3 = Node(
        package='rpl_ros2',
        condition=IfCondition(PythonExpression([n ,'>=3'])),
        executable='shelfino_path_executor',
        namespace='shelfino3'

    )


    return LaunchDescription([
        world_descriptor_node,
        roadmap_publisher_node,    
        shelfino1_planner,
        shelfino2_planner,
        shelfino3_planner,
        shelfino_delay_node,
        shelfino1,
        shelfino2,
        shelfino3,
    ])