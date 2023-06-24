import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    # Robot name
    robot = LaunchConfiguration('robot', default='shelfino404')
    # Robot default pose
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    Y = LaunchConfiguration('Y')


    launch_file_dir    = os.path.join(get_package_share_directory('shelfino_description'), 'launch')
    gazebo_models_path = os.path.join(get_package_share_directory('shelfino_description'), 'models')
    sdf_file           = os.path.join(gazebo_models_path, 'shelfino', 'model.sdf')

    try:
        if gazebo_models_path not in os.environ["GAZEBO_MODEL_PATH"]:
            os.environ["GAZEBO_MODEL_PATH"] += ':' + gazebo_models_path
    except KeyError:
        os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path 


    declare_robot_name = DeclareLaunchArgument(name='robot')
    declare_x          = DeclareLaunchArgument(name='x')
    declare_y          = DeclareLaunchArgument(name='y')
    declare_Y          = DeclareLaunchArgument(name='Y')

    shelfino = Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    arguments=[
                        '-file', sdf_file,
                        '-entity', robot,
                        '-robot_namespace', robot,
                        '-x', x,
                        '-y', y,
                        '-Y', Y]
        ) 

    robot_state_publisher = IncludeLaunchDescription(
                        PythonLaunchDescriptionSource([launch_file_dir, '/robot_state_publisher.launch.py']),
                        launch_arguments={
                            'use_sim_time': use_sim_time,
                            'robot_id': robot
                        }.items()
        )

    get_position_node = Node(
                            package='get_positions',
                            executable='get_positions',
                            namespace=robot,
                            remappings=[
                            ('/tf', 'tf')],
            )

    
    return LaunchDescription([
        declare_robot_name,
        declare_x,
        declare_y,
        declare_Y,
        robot_state_publisher,
        shelfino,
        get_position_node,
    ])