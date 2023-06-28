import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    gui          = LaunchConfiguration('gui', default='true')

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    
    world_file_name          = 'mindstorm_arena.world'
    world                    = os.path.join(get_package_share_directory('simulator'), 'worlds', world_file_name)
    gazebo_world_models_path = os.path.join(get_package_share_directory('simulator'), 'models')

    try:
        if gazebo_world_models_path not in os.environ["GAZEBO_MODEL_PATH"]:
            os.environ["GAZEBO_MODEL_PATH"] += ':' + gazebo_world_models_path
    except KeyError:
        os.environ["GAZEBO_MODEL_PATH"] = gazebo_world_models_path 



    declare_use_sim_time =  DeclareLaunchArgument(
                                name='use_sim_time', 
                                default_value='true',
                                choices=['true', 'false']
                            )

    declare_gui          =  DeclareLaunchArgument(
                                name='gui', 
                                default_value='true', 
                                choices=['true', 'false']
                            )

    gazebo =    IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        [os.path.join(pkg_gazebo_ros, 'launch'), '/gazebo.launch.py']
                    ),
                    launch_arguments={
                        'world': world,
                        'gui': gui
                    }.items(),
                )

    map_to_odom_transform = Node( 
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_server',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', '1.0', 'map', 'odom'],
        parameters=[{'use_sim_time': use_sim_time}])
    
    return LaunchDescription([
        declare_gui,
        declare_use_sim_time,
        gazebo,
        map_to_odom_transform
    ])