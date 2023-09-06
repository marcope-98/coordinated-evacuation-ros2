import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression, AndSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource



def generate_launch_description():

    n       = LaunchConfiguration('n', default='0')
    # Robot name
    robot_0 = LaunchConfiguration('robot_0', default='shelfino1')
    robot_1 = LaunchConfiguration('robot_1', default='shelfino2')
    robot_2 = LaunchConfiguration('robot_2', default='shelfino3')
    # Robot default pose
    x_0     = LaunchConfiguration('x_0', default='2')
    y_0     = LaunchConfiguration('y_0', default='2')
    yaw_0   = LaunchConfiguration('yaw_0', default='0')

    x_1     = LaunchConfiguration('x_1', default='2')
    y_1     = LaunchConfiguration('y_1', default='9')
    yaw_1   = LaunchConfiguration('yaw_1', default='0')
    
    x_2     = LaunchConfiguration('x_2', default='14')
    y_2     = LaunchConfiguration('y_2', default='2')
    yaw_2   = LaunchConfiguration('yaw_2', default='-1.57')


    #  sim time and gui
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    gui          = LaunchConfiguration('gui', default='true')

    #  simulator/*.launch.py
    launch_dir = os.path.join(get_package_share_directory('simulator'), 'launch')


    # Declare Launch Description
    declare_gui = DeclareLaunchArgument(
                                        name='gui', 
                                        default_value='true', choices=['true', 'false'],
                                        description='Flag to enable gazebo visualization'
                                        )

    world_spawner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_dir, '/init_world.launch.py']),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'gui': gui
        }.items()
    )



    shelfino1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_dir, '/init_shelfino.launch.py']),
        condition=IfCondition(PythonExpression([n ,'>=1'])),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'robot': robot_0,
            'x': x_0,
            'y': y_0,
            'Y': yaw_0
        }.items()
    )

    shelfino2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_dir, '/init_shelfino.launch.py']),
        condition=IfCondition(PythonExpression([n ,'>=2'])),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'robot': robot_1,
            'x': x_1,
            'y': y_1,
            'Y': yaw_1
        }.items()
    )

    shelfino3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_dir, '/init_shelfino.launch.py']),
        condition=IfCondition(PythonExpression([n ,'>=3'])),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'robot': robot_2,
            'x': x_2,
            'y': y_2,
            'Y': yaw_2
        }.items()
    )

    


    return  LaunchDescription([
        declare_gui,
        world_spawner,
        shelfino1,
        shelfino2,
        shelfino3,
        Node(
            package='simulator',
            executable='send_obstacles'
        ),
        Node(
            package='simulator',
            executable='send_borders'
        ),
        Node(
            package='simulator',
            executable='send_gates'
        ),
    ])
