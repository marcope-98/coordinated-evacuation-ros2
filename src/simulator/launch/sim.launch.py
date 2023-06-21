import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression, AndSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    world_file_name = 'mindstorm_arena.world'
    world                    = os.path.join(get_package_share_directory('simulator'), 'worlds', world_file_name)
    launch_file_dir          = os.path.join(get_package_share_directory('shelfino_description'), 'launch')
    gazebo_models_path       = os.path.join(get_package_share_directory('shelfino_description'), 'models')
    gazebo_world_models_path = os.path.join(get_package_share_directory('simulator'), 'models')
    
    os.environ["GAZEBO_MODEL_PATH"] = gazebo_world_models_path + ':' + gazebo_models_path


    # Number of robots
    n = LaunchConfiguration('n', default='1')
    # Robot name
    robot_0 = LaunchConfiguration('robot_0', default='shelfino1')
    robot_1 = LaunchConfiguration('robot_1', default='shelfino2')
    robot_2 = LaunchConfiguration('robot_2', default='shelfino3')
    # Robot default pose
    x_0 = LaunchConfiguration('x_0', default='2')
    y_0 = LaunchConfiguration('y_0', default='2')
    yaw_0 = LaunchConfiguration('yaw_0', default='0')

    x_1 = LaunchConfiguration('x_1', default='2')
    y_1 = LaunchConfiguration('y_1', default='9')
    yaw_1 = LaunchConfiguration('yaw_1', default='0')
    
    x_2 = LaunchConfiguration('x_2', default='14')
    y_2 = LaunchConfiguration('y_2', default='2')
    yaw_2 = LaunchConfiguration('yaw_2', default='-1.57')

    # rviz config
    rviz_config1 = os.path.join(get_package_share_directory('shelfino_gazebo'), 'rviz', 'shelfino1.rviz')
    rviz_config2 = os.path.join(get_package_share_directory('shelfino_gazebo'), 'rviz', 'shelfino2.rviz')
    rviz_config3 = os.path.join(get_package_share_directory('shelfino_gazebo'), 'rviz', 'shelfino3.rviz')

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    return LaunchDescription([
        DeclareLaunchArgument(name='gui', default_value='true', choices=['true', 'false'],
                                description='Flag to enable gazebo visualization'),
        
        DeclareLaunchArgument(name='rviz', default_value='false', choices=['true', 'false'],
                                description='Flag to enable rviz visualization'),

        DeclareLaunchArgument('model', default_value=[os.path.join(gazebo_models_path, 'shelfino'),'/model.sdf']),
        LogInfo(msg=LaunchConfiguration('model')),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': world}.items(),
        ), 
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
            ),
            condition=IfCondition(gui)
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            condition=IfCondition(PythonExpression([n ,'>=1'])),
            arguments=['-file', LaunchConfiguration('model'),
                       '-entity', robot_0,
                       '-robot_namespace', robot_0,
                       '-x', x_0,
                       '-y', y_0,
                       '-Y', yaw_0]
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            condition=IfCondition(PythonExpression([n ,'>=2'])),
            arguments=['-file', LaunchConfiguration('model'),
                       '-entity', robot_1,
                       '-robot_namespace', robot_1,
                       '-x', x_1,
                       '-y', y_1,
                       '-Y', yaw_1]
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            condition=IfCondition(PythonExpression([n ,'>=3'])),
            arguments=['-file', LaunchConfiguration('model'),
                       '-entity', robot_2,
                       '-robot_namespace', robot_2,
                       '-x', x_2,
                       '-y', y_2,
                       '-Y', yaw_2]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/robot_state_publisher.launch.py']),
            condition=IfCondition(PythonExpression([n ,'>=1'])),
            launch_arguments={'use_sim_time': use_sim_time,
                              'robot_id': robot_0}.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/robot_state_publisher.launch.py']),
            condition=IfCondition(PythonExpression([n ,'>=2'])),
            launch_arguments={'use_sim_time': use_sim_time,
                              'robot_id': robot_1}.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/robot_state_publisher.launch.py']),
            condition=IfCondition(PythonExpression([n ,'>=3'])),
            launch_arguments={'use_sim_time': use_sim_time,
                              'robot_id': robot_2}.items()
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            namespace='shelfino1',
            arguments=['-d', rviz_config1],
            condition=IfCondition(AndSubstitution(PythonExpression([n ,'>=1']), rviz)),
            remappings=remappings,
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            namespace='shelfino2',
            arguments=['-d', rviz_config2],
            condition=IfCondition(AndSubstitution(PythonExpression([n ,'>=2']), rviz)),
            remappings=remappings,
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            namespace='shelfino3',
            arguments=['-d', rviz_config3],
            condition=IfCondition(AndSubstitution(PythonExpression([n ,'>=3']), rviz)),
            remappings=remappings,
        ),

        Node(
            package='get_positions',
            executable='get_positions',
            condition=IfCondition(PythonExpression([n ,'>=1'])),
            namespace=robot_0,
            remappings=[
            ('/tf', 'tf')],
            ),
        Node(
            package='get_positions',
            executable='get_positions',
            condition=IfCondition(PythonExpression([n ,'>=2'])),
            namespace=robot_1,
            remappings=[
            ('/tf', 'tf')],
            ),
        Node(
            package='get_positions',
            executable='get_positions',
            condition=IfCondition(PythonExpression([n ,'>=3'])),
            namespace=robot_2,
            remappings=[
            ('/tf', 'tf')],
            ),

        Node(
            package='send_obstacles',
            executable='send_obstacles'
        ),
        Node(
            package='send_borders',
            executable='send_borders'
        ),
        Node(
            package='send_gates',
            executable='send_gates'
        ),
    ])
