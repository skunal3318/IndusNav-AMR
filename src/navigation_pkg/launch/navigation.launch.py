import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node, LifecycleNode
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import TimerAction
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml



def generate_launch_description():

    initial_pose_x = LaunchConfiguration('initial_pose_x')
    initial_pose_y = LaunchConfiguration('initial_pose_y')
    initial_pose_yaw = LaunchConfiguration('initial_pose_yaw')

    declare_initial_pose_x = DeclareLaunchArgument(
        'initial_pose_x', default_value='0.0',
        description='AMCL initial pose X in the map frame'
    )
    declare_initial_pose_y = DeclareLaunchArgument(
        'initial_pose_y', default_value='0.0',
        description='AMCL initial pose Y in the map frame'
    )
    declare_initial_pose_yaw = DeclareLaunchArgument(
        'initial_pose_yaw', default_value='0.0',
        description='AMCL initial pose yaw (radians) in the map frame'
    )

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('navigation_pkg'), 'launch', 'simulation.launch.py')
        ),
        launch_arguments = {'use_sim_time': 'true'}.items()
    )


    nav_params = os.path.join(get_package_share_directory('navigation_pkg'), 'config', 'nav2_params.yaml')
    map_file = os.path.join(get_package_share_directory('navigation_pkg'), 'maps', 'industrial.yaml')

    configured_nav_params = RewrittenYaml(
        source_file=nav_params,
        root_key='',
        param_rewrites={
            # nav2_amcl (Jazzy) declares these as nested 'initial_pose.x/y/yaw'
            # parameters, not the flat 'initial_pose_x' names used in older Nav2
            # releases, so the full dotted path is required here.
            'amcl.ros__parameters.initial_pose.x': initial_pose_x,
            'amcl.ros__parameters.initial_pose.y': initial_pose_y,
            'amcl.ros__parameters.initial_pose.yaw': initial_pose_yaw,
        },
        convert_types=True,
    )

    #nav2
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py'),
        ),
        launch_arguments = {
            'map': map_file,
            'use_sim_time': 'true',
            'params_file': configured_nav_params
            }.items(),
    )

    rviz_file = os.path.join(get_package_share_directory('navigation_pkg'), 'rviz', 'rviz_config.rviz')
    #rviz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', rviz_file],
    )

    return LaunchDescription([
        declare_initial_pose_x,
        declare_initial_pose_y,
        declare_initial_pose_yaw,
        simulation,
        TimerAction(period=5.0, actions=[nav2]),
        rviz,
    ])

