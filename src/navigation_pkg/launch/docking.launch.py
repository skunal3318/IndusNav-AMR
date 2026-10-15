import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml


DOCK_X       = 0.0
DOCK_Y       = 0.0
DOCK_YAW_DEG = 180.0

def generate_launch_description():
    pkg = get_package_share_directory('navigation_pkg')

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

    configured_nav_params = RewrittenYaml(
        source_file=os.path.join(pkg, 'config', 'nav2_params.yaml'),
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

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'simulation.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch', 'bringup_launch.py'
            )
        ),
        launch_arguments={
            'map':         os.path.join(pkg, 'maps',   'industrial.yaml'),
            'params_file': configured_nav_params,
            'use_sim_time': 'true',
        }.items(),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', os.path.join(pkg, 'rviz', 'rviz_config.rviz')],
    )

    battery_simulator = Node(
        package='navigation_pkg',
        executable='battery_simulator',
        name='battery_simulator',
        output='screen',
        parameters=[{
            'use_sim_time':          True,
            'initial_percentage':    1.0,
            'idle_drain_rate':       0.0008,
            'motion_drain_rate':     0.006,
            'cmd_vel_timeout':       0.5,
            'charge_rate':           0.005,
            'publish_frequency':    10.0,
            'low_battery_threshold': 0.20,
            'full_battery_threshold': 0.95,
            'voltage_nominal':       24.0,
            'capacity_ah':            5.0,
        }],
    )

    dock_pose_broadcaster = Node(
        package='navigation_pkg',
        executable='dock_pose_broadcaster',
        name='dock_pose_broadcaster',
        output='screen',
        parameters=[{
            'use_sim_time':      True,
            'dock_x':            DOCK_X,
            'dock_y':            DOCK_Y,
            'dock_yaw_deg':      DOCK_YAW_DEG,
            'publish_frequency': 1.0,
        }],
    )

    patrol_node = Node(
        package='navigation_pkg',
        executable='patrol_node',
        name='patrol_node',
        output='screen',
        parameters= [{'use_sim_time': True}]
    )

    auto_dock_manager = Node(
        package='navigation_pkg',
        executable='auto_dock_manager',
        name='auto_dock_manager',
        output='screen',
        parameters=[{
            'use_sim_time':           True,
            'low_battery_threshold':  0.20,
            'full_battery_threshold': 0.95,
            'check_frequency':         1.0,
            'dock_id':               'home_dock',
            'dock_type':             'simple_charging_dock',

            'dock_pose': [DOCK_X, DOCK_Y, DOCK_YAW_DEG],
        }],
    )

    return LaunchDescription([
        declare_initial_pose_x,
        declare_initial_pose_y,
        declare_initial_pose_yaw,
        simulation,
        TimerAction(period=5.0, actions=[nav2]),
        TimerAction(period=10.0, actions=[patrol_node]),
        rviz,
        battery_simulator,
        dock_pose_broadcaster,

        TimerAction(period=8.0, actions=[auto_dock_manager]),
    ])