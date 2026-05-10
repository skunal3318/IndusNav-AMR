import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


DOCK_X       = 0.0     
DOCK_Y       = 0.0     
DOCK_YAW_DEG = 180.0   

def generate_launch_description():
    pkg = get_package_share_directory('navigation_pkg')

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
            'params_file': os.path.join(pkg, 'config', 'nav2_params.yaml'),
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

    person_detector = Node(
        package='navigation_pkg',
        executable='person_detector',
        name='person_detector',
        output='screen',
    )


    battery_simulator = Node(
        package='navigation_pkg',
        executable='battery_simulator',
        name='battery_simulator',
        output='screen',
        parameters=[{
            'use_sim_time':          True,
            'initial_percentage':    1.0,
            'drain_rate':            0.003,  
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
        simulation,
        TimerAction(period=5.0, actions=[nav2]),

        rviz,
        person_detector,
        battery_simulator,
        dock_pose_broadcaster,

        TimerAction(period=8.0, actions=[auto_dock_manager]),
    ])