import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node, LifecycleNode
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import TimerAction



def generate_launch_description():
    
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('navigation_pkg'), 'launch', 'simulation.launch.py')
        ),
        launch_arguments = {'use_sim_time': 'true'}.items()
    )


    nav_params = os.path.join(get_package_share_directory('navigation_pkg'), 'config', 'nav2_params.yaml')
    map_file = os.path.join(get_package_share_directory('navigation_pkg'), 'maps', 'industrial.yaml')
    
    #nav2
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py'),
        ),
        launch_arguments = {
            'map': map_file,
            'use_sim_time': 'true',
            'params_file': nav_params
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

    person_detector = Node(
        package='navigation_pkg',
        executable='person_detector',
        name='person_detector',
        output='screen',
    )

    return LaunchDescription([
        simulation,
        TimerAction(period=5.0, actions=[nav2]),
        rviz,
        person_detector,
    ])

