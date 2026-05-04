import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('navigation_pkg'), 'launch', 'simulation.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    mapper_params = os.path.join(get_package_share_directory('navigation_pkg'), 'config', 'mapper_params_online_async.yaml')

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py'),
        ),
        launch_arguments={'params_file': mapper_params}.items(),
    )

    rviz_file = os.path.join(get_package_share_directory('navigation_pkg'), 'rviz', 'rviz_config.rviz')

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', rviz_file],
    )

    return LaunchDescription([
        simulation,
        slam,
        rviz,
    ])