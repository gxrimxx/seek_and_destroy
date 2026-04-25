import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_sim = get_package_share_directory('seek_and_destroy_sim')
    world_file = os.path.join(pkg_sim, 'worlds', 'lab.world')

    # ── Gazebo ──────────────────────────────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'),
                         'launch', 'gazebo.launch.py')),
        launch_arguments={
            'world': world_file,
            'verbose': 'false',
        }.items()
    )

    # ── SLAM Toolbox ─────────────────────────────────────────
    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            os.path.join(pkg_sim, 'config', 'slam_params.yaml'),
            {'use_sim_time': True}
        ],
    )

    # ── Go Home node ─────────────────────────────────────────
    go_home = Node(
        package='seek_and_destroy_sim',
        executable='go_home',
        output='screen',
    )

    return LaunchDescription([
        gazebo,
        TimerAction(period=3.0, actions=[slam]),
        TimerAction(period=6.0, actions=[go_home]),
    ])