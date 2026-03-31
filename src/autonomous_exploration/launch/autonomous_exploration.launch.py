import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # --- Package Directories ---
    rosbot_gazebo = get_package_share_directory('rosbot_gazebo')
    nav2_bringup  = get_package_share_directory('nav2_bringup')
    slam_toolbox  = get_package_share_directory('slam_toolbox')
    explore_lite  = get_package_share_directory('explore_lite')

    # --- File Paths ---
    nav2_params = '/home/user/seek_destroy_ws/src/seek_destroy/config/nav2_params.yaml'
    slam_params = '/home/user/seek_destroy_ws/my_slam_params.yaml'
    rviz_config = '/home/user/seek_destroy_ws/src/seek_destroy/config/nav2_default_view.rviz'

    return LaunchDescription([
        
        # 1. Gazebo simulation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(rosbot_gazebo, 'launch', 'simulation.launch.py')
            ),
            launch_arguments={'robot_model': 'rosbot'}.items()
        ),

        # 2. SLAM Toolbox — Delayed 5s
        TimerAction(period=5.0, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(slam_toolbox, 'launch', 'online_async_launch.py')
                ),
                launch_arguments={
                    'slam_params_file': slam_params,
                    'use_sim_time': 'true'
                }.items()
            )
        ]),

        # 3. Nav2 — Delayed 10s
        TimerAction(period=10.0, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup, 'launch', 'navigation_launch.py')
                ),
                launch_arguments={
                    'use_sim_time': 'true',
                    'params_file': nav2_params
                }.items()
            )
        ]),

        # 4. RViz2 — Delayed 12s, loading your custom config
        TimerAction(period=12.0, actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config],
                parameters=[{'use_sim_time': True}],
                output='screen'
            )
        ]),

        # 5. Explore Lite — Delayed 15s to ensure costmaps are ready
        TimerAction(period=15.0, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(explore_lite, 'launch', 'explore.launch.py')
                ),
                launch_arguments={'use_sim_time': 'true'}.items()
            )
        ]),

        # 6. Auto Map Saver — Delayed 20s to silently monitor logs
        TimerAction(period=20.0, actions=[
            Node(
                package='autonomous_exploration',
                executable='auto_map_saver',
                name='auto_map_saver',
                output='screen'
            )
        ]),
    ])