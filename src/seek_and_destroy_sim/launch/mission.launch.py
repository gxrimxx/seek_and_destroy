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
    pkg_sim       = get_package_share_directory('seek_and_destroy_sim')
    pkg_brain     = get_package_share_directory('seek_and_destroy_brain')

    # --- File Paths ---
    ws_src       = os.path.expanduser('~/seek_destroy_ws/src')
    world_file   = os.path.join(pkg_sim, 'worlds', 'lab.world')
    slam_params  = os.path.join(pkg_sim, 'config', 'slam_params.yaml')
    nav2_params  = os.path.join(ws_src, 'seek_and_destroy_sim', 'config', 'nav2_params.yaml')
    targets_file = os.path.join(ws_src, 'seek_and_destroy_brain', 'config', 'targets.yaml')
    
    # Using your RViz config from the previous deliverable
    rviz_config  = os.path.join(ws_src, 'seek_destroy', 'config', 'nav2_default_view.rviz')

    return LaunchDescription([
        
        # 1. Gazebo + Robot
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(rosbot_gazebo, 'launch', 'simulation.launch.py')
            ),
            # Keep RViz false here so we can spawn our own custom one below
            launch_arguments={
                'robot_model': 'rosbot',
                'gz_world': world_file,
                'rviz': 'false',       
            }.items()
        ),

        # 2. SLAM Toolbox (Delayed 5s)
        TimerAction(period=5.0, actions=[
            Node(
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                output='screen',
                parameters=[slam_params, {'use_sim_time': True}],
            )
        ]),

        # 3. Nav2 (Delayed 10s)
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

        # 4. RViz2 & Camera View (Delayed 12s)
        # These give you your visual tools!
        TimerAction(period=12.0, actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config],
                parameters=[{'use_sim_time': True}],
                output='screen'
            ),
            Node(
                package='rqt_image_view',
                executable='rqt_image_view',
                name='rqt_image_view',
                output='screen'
            )
        ]),

        # 5. Color Detector (Delayed 15s)
        TimerAction(period=15.0, actions=[
            Node(
                package='seek_and_destroy_brain',
                executable='color_detector',
                name='color_detector',
                output='screen',
                parameters=[{'targets_file': targets_file}, {'use_sim_time': True}]
            )
        ]),

        # 6. State Machine UI (Delayed 20s)
        # Notice the updated prefix! This forces a larger window (100x30) and bigger font (fs 14)
        TimerAction(period=20.0, actions=[
            Node(
                package='seek_and_destroy_brain',
                executable='state_machine',
                name='state_machine',
                output='screen',
                prefix=['xterm -fa Monospace -fs 14 -geometry 100x30 -e '] 
            )
        ]),
    ])