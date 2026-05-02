import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, EmitEvent
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import launch
from launch.events import Shutdown




def generate_launch_description():
    # --- Package Directories ---
    rosbot_gazebo = get_package_share_directory('rosbot_gazebo')
    nav2_bringup  = get_package_share_directory('nav2_bringup')
    pkg_sim       = get_package_share_directory('seek_and_destroy_sim')
    pkg_brain     = get_package_share_directory('seek_and_destroy_brain')
   
    # NEW: Get explore_lite directory
    explore_lite_pkg = get_package_share_directory('explore_lite')




    # --- File Paths ---
    ws_src       = os.path.expanduser('~/seek_destroy_ws/src')
    world_file   = os.path.join(pkg_sim, 'worlds', 'lab.sdf')
    slam_params  = os.path.join(pkg_sim, 'config', 'slam_params.yaml')
    nav2_params  = os.path.join(ws_src, 'seek_and_destroy_sim', 'config', 'nav2_params.yaml')
    targets_file = os.path.join(ws_src, 'seek_and_destroy_brain', 'config', 'targets.yaml')
    map_save_dir = os.path.join(ws_src, 'saved_maps')




    # Gazebo config
    gz_gui_config = os.path.join(pkg_sim, 'config', 'lab_world_gazebo.config')
   
    # RViz config
    rviz_config  = os.path.join(ws_src, 'seek_destroy', 'config', 'nav2_default_view.rviz')




    return LaunchDescription([
       
        # 1. Gazebo + Robot
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(rosbot_gazebo, 'launch', 'simulation.launch.py')
            ),
            launch_arguments={
                'robot_model': 'rosbot',
                'gz_world': world_file,
                'rviz': 'false',  
                'gz_gui': gz_gui_config,
            }.items()
        ),




        # 2. SLAM Toolbox (Delayed 5s)
        TimerAction(period=5.0, actions=[
            Node(
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                output='screen',
                parameters=[slam_params, {
                    'use_sim_time': True,
                    'odom_frame':   'odom',
                    'map_frame':    'map',
                    'base_frame':   'base_link',
                    'scan_topic':   '/scan',
                }],
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




        # 5. Brain Nodes: Detector, Go Home, and Explore Lite (Delayed 15s)
        TimerAction(period=15.0, actions=[
            # Color Detector
            Node(
                package='seek_and_destroy_brain',
                executable='color_detector',
                name='color_detector',
                output='screen',
                parameters=[{'targets_file': targets_file}, {'use_sim_time': True}]
            ),
            # NEW: Go Home Node
            Node(
                package='seek_and_destroy_brain',
                executable='navigator',
                name='navigator',
                output='screen',
                parameters=[{'use_sim_time': True}]
            ),
            # Explore Lite Node
            # IncludeLaunchDescription(
            #     PythonLaunchDescriptionSource(
            #         os.path.join(explore_lite_pkg, 'launch', 'explore.launch.py')
            #     ),
            #     launch_arguments={
            #         'use_sim_time': 'true'
            #     }.items()
            # ),
            Node(
                package='explore_lite',
                executable='explore',
                name='explore_node',
                output='screen',   # ← screen so you can see its internal logs
                parameters=[{
                    'use_sim_time':          True,
                    'robot_base_frame':      'base_link',
                    'costmap_topic':         '/global_costmap/costmap',      
                    'costmap_updates_topic': '/global_costmap/costmap_updates',
                    'visualize':             True,
                    'planner_frequency':     0.5,
                    'progress_timeout':      10.0,
                    'potential_scale':       3.0,
                    'gain_scale':            1.0,
                    'min_frontier_size':     0.3,
                    'start_stopped':         True,
                }],
            ),
        ]),




        # 6. State Machine UI (Delayed 20s)
        TimerAction(period=22.0, actions=[
            Node(
                package='seek_and_destroy_brain',
                executable='state_machine',
                name='state_machine',
                output='screen',
                prefix=['xterm -fa Monospace -fs 14 -geometry 40x15 -sb -sl 1000 -e'],
                on_exit=EmitEvent(event=Shutdown()),
                parameters=[{
                    'targets_file': targets_file,
                    'map_save_dir': map_save_dir,
                    'use_sim_time': True,
                }],
            )
        ]),




        # 7. Mission Control GUI @ +21 s
        # Launched 1 second after state_machine so the /mission/state topic
        # is already being published before the GUI subscribes to it.
        TimerAction(period=23.0, actions=[
            Node(
                package='seek_and_destroy_brain',
                executable='mission_gui',
                name='mission_gui',
                output='log',
                on_exit=EmitEvent(event=Shutdown()),
                parameters=[{'use_sim_time': True}],
            ),
        ]),
    ])