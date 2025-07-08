# In your_drone_bringup/launch/main.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
# from state_publisher.state_publisher import odom_to_tf

def generate_launch_description():

    # =================================================================
    # === 1. DEFINE FILE PATHS & PACKAGES                           ===
    # =================================================================
    # Find the path to your own package
    my_package_dir = get_package_share_directory('state_publisher') #<-- CHANGE THIS

    # Find the path to the PX4 Autopilot directory
    px4_autopilot_dir = os.path.join(os.path.expanduser('~'), 'belajar-ROS/PX4-Autopilot') #<-- CHANGE IF DIFFERENT

    # Find the path to your custom stereo processing launch file
    stereo_processing_launch = os.path.join(
        my_package_dir,
        'launch',
        'stereo.launch.py' #<-- The file you created in the last step
    )

    # Find the path to your custom VIO launch file
    vio_launch = os.path.join(
        my_package_dir,
        'launch',
        'vio.launch.py'
    )

    state_publisher_launch = os.path.join(
        my_package_dir,
        'launch',
        'spawn.launch.py'
    )
    
    odom_to_tf_launch = os.path.join(
        my_package_dir,
        'launch',
        'odom_to_tf.launch.py'
    )

    # Path to your RViz config file
    rviz_config_file = os.path.join(my_package_dir, 'config', 'main.rviz')


    # =================================================================
    # === 2. DEFINE THE ACTIONS TO LAUNCH                           ===
    # =================================================================

    # Action to launch the PX4 SITL Gazebo simulation
    # We use ExecuteProcess to run the 'make' command
    start_px4_gazebo_sim = ExecuteProcess(
        cmd=[
            'make',
            'px4_sitl',
            'gazebo-classic_iris' #<-- Your custom model target
        ],
        cwd=px4_autopilot_dir, #<-- Crucial: run the command in the PX4 directory
        output='screen'
    )

    # Action to include your stereo image processing launch file
    include_stereo_processing = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(stereo_processing_launch)
    )

    # Action to include your VIO (RTAB-Map) launch file
    include_vio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(vio_launch)
    )

    include_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(state_publisher_launch)
    )

    include_odom_to_tf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(odom_to_tf_launch)
    )
    
    static_tf_pub = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
        )

    # Action to launch RViz2 with a pre-made configuration
    start_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )


    # =================================================================
    # === 3. ASSEMBLE THE FINAL LAUNCH DESCRIPTION                  ===
    # =================================================================
    
    return LaunchDescription([
        # Start the Gazebo simulation first
        # start_px4_gazebo_sim,

        # Then, start all the ROS 2 nodes
        include_stereo_processing,

        # include_vio,
        include_state_publisher,
        # include_odom_to_tf,
        start_rviz,

        # You can also add more complex logic, like starting nodes after
        # the simulation has fully loaded, but this is a great start.
    ])