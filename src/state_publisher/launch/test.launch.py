# In your_package/launch/test_ground_truth.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    # Path to your custom drone's SDF file
    # This MUST be the SDF with the 'gazebo_ros_state' plugin added!
    sdf_file_path = os.path.join(
        os.path.expanduser('~'), # Or find the path another way
        '/home/deeric/belajar-ROS/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris/iris.sdf'
    )

    print(get_package_share_directory('state_publisher'))
    # Path to your empty world file
    world_path = os.path.join(
        '/home/deeric/belajar-ROS/ros2_ws/models/default.world'
    )

    # 1. Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        launch_arguments={'world': world_path}.items()
    )

    # 2. Spawn your drone in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_iris'],
        output='screen'
    )

    # 3. Publish the robot's state (TF for joints)
    #    It reads from the 'robot_description' topic published by the xacro conversion
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            # This is the key: we tell it to get the model from the 'robot_description' topic
            'robot_description': open(sdf_file_path).read() 
        }]
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
    ])