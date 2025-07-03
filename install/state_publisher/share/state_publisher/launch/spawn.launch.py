# spawn_and_visualize.launch.py

from inspect import Parameter
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # IMPORTANT: Find your drone's SDF file.
    # The PX4 environment is complex. Let's assume the model you are using
    # is the one you modified. We need its full path.
    # This path is an EXAMPLE, you must find the correct one on your system
    # 
        # Construct the path to the new URDF file

    # Read the URDF file to get the robot description
    model_path = os.path.join(
        os.path.expanduser('~'), # Gets your home directory
	'belajar-ROS',
	'PX4-Autopilot',
        'Tools', 'simulation',
        'gazebo-classic', 'sitl_gazebo-classic',
        'models',
        'iris', # The folder of the model you are launching
        'iris.sdf'
    )
    
    # Check if the model file exists
    if not os.path.exists(model_path):
        raise FileNotFoundError(f"The model file was not found at {model_path}. Please update the path in the launch file.")

    # Read the SDF file to get the robot description
    robot_description = open(model_path).read()
    print(robot_description)
    # 1. The Robot State Publisher
    # This node reads the robot_description and publishes the TF transforms
    # for all the links of the robot.
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True, # Use Gazebo's simulation time
            'robot_description': robot_description
        }]
    )

    # 2. The RViz2 Node
    # We can also start RViz automatically for convenience.
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )
    
    

    return LaunchDescription([
        robot_state_publisher_node,
        # rviz2_node,
    ])
