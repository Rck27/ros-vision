# This is the content for your LAUNCH file: odomtotf.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # This action tells ROS 2 to run your Python script
    odom_to_tf_node = Node(
        package='state_publisher',  # <-- CHANGE to the name of your package
        executable='odom_to_tf',      # <-- The name you gave it in setup.py's entry_points
        name='odom_to_tf_broadcaster'
    )

    return LaunchDescription([
        odom_to_tf_node
    ])