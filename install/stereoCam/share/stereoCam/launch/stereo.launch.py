# stereo.launch.py
import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # The stereo_image_proc node expects topics under a common namespace
    # Here, we use '/stereo'
    # It will look for /stereo/left/image_raw and /stereo/right/image_raw
    stereo_proc_node = Node(
        package='stereo_image_proc',
        executable='stereo_image_proc',
        name='stereo_image_proc',
        namespace='/stereo', # This is important!
        remappings=[
            # We remap our actual camera topics to what the node expects
            ('left/image_raw', '/camera/left/image_raw'),
            ('left/camera_info', '/camera/left/camera_info'),
            ('right/image_raw', '/camera/right/image_raw'),
            ('right/camera_info', '/camera/right/camera_info'),
        ]
    )

    return LaunchDescription([
        stereo_proc_node
    ])
