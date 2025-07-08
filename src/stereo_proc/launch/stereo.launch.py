# In your_package/launch/stereo_processing.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    dir = '/home/deeric/belajar-ROS/ros2_ws/src/stereo_proc'    # 1. Find the path to the official stereo_image_proc launch file
    stereo_image_proc_dir = get_package_share_directory('stereo_image_proc')
    stereo_image_proc_launch = os.path.join(
        stereo_image_proc_dir,
        'launch',
        'stereo_image_proc.launch.py'
    )

    # 2. Define the launch arguments that we want to pass
    #    This is equivalent to writing 'name:=value' on the command line
    launch_args = {
        'left_namespace':  '/simple_drone/camera/left',
        'right_namespace': '/simple_drone/camera/right',
        'approximate_sync': 'True',  # Note: Pass boolean as a string 'True' or 'False'
        # The complex qos_overrides argument is a bit tricky. We pass it as a string.
        # The outer quotes are for Python, the inner ones are for the YAML syntax.
        'qos_overrides': "{'depth': { 'reliability': 'best_effort' }}",
        'left_camera_info_url': 'file://' +  os.path.join(dir, 'config', 'left.yaml'),
        'right_camera_info_url': 'file://' + os.path.join(dir, 'config', 'right.yaml'),
    }

    # 3. Create the IncludeLaunchDescription action
    #    This will run the specified launch file with our arguments
    stereo_processing_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(stereo_image_proc_launch),
        launch_arguments=launch_args.items()
    )

    # 4. Return the LaunchDescription object
    return LaunchDescription([
        stereo_processing_node
    ])