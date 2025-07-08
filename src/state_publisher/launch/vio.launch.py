# In your_package/launch/vio.launch.py
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():

    # This is the "brute force" method. We manually construct the command line
    # to start the container and load the libraries. This bypasses the Ament
    # index discovery that is failing.

    # 1. Define the command to start the container process.
    #    The 'ros2 run rclcpp_components component_container' command is the
    #    standard way to start a generic container.
    command_to_run = [
        'ros2',
        'run',
        'rclcpp_components',
        'component_container',
        '--ros-args',

        # =======================================================
        # === REMAP THE ODOMETRY NODE'S TOPICS
        # =======================================================
        '-r', '__node:=rtabmap_stereo_odometry', # Give the node a name
        '-r', 'left/image_rect:=/camera/left/image_rect',
        '-r', 'right/image_rect:=/camera/right/image_rect',
        '-r', 'left/camera_info:=/camera/left/camera_info',
        '-r', 'right/camera_info:=/camera/right/camera_info',
        '-r', 'odom:=/odom',

        # =======================================================
        # === SET THE ODOMETRY NODE'S PARAMETERS
        # =======================================================
        '-p', 'frame_id:=base_link',
        '-p', 'odom_frame_id:=odom',
        '-p', 'publish_tf:=true',
        '-p', 'approx_sync:=true',
        
        # =======================================================
        # === NOW, DO THE SAME FOR THE SLAM NODE
        # =======================================================
        '-r', '__node:=rtabmap_slam', # Give the SLAM node a name
        '-r', 'left/image_rect:=/camera/left/image_rect',
        '-r', 'right/image_rect:=/camera/right/image_rect',
        '-r', 'left/camera_info:=/camera/left/camera_info',
        '-r', 'right/camera_info:=/camera/right/camera_info',
        '-r', 'odom:=/odom',
        
        '-p', 'subscribe_depth:=false',
        '-p', 'subscribe_stereo:=true',
        '-p', 'approx_sync:=true',
    ]

    # 2. Create the ExecuteProcess action
    #    This will run the command we just built.
    rtabmap_process = ExecuteProcess(
        cmd=command_to_run,
        output='screen'
    )
    
    # 3. Return the launch description
    return LaunchDescription([
        rtabmap_process
    ])