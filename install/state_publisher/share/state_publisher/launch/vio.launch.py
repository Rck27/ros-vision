# In your_package/launch/vio.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # This is the launch description for the VIO system.
    # It starts the stereo_odometry node from the rtabmap_ros package.

    return LaunchDescription([
        Node(
            package='rtabmap_ros',
            executable='stereo_odometry',
            name='rtabmap_stereo_odometry',
            output='screen',
            parameters=[{
                'frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'publish_tf': True,
                'approx_sync': True,
                'queue_size': 10,
            }],
            # This 'remappings' section is the guaranteed way to connect the topics.
            remappings=[
                ('left/image_rect', '/camera/left/image_rect'),
                ('right/image_rect', '/camera/right/image_rect'),
                ('left/camera_info', '/camera/left/camera_info'),
                ('right/camera_info', '/camera/right/camera_info'),
            ]
        )
    ])