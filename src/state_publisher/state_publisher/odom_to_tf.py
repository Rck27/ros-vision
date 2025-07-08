# In your odom_to_tf.py

import rclpy
from rclpy.node import Node
# We will use the standard nav_msgs/msg/Odometry, since the publisher provides it.
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy



class OdomToTfNode(Node):
    def __init__(self):
        super().__init__('odom_to_tf_broadcaster')
        
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.subscription = self.create_subscription(
            Odometry, 
            '/fmu/out/vehicle_odometry',
            self.odom_callback,
            qos_profile) # <-- Use the custom QoS profile
        # 1. Create a TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # 2. Subscribe to the odometry topic.
        #    We explicitly ask for the nav_msgs/msg/Odometry type.
        self.subscription = self.create_subscription(
            Odometry,
            '/fmu/out/vehicle_odometry',
            self.odom_callback,
            10)
        self.get_logger().info('Odom to TF broadcaster started, listening for nav_msgs/Odometry.')

    def odom_callback(self, msg: Odometry): # Added type hint for clarity
        # This function is called every time a new Odometry message arrives
        self.get_logger().info('>>> Odom callback received a message!', once=True)

        # 3. Create a TransformStamped message
        t = TransformStamped()

        # 4. Fill in the message header
        t.header.stamp = self.get_clock().now().to_msg() # Using now() is fine for this test
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        # t.header.stamp = msg.header.stamp # **THE FIX**: Use the timestamp from the message!
        # t.header.frame_id = msg.header.frame_id # **THE FIX**: Use the frame_id from the message! (Usually 'odom')
        # t.child_frame_id = msg.child_frame_id # **THE FIX**: Use the child_frame_id from the message! (Usually 'base_link')

        # 5. Fill in the transform data
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z

        t.transform.rotation.x = msg.pose.pose.orientation.x
        t.transform.rotation.y = msg.pose.pose.orientation.y
        t.transform.rotation.z = msg.pose.pose.orientation.z
        t.transform.rotation.w = msg.pose.pose.orientation.w

        # 6. Broadcast the transform
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = OdomToTfNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()