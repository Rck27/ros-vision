#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
import cv2
import numpy as np
import math

class DepthToLaserScanNode(Node):
    def __init__(self):
        super().__init__('depth_to_laserscan_node')
        self.bridge = CvBridge()
        self.left_img, self.right_img = None, None

        self.left_sub = self.create_subscription(Image, '/simple_drone/camera/left/image_raw', self.left_callback, 10)
        self.right_sub = self.create_subscription(Image, '/simple_drone/camera/right/image_raw', self.right_callback, 10)
        self.scan_pub = self.create_publisher(LaserScan, '/stereo/scan', 10)

        try:
            self.calib_data = np.load('stereo_calibration.npz')
            self.K1, self.D1, self.K2, self.D2, self.R, self.T = (self.calib_data[k] for k in ('K1','D1','K2','D2','R','T'))
            self.focal_length_x = self.K1[0, 0]
        except (FileNotFoundError, KeyError) as e:
            self.get_logger().error(f"Failed to load calibration data: {e}"); rclpy.shutdown(); return

        self.img_size = None
        self.map1_l, self.map2_l, self.map1_r, self.map2_r, self.Q = [None]*5
        # We don't need a UI, so we can hardcode some reasonable SGBM parameters
        self.stereo = cv2.StereoSGBM_create(
            numDisparities=128,
            blockSize=7,
            uniquenessRatio=10,
            speckleWindowSize=100,
            speckleRange=32,
            disp12MaxDiff=1
        )
        self.stereo.setP1(8*3*7**2)
        self.stereo.setP2(32*3*7**2)

        self.timer = self.create_timer(1.0 / 15.0, self.process_and_publish_scan)
        self.get_logger().info("HEADLESS Depth to LaserScan Node initialized.")

    def left_callback(self, msg):
        self.left_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        if self.img_size is None and self.left_img is not None:
            self.img_size = (self.left_img.shape[1], self.left_img.shape[0])
            self.horizontal_fov = 2 * math.atan(self.img_size[0] / (2 * self.focal_length_x))
            self._init_rectification()

    def right_callback(self, msg):
        self.right_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def _init_rectification(self):
        # Use alpha=1 for widest FOV
        R1, R2, P1, P2, self.Q, _, _ = cv2.stereoRectify(self.K1, self.D1, self.K2, self.D2, self.img_size, self.R, self.T, alpha=1)
        self.map1_l, self.map2_l = cv2.initUndistortRectifyMap(self.K1, self.D1, R1, P1, self.img_size, cv2.CV_32FC1)
        self.map1_r, self.map2_r = cv2.initUndistortRectifyMap(self.K2, self.D2, R2, P2, self.img_size, cv2.CV_32FC1)

    def process_and_publish_scan(self):
        if self.left_img is None or self.right_img is None or self.map1_l is None: return
        
        # --- NO MORE cv2.waitKey() or cv2.imshow() ---

        # Rectify and compute disparity
        rect_l = cv2.remap(cv2.cvtColor(self.left_img, cv2.COLOR_BGR2GRAY), self.map1_l, self.map2_l, cv2.INTER_LINEAR)
        rect_r = cv2.remap(cv2.cvtColor(self.right_img, cv2.COLOR_BGR2GRAY), self.map1_r, self.map2_r, cv2.INTER_LINEAR)
        disparity_map = self.stereo.compute(rect_l, rect_r).astype(np.float32) / 16.0

        points_3d = cv2.reprojectImageTo3D(disparity_map, self.Q)
        depth_map = points_3d[:, :, 2]

        h, w = depth_map.shape
        scan_slice = depth_map[h // 2]
        
        scan_slice[np.isinf(scan_slice)] = 0.0
        scan_slice[np.isnan(scan_slice)] = 0.0
        
        scan_msg = LaserScan()
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = 'camera_mount'
        
        scan_msg.angle_min = -self.horizontal_fov / 2.0
        scan_msg.angle_max = self.horizontal_fov / 2.0
        scan_msg.angle_increment = self.horizontal_fov / (w - 1)
        scan_msg.scan_time = self.timer.timer_period_ns / 1e9
        scan_msg.range_min = 0.2
        scan_msg.range_max = 10.0
        
        scan_slice[scan_slice < scan_msg.range_min] = 0.0
        scan_slice[scan_slice > scan_msg.range_max] = 0.0
        
        scan_msg.ranges = scan_slice.tolist()

        self.scan_pub.publish(scan_msg)
        self.get_logger().info("Published a LaserScan message.", throttle_duration_sec=1)

def main(args=None):
    rclpy.init(args=args)
    node = DepthToLaserScanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        # No need to destroy OpenCV windows since we didn't create them
        if node and rclpy.ok(): node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()