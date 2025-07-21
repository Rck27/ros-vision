#!/usr/bin/env python3

from time import sleep
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import ros2_numpy as rnp
from sensor_msgs.msg import PointCloud2
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy


class StereoDepthNode(Node):
    def __init__(self):
        super().__init__('stereo_depth_node')
        self.bridge = CvBridge()

        # Image holders
        self.left_img = None
        self.right_img = None
        qos_profile = QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1  # Keep only the latest message
            )
        # -------------------
        # Subscribers
        self.left_sub = self.create_subscription(Image, '/simple_drone/camera/left/image_raw', self.left_callback, 10)
        self.right_sub = self.create_subscription(Image, '/simple_drone/camera/right/image_raw', self.right_callback, 10)
        self.pcd_pub = self.create_publisher(PointCloud2, '/stereo/points2', 10)
        # --- TIMER-BASED APPROACH (THE FIX) ---
        # Define the publishing rate (e.g., 10 Hz)
        self.timer_period = 1.0 / 10.0  # seconds
        # Create a timer that will call the process_frames function periodically
        self.timer = self.create_timer(self.timer_period, self.process_frames)
        # Load calibration data
        try:
            self.calib_data = np.load('stereo_calibration.npz')
            self.K1 = self.calib_data['K1']
            self.D1 = self.calib_data['D1']
            self.K2 = self.calib_data['K2']
            self.D2 = self.calib_data['D2']
            self.R = self.calib_data['R']
            self.T = self.calib_data['T']
            self.get_logger().info("Successfully loaded calibration data.")
            # Sanity check the baseline from calibration vs. your URDF
            baseline_calib = np.linalg.norm(self.T)
            self.get_logger().info(f"URDF baseline is ~0.14m. Calibrated baseline is {baseline_calib:.4f}m.")

        except FileNotFoundError:
            self.get_logger().error("Calibration file 'stereo_calibration.npz' not found! Shutting down.")
            rclpy.shutdown()
            return
            
        # Rectification and Disparity parameters
        self.img_size = None
        self.map1_l, self.map2_l = None, None
        self.map1_r, self.map2_r = None, None
        self.Q = None

        # StereoSGBM algorithm instance
        # You will need to TUNE these parameters for your specific setup
        self.stereo = cv2.StereoSGBM_create(
            minDisparity=0,
            numDisparities=64,       # Must be divisible by 16
            blockSize=9,             # Must be odd (3-11 are good values)
            P1=8 * 3 * self.K1.shape[0]**2,  # P1 and P2 can be related to image channels and blocksize
            P2=32 * 3 * self.K1.shape[0]**2,
            disp12MaxDiff=1,
            uniquenessRatio=10,
            speckleWindowSize=100,
            speckleRange=32
        )
        self.get_logger().info("StereoDepthNode initialized.")

    def left_callback(self, msg):
        try:
            self.left_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            # Initialize rectification maps only once
            if self.img_size is None and self.left_img is not None:
                self.img_size = (self.left_img.shape[1], self.left_img.shape[0]) # (width, height)
                self._init_rectification()
        except Exception as e:
            self.get_logger().error(f"Left camera callback error: {e}")

    def right_callback(self, msg):
        try:
            self.right_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Right camera callback error: {e}")

    def _init_rectification(self):
        self.get_logger().info("Initializing rectification maps...")
        try:
            R1, R2, P1, P2, self.Q, _, _ = cv2.stereoRectify(
                self.K1, self.D1, self.K2, self.D2,
                self.img_size, self.R, self.T, alpha=0.9
            )
            self.map1_l, self.map2_l = cv2.initUndistortRectifyMap(
                self.K1, self.D1, R1, P1, self.img_size, cv2.CV_32FC1
            )
            self.map1_r, self.map2_r = cv2.initUndistortRectifyMap(
                self.K2, self.D2, R2, P2, self.img_size, cv2.CV_32FC1
            )
            self.get_logger().info("Rectification maps initialized successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize rectification: {e}")

    def process_frames(self):
        # --- Saftey Checks (Unchanged) ---
        if self.left_img is None or self.right_img is None:
            self.get_logger().info("Waiting for both images...", throttle_duration_sec=2)
            return
        if self.map1_l is None or self.Q is None:
            self.get_logger().info("Waiting for rectification maps...", throttle_duration_sec=2)
            return

        # --- Rectification (Updated for Color) ---
        # We need both a grayscale version for SGBM and a color version for the point cloud.
        gray_l = cv2.cvtColor(self.left_img, cv2.COLOR_BGR2GRAY)
        gray_r = cv2.cvtColor(self.right_img, cv2.COLOR_BGR2GRAY)
        
        # Rectify the grayscale images for the stereo algorithm
        rect_l_gray = cv2.remap(gray_l, self.map1_l, self.map2_l, cv2.INTER_LINEAR)
        rect_r_gray = cv2.remap(gray_r, self.map1_r, self.map2_r, cv2.INTER_LINEAR)

        # Rectify the original color image to get colors for the point cloud
        rect_l_color = cv2.remap(self.left_img, self.map1_l, self.map2_l, cv2.INTER_LINEAR)

        # --- Disparity and 3D Projection (Unchanged) ---
        # Compute disparity map using the grayscale rectified images
        disparity_map = self.stereo.compute(rect_l_gray, rect_r_gray).astype(np.float32) / 16.0
        
        # Use the Q matrix to reproject the disparity to 3D space
        points_3d = cv2.reprojectImageTo3D(disparity_map, self.Q)

        # --- THIS IS THE NEW SECTION THAT CREATES THE REQUIRED VARIABLES ---
        # 1. Create a mask for valid points
        # A valid point has a positive disparity and is not infinitely far away.
        MAX_DEPTH = 10.0
        mask = (disparity_map > 0) & (points_3d[:, :, 2] < MAX_DEPTH)

        # 2. Apply the mask to get the list of valid 3D points
        valid_points = points_3d[mask]
        
        # --- ROBUSTNESS CHECK ---
        # If there are no valid points, there's nothing more to do this frame.
        # This prevents the crash.
        if valid_points.shape[0] == 0:
            self.get_logger().warn("No valid points found in the scene. Is it textureless?", throttle_duration_sec=5)
            # You could publish an empty point cloud here if you want, or just return.
            # Let's just return to keep it simple.
            return
        # ------------------------

        # 3. Apply the same mask to the color image to get the corresponding colors
        colors_bgr = rect_l_color[mask]
        valid_colors = cv2.cvtColor(colors_bgr.reshape(-1, 1, 3), cv2.COLOR_BGR2RGB).reshape(-1, 3)

        # --- Build and Publish PointCloud2 Message ---
        # Now that we have `valid_points` and `valid_colors`, this will work.
        point_cloud_dict = {
            'xyz': valid_points,
            'rgb': valid_colors # Use 'rgb' key for color
        }

        pcd_msg = rnp.msgify(
            PointCloud2,
            point_cloud_dict,
            # stamp=self.get_clock().now().to_msg(),
            frame_id='camera_mount' # Make sure this matches your TF tree
        )
        self.pcd_pub.publish(pcd_msg)
        

        # --- Visualization (Updated to use clearer variables) ---
        disparity_vis = cv2.normalize(disparity_map, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
        
        # Show the rectified color image
        cv2.imshow("Rectified Left", rect_l_color)
        cv2.imshow("Disparity", disparity_vis)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            raise KeyboardInterrupt
def main(args=None):
    rclpy.init(args=args)
    node = StereoDepthNode()
    
    try:
        # This loop runs your processing while also handling ROS callbacks
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1) # Check for new messages
            node.process_frames() # Run your main logic
            # sleep(0.1)
    except (KeyboardInterrupt, Exception) as e:
        if isinstance(e, KeyboardInterrupt):
            print("Shutting down cleanly.")
        else:
            node.get_logger().error(f"Node crashed with error: {e}")
            import traceback
            traceback.print_exc()

    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()