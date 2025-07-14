#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class StereoDepthNode(Node):
    def __init__(self):
        super().__init__('stereo_depth_node')
        # ... (initialization is the same)
        self.bridge = CvBridge()
        self.left_img, self.right_img = None, None

        self.left_sub = self.create_subscription(Image, '/simple_drone/camera/left/image_raw', self.left_callback, 10)
        self.right_sub = self.create_subscription(Image, '/simple_drone/camera/right/image_raw', self.right_callback, 10)

        try:
            self.calib_data = np.load('stereo_calibration.npz')
            self.K1, self.D1 = self.calib_data['K1'], self.calib_data['D1']
            self.K2, self.D2 = self.calib_data['K2'], self.calib_data['D2']
            self.R, self.T = self.calib_data['R'], self.calib_data['T']
            self.get_logger().info("Successfully loaded calibration data.")
        except FileNotFoundError:
            self.get_logger().error("Calibration file 'stereo_calibration.npz' not found! Shutting down.")
            rclpy.shutdown()
            return

        self.img_size = None
        self.map1_l, self.map2_l = None, None
        self.map1_r, self.map2_r = None, None
        self.Q = None
        self.stereo = cv2.StereoSGBM_create()
        self._create_tuning_ui()
        self.get_logger().info("StereoDepthNode initialized.")

    def _create_tuning_ui(self):
        cv2.namedWindow('SGBM Tuner')
        def on_trackbar(val): pass
        
        # --- NEW: alpha trackbar for rectification zoom ---
        # We use a scale of 100 so the slider is integer-based
        cv2.createTrackbar('alpha (x100)', 'SGBM Tuner', 0, 100, on_trackbar)
        
        cv2.createTrackbar('numDisparities', 'SGBM Tuner', 4, 16, on_trackbar)
        cv2.createTrackbar('blockSize', 'SGBM Tuner', 4, 10, on_trackbar)
        # ... (rest of the SGBM trackbars are the same)
        cv2.createTrackbar('uniquenessRatio', 'SGBM Tuner', 10, 25, on_trackbar)
        cv2.createTrackbar('speckleWindowSize', 'SGBM Tuner', 100, 200, on_trackbar)
        cv2.createTrackbar('speckleRange', 'SGBM Tuner', 32, 50, on_trackbar)
        cv2.createTrackbar('disp12MaxDiff', 'SGBM Tuner', 1, 25, on_trackbar)
        cv2.createTrackbar('minDisparity', 'SGBM Tuner', 0, 25, on_trackbar)

    def _update_sgbm_from_ui(self):
        # ... (This function is unchanged)
        numDisparities = max(1, cv2.getTrackbarPos('numDisparities', 'SGBM Tuner')) * 16
        blockSize = cv2.getTrackbarPos('blockSize', 'SGBM Tuner') * 2 + 3
        uniquenessRatio = max(1, cv2.getTrackbarPos('uniquenessRatio', 'SGBM Tuner'))
        speckleWindowSize = cv2.getTrackbarPos('speckleWindowSize', 'SGBM Tuner')
        speckleRange = cv2.getTrackbarPos('speckleRange', 'SGBM Tuner')
        disp12MaxDiff = cv2.getTrackbarPos('disp12MaxDiff', 'SGBM Tuner')
        minDisparity = cv2.getTrackbarPos('minDisparity', 'SGBM Tuner')
        
        self.stereo.setNumDisparities(numDisparities)
        self.stereo.setBlockSize(blockSize)
        self.stereo.setUniquenessRatio(uniquenessRatio)
        self.stereo.setSpeckleWindowSize(speckleWindowSize)
        self.stereo.setSpeckleRange(speckleRange)
        self.stereo.setDisp12MaxDiff(disp12MaxDiff)
        self.stereo.setMinDisparity(minDisparity)
        
        P1 = 8 * 3 * blockSize**2
        P2 = 32 * 3 * blockSize**2
        self.stereo.setP1(P1)
        self.stereo.setP2(P2)


    def left_callback(self, msg):
        try:
            self.left_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            if self.img_size is None and self.left_img is not None:
                self.img_size = (self.left_img.shape[1], self.left_img.shape[0])
                # Initial rectification on startup
                self._init_rectification()
        except Exception as e:
            self.get_logger().error(f"Left camera callback error: {e}")

    def right_callback(self, msg):
        try:
            self.right_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Right camera callback error: {e}")

    def _init_rectification(self):
        # --- UPDATED: Reads alpha value from the UI ---
        try:
            alpha_val = cv2.getTrackbarPos('alpha (x100)', 'SGBM Tuner') / 100.0
            self.get_logger().info(f"Re-initializing rectification with alpha = {alpha_val}")

            R1, R2, P1, P2, self.Q, _, _ = cv2.stereoRectify(
                self.K1, self.D1, self.K2, self.D2,
                self.img_size, self.R, self.T, alpha=alpha_val
            )
            self.map1_l, self.map2_l = cv2.initUndistortRectifyMap(self.K1, self.D1, R1, P1, self.img_size, cv2.CV_32FC1)
            self.map1_r, self.map2_r = cv2.initUndistortRectifyMap(self.K2, self.D2, R2, P2, self.img_size, cv2.CV_32FC1)
            self.get_logger().info("Rectification maps updated successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize rectification: {e}")

    def process_frames(self):
        if self.left_img is None or self.right_img is None or self.map1_l is None:
            return

        self._update_sgbm_from_ui()

        rect_l = cv2.remap(cv2.cvtColor(self.left_img, cv2.COLOR_BGR2GRAY), self.map1_l, self.map2_l, cv2.INTER_LINEAR)
        rect_r = cv2.remap(cv2.cvtColor(self.right_img, cv2.COLOR_BGR2GRAY), self.map1_r, self.map2_r, cv2.INTER_LINEAR)
        
        disparity_map = self.stereo.compute(rect_l, rect_r).astype(np.float32) / 16.0
        
        # --- Visualization ---
        disparity_vis = cv2.normalize(disparity_map, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)

        # To check alignment, draw horizontal lines on the combined rectified image
        combined_rectified = np.hstack((rect_l, rect_r))
        h, w = combined_rectified.shape
        for i in range(1, 10):
            cv2.line(combined_rectified, (0, h * i // 10), (w, h * i // 10), (0, 255, 0), 1)

        cv2.imshow("Rectified Pair (with alignment lines)", combined_rectified)
        cv2.imshow("Disparity Map", disparity_vis)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            raise KeyboardInterrupt
        elif key == ord('r'):
            # --- NEW: Re-run rectification on key press ---
            self._init_rectification()

# ... (main function is unchanged)
def main(args=None):
    rclpy.init(args=args)
    node = StereoDepthNode()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            node.process_frames()
    except (KeyboardInterrupt, Exception) as e:
        if not isinstance(e, KeyboardInterrupt):
            node.get_logger().error(f"Node crashed with error: {e}")
            import traceback
            traceback.print_exc()
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()