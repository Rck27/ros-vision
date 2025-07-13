#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import time

class CalibrationGuidance:
    """A helper class to manage and display calibration guidance UI."""
    def __init__(self, image_shape):
        self.h, self.w = image_shape[:2]
        self.dashboard = np.zeros((300, 300, 3), dtype=np.uint8)

        # Define bins for coverage. We want to fill all of them.
        self.coverage = {
            'x': {'left': False, 'center': False, 'right': False},
            'y': {'top': False, 'center': False, 'bottom': False},
            'size': {'small': False, 'large': False},
            'skew': {'low': False, 'high': False}
        }

    def _draw_progress_bar(self, y_pos, label, value, max_value):
        cv2.putText(self.dashboard, label, (10, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        # Background of the bar
        cv2.rectangle(self.dashboard, (100, y_pos - 15), (280, y_pos + 5), (50, 50, 50), -1)
        # Filled part of the bar
        fill_width = int((value / max_value) * 180)
        cv2.rectangle(self.dashboard, (100, y_pos - 15), (100 + fill_width, y_pos + 5), (0, 255, 0), -1)

    def update_dashboard(self, current_metrics=None):
        self.dashboard.fill(0) # Reset dashboard
        
        # --- Draw current status bars ---
        total_bins = sum(len(v) for v in self.coverage.values())
        filled_bins = sum(v for d in self.coverage.values() for v in d.values())
        self._draw_progress_bar(30, "Overall", filled_bins, total_bins)

        x_bins = len(self.coverage['x'])
        x_filled = sum(self.coverage['x'].values())
        self._draw_progress_bar(70, "X (Horz)", x_filled, x_bins)
        
        y_bins = len(self.coverage['y'])
        y_filled = sum(self.coverage['y'].values())
        self._draw_progress_bar(110, "Y (Vert)", y_filled, y_bins)

        size_bins = len(self.coverage['size'])
        size_filled = sum(self.coverage['size'].values())
        self._draw_progress_bar(150, "Size", size_filled, size_bins)
        
        skew_bins = len(self.coverage['skew'])
        skew_filled = sum(self.coverage['skew'].values())
        self._draw_progress_bar(190, "Skew", skew_filled, skew_bins)

        # --- Draw a live indicator for the current frame ---
        if current_metrics:
            x_pos = int((current_metrics['x_center'] / self.w) * 180)
            y_pos = int((current_metrics['y_center'] / self.h) * 180)
            size_val = int((current_metrics['size_area'] / (self.w * self.h * 0.5)) * 180)
            skew_val = int((current_metrics['skew_ratio'] / 5.0) * 180)

            # Draw a small circle indicating current position
            cv2.circle(self.dashboard, (100 + x_pos, 70 - 5), 5, (0, 165, 255), -1)
            cv2.circle(self.dashboard, (100 + y_pos, 110 - 5), 5, (0, 165, 255), -1)
            cv2.circle(self.dashboard, (100 + size_val, 150 - 5), 5, (0, 165, 255), -1)
            cv2.circle(self.dashboard, (100 + skew_val, 190 - 5), 5, (0, 165, 255), -1)
        
        cv2.imshow("Calibration Dashboard", self.dashboard)

    def analyze_corners(self, corners):
        """Calculate metrics based on the detected corners."""
        hull = cv2.convexHull(corners)
        size_area = cv2.contourArea(hull)
        
        # Centroid for X/Y position
        M = cv2.moments(hull)
        x_center = M['m10'] / M['m00'] if M['m00'] != 0 else 0
        y_center = M['m01'] / M['m00'] if M['m00'] != 0 else 0

        # Skew via aspect ratio of minimum area rectangle
        _, (w, h), _ = cv2.minAreaRect(corners)
        aspect_ratio = max(w, h) / min(w, h) if min(w, h) > 0 else 0
        
        return {'size_area': size_area, 'x_center': x_center, 'y_center': y_center, 'skew_ratio': aspect_ratio}

    def update_coverage(self, metrics):
        """Update the coverage bins based on the latest capture."""
        # Update X
        if metrics['x_center'] < self.w * 0.33: self.coverage['x']['left'] = True
        elif metrics['x_center'] > self.w * 0.66: self.coverage['x']['right'] = True
        else: self.coverage['x']['center'] = True
        
        # Update Y
        if metrics['y_center'] < self.h * 0.33: self.coverage['y']['top'] = True
        elif metrics['y_center'] > self.h * 0.66: self.coverage['y']['bottom'] = True
        else: self.coverage['y']['center'] = True
        
        # Update Size
        if metrics['size_area'] < self.w * self.h * 0.1: self.coverage['size']['small'] = True
        elif metrics['size_area'] > self.w * self.h * 0.3: self.coverage['size']['large'] = True
        
        # Update Skew
        if metrics['skew_ratio'] < 1.5: self.coverage['skew']['low'] = True # relatively flat
        elif metrics['skew_ratio'] > 2.5: self.coverage['skew']['high'] = True # highly tilted

class StereoCalibration(Node):
    def __init__(self):
        super().__init__('stereo_calibration_node')
        self.bridge = CvBridge()
        self.left_img, self.right_img = None, None
        self.is_done = False
        
        self.objpoints, self.imgpoints_left, self.imgpoints_right = [], [], []
        self.capture_count = 0

        self.CHECKERBOARD = (5, 7) # IMPORTANT: Inner corners count.
        self.square_size_m = 0.0254  # IMPORTANT: Your square size in meters.
        
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        self.objp = np.zeros((self.CHECKERBOARD[0] * self.CHECKERBOARD[1], 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:self.CHECKERBOARD[0], 0:self.CHECKERBOARD[1]].T.reshape(-1, 2)
        self.objp *= self.square_size_m

        self.left_sub = self.create_subscription(Image, '/simple_drone/camera/left/image_raw', self.left_callback, 10)
        self.right_sub = self.create_subscription(Image, '/simple_drone/camera/right/image_raw', self.right_callback, 10)
        
        self.guidance = None # Will be initialized when first image arrives
        self.last_capture_time = 0

    def left_callback(self, msg):
        self.left_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        if self.guidance is None and self.left_img is not None:
            self.guidance = CalibrationGuidance(self.left_img.shape)

    def right_callback(self, msg):
        self.right_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def capture_pair(self):
        if time.time() - self.last_capture_time < 2: # Debounce button press
            self.get_logger().warn("Please wait a moment before capturing again.")
            return

        if self.left_img is None or self.right_img is None:
            self.get_logger().warn("Images not ready.")
            return

        gray_left = cv2.cvtColor(self.left_img, cv2.COLOR_BGR2GRAY)
        gray_right = cv2.cvtColor(self.right_img, cv2.COLOR_BGR2GRAY)

        find_flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
        ret_l, corners_l = cv2.findChessboardCorners(gray_left, self.CHECKERBOARD, find_flags)
        ret_r, corners_r = cv2.findChessboardCorners(gray_right, self.CHECKERBOARD, find_flags)

        if ret_l and ret_r:
            self.last_capture_time = time.time()
            self.capture_count += 1
            self.get_logger().info(f"Pair #{self.capture_count} FOUND in both images!")
            
            corners2_l = cv2.cornerSubPix(gray_left, corners_l, (11, 11), (-1, -1), self.criteria)
            corners2_r = cv2.cornerSubPix(gray_right, corners_r, (11, 11), (-1, -1), self.criteria)

            self.objpoints.append(self.objp)
            self.imgpoints_left.append(corners2_l)
            self.imgpoints_right.append(corners2_r)

            # Update guidance system with this successful capture
            metrics = self.guidance.analyze_corners(corners_l)
            self.guidance.update_coverage(metrics)
        else:
            self.get_logger().warn("Checkerboard NOT found in one or both images. Try a different angle.")

    def run_calibration_cycle(self):
        if self.is_done: return
        if self.left_img is None or self.guidance is None:
            self.get_logger().info("Waiting for camera feed...", throttle_duration_sec=2)
            return

        display_left = self.left_img.copy()
        
        # Analyze current corners for live feedback (if they exist)
        gray_left = cv2.cvtColor(display_left, cv2.COLOR_BGR2GRAY)
        ret_l, corners_l = cv2.findChessboardCorners(gray_left, self.CHECKERBOARD, cv2.CALIB_CB_FAST_CHECK)
        
        current_metrics = None
        if ret_l:
            # Draw corners for visual feedback
            cv2.drawChessboardCorners(display_left, self.CHECKERBOARD, corners_l, ret_l)
            current_metrics = self.guidance.analyze_corners(corners_l)

        self.guidance.update_dashboard(current_metrics)
        
        cv2.putText(display_left, f'Captured: {self.capture_count}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.imshow("Left Camera", display_left)
        if self.right_img is not None:
            cv2.imshow("Right Camera", self.right_img)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('y'):
            self.capture_pair()
        elif key == ord('n'):
            self.get_logger().info("--- Finalizing and running calibration. ---")
            if len(self.objpoints) > 5:
                self.do_stereo_calibration()
            else:
                self.get_logger().error(f"Need at least 5 captures, only have {len(self.objpoints)}. Capture more or press 'Q' to quit.")
            self.is_done = True
        elif key == ord('q'):
            self.is_done = True

    def do_stereo_calibration(self):
        # This function remains the same as before
        self.get_logger().info(f"Starting calibration with {len(self.objpoints)} image pairs...")
        img_shape = self.left_img.shape[:2][::-1]
        
        ret_l, K1, D1, _, _ = cv2.calibrateCamera(self.objpoints, self.imgpoints_left, img_shape, None, None)
        ret_r, K2, D2, _, _ = cv2.calibrateCamera(self.objpoints, self.imgpoints_right, img_shape, None, None)

        flags = cv2.CALIB_FIX_INTRINSIC
        ret, K1, D1, K2, D2, R, T, E, F = cv2.stereoCalibrate(
            self.objpoints, self.imgpoints_left, self.imgpoints_right,
            K1, D1, K2, D2, img_shape, criteria=self.criteria, flags=flags)

        if ret:
            self.get_logger().info(f"Stereo calibration successful! RMS Error: {ret}")
            output_file = "stereo_calibration.npz"
            np.savez(output_file, K1=K1, D1=D1, K2=K2, D2=D2, R=R, T=T, E=E, F=F)
            self.get_logger().info(f"Calibration data saved to '{os.path.abspath(output_file)}'")
        else:
            self.get_logger().error("Stereo calibration failed!")

def main(args=None):
    rclpy.init(args=args)
    calibrator = StereoCalibration()
    
    print("\n--- Interactive Stereo Calibration with Guidance ---")
    print("1. A 'Calibration Dashboard' window will appear.")
    print("2. Move the checkerboard to fill the progress bars for X, Y, Size, and Skew.")
    print("3. An orange circle on a bar shows the board's CURRENT position.")
    print("4. Press 'Y' to capture a good, varied position. The bars will fill up.")
    print("5. Once all bars are green, press 'N' to calibrate.")
    print("6. Press 'Q' to quit.\n")

    try:
        while rclpy.ok() and not calibrator.is_done:
            rclpy.spin_once(calibrator, timeout_sec=0.01)
            calibrator.run_calibration_cycle()
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        calibrator.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()