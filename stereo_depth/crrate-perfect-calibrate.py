#!/usr/bin/env python3

import numpy as np
import os

def create_perfect_calibration_file():
    """
    Creates a 'stereo_calibration.npz' file with perfect ground-truth
    values from a Gazebo simulation.
    """
    print("Creating perfect calibration file for Gazebo simulation...")

    # --- Step 1: Intrinsics (K and D) ---
    # PASTE THE VALUES YOU GOT FROM 'ros2 topic echo' HERE.
    # The cameras are identical, so we can use the same values for both.
    
    # This is a 3x3 matrix.
    K_left = np.array([
        [277.19135641,0, 160.5],
        [0,277.19135641, 120.5],
        [0,  0,  1]
    ])

    # This is a 1x5 vector.
    D_left = np.array([[0.0, 0.0, 0.0, 0.0, 0.0]])

    # Since the right camera is identical in the URDF, we use the same intrinsics.
    K_right = K_left
    D_right = D_left

    # --- Step 2: Extrinsics (R and T) ---
    # These are calculated from your URDF.
    
    # Rotation: No rotation between cameras, so it's the 3x3 Identity matrix.
    R = np.identity(3)

    # Translation: 0.14m along the Y-axis.
    # T must be a column vector (3, 1) or a flat array for np.savez.
    T = np.array([0.0, 0.14, 0.0])

    # --- Step 3: Save the file ---
    output_file = "stereo_calibration.npz"
    
    np.savez(
        output_file,
        K1=K_left,
        D1=D_left,
        K2=K_right,
        D2=D_right,
        R=R,
        T=T
    )

    print(f"Successfully created '{output_file}' in the current directory:")
    print(os.path.abspath(output_file))
    print("\nYou can now run your stereo_depth_node.")


if __name__ == '__main__':
    create_perfect_calibration_file()