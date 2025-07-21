import numpy as np
import ros2_numpy as rnp
from sensor_msgs.msg import PointCloud2

# Your original data generation
x_data = np.arange(100, dtype=np.float32)
y_data = x_data * 2
z_data = np.zeros(100, dtype=np.float32) # Create a Z component
vectors_data = np.arange(100, dtype=np.float32)[:, np.newaxis] * np.ones((1, 3), dtype=np.float32)

# --- THE FIX ---
# Combine x, y, z into a single Nx3 array
xyz_data = np.stack([x_data, y_data, z_data], axis=-1)

# Create the dictionary with the required 'xyz' key
point_cloud_dict = {
    'xyz': xyz_data,
    'vectors': vectors_data
}
# ----------------

# Now, msgify will work.
# We need to provide stamp and frame_id as keyword arguments.
from rclpy.time import Time
dummy_stamp = Time(seconds=0, nanoseconds=0).to_msg()
dummy_frame_id = 'map'

msg = rnp.msgify(PointCloud2, point_cloud_dict, frame_id=dummy_frame_id)

print("Successfully created PointCloud2 message with 'xyz' key.")
print(f"Number of points: {msg.width}")
print("Fields in the message:")
for field in msg.fields:
    print(f"- {field.name} (dtype: {field.datatype})")