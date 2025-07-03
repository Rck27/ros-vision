# ROS VISION

control single drone autonomously using stereo camera and UWB positioning module


### whats working?

1. PX4 iris models with camera
2. Gazebo-classic world (not accurate yet)
3. state publisher for drone frames
4. stereo image proc for producing point cloud using 2 cameras (uncalibrated)
5. soon :)


### To-do list

1. get [VIO](https://github.com/introlab/rtabmap_ros?) working (see vio.launch.py)
2. get [UWB simulation](https://github.com/Xpect8tions/Decawave-ros-data-sim) working
3. fuse VIO and UWB data together (how?)
4. get [NAV2](https://docs.nav2.org/) running to read point cloud and generate trajectory
5. make drone goes whoosh by its own
6. idk

### Dependencies

idk, just try it :)


### Get it running

```
colcon build
ros2 launch state_publisher main.launch.py
#or you can run each launch.py individually
```
