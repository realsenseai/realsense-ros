# Launching RS ROS2 node from rosbag File
The following example allows streaming a rosbag file, saved by Intel RealSense Viewer, instead of streaming live with a camera. It can be used for testing and repetition of the same sequence.
```
ros2 launch realsense2_camera rs_launch_from_rosbag.py
```
By default, the 'rs_launch_from_rosbag.py' launch file uses the "/rosbag/D435i_Depth_and_IMU_Stands_still.bag" rosbag file.

User can also provide a different rosbag file through cmd line as follows:
```
ros2 launch realsense2_camera rs_launch_from_rosbag.py rosbag_filename:="/full/path/to/rosbag/file"
```
or
```
ros2 launch realsense2_camera rs_launch.py rosbag_filename:="/full/path/to/rosbag/file"
```

Additionally, the 'rosbag_loop' cmd line argument enables the looped playback of the rosbag file:
```
ros2 launch realsense2_camera rs_launch_from_rosbag.py rosbag_filename:="/full/path/to/rosbag/file" rosbag_loop:="true"
```

Check-out [sample-recordings](https://github.com/IntelRealSense/librealsense/blob/master/doc/sample-data.md) for a few recorded samples.