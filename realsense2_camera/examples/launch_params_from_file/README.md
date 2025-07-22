# Get RS ROS2 node params from YAML file
The following example gets the RS ROS2 node params from YAML file.
```
ros2 launch realsense2_camera rs_launch_get_params_from_yaml.py
```

By default, 'rs_launch_get_params_from_yaml.py' launch file uses the "/config/config.yaml" YAML file.

User can provide a different YAML file through cmd line as follows:
```
ros2 launch realsense2_camera rs_launch_get_params_from_yaml.py config_file:="/full/path/to/config/file"
```
or
```
ros2 launch realsense2_camera rs_launch.py config_file:="/full/path/to/config/file"
```

## Syntax for defining params in YAML file
```
param1: value
param2: value
```

Example:
```
enable_color: true
rgb_camera.color_profile: 1280x720x15
enable_depth: true
align_depth.enable: true
enable_sync: true
publish_tf: true
tf_publish_rate: 1.0
```