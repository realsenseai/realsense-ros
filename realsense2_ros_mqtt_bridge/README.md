
<p align="center">
  <!-- Light mode -->
  <img src="../res/realsense-logo-light-mode.png#gh-light-mode-only" alt="Logo for light mode" width="70%"/>

  <!-- Dark mode -->
  <img src="../res/realsense-logo-dark-mode.png#gh-dark-mode-only" alt="Logo for dark mode" width="70%"/>
  <br><br>
</p>



<h1 align="center">
  MQTT <-> ROS bridge for Intel&copy; RealSense&trade; Cameras<br>
</h1>


## Table of contents
- [Installation](#installation)
- [Starting the ros-mqtt-bridge node](#starting-the-ros-mqtt-bridge-node)
    - [ros2 run](#ros2-run)
    - [ros2 launch](#ros2-launch)
- [Parameters](#parameters)
- [Client Usage](#client-usage)
  - [Enumerate Devices](#enumerate-devices)
  - [Reset the device](#reset-the-device)
  - [Get Device Info](#get-device-info)
  - [Get Transformation](#get-transformation)
  - [Send HWM Command](#send-hwm-command)
  - [Get Parameter](#get-parameter)
  - [Set Parameter](#set-parameter)
  - [Get Frame](#get-frame)
  - [Get Safety Preset](#get-safety-preset)
  - [Set Safety Preset](#set-safety-preset)
  - [Get Safety Interface Config](#get-safety-interface-config)
  - [Set Safety Interface Config](#set-safety-interface-config)
  - [Get Calib Config](#get-calib-config)
  - [Set Calib Config](#set-calib-config)
  - [Get Safety Application Config](#get-application-config)
  - [Set Safety Application Config](#set-application-config)
  - [Triggered Calibration](#triggered-calibration)
- [Supported Parameters For Set/Get](#supported-parameters-for-setget)
- [Supported Streams](#supported-streams)
- [Usage Example](#usage-example)

# Installation  
***This step assumes you have installed ROS environment and ROS Wrapper for Realsense Cameras (including RealSense SDK). For more info about these steps, click [here](https://github.com/realsenseai/realsense-ros/tree/ros2-development?tab=readme-ov-file)***

  - Install paho-mqtt from https://pypi.org/project/paho-mqtt/2.1.0/
    ```
    sudo pip3 install paho-mqtt==2.1.0
    ```

  - Create a ROS2 workspace
    ```bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src/
    ```
    
  - Build
    ```bash
    colcon build
    ```

  - Source environment
    ```bash
    ROS_DISTRO=<YOUR_SYSTEM_ROS_DISTRO>  # set your ROS_DISTRO: iron, humble
    source /opt/ros/$ROS_DISTRO/setup.bash
    cd ~/ros2_ws
    ./install/local_setup.bash
    ```

# Starting the ros-mqtt-bridge node
***this step assumes there is a running and configured MQTT broker, and at least one running realsense2_camera node***
  ### ros2 run
    ros2 run realsense2_ros_mqtt_bridge realsense2_ros_mqtt_bridge
    # or, with parameters, for example
    ros2 run realsense2_ros_mqtt_bridge realsense2_ros_mqtt_bridge --ros-args -p broker_ip:='localhost'
  ### ros2 launch
    ros2 launch realsense2_ros_mqtt_bridge rs_launch.py
    # or, with parameters, for example
    ros2 launch realsense2_ros_mqtt_bridge rs_launch.py broker_ip:='localhost'

# Parameters
***All parameters can be configured or overriden by the `ros2 run` and `ros2 launch` commands (see examples in above section), or from the `rs_launch.py` file***
- broker_ip
  - description: MQTT broker ip address
  - default value: 'localhost'
- broker_port
  - description: MQTT port
  - default value: '1883'
- log_level
  - description: log level [DEBUG|INFO|WARN|ERROR|FATAL]
  - default value: INFO

# Client Usage
## Enumerate Devices
* mqtt request message example
  ```
    {
      "camera_namespace_prefix": "robot",
      "camera_names_prefix": "c_"
    }
  ```
* request topic
  ```
  enumrete_devices_request
  ```
* response topic
  ```
  enumerate_devices_response
  ```
* mqtt response message example:
  ```
    {
      "success": True,
      "error_msg": "",
      "available_nodes_count": "1",
      "available_nodes":
        "[
          {camera_namespace: robot1, camera_name: c_333622320169}
        ]"
    }
  ```
## Reset the Device
* mqtt request message example
  ```
  {
    "camera_namespace": "robot1",
    "camera_name": "c_333622320169",
  }
  ```
* request topic
  ```
  send_hw_reset_request
  ```
* response topic
  ```
  send_hw_reset_response
  ```
* mqtt response message example: 
  ```
  {
    "camera_namespace": "camera",
    "camera_name": "camera",
    "success": true, 
    "error_msg": ""
  }
  ```
## Get Device Info
* mqtt request message example
  ```
  {
    "camera_namespace": "robot1",
    "camera_name": "c_333622320169",
  }
  ```
* request topic
  ```
  get_device_info_request
  ```
* response topic
  ```
  get_device_info_response
  ```
* mqtt response message example: 
  ```
  {
    "camera_namespace": "camera",
    "camera_name": "camera",
    "device_name": "intel_realsense_d585s",
    "serial_number": "333622320169",
    "firmware_version": "8.17.15566.148",
    "usb_type_descriptor": "3.2",
    "firmware_update_id": "333622320169",
    "sensors": "depth_module,rgb_camera,safety_camera,depth_mapping_camera,motion_module",
    "physical_port": "/sys/devices/pci0000:00/0000:00:14.0/usb2/2-1/2-1:1.0/video4linux/video0",
  }
  ```

## Get Transformation
* mqtt request message example
  ```
  {
    "source": "c_353322320702_link",
    "destination": "c_353322320702_color_frame",
  }
  ```
* request topic
  ```
  get_transformation_request
  ```
* response topic
  ```
  get_transformation_response
  ```
* mqtt response message example: 
  ```
  {
    "rotation": {"x": -0.0022762807482196233, "y": -0.0011598517160371544, "z": -0.0011766824618274568, "w": 0.9999960443463445},
    "translation": {"x": -4.12423032685183e-05, "y": -0.04789295792579651, "z": 0.0005400447407737374},
    "success": true,
    "error_msg": ""
  }
  ```

## Send HWM Command
* mqtt request message example (read of safety interface config table)
  ```
  {
    "camera_namespace": "robot1",
    "camera_name": "c_353322320702",
    "opcode": 167,     # opcode of GET_HKR_CONFIG_TABLE 0xA7 
    "param1": 1,       # read from flash (1)
    "param2": 49372,   # table id (safety interface config) 0xC0DC
    "param3": 1,       # 0 dynamic, 1 gold
    "param4": 0,       # unused (ChunkId)
    "data": []
  }
  ```
* request topic
  ```
  send_hwm_command_request
  ```
* response topic
  ```
  send_hwm_command_response
  ```
* mqtt response message example: 
  ```
  {
    "camera_namespace": "robot1",
    "camera_name": "c_353322320702",
    "success": true,
    "result": [167, 0, 0, 0, 0, 5, 220, 192, 148, 0, 0, 0, 0, 0, 0, 0, 250, 239, 161, 49, 0, 1, 1, 3, 1, 2, 0, 12, 0, 13, 0, 14, 0, 9, 0, 8, 0, 16, 0, 17, 0, 19, 0, 18, 0, 11, 1, 20, 0, 10, 0, 15, 0, 0, 150, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 128, 63, 0, 0, 128, 191, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 128, 191, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 113, 61, 138, 62, 20, 0, 12, 6, 4, 100, 0, 20, 100, 0, 20, 10, 15, 10, 95, 23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    "error_msg": ""
  }
  ```

## Get Parameter
***See [available parameters and their types](#supported-parameters-for-setget)***
* mqtt request message example
  ```
  {
    "camera_namespace": "robot1",
    "camera_name": "c_333622320169",
    "parameter_name": "rgb_camera.exposure"
  }
  ```
* request topic
  ```
  get_param_request
  ```
* response topic
  ```
  get_param_response
  ```
* mqtt response message example: 
  ```
  {
    "camera_namespace": "robot1",
    "camera_name": "c_333622320169",
    "parameter_name": "rgb_camera.exposure",
    "success": True,
    "error_msg": "",
    "parameter_type": "integer",
    "parameter_value": "6012"
  }
  ```

## Set Parameter
***See [available parameters and their types](#supported-parameters-for-setget)***
* mqtt request message example:
  ```
  {
    "camera_namespace": "robot1",
    "camera_name": "c_333622320169",
    "parameter_name": "rgb_camera.exposure"
    "parameter_value": "6012",
    "parameter_type": "int"
  }
  ```
* request topic
  ```
  set_param_request
  ```
* response topic
  ```
  set_param_response
  ```
* mqtt response message example: 
  ```
  {
    "camera_namespace": "robot1",
    "camera_name": "c_333622320169",
    "success": True,
    "error_msg": ""
  }
  ```
  
## Get Frame
***See [Supported Streams](#supported-streams)***
* mqtt request message example:
  ```
  {
    "camera_namespace": "robot1",
    "camera_name": "c_333622320169",
    "stream_name": "color"
  }
  ```
* request topic:
  ```
  get_frame_request
  ```
* response topic:
  ```
  get_frame_response
  ```
* mqtt response message example: 
  ```
  {
    "camera_namespace": "robot1",
    "camera_name": "c_333622320169",
    "stream_name": "color"
    "success": True,
    "error_msg": "",
    "frame": "[0, 118, 124, 0, 0, ...., 255]" # array of bytes
  }
  ```

## Get Safety Preset
* mqtt request message example:
  ```
  {
    "camera_namespace": "robot1",
    "camera_name": "c_333622320169",
    "index": "1"
  }
  ```
* request topic:
  ```
  get_safety_preset_request
  ```
* response topic:
  ```
  get_safety_preset_response
  ```
* mqtt response message example: 
  ```
  {
    "camera_namespace": "robot1",
    "camera_name": "c_333622320169",
    "success": True,
    "error_msg": "",
    "safety_preset": "{safety preset as json}"
  }
  ```
## Set Safety Preset
* mqtt request message example:
  ```
  {
    "camera_namespace": "robot1",
    "camera_name": "c_333622320169",
    "safety_preset": "{safety preset as json}"
    "index": "1"
  }
  ```
* request topic:
  ```
  set_safety_preset_request
  ```
 * response topic:
  ```
  set_safety_preset_response
  ```
* mqtt response message example: 
  ```
  {
    "camera_namespace": "robot1",
    "camera_name": "c_333622320169",
    "success": True,
    "error_msg": "",
  }
  ```

## Get Safety Interface Config
* mqtt request message example:
  ```
  {
    "camera_namespace": "robot1",
    "camera_name": "c_333622320169",
  }
  ```
* request topic:
  ```
  get_safety_interface_config_request
  ```
* response topic:
  ```
  get_safety_interface_config_response
  ```
* mqtt response message example: 
  ```
  {
    "camera_namespace": "robot1",
    "camera_name": "c_333622320169",
    "safety_inteface_config": "{safety interface config as JSON}",
    "success": True,
    "error_msg": "",
  }
  ```

## Set Safety Interface Config
* mqtt request message example:
  ```
  {
    "camera_namespace": "robot1",
    "camera_name": "c_333622320169",
    "safety_inteface_config": "{safety interface config as JSON}",
  }
  ```
* request topic:
  ```
  set_safety_interface_config_request
  ```
* response topic:
  ```
  set_safety_interface_config_response
  ```
* mqtt response message example: 
  ```
  {
    "camera_namespace": "robot1",
    "camera_name": "c_333622320169",
    "success": True,
    "error_msg": "",
  }
  ```

## Get Calib Config
* mqtt request message example:
  ```
  {
    "camera_namespace": "robot1",
    "camera_name": "c_333622320169",
  }
  ```
* request topic:
  ```
  get_calib_config_request
  ```
* response topic:
  ```
  get_calib_config_response
  ```
* mqtt response message example: 
  ```
  {
    "camera_namespace": "robot1",
    "camera_name": "c_333622320169",
    "calib_config": "{calib config as JSON}",
    "success": True,
    "error_msg": "",
  }
  ```

## Set Calib Config

* mqtt request message example:
  ```
  {
    "camera_namespace": "robot1",
    "camera_name": "c_333622320169",
    "calib_config": "{calib config as JSON}"
  }
  ```
* request topic:
  ```
  set_calib_config_request
  ```
* response topic:
  ```
  set_calib_config_response
  ```
* mqtt response message example: 
  ```
  {
    "camera_namespace": "robot1",
    "camera_name": "c_333622320169",
    "success": True,
    "error_msg": "",
  }
  ```

## Get Application Config
* mqtt request message example:
  ```
  {
    "camera_namespace": "robot1",
    "camera_name": "c_333622320169",
  }
  ```
* request topic:
  ```
  get_application_config_request
  ```
* response topic:
  ```
  get_application_config_response
  ```
* mqtt response message example: 
  ```
  {
    "camera_namespace": "robot1",
    "camera_name": "c_333622320169",
    "application_config": "{application config as JSON}",
    "success": True,
    "error_msg": "",
  }
  ```

## Set Application Config

* mqtt request message example:
  ```
  {
    "camera_namespace": "robot1",
    "camera_name": "c_333622320169",
    "application_config": "{application config as JSON}"
  }
  ```
* request topic:
  ```
  set_application_config_request
  ```
* response topic:
  ```
  set_application_config_response
  ```
* mqtt response message example: 
  ```
  {
    "camera_namespace": "robot1",
    "camera_name": "c_333622320169",
    "success": True,
    "error_msg": "",
  }
  ```

## Triggered Calibration

* Before calling triggered calibration, user should set the following parameters:
  * `safety_camera.safety_mode: 2` # switch to service mode
  * `depth_module.visual_preset: 1` # switch to visual preset #1 in depth module
  * `depth_module.emitter_enabled: true` # enable emitter in depth module
  * `depth_module.enable_auto_exposure: true` # enable AE in depth moudle
  * `enable_depth: false` # turn off depth stream
  * `enable_infra1: false` # turn off infra1 stream
  * `enable_infra2: false` # turn off infra2 stream
  * `enable_safety: false` # turn off safety stream
  * `enable_labeled_point_cloud: false` # turn off labeled pointcloud stream
  * `enable_occupancy: false` # turn off occupancy stream

* mqtt request message example:
  ```
  {
    "camera_namespace": "robot1",
    "camera_name": "c_333622320169",
  }
  ```
* request topic:
  ```
  triggered_calibration_request
  ```
* response topic:
  ```
  triggered_calibration_response
  ```
* mqtt response message example: (user will get messages on the `triggered_calibration_response` on every progress update)
  ```
  {
    "camera_namespace": "robot1",
    "camera_name": "c_333622320169",
    "success": True,
    "error_msg": "",
    "calibration": {calibration table as json}
    "health": 3.0
    "progress": 100.0   # [%0...%100]
  }
  ```

# Supported Parameters For Set/Get

```
  accel_fps (type: integer)
  accel_info_qos (type: string)
  accel_qos (type: string)

  align_depth.enable (type: boolean)
  align_depth.frames_queue_size (type: integer)

  angular_velocity_cov (type: double)
  base_frame_id (type: string)
  camera_name (type: string)
  clip_distance (type: double)
  color_info_qos (type: string)
  color_qos (type: string)

  colorizer.color_scheme (type: integer)
  colorizer.enable (type: boolean)
  colorizer.frames_queue_size (type: integer)
  colorizer.histogram_equalization_enabled (type: boolean)
  colorizer.max_distance (type: double)
  colorizer.min_distance (type: double)
  colorizer.stream_filter (type: integer)
  colorizer.stream_format_filter (type: integer)
  colorizer.stream_index_filter (type: integer)
  colorizer.visual_preset (type: integer)

  decimation_filter.enable (type: boolean)
  decimation_filter.filter_magnitude (type: integer)
  decimation_filter.frames_queue_size (type: integer)
  decimation_filter.stream_filter (type: integer)
  decimation_filter.stream_format_filter (type: integer)
  decimation_filter.stream_index_filter (type: integer)

  depth_info_qos (type: string)

  depth_mapping_camera.frames_queue_size (type: integer)
  depth_mapping_camera.global_time_enabled (type: boolean)
  depth_mapping_camera.labeled_point_cloud_format (type: string)
  depth_mapping_camera.labeled_point_cloud_profile (type: string)
  depth_mapping_camera.occupancy_format (type: string)
  depth_mapping_camera.occupancy_profile (type: string)

  depth_module.auto_exposure_roi.bottom (type: integer)
  depth_module.auto_exposure_roi.left (type: integer)
  depth_module.auto_exposure_roi.right (type: integer)
  depth_module.auto_exposure_roi.top (type: integer)
  depth_module.depth_format (type: string)
  depth_module.depth_profile (type: string)
  depth_module.emitter_always_on (type: boolean)
  depth_module.emitter_enabled (type: boolean)
  depth_module.enable_auto_exposure (type: boolean)
  depth_module.error_polling_enabled (type: boolean)
  depth_module.exposure (type: integer)
  depth_module.frames_queue_size (type: integer)
  depth_module.gain (type: integer)
  depth_module.global_time_enabled (type: boolean)
  depth_module.infra1_format (type: string)
  depth_module.infra2_format (type: string)
  depth_module.infra_profile (type: string)
  depth_module.inter_cam_sync_mode (type: integer)
  depth_module.laser_power (type: double)
  depth_module.visual_preset (type: integer)
  depth_qos (type: string)

  device_type (type: string)
  diagnostics_period (type: double)
  disparity_filter.enable (type: boolean)
  disparity_to_depth.enable (type: boolean)

  enable_accel (type: boolean)
  enable_color (type: boolean)
  enable_depth (type: boolean)
  enable_gyro (type: boolean)
  enable_infra1 (type: boolean)
  enable_infra2 (type: boolean)
  enable_labeled_point_cloud (type: boolean)
  enable_occupancy (type: boolean)
  enable_rgbd (type: boolean)
  enable_safety (type: boolean)
  enable_sync (type: boolean)

  filter_by_sequence_id.enable (type: boolean)
  filter_by_sequence_id.frames_queue_size (type: integer)
  filter_by_sequence_id.sequence_id (type: integer)

  gyro_fps (type: integer)
  gyro_info_qos (type: string)
  gyro_qos (type: string)

  hdr_merge.enable (type: boolean)
  hdr_merge.frames_queue_size (type: integer)

  hold_back_imu_for_frames (type: boolean)
  hole_filling_filter.enable (type: boolean)
  hole_filling_filter.frames_queue_size (type: integer)
  hole_filling_filter.holes_fill (type: integer)
  hole_filling_filter.stream_filter (type: integer)
  hole_filling_filter.stream_format_filter (type: integer)
  hole_filling_filter.stream_index_filter (type: integer)

  infra1_info_qos (type: string)
  infra1_qos (type: string)
  infra2_info_qos (type: string)
  infra2_qos (type: string)

  initial_reset (type: boolean)
  json_file_path (type: string)

  labeled_point_cloud_info_qos (type: string)
  labeled_point_cloud_qos (type: string)

  linear_accel_cov (type: double)

  motion_module.enable_motion_correction (type: boolean)
  motion_module.frames_queue_size (type: integer)
  motion_module.global_time_enabled (type: boolean)

  occupancy_info_qos (type: string)
  occupancy_qos (type: string)

  pointcloud.allow_no_texture_points (type: boolean)
  pointcloud.enable (type: boolean)
  pointcloud.filter_magnitude (type: integer)
  pointcloud.frames_queue_size (type: integer)
  pointcloud.ordered_pc (type: boolean)
  pointcloud.pointcloud_qos (type: string)
  pointcloud.stream_filter (type: integer)
  pointcloud.stream_format_filter (type: integer)
  pointcloud.stream_index_filter (type: integer)

  publish_tf (type: boolean)
  reconnect_timeout (type: double)

  rgb_camera.auto_exposure_priority (type: boolean)
  rgb_camera.auto_exposure_roi.bottom (type: integer)
  rgb_camera.auto_exposure_roi.left (type: integer)
  rgb_camera.auto_exposure_roi.right (type: integer)
  rgb_camera.auto_exposure_roi.top (type: integer)
  rgb_camera.brightness (type: integer)
  rgb_camera.color_format (type: string)
  rgb_camera.color_profile (type: string)
  rgb_camera.contrast (type: integer)
  rgb_camera.enable_auto_exposure (type: boolean)
  rgb_camera.enable_auto_white_balance (type: boolean)
  rgb_camera.exposure (type: integer)
  rgb_camera.frames_queue_size (type: integer)
  rgb_camera.gain (type: integer)
  rgb_camera.gamma (type: integer)
  rgb_camera.global_time_enabled (type: boolean)
  rgb_camera.hue (type: integer)
  rgb_camera.power_line_frequency (type: integer)
  rgb_camera.saturation (type: integer)
  rgb_camera.sharpness (type: integer)
  rgb_camera.white_balance (type: double)

  rosbag_filename (type: string)

  safety_camera.frames_queue_size (type: integer)
  safety_camera.global_time_enabled (type: boolean)
  safety_camera.safety_format (type: string)
  safety_camera.safety_mode (type: integer)
  safety_camera.safety_preset_active_index (type: integer)
  safety_camera.safety_profile (type: string)
  safety_info_qos (type: string)
  safety_qos (type: string)

  serial_no (type: string)

  spatial_filter.enable (type: boolean)
  spatial_filter.filter_magnitude (type: integer)
  spatial_filter.filter_smooth_alpha (type: double)
  spatial_filter.filter_smooth_delta (type: integer)
  spatial_filter.frames_queue_size (type: integer)
  spatial_filter.holes_fill (type: integer)
  spatial_filter.stream_filter (type: integer)
  spatial_filter.stream_format_filter (type: integer)
  spatial_filter.stream_index_filter (type: integer)

  temporal_filter.enable (type: boolean)
  temporal_filter.filter_smooth_alpha (type: double)
  temporal_filter.filter_smooth_delta (type: integer)
  temporal_filter.frames_queue_size (type: integer)
  temporal_filter.holes_fill (type: integer)
  temporal_filter.stream_filter (type: integer)
  temporal_filter.stream_format_filter (type: integer)
  temporal_filter.stream_index_filter (type: integer)

  tf_publish_rate (type: double)
  unite_imu_method (type: integer)
  usb_port_id (type: string)
  use_sim_time (type: boolean)
  wait_for_device_timeout (type: double)

```

# Supported Streams
- color
- depth
- infra1 (Left IR)
- infra2 (Right IR)

# Usage Example
[Minimal Python MQTT Client Example](examples/minimal_mqtt_client.py)


[jazzy-badge]: https://img.shields.io/badge/-JAZZY-orange?style=flat-square&logo=ros
[jazzy]: https://docs.ros.org/en/jazzy/index.html
[humble-badge]: https://img.shields.io/badge/-HUMBLE-orange?style=flat-square&logo=ros
[humble]: https://docs.ros.org/en/humble/index.html
[iron-badge]: https://img.shields.io/badge/-IRON-orange?style=flat-square&logo=ros
[iron]: https://docs.ros.org/en/iron/index.html
[ubuntu24-badge]: https://img.shields.io/badge/-UBUNTU%2024%2E04-blue?style=flat-square&logo=ubuntu&logoColor=white
[ubuntu24]: https://releases.ubuntu.com/noble/
[ubuntu22-badge]: https://img.shields.io/badge/-UBUNTU%2022%2E04-blue?style=flat-square&logo=ubuntu&logoColor=white
[ubuntu22]: https://releases.ubuntu.com/jammy/
