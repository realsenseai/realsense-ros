^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package realsense2_camera
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

4.57.3 (2025-09-15)
-------------------
* PR `#3430 <https://github.com/realsenseai/realsense-ros/issues/3430>`_ from Gilaadb: Create a singleton wrapper to rs2::context
* PR `#3429 <https://github.com/realsenseai/realsense-ros/issues/3429>`_ from remibettan: intel removed, realsense added
* PR `#3421 <https://github.com/realsenseai/realsense-ros/issues/3421>`_ from ynyBonfennil: Fix argument names (`_usb_port_id` and `_device_type`)
* PR `#3417 <https://github.com/realsenseai/realsense-ros/issues/3417>`_ from remibettan: Merging ros2 hkr to ros2 dev final
* PR `#3410 <https://github.com/realsenseai/realsense-ros/issues/3410>`_ from Nir-Az: Update copyrights
* PR `#3356 <https://github.com/realsenseai/realsense-ros/issues/3356>`_ from ashrafk93: Ashraf/glsl pointcloud
* PR `#3374 <https://github.com/realsenseai/realsense-ros/issues/3374>`_ from remibettan: kilted added to wrapper
* PR `#3392 <https://github.com/realsenseai/realsense-ros/issues/3392>`_ from remibettan: adding D436
* PR `#3385 <https://github.com/realsenseai/realsense-ros/issues/3385>`_ from Gilaadb: Fix RGBD camera_info and frame_id
* PR `#3371 <https://github.com/realsenseai/realsense-ros/issues/3371>`_ from Gilaadb: replace posix argument with suffix which is what it was meant to be
* PR `#3347 <https://github.com/realsenseai/realsense-ros/issues/3347>`_ from remibettan: few logs and exceptions catches added
* PR `#41 <https://github.com/realsenseai/realsense-ros/issues/41>`_ from remibettan: Merge dev to hkr 2025 05 04
* PR `#3352 <https://github.com/realsenseai/realsense-ros/issues/3352>`_ from ashrafk93: Fix unit test of Support TF Prefixing
* PR `#3332 <https://github.com/realsenseai/realsense-ros/issues/3332>`_ from pondersome: Support TF Prefixing
* PR `#3340 <https://github.com/realsenseai/realsense-ros/issues/3340>`_ from Gilaadb: Support rgbd type and fix bug for FPS lower than 1
* PR `#3325 <https://github.com/realsenseai/realsense-ros/issues/3325>`_ from ashrafk93: use ykush to switch ports
* PR `#3319 <https://github.com/realsenseai/realsense-ros/issues/3319>`_ from ashrafk93: Add LifeCycle Node support at compile time
* PR `#3303 <https://github.com/realsenseai/realsense-ros/issues/3303>`_ from noacoohen: Enable rotation filter for color and depth sensors
* PR `#3293 <https://github.com/realsenseai/realsense-ros/issues/3293>`_ from remibettan: align_depth_to_infra2 enabled, pointcloud and align_depth filters to own files
* PR `#3284 <https://github.com/realsenseai/realsense-ros/issues/3284>`_ from noacoohen: Add color format to depth module in the launch file
* PR  `#3274 <https://github.com/realsenseai/realsense-ros/issues/3274>`_ from noacoohen: Enable rotation filter ROS2
* PR `#3276 <https://github.com/realsenseai/realsense-ros/issues/3276>`_ from remibettan: removing dead code in RosSensor class
* PR `#3214 <https://github.com/realsenseai/realsense-ros/issues/3214>`_ from acornaglia: Add ROS bag loop option
* PR `#3239 <https://github.com/realsenseai/realsense-ros/issues/3239>`_ from SamerKhshiboun: Update CMakeLists.txt - remove find_package(fastrtps REQUIRED)
* PR `#3225 <https://github.com/realsenseai/realsense-ros/issues/3225>`_ from SamerKhshiboun: Use new APIs for motion, accel and gryo streams
* PR `#3222 <https://github.com/realsenseai/realsense-ros/issues/3222>`_ from SamerKhshiboun: Support D555 and its motion profiles
* PR `#3221 <https://github.com/realsenseai/realsense-ros/issues/3221>`_ from patrickwasp: fix config typo
* PR `#33 <https://github.com/realsenseai/realsense-ros/issues/33>`_ from PrasRsRos: add reset service tests
* PR `#35 <https://github.com/realsenseai/realsense-ros/issues/35>`_ from PrasRsRos: align private to public 30.9.2024
* PR `#32 <https://github.com/realsenseai/realsense-ros/issues/32>`_ from SamerKhshiboun: Support HWM command as ROS2 service and in the ROS-MQTT bridge node
* PR `#3216 <https://github.com/realsenseai/realsense-ros/issues/3216>`_ from PrasRsRos: hw_reset implementation
* PR `#30 <https://github.com/realsenseai/realsense-ros/issues/30>`_ from SamerKhshiboun: Use new apis of SIC and SP that works directly with JSON inputs/outputs
* PR `#28 <https://github.com/realsenseai/realsense-ros/issues/28>`_ from SamerKhshiboun: Fix MQTT Demo and update values for and update TC consecutives failures threshold
* PR `#27 <https://github.com/realsenseai/realsense-ros/issues/27>`_ from SamerKhshiboun: Add new flash 0.93 fields to app config
* PR `#3200 <https://github.com/realsenseai/realsense-ros/issues/3200>`_ from kadiredd: retry thrice finding devices with Ykush reset
* PR `#23 <https://github.com/realsenseai/realsense-ros/issues/23>`_ from PrasRsRos: Ros tc implementation
* PR `#20 <https://github.com/realsenseai/realsense-ros/issues/20>`_ from SamerKhshiboun: Revert switching to service mode in order to set depth params
* PR `#21 <https://github.com/realsenseai/realsense-ros/issues/21>`_ from SamerKhshiboun: Fix l4 threshold to l2 threshold
* PR `#19 <https://github.com/realsenseai/realsense-ros/issues/19>`_ from PrasRsRos: DeviceInfo and TC Mqtt tests
* PR `#3178 <https://github.com/realsenseai/realsense-ros/issues/3178>`_ from kadiredd: disabling FPS & TF tests for ROS-CI
* PR `#16 <https://github.com/realsenseai/realsense-ros/issues/16>`_ from PrasRsRos: RS ROS Mqtt bridge unit tests
* PR `#3166 <https://github.com/realsenseai/realsense-ros/issues/3166>`_ from SamerKhshiboun: Update Calibration Config API
* PR `#13 <https://github.com/realsenseai/realsense-ros/issues/13>`_ from SamerKhshiboun: Sٍupport set/get application config as ROS service and in ROS-MQTT bridge
* PR `#3159 <https://github.com/realsenseai/realsense-ros/issues/3159>`_ from noacoohen: Add D421 PID
* PR `#10 <https://github.com/realsenseai/realsense-ros/issues/10>`_ from SamerKhshiboun: Add ROS MQTT Bridge (Python) Node Into realsense-ros-private
* PR `#3153 <https://github.com/realsenseai/realsense-ros/issues/3153>`_ from SamerKhshiboun: TC | Fix feedback and update readme
* fix feedback and update readme for TC
* PR `#3138 <https://github.com/realsenseai/realsense-ros/issues/3138>`_ from SamerKhshiboun: Support Triggered Calibration as ROS2 Action
* implement Triggered Calibration action
* PR `#3135 <https://github.com/realsenseai/realsense-ros/issues/3135>`_ from kadiredd: Casefolding device name instead of strict case sensitive comparison
* Casefolding device name instead os strict case sensitive comparison
* PR `#3133 <https://github.com/realsenseai/realsense-ros/issues/3133>`_ from SamerKhshiboun: update librealsense2 version to 2.56.0
* update librealsense2 version to 2.56.0
  since it includes new API that need for ros2-development
* PR `#3124 <https://github.com/realsenseai/realsense-ros/issues/3124>`_ from kadiredd: Support testing ROS2 service call device_info
* PR `#3125 <https://github.com/realsenseai/realsense-ros/issues/3125>`_ from SamerKhshiboun: Support calibration config read/write services
* PR `#5 <https://github.com/realsenseai/realsense-ros/issues/5>`_ from SamerKhshiboun: Update README and fix SIC fields in the examples
* PR `#3114 <https://github.com/realsenseai/realsense-ros/issues/3114>`_ from Arun-Prasad-V: Ubuntu 24.04 support for Rolling and Jazzy distros
* PR `#3 <https://github.com/realsenseai/realsense-ros/issues/3>`_ from SamerKhshiboun: Support sic read write services
* PR `#2 <https://github.com/realsenseai/realsense-ros/issues/2>`_ from SamerKhshiboun: Support Safety Preset Read/Write Services
* PR `#3102 <https://github.com/realsenseai/realsense-ros/issues/3102>`_ from fortizcuesta: Allow hw synchronization of several realsense using a synchonization cable
* PR `#3096 <https://github.com/realsenseai/realsense-ros/issues/3096>`_ from anisotropicity: Update rs_launch.py to add depth_module.color_profile
* PR `#1 <https://github.com/realsenseai/realsense-ros/issues/1>`_ from Arun-Prasad-V: Set Safety mode to SERVICE when loading preset
* PR `#3061 <https://github.com/realsenseai/realsense-ros/issues/3061>`_ from Arun-Prasad-V: Updated rs_launch.py for LPC and Occupancy stream profile names
* rs-launch.py update
* PR `#3038 <https://github.com/realsenseai/realsense-ros/issues/3038>`_ from Arun-Prasad-V: Set Safety mode to service before updating Depth controls during launch
* PR `#3032 <https://github.com/realsenseai/realsense-ros/issues/3032>`_ from SamerKhshiboun: Support occupancy grid cells
* PR `#2971 <https://github.com/realsenseai/realsense-ros/issues/2971>`_ from SamerKhshiboun: Occupancy Height Fix
* PR `#2952 <https://github.com/realsenseai/realsense-ros/issues/2952>`_ from Nir-Az: Support 2 res for LPC
* PR `#2827 <https://github.com/realsenseai/realsense-ros/issues/2827>`_ from SamerKhshiboun: Fix empty frames of rgbd
* PR `#2821 <https://github.com/realsenseai/realsense-ros/issues/2821>`_ from SamerKhshiboun: fix missing else due to merge from ros2-development
* PR `#2813 <https://github.com/realsenseai/realsense-ros/issues/2813>`_ from SamerKhshiboun: Fix URDF and LPCL for SC
* PR `#2815 <https://github.com/realsenseai/realsense-ros/issues/2815>`_ from SamerKhshiboun: fix labeled point cloud publisher reset condition
* PR `#2802 <https://github.com/realsenseai/realsense-ros/issues/2802>`_ from SamerKhshiboun: add new RGBD topic
* PR `#2800 <https://github.com/realsenseai/realsense-ros/issues/2800>`_ from SamerKhshiboun: Fix overriding frames on same topics/CV-images due to a bug in PR2759
* PR `#2776 <https://github.com/realsenseai/realsense-ros/issues/2776>`_ from SamerKhshiboun: Fix LPCL in SC
* PR `#2757 <https://github.com/realsenseai/realsense-ros/issues/2757>`_ from SamerKhshiboun: Support Depth Mapping Streams
* PR `#2659 <https://github.com/realsenseai/realsense-ros/issues/2659>`_ from SamerKhshiboun: Warn instead of error for undefined sensor callbacks
* PR `#2590 <https://github.com/realsenseai/realsense-ros/issues/2590>`_ from SamerKhshiboun: Add SC to ROS
* Contributors: Aman Chulawala, Arun-Prasad-V, Ashraf Kattoura, AviaAv, Cornaglia, Alessandro, Gilad Bretter, Madhukar Reddy Kadireddy, Nir Azkiel, Ortiz Cuesta, Fernando, Patrick Wspanialy, PrasRsRos, Remi Bettan, Samer Khshiboun, acornaglia, administrator, anisotropicity, louislelay, noacoohen, pondersome, ynyBonfennil

4.55.1 (2024-05-28)
-------------------
* PR `#3106 <https://github.com/realsenseai/realsense-ros/issues/3106>`_ from SamerKhshiboun: Remove unused parameter _is_profile_exist
* PR `#3098 <https://github.com/realsenseai/realsense-ros/issues/3098>`_ from kadiredd: ROS live cam test fixes
* PR `#3094 <https://github.com/realsenseai/realsense-ros/issues/3094>`_ from kadiredd: ROSCI infra for live camera testing
* PR `#3066 <https://github.com/realsenseai/realsense-ros/issues/3066>`_ from SamerKhshiboun: Revert Foxy Build Support (From Source)
* PR `#3052 <https://github.com/realsenseai/realsense-ros/issues/3052>`_ from Arun-Prasad-V: Support for selecting profile for each stream_type
* PR `#3056 <https://github.com/realsenseai/realsense-ros/issues/3056>`_ from SamerKhshiboun: Add documentation for RealSense ROS2 Wrapper Windows installation
* PR `#3049 <https://github.com/realsenseai/realsense-ros/issues/3049>`_ from Arun-Prasad-V: Applying Colorizer filter to Aligned-Depth image
* PR `#3053 <https://github.com/realsenseai/realsense-ros/issues/3053>`_ from Nir-Az: Fix Coverity issues + remove empty warning log
* PR `#3007 <https://github.com/realsenseai/realsense-ros/issues/3007>`_ from Arun-Prasad-V: Skip updating Exp 1,2 & Gain 1,2 when HDR is disabled
* PR `#3042 <https://github.com/realsenseai/realsense-ros/issues/3042>`_ from kadiredd: Assert Fail if camera not found
* PR `#3008 <https://github.com/realsenseai/realsense-ros/issues/3008>`_ from Arun-Prasad-V: Renamed GL GPU enable param
* PR `#2989 <https://github.com/realsenseai/realsense-ros/issues/2989>`_ from Arun-Prasad-V: Dynamically switching b/w CPU & GPU processing
* PR `#3001 <https://github.com/realsenseai/realsense-ros/issues/3001>`_ from deep0294: Update ReadMe to run ROS2 Unit Test
* PR `#2998 <https://github.com/realsenseai/realsense-ros/issues/2998>`_ from SamerKhshiboun: fix calibration intrinsic fail
* PR `#2987 <https://github.com/realsenseai/realsense-ros/issues/2987>`_ from SamerKhshiboun: Remove D465 SKU
* PR `#2984 <https://github.com/realsenseai/realsense-ros/issues/2984>`_ from deep0294: Fix All Profiles Test
* PR `#2956 <https://github.com/realsenseai/realsense-ros/issues/2956>`_ from Arun-Prasad-V: Extending LibRS's GL support to RS ROS2
* PR `#2953 <https://github.com/realsenseai/realsense-ros/issues/2953>`_ from Arun-Prasad-V: Added urdf & mesh files for D405 model
* PR `#2940 <https://github.com/realsenseai/realsense-ros/issues/2940>`_ from Arun-Prasad-V: Fixing the data_type of ROS Params exposure & gain
* PR `#2948 <https://github.com/realsenseai/realsense-ros/issues/2948>`_ from Arun-Prasad-V: Disabling HDR during INIT
* PR `#2934 <https://github.com/realsenseai/realsense-ros/issues/2934>`_ from Arun-Prasad-V: Disabling hdr while updating exposure & gain values
* PR `#2946 <https://github.com/realsenseai/realsense-ros/issues/2946>`_ from gwen2018: fix ros random crash with error hw monitor command for asic temperature failed
* PR `#2865 <https://github.com/realsenseai/realsense-ros/issues/2865>`_ from PrasRsRos: add live camera tests
* PR `#2891 <https://github.com/realsenseai/realsense-ros/issues/2891>`_ from Arun-Prasad-V: revert PR2872
* PR `#2853 <https://github.com/realsenseai/realsense-ros/issues/2853>`_ from Arun-Prasad-V: Frame latency for the '/topic' provided by user
* PR `#2872 <https://github.com/realsenseai/realsense-ros/issues/2872>`_ from Arun-Prasad-V: Updating _camera_name with RS node's name
* PR `#2878 <https://github.com/realsenseai/realsense-ros/issues/2878>`_ from Arun-Prasad-V: Updated ros2 examples and readme
* PR `#2841 <https://github.com/realsenseai/realsense-ros/issues/2841>`_ from SamerKhshiboun: Remove Dashing, Eloquent, Foxy, L500 and SR300 support
* PR `#2868 <https://github.com/realsenseai/realsense-ros/issues/2868>`_ from Arun-Prasad-V: Fix Pointcloud topic frame_id
* PR `#2849 <https://github.com/realsenseai/realsense-ros/issues/2849>`_ from Arun-Prasad-V: Create /imu topic only when motion streams enabled
* PR `#2847 <https://github.com/realsenseai/realsense-ros/issues/2847>`_ from Arun-Prasad-V: Updated rs_launch param names
* PR `#2839 <https://github.com/realsenseai/realsense-ros/issues/2839>`_ from Arun-Prasad: Added ros2 examples
* PR `#2861 <https://github.com/realsenseai/realsense-ros/issues/2861>`_ from SamerKhshiboun: fix readme and nodefactory for ros2 run
* PR `#2859 <https://github.com/realsenseai/realsense-ros/issues/2859>`_ from PrasRsRos: Fix tests (topic now has camera name)
* PR `#2857 <https://github.com/realsenseai/realsense-ros/issues/2857>`_ from lge-ros2: Apply camera name in topics
* PR `#2840 <https://github.com/realsenseai/realsense-ros/issues/2840>`_ from SamerKhshiboun: Support Depth, IR and Color formats in ROS2
* PR `#2764 <https://github.com/realsenseai/realsense-ros/issues/2764>`_ from lge-ros2 : support modifiable camera namespace
* PR `#2830 <https://github.com/realsenseai/realsense-ros/issues/2830>`_ from SamerKhshiboun: Add RGBD + reduce changes between hkr and development
* PR `#2811 <https://github.com/realsenseai/realsense-ros/issues/2811>`_ from Arun-Prasad-V: Exposing stream formats params to user
* PR `#2825 <https://github.com/realsenseai/realsense-ros/issues/2825>`_ from SamerKhshiboun: Fix align_depth + add test
* PR `#2822 <https://github.com/realsenseai/realsense-ros/issues/2822>`_ from Arun-Prasad-V: Updated rs_launch configurations
* PR `#2726 <https://github.com/realsenseai/realsense-ros/issues/2726>`_ from PrasRsRos: Integration test template
* PR `#2742 <https://github.com/realsenseai/realsense-ros/issues/2742>`_ from danielhonies:Update rs_launch.py
* PR `#2806 <https://github.com/realsenseai/realsense-ros/issues/2806>`_ from Arun-Prasad-V: Enabling RGB8 Infrared stream
* PR `#2799 <https://github.com/realsenseai/realsense-ros/issues/2799>`_ from SamerKhshiboun: Fix overriding frames on same topics/CV-images due to a bug in PR2759
* PR `#2759 <https://github.com/realsenseai/realsense-ros/issues/2759>`_ from SamerKhshiboun: Cleanups and name fixes
* Contributors: (=YG=) Hyunseok Yang, Arun Prasad, Arun-Prasad-V, Daniel Honies, Hyunseok, Madhukar Reddy Kadireddy, Nir, Nir Azkiel, PrasRsRos, Samer Khshiboun, SamerKhshiboun, deep0294, gwen2018, nairps

4.54.1 (2023-06-27)
-------------------
* Applying AlignDepth filter after Pointcloud
* Publish /aligned_depth_to_color topic only when color frame present
* Support Iron distro
* Protect empty string dereference
* Fix: /tf and /static_tf topics' inconsistencies
* Revamped the TF related code
* Fixing TF frame links b/w multi camera nodes when using custom names
* Updated TF descriptions in launch py and readme
* Fixing /tf topic has only TFs of last started sensor
* add D430i support
* Fix Swapped TFs Axes
* replace stereo module with depth module
* use rs2_to_ros to replace stereo module with depth moudle
* calculate extriniscs twice in two opposite ways to save inverting rotation matrix
* fix matrix rotation
* Merge branch 'ros2-development' into readme_fix
* invert translation
* Added 'publish_tf' param in rs launch files
* Indentation corrections
* Fix: Don't publish /tf when publish_tf is false
* use playback device for rosbags
* Avoid configuring dynamic_tf_broadcaster within tf_publish_rate param's callback
* Fix lower FPS in D405, D455
* update rs_launch.py to support enable_auto_exposure and manual exposure
* fix timestamp calculation metadata header to be aligned with metadata json timestamp
* Expose USB port in DeviceInfo service
* Use latched QoS for Extrinsic topic when intra-process is used
* add cppcheck to GHA
* Fix Apache License Header and Intel Copyrights
* apply copyrights and license on project
* Enable intra-process communication for point clouds
* Fix ros2 parameter descriptions and range values
* T265 clean up
* fix float_to_double method
* realsense2_camera/src/sensor_params.cpp
* remove T265 device from ROS Wrapper - step1
* Enable D457
* Fix hdr_merge filter initialization in ros2 launch
* if default profile is not defined, take the first available profile as default
* changed to static_cast and added descriptor name and type
* remove extra ';'
* remove unused variable format_str
* publish point cloud via unique shared pointer
* make source backward compatible to older versions of cv_bridge and rclcpp
* add hdr_merge.enable and depth_module.hdr_enabled to rs_launch.py
* fix compilation errors
* fix tabs
* if default profile is not defined, take the first available profile as default
* Fix ros2 sensor controls steps and add control default value to param description
* Publish static transforms when intra porocess communication is enabled
* Properly read camera config files in rs_launch.py
* fix deprecated API
* Add D457
* Windows bring-up
* publish actual IMU optical frame ID in IMU messages
* Publish static tf for IMU frames
* fix extrinsics calculation
* fix ordered_pc arg prefix
* publish IMU frames only if unite/sync imu method is not none
* Publish static tf for IMU frames
* add D430i support
* Contributors: Arun Prasad, Arun Prasad V, Arun-Prasad-V, Christian Rauch, Daniel Honies, Gilad Bretter, Nir Azkiel, NirAz, Pranav Dhulipala, Samer Khshiboun, SamerKhshiboun, Stephan Wirth, Xiangyu, Yadunund, nvidia

4.51.1 (2022-09-13)
-------------------
* Fix crash when activating IMU & aligned depth together
* Fix rosbag device loading by preventing set_option to HDR/Gain/Exposure
* Support ROS2 Humble
* Publish real frame rate of realsense camera node topics/publishers
* No need to start/stop sensors for align depth changes
* Fix colorizer filter which returns null reference ptr
* Fix align_depth enable/disable
* Add colorizer.enable to rs_launch.py
* Add copyright and license to all ROS2-beta source files
* Fix CUDA suffix for pointcloud and align_depth topics
* Add ROS build farm pre-release to ci

* Contributors: Eran, NirAz, SamerKhshiboun

4.0.4 (2022-03-20)
------------------
* fix required packages for building debians for ros2-beta branch

* Contributors: NirAz

4.0.3 (2022-03-16)
------------------
* Support intra-process zero-copy
* Update README
* Fix Galactic deprecated-declarations compilation warning
* Fix Eloquent compilation error

* Contributors: Eran, Nir-Az, SamerKhshiboun

4.0.2 (2022-02-24)
------------------
* version 4.4.0 changed to 4.0.0 in CHANGELOG
* add frequency monitoring to /diagnostics topic.
* fix topic_hz.py to recognize message type from topic name. (Naive)
* move diagnostic updater for stream frequencies into the RosSensor class.
* add frequency monitoring to /diagnostics topic.
* fix galactic issue with undeclaring parameters
* fix to support Rolling.
* fix dynamic_params syntax.
* fix issue with Galactic parameters set by default to static which prevents them from being undeclared.

* Contributors: Haowei Wen, doronhi, remibettan

4.0.1 (2022-02-01)
------------------
* fix reset issue when multiple devices are connected
* fix /rosout issue
* fix PID for D405 device
* fix bug: frame_id is based on camera_name
* unite_imu_method is now changeable in runtime.
* fix motion module default values.
* add missing extrinsics topics
* fix crash when camera disconnects.
* fix header timestamp for metadata messages.

* Contributors: nomumu, JamesChooWK, benlev, doronhi

4.0.0 (2021-11-17)
-------------------
* changed parameters: 
  - "stereo_module", "l500_depth_sensor" are replaced by "depth_module"
  - for video streams: <module>.profile replaces <stream>_width, <stream>_height, <stream>_fps
  - removed paramets <stream>_frame_id, <stream>_optical_frame_id. frame_ids are defined by camera_name
  - "filters" is removed. All filters (or post-processing blocks) are enabled/disabled using "<filter>.enable"
  - "align_depth" is replaced with "align_depth.enable"
  - "allow_no_texture_points", "ordered_pc" replaced by "pointcloud.allow_no_texture_points", "pointcloud.ordered_pc"
  - "pointcloud_texture_stream", "pointcloud_texture_index" are replaced by "pointcloud.stream_filter", "pointcloud.stream_index_filter"

* Allow enable/disable of sensors in runtime.
* Allow enable/disable of filters in runtime.
