^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package realsense2_camera
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

4.56.4 (2025-07-22)
-------------------
* PR `#3329 <https://github.com/IntelRealSense/realsense-ros/issues/3329>`_ from Nir-Az: Update version to 4.56.4
* update version to 4.56.4
* PR `#3325 <https://github.com/IntelRealSense/realsense-ros/issues/3325>`_ from ashrafk93: use ykush to switch ports
* test ci
* disable all ports
* change port logic
* map ports dynamically
* use yakush to switch ports
* PR `#3319 <https://github.com/IntelRealSense/realsense-ros/issues/3319>`_ from ashrafk93: Add LifeCycle Node support at compile time
* Remove message on default normal node
* Fix typo
* Update config file name
* Update README
* Fix PR markups
* Add lifecycle node dependency
* Merge pull request `#3313 <https://github.com/IntelRealSense/realsense-ros/issues/3313>`_ from FarStryke21/ros2-development
  Added base_frame_id param to the rs_launch.py file
* Added base_frame_id param to the rs_launch.py file
* Merge remote-tracking branch 'origin/ros2-development' into change_topics_in_readme
* PR `#3303 <https://github.com/IntelRealSense/realsense-ros/issues/3303>`_ from noacoohen: Enable rotation filter for color and depth sensors
* rotate infra sensor
* enable rotation filter for color and depth
* PR `#3293 <https://github.com/IntelRealSense/realsense-ros/issues/3293>`_ from remibettan: align_depth_to_infra2 enabled, pointcloud and align_depth filters to own files
* cr adjustments
* align_depth_to_infra2 enabled, pointcloud and align_depth filters to own files
* PR `#3284 <https://github.com/IntelRealSense/realsense-ros/issues/3284>`_ from noacoohen: Add color format to depth module in the launch file
* edit description
* add color foramt for depth module for d405
* PR  `#3274 <https://github.com/IntelRealSense/realsense-ros/issues/3274>`_ from noacoohen: Enable rotation filter ROS2
* PR `#3276 <https://github.com/IntelRealSense/realsense-ros/issues/3276>`_ from remibettan: removing dead code in RosSensor class
  runFirstFrameInitialization and _first_frame_functions_stack removed
* removing dead code
* init _is_first_frame to true
* Update rs_launch.py to include additional rotation filter support
* add rotation filter to ros2
* PR `#3214 <https://github.com/IntelRealSense/realsense-ros/issues/3214>`_ from acornaglia: Add ROS bag loop option
* Merge branch 'ros2-development' into change_topics_in_readme
* PR `#3239 <https://github.com/IntelRealSense/realsense-ros/issues/3239>`_ from SamerKhshiboun: Update CMakeLists.txt - remove find_package(fastrtps REQUIRED)
* Update CMakeLists.txt
* Update CMakeLists.txt
* Merge branch 'ros2-development' of github.com:IntelRealSense/realsense-ros into rosbag_loop
* Using variable for avoiding double function call
* Merge branch 'ros2-development' into rosbag_loop
* PR `#3225 <https://github.com/IntelRealSense/realsense-ros/issues/3225>`_ from SamerKhshiboun: Use new APIs for motion, accel and gryo streams
* use new APIs for motion, accel and gryo streams
* PR `#3222 <https://github.com/IntelRealSense/realsense-ros/issues/3222>`_ from SamerKhshiboun: Support D555 and its motion profiles
* support latched QoS for imu_info
* adjusments to DDS Support for ROS Wrapper
* Add D555 PID
* PR `#3221 <https://github.com/IntelRealSense/realsense-ros/issues/3221>`_ from patrickwasp: fix config typo
* fix config typo
* PR `#3216 <https://github.com/IntelRealSense/realsense-ros/issues/3216>`_ from PrasRsRos: hw_reset implementation
* Added rosbag_loop parameter to launch_from_rosbag example
* Added loop back argument description to launch_from_rosbag README
* Added hw_reset service implementation
  Added a test in D455 to validate the hw_reset
* Added hw_reset service implementation
  Added a test in D455 to validate the hw_reset
* Added hw_reset service implementation
  Added a test in D455 to validate the hw_reset
* Added hw_reset service call to reset the device
  Added a test in D455 to test the reset device
* Added hw_reset service call to reset the device
* Fixed bag looping mechanism
* Fixed compilation error
* Added ROS bag loop option
* PR `#3200 <https://github.com/IntelRealSense/realsense-ros/issues/3200>`_ from kadiredd: retry thrice finding devices with Ykush reset
* retry thrice finding devices with Ykush reset
* retry thrice finding devices with Ykush reset
* PR `#3178 <https://github.com/IntelRealSense/realsense-ros/issues/3178>`_ from kadiredd: disabling FPS & TF tests for ROS-CI
* disabling FPS & TF tests for ROS-CI
* PR `#3166 <https://github.com/IntelRealSense/realsense-ros/issues/3166>`_ from SamerKhshiboun: Update Calibration Config API
* update calib config usage to the new API, and update readme
* PR `#3159 <https://github.com/IntelRealSense/realsense-ros/issues/3159>`_ from noacoohen: Add D421 PID
* add D421 pid
* PR `#3153 <https://github.com/IntelRealSense/realsense-ros/issues/3153>`_ from SamerKhshiboun: TC | Fix feedback and update readme
* fix feedback and update readme for TC
* PR `#3147 <https://github.com/IntelRealSense/realsense-ros/issues/3147>`_ from SamerKhshiboun: Update READMEs and CONTRIBUTING files regarding ros2-master
* PR `#3148 <https://github.com/IntelRealSense/realsense-ros/issues/3148>`_ from SamerKhshiboun: update READMEs and CONTRIBUTING files regarding ros2-master
* update READMEs and CONTRIBUTING files regarding ros2-master
* update READMEs and CONTRIBUTING files regarding ros2-master
* PR `#3138 <https://github.com/IntelRealSense/realsense-ros/issues/3138>`_ from SamerKhshiboun: Support Triggered Calibration as ROS2 Action
* implement Triggered Calibration action
* Update CMakeLists.txt
* PR `#3135 <https://github.com/IntelRealSense/realsense-ros/issues/3135>`_ from kadiredd: Casefolding device name instead of strict case sensitive comparison
* Casefolding device name instead os strict case sensitive comparison
* PR `#3133 <https://github.com/IntelRealSense/realsense-ros/issues/3133>`_ from SamerKhshiboun: update librealsense2 version to 2.56.0
* update librealsense2 version to 2.56.0
  since it includes new API that need for ros2-development
* PR `#3124 <https://github.com/IntelRealSense/realsense-ros/issues/3124>`_ from kadiredd: Support testing ROS2 service call device_info
* PR `#3125 <https://github.com/IntelRealSense/realsense-ros/issues/3125>`_ from SamerKhshiboun: Support calibration config read/write services
* support calib config read/write services
* [ROS][Test Infra] Support testing ROS2 service call device_info
* PR `#3114 <https://github.com/IntelRealSense/realsense-ros/issues/3114>`_ from Arun-Prasad-V: Ubuntu 24.04 support for Rolling and Jazzy distros
* Ubuntu 24.04 support for Rolling and Jazzy
* PR `#3102 <https://github.com/IntelRealSense/realsense-ros/issues/3102>`_ from fortizcuesta: Allow hw synchronization of several realsense using a synchonization cable
* PR `#3096 <https://github.com/IntelRealSense/realsense-ros/issues/3096>`_ from anisotropicity: Update rs_launch.py to add depth_module.color_profile
* Expose depth_module.inter_cam_sync_mode in launch files
* Revert changes
* Merge branch 'ros2-development' into feature/ros2-development-allow-hw-synchronization
* Allow hw synchronization of several realsense devices
* Update rs_launch.py to add depth_module.color_profile
  Fix for not being able to set the color profile for D405 cameras.
  See https://github.com/IntelRealSense/realsense-ros/issues/3090
* Contributors: Aman Chulawala, Arun-Prasad-V, Ashraf Kattoura, Cornaglia, Alessandro, Madhukar Reddy Kadireddy, Nir Azkiel, Ortiz Cuesta, Fernando, Patrick Wspanialy, PrasRsRos, Remi Bettan, Samer Khshiboun, SamerKhshiboun, acornaglia, administrator, anisotropicity, louislelay, noacoohen, remibettan

4.55.1 (2024-05-28)
-------------------
* PR `#3106 <https://github.com/IntelRealSense/realsense-ros/issues/3106>`_ from SamerKhshiboun: Remove unused parameter _is_profile_exist
* PR `#3098 <https://github.com/IntelRealSense/realsense-ros/issues/3098>`_ from kadiredd: ROS live cam test fixes
* PR `#3094 <https://github.com/IntelRealSense/realsense-ros/issues/3094>`_ from kadiredd: ROSCI infra for live camera testing
* PR `#3066 <https://github.com/IntelRealSense/realsense-ros/issues/3066>`_ from SamerKhshiboun: Revert Foxy Build Support (From Source)
* PR `#3052 <https://github.com/IntelRealSense/realsense-ros/issues/3052>`_ from Arun-Prasad-V: Support for selecting profile for each stream_type
* PR `#3056 <https://github.com/IntelRealSense/realsense-ros/issues/3056>`_ from SamerKhshiboun: Add documentation for RealSense ROS2 Wrapper Windows installation
* PR `#3049 <https://github.com/IntelRealSense/realsense-ros/issues/3049>`_ from Arun-Prasad-V: Applying Colorizer filter to Aligned-Depth image
* PR `#3053 <https://github.com/IntelRealSense/realsense-ros/issues/3053>`_ from Nir-Az: Fix Coverity issues + remove empty warning log
* PR `#3007 <https://github.com/IntelRealSense/realsense-ros/issues/3007>`_ from Arun-Prasad-V: Skip updating Exp 1,2 & Gain 1,2 when HDR is disabled
* PR `#3042 <https://github.com/IntelRealSense/realsense-ros/issues/3042>`_ from kadiredd: Assert Fail if camera not found
* PR `#3008 <https://github.com/IntelRealSense/realsense-ros/issues/3008>`_ from Arun-Prasad-V: Renamed GL GPU enable param
* PR `#2989 <https://github.com/IntelRealSense/realsense-ros/issues/2989>`_ from Arun-Prasad-V: Dynamically switching b/w CPU & GPU processing
* PR `#3001 <https://github.com/IntelRealSense/realsense-ros/issues/3001>`_ from deep0294: Update ReadMe to run ROS2 Unit Test
* PR `#2998 <https://github.com/IntelRealSense/realsense-ros/issues/2998>`_ from SamerKhshiboun: fix calibration intrinsic fail
* PR `#2987 <https://github.com/IntelRealSense/realsense-ros/issues/2987>`_ from SamerKhshiboun: Remove D465 SKU
* PR `#2984 <https://github.com/IntelRealSense/realsense-ros/issues/2984>`_ from deep0294: Fix All Profiles Test
* PR `#2956 <https://github.com/IntelRealSense/realsense-ros/issues/2956>`_ from Arun-Prasad-V: Extending LibRS's GL support to RS ROS2
* PR `#2953 <https://github.com/IntelRealSense/realsense-ros/issues/2953>`_ from Arun-Prasad-V: Added urdf & mesh files for D405 model
* PR `#2940 <https://github.com/IntelRealSense/realsense-ros/issues/2940>`_ from Arun-Prasad-V: Fixing the data_type of ROS Params exposure & gain
* PR `#2948 <https://github.com/IntelRealSense/realsense-ros/issues/2948>`_ from Arun-Prasad-V: Disabling HDR during INIT
* PR `#2934 <https://github.com/IntelRealSense/realsense-ros/issues/2934>`_ from Arun-Prasad-V: Disabling hdr while updating exposure & gain values
* PR `#2946 <https://github.com/IntelRealSense/realsense-ros/issues/2946>`_ from gwen2018: fix ros random crash with error hw monitor command for asic temperature failed
* PR `#2865 <https://github.com/IntelRealSense/realsense-ros/issues/2865>`_ from PrasRsRos: add live camera tests
* PR `#2891 <https://github.com/IntelRealSense/realsense-ros/issues/2891>`_ from Arun-Prasad-V: revert PR2872
* PR `#2853 <https://github.com/IntelRealSense/realsense-ros/issues/2853>`_ from Arun-Prasad-V: Frame latency for the '/topic' provided by user
* PR `#2872 <https://github.com/IntelRealSense/realsense-ros/issues/2872>`_ from Arun-Prasad-V: Updating _camera_name with RS node's name
* PR `#2878 <https://github.com/IntelRealSense/realsense-ros/issues/2878>`_ from Arun-Prasad-V: Updated ros2 examples and readme
* PR `#2841 <https://github.com/IntelRealSense/realsense-ros/issues/2841>`_ from SamerKhshiboun: Remove Dashing, Eloquent, Foxy, L500 and SR300 support
* PR `#2868 <https://github.com/IntelRealSense/realsense-ros/issues/2868>`_ from Arun-Prasad-V: Fix Pointcloud topic frame_id
* PR `#2849 <https://github.com/IntelRealSense/realsense-ros/issues/2849>`_ from Arun-Prasad-V: Create /imu topic only when motion streams enabled
* PR `#2847 <https://github.com/IntelRealSense/realsense-ros/issues/2847>`_ from Arun-Prasad-V: Updated rs_launch param names
* PR `#2839 <https://github.com/IntelRealSense/realsense-ros/issues/2839>`_ from Arun-Prasad: Added ros2 examples
* PR `#2861 <https://github.com/IntelRealSense/realsense-ros/issues/2861>`_ from SamerKhshiboun: fix readme and nodefactory for ros2 run
* PR `#2859 <https://github.com/IntelRealSense/realsense-ros/issues/2859>`_ from PrasRsRos: Fix tests (topic now has camera name)
* PR `#2857 <https://github.com/IntelRealSense/realsense-ros/issues/2857>`_ from lge-ros2: Apply camera name in topics
* PR `#2840 <https://github.com/IntelRealSense/realsense-ros/issues/2840>`_ from SamerKhshiboun: Support Depth, IR and Color formats in ROS2
* PR `#2764 <https://github.com/IntelRealSense/realsense-ros/issues/2764>`_ from lge-ros2 : support modifiable camera namespace
* PR `#2830 <https://github.com/IntelRealSense/realsense-ros/issues/2830>`_ from SamerKhshiboun: Add RGBD + reduce changes between hkr and development
* PR `#2811 <https://github.com/IntelRealSense/realsense-ros/issues/2811>`_ from Arun-Prasad-V: Exposing stream formats params to user
* PR `#2825 <https://github.com/IntelRealSense/realsense-ros/issues/2825>`_ from SamerKhshiboun: Fix align_depth + add test
* PR `#2822 <https://github.com/IntelRealSense/realsense-ros/issues/2822>`_ from Arun-Prasad-V: Updated rs_launch configurations
* PR `#2726 <https://github.com/IntelRealSense/realsense-ros/issues/2726>`_ from PrasRsRos: Integration test template
* PR `#2742 <https://github.com/IntelRealSense/realsense-ros/issues/2742>`_ from danielhonies:Update rs_launch.py
* PR `#2806 <https://github.com/IntelRealSense/realsense-ros/issues/2806>`_ from Arun-Prasad-V: Enabling RGB8 Infrared stream
* PR `#2799 <https://github.com/IntelRealSense/realsense-ros/issues/2799>`_ from SamerKhshiboun: Fix overriding frames on same topics/CV-images due to a bug in PR2759
* PR `#2759 <https://github.com/IntelRealSense/realsense-ros/issues/2759>`_ from SamerKhshiboun: Cleanups and name fixes
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
