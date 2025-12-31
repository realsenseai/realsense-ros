^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package realsense2_ros_mqtt_bridge
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

4.57.3 (2025-09-15)
-------------------
* PR `#3428 <https://github.com/realsenseai/realsense-ros/issues/3428>`_ from remibettan: fixing mqtt readme
* PR `#3417 <https://github.com/realsenseai/realsense-ros/issues/3417>`_ from remibettan: Merging ros2 hkr to ros2 dev final
* PR `#38 <https://github.com/realsenseai/realsense-ros/issues/38>`_ from PrasRsRos: Implement hw reset in mqtt and add test
* PR `#37 <https://github.com/realsenseai/realsense-ros/issues/37>`_ from PrasRsRos: Hmc test update
* PR `#36 <https://github.com/realsenseai/realsense-ros/issues/36>`_ from PrasRsRos: Add hmc command tests to mqtt
* PR `#32 <https://github.com/realsenseai/realsense-ros/issues/32>`_ from SamerKhshiboun: Support HWM command as ROS2 service and in the ROS-MQTT bridge node
* PR `#34 <https://github.com/realsenseai/realsense-ros/issues/34>`_ from PrasRsRos: mqtt tf testcase
* PR `#30 <https://github.com/realsenseai/realsense-ros/issues/30>`_ from SamerKhshiboun: Use new apis of SIC and SP that works directly with JSON inputs/outputs
* PR `#31 <https://github.com/realsenseai/realsense-ros/issues/31>`_ from SamerKhshiboun: Support TF lookup for ROS-MQTT bridge node
* PR `#29 <https://github.com/realsenseai/realsense-ros/issues/29>`_ from PrasRsRos/mqtt_negative__tests
* PR `#28 <https://github.com/realsenseai/realsense-ros/issues/28>`_ from SamerKhshiboun: Fix MQTT Demo and update values for and update TC consecutives
* PR `#26 <https://github.com/realsenseai/realsense-ros/issues/26>`_ from PrasRsRos: More system tests for mqtt
* PR `#24 <https://github.com/realsenseai/realsense-ros/issues/24>`_ from PrasRsRos: Test TC on two cameras in parallel
* PR `#23 <https://github.com/realsenseai/realsense-ros/issues/23>`_ from PrasRsRos: Ros tc implementation
* PR `#22 <https://github.com/realsenseai/realsense-ros/issues/22>`_ from PrasRsRos: Added device info test at the system level
* PR `#17 <https://github.com/realsenseai/realsense-ros/issues/17>`_ from SamerKhshiboun: Change sucess response "true/false" to be boolean
* PR `#19 <https://github.com/realsenseai/realsense-ros/issues/19>`_ from PrasRsRos: DeviceInfo and TC Mqtt tests
* PR `#16 <https://github.com/realsenseai/realsense-ros/issues/16>`_ from PrasRsRos: RS ROS Mqtt bridge unit tests
* PR `#15 <https://github.com/realsenseai/realsense-ros/issues/15>`_ from SamerKhshiboun: Support device info service in ROS-MQTT bridge
* PR `#14 <https://github.com/realsenseai/realsense-ros/issues/14>`_ from SamerKhshiboun: Merge ros2-development into hkr
* PR `#13 <https://github.com/realsenseai/realsense-ros/issues/13>`_ from SamerKhshiboun: Sٍupport set/get application config as ROS service and in ROS-MQTT bridge
* PR `#12 <https://github.com/realsenseai/realsense-ros/issues/12>`_ from SamerKhshiboun: TC Support in ROS-MQTT Bridge Node
* PR `#10 <https://github.com/realsenseai/realsense-ros/issues/10>`_ from SamerKhshiboun: Add ROS MQTT Bridge (Python) Node Into realsense-ros-private
* Contributors: Nir Azkiel, PrasRsRos, Remi Bettan, Samer Khshiboun

* PR `#3428 <https://github.com/realsenseai/realsense-ros/issues/3428>`_ from remibettan: fixing mqtt readme
* readme fixed
* PR `#3417 <https://github.com/realsenseai/realsense-ros/issues/3417>`_ from remibettan: Merging ros2 hkr to ros2 dev final
* PR `#38 <https://github.com/realsenseai/realsense-ros/issues/38>`_ from PrasRsRos: Implement hw reset in mqtt and add test
* PR `#37 <https://github.com/realsenseai/realsense-ros/issues/37>`_ from PrasRsRos: Hmc test update
* PR `#36 <https://github.com/realsenseai/realsense-ros/issues/36>`_ from PrasRsRos: Add hmc command tests to mqtt
* PR `#32 <https://github.com/realsenseai/realsense-ros/issues/32>`_ from SamerKhshiboun: Support HWM command as ROS2 service and in the ROS-MQTT bridge node
* PR `#34 <https://github.com/realsenseai/realsense-ros/issues/34>`_ from PrasRsRos: mqtt tf testcase
* PR `#30 <https://github.com/realsenseai/realsense-ros/issues/30>`_ from SamerKhshiboun: Use new apis of SIC and SP that works directly with JSON inputs/outputs
* PR `#31 <https://github.com/realsenseai/realsense-ros/issues/31>`_ from SamerKhshiboun: Support TF lookup for ROS-MQTT bridge node
* PR `#29 <https://github.com/realsenseai/realsense-ros/issues/29>`_ from PrasRsRos/mqtt_negative__tests
* PR `#28 <https://github.com/realsenseai/realsense-ros/issues/28>`_ from SamerKhshiboun: Fix MQTT Demo and update values for and update TC consecutives failures threshold
* PR `#26 <https://github.com/realsenseai/realsense-ros/issues/26>`_ from PrasRsRos: More system tests for mqtt
* PR `#24 <https://github.com/realsenseai/realsense-ros/issues/24>`_ from PrasRsRos: Test TC on two cameras in parallel
* PR `#23 <https://github.com/realsenseai/realsense-ros/issues/23>`_ from PrasRsRos: Ros tc implementation
* PR `#22 <https://github.com/realsenseai/realsense-ros/issues/22>`_ from PrasRsRos: Added device info test at the system level
* PR `#17 <https://github.com/realsenseai/realsense-ros/issues/17>`_ from SamerKhshiboun: Change sucess response "true/false" to be boolean
* PR `#19 <https://github.com/realsenseai/realsense-ros/issues/19>`_ from PrasRsRos: DeviceInfo and TC Mqtt tests
* PR `#16 <https://github.com/realsenseai/realsense-ros/issues/16>`_ from PrasRsRos: RS ROS Mqtt bridge unit tests
* PR `#15 <https://github.com/realsenseai/realsense-ros/issues/15>`_ from SamerKhshiboun: Support device info service in ROS-MQTT bridge
* PR `#14 <https://github.com/realsenseai/realsense-ros/issues/14>`_ from SamerKhshiboun: Merge ros2-development into hkr
* PR `#13 <https://github.com/realsenseai/realsense-ros/issues/13>`_ from SamerKhshiboun: Sٍupport set/get application config as ROS service and in ROS-MQTT bridge
* PR `#12 <https://github.com/realsenseai/realsense-ros/issues/12>`_ from SamerKhshiboun: TC Support in ROS-MQTT Bridge Node
* PR `#10 <https://github.com/realsenseai/realsense-ros/issues/10>`_ from SamerKhshiboun: Add ROS MQTT Bridge (Python) Node Into realsense-ros-private
* Contributors: Nir Azkiel, PrasRsRos, Remi Bettan, Samer Khshiboun
