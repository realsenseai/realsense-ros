^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package realsense2_rviz_plugin
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

4.57.4 (2025-11-02)
-------------------
* Update realsense2_rgbd_plugin package.xml to 4.57.4
* PR `#3387 <https://github.com/IntelRealSense/realsense-ros/issues/3387>`_ from NetanelBlumenfeld: add rgbd plug-in for rviz2
* Update CMakeLists.txt CR year
* fix copy rights
* feat(rviz): add RGBD plugin and clean integration
  - Add RealSense RGBD RViz plugin (adapted from private repo)
  - Disable TF filtering to avoid message queue drops
  - Use target_link_libraries (no ament_target_dependencies)
  - Fix CMake message targets (ROS 2 Humble typesupport)
  - Minor README fixes
* Contributors: NetanelBlumenfeld, Nir Azkiel
