# Copyright 2025 RealSense, Inc. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.8)
project(realsense2_rviz_plugin)

# C++ Standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

option(RVIZ_RGBD_PLUGIN "Enable RVIZ RGBD plugin-specific code" OFF)

find_package(ament_cmake REQUIRED)

if(RVIZ_RGBD_PLUGIN)
  message(STATUS "RVIZ_RGBD_PLUGIN is ENABLED")
  find_package(rclcpp REQUIRED)
  find_package(realsense2_camera_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(cv_bridge REQUIRED)
  find_package(rviz_common REQUIRED)
  find_package(pluginlib REQUIRED)
  find_package(OpenCV REQUIRED)
  find_package(Qt5 REQUIRED COMPONENTS Widgets)
  find_package(rviz_rendering REQUIRED)
  find_package(rviz_default_plugins REQUIRED)

  set(extra_deps "")

  # Enable Qt MOC processing
  set(CMAKE_AUTOMOC ON)
  set(CMAKE_INCLUDE_CURRENT_DIR ON)

  qt5_wrap_cpp(MOC_FILES
    include/realsense_rviz_plugin.hpp
  )

  # Include directories
  include_directories(
    include
    ${Qt5Widgets_INCLUDE_DIRS}
  )

  # Sources
  set(SOURCES
    src/realsense_rviz_plugin.cpp
    src/ros_image_texture.cpp
    ${MOC_FILES}
  )

  # Add the plugin library
  add_library(realsense_rviz_plugin SHARED
    ${SOURCES}
  )

  target_compile_definitions(realsense_rviz_plugin PRIVATE "RVIZ_DEFAULT_PLUGINS_BUILDING_LIBRARY")
  target_compile_definitions(realsense_rviz_plugin PRIVATE RVIZ_RGBD_PLUGIN)

  # --- Modern CMake: link all dependencies explicitly ---
  # NOTE: Use ROS 2 typesupport targets for message packages.
  target_link_libraries(realsense_rviz_plugin
    PUBLIC
      rclcpp::rclcpp
      cv_bridge::cv_bridge
      rviz_common::rviz_common
      rviz_rendering::rviz_rendering
      pluginlib::pluginlib
      rviz_default_plugins::rviz_default_plugins
      sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
      realsense2_camera_msgs::realsense2_camera_msgs__rosidl_typesupport_cpp
      Qt5::Widgets
    PRIVATE
      ${OpenCV_LIBRARIES}
  )

  # Write a config YAML indicating if the feature is enabled
  file(WRITE "${CMAKE_BINARY_DIR}/rviz_rgbd_plugin_settings.yaml"
    "rviz_rgbd_plugin_enabled: true\n"
  )

  install(FILES "${CMAKE_BINARY_DIR}/rviz_rgbd_plugin_settings.yaml"
    DESTINATION share/${PROJECT_NAME}/config
  )

  # Install plugin
  install(TARGETS realsense_rviz_plugin
    EXPORT ${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )

  # Install plugin description
  install(FILES plugins/realsense_rviz_plugin_description.xml
    DESTINATION share/${PROJECT_NAME}
  )

  # Set up package info
  pluginlib_export_plugin_description_file(rviz_common plugins/realsense_rviz_plugin_description.xml)

  ament_export_include_directories(include)
  ament_export_libraries(realsense_rviz_plugin)
  ament_export_dependencies(
    rclcpp
    realsense2_camera_msgs
    sensor_msgs
    cv_bridge
    rviz_common
    rviz_rendering
    Qt5
    rviz_default_plugins
    ${extra_deps}
  )

else()
  message(STATUS "RVIZ_RGBD_PLUGIN is DISABLED: Plugin will not be built.")
  file(WRITE "${CMAKE_BINARY_DIR}/rviz_rgbd_plugin_settings.yaml"
    "rviz_rgbd_plugin_enabled: false\n"
  )
  install(FILES "${CMAKE_BINARY_DIR}/rviz_rgbd_plugin_settings.yaml"
    DESTINATION share/${PROJECT_NAME}/config
  )
endif()

ament_package()
