# Copyright 2024 Intel Corporation. All Rights Reserved.
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

cmake_minimum_required(VERSION 3.5)
project(realsense2_camera)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
option(BUILD_WITH_OPENMP "Use OpenMP" OFF)
option(SET_USER_BREAK_AT_STARTUP "Set user wait point in startup (for debug)" OFF)
# Define an option to enable or disable lifecycle nodes
option(USE_LIFECYCLE_NODE "Enable lifecycle nodes (ON/OFF)" OFF)

# Compiler Defense Flags
if(UNIX OR APPLE)
  # Linker flags.
  if(${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU" OR ${CMAKE_CXX_COMPILER_ID} STREQUAL "Intel")
    # GCC specific flags. ICC is compatible with them.
    set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -z noexecstack -z relro -z now")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -z noexecstack -z relro -z now")
  elseif(${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
    # In Clang, -z flags are not compatible, they need to be passed to linker via -Wl.
    set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,-z,noexecstack -Wl,-z,relro -Wl,-z,now")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-z,noexecstack -Wl,-z,relro -Wl,-z,now")
  endif()

  # Compiler flags.
  if(${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU")
    # GCC specific flags.
    if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 4.9 OR CMAKE_CXX_COMPILER_VERSION VERSION_EQUAL 4.9)
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIE -fstack-protector-strong")
    else()
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIE -fstack-protector")
    endif()
  elseif(${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
    # Clang is compatbile with some of the flags.
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIE -fstack-protector")
  elseif(${CMAKE_CXX_COMPILER_ID} STREQUAL "Intel")
    # Same as above, with exception that ICC compilation crashes with -fPIE option, even
    # though it uses -pie linker option that require -fPIE during compilation. Checksec
    # shows that it generates correct PIE anyway if only -pie is provided.
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fstack-protector")
  endif()

  # Generic flags.
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -fno-operator-names -Wformat -Wformat-security -Wall")
  # Dot not forward c++ flag to GPU beucause it is not supported
  set( CUDA_PROPAGATE_HOST_FLAGS OFF )
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -D_FORTIFY_SOURCE=2")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pie")
endif()

if(WIN32)
  add_definitions(-D_USE_MATH_DEFINES)
endif()

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

string(TOUPPER "${CMAKE_BUILD_TYPE}" uppercase_CMAKE_BUILD_TYPE)
if (${uppercase_CMAKE_BUILD_TYPE} STREQUAL "RELEASE")
    message(STATUS "Create Release Build.")
    set(CMAKE_CXX_FLAGS "-O2 ${CMAKE_CXX_FLAGS}")
else()
    message(STATUS "Create Debug Build.")
endif()

if(BUILD_WITH_OPENMP)
    find_package(OpenMP)
    if(NOT OpenMP_FOUND)
        message(FATAL_ERROR "\n\n OpenMP is missing!\n\n")
    else()
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -fopenmp")
        set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
    endif()
endif()

if(SET_USER_BREAK_AT_STARTUP)
    message("GOT FLAG IN CmakeLists.txt")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DBPDEBUG")
endif()


find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(realsense2_camera_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core)

find_package(realsense2 2.56)
if (BUILD_ACCELERATE_GPU_WITH_GLSL)
  find_package(realsense2-gl 2.56)
endif()
if(NOT realsense2_FOUND)
    message(FATAL_ERROR "\n\n Intel RealSense SDK 2.0 is missing, please install it from https://github.com/IntelRealSense/librealsense/releases\n\n")
endif()

#set(CMAKE_NO_SYSTEM_FROM_IMPORTED true)
include_directories(include)

include_directories(${OpenCV_INCLUDE_DIRS})  # add OpenCV includes to the included dirs

set(node_plugins "")

set(SOURCES
    src/realsense_node_factory.cpp
    src/base_realsense_node.cpp
    src/parameters.cpp
    src/rs_node_setup.cpp
    src/ros_sensor.cpp
    src/ros_utils.cpp
    src/dynamic_params.cpp
    src/sensor_params.cpp
    src/named_filter.cpp
    src/pointcloud_filter.cpp
    src/align_depth_filter.cpp
    src/profile_manager.cpp
    src/image_publisher.cpp
    src/tfs.cpp
    src/actions.cpp
  )

if (BUILD_ACCELERATE_GPU_WITH_GLSL)
  list(APPEND SOURCES src/gl_gpu_processing.cpp)
endif()

if(NOT DEFINED ENV{ROS_DISTRO})
  message(FATAL_ERROR "ROS_DISTRO is not defined." )
endif()
if("$ENV{ROS_DISTRO}" STREQUAL "foxy")
  message(STATUS "Build for ROS2 Foxy")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DFOXY")
  set(SOURCES "${SOURCES}" src/ros_param_backend.cpp)
elseif("$ENV{ROS_DISTRO}" STREQUAL "humble")
  message(STATUS "Build for ROS2 Humble")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DHUMBLE")
  set(SOURCES "${SOURCES}" src/ros_param_backend.cpp)
elseif("$ENV{ROS_DISTRO}" STREQUAL "iron")
  message(STATUS "Build for ROS2 Iron")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DIRON")
  set(SOURCES "${SOURCES}" src/ros_param_backend.cpp)
elseif("$ENV{ROS_DISTRO}" STREQUAL "rolling")
  message(STATUS "Build for ROS2 Rolling")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DROLLING")
  set(SOURCES "${SOURCES}" src/ros_param_backend.cpp)
elseif("$ENV{ROS_DISTRO}" STREQUAL "jazzy")
  message(STATUS "Build for ROS2 Jazzy")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DJAZZY")
  set(SOURCES "${SOURCES}" src/ros_param_backend.cpp)
elseif("$ENV{ROS_DISTRO}" STREQUAL "kilted")
  message(STATUS "Build for ROS2 Kilted")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DKILTED")
  set(SOURCES "${SOURCES}" src/ros_param_backend.cpp)
else()
  message(FATAL_ERROR "Unsupported ROS Distribution: " "$ENV{ROS_DISTRO}")
endif()

# The header 'cv_bridge/cv_bridge.hpp' was added in version 3.3.0. For older
# cv_bridge versions, we have to use the header 'cv_bridge/cv_bridge.h'.
if(${cv_bridge_VERSION} VERSION_GREATER_EQUAL "3.3.0")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DCV_BRDIGE_HAS_HPP")
endif()

# 'OnSetParametersCallbackType' is only defined for rclcpp 17 and onward.
if(${rclcpp_VERSION} VERSION_GREATER_EQUAL "17.0")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DRCLCPP_HAS_OnSetParametersCallbackType")
endif()

if (BUILD_ACCELERATE_GPU_WITH_GLSL)
  add_definitions(-DACCELERATE_GPU_WITH_GLSL)
endif()

set(INCLUDES
    include/constants.h
    include/realsense_node_factory.h
    include/base_realsense_node.h
    include/ros_sensor.h
    include/ros_utils.h
    include/dynamic_params.h
    include/sensor_params.h
    include/named_filter.h
    include/pointcloud_filter.h
    include/align_depth_filter.h
    include/ros_param_backend.h
    include/profile_manager.h
    include/image_publisher.h)

if (BUILD_ACCELERATE_GPU_WITH_GLSL)
  list(APPEND INCLUDES include/gl_window.h)
endif()

if (BUILD_TOOLS)

  include_directories(tools)
  set(INCLUDES ${INCLUDES}
    tools/frame_latency/frame_latency.h)

  set(SOURCES ${SOURCES}
    tools/frame_latency/frame_latency.cpp)
endif()

add_library(${PROJECT_NAME} SHARED
    ${INCLUDES}
    ${SOURCES}
)

if (BUILD_ACCELERATE_GPU_WITH_GLSL)
  set(link_libraries ${realsense2-gl_LIBRARY})
else()
  set(link_libraries ${realsense2_LIBRARY})
endif()

list(APPEND link_libraries ${OpenCV_LIBS})  # add OpenCV libs to link_libraries

target_link_libraries(${PROJECT_NAME}
    ${link_libraries}
)

set(dependencies
  cv_bridge
  image_transport
  rclcpp
  rclcpp_components
  realsense2_camera_msgs
  std_srvs
  std_msgs
  sensor_msgs
  nav_msgs
  tf2
  tf2_ros
  diagnostic_updater
)

set (targets
  cv_bridge::cv_bridge
  image_transport::image_transport
  rclcpp::rclcpp
  rclcpp_components::component
  ${realsense2_camera_msgs_TARGETS}
  ${std_srvs_TARGETS}
  ${std_msgs_TARGETS}
  sensor_msgs::sensor_msgs_library
  ${nav_msgs_TARGETS}
  tf2::tf2
  tf2_ros::tf2_ros
  diagnostic_updater::diagnostic_updater
)

if (BUILD_ACCELERATE_GPU_WITH_GLSL)
  list(APPEND dependencies realsense2-gl)
  list(APPEND targets realsense2-gl::realsense2-gl)
else()
  list(APPEND dependencies realsense2)
  list(APPEND targets realsense2::realsense2)
endif()

# If the flag is enabled, define the macro
if(USE_LIFECYCLE_NODE)
    find_package(rclcpp_lifecycle REQUIRED)
    find_package(lifecycle_msgs REQUIRED)
    list(APPEND dependencies rclcpp_lifecycle lifecycle_msgs)
    list(APPEND targets 
        rclcpp_lifecycle::rclcpp_lifecycle
        lifecycle_msgs::lifecycle_msgs__rosidl_typesupport_cpp)
    add_definitions(-DUSE_LIFECYCLE_NODE)
    message("🚀 USE_LIFECYCLE_NODE is ENABLED")

    # Create a configuration file for the launch file
    file(WRITE "${CMAKE_BINARY_DIR}/global_settings.yaml"
        "use_lifecycle_node: true\n")
else()
    file(WRITE "${CMAKE_BINARY_DIR}/global_settings.yaml"
        "use_lifecycle_node: false\n")
endif()

install(FILES "${CMAKE_BINARY_DIR}/global_settings.yaml"
    DESTINATION share/${PROJECT_NAME}/config)

target_link_libraries(${PROJECT_NAME}
  ${targets}
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "realsense2_camera::RealSenseNodeFactory"
  EXECUTABLE realsense2_camera_node
)

if(BUILD_TOOLS)

  rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "rs2_ros::tools::frame_latency::FrameLatencyNode"
  EXECUTABLE realsense2_frame_latency_node
  )

endif()

# Install binaries
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install headers
install(
  DIRECTORY include/
  DESTINATION include
)

# Install launch files
install(DIRECTORY 
    launch
    DESTINATION share/${PROJECT_NAME}
    )

# Install example files
install(DIRECTORY 
    examples
    DESTINATION share/${PROJECT_NAME}
)

# Test
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  set(_gtest_folders
    test
  )
  foreach(test_folder ${_gtest_folders})
    file(GLOB files "${test_folder}/gtest_*.cpp")
    foreach(file ${files})
       get_filename_component(_test_name ${file} NAME_WE)
       ament_add_gtest(${_test_name} ${file})
       target_include_directories(${_test_name} PUBLIC
          $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
          $<INSTALL_INTERFACE:include>
       )
       target_link_libraries(${_test_name}
          std_srvs::std_srvs__rosidl_typesupport_cpp
          std_msgs::std_msgs__rosidl_typesupport_cpp
       )
       #target_link_libraries(${_test_name} name_of_local_library)
    endforeach()
  endforeach()


  find_package(ament_cmake_pytest REQUIRED)
  set(_pytest_folders
    test
    test/templates
    test/rosbag
    test/post_processing_filters
  )
  foreach(test_folder ${_pytest_folders})
    file(GLOB files "${test_folder}/test_*.py")
    foreach(file ${files})

    get_filename_component(_test_name ${file} NAME_WE)
    ament_add_pytest_test(${_test_name} ${file}
      APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}:${CMAKE_SOURCE_DIR}/test/utils:${CMAKE_SOURCE_DIR}/launch:${CMAKE_SOURCE_DIR}/scripts
      TIMEOUT 60
      WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    )
    endforeach()
  endforeach()

  unset(_pytest_folders)

  set(rs_query_cmd "rs-enumerate-devices -s")
  execute_process(COMMAND bash -c ${rs_query_cmd}
      WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
      RESULT_VARIABLE rs_result
      OUTPUT_VARIABLE RS_DEVICE_INFO)
  message(STATUS "rs_device_info:")
  message(STATUS "${RS_DEVICE_INFO}")
  if((RS_DEVICE_INFO MATCHES "D455") OR (RS_DEVICE_INFO MATCHES "D415") OR (RS_DEVICE_INFO MATCHES "D435"))
    message(STATUS "D455 device found")
    set(_pytest_live_folders
      test/live_camera
    )
  endif()

  foreach(test_folder ${_pytest_live_folders})
    file(GLOB files "${test_folder}/test_*.py")
    foreach(file ${files})

    get_filename_component(_test_name ${file} NAME_WE)
    ament_add_pytest_test(${_test_name} ${file}
      APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}:${CMAKE_SOURCE_DIR}/test/utils:${CMAKE_SOURCE_DIR}/launch:${CMAKE_SOURCE_DIR}/scripts
      TIMEOUT 500
      WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    )
    endforeach()
  endforeach()

endif()

# Ament exports
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

ament_package()
