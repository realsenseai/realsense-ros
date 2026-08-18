cmake_minimum_required(VERSION 3.22)
project(nitros_zc_bench LANGUAGES CXX CUDA)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(CUDA REQUIRED)
find_package(isaac_ros_nitros REQUIRED)
find_package(isaac_ros_managed_nitros REQUIRED)
find_package(isaac_ros_nitros_image_type REQUIRED)
find_package(rclcpp_components REQUIRED)

include_directories(src ${CUDA_INCLUDE_DIRS})

# The pixel-touching kernel both consumers run, so the GPU work is identical on both paths.
add_library(bench_kernel STATIC src/bench_kernel.cu src/viz_kernel.cu)
set_target_properties(bench_kernel PROPERTIES POSITION_INDEPENDENT_CODE ON CUDA_ARCHITECTURES "87")

add_executable(cpu_consumer src/cpu_consumer.cpp)
target_link_libraries(cpu_consumer bench_kernel ${CUDA_LIBRARIES})
ament_target_dependencies(cpu_consumer rclcpp sensor_msgs)

add_executable(nitros_consumer src/nitros_consumer.cpp)
target_link_libraries(nitros_consumer bench_kernel ${CUDA_LIBRARIES})
ament_target_dependencies(nitros_consumer
  rclcpp sensor_msgs isaac_ros_nitros isaac_ros_managed_nitros isaac_ros_nitros_image_type)

# Same consumers as loadable components, so they can be composed into the camera node's process.
# That is the configuration where NITROS actually hands over a pointer: across a process boundary
# rclcpp has to run the NitrosImage type adapter, which does a device->host copy.
add_library(nitros_zc_bench_components SHARED
  src/cpu_consumer.cpp src/nitros_consumer.cpp src/nitros_viewer.cpp)
target_compile_definitions(nitros_zc_bench_components PRIVATE NITROS_ZC_BENCH_COMPONENT_ONLY)
target_link_libraries(nitros_zc_bench_components bench_kernel ${CUDA_LIBRARIES})
ament_target_dependencies(nitros_zc_bench_components
  rclcpp rclcpp_components sensor_msgs isaac_ros_nitros isaac_ros_managed_nitros
  isaac_ros_nitros_image_type)
rclcpp_components_register_nodes(nitros_zc_bench_components
  "nitros_zc_bench::NitrosConsumer" "nitros_zc_bench::CpuConsumer" "nitros_zc_bench::NitrosViewer")

install(TARGETS nitros_zc_bench_components
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS cpu_consumer nitros_consumer DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/run_bench.sh DESTINATION lib/${PROJECT_NAME})

ament_package()
