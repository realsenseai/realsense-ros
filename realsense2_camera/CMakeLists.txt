cmake_minimum_required(VERSION 2.8.3)
project(realsense2_camera)
add_compile_options(-std=c++11)

option(BUILD_WITH_OPENMP "Use OpenMP" OFF)
option(SET_USER_BREAK_AT_STARTUP "Set user wait point in startup (for debug)" OFF)

add_definitions(-D_CRT_SECURE_NO_WARNINGS)
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

find_package(catkin REQUIRED COMPONENTS
    message_generation
    nav_msgs
    roscpp
    sensor_msgs
    std_msgs
    std_srvs
    nodelet
    cv_bridge
    image_transport
    tf
    ddynamic_reconfigure
    diagnostic_updater
    )

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

if (WIN32)
find_package(realsense2 CONFIG REQUIRED)
else()
find_package(realsense2 2.50.0)
endif()

if(NOT realsense2_FOUND)
    message(FATAL_ERROR "\n\n Intel RealSense SDK 2.0 is missing, please install it from https://github.com/IntelRealSense/librealsense/releases\n\n")
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

if (WIN32)
else()
set(CMAKE_CXX_FLAGS "-fPIE -fPIC -std=c++11 -D_FORTIFY_SOURCE=2 -fstack-protector -Wformat -Wformat-security -Wall ${CMAKE_CXX_FLAGS}")
endif()

add_message_files(
    FILES
    IMUInfo.msg
    Extrinsics.msg
    Metadata.msg
    )

add_service_files(
    FILES
    DeviceInfo.srv
    GetLatestFrame.srv
)

generate_messages(
    DEPENDENCIES
    sensor_msgs
    std_msgs
    )

set(CMAKE_NO_SYSTEM_FROM_IMPORTED true)
include_directories(
    include
    ${realsense2_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
    )

# RealSense ROS Node
catkin_package(
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS message_runtime roscpp sensor_msgs std_msgs
    nodelet
    cv_bridge
    image_transport
    ddynamic_reconfigure
    nav_msgs
    )

add_library(${PROJECT_NAME}
    include/constants.h
    include/realsense_node_factory.h
    include/base_realsense_node.h
    include/t265_realsense_node.h
    src/realsense_node_factory.cpp
    src/base_realsense_node.cpp
    src/t265_realsense_node.cpp
    )

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

target_include_directories(${PROJECT_NAME}
  PRIVATE ${realsense2_INCLUDE_DIR})

target_link_libraries(${PROJECT_NAME}
    ${realsense2_LIBRARY}
    ${catkin_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
    )

if(WIN32)
set_target_properties(${realsense2_LIBRARY} PROPERTIES MAP_IMPORTED_CONFIG_RELWITHDEBINFO RELEASE)
target_link_libraries(${PROJECT_NAME}
    realsense2::realsense2 
    realsense2::realsense-file
    )
endif()


# Install nodelet library
install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

# Install header files
install(DIRECTORY include/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    )

# Install launch files
install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    )

# Install rviz files
install(DIRECTORY rviz/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
    )

# Install xml files
install(FILES nodelet_plugins.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    )

