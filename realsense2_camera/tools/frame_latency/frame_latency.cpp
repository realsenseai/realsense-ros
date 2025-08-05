// Copyright 2023 RealSense, Inc. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// DESCRIPTION: #
// ------------ #
// This tool created a node which can be used to calulate the specified topic's latency.
// Input parameters: 
//    - topic_name : <String>
//          - topic to which latency need to be calculated 
//    - topic_type : <String>
//          - Message type of the topic.
//          - Valid inputs: {'image','points','imu','metadata','camera_info','rgbd','imu_info','tf'}
// Note: 
//    - This tool doesn't support calulating latency for extrinsic topics.
//      Because, those topics doesn't have timestamp in it and this tool uses
//      that timestamp as an input to calculate the latency.
//

#include <sstream>
#include <string>
#include <frame_latency/frame_latency.h>
#include <constants.h>

using namespace rs2_ros::tools::frame_latency;

FrameLatencyNode::FrameLatencyNode( const std::string & node_name,
                                    const std::string & ns,
                                    const rclcpp::NodeOptions & node_options )
    : Node( node_name, ns, node_options )
    , _logger( this->get_logger() )
{
}

std::string topic_name = "/camera/color/image_raw";
std::string topic_type = "image";

template <typename MsgType>
void FrameLatencyNode::createListener(std::string topicName, const rmw_qos_profile_t qos_profile)
{
    _sub = this->create_subscription<MsgType>(
                topicName,
                rclcpp::QoS( rclcpp::QoSInitialization::from_rmw( qos_profile ),
                            qos_profile ),
                [&, this]( const std::shared_ptr< MsgType> msg ) {
                    rclcpp::Time curr_time = this->get_clock()->now();
                    auto latency = ( curr_time - msg->header.stamp ).seconds();
                    ROS_INFO_STREAM( "Got msg with "<< msg->header.frame_id <<" frame id at address 0x"
                                    << std::hex << reinterpret_cast< std::uintptr_t >( msg.get() )
                                    << std::dec << " with latency of " << latency << " [sec]" );
                } );
}

void FrameLatencyNode::createTFListener(std::string topicName, const rmw_qos_profile_t qos_profile)
{
    _sub = this->create_subscription<tf2_msgs::msg::TFMessage>(
                topicName,
                rclcpp::QoS( rclcpp::QoSInitialization::from_rmw( qos_profile ),
                            qos_profile ),
                [&, this]( const std::shared_ptr<tf2_msgs::msg::TFMessage> msg ) {
                    rclcpp::Time curr_time = this->get_clock()->now();
                    auto latency = ( curr_time - msg->transforms.back().header.stamp ).seconds();
                    ROS_INFO_STREAM( "Got msg with "<< msg->transforms.back().header.frame_id <<" frame id at address 0x"
                                    << std::hex << reinterpret_cast< std::uintptr_t >( msg.get() )
                                    << std::dec << " with latency of " << latency << " [sec]" );
                } );
}

FrameLatencyNode::FrameLatencyNode( const rclcpp::NodeOptions & node_options )
    : Node( "frame_latency", "/", node_options )
    , _logger( this->get_logger() )
{
    ROS_INFO_STREAM( "frame_latency node is UP!" );
    ROS_INFO_STREAM( "Intra-Process is "
                     << ( this->get_node_options().use_intra_process_comms() ? "ON" : "OFF" ) );

    topic_name = this->declare_parameter("topic_name", topic_name);
    topic_type = this->declare_parameter("topic_type", topic_type);

    ROS_INFO_STREAM( "Subscribing to Topic: " << topic_name);

    if (topic_type == "image")
        createListener<sensor_msgs::msg::Image>(topic_name, rmw_qos_profile_default);
    else if (topic_type == "points")
        createListener<sensor_msgs::msg::PointCloud2>(topic_name, rmw_qos_profile_default);
    else if (topic_type == "imu")
        createListener<sensor_msgs::msg::Imu>(topic_name, rmw_qos_profile_sensor_data);
    else if (topic_type == "metadata")
        createListener<realsense2_camera_msgs::msg::Metadata>(topic_name, rmw_qos_profile_default);
    else if (topic_type == "camera_info")
        createListener<sensor_msgs::msg::CameraInfo>(topic_name, rmw_qos_profile_default);
    else if (topic_type == "rgbd")
        createListener<realsense2_camera_msgs::msg::RGBD>(topic_name, rmw_qos_profile_default);
    else if (topic_type == "imu_info")
        createListener<realsense2_camera_msgs::msg::IMUInfo>(topic_name, rmw_qos_profile_default);
    else if (topic_type == "tf")
        createTFListener(topic_name, rmw_qos_profile_default);
    else
        ROS_ERROR_STREAM("Specified message type '" << topic_type << "' is not supported");
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE( rs2_ros::tools::frame_latency::FrameLatencyNode )
