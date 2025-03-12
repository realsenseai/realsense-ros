// Copyright 2023 Intel Corporation. All Rights Reserved.
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

#include <image_publisher.h>

using namespace realsense2_camera;

// --- image_rcl_publisher implementation ---
image_rcl_publisher::image_rcl_publisher( rclcpp::Node & node,
                                          const std::string & topic_name,
                                          const rmw_qos_profile_t & qos )
{
    image_publisher_impl = node.create_publisher< sensor_msgs::msg::Image >(
        topic_name,
        rclcpp::QoS( rclcpp::QoSInitialization::from_rmw( qos ), qos ) );
}

void image_rcl_publisher::publish( sensor_msgs::msg::Image::UniquePtr image_ptr )
{
    std::cout << "image_rcl_publisher::publish" << std::endl;
    image_publisher_impl->publish( std::move( image_ptr ) );
}

size_t image_rcl_publisher::get_subscription_count() const
{
    return image_publisher_impl->get_subscription_count();
}

// --- image_transport_publisher implementation ---
image_transport_publisher::image_transport_publisher( rclcpp::Node & node,
                                                      const std::string & topic_name,
                                                      const rmw_qos_profile_t & qos )
{
    image_publisher_impl = std::make_shared< image_transport::Publisher >(
        image_transport::create_publisher( &node, topic_name, qos ) );
}
void image_transport_publisher::publish( sensor_msgs::msg::Image::UniquePtr image_ptr )
{
    bool can_read = true;
    if ( !img_shared_mem )
    {
        std::string topic = image_publisher_impl.get()->getTopic();
        std::cout << "topic: " << topic << std::endl;
        topic = topic.substr(1);  // Remove leading '/'
        std::istringstream stream(topic);
        std::string segment;
        std::vector<std::string> words;

        // Split the string by '/'
        while (std::getline(stream, segment, '/')) {
            words.push_back(segment);
        }
        std::string camera_location = words[1];
        std::string stream_type = words[2];
        auto msg = image_ptr.get();
        int img_size;
        if(msg->encoding == "mono8"){
            img_size = msg->width * msg->height * 1;
          }else if (msg->encoding == "bgr8"){
            img_size = msg->width * msg->height * 3;
          }else if (msg->encoding == "rgb8"){
            img_size = msg->width * msg->height * 3;
          }else if (msg->encoding == "rgba8"){
            img_size = msg->width * msg->height * 4;
          }else if (msg->encoding == "bgra8"){
            img_size = msg->width * msg->height * 4;
          }else if (msg->encoding == "mono16"){
            img_size = msg->width * msg->height * 2;
          }else if (msg->encoding == "16UC1"){
            img_size = msg->width * msg->height * 2;
          }
        std::string img_shm_name = stream_type + std::string("_") + camera_location;
        std::string rw_img_shm_name = img_shm_name + std::string("_rw");
        img_shared_mem = std::make_unique<SharedMem>(img_shm_name.c_str(), img_size, true);
        // rw_shared_mem = std::make_unique<SharedMem>(rw_img_shm_name.c_str(), sizeof(bool), true);
        
    }
    // can_read = !can_read;
    // memcpy(rw_shared_mem->data(), &can_read, sizeof(can_read)); // Signal that write is occuring
    memcpy(img_shared_mem->data(), image_ptr->data.data(), img_shared_mem->size());
    std::string shm_frame = std::string("--shm--") + img_shared_mem->name();
    image_ptr->header.frame_id = shm_frame;
    image_ptr->data.clear();
    image_publisher_impl->publish( *image_ptr );
    // can_read = !can_read;
    // memcpy(rw_shared_mem->data(), &can_read, sizeof(can_read)); // Signal that write is finished
}

size_t image_transport_publisher::get_subscription_count() const
{
    return image_publisher_impl->getNumSubscribers();
}
