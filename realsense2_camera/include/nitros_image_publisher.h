// Copyright 2026 RealSense, Inc. All Rights Reserved.
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

#pragma once

#ifdef BUILD_WITH_NITROS

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <functional>
#include <memory>
#include <string>

namespace realsense2_camera
{
// Thin, GXF-free wrapper around nvidia::isaac_ros::nitros::ManagedNitrosPublisher<NitrosImage>.
// The heavy Isaac ROS / GXF / CUDA headers are confined to nitros_image_publisher.cpp, so the
// rest of the wrapper (base_realsense_node.*) only sees this handle and never pulls in NITROS.
//
// NOTE on ownership (Isaac ROS release-3.2): NitrosImageBuilder::WithGpuData() hands the pointer
// to a GXF VideoBuffer whose release callback calls cudaFree(). GXF therefore OWNS the buffer and
// frees it once downstream is done. We must NOT pass the librealsense frame-pool pointer directly
// (GXF would cudaFree memory the SDK owns). Instead publish() cudaMalloc's a fresh device buffer,
// device-to-device copies the frame's GPU pixels into it, and hands that to GXF. This still avoids
// the GPU->CPU->GPU round-trip to the perception graph (the goal); the D2D copy is the cost of
// release-3.2's owning model (release-3.4+/main add WithReleaseCallback for a true alias).
class NitrosImagePublisher
{
public:
    // nitros_format is a NITROS supported-type name (e.g. "nitros_image_rgb8").
    NitrosImagePublisher(rclcpp::Node * node, const std::string & topic, const std::string & nitros_format);
    ~NitrosImagePublisher();

    // gpu_src   : CUDA device pointer to the frame pixels (aliases the SDK frame buffer).
    // size_bytes: number of bytes to copy (width * height * bytes_per_pixel).
    // encoding  : sensor_msgs::image_encodings string matching nitros_format (e.g. "rgb8").
    // The frame's pixels are D2D-copied into a GXF-owned buffer; the caller need not keep the
    // frame alive after this returns (the copy is synchronous).
    void publish(const void * gpu_src,
                 uint32_t width,
                 uint32_t height,
                 size_t size_bytes,
                 const std::string & encoding,
                 const std_msgs::msg::Header & header);

private:
    struct Impl;
    std::unique_ptr<Impl> _impl;
};
}  // namespace realsense2_camera

#endif  // BUILD_WITH_NITROS
