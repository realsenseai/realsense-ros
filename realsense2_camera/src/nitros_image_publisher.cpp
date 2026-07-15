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

#ifdef BUILD_WITH_NITROS

#include "nitros_image_publisher.h"

#include <isaac_ros_managed_nitros/managed_nitros_publisher.hpp>
#include <isaac_ros_nitros_image_type/nitros_image.hpp>
#include <isaac_ros_nitros_image_type/nitros_image_builder.hpp>

#include <cuda_runtime.h>
#include <utility>

namespace realsense2_camera
{
using nvidia::isaac_ros::nitros::ManagedNitrosPublisher;
using nvidia::isaac_ros::nitros::NitrosImage;
using nvidia::isaac_ros::nitros::NitrosImageBuilder;

struct NitrosImagePublisher::Impl
{
    std::shared_ptr<ManagedNitrosPublisher<NitrosImage>> pub;
};

NitrosImagePublisher::NitrosImagePublisher(
    rclcpp::Node * node, const std::string & topic, const std::string & nitros_format)
: _impl(std::make_unique<Impl>())
{
    // ManagedNitrosPublisher performs REP-2007/2009 type negotiation internally; we only
    // choose the compatible data format (e.g. "nitros_image_rgb8").
    _impl->pub = std::make_shared<ManagedNitrosPublisher<NitrosImage>>(node, topic, nitros_format);
}

NitrosImagePublisher::~NitrosImagePublisher() = default;

void NitrosImagePublisher::publish(
    const void * gpu_src,
    uint32_t width,
    uint32_t height,
    size_t size_bytes,
    const std::string & encoding,
    const std_msgs::msg::Header & header)
{
    // GXF will cudaFree() this buffer once downstream is done, so it must be a fresh cudaMalloc.
    void * dev = nullptr;
    cudaError_t err = cudaMalloc(&dev, size_bytes);
    if (err != cudaSuccess) {
        RCLCPP_WARN(
            rclcpp::get_logger("NitrosImagePublisher"),
            "cudaMalloc(%zu) failed: %s; dropping NITROS frame", size_bytes, cudaGetErrorString(err));
        return;
    }
    // Synchronous device-to-device copy from the frame's GPU pixels (cudaMemcpyDefault lets UVA
    // resolve the source, which may be device-mapped pinned host memory on a zero-copy build).
    err = cudaMemcpy(dev, gpu_src, size_bytes, cudaMemcpyDefault);
    if (err != cudaSuccess) {
        RCLCPP_WARN(
            rclcpp::get_logger("NitrosImagePublisher"),
            "cudaMemcpy(%zu) failed: %s; dropping NITROS frame", size_bytes, cudaGetErrorString(err));
        cudaFree(dev);
        return;
    }

    try {
        NitrosImage img = NitrosImageBuilder()
            .WithHeader(header)
            .WithEncoding(encoding)
            .WithDimensions(height, width)
            .WithGpuData(dev)   // ownership transfers to GXF (frees via cudaFree on release)
            .Build();
        _impl->pub->publish(std::move(img));
    } catch (const std::exception & e) {
        // Build() throws (e.g. odd dimensions / unsupported encoding) before taking ownership.
        cudaFree(dev);
        RCLCPP_WARN(rclcpp::get_logger("NitrosImagePublisher"), "NitrosImage Build failed: %s", e.what());
    }
}
}  // namespace realsense2_camera

#endif  // BUILD_WITH_NITROS
