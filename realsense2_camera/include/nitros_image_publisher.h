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
#include <memory>
#include <string>

namespace realsense2_camera
{
// Thin, GXF-free wrapper around nvidia::isaac_ros::nitros::ManagedNitrosPublisher<NitrosImage>.
// The heavy Isaac ROS / GXF / CUDA headers are confined to nitros_image_publisher.cpp, so the rest
// of the wrapper (base_realsense_node.*) only sees this handle and never pulls in NITROS.
//
// Zero copy only happens when the subscriber lives in the same process. Across a process boundary
// rclcpp has to run the NitrosImage type adapter, which copies the frame device->host, and the cost
// of that lands on this publisher; publish() detects it and says so once. See the "GPU zero-copy
// publishing (NITROS)" section of the README.
//
// Buffer ownership (Isaac ROS release-3.2): NitrosImageBuilder::WithGpuData() hands the pointer to a
// GXF VideoBuffer whose release callback calls cudaFree(). GXF therefore OWNS the buffer, so the
// librealsense frame-pool pointer must never be passed directly. publish() gives GXF a buffer from a
// CUDA stream-ordered memory pool instead and copies the pixels in device-to-device; cudaFree() is
// legal on pool memory, so GXF's release recycles the buffer rather than returning it to the driver.
// Isaac ROS >= 4.0 adds WithReleaseCallback(), which would remove the copy, but 4.x is Jetson Thor /
// JetPack 7 only.
//
// Not thread safe: one publisher instance serves one stream, and the wrapper only ever publishes a
// given stream from that stream's frame-callback thread.
class NitrosImagePublisher
{
public:
    // nitros_format is a NITROS supported-type name (e.g. "nitros_image_rgb8").
    NitrosImagePublisher(rclcpp::Node * node, const std::string & topic, const std::string & nitros_format);
    ~NitrosImagePublisher();

    // Fully-qualified topic name.
    const std::string & getTopicName() const;

    // Whether anything is subscribed. Publishing costs ~0.4 ms of GPU work per frame on the SDK
    // callback thread, so callers skip that entirely when nobody is listening. The graph is queried
    // at most once every few frames, which bounds how long a new subscriber waits for its first one.
    bool hasSubscribers();

    // Returns true the first time only, for a warning that should be reported once per stream.
    // Needs no locking: an instance serves one stream and is only used from that stream's thread.
    bool takeUploadWarning();

    // gpu_src   : CUDA device pointer to the frame pixels (aliases the SDK frame buffer).
    // size_bytes: number of bytes to copy (width * height * bytes_per_pixel).
    // encoding  : sensor_msgs::image_encodings string matching nitros_format (e.g. "rgb8").
    // The pixels are copied into a GXF-owned buffer and the copy is awaited before the message goes
    // out, so the caller need not keep the frame alive after this returns.
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
