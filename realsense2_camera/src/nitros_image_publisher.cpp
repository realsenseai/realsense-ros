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
#include <chrono>
#include <cstdint>
#include <exception>
#include <optional>
#include <utility>

// cudaMemPool_t / cudaMallocFromPoolAsync (the stream-ordered allocator) need CUDA 11.2.
#if defined(CUDART_VERSION) && CUDART_VERSION >= 11020
#define RS_NITROS_HAS_MEMPOOL 1
#else
#define RS_NITROS_HAS_MEMPOOL 0
#endif

namespace realsense2_camera
{
using nvidia::isaac_ros::nitros::ManagedNitrosPublisher;
using nvidia::isaac_ros::nitros::NitrosImage;
using nvidia::isaac_ros::nitros::NitrosImageBuilder;

namespace
{
// How many publish attempts to wait between graph queries for subscribers (~0.3 s at 30 FPS).
constexpr uint64_t SUBSCRIBER_CHECK_INTERVAL = 10;
// Frames between reports of the accumulated publish cost (~3 s at 30 FPS).
constexpr uint64_t STATS_LOG_INTERVAL = 100;
// Above this mean publish cost, the frame is being copied device->host for a subscriber in another
// process; zero copy is not happening and the user needs to know.
constexpr double CROSS_PROCESS_COST_US = 1000.0;
constexpr int WARN_THROTTLE_MS = 10000;
}  // namespace

struct NitrosImagePublisher::Impl
{
    explicit Impl(rclcpp::Node * node_ptr)
    : node(node_ptr), logger(node_ptr->get_logger().get_child("nitros")), clock(RCL_STEADY_TIME) {}

    rclcpp::Node * node;
    rclcpp::Logger logger;
    rclcpp::Clock clock;
    std::string topic_name;
    std::shared_ptr<ManagedNitrosPublisher<NitrosImage>> pub;

    // Stream-ordered pool the GXF-owned buffers are carved from; null if the pool could not be
    // created, in which case publish() falls back to cudaMalloc.
#if RS_NITROS_HAS_MEMPOOL
    cudaMemPool_t pool{nullptr};
#endif
    cudaStream_t stream{nullptr};
    cudaEvent_t copy_done{nullptr};

    bool upload_warning_pending{true};

    // Cached subscriber state; the counter starts due so the first call queries the graph.
    bool has_subscribers{false};
    uint64_t subscriber_check_counter{SUBSCRIBER_CHECK_INTERVAL};

    // Publish cost accounting, reported every STATS_LOG_INTERVAL frames.
    uint64_t frames{0};
    double us_sum{0.0};
    bool cross_process_warned{false};

    bool pooled() const
    {
#if RS_NITROS_HAS_MEMPOOL
        return pool != nullptr;
#else
        return false;
#endif
    }

    // Frees a buffer publish() allocated but never handed to GXF. cudaFree is correct for both
    // allocation strategies: it is legal on stream-ordered pool memory and returns it to the pool.
    void freeUnpublished(void * dev) const
    {
        if (dev)
        {
            cudaFree(dev);
        }
    }
};

NitrosImagePublisher::NitrosImagePublisher(
    rclcpp::Node * node, const std::string & topic, const std::string & nitros_format)
: _impl(std::make_unique<Impl>(node))
{
    // ManagedNitrosPublisher performs REP-2007/2009 type negotiation internally; we only choose the
    // compatible data format (e.g. "nitros_image_rgb8").
    _impl->pub = std::make_shared<ManagedNitrosPublisher<NitrosImage>>(node, topic, nitros_format);
    _impl->topic_name =
        node->get_node_base_interface()->resolve_topic_or_service_name(topic, false);

    // A private non-blocking stream keeps our copy out of the legacy default stream, which
    // librealsense's own CUDA work uses. Highest available priority, because the frame cannot be
    // published until this copy lands: a short copy waiting behind a consumer's long kernels delays
    // the whole graph.
    int lowest_priority = 0;
    int highest_priority = 0;
    cudaError_t err = cudaDeviceGetStreamPriorityRange(&lowest_priority, &highest_priority);
    if (err != cudaSuccess)
    {
        highest_priority = 0;
    }
    err = cudaStreamCreateWithPriority(&_impl->stream, cudaStreamNonBlocking, highest_priority);
    if (err != cudaSuccess)
    {
        RCLCPP_WARN(_impl->logger,
                    "cudaStreamCreateWithPriority failed: %s; falling back to the default stream",
                    cudaGetErrorString(err));
        _impl->stream = nullptr;
    }
    err = cudaEventCreateWithFlags(&_impl->copy_done, cudaEventDisableTiming);
    if (err != cudaSuccess)
    {
        RCLCPP_WARN(_impl->logger,
                    "cudaEventCreateWithFlags failed: %s; will synchronize on the stream instead",
                    cudaGetErrorString(err));
        _impl->copy_done = nullptr;
    }

#if RS_NITROS_HAS_MEMPOOL
    // Allocate from the device this node is actually using rather than assuming device 0.
    int cuda_device = 0;
    if (cudaGetDevice(&cuda_device) != cudaSuccess)
    {
        cuda_device = 0;
    }
    cudaMemPoolProps props{};
    props.allocType = cudaMemAllocationTypePinned;
    props.handleTypes = cudaMemHandleTypeNone;
    props.location.type = cudaMemLocationTypeDevice;
    props.location.id = cuda_device;
    err = cudaMemPoolCreate(&_impl->pool, &props);
    if (err != cudaSuccess)
    {
        RCLCPP_WARN(_impl->logger, "cudaMemPoolCreate failed: %s; falling back to cudaMalloc per frame",
                    cudaGetErrorString(err));
        _impl->pool = nullptr;
    }
    else
    {
        // Never hand pages back to the driver: the pool should keep serving the same few
        // frame-sized blocks for the lifetime of the stream.
        uint64_t release_threshold = UINT64_MAX;
        err = cudaMemPoolSetAttribute(
            _impl->pool, cudaMemPoolAttrReleaseThreshold, &release_threshold);
        if (err != cudaSuccess)
        {
            RCLCPP_WARN(_impl->logger, "cudaMemPoolSetAttribute(ReleaseThreshold) failed: %s",
                        cudaGetErrorString(err));
        }
    }
#else
    RCLCPP_INFO(_impl->logger, "CUDA < 11.2: using cudaMalloc/cudaFree per frame");
#endif
}

NitrosImagePublisher::~NitrosImagePublisher()
{
    if (_impl->stream)
    {
        cudaStreamSynchronize(_impl->stream);
    }
    if (_impl->copy_done)
    {
        cudaEventDestroy(_impl->copy_done);
    }
    if (_impl->stream)
    {
        cudaStreamDestroy(_impl->stream);
    }
#if RS_NITROS_HAS_MEMPOOL
    if (_impl->pool)
    {
        // Safe with buffers still in flight: the pool's resources are released once the last
        // outstanding allocation is freed.
        cudaMemPoolDestroy(_impl->pool);
    }
#endif
}

const std::string & NitrosImagePublisher::getTopicName() const
{
    return _impl->topic_name;
}

bool NitrosImagePublisher::takeUploadWarning()
{
    if (!_impl->upload_warning_pending)
    {
        return false;
    }
    _impl->upload_warning_pending = false;
    return true;
}

bool NitrosImagePublisher::hasSubscribers()
{
    // ManagedNitrosPublisher exposes no subscription count, so ask the graph instead.
    if (++_impl->subscriber_check_counter >= SUBSCRIBER_CHECK_INTERVAL)
    {
        _impl->subscriber_check_counter = 0;
        _impl->has_subscribers = _impl->node->count_subscribers(_impl->topic_name) > 0;
    }
    return _impl->has_subscribers;
}

void NitrosImagePublisher::publish(
    const void * gpu_src,
    uint32_t width,
    uint32_t height,
    size_t size_bytes,
    const std::string & encoding,
    const std_msgs::msg::Header & header)
{
    const auto t_start = std::chrono::steady_clock::now();

    // GXF will cudaFree() this buffer once downstream is done with it, so it must be a buffer we
    // own rather than the SDK's frame-pool memory.
    void * dev = nullptr;
    cudaError_t err = cudaSuccess;
#if RS_NITROS_HAS_MEMPOOL
    if (_impl->pooled())
    {
        err = cudaMallocFromPoolAsync(&dev, size_bytes, _impl->pool, _impl->stream);
    }
    else
    {
        err = cudaMalloc(&dev, size_bytes);
    }
#else
    err = cudaMalloc(&dev, size_bytes);
#endif
    if (err != cudaSuccess)
    {
        RCLCPP_WARN_THROTTLE(_impl->logger, _impl->clock, WARN_THROTTLE_MS,
                             "device allocation of %zu bytes failed: %s; dropping NITROS frame",
                             size_bytes, cudaGetErrorString(err));
        return;
    }

    // Device-to-device copy from the frame's GPU pixels. This is an ownership handoff, not a format
    // conversion: cudaMemcpyDefault lets UVA resolve the source, which is device-mapped pinned host
    // memory on a zero-copy build.
    err = cudaMemcpyAsync(dev, gpu_src, size_bytes, cudaMemcpyDefault, _impl->stream);
    if (err != cudaSuccess)
    {
        RCLCPP_WARN_THROTTLE(_impl->logger, _impl->clock, WARN_THROTTLE_MS,
                             "cudaMemcpyAsync(%zu) failed: %s; dropping NITROS frame",
                             size_bytes, cudaGetErrorString(err));
        _impl->freeUnpublished(dev);
        return;
    }

    // release-3.2's NitrosImage carries no CUDA event, so a downstream process has no way to order
    // its kernels after our copy: the copy has to be complete before the message goes out.
    if (_impl->copy_done)
    {
        err = cudaEventRecord(_impl->copy_done, _impl->stream);
        if (err == cudaSuccess)
        {
            err = cudaEventSynchronize(_impl->copy_done);
        }
    }
    else
    {
        err = cudaStreamSynchronize(_impl->stream);
    }
    if (err != cudaSuccess)
    {
        RCLCPP_WARN_THROTTLE(_impl->logger, _impl->clock, WARN_THROTTLE_MS,
                             "waiting for the device-to-device copy failed: %s; dropping NITROS frame",
                             cudaGetErrorString(err));
        _impl->freeUnpublished(dev);
        return;
    }

    std::optional<NitrosImage> img;
    try
    {
        img.emplace(
            NitrosImageBuilder()
            .WithHeader(header)
            .WithEncoding(encoding)
            .WithDimensions(height, width)
            .WithGpuData(dev)   // ownership transfers to GXF (frees via cudaFree on release)
            .Build());
    }
    catch (const std::exception & e)
    {
        // Build() throws (e.g. odd dimensions / unsupported encoding) before taking ownership, so
        // the buffer is still ours to free. The catch deliberately covers Build() only: once it has
        // returned, GXF owns the buffer and freeing it here would be a double free.
        _impl->freeUnpublished(dev);
        RCLCPP_WARN_THROTTLE(_impl->logger, _impl->clock, WARN_THROTTLE_MS,
                             "NitrosImage Build failed: %s", e.what());
        return;
    }
    _impl->pub->publish(std::move(*img));

    _impl->us_sum += std::chrono::duration<double, std::micro>(
        std::chrono::steady_clock::now() - t_start).count();
    if (++_impl->frames % STATS_LOG_INTERVAL == 0)
    {
        const double mean_us = _impl->us_sum / static_cast<double>(STATS_LOG_INTERVAL);
        _impl->us_sum = 0.0;
        RCLCPP_DEBUG(_impl->logger, "%s publish cost: mean %.1f us/frame over %llu frames",
                     _impl->pooled() ? "pooled" : "cudaMalloc", mean_us,
                     static_cast<unsigned long long>(STATS_LOG_INTERVAL));
        if (mean_us > CROSS_PROCESS_COST_US && !_impl->cross_process_warned)
        {
            _impl->cross_process_warned = true;
            RCLCPP_WARN(_impl->logger,
                        "NITROS publish costs %.1f ms/frame on %s, which means a subscriber in "
                        "another process is forcing a device-to-host copy of every frame. Load the "
                        "consumer as a component in this node's container to get the zero-copy path.",
                        mean_us / 1000.0, _impl->topic_name.c_str());
        }
    }
}
}  // namespace realsense2_camera

#endif  // BUILD_WITH_NITROS
