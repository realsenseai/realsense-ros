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
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdlib>
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
rclcpp::Logger logger()
{
    return rclcpp::get_logger("NitrosImagePublisher");
}

// Set RS_NITROS_LEGACY_ALLOC=1 to get a plain cudaMalloc + synchronous cudaMemcpy per frame instead
// of the pooled + async path. Only useful for A/B benchmarking the two allocation strategies.
bool legacyAllocRequested()
{
    const char * env = std::getenv("RS_NITROS_LEGACY_ALLOC");
    return env != nullptr && env[0] == '1';
}

// How often the accumulated per-frame publish cost is reported (at DEBUG).
constexpr uint64_t STATS_LOG_INTERVAL = 100;
}  // namespace

struct NitrosImagePublisher::Impl
{
    std::shared_ptr<ManagedNitrosPublisher<NitrosImage>> pub;

    // Stream-ordered pool the GXF-owned buffers are carved from; null when pooling is unavailable
    // or RS_NITROS_LEGACY_ALLOC=1, in which case publish() falls back to cudaMalloc.
#if RS_NITROS_HAS_MEMPOOL
    cudaMemPool_t pool{nullptr};
#endif
    cudaStream_t stream{nullptr};
    cudaEvent_t copy_done{nullptr};

    // Per-frame publish cost, reported every STATS_LOG_INTERVAL frames.
    uint64_t frames{0};
    double us_sum{0.0};
    double us_max{0.0};
    double alloc_us_sum{0.0};
    double wait_us_sum{0.0};
    double build_us_sum{0.0};

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
        if (dev) {
            cudaFree(dev);
        }
    }
};

NitrosImagePublisher::NitrosImagePublisher(
    rclcpp::Node * node, const std::string & topic, const std::string & nitros_format)
: _impl(std::make_unique<Impl>())
{
    // ManagedNitrosPublisher performs REP-2007/2009 type negotiation internally; we only
    // choose the compatible data format (e.g. "nitros_image_rgb8").
    _impl->pub = std::make_shared<ManagedNitrosPublisher<NitrosImage>>(node, topic, nitros_format);

    // A non-blocking stream keeps our copy out of the legacy default stream, which librealsense's
    // own CUDA work (alignment, colorizer) uses. It is created at the highest priority the device
    // offers: the frame cannot be published until this copy lands, so a short copy waiting behind a
    // consumer's long-running kernels directly delays the whole graph.
    int lowest_priority = 0, highest_priority = 0;
    cudaError_t err = cudaDeviceGetStreamPriorityRange(&lowest_priority, &highest_priority);
    if (err != cudaSuccess) {
        highest_priority = 0;
    }
    err = cudaStreamCreateWithPriority(&_impl->stream, cudaStreamNonBlocking, highest_priority);
    if (err != cudaSuccess) {
        RCLCPP_WARN(
            logger(), "cudaStreamCreateWithPriority failed: %s; falling back to the default stream",
            cudaGetErrorString(err));
        _impl->stream = nullptr;
    }
    err = cudaEventCreateWithFlags(&_impl->copy_done, cudaEventDisableTiming);
    if (err != cudaSuccess) {
        RCLCPP_WARN(
            logger(), "cudaEventCreateWithFlags failed: %s; will synchronize on the stream instead",
            cudaGetErrorString(err));
        _impl->copy_done = nullptr;
    }

#if RS_NITROS_HAS_MEMPOOL
    if (legacyAllocRequested()) {
        RCLCPP_INFO(logger(), "RS_NITROS_LEGACY_ALLOC=1: using cudaMalloc/cudaFree per frame");
    } else {
        cudaMemPoolProps props{};
        props.allocType = cudaMemAllocationTypePinned;
        props.handleTypes = cudaMemHandleTypeNone;
        props.location.type = cudaMemLocationTypeDevice;
        props.location.id = 0;
        err = cudaMemPoolCreate(&_impl->pool, &props);
        if (err != cudaSuccess) {
            RCLCPP_WARN(
                logger(), "cudaMemPoolCreate failed: %s; falling back to cudaMalloc per frame",
                cudaGetErrorString(err));
            _impl->pool = nullptr;
        } else {
            // Never hand pages back to the driver: the pool should keep serving the same few
            // frame-sized blocks for the lifetime of the stream.
            uint64_t release_threshold = UINT64_MAX;
            err = cudaMemPoolSetAttribute(
                _impl->pool, cudaMemPoolAttrReleaseThreshold, &release_threshold);
            if (err != cudaSuccess) {
                RCLCPP_WARN(
                    logger(), "cudaMemPoolSetAttribute(ReleaseThreshold) failed: %s",
                    cudaGetErrorString(err));
            }
        }
    }
#else
    RCLCPP_INFO(logger(), "CUDA < 11.2: using cudaMalloc/cudaFree per frame");
#endif
}

NitrosImagePublisher::~NitrosImagePublisher()
{
    if (_impl->stream) {
        cudaStreamSynchronize(_impl->stream);
    }
    if (_impl->copy_done) {
        cudaEventDestroy(_impl->copy_done);
    }
    if (_impl->stream) {
        cudaStreamDestroy(_impl->stream);
    }
#if RS_NITROS_HAS_MEMPOOL
    if (_impl->pool) {
        // Safe with buffers still in flight: the pool's resources are released once the last
        // outstanding allocation is freed.
        cudaMemPoolDestroy(_impl->pool);
    }
#endif
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
    auto t_alloc = t_start, t_wait = t_start;

    // GXF will cudaFree() this buffer once downstream is done with it, so it must be a buffer we
    // own rather than the SDK's frame-pool memory.
    void * dev = nullptr;
    cudaError_t err = cudaSuccess;
#if RS_NITROS_HAS_MEMPOOL
    if (_impl->pooled()) {
        err = cudaMallocFromPoolAsync(&dev, size_bytes, _impl->pool, _impl->stream);
    } else {
        err = cudaMalloc(&dev, size_bytes);
    }
#else
    err = cudaMalloc(&dev, size_bytes);
#endif
    if (err != cudaSuccess) {
        RCLCPP_WARN(
            logger(), "device allocation of %zu bytes failed: %s; dropping NITROS frame",
            size_bytes, cudaGetErrorString(err));
        return;
    }

    t_alloc = std::chrono::steady_clock::now();

    // Device-to-device copy from the frame's GPU pixels (cudaMemcpyDefault lets UVA resolve the
    // source, which is device-mapped pinned host memory on a zero-copy build).
    err = cudaMemcpyAsync(dev, gpu_src, size_bytes, cudaMemcpyDefault, _impl->stream);
    if (err != cudaSuccess) {
        RCLCPP_WARN(
            logger(), "cudaMemcpyAsync(%zu) failed: %s; dropping NITROS frame",
            size_bytes, cudaGetErrorString(err));
        _impl->freeUnpublished(dev);
        return;
    }

    // release-3.2's NitrosImage carries no CUDA event, so a downstream process has no way to order
    // its kernels after our copy: the copy has to be complete before the message goes out.
    if (_impl->copy_done) {
        err = cudaEventRecord(_impl->copy_done, _impl->stream);
        if (err == cudaSuccess) {
            err = cudaEventSynchronize(_impl->copy_done);
        }
    } else {
        err = cudaStreamSynchronize(_impl->stream);
    }
    if (err != cudaSuccess) {
        RCLCPP_WARN(
            logger(), "waiting for the D2D copy failed: %s; dropping NITROS frame",
            cudaGetErrorString(err));
        _impl->freeUnpublished(dev);
        return;
    }

    t_wait = std::chrono::steady_clock::now();

    std::optional<NitrosImage> img;
    try {
        img.emplace(
            NitrosImageBuilder()
            .WithHeader(header)
            .WithEncoding(encoding)
            .WithDimensions(height, width)
            .WithGpuData(dev)   // ownership transfers to GXF (frees via cudaFree on release)
            .Build());
    } catch (const std::exception & e) {
        // Build() throws (e.g. odd dimensions / unsupported encoding) before taking ownership, so
        // the buffer is still ours to free. Note the catch deliberately covers Build() only: once
        // Build() has returned, GXF owns the buffer and freeing it here would be a double free.
        _impl->freeUnpublished(dev);
        RCLCPP_WARN(logger(), "NitrosImage Build failed: %s", e.what());
        return;
    }
    _impl->pub->publish(std::move(*img));

    const auto t_end = std::chrono::steady_clock::now();
    const double us = std::chrono::duration<double, std::micro>(t_end - t_start).count();
    _impl->us_sum += us;
    _impl->us_max = std::max(_impl->us_max, us);
    _impl->alloc_us_sum += std::chrono::duration<double, std::micro>(t_alloc - t_start).count();
    _impl->wait_us_sum += std::chrono::duration<double, std::micro>(t_wait - t_alloc).count();
    _impl->build_us_sum += std::chrono::duration<double, std::micro>(t_end - t_wait).count();
    if (++_impl->frames % STATS_LOG_INTERVAL == 0) {
        const double n = static_cast<double>(STATS_LOG_INTERVAL);
        // How much memory the pool is holding tells us whether GXF is releasing our buffers
        // promptly: reserved should settle at a few frames' worth, not keep climbing.
        double reserved_mb = 0.0, used_mb = 0.0;
#if RS_NITROS_HAS_MEMPOOL
        if (_impl->pool) {
            uint64_t reserved = 0, used = 0;
            cudaMemPoolGetAttribute(_impl->pool, cudaMemPoolAttrReservedMemCurrent, &reserved);
            cudaMemPoolGetAttribute(_impl->pool, cudaMemPoolAttrUsedMemCurrent, &used);
            reserved_mb = static_cast<double>(reserved) / (1024.0 * 1024.0);
            used_mb = static_cast<double>(used) / (1024.0 * 1024.0);
        }
#endif
        RCLCPP_DEBUG(
            logger(),
            "%s publish cost over %.0f frames: mean %.1f us (alloc %.1f, copy-wait %.1f, build %.1f), "
            "max %.1f us; pool reserved %.1f MB, in use %.1f MB",
            _impl->pooled() ? "pooled" : "cudaMalloc", n, _impl->us_sum / n,
            _impl->alloc_us_sum / n, _impl->wait_us_sum / n, _impl->build_us_sum / n,
            _impl->us_max, reserved_mb, used_mb);
        _impl->us_sum = 0.0;
        _impl->us_max = 0.0;
        _impl->alloc_us_sum = 0.0;
        _impl->wait_us_sum = 0.0;
        _impl->build_us_sum = 0.0;
    }
}
}  // namespace realsense2_camera

#endif  // BUILD_WITH_NITROS
