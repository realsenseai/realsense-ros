// Look at the actual GPU frame the wrapper published, without putting the copy we removed back
// into the pipeline.
//
// This node must be COMPOSED into the camera's container. A viewer in another process would force
// NITROS's type adapter to copy every frame device->host (measured: publisher cost 400 us -> 4400 us
// per frame), which is precisely what the zero-copy path exists to avoid.
//
// Instead it reads the GPU buffer in place, downscales it on the GPU, and copies back only a small
// thumbnail, at a throttled rate. At the defaults (480x270, 2 Hz) that is 57 KB twice a second
// against 6.22 MB thirty times a second - about 0.06% of the frame bytes.
#include "bench_kernel.h"
#include "viz_kernel.h"

#include <isaac_ros_managed_nitros/managed_nitros_subscriber.hpp>
#include <isaac_ros_nitros_image_type/nitros_image.hpp>
#include <isaac_ros_nitros_image_type/nitros_image_view.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <cstdio>
#include <memory>
#include <string>
#include <vector>

namespace nitros_zc_bench
{
using nvidia::isaac_ros::nitros::ManagedNitrosSubscriber;
using nvidia::isaac_ros::nitros::NitrosImageView;

class NitrosViewer : public rclcpp::Node
{
public:
    explicit NitrosViewer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : rclcpp::Node("nitros_viewer", options),
      _topic(declare_parameter<std::string>("topic", "/camera/camera/color/nitros_image")),
      _format(declare_parameter<std::string>("format", "nitros_image_rgb8")),
      _out_topic(declare_parameter<std::string>("out_topic", "~/gpu_view")),
      _view_width(declare_parameter<int>("view_width", 480)),
      _view_height(declare_parameter<int>("view_height", 270)),
      _rate_hz(declare_parameter<double>("rate_hz", 2.0)),
      _full_res(declare_parameter<bool>("full_res", false)),
      _snapshot_path(declare_parameter<std::string>("snapshot_path", ""))
    {
        cudaSetDevice(0);
        cudaStreamCreateWithFlags(&_stream, cudaStreamNonBlocking);
        _pub = create_publisher<sensor_msgs::msg::Image>(_out_topic, rclcpp::QoS(1));
        _sub = std::make_shared<ManagedNitrosSubscriber<NitrosImageView>>(
            this, _topic, _format, [this](const NitrosImageView & view) {onImage(view);});
        RCLCPP_INFO(
            get_logger(),
            "viewing %s -> %s at %.1f Hz, %s (this node must be composed in the camera's container)",
            _topic.c_str(), _pub->get_topic_name(), _rate_hz,
            _full_res ? "full resolution" : "GPU-downscaled thumbnail");
    }

    ~NitrosViewer() override
    {
        if (_d_thumb) {
            cudaFree(_d_thumb);
        }
        if (_stream) {
            cudaStreamDestroy(_stream);
        }
    }

private:
    void onImage(const NitrosImageView & view)
    {
        const auto now_s = now().seconds();
        if (_rate_hz <= 0.0 || (now_s - _last_published) < (1.0 / _rate_hz)) {
            return;   // throttled: most frames are looked at by nobody and cost nothing
        }
        _last_published = now_s;

        const void * gpu = view.GetGpuData();
        const uint32_t src_w = view.GetWidth(), src_h = view.GetHeight();
        if (!gpu || src_w == 0 || src_h == 0) {
            return;
        }
        const uint32_t out_w = _full_res ? src_w : static_cast<uint32_t>(_view_width);
        const uint32_t out_h = _full_res ? src_h : static_cast<uint32_t>(_view_height);
        const size_t out_bytes = static_cast<size_t>(out_w) * out_h * 3;

        sensor_msgs::msg::Image msg;
        msg.header.frame_id = view.GetFrameId();
        msg.header.stamp.sec = static_cast<int32_t>(view.GetTimestampSeconds());
        msg.header.stamp.nanosec = view.GetTimestampNanoseconds();
        msg.width = out_w;
        msg.height = out_h;
        msg.encoding = view.GetEncoding();
        msg.step = out_w * 3;
        msg.data.resize(out_bytes);

        cudaError_t err = cudaSuccess;
        if (_full_res) {
            // Straight device->host of the whole frame. Only for eyeballing; do not benchmark with it.
            err = cudaMemcpyAsync(
                msg.data.data(), gpu, out_bytes, cudaMemcpyDeviceToHost, _stream);
        } else {
            if (out_bytes != _thumb_bytes) {
                if (_d_thumb) {
                    cudaFree(_d_thumb);
                }
                if (cudaMalloc(&_d_thumb, out_bytes) != cudaSuccess) {
                    RCLCPP_ERROR(get_logger(), "cudaMalloc(%zu) for the thumbnail failed", out_bytes);
                    _d_thumb = nullptr;
                    return;
                }
                _thumb_bytes = out_bytes;
            }
            // Shrink on the GPU first, then copy back only the thumbnail.
            err = launch_downscale_rgb8(gpu, src_w, src_h, _d_thumb, out_w, out_h, _stream);
            if (err == cudaSuccess) {
                err = cudaMemcpyAsync(
                    msg.data.data(), _d_thumb, out_bytes, cudaMemcpyDeviceToHost, _stream);
            }
        }
        if (err == cudaSuccess) {
            err = cudaStreamSynchronize(_stream);
        }
        if (err != cudaSuccess) {
            RCLCPP_ERROR(get_logger(), "GPU view failed: %s", cudaGetErrorString(err));
            return;
        }

        if (!_snapshot_path.empty() && !_snapshot_written) {
            writeSnapshot(msg, out_w, out_h);
            _snapshot_written = true;
        }
        _pub->publish(std::move(msg));
        ++_published;
    }

    // A one-off PPM of the GPU pixels, for verifying content without a display attached.
    void writeSnapshot(const sensor_msgs::msg::Image & msg, uint32_t w, uint32_t h)
    {
        std::FILE * f = std::fopen(_snapshot_path.c_str(), "wb");
        if (!f) {
            RCLCPP_WARN(get_logger(), "cannot write snapshot to %s", _snapshot_path.c_str());
            return;
        }
        std::fprintf(f, "P6\n%u %u\n255\n", w, h);
        std::fwrite(msg.data.data(), 1, msg.data.size(), f);
        std::fclose(f);
        RCLCPP_INFO(
            get_logger(), "wrote %ux%u snapshot of the GPU frame to %s", w, h, _snapshot_path.c_str());
    }

    std::string _topic, _format, _out_topic;
    int _view_width, _view_height;
    double _rate_hz;
    bool _full_res;
    std::string _snapshot_path;
    bool _snapshot_written{false};
    double _last_published{0.0};
    uint64_t _published{0};
    std::shared_ptr<ManagedNitrosSubscriber<NitrosImageView>> _sub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _pub;
    cudaStream_t _stream{nullptr};
    void * _d_thumb{nullptr};
    size_t _thumb_bytes{0};
};
}  // namespace nitros_zc_bench

#ifdef NITROS_ZC_BENCH_COMPONENT_ONLY
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(nitros_zc_bench::NitrosViewer)
#endif
