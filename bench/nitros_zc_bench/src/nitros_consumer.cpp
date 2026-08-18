// Zero-copy consumer: negotiates NITROS with the wrapper and runs the same kernel straight on the
// frame's GPU buffer. No host-to-device copy happens anywhere in this process.
#include "bench_kernel.h"
#include "bench_stats.h"

#include <isaac_ros_managed_nitros/managed_nitros_subscriber.hpp>
#include <isaac_ros_nitros_image_type/nitros_image.hpp>
#include <isaac_ros_nitros_image_type/nitros_image_view.hpp>

#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <memory>
#include <string>

namespace nitros_zc_bench
{
using namespace std::chrono_literals;
using nvidia::isaac_ros::nitros::ManagedNitrosSubscriber;
using nvidia::isaac_ros::nitros::NitrosImageView;

class NitrosConsumer : public rclcpp::Node
{
public:
    explicit NitrosConsumer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : rclcpp::Node("nitros_consumer", options),
      _topic(declare_parameter<std::string>("topic", "/camera/camera/color/nitros_image")),
      _format(declare_parameter<std::string>("format", "nitros_image_rgb8")),
      _warmup_s(declare_parameter<double>("warmup_s", 5.0)),
      _duration_s(declare_parameter<double>("duration_s", 30.0)),
      _expected_fps(declare_parameter<double>("expected_fps", 30.0)),
      _csv_path(declare_parameter<std::string>("csv_path", ""))
    {
        _run.label = "NITROS zero-copy (frame already on the GPU)";
        cudaSetDevice(0);
        cudaStreamCreateWithFlags(&_stream, cudaStreamNonBlocking);
        cudaMalloc(reinterpret_cast<void **>(&_d_out), sizeof(unsigned long long));

        _sub = std::make_shared<ManagedNitrosSubscriber<NitrosImageView>>(
            this, _topic, _format,
            [this](const NitrosImageView & view) {onImage(view);});

        _run.openCsv(_csv_path);
        _start = now();
        RCLCPP_INFO(
            get_logger(), "negotiating %s on %s; %.0f s warmup then %.0f s measured",
            _format.c_str(), _topic.c_str(), _warmup_s, _duration_s);
        _timer = create_wall_timer(200ms, [this]() {checkDone();});
    }

    ~NitrosConsumer() override
    {
        reportIfMeasured();
        if (_d_out) {
            cudaFree(_d_out);
        }
        if (_stream) {
            cudaStreamDestroy(_stream);
        }
    }

    // Composed into a container there is no main() to call this, so the destructor does it.
    void reportIfMeasured()
    {
        if (_reported) {
            return;
        }
        _reported = true;
        if (_run.measuring) {
            _run.report(get_logger(), _expected_fps);
        } else {
            RCLCPP_ERROR(
                get_logger(), "no frames measured on %s (%llu seen) - did negotiation succeed?",
                _topic.c_str(), static_cast<unsigned long long>(_run.frames_seen));
        }
    }

private:
    void onImage(const NitrosImageView & view)
    {
        const auto arrival = now();
        _run.frames_seen++;
        const double since_start = (arrival - _start).seconds();
        if (since_start < _warmup_s) {
            return;
        }
        if (!_run.measuring) {
            _run.beginMeasurement(arrival.seconds());
        }

        const void * gpu = view.GetGpuData();
        const size_t bytes = view.GetSizeInBytes();
        if (!gpu || bytes == 0) {
            return;
        }

        // Nothing to prepare: the pixels are already in GPU memory this process can read.
        const auto t1 = std::chrono::steady_clock::now();
        cudaError_t err = launch_checksum(gpu, bytes, _d_out, _stream);
        if (err == cudaSuccess) {
            err = cudaStreamSynchronize(_stream);
        }
        const auto t2 = std::chrono::steady_clock::now();
        if (err != cudaSuccess) {
            RCLCPP_ERROR(get_logger(), "kernel failed: %s", cudaGetErrorString(err));
            return;
        }

        const rclcpp::Time stamp(
            static_cast<int32_t>(view.GetTimestampSeconds()), view.GetTimestampNanoseconds(),
            arrival.get_clock_type());
        if (_run.latency_ms.size() < 5) {
            RCLCPP_INFO(
                get_logger(), "STAMP stamp_ns=%ld arrival_ns=%ld",
                stamp.nanoseconds(), arrival.nanoseconds());
        }
        _run.prep_us.push_back(0.0);
        _run.kernel_us.push_back(std::chrono::duration<double, std::micro>(t2 - t1).count());
        _run.transport_ms.push_back((arrival - stamp).seconds() * 1e3);
        _run.latency_ms.push_back((now() - stamp).seconds() * 1e3);
        _run.recordCsv(stamp.nanoseconds(), arrival.nanoseconds(), 0.0, _run.kernel_us.back());
        _run.wall_end = now().seconds();
    }

    void checkDone()
    {
        if (_run.measuring && (now().seconds() - _run.wall_start) >= _duration_s) {
            rclcpp::shutdown();
        } else if (!_run.measuring && (now() - _start).seconds() > _warmup_s + 15.0) {
            RCLCPP_ERROR(get_logger(), "no frames arrived on %s", _topic.c_str());
            rclcpp::shutdown();
        }
    }

    std::string _topic, _format;
    double _warmup_s, _duration_s, _expected_fps;
    std::string _csv_path;
    bool _reported{false};
    rclcpp::Time _start;
    std::shared_ptr<ManagedNitrosSubscriber<NitrosImageView>> _sub;
    rclcpp::TimerBase::SharedPtr _timer;
    bench::Run _run;
    cudaStream_t _stream{nullptr};
    unsigned long long * _d_out{nullptr};
};
}  // namespace nitros_zc_bench

#ifndef NITROS_ZC_BENCH_COMPONENT_ONLY
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<nitros_zc_bench::NitrosConsumer>();
    rclcpp::spin(node);
    node->reportIfMeasured();
    rclcpp::shutdown();
    return 0;
}
#endif  // NITROS_ZC_BENCH_COMPONENT_ONLY

#ifdef NITROS_ZC_BENCH_COMPONENT_ONLY
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(nitros_zc_bench::NitrosConsumer)
#endif
