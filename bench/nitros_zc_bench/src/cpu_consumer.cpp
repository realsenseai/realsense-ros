// Baseline consumer: takes the ordinary sensor_msgs/Image topic and copies every frame
// host-to-device before it can run any GPU work. This is how a GPU perception node consumes
// RealSense frames today.
#include "bench_kernel.h"
#include "bench_stats.h"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <chrono>
#include <memory>
#include <string>

namespace nitros_zc_bench
{
using namespace std::chrono_literals;

class CpuConsumer : public rclcpp::Node
{
public:
    explicit CpuConsumer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : rclcpp::Node("cpu_consumer", options),
      _topic(declare_parameter<std::string>("topic", "/camera/camera/color/image_raw")),
      _warmup_s(declare_parameter<double>("warmup_s", 5.0)),
      _duration_s(declare_parameter<double>("duration_s", 30.0)),
      _expected_fps(declare_parameter<double>("expected_fps", 30.0)),
      _csv_path(declare_parameter<std::string>("csv_path", ""))
    {
        _run.label = "CPU baseline (sensor_msgs/Image + host-to-device copy)";
        cudaSetDevice(0);
        cudaStreamCreateWithFlags(&_stream, cudaStreamNonBlocking);
        cudaMalloc(reinterpret_cast<void **>(&_d_out), sizeof(unsigned long long));

        // Best-effort/depth-1 mirrors the NITROS subscriber's default QoS so neither path gets a
        // deeper queue than the other.
        rclcpp::QoS qos(1);
        qos.best_effort();
        _sub = create_subscription<sensor_msgs::msg::Image>(
            _topic, qos, [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {onImage(msg);});

        _run.openCsv(_csv_path);
        _start = now();
        RCLCPP_INFO(
            get_logger(), "subscribed to %s; %.0f s warmup then %.0f s measured",
            _topic.c_str(), _warmup_s, _duration_s);
        _timer = create_wall_timer(200ms, [this]() {checkDone();});
    }

    ~CpuConsumer() override
    {
        reportIfMeasured();
        if (_d_buf) {
            cudaFree(_d_buf);
        }
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
                get_logger(), "no frames measured on %s (%llu seen) - is the camera publishing?",
                _topic.c_str(), static_cast<unsigned long long>(_run.frames_seen));
        }
    }

private:
    void onImage(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
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

        const size_t bytes = msg->data.size();
        if (bytes == 0) {
            return;
        }
        if (bytes != _buf_bytes) {
            if (_d_buf) {
                cudaFree(_d_buf);
            }
            if (cudaMalloc(&_d_buf, bytes) != cudaSuccess) {
                RCLCPP_ERROR(get_logger(), "cudaMalloc(%zu) failed", bytes);
                _d_buf = nullptr;
                return;
            }
            _buf_bytes = bytes;
        }

        // The copy this benchmark exists to measure: the frame arrives in host memory, so it has
        // to be pushed across to the GPU before anything can look at it.
        const auto t0 = std::chrono::steady_clock::now();
        cudaError_t err = cudaMemcpyAsync(
            _d_buf, msg->data.data(), bytes, cudaMemcpyHostToDevice, _stream);
        if (err == cudaSuccess) {
            err = cudaStreamSynchronize(_stream);
        }
        const auto t1 = std::chrono::steady_clock::now();
        if (err != cudaSuccess) {
            RCLCPP_ERROR(get_logger(), "H2D copy failed: %s", cudaGetErrorString(err));
            return;
        }

        err = launch_checksum(_d_buf, bytes, _d_out, _stream);
        if (err == cudaSuccess) {
            err = cudaStreamSynchronize(_stream);
        }
        const auto t2 = std::chrono::steady_clock::now();
        if (err != cudaSuccess) {
            RCLCPP_ERROR(get_logger(), "kernel failed: %s", cudaGetErrorString(err));
            return;
        }

        const rclcpp::Time stamp(msg->header.stamp);
        if (_run.latency_ms.size() < 5) {
            RCLCPP_INFO(
                get_logger(), "STAMP stamp_ns=%ld arrival_ns=%ld",
                stamp.nanoseconds(), arrival.nanoseconds());
        }
        _run.prep_us.push_back(std::chrono::duration<double, std::micro>(t1 - t0).count());
        _run.kernel_us.push_back(std::chrono::duration<double, std::micro>(t2 - t1).count());
        _run.transport_ms.push_back((arrival - stamp).seconds() * 1e3);
        _run.latency_ms.push_back((now() - stamp).seconds() * 1e3);
        _run.recordCsv(
            stamp.nanoseconds(), arrival.nanoseconds(), _run.prep_us.back(), _run.kernel_us.back());
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

    std::string _topic;
    double _warmup_s, _duration_s, _expected_fps;
    std::string _csv_path;
    bool _reported{false};
    rclcpp::Time _start;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _sub;
    rclcpp::TimerBase::SharedPtr _timer;
    bench::Run _run;
    cudaStream_t _stream{nullptr};
    void * _d_buf{nullptr};
    size_t _buf_bytes{0};
    unsigned long long * _d_out{nullptr};
};
}  // namespace nitros_zc_bench

#ifndef NITROS_ZC_BENCH_COMPONENT_ONLY
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<nitros_zc_bench::CpuConsumer>();
    rclcpp::spin(node);
    node->reportIfMeasured();
    rclcpp::shutdown();
    return 0;
}
#endif  // NITROS_ZC_BENCH_COMPONENT_ONLY

#ifdef NITROS_ZC_BENCH_COMPONENT_ONLY
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(nitros_zc_bench::CpuConsumer)
#endif
