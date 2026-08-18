#pragma once
// Shared measurement plumbing for the two consumers, so both report exactly the same numbers.
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <cstdio>
#include <fstream>
#include <numeric>
#include <string>
#include <vector>

namespace bench
{
// CPU time this process has burned, in seconds (utime + stime from /proc/self/stat).
inline double processCpuSeconds()
{
    std::ifstream f("/proc/self/stat");
    if (!f) {
        return 0.0;
    }
    std::string ignored;
    // Fields 14 and 15 are utime and stime, in clock ticks. Field 2 (comm) may contain spaces but
    // is parenthesised, so skip past the closing paren first.
    std::string all((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    const size_t close = all.rfind(')');
    if (close == std::string::npos) {
        return 0.0;
    }
    std::vector<std::string> fields;
    std::string rest = all.substr(close + 2);   // skip ") "
    size_t pos = 0;
    while (pos < rest.size()) {
        const size_t sp = rest.find(' ', pos);
        fields.push_back(rest.substr(pos, sp == std::string::npos ? std::string::npos : sp - pos));
        if (sp == std::string::npos) {
            break;
        }
        pos = sp + 1;
    }
    // rest[0] is field 3 (state), so utime is index 11 and stime index 12.
    if (fields.size() < 13) {
        return 0.0;
    }
    const double ticks = static_cast<double>(sysconf(_SC_CLK_TCK));
    return (std::stod(fields[11]) + std::stod(fields[12])) / ticks;
}

inline double percentile(std::vector<double> v, double p)
{
    if (v.empty()) {
        return 0.0;
    }
    std::sort(v.begin(), v.end());
    const size_t idx = std::min(v.size() - 1, static_cast<size_t>(v.size() * p));
    return v[idx];
}

inline double mean(const std::vector<double> & v)
{
    return v.empty() ? 0.0 : std::accumulate(v.begin(), v.end(), 0.0) / v.size();
}

// One consumer run: counts, per-frame timings, and CPU usage over the measured window.
struct Run
{
    std::string label;
    // Optional per-frame record. Absolute stamp-to-arrival latency is not comparable across
    // transports (the RealSense stamp and the consumer clock are not the same domain), so the
    // honest comparison is to pair the two consumers' rows by stamp_ns and diff their arrivals.
    std::ofstream csv;
    std::vector<double> latency_ms;      // frame timestamp -> GPU work complete
    std::vector<double> transport_ms;    // frame timestamp -> callback entry
    std::vector<double> prep_us;         // getting the frame onto the GPU (H2D copy, or nothing)
    std::vector<double> kernel_us;       // the identical pixel-reading kernel
    uint64_t frames_seen{0};             // includes warmup
    double cpu_start{0.0};
    double wall_start{0.0};
    double wall_end{0.0};
    bool measuring{false};

    void openCsv(const std::string & path)
    {
        if (path.empty()) {
            return;
        }
        csv.open(path);
        if (csv) {
            csv << "stamp_ns,arrival_ns,prep_us,kernel_us\n";
        }
    }

    void recordCsv(int64_t stamp_ns, int64_t arrival_ns, double prep_us, double kernel_us)
    {
        if (csv) {
            csv << stamp_ns << ',' << arrival_ns << ',' << prep_us << ',' << kernel_us << '\n';
        }
    }

    void beginMeasurement(double now)
    {
        measuring = true;
        cpu_start = processCpuSeconds();
        wall_start = now;
    }

    void report(const rclcpp::Logger & logger, double expected_fps) const
    {
        const double wall = wall_end - wall_start;
        const double cpu = processCpuSeconds() - cpu_start;
        const size_t n = latency_ms.size();
        const double fps = wall > 0 ? n / wall : 0.0;
        RCLCPP_INFO(logger, "================ %s ================", label.c_str());
        RCLCPP_INFO(logger, "measured window      : %.1f s (%zu frames)", wall, n);
        RCLCPP_INFO(
            logger, "delivered rate       : %.2f FPS  (%.1f%% of the %.0f FPS stream)",
            fps, expected_fps > 0 ? 100.0 * fps / expected_fps : 0.0, expected_fps);
        RCLCPP_INFO(
            logger, "onto-GPU cost / frame: mean %.1f us  p99 %.1f us",
            mean(prep_us), percentile(prep_us, 0.99));
        RCLCPP_INFO(
            logger, "kernel / frame       : mean %.1f us  p99 %.1f us",
            mean(kernel_us), percentile(kernel_us, 0.99));
        RCLCPP_INFO(
            logger, "transport latency    : mean %.1f ms  p50 %.1f  p99 %.1f",
            mean(transport_ms), percentile(transport_ms, 0.50), percentile(transport_ms, 0.99));
        RCLCPP_INFO(
            logger, "end-to-end latency   : mean %.1f ms  p50 %.1f  p99 %.1f",
            mean(latency_ms), percentile(latency_ms, 0.50), percentile(latency_ms, 0.99));
        RCLCPP_INFO(
            logger, "consumer CPU         : %.1f%% of one core (%.2f s CPU / %.1f s wall)",
            wall > 0 ? 100.0 * cpu / wall : 0.0, cpu, wall);
        // Machine-readable line for the runner script.
        printf(
            "RESULT {\"label\":\"%s\",\"frames\":%zu,\"wall_s\":%.2f,\"fps\":%.3f,"
            "\"prep_us_mean\":%.1f,\"prep_us_p99\":%.1f,\"kernel_us_mean\":%.1f,"
            "\"transport_ms_mean\":%.2f,\"transport_ms_p99\":%.2f,"
            "\"e2e_ms_mean\":%.2f,\"e2e_ms_p50\":%.2f,\"e2e_ms_p99\":%.2f,\"cpu_pct\":%.2f}\n",
            label.c_str(), n, wall, fps, mean(prep_us), percentile(prep_us, 0.99),
            mean(kernel_us), mean(transport_ms), percentile(transport_ms, 0.99),
            mean(latency_ms), percentile(latency_ms, 0.50), percentile(latency_ms, 0.99),
            wall > 0 ? 100.0 * cpu / wall : 0.0);
        fflush(stdout);
    }
};
}  // namespace bench
