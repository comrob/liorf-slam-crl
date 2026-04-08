#pragma once

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <string>
#include <unordered_map>
#include <cstdlib>
#include <chrono>
#include <ctime>
#include <functional>
#include <algorithm>

class LiorfDiagnostics
{
public:
    LiorfDiagnostics(
        rclcpp::Node *node,
        const rclcpp::QoS &qos,
        const std::string &history_policy,
        const std::string &reliability_policy,
        const std::string &base_dir = "~/.ros/liorf_logs",
        const std::string &topic = "/liorf/diagnostics",
        double publish_hz = 1.0)
        : node_(node),
          history_policy_(history_policy),
          reliability_policy_(reliability_policy)
    {
        if (!node_)
            return;

        run_dir_ = createRunDirectory(base_dir);

        timing_stats_.open((run_dir_ / "timing_stats.csv").string(), std::ios::out);
        events_log_.open((run_dir_ / "events.log").string(), std::ios::out);
        run_parameters_.open((run_dir_ / "run_parameters.yaml").string(), std::ios::out);

        if (timing_stats_.is_open())
            timing_stats_ << "stamp_sec,stage,elapsed_ms\n";

        dumpActiveParameters();

        diagnostics_pub_ = node_->create_publisher<std_msgs::msg::String>(topic, qos);

        const double hz = std::max(0.1, publish_hz);
        const auto period_ms = std::chrono::milliseconds(static_cast<int>(1000.0 / hz));
        diagnostics_timer_ = node_->create_wall_timer(period_ms, std::bind(&LiorfDiagnostics::publishTelemetry, this));

        RCLCPP_INFO(node_->get_logger(), "[LIORF_DIAG] log directory: %s", run_dir_.string().c_str());
    }

    ~LiorfDiagnostics()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (timing_stats_.is_open())
            timing_stats_.flush();
        if (events_log_.is_open())
            events_log_.flush();
        if (run_parameters_.is_open())
            run_parameters_.flush();
    }

    void markLidarUpdate(const rclcpp::Time &stamp)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        last_lidar_update_ = stamp;
    }

    void markGpsUpdate(const rclcpp::Time &stamp)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        last_gps_update_ = stamp;
    }

    void recordSlice(const std::string &stage, double elapsed_ms)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        if (timing_stats_.is_open())
            timing_stats_ << std::fixed << std::setprecision(6) << node_->now().seconds() << "," << stage << "," << elapsed_ms << "\n";

        auto it = moving_avg_ms_.find(stage);
        if (it == moving_avg_ms_.end())
            moving_avg_ms_[stage] = elapsed_ms;
        else
            it->second = 0.9 * it->second + 0.1 * elapsed_ms;
    }

    void logEvent(const std::string &message)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (events_log_.is_open())
            events_log_ << std::fixed << std::setprecision(6) << node_->now().seconds() << " " << message << "\n";
    }

    void logEventThrottle(const std::string &key, double period_sec, const std::string &message)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        const double now = node_->now().seconds();
        const auto it = last_event_sec_.find(key);
        if (it != last_event_sec_.end() && (now - it->second) < period_sec)
            return;

        last_event_sec_[key] = now;
        if (events_log_.is_open())
            events_log_ << std::fixed << std::setprecision(6) << now << " " << message << "\n";
    }

private:
    std::filesystem::path createRunDirectory(const std::string &base_dir)
    {
        std::string expanded = base_dir;
        if (!expanded.empty() && expanded[0] == '~')
        {
            const char *home = std::getenv("HOME");
            if (home)
                expanded = std::string(home) + expanded.substr(1);
        }

        auto now = std::chrono::system_clock::now();
        std::time_t now_c = std::chrono::system_clock::to_time_t(now);
        std::tm tm_buf = *std::localtime(&now_c);
        std::ostringstream stamp;
        stamp << std::put_time(&tm_buf, "run_%Y%m%d_%H%M%S");

        std::filesystem::path path = std::filesystem::path(expanded) / stamp.str();
        std::filesystem::create_directories(path);

        // Maintain a stable symlink to the newest run directory.
        // On Linux this will point to ~/.ros/liorf_logs/latest -> run_YYYYMMDD_HHMMSS.
        std::filesystem::path latest_link = std::filesystem::path(expanded) / "latest";
        std::error_code ec;
        std::filesystem::remove(latest_link, ec);
        ec.clear();
        std::filesystem::create_directory_symlink(path.filename(), latest_link, ec);

        return path;
    }

    void dumpActiveParameters()
    {
        if (!run_parameters_.is_open())
            return;

        auto listed = node_->list_parameters({}, 1000);
        run_parameters_ << "node_name: " << node_->get_name() << "\n";
        run_parameters_ << "history_policy: " << history_policy_ << "\n";
        run_parameters_ << "reliability_policy: " << reliability_policy_ << "\n";
        run_parameters_ << "parameters:\n";

        for (const auto &name : listed.names)
        {
            rclcpp::Parameter p;
            if (!node_->get_parameter(name, p))
                continue;
            run_parameters_ << "  " << name << ": " << p.value_to_string() << "\n";
        }
    }

    void publishTelemetry()
    {
        if (!diagnostics_pub_)
            return;

        std_msgs::msg::String msg;
        std::ostringstream ss;

        double lidar_delta = -1.0;
        double gps_delta = -1.0;
        std::unordered_map<std::string, double> moving_copy;

        {
            std::lock_guard<std::mutex> lock(mutex_);
            const double now_sec = node_->now().seconds();
            if (last_lidar_update_.nanoseconds() > 0)
                lidar_delta = now_sec - last_lidar_update_.seconds();
            if (last_gps_update_.nanoseconds() > 0)
                gps_delta = now_sec - last_gps_update_.seconds();
            moving_copy = moving_avg_ms_;
        }

        ss << "{";
        ss << "\"time_since_last_lidar_s\":" << lidar_delta << ",";
        ss << "\"time_since_last_gps_s\":" << gps_delta << ",";
        ss << "\"moving_avg_ms\":{";

        bool first = true;
        for (const auto &kv : moving_copy)
        {
            if (!first)
                ss << ",";
            first = false;
            ss << "\"" << kv.first << "\":" << kv.second;
        }

        ss << "}}";
        msg.data = ss.str();
        diagnostics_pub_->publish(msg);
    }

    rclcpp::Node *node_ = nullptr;
    std::string history_policy_;
    std::string reliability_policy_;

    std::filesystem::path run_dir_;
    std::ofstream timing_stats_;
    std::ofstream events_log_;
    std::ofstream run_parameters_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr diagnostics_pub_;
    rclcpp::TimerBase::SharedPtr diagnostics_timer_;

    std::mutex mutex_;
    rclcpp::Time last_lidar_update_;
    rclcpp::Time last_gps_update_;
    std::unordered_map<std::string, double> moving_avg_ms_;
    std::unordered_map<std::string, double> last_event_sec_;
};
