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
        const std::string &topic = "/liorf/debug/telemetry",
        double publish_hz = 1.0,
        bool write_files_master = true,
        bool write_timing_stats = true,
        bool write_event = true,
        bool write_warnings = true,
        bool write_telemetry = true,
        bool write_time_deltas = true,
        bool write_frame_metrics = true);

    ~LiorfDiagnostics();

    void markLidarUpdate(const rclcpp::Time &stamp);
    void markGpsUpdate(const rclcpp::Time &stamp);
    void recordSlice(const std::string &stage, double elapsed_ms);
    void logEvent(const std::string &message);
    void logEventThrottle(const std::string &key, double period_sec, const std::string &message);
    void publishWarning(const std::string &message);
    void recordTimeDelta(double delta_s);
    void recordTranslationPrediction(double delta_m);
    void recordFrameMetrics(double stamp_sec,
                            double time_delta_s,
                            double last_time_delta_s,
                            double prediction_delta_m,
                            double optimized_delta_m,
                            double estimated_velocity_mps);
    double getLastPredictionDelta() const;

private:
    std::filesystem::path createRunDirectory(const std::string &base_dir);
    void dumpActiveParameters();
    void publishTelemetry();

    rclcpp::Node *node_ = nullptr;
    std::string history_policy_;
    std::string reliability_policy_;

    std::filesystem::path run_dir_;
    std::ofstream timing_stats_;
    std::ofstream event_log_;
    std::ofstream warning_log_;
    std::ofstream run_parameters_;
    std::ofstream telemetry_csv_;
    std::ofstream time_deltas_csv_;
    std::ofstream frame_metrics_csv_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr telemetry_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr timing_stats_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr time_deltas_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr event_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr warnings_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr frame_metrics_pub_;
    rclcpp::TimerBase::SharedPtr diagnostics_timer_;

    mutable std::mutex mutex_;
    bool write_files_master_ = true;
    bool write_timing_stats_ = true;
    bool write_event_ = true;
    bool write_warnings_ = true;
    bool write_telemetry_ = true;
    bool write_time_deltas_ = true;
    bool write_frame_metrics_ = true;
    rclcpp::Time last_lidar_update_;
    rclcpp::Time last_gps_update_;
    double max_translation_delta_last_batch_m_ = 0.0;
    double last_prediction_delta_m_ = 0.0;
    std::unordered_map<std::string, double> moving_avg_ms_;
    std::unordered_map<std::string, double> last_event_sec_;
};
