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
#include <array>
#include <vector>
#include <limits>

#include <Eigen/Core>
#include "scanAlignment/IMappingBackend.hpp"

struct TumPoseSample
{
    double stamp_sec = 0.0;
    double tx = 0.0;
    double ty = 0.0;
    double tz = 0.0;
    double qx = 0.0;
    double qy = 0.0;
    double qz = 0.0;
    double qw = 1.0;
};

struct DiagnosticsOutputPolicy
{
    bool write_files_master = true;
    bool write_timing_stats = true;
    bool write_event = true;
    bool write_warnings = true;
    bool write_telemetry = true;
    bool write_time_deltas = true;
    bool write_frame_metrics = true;
    bool write_scale_replay_frames = false;
};

struct TrajectoryOutputPolicy
{
    bool write_odom_trajectory_tum = false;
};

// One-shot metadata describing the complementary odometry source. Written next
// to the raw odometry stream so that an offline replay can re-synchronize a
// different odometry source against the same LiDAR frames.
struct ComplementaryOdomMetaSample
{
    // T_complementary_to_lidar (odometry frame -> LiDAR frame).
    std::array<double, 3> extrinsic_translation{};
    std::array<double, 4> extrinsic_rotation_xyzw{{0.0, 0.0, 0.0, 1.0}};
    std::string source_frame;
    std::string lidar_frame;
    // translationScale the online run baked into the queued poses. The raw
    // stream is logged before it is applied, so replay owns this factor.
    double translation_scale_applied_online = 1.0;
};

struct ComplementaryOdomTwistDebugSample
{
    double stamp_sec = 0.0;
    double lidar_prev_stamp_s = std::numeric_limits<double>::quiet_NaN();
    double lidar_curr_stamp_s = std::numeric_limits<double>::quiet_NaN();
    double odom_prev_stamp_s = std::numeric_limits<double>::quiet_NaN();
    double odom_curr_stamp_s = std::numeric_limits<double>::quiet_NaN();
    int odom_queue_size = 0;
    int odom_samples_between = 0;
    double dt_complementary_s = std::numeric_limits<double>::quiet_NaN();
    double prev_match_abs_dt_s = std::numeric_limits<double>::quiet_NaN();
    double curr_match_abs_dt_s = std::numeric_limits<double>::quiet_NaN();
    double lin_norm = std::numeric_limits<double>::quiet_NaN();
    double ang_norm = std::numeric_limits<double>::quiet_NaN();
};

struct ComplementaryOdomScaleDebugSample
{
    double stamp_sec = 0.0;
    double frame_stamp_s = std::numeric_limits<double>::quiet_NaN();
    double lidar_prev_stamp_s = std::numeric_limits<double>::quiet_NaN();
    double lidar_curr_stamp_s = std::numeric_limits<double>::quiet_NaN();
    double odom_prev_stamp_s = std::numeric_limits<double>::quiet_NaN();
    double odom_curr_stamp_s = std::numeric_limits<double>::quiet_NaN();
    int odom_queue_size = 0;
    int odom_samples_between = 0;
    double odom_span_s = std::numeric_limits<double>::quiet_NaN();
    double prev_match_abs_dt_s = std::numeric_limits<double>::quiet_NaN();
    double curr_match_abs_dt_s = std::numeric_limits<double>::quiet_NaN();
    bool enabled = false;
    bool gate_observable = false;
    double scale_instant_raw = std::numeric_limits<double>::quiet_NaN();
    double scale_instant_raw_clamped = std::numeric_limits<double>::quiet_NaN();
    double scale_instant_raw_safe = std::numeric_limits<double>::quiet_NaN();
    double scale_fallback_history = std::numeric_limits<double>::quiet_NaN();
    double scale_smooth = std::numeric_limits<double>::quiet_NaN();
    double scale_smooth_ratio = std::numeric_limits<double>::quiet_NaN();
    double scale_smooth_legacy = std::numeric_limits<double>::quiet_NaN();
    double scale_applied = std::numeric_limits<double>::quiet_NaN();
    bool smooth_from_numden_enabled = false;
    int estimator_mode = 0; // 0=shadow, 1=active
    bool scale_estimate_updated = false;
    bool scale_applied_to_state = false;
    double scale_ratio_raw = std::numeric_limits<double>::quiet_NaN();
    double scale_ratio_unprojected_raw = std::numeric_limits<double>::quiet_NaN();
    double scale_ls_raw = std::numeric_limits<double>::quiet_NaN();
    double theta_deg = std::numeric_limits<double>::quiet_NaN();
    double lidar_body_dx_m = std::numeric_limits<double>::quiet_NaN();
    double lidar_body_dy_m = std::numeric_limits<double>::quiet_NaN();
    double lidar_body_dz_m = std::numeric_limits<double>::quiet_NaN();
    double complementary_body_dx_m = std::numeric_limits<double>::quiet_NaN();
    double complementary_body_dy_m = std::numeric_limits<double>::quiet_NaN();
    double complementary_body_dz_m = std::numeric_limits<double>::quiet_NaN();
    double lidar_body_nondeg_dx_m = std::numeric_limits<double>::quiet_NaN();
    double lidar_body_nondeg_dy_m = std::numeric_limits<double>::quiet_NaN();
    double lidar_body_nondeg_dz_m = std::numeric_limits<double>::quiet_NaN();
    double complementary_body_nondeg_dx_m = std::numeric_limits<double>::quiet_NaN();
    double complementary_body_nondeg_dy_m = std::numeric_limits<double>::quiet_NaN();
    double complementary_body_nondeg_dz_m = std::numeric_limits<double>::quiet_NaN();
    double lidar_nondeg_unprojected_m = std::numeric_limits<double>::quiet_NaN();
    double lidar_nondeg_m = std::numeric_limits<double>::quiet_NaN();
    double complementary_nondeg_m = std::numeric_limits<double>::quiet_NaN();
    double min_nondeg_m = std::numeric_limits<double>::quiet_NaN();
    int scale_smoothing_window = 0;
    double complementary_odom_lin_speed_orig_mps = std::numeric_limits<double>::quiet_NaN();
    double lidar_lin_speed_nondeg_mps = std::numeric_limits<double>::quiet_NaN();
    double complementary_odom_lin_speed_nondeg_mps = std::numeric_limits<double>::quiet_NaN();
    double lidar_lin_speed_proj_scale1_mps = std::numeric_limits<double>::quiet_NaN();
    double lidar_lin_speed_after_scale_mps = std::numeric_limits<double>::quiet_NaN();
    double dt_scan_s = std::numeric_limits<double>::quiet_NaN();
    double dt_scale_odom_interval_s = std::numeric_limits<double>::quiet_NaN();
    double dt_scale_lidar_interval_s = std::numeric_limits<double>::quiet_NaN();
    double dt_scale_odom_pair_interval_s = std::numeric_limits<double>::quiet_NaN();
    double dt_immediate_odom_pair_interval_s = std::numeric_limits<double>::quiet_NaN();
};

// Per-frame record of everything the complementary-odom scaling logic consumes,
// intended for offline replay of the scaling math with different parameters.
// All twists/basis vectors are expressed in the body frame of the optimized pose.
struct ScaleReplayFrameSample
{
    double stamp_sec = 0.0;
    double lidar_prev_stamp_s = std::numeric_limits<double>::quiet_NaN();
    double dt_scan_s = std::numeric_limits<double>::quiet_NaN();
    bool degeneracy_detected = false;
    bool has_degeneracy_basis = false;
    bool has_complementary_twist = false;
    bool override_applied_to_state = false;
    double scale_applied = std::numeric_limits<double>::quiet_NaN();
    int basis_size = 0;
    // Orthonormal degeneracy basis twists (vx vy vz wx wy wz); NaN beyond basis_size.
    std::array<std::array<double, 6>, 3> basis_twists{};
    // Displacement twist of T_prev^-1 * T_optimized (pre-override LiDAR increment).
    std::array<double, 6> lidar_increment_twist{};
    // Unscaled complementary odometry velocity twist in the LiDAR frame.
    std::array<double, 6> complementary_twist{};
    double dt_complementary_s = std::numeric_limits<double>::quiet_NaN();
    double odom_prev_stamp_s = std::numeric_limits<double>::quiet_NaN();
    double odom_curr_stamp_s = std::numeric_limits<double>::quiet_NaN();
    // Absolute map-frame poses: tx ty tz qx qy qz qw.
    std::array<double, 7> pose_prev{};
    std::array<double, 7> pose_optimized{};
    std::array<double, 7> pose_effective{};
};

class LiliDiagnostics
{
public:
    LiliDiagnostics(
        rclcpp::Node *node,
        const rclcpp::QoS &qos,
        const std::string &history_policy,
        const std::string &reliability_policy,
        const std::string &base_dir = "~/.ros/lili_logs",
        const std::string &run_suffix = "",
        const std::string &topic = "/lili/debug/telemetry",
        double publish_hz = 1.0,
        DiagnosticsOutputPolicy diagnostics_output_policy = {},
        TrajectoryOutputPolicy trajectory_output_policy = {});

    ~LiliDiagnostics();

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
    void recordDegeneracyTelemetry(
        double stamp_sec, 
        const std::string &module_name, 
        bool is_degenerate, 
        const std::vector<Eigen::Matrix<float, 6, 1>> &twists);
    void recordJacobianDegeneracyTelemetry(
        double stamp_sec,
        const std::string &module_name,
        const lio::JacobianDegeneracyInfo &info);
    void recordPerturbationDegeneracyTelemetry(
        double stamp_sec,
        bool detected,
        size_t raw_twist_count,
        size_t pca_basis_count,
        size_t sparsified_basis_count,
        bool failed,
        const std::string &fail_reason);
    void recordComplementaryOdomScaleCsv(const ComplementaryOdomScaleDebugSample &sample);
    void recordComplementaryOdomTwistCsv(const ComplementaryOdomTwistDebugSample &sample);
    void recordScaleReplayFrameCsv(const ScaleReplayFrameSample &sample);
    // Raw (pre-translationScale) complementary odometry poses at full rate.
    void recordComplementaryOdomStreamTum(const TumPoseSample &sample);
    void recordComplementaryOdomMeta(const ComplementaryOdomMetaSample &sample);
    void recordOdomTrajectoryTum(const TumPoseSample &sample);
    double getLastPredictionDelta() const;
    std::filesystem::path runDirectory() const { return run_dir_; }

private:
    std::filesystem::path createRunDirectory(const std::string &base_dir, const std::string &run_suffix);
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
    std::ofstream odom_trajectory_tum_;
    std::ofstream degeneracy_metrics_csv_;
    std::ofstream jacobian_degeneracy_metrics_csv_;
    std::ofstream perturbation_degeneracy_metrics_csv_;
    std::ofstream complementary_odom_scale_csv_;
    std::ofstream complementary_odom_twist_csv_;
    std::ofstream scale_replay_frames_csv_;
    std::ofstream complementary_odom_stream_tum_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr telemetry_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr timing_stats_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr time_deltas_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr event_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr warnings_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr frame_metrics_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr degeneracy_metrics_pub_;
    rclcpp::TimerBase::SharedPtr diagnostics_timer_;

    mutable std::mutex mutex_;
    DiagnosticsOutputPolicy diagnostics_output_policy_{};
    TrajectoryOutputPolicy trajectory_output_policy_{};
    rclcpp::Time last_lidar_update_;
    rclcpp::Time last_gps_update_;
    double max_translation_delta_last_batch_m_ = 0.0;
    double last_prediction_delta_m_ = 0.0;
    std::unordered_map<std::string, double> moving_avg_ms_;
    std::unordered_map<std::string, double> last_event_sec_;

};
