#include "lili_diagnostics.h"

LiliDiagnostics::LiliDiagnostics(
    rclcpp::Node *node,
    const rclcpp::QoS &qos,
    const std::string &history_policy,
    const std::string &reliability_policy,
    const std::string &base_dir,
    const std::string &run_suffix,
    const std::string &topic,
    double publish_hz,
    DiagnosticsOutputPolicy diagnostics_output_policy,
    TrajectoryOutputPolicy trajectory_output_policy)
    : node_(node),
      history_policy_(history_policy),
      reliability_policy_(reliability_policy),
      diagnostics_output_policy_(diagnostics_output_policy),
      trajectory_output_policy_(trajectory_output_policy)
{
    if (!node_)
        return;

    run_dir_ = createRunDirectory(base_dir, run_suffix);

    const bool diagnostics_files_enabled = diagnostics_output_policy_.write_files_master;
    const bool odom_trajectory_export_enabled = trajectory_output_policy_.write_odom_trajectory_tum;

    run_parameters_.open((run_dir_ / "run_parameters.yaml").string(), std::ios::out);
    if (diagnostics_files_enabled && diagnostics_output_policy_.write_timing_stats)
        timing_stats_.open((run_dir_ / "timing_stats.csv").string(), std::ios::out);
    if (diagnostics_files_enabled && diagnostics_output_policy_.write_event)
        event_log_.open((run_dir_ / "event.txt").string(), std::ios::out);
    if (diagnostics_files_enabled && diagnostics_output_policy_.write_warnings)
        warning_log_.open((run_dir_ / "warnings.txt").string(), std::ios::out);
    if (diagnostics_files_enabled && diagnostics_output_policy_.write_telemetry)
        telemetry_csv_.open((run_dir_ / "telemetry.csv").string(), std::ios::out);
    if (diagnostics_files_enabled && diagnostics_output_policy_.write_time_deltas)
        time_deltas_csv_.open((run_dir_ / "time_deltas.csv").string(), std::ios::out);
    if (diagnostics_files_enabled && diagnostics_output_policy_.write_frame_metrics)
        frame_metrics_csv_.open((run_dir_ / "frame_metrics.csv").string(), std::ios::out);
    if (diagnostics_files_enabled)
    {
        complementary_odom_scale_csv_.open((run_dir_ / "complementary_odom_scale.csv").string(), std::ios::out);
        complementary_odom_twist_csv_.open((run_dir_ / "complementary_odom_twist.csv").string(), std::ios::out);
    }
    if (diagnostics_files_enabled && diagnostics_output_policy_.write_scale_replay_frames)
    {
        scale_replay_frames_csv_.open((run_dir_ / "scale_replay_frames.csv").string(), std::ios::out);
        complementary_odom_stream_tum_.open((run_dir_ / "complementary_odom_stream.tum").string(), std::ios::out);
    }

    // Odom trajectory TUM export is controlled independently from diagnostics file gating.
    if (odom_trajectory_export_enabled)
        odom_trajectory_tum_.open((run_dir_ / "trajectory_odom.tum").string(), std::ios::out);
    
    if (diagnostics_files_enabled && diagnostics_output_policy_.write_telemetry) // Or a new write_degeneracy flag
    {
        degeneracy_metrics_csv_.open((run_dir_ / "degeneracy_metrics.csv").string(), std::ios::out);
        if (degeneracy_metrics_csv_.is_open())
            degeneracy_metrics_csv_ << "stamp_sec,module_name,is_degenerate,twists\n";

        jacobian_degeneracy_metrics_csv_.open((run_dir_ / "jacobian_degeneracy_metrics.csv").string(), std::ios::out);
        if (jacobian_degeneracy_metrics_csv_.is_open())
            jacobian_degeneracy_metrics_csv_ << "stamp_sec,module_name,computed,is_degenerate,selected_correspondences,reason,eigenvalues,thresholds,zeroed_modes,eigenvectors\n";

        perturbation_degeneracy_metrics_csv_.open((run_dir_ / "perturbation_degeneracy_metrics.csv").string(), std::ios::out);
        if (perturbation_degeneracy_metrics_csv_.is_open())
            perturbation_degeneracy_metrics_csv_ << "stamp_sec,detected,raw_twist_count,pca_basis_count,sparsified_basis_count,failed,fail_reason\n";
    }

    if (timing_stats_.is_open())
        timing_stats_ << "stamp_sec,stage,elapsed_ms\n";
    if (telemetry_csv_.is_open())
        telemetry_csv_ << "stamp_sec,time_since_last_lidar_s,time_since_last_gps_s\n";
    if (time_deltas_csv_.is_open())
        time_deltas_csv_ << "stamp_sec,delta_s\n";
    if (frame_metrics_csv_.is_open())
        frame_metrics_csv_ << "stamp_sec,time_delta_s,last_time_delta_s,prediction_delta_m,optimized_delta_m,estimated_velocity_mps\n";
    if (complementary_odom_scale_csv_.is_open())
    {
        complementary_odom_scale_csv_
            << "time"
            << ",complementary_odom_scale/stamp/frame_s"
            << ",complementary_odom_scale/stamp/lidar_prev_s"
            << ",complementary_odom_scale/stamp/lidar_curr_s"
            << ",complementary_odom_scale/stamp/odom_prev_s"
            << ",complementary_odom_scale/stamp/odom_curr_s"
            << ",complementary_odom_scale/alignment/odom_queue_size"
            << ",complementary_odom_scale/alignment/odom_samples_between"
            << ",complementary_odom_scale/alignment/odom_span_s"
            << ",complementary_odom_scale/alignment/prev_match_abs_dt_s"
            << ",complementary_odom_scale/alignment/curr_match_abs_dt_s"
            << ",complementary_odom_scale/gate/enabled"
            << ",complementary_odom_scale/gate/observable"
            << ",complementary_odom_scale/scale/instant_raw"
            << ",complementary_odom_scale/scale/instant_raw_clamped"
            << ",complementary_odom_scale/scale/instant_raw_safe"
            << ",complementary_odom_scale/scale/fallback_history"
            << ",complementary_odom_scale/scale/smooth"
            << ",complementary_odom_scale/scale/smooth_ratio"
            << ",complementary_odom_scale/scale/smooth_legacy"
            << ",complementary_odom_scale/scale/applied"
            << ",complementary_odom_scale/smoothing/use_numden"
            << ",complementary_odom_scale/mode/estimator_mode"
            << ",complementary_odom_scale/mode/scale_estimate_updated"
            << ",complementary_odom_scale/mode/scale_applied_to_state"
            << ",complementary_odom_scale/scale/raw/projected_ratio"
            << ",complementary_odom_scale/scale/raw/unprojected_ratio"
            << ",complementary_odom_scale/scale/raw/least_squares"
            << ",complementary_odom_scale/scale/raw/theta_deg"
            << ",complementary_odom_scale/body/lidar/dx_m"
            << ",complementary_odom_scale/body/lidar/dy_m"
            << ",complementary_odom_scale/body/lidar/dz_m"
            << ",complementary_odom_scale/body/complementary/dx_m"
            << ",complementary_odom_scale/body/complementary/dy_m"
            << ",complementary_odom_scale/body/complementary/dz_m"
            << ",complementary_odom_scale/body_nondeg/lidar/dx_m"
            << ",complementary_odom_scale/body_nondeg/lidar/dy_m"
            << ",complementary_odom_scale/body_nondeg/lidar/dz_m"
            << ",complementary_odom_scale/body_nondeg/complementary/dx_m"
            << ",complementary_odom_scale/body_nondeg/complementary/dy_m"
            << ",complementary_odom_scale/body_nondeg/complementary/dz_m"
            << ",complementary_odom_scale/magnitude/lidar_nondeg_unprojected_m"
            << ",complementary_odom_scale/magnitude/lidar_nondeg_projected_m"
            << ",complementary_odom_scale/magnitude/complementary_nondeg_m"
            << ",complementary_odom_scale/magnitude/min_nondeg_m"
            << ",complementary_odom_scale/smoothing/window"
            << ",complementary_odom_scale/speed/complementary_orig_mps"
            << ",complementary_odom_scale/speed/lidar_nondeg_mps"
            << ",complementary_odom_scale/speed/complementary_nondeg_mps"
            << ",complementary_odom_scale/speed/lidar_proj_scale1_mps"
            << ",complementary_odom_scale/speed/lidar_after_scale_mps"
            << ",complementary_odom_scale/dt/scan_s"
            << ",complementary_odom_scale/dt/scale_odom_interval_s"
            << ",complementary_odom_scale/dt/scale_lidar_pair_interval_s"
            << ",complementary_odom_scale/dt/scale_odom_pair_interval_s"
            << ",complementary_odom_scale/dt/immediate_odom_pair_interval_s\n";
    }
    if (complementary_odom_twist_csv_.is_open())
    {
        complementary_odom_twist_csv_
            << "time"
            << ",complementary_odom_twist/stamp/lidar_prev_s"
            << ",complementary_odom_twist/stamp/lidar_curr_s"
            << ",complementary_odom_twist/stamp/odom_prev_s"
            << ",complementary_odom_twist/stamp/odom_curr_s"
            << ",complementary_odom_twist/alignment/odom_queue_size"
            << ",complementary_odom_twist/alignment/odom_samples_between"
            << ",complementary_odom_twist/alignment/dt_complementary_s"
            << ",complementary_odom_twist/alignment/prev_match_abs_dt_s"
            << ",complementary_odom_twist/alignment/curr_match_abs_dt_s"
            << ",complementary_odom_twist/norm/lin"
            << ",complementary_odom_twist/norm/ang\n";
    }
    if (scale_replay_frames_csv_.is_open())
    {
        static const char *kTwistAxes[6] = {"vx", "vy", "vz", "wx", "wy", "wz"};
        static const char *kPoseAxes[7] = {"tx", "ty", "tz", "qx", "qy", "qz", "qw"};
        scale_replay_frames_csv_
            << "time"
            << ",scale_replay/stamp/lidar_prev_s"
            << ",scale_replay/dt/scan_s"
            << ",scale_replay/flags/degeneracy_detected"
            << ",scale_replay/flags/has_degeneracy_basis"
            << ",scale_replay/flags/has_complementary_twist"
            << ",scale_replay/flags/override_applied_to_state"
            << ",scale_replay/scale/applied"
            << ",scale_replay/basis/size";
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 6; ++j)
                scale_replay_frames_csv_ << ",scale_replay/basis/" << i << "/" << kTwistAxes[j];
        for (int j = 0; j < 6; ++j)
            scale_replay_frames_csv_ << ",scale_replay/lidar_increment/" << kTwistAxes[j];
        for (int j = 0; j < 6; ++j)
            scale_replay_frames_csv_ << ",scale_replay/complementary_twist/" << kTwistAxes[j];
        scale_replay_frames_csv_
            << ",scale_replay/complementary/dt_s"
            << ",scale_replay/complementary/odom_prev_stamp_s"
            << ",scale_replay/complementary/odom_curr_stamp_s";
        for (int j = 0; j < 7; ++j)
            scale_replay_frames_csv_ << ",scale_replay/pose_prev/" << kPoseAxes[j];
        for (int j = 0; j < 7; ++j)
            scale_replay_frames_csv_ << ",scale_replay/pose_optimized/" << kPoseAxes[j];
        for (int j = 0; j < 7; ++j)
            scale_replay_frames_csv_ << ",scale_replay/pose_effective/" << kPoseAxes[j];
        scale_replay_frames_csv_ << "\n";
    }

    dumpActiveParameters();

    telemetry_pub_ = node_->create_publisher<std_msgs::msg::String>(topic, qos);
    timing_stats_pub_ = node_->create_publisher<std_msgs::msg::String>("/lili/debug/timing_stats", qos);
    time_deltas_pub_ = node_->create_publisher<std_msgs::msg::String>("/lili/debug/time_deltas", qos);
    event_pub_ = node_->create_publisher<std_msgs::msg::String>("/lili/debug/event", qos);
    warnings_pub_ = node_->create_publisher<std_msgs::msg::String>("/lili/debug/warnings", qos);
    frame_metrics_pub_ = node_->create_publisher<std_msgs::msg::String>("/lili/debug/frame_metrics", qos);
    degeneracy_metrics_pub_ = node_->create_publisher<std_msgs::msg::String>("/lili/debug/degeneracy_metrics", qos);

    const double hz = std::max(0.1, publish_hz);
    const auto period_ms = std::chrono::milliseconds(static_cast<int>(1000.0 / hz));
    diagnostics_timer_ = node_->create_wall_timer(period_ms, std::bind(&LiliDiagnostics::publishTelemetry, this));

    RCLCPP_INFO(node_->get_logger(), "[LILI_DIAG] log directory: %s", run_dir_.string().c_str());
}

LiliDiagnostics::~LiliDiagnostics()
{
    std::lock_guard<std::mutex> lock(mutex_);
    const bool diagnostics_files_enabled = diagnostics_output_policy_.write_files_master;
    if (timing_stats_.is_open())
        timing_stats_.flush();
    if (diagnostics_files_enabled && event_log_.is_open())
        event_log_.flush();
    if (diagnostics_files_enabled && warning_log_.is_open())
        warning_log_.flush();
    if (run_parameters_.is_open())
        run_parameters_.flush();
    if (diagnostics_files_enabled && telemetry_csv_.is_open())
        telemetry_csv_.flush();
    if (diagnostics_files_enabled && time_deltas_csv_.is_open())
        time_deltas_csv_.flush();
    if (diagnostics_files_enabled && frame_metrics_csv_.is_open())
        frame_metrics_csv_.flush();
    if (diagnostics_files_enabled && complementary_odom_scale_csv_.is_open())
        complementary_odom_scale_csv_.flush();
    if (diagnostics_files_enabled && complementary_odom_twist_csv_.is_open())
        complementary_odom_twist_csv_.flush();
    if (diagnostics_files_enabled && scale_replay_frames_csv_.is_open())
        scale_replay_frames_csv_.flush();
    if (odom_trajectory_tum_.is_open())
        odom_trajectory_tum_.flush();
    if (diagnostics_files_enabled && jacobian_degeneracy_metrics_csv_.is_open())
        jacobian_degeneracy_metrics_csv_.flush();
    if (diagnostics_files_enabled && perturbation_degeneracy_metrics_csv_.is_open())
        perturbation_degeneracy_metrics_csv_.flush();
}

void LiliDiagnostics::markLidarUpdate(const rclcpp::Time &stamp)
{
    std::lock_guard<std::mutex> lock(mutex_);
    last_lidar_update_ = stamp;
}

void LiliDiagnostics::markGpsUpdate(const rclcpp::Time &stamp)
{
    std::lock_guard<std::mutex> lock(mutex_);
    last_gps_update_ = stamp;
}

void LiliDiagnostics::recordSlice(const std::string &stage, double elapsed_ms)
{
    std::lock_guard<std::mutex> lock(mutex_);
    const double stamp_sec = node_->now().seconds();

    if (timing_stats_.is_open())
        timing_stats_ << std::fixed << std::setprecision(6) << stamp_sec << "," << stage << "," << elapsed_ms << "\n";

    if (timing_stats_pub_)
    {
        std_msgs::msg::String msg;
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(6);
        ss << "{";
        ss << "\"stamp_sec\":" << stamp_sec << ",";
        ss << "\"stage\":\"" << stage << "\",";
        ss << "\"elapsed_ms\":" << elapsed_ms;
        ss << "}";
        msg.data = ss.str();
        timing_stats_pub_->publish(msg);
    }

    auto it = moving_avg_ms_.find(stage);
    if (it == moving_avg_ms_.end())
        moving_avg_ms_[stage] = elapsed_ms;
    else
        it->second = 0.9 * it->second + 0.1 * elapsed_ms;
}

void LiliDiagnostics::logEvent(const std::string &message)
{
    std::lock_guard<std::mutex> lock(mutex_);
    const double stamp_sec = node_->now().seconds();
    if (event_log_.is_open())
        event_log_ << std::fixed << std::setprecision(6) << stamp_sec << " " << message << "\n";

    if (event_pub_)
    {
        std_msgs::msg::String msg;
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(6);
        ss << "{";
        ss << "\"stamp_sec\":" << stamp_sec << ",";
        ss << "\"message\":\"" << message << "\"";
        ss << "}";
        msg.data = ss.str();
        event_pub_->publish(msg);
    }
}

void LiliDiagnostics::logEventThrottle(const std::string &key, double period_sec, const std::string &message)
{
    std::lock_guard<std::mutex> lock(mutex_);

    const double now = node_->now().seconds();
    const auto it = last_event_sec_.find(key);
    if (it != last_event_sec_.end() && (now - it->second) < period_sec)
        return;

    last_event_sec_[key] = now;
    if (event_log_.is_open())
        event_log_ << std::fixed << std::setprecision(6) << now << " " << message << "\n";

    if (event_pub_)
    {
        std_msgs::msg::String msg;
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(6);
        ss << "{";
        ss << "\"stamp_sec\":" << now << ",";
        ss << "\"message\":\"" << message << "\"";
        ss << "}";
        msg.data = ss.str();
        event_pub_->publish(msg);
    }
}

void LiliDiagnostics::recordComplementaryOdomScaleCsv(const ComplementaryOdomScaleDebugSample &sample)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!complementary_odom_scale_csv_.is_open())
        return;

    complementary_odom_scale_csv_ << std::fixed << std::setprecision(6)
                                   << sample.stamp_sec << ","
                                   << sample.frame_stamp_s << ","
                                   << sample.lidar_prev_stamp_s << ","
                                   << sample.lidar_curr_stamp_s << ","
                                   << sample.odom_prev_stamp_s << ","
                                   << sample.odom_curr_stamp_s << ","
                                   << sample.odom_queue_size << ","
                                   << sample.odom_samples_between << ","
                                   << sample.odom_span_s << ","
                                   << sample.prev_match_abs_dt_s << ","
                                   << sample.curr_match_abs_dt_s << ","
                                   << (sample.enabled ? 1 : 0) << ","
                                   << (sample.gate_observable ? 1 : 0) << ","
                                   << sample.scale_instant_raw << ","
                                   << sample.scale_instant_raw_clamped << ","
                                   << sample.scale_instant_raw_safe << ","
                                   << sample.scale_fallback_history << ","
                                   << sample.scale_smooth << ","
                                   << sample.scale_smooth_ratio << ","
                                   << sample.scale_smooth_legacy << ","
                                   << sample.scale_applied << ","
                                   << (sample.smooth_from_numden_enabled ? 1 : 0) << ","
                                   << sample.estimator_mode << ","
                                   << (sample.scale_estimate_updated ? 1 : 0) << ","
                                   << (sample.scale_applied_to_state ? 1 : 0) << ","
                                   << sample.scale_ratio_raw << ","
                                   << sample.scale_ratio_unprojected_raw << ","
                                   << sample.scale_ls_raw << ","
                                   << sample.theta_deg << ","
                                   << sample.lidar_body_dx_m << ","
                                   << sample.lidar_body_dy_m << ","
                                   << sample.lidar_body_dz_m << ","
                                   << sample.complementary_body_dx_m << ","
                                   << sample.complementary_body_dy_m << ","
                                   << sample.complementary_body_dz_m << ","
                                   << sample.lidar_body_nondeg_dx_m << ","
                                   << sample.lidar_body_nondeg_dy_m << ","
                                   << sample.lidar_body_nondeg_dz_m << ","
                                   << sample.complementary_body_nondeg_dx_m << ","
                                   << sample.complementary_body_nondeg_dy_m << ","
                                   << sample.complementary_body_nondeg_dz_m << ","
                                   << sample.lidar_nondeg_unprojected_m << ","
                                   << sample.lidar_nondeg_m << ","
                                   << sample.complementary_nondeg_m << ","
                                   << sample.min_nondeg_m << ","
                                   << sample.scale_smoothing_window << ","
                                   << sample.complementary_odom_lin_speed_orig_mps << ","
                                   << sample.lidar_lin_speed_nondeg_mps << ","
                                   << sample.complementary_odom_lin_speed_nondeg_mps << ","
                                   << sample.lidar_lin_speed_proj_scale1_mps << ","
                                   << sample.lidar_lin_speed_after_scale_mps << ","
                                   << sample.dt_scan_s << ","
                                   << sample.dt_scale_odom_interval_s << ","
                                   << sample.dt_scale_lidar_interval_s << ","
                                   << sample.dt_scale_odom_pair_interval_s << ","
                                   << sample.dt_immediate_odom_pair_interval_s << "\n";
}

void LiliDiagnostics::recordComplementaryOdomTwistCsv(const ComplementaryOdomTwistDebugSample &sample)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!complementary_odom_twist_csv_.is_open())
        return;

    complementary_odom_twist_csv_ << std::fixed << std::setprecision(6)
                                   << sample.stamp_sec << ","
                                   << sample.lidar_prev_stamp_s << ","
                                   << sample.lidar_curr_stamp_s << ","
                                   << sample.odom_prev_stamp_s << ","
                                   << sample.odom_curr_stamp_s << ","
                                   << sample.odom_queue_size << ","
                                   << sample.odom_samples_between << ","
                                   << sample.dt_complementary_s << ","
                                   << sample.prev_match_abs_dt_s << ","
                                   << sample.curr_match_abs_dt_s << ","
                                   << sample.lin_norm << ","
                                   << sample.ang_norm << "\n";
}

void LiliDiagnostics::recordScaleReplayFrameCsv(const ScaleReplayFrameSample &sample)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!scale_replay_frames_csv_.is_open())
        return;

    scale_replay_frames_csv_ << std::fixed << std::setprecision(9)
                             << sample.stamp_sec << ","
                             << sample.lidar_prev_stamp_s << ","
                             << sample.dt_scan_s << ","
                             << (sample.degeneracy_detected ? 1 : 0) << ","
                             << (sample.has_degeneracy_basis ? 1 : 0) << ","
                             << (sample.has_complementary_twist ? 1 : 0) << ","
                             << (sample.override_applied_to_state ? 1 : 0) << ","
                             << sample.scale_applied << ","
                             << sample.basis_size;
    for (const auto &twist : sample.basis_twists)
        for (const double v : twist)
            scale_replay_frames_csv_ << "," << v;
    for (const double v : sample.lidar_increment_twist)
        scale_replay_frames_csv_ << "," << v;
    for (const double v : sample.complementary_twist)
        scale_replay_frames_csv_ << "," << v;
    scale_replay_frames_csv_ << "," << sample.dt_complementary_s
                             << "," << sample.odom_prev_stamp_s
                             << "," << sample.odom_curr_stamp_s;
    for (const double v : sample.pose_prev)
        scale_replay_frames_csv_ << "," << v;
    for (const double v : sample.pose_optimized)
        scale_replay_frames_csv_ << "," << v;
    for (const double v : sample.pose_effective)
        scale_replay_frames_csv_ << "," << v;
    scale_replay_frames_csv_ << "\n";
}

void LiliDiagnostics::recordComplementaryOdomStreamTum(const TumPoseSample &sample)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!complementary_odom_stream_tum_.is_open())
        return;

    complementary_odom_stream_tum_ << std::fixed << std::setprecision(9)
                                   << sample.stamp_sec << " "
                                   << sample.tx << " "
                                   << sample.ty << " "
                                   << sample.tz << " "
                                   << sample.qx << " "
                                   << sample.qy << " "
                                   << sample.qz << " "
                                   << sample.qw << "\n";
}

void LiliDiagnostics::recordComplementaryOdomMeta(const ComplementaryOdomMetaSample &sample)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!diagnostics_output_policy_.write_files_master ||
        !diagnostics_output_policy_.write_scale_replay_frames)
        return;

    std::ofstream meta((run_dir_ / "complementary_odom_meta.yaml").string(), std::ios::out);
    if (!meta.is_open())
        return;

    meta << std::fixed << std::setprecision(9)
         << "# Complementary odometry source description for offline replay.\n"
         << "# complementary_odom_stream.tum holds the raw poses this describes.\n"
         << "T_complementary_to_lidar:\n"
         << "  translation: [" << sample.extrinsic_translation[0] << ", "
         << sample.extrinsic_translation[1] << ", "
         << sample.extrinsic_translation[2] << "]\n"
         << "  rotation_quat_xyzw: [" << sample.extrinsic_rotation_xyzw[0] << ", "
         << sample.extrinsic_rotation_xyzw[1] << ", "
         << sample.extrinsic_rotation_xyzw[2] << ", "
         << sample.extrinsic_rotation_xyzw[3] << "]\n"
         << "source_frame: \"" << sample.source_frame << "\"\n"
         << "lidar_frame: \"" << sample.lidar_frame << "\"\n"
         << "translation_scale_applied_online: " << sample.translation_scale_applied_online << "\n";
}

void LiliDiagnostics::publishWarning(const std::string &message)
{
    if (!warnings_pub_)
        return;

    const double stamp_sec = node_->now().seconds();

    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (warning_log_.is_open())
            warning_log_ << std::fixed << std::setprecision(6) << stamp_sec << " " << message << "\n";
    }

    std_msgs::msg::String msg;
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(6);
    ss << "{";
    ss << "\"stamp_sec\":" << stamp_sec << ",";
    ss << "\"message\":\"" << message << "\"";
    ss << "}";
    msg.data = ss.str();
    warnings_pub_->publish(msg);
    logEvent(message);
}

void LiliDiagnostics::recordTimeDelta(double delta_s)
{
    std::lock_guard<std::mutex> lock(mutex_);
    const double stamp_sec = node_->now().seconds();

    if (time_deltas_csv_.is_open())
        time_deltas_csv_ << std::fixed << std::setprecision(6)
                         << stamp_sec << ","
                         << delta_s << "\n";

    if (time_deltas_pub_)
    {
        std_msgs::msg::String msg;
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(6);
        ss << "{";
        ss << "\"stamp_sec\":" << stamp_sec << ",";
        ss << "\"delta_s\":" << delta_s;
        ss << "}";
        msg.data = ss.str();
        time_deltas_pub_->publish(msg);
    }
}

void LiliDiagnostics::recordTranslationPrediction(double delta_m)
{
    std::lock_guard<std::mutex> lock(mutex_);
    max_translation_delta_last_batch_m_ = std::max(max_translation_delta_last_batch_m_, delta_m);
    last_prediction_delta_m_ = delta_m;
}

void LiliDiagnostics::recordFrameMetrics(double stamp_sec,
                                          double time_delta_s,
                                          double last_time_delta_s,
                                          double prediction_delta_m,
                                          double optimized_delta_m,
                                          double estimated_velocity_mps)
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (frame_metrics_csv_.is_open())
        frame_metrics_csv_ << std::fixed << std::setprecision(6)
                           << stamp_sec << ","
                           << time_delta_s << ","
                           << last_time_delta_s << ","
                           << prediction_delta_m << ","
                           << optimized_delta_m << ","
                           << estimated_velocity_mps << "\n";

    if (frame_metrics_pub_)
    {
        std_msgs::msg::String msg;
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(3);

        ss << "{";
        ss << "\"stamp_sec\":" << stamp_sec << ",";
        ss << "\"time_delta_s\":" << time_delta_s << ",";
        ss << "\"last_time_delta_s\":" << last_time_delta_s << ",";
        ss << "\"prediction_delta_m\":" << prediction_delta_m << ",";
        ss << "\"optimized_delta_m\":" << optimized_delta_m << ",";
        ss << "\"estimated_velocity_mps\":" << estimated_velocity_mps;
        ss << "}";
        msg.data = ss.str();
        frame_metrics_pub_->publish(msg);
    }
}

void LiliDiagnostics::recordDegeneracyTelemetry(
    double stamp_sec, 
    const std::string &module_name, 
    bool is_degenerate, 
    const std::vector<Eigen::Matrix<float, 6, 1>> &twists)
{
    std::lock_guard<std::mutex> lock(mutex_);

    // 1. Write to CSV
    if (degeneracy_metrics_csv_.is_open()) {
        degeneracy_metrics_csv_ << std::fixed << std::setprecision(6) << stamp_sec << ","
                                << module_name << "," << (is_degenerate ? "1" : "0") << ",\"";
        for (size_t i = 0; i < twists.size(); ++i) {
            for (int j = 0; j < 6; ++j) {
                degeneracy_metrics_csv_ << std::setprecision(4) << twists[i](j) << (j < 5 ? "|" : "");
            }
            if (i < twists.size() - 1) degeneracy_metrics_csv_ << ";";
        }
        degeneracy_metrics_csv_ << "\"\n";
    }

    // 2. Publish JSON via ROS2
    if (degeneracy_metrics_pub_) {
        std_msgs::msg::String msg;
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(6) << "{\"stamp_sec\":" << stamp_sec 
           << ",\"module\":\"" << module_name << "\",\"is_degenerate\":" << (is_degenerate ? "true" : "false") << ",\"twists\":[";
           
        for (size_t i = 0; i < twists.size(); ++i) {
            ss << "[";
            for (int j = 0; j < 6; ++j) {
                ss << std::setprecision(4) << twists[i](j) << (j < 5 ? "," : "");
            }
            ss << "]";
            if (i < twists.size() - 1) ss << ",";
        }
        ss << "]}";
        msg.data = ss.str();
        degeneracy_metrics_pub_->publish(msg);
    }
}

void LiliDiagnostics::recordJacobianDegeneracyTelemetry(
    double stamp_sec,
    const std::string &module_name,
    const lio::JacobianDegeneracyInfo &info)
{
    std::lock_guard<std::mutex> lock(mutex_);

    auto serializeFloatArray = [](const auto &arr) {
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(6);
        for (size_t i = 0; i < arr.size(); ++i)
        {
            ss << arr[i];
            if (i + 1 < arr.size()) ss << "|";
        }
        return ss.str();
    };

    auto serializeIntArray = [](const auto &arr) {
        std::ostringstream ss;
        for (size_t i = 0; i < arr.size(); ++i)
        {
            ss << arr[i];
            if (i + 1 < arr.size()) ss << "|";
        }
        return ss.str();
    };

    const std::string eigenvalues = serializeFloatArray(info.eigenvalues);
    const std::string thresholds = serializeFloatArray(info.thresholds);
    const std::string zeroedModes = serializeIntArray(info.zeroed_modes);
    const std::string eigenvectors = serializeFloatArray(info.eigenvectors);

    if (jacobian_degeneracy_metrics_csv_.is_open())
    {
        jacobian_degeneracy_metrics_csv_ << std::fixed << std::setprecision(6)
                                         << stamp_sec << ","
                                         << module_name << ","
                                         << (info.computed ? 1 : 0) << ","
                                         << (info.is_degenerate ? 1 : 0) << ","
                                         << info.selected_correspondences << ","
                                         << "\"" << info.reason << "\"," 
                                         << "\"" << eigenvalues << "\"," 
                                         << "\"" << thresholds << "\"," 
                                         << "\"" << zeroedModes << "\"," 
                                         << "\"" << eigenvectors << "\"\n";
    }

    if (degeneracy_metrics_pub_)
    {
        std_msgs::msg::String msg;
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(6);
        ss << "{";
        ss << "\"stamp_sec\":" << stamp_sec << ",";
        ss << "\"module\":\"" << module_name << "\",";
        ss << "\"type\":\"jacobian\",";
        ss << "\"computed\":" << (info.computed ? "true" : "false") << ",";
        ss << "\"is_degenerate\":" << (info.is_degenerate ? "true" : "false") << ",";
        ss << "\"selected_correspondences\":" << info.selected_correspondences << ",";
        ss << "\"reason\":\"" << info.reason << "\",";
        ss << "\"eigenvalues\":\"" << eigenvalues << "\",";
        ss << "\"thresholds\":\"" << thresholds << "\",";
        ss << "\"zeroed_modes\":\"" << zeroedModes << "\",";
        ss << "\"eigenvectors\":\"" << eigenvectors << "\"";
        ss << "}";
        msg.data = ss.str();
        degeneracy_metrics_pub_->publish(msg);
    }
}

void LiliDiagnostics::recordPerturbationDegeneracyTelemetry(
    double stamp_sec,
    bool detected,
    size_t raw_twist_count,
    size_t pca_basis_count,
    size_t sparsified_basis_count,
    bool failed,
    const std::string &fail_reason)
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (perturbation_degeneracy_metrics_csv_.is_open())
    {
        perturbation_degeneracy_metrics_csv_ << std::fixed << std::setprecision(6)
                                             << stamp_sec << ","
                                             << (detected ? 1 : 0) << ","
                                             << raw_twist_count << ","
                                             << pca_basis_count << ","
                                             << sparsified_basis_count << ","
                                             << (failed ? 1 : 0) << ","
                                             << "\"" << fail_reason << "\"\n";
    }

    if (degeneracy_metrics_pub_)
    {
        std_msgs::msg::String msg;
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(6);
        ss << "{";
        ss << "\"stamp_sec\":" << stamp_sec << ",";
        ss << "\"type\":\"perturbation\",";
        ss << "\"detected\":" << (detected ? "true" : "false") << ",";
        ss << "\"raw_twist_count\":" << raw_twist_count << ",";
        ss << "\"pca_basis_count\":" << pca_basis_count << ",";
        ss << "\"sparsified_basis_count\":" << sparsified_basis_count << ",";
        ss << "\"failed\":" << (failed ? "true" : "false") << ",";
        ss << "\"fail_reason\":\"" << fail_reason << "\"";
        ss << "}";
        msg.data = ss.str();
        degeneracy_metrics_pub_->publish(msg);
    }
}

void LiliDiagnostics::recordOdomTrajectoryTum(const TumPoseSample &sample)
{
    if (!trajectory_output_policy_.write_odom_trajectory_tum || !odom_trajectory_tum_.is_open())
        return;

    std::lock_guard<std::mutex> lock(mutex_);
    odom_trajectory_tum_ << std::fixed << std::setprecision(9)
                         << sample.stamp_sec << " "
                         << sample.tx << " "
                         << sample.ty << " "
                         << sample.tz << " "
                         << sample.qx << " "
                         << sample.qy << " "
                         << sample.qz << " "
                         << sample.qw << "\n";
}

double LiliDiagnostics::getLastPredictionDelta() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return last_prediction_delta_m_;
}

std::filesystem::path LiliDiagnostics::createRunDirectory(const std::string &base_dir, const std::string &run_suffix)
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
    if (!run_suffix.empty()) stamp << "_" << run_suffix;

    std::filesystem::path path = std::filesystem::path(expanded) / stamp.str();
    std::filesystem::create_directories(path);

    std::filesystem::path latest_link = std::filesystem::path(expanded) / "latest";
    std::error_code ec;
    std::filesystem::remove(latest_link, ec);
    ec.clear();
    std::filesystem::create_directory_symlink(path.filename(), latest_link, ec);

    return path;
}

void LiliDiagnostics::dumpActiveParameters()
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

void LiliDiagnostics::publishTelemetry()
{
    if (!telemetry_pub_)
        return;

    std_msgs::msg::String msg;
    std::ostringstream ss;

    double lidar_delta = -1.0;
    double gps_delta = -1.0;
    std::unordered_map<std::string, double> moving_copy;
    double max_translation_delta_last_batch_m = 0.0;

    {
        std::lock_guard<std::mutex> lock(mutex_);
        const double now_sec = node_->now().seconds();
        if (last_lidar_update_.nanoseconds() > 0)
            lidar_delta = now_sec - last_lidar_update_.seconds();
        if (last_gps_update_.nanoseconds() > 0)
            gps_delta = now_sec - last_gps_update_.seconds();
        moving_copy = moving_avg_ms_;
        max_translation_delta_last_batch_m = max_translation_delta_last_batch_m_;
        max_translation_delta_last_batch_m_ = 0.0;
    }

    ss << "{";
    ss << "\"time_since_last_lidar_s\":" << lidar_delta << ",";
    ss << "\"time_since_last_gps_s\":" << gps_delta << ",";
    ss << "\"max_translation_delta_last_batch_m\":" << max_translation_delta_last_batch_m << ",";
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
    telemetry_pub_->publish(msg);

    if (telemetry_csv_.is_open())
        telemetry_csv_ << std::fixed << std::setprecision(6)
                       << node_->now().seconds() << ","
                       << lidar_delta << ","
                       << gps_delta << "\n";
}
