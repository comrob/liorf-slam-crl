#include "liorf_diagnostics.h"

LiorfDiagnostics::LiorfDiagnostics(
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

    dumpActiveParameters();

    telemetry_pub_ = node_->create_publisher<std_msgs::msg::String>(topic, qos);
    timing_stats_pub_ = node_->create_publisher<std_msgs::msg::String>("/liorf/debug/timing_stats", qos);
    time_deltas_pub_ = node_->create_publisher<std_msgs::msg::String>("/liorf/debug/time_deltas", qos);
    event_pub_ = node_->create_publisher<std_msgs::msg::String>("/liorf/debug/event", qos);
    warnings_pub_ = node_->create_publisher<std_msgs::msg::String>("/liorf/debug/warnings", qos);
    frame_metrics_pub_ = node_->create_publisher<std_msgs::msg::String>("/liorf/debug/frame_metrics", qos);
    degeneracy_metrics_pub_ = node_->create_publisher<std_msgs::msg::String>("/liorf/debug/degeneracy_metrics", qos);

    const double hz = std::max(0.1, publish_hz);
    const auto period_ms = std::chrono::milliseconds(static_cast<int>(1000.0 / hz));
    diagnostics_timer_ = node_->create_wall_timer(period_ms, std::bind(&LiorfDiagnostics::publishTelemetry, this));

    RCLCPP_INFO(node_->get_logger(), "[LIORF_DIAG] log directory: %s", run_dir_.string().c_str());
}

LiorfDiagnostics::~LiorfDiagnostics()
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
    if (odom_trajectory_tum_.is_open())
        odom_trajectory_tum_.flush();
    if (diagnostics_files_enabled && jacobian_degeneracy_metrics_csv_.is_open())
        jacobian_degeneracy_metrics_csv_.flush();
    if (diagnostics_files_enabled && perturbation_degeneracy_metrics_csv_.is_open())
        perturbation_degeneracy_metrics_csv_.flush();
}

void LiorfDiagnostics::markLidarUpdate(const rclcpp::Time &stamp)
{
    std::lock_guard<std::mutex> lock(mutex_);
    last_lidar_update_ = stamp;
}

void LiorfDiagnostics::markGpsUpdate(const rclcpp::Time &stamp)
{
    std::lock_guard<std::mutex> lock(mutex_);
    last_gps_update_ = stamp;
}

void LiorfDiagnostics::recordSlice(const std::string &stage, double elapsed_ms)
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

void LiorfDiagnostics::logEvent(const std::string &message)
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

void LiorfDiagnostics::logEventThrottle(const std::string &key, double period_sec, const std::string &message)
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

void LiorfDiagnostics::publishWarning(const std::string &message)
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

void LiorfDiagnostics::recordTimeDelta(double delta_s)
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

void LiorfDiagnostics::recordTranslationPrediction(double delta_m)
{
    std::lock_guard<std::mutex> lock(mutex_);
    max_translation_delta_last_batch_m_ = std::max(max_translation_delta_last_batch_m_, delta_m);
    last_prediction_delta_m_ = delta_m;
}

void LiorfDiagnostics::recordFrameMetrics(double stamp_sec,
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

void LiorfDiagnostics::recordDegeneracyTelemetry(
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

void LiorfDiagnostics::recordJacobianDegeneracyTelemetry(
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

void LiorfDiagnostics::recordPerturbationDegeneracyTelemetry(
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

void LiorfDiagnostics::recordOdomTrajectoryTum(const TumPoseSample &sample)
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

double LiorfDiagnostics::getLastPredictionDelta() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return last_prediction_delta_m_;
}

std::filesystem::path LiorfDiagnostics::createRunDirectory(const std::string &base_dir, const std::string &run_suffix)
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

void LiorfDiagnostics::dumpActiveParameters()
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

void LiorfDiagnostics::publishTelemetry()
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
