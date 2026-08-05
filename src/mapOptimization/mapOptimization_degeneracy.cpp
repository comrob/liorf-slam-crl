#include "mapOptimization/mapOptimization.hpp"

// This translation unit owns perturbation-based degeneracy detection
// orchestration and the complementary odometry aiding used to compensate
// degenerate directions (input handling, extrinsics resolution, and the
// state override). Related visualization lives in mapOptimization_publish.cpp.

void mapOptimization::complementaryOdomHandler(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // Log the raw pose before translationScale is baked in, so that an offline
    // replay owns that factor and can re-synchronize this stream itself.
    if (diagnostics)
    {
        TumPoseSample stream_sample;
        stream_sample.stamp_sec = ROS_TIME(msg->header.stamp);
        stream_sample.tx = msg->pose.pose.position.x;
        stream_sample.ty = msg->pose.pose.position.y;
        stream_sample.tz = msg->pose.pose.position.z;
        stream_sample.qx = msg->pose.pose.orientation.x;
        stream_sample.qy = msg->pose.pose.orientation.y;
        stream_sample.qz = msg->pose.pose.orientation.z;
        stream_sample.qw = msg->pose.pose.orientation.w;
        diagnostics->recordComplementaryOdomStreamTum(stream_sample);
    }

    std::lock_guard<std::mutex> lock(complementaryOdomMutex);
    nav_msgs::msg::Odometry scaled_msg = *msg;
    scaled_msg.pose.pose.position.x *= complementaryOdom.translationScale;
    scaled_msg.pose.pose.position.y *= complementaryOdom.translationScale;
    scaled_msg.pose.pose.position.z *= complementaryOdom.translationScale;
    complementaryOdomQueue.push_back(scaled_msg);

    const size_t odomBufferSizeCap = static_cast<size_t>(
        std::max(5000, 100 * std::max(1, complementaryOdom.scaleBaselineFrameLag)));
    while (complementaryOdomQueue.size() > odomBufferSizeCap)
    {
        complementaryOdomQueue.pop_front();
    }

    // Prune messages older than 5.0 seconds from the current processing timestamp
    while (!complementaryOdomQueue.empty() && 
           ROS_TIME(complementaryOdomQueue.front().header.stamp) < (timeLaserInfoCur - 5.0))
    {
        complementaryOdomQueue.pop_front();
    }
}

bool mapOptimization::resolveComplementaryOdomExtrinsics(const std::string &msgFrameId)
{
    if (complementaryOdomTfResolved) return true;

    // Record the resolved extrinsic once, so that an offline replay can push an
    // alternative odometry source through the same transform.
    const auto recordMeta = [this](const std::string &source_frame)
    {
        if (!diagnostics)
            return;

        ComplementaryOdomMetaSample meta;
        const Eigen::Quaternionf q(Eigen::Matrix3f(T_complementary_to_lidar.block<3, 3>(0, 0)));
        meta.extrinsic_translation = {static_cast<double>(T_complementary_to_lidar(0, 3)),
                                      static_cast<double>(T_complementary_to_lidar(1, 3)),
                                      static_cast<double>(T_complementary_to_lidar(2, 3))};
        meta.extrinsic_rotation_xyzw = {static_cast<double>(q.x()), static_cast<double>(q.y()),
                                        static_cast<double>(q.z()), static_cast<double>(q.w())};
        meta.source_frame = source_frame;
        meta.lidar_frame = lidarFrame;
        meta.translation_scale_applied_online = static_cast<double>(complementaryOdom.translationScale);
        diagnostics->recordComplementaryOdomMeta(meta);
    };

    if (!complementaryOdom.autoLookupLidarToTf)
    {
        T_complementary_to_lidar = Eigen::Matrix4f::Identity();
        T_complementary_to_lidar.block<3, 3>(0, 0) = complementaryOdom.extRot.cast<float>();
        T_complementary_to_lidar.block<3, 1>(0, 3) = complementaryOdom.extTrans.cast<float>();
        complementaryOdomTfResolved = true;
        recordMeta(complementaryOdom.frame.empty() ? msgFrameId : complementaryOdom.frame);
        return true;
    }

    std::string target_frame = complementaryOdom.frame.empty() ? msgFrameId : complementaryOdom.frame;
    if (target_frame.empty() || lidarFrame.empty()) return false;

    try
    {
        geometry_msgs::msg::TransformStamped tf_msg =
            runtimeTfCoordinator->getTfBuffer()->lookupTransform(lidarFrame, target_frame, rclcpp::Time(0));

        Eigen::Isometry3d tf_iso = tf2::transformToEigen(tf_msg);
        T_complementary_to_lidar = tf_iso.matrix().cast<float>();
        complementaryOdomTfResolved = true;
        RCLCPP_INFO_STREAM(get_logger(), "[COMPLEMENTARY_ODOM_TF] Resolved T_complementary_to_lidar from TF tree.");
        recordMeta(target_frame);
        return true;
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, 
                             "[COMPLEMENTARY_ODOM_TF] Waiting for TF from %s to %s: %s", 
                             target_frame.c_str(), lidarFrame.c_str(), ex.what());
        return false;
    }
}

void mapOptimization::runDegeneracyDetectionAndCompensation()
{
    auto localMapForDegeneracy = mappingBackend->getLocalMapCloud();
    if (!localMapForDegeneracy || localMapForDegeneracy->empty())
        localMapForDegeneracy = laserCloudSurfLastDS;

    degeneracyDetector->evalDegeneracyPerturbation(
        transformTobeMapped,
        laserCloudSurfLastDS,
        localMapForDegeneracy,
        mappingBackend);

    const auto perturbationTwists = degeneracyDetector->getTwistsPerturbationsDegeneracy();
    degeneracyDetector->extractBasisFromTwists(perturbationTwists, laserCloudSurfLastDS);

    const auto rawTwists = degeneracyDetector->getRawTwists();
    const auto pcaBasis = degeneracyDetector->getPcaBasis();
    const auto sparsifiedBasis = degeneracyDetector->getSparsifiedBasis();
    const bool degeneracyDetected = degeneracyDetector->isDegeneracyDetected();
    const Eigen::Affine3f poseBeforeDegeneracyOverride = trans2Affine3f(transformTobeMapped);

    // LOG THE FAILURE REASON IF ANY
    if (degeneracyDetector->isFailed()) {
        RCLCPP_WARN(this->get_logger(), "Degeneracy failed: %s", degeneracyDetector->getFailReason().c_str());
    }

    if (diagnostics)
    {
        diagnostics->recordPerturbationDegeneracyTelemetry(
            timeLaserInfoCur,
            degeneracyDetected,
            rawTwists.size(),
            pcaBasis.size(),
            sparsifiedBasis.size(),
            degeneracyDetector->isFailed(),
            degeneracyDetector->getFailReason());
    }

    if (degeneracyDetected)
    {
        std::ostringstream oss;
        oss << "[DEGENERACY_DETECTED]"
            << " raw=" << rawTwists.size()
            << " pca=" << pcaBasis.size()
            << " basis=" << sparsifiedBasis.size();
        if (diagnostics)
            diagnostics->logEventThrottle("degeneracy_detected", 1.0, oss.str());
    }

    const double dt_scan = (curTimeDiff > 1e-5) ? curTimeDiff : 0.1;
    applyDegeneracyStateOverride(dt_scan, degeneracyDetected);

    const Eigen::Affine3f poseAfterDegeneracyOverride = trans2Affine3f(transformTobeMapped);

    publishPerturbationDebugProducts(
        degeneracyDetector->getPerturbedScans(),
        degeneracyDetector->getAlignedScans(),
        degeneracyDetector->getPerturbedPoses(),
        degeneracyDetector->getAlignedPoses(),
        degeneracyDetector->getOptimizationPaths(),
        timeLaserInfoStamp,
        poseBeforeDegeneracyOverride,
        poseAfterDegeneracyOverride);

    publishTwistMarkers(pubDegeneracyRaw, "raw", rawTwists, timeLaserInfoStamp,
                        1.0, 0.0, 0.0,   1.0, 1.0, 0.0); // Red/Yellow
    publishTwistMarkers(pubDegeneracyPCA, "pca", pcaBasis, timeLaserInfoStamp,
                        0.0, 0.5, 1.0,   0.0, 1.0, 1.0); // Blue/Cyan
    publishTwistMarkers(pubDegeneracyBasis, "basis", sparsifiedBasis, timeLaserInfoStamp,
                        0.0, 1.0, 0.0,   1.0, 0.0, 1.0); // Green/Magenta
    publishDegeneracyPaths(pubDegeneracyPaths, "degeneracy_paths", sparsifiedBasis, timeLaserInfoStamp);
}

bool mapOptimization::prepareDegeneracyOrthoBasis(std::vector<TwistVector> &orthoBasis)
{
    auto basis = degeneracyDetector->getSparsifiedBasis();
    if (basis.empty())
        return false;

    orthoBasis = degeneracyDetector->orthonormalizeBasis(basis);
    return !orthoBasis.empty();
}

bool mapOptimization::inferComplementaryOdomTwist(double dt_scan,
                                                  TwistVector &xi_complementary_lidar,
                                                  ComplementaryOdomMatchInfo *match_info,
                                                  double lidar_prev_stamp_override_s,
                                                  double lidar_curr_stamp_override_s)
{
    xi_complementary_lidar = TwistVector::Zero();
    if (degeneracyDetection.compensation_source != "complementary_odom")
        return false;

    std::lock_guard<std::mutex> lock(complementaryOdomMutex);
    if (complementaryOdomQueue.size() < 2)
    {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "[COMPLEMENTARY_ODOM_MATCH] insufficient odom samples: queue_size=%zu",
                             complementaryOdomQueue.size());
        return false;
    }

    const double lidar_dt = (curTimeDiff > 1e-5) ? curTimeDiff : dt_scan;
    const double prev_lidar_time = std::isfinite(lidar_prev_stamp_override_s)
                                       ? lidar_prev_stamp_override_s
                                       : (timeLaserInfoCur - lidar_dt);
    const double curr_lidar_time = std::isfinite(lidar_curr_stamp_override_s)
                                       ? lidar_curr_stamp_override_s
                                       : timeLaserInfoCur;

    const auto closestIdxToTime = [this](double target_time,
                                         int exclude_idx,
                                         double *best_abs_dt_out) -> int
    {
        int best_idx = -1;
        double best_abs_dt = std::numeric_limits<double>::max();
        for (int i = 0; i < static_cast<int>(complementaryOdomQueue.size()); ++i)
        {
            if (i == exclude_idx)
                continue;

            const double msg_time = ROS_TIME(complementaryOdomQueue[i].header.stamp);
            const double abs_dt = std::abs(msg_time - target_time);
            if (abs_dt < best_abs_dt)
            {
                best_abs_dt = abs_dt;
                best_idx = i;
            }
        }

        if (best_abs_dt_out)
        {
            *best_abs_dt_out = (best_idx >= 0) ? best_abs_dt : std::numeric_limits<double>::infinity();
        }

        return best_idx;
    };

    double prev_closest_abs_dt = std::numeric_limits<double>::infinity();
    double curr_closest_abs_dt = std::numeric_limits<double>::infinity();
    const int idx_prev_near = closestIdxToTime(prev_lidar_time, -1, &prev_closest_abs_dt);
    const int idx_curr_near = closestIdxToTime(curr_lidar_time, idx_prev_near, &curr_closest_abs_dt);
    if (idx_prev_near < 0 || idx_curr_near < 0)
    {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "[COMPLEMENTARY_ODOM_MATCH] no nearest pair found: prev_abs_dt=%.3f curr_abs_dt=%.3f queue_size=%zu",
                             prev_closest_abs_dt,
                             curr_closest_abs_dt,
                             complementaryOdomQueue.size());
        return false;
    }

    const double t_prev_near = ROS_TIME(complementaryOdomQueue[idx_prev_near].header.stamp);
    const double t_curr_near = ROS_TIME(complementaryOdomQueue[idx_curr_near].header.stamp);
    const double prev_abs_dt = std::abs(t_prev_near - prev_lidar_time);
    const double curr_abs_dt = std::abs(t_curr_near - curr_lidar_time);
    if (prev_abs_dt > 0.25 || curr_abs_dt > 0.25)
    {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "[COMPLEMENTARY_ODOM_MATCH] closest pair too far: prev_abs_dt=%.3f curr_abs_dt=%.3f threshold=0.250 prev_target=%.3f curr_target=%.3f prev_odom=%.3f curr_odom=%.3f",
                             prev_abs_dt,
                             curr_abs_dt,
                             prev_lidar_time,
                             curr_lidar_time,
                             t_prev_near,
                             t_curr_near);
        return false;
    }

    int idx1 = idx_prev_near;
    int idx2 = idx_curr_near;
    if (ROS_TIME(complementaryOdomQueue[idx1].header.stamp) > ROS_TIME(complementaryOdomQueue[idx2].header.stamp))
        std::swap(idx1, idx2);

    if (!resolveComplementaryOdomExtrinsics(complementaryOdomQueue[idx2].header.frame_id))
        return false;

    const double odom_prev_stamp_s = ROS_TIME(complementaryOdomQueue[idx1].header.stamp);
    const double odom_curr_stamp_s = ROS_TIME(complementaryOdomQueue[idx2].header.stamp);
    const double dt_complementary = odom_curr_stamp_s - odom_prev_stamp_s;
    if (dt_complementary < 1e-3)
        return false;

    if (match_info)
    {
        match_info->lidar_prev_stamp_s = prev_lidar_time;
        match_info->lidar_curr_stamp_s = curr_lidar_time;
        match_info->odom_prev_stamp_s = odom_prev_stamp_s;
        match_info->odom_curr_stamp_s = odom_curr_stamp_s;
        match_info->dt_complementary_s = dt_complementary;
        match_info->prev_match_abs_dt_s = prev_abs_dt;
        match_info->curr_match_abs_dt_s = curr_abs_dt;
        match_info->odom_queue_size = static_cast<int>(complementaryOdomQueue.size());
        match_info->odom_samples_between = std::abs(idx2 - idx1) + 1;
    }

    Eigen::Isometry3d iso1;
    Eigen::Isometry3d iso2;
    tf2::fromMsg(complementaryOdomQueue[idx1].pose.pose, iso1);
    tf2::fromMsg(complementaryOdomQueue[idx2].pose.pose, iso2);
    const Eigen::Matrix4f T1 = iso1.matrix().cast<float>();
    const Eigen::Matrix4f T2 = iso2.matrix().cast<float>();
    const Eigen::Matrix4f T_add_delta = T1.inverse() * T2;

    // Conjugate motion into LiDAR frame: T_l_delta = T_l_a * T_a_delta * T_a_l
    const Eigen::Matrix4f T_complementary_lidar_delta = T_complementary_to_lidar * T_add_delta * T_complementary_to_lidar.inverse();
    xi_complementary_lidar = matrixToTwist(T_complementary_lidar_delta, static_cast<float>(dt_complementary));

    if (diagnostics)
    {
        ComplementaryOdomTwistDebugSample sample;
        sample.stamp_sec = timeLaserInfoCur;
        sample.lidar_prev_stamp_s = prev_lidar_time;
        sample.lidar_curr_stamp_s = curr_lidar_time;
        sample.odom_prev_stamp_s = odom_prev_stamp_s;
        sample.odom_curr_stamp_s = odom_curr_stamp_s;
        sample.odom_queue_size = static_cast<int>(complementaryOdomQueue.size());
        sample.odom_samples_between = std::abs(idx2 - idx1) + 1;
        sample.dt_complementary_s = dt_complementary;
        sample.prev_match_abs_dt_s = prev_abs_dt;
        sample.curr_match_abs_dt_s = curr_abs_dt;
        sample.lin_norm = xi_complementary_lidar.head<3>().norm();
        sample.ang_norm = xi_complementary_lidar.tail<3>().norm();
        diagnostics->recordComplementaryOdomTwistCsv(sample);
    }

    return true;
}

Eigen::Affine3f mapOptimization::buildScaledComplementaryPrediction(const Eigen::Affine3f &T_previous,
                                                                    const Eigen::Affine3f &T_optimized,
                                                                    const std::vector<TwistVector> &orthoBasis,
                                                                    const TwistVector &xi_complementary_lidar,
                                                                    const ComplementaryOdomMatchInfo &match_info,
                                                                    bool estimator_mode_active,
                                                                    bool scale_applied_to_state,
                                                                    double dt_scan,
                                                                    double scale_applied,
                                                                    Eigen::Vector3f &t_lidar_nondeg_map,
                                                                    Eigen::Vector3f &t_complementary_nondeg_map,
                                                                    Eigen::Vector3f &t_lidar_nondeg_proj_on_complementary_map,
                                                                    Eigen::Vector3f &t_complementary_raw_map,
                                                                    Eigen::Vector3f &t_complementary_scaled_map,
                                                                    Eigen::Affine3f &T_complementary_unscaled_abs,
                                                                    Eigen::Affine3f &T_complementary_scaled_abs)
{
    const double dt_prediction = (match_info.dt_complementary_s > 1e-5)
                                     ? match_info.dt_complementary_s
                                     : dt_scan;
    const Eigen::Matrix4f T_lidar_complementary_predicted =
        expMap(xi_complementary_lidar, static_cast<float>(dt_prediction));

    Eigen::Vector3f t_lidar =
        (T_previous.matrix().inverse() * T_optimized.matrix()).block<3, 1>(0, 3);
    Eigen::Vector3f t_complementary = T_lidar_complementary_predicted.block<3, 1>(0, 3);

    // Optional 2D mode for degeneracy-direction projection only.
    Eigen::Vector3f t_lidar_scale = t_lidar;
    Eigen::Vector3f t_complementary_scale = t_complementary;
    if (complementaryOdom.ignore_dz)
    {
        t_lidar_scale.z() = 0.0f;
        t_complementary_scale.z() = 0.0f;
    }

    const Eigen::Vector3f t_lidar_nondeg = t_lidar_scale - projectOntoBasisTranslation(t_lidar_scale, orthoBasis);
    const Eigen::Vector3f t_complementary_nondeg = t_complementary_scale - projectOntoBasisTranslation(t_complementary_scale, orthoBasis);
    // Project lidar displacement onto complementary odometry displacement.
    const float complementaryNondegNorm = t_complementary_nondeg.norm();
    const float lidarNondegNormUnprojected = t_lidar_nondeg.norm();
    Eigen::Vector3f t_complementary_nondeg_unit = Eigen::Vector3f::Zero();
    if (complementaryNondegNorm > 1e-6f)
        t_complementary_nondeg_unit = t_complementary_nondeg / complementaryNondegNorm;

    const Eigen::Vector3f t_lidar_nondeg_proj =
        t_lidar_nondeg.dot(t_complementary_nondeg_unit) * t_complementary_nondeg_unit;
    const float minNondegNorm =
        static_cast<float>(complementaryOdom.scaleMinNonDegenerateSpeed) * static_cast<float>(dt_prediction);

    const float lidarNondegNorm = t_lidar_nondeg_proj.norm();
    const bool gateObservable =
        complementaryOdom.scaleEstimationEnabled &&
        lidarNondegNorm > minNondegNorm &&
        complementaryNondegNorm > minNondegNorm;

    float scaleRatioRaw = std::numeric_limits<float>::quiet_NaN();
    float scaleRatioUnprojectedRaw = std::numeric_limits<float>::quiet_NaN();
    float scaleLsRaw = std::numeric_limits<float>::quiet_NaN();
    float thetaDeg = std::numeric_limits<float>::quiet_NaN();
    if (complementaryNondegNorm > 1e-6f)
    {
        scaleRatioRaw = lidarNondegNorm / complementaryNondegNorm;
        scaleRatioUnprojectedRaw = lidarNondegNormUnprojected / complementaryNondegNorm;

        const float dot = t_complementary_nondeg.dot(t_lidar_nondeg);
        scaleLsRaw = dot / (complementaryNondegNorm * complementaryNondegNorm);
        if (lidarNondegNormUnprojected > 1e-6f)
        {
            const float cosTheta = std::clamp(dot / (complementaryNondegNorm * lidarNondegNormUnprojected), -1.0f, 1.0f);
            thetaDeg = std::acos(cosTheta) * 180.0f / static_cast<float>(M_PI);
        }
    }

    const float scaleApplied = std::isfinite(scale_applied)
                                   ? static_cast<float>(scale_applied)
                                   : 1.0f;

    const Eigen::Matrix3f R_base = T_previous.rotation();
    t_lidar_nondeg_map = R_base * t_lidar_nondeg;
    t_complementary_nondeg_map = R_base * t_complementary_nondeg;
    t_lidar_nondeg_proj_on_complementary_map = R_base * t_lidar_nondeg_proj;
    t_complementary_raw_map = R_base * t_complementary;
    t_complementary_scaled_map = scaleApplied * t_complementary_raw_map;

    // Immediate-pair debug publishing is intentionally disabled.


    const float complementaryOdomLinearSpeedOrig = xi_complementary_lidar.head<3>().norm();
    const float lidarLinearSpeedNondeg = lidarNondegNorm / static_cast<float>(dt_prediction);
    const float complementaryOdomLinearSpeedNondeg = complementaryNondegNorm / static_cast<float>(dt_prediction);
    const float lidarLinearSpeedAfterScale =
        (complementaryNondegNorm * scaleApplied) / static_cast<float>(dt_scan);

    Eigen::Matrix4f T_lidar_complementary_predicted_scaled = T_lidar_complementary_predicted;
    T_lidar_complementary_predicted_scaled.block<3, 1>(0, 3) *= scaleApplied;

    T_complementary_unscaled_abs = Eigen::Affine3f(T_previous.matrix() * T_lidar_complementary_predicted);
    T_complementary_scaled_abs = Eigen::Affine3f(T_previous.matrix() * T_lidar_complementary_predicted_scaled);

    const Eigen::Affine3f T_complementary_odom_scale1(
        T_complementary_scaled_abs.matrix());
    const Eigen::Matrix4f T_diff_scale1 =
        T_optimized.matrix().inverse() * T_complementary_odom_scale1.matrix();
    const TwistVector xi_diff_scale1 = matrixToTwist(T_diff_scale1);
    const TwistVector xi_proj_scale1 = projectOntoBasis(xi_diff_scale1, orthoBasis);
    const Eigen::Matrix4f T_proj_motion_scale1 = expMap(xi_proj_scale1);
    const float lidarLinearSpeedProjScale1 =
        T_proj_motion_scale1.block<3, 1>(0, 3).norm() / static_cast<float>(dt_scan);

    if (diagnostics)
    {
        ComplementaryOdomScaleDebugSample sample;
        sample.stamp_sec = timeLaserInfoCur;
        sample.frame_stamp_s = timeLaserInfoCur;
        sample.lidar_prev_stamp_s = match_info.lidar_prev_stamp_s;
        sample.lidar_curr_stamp_s = match_info.lidar_curr_stamp_s;
        sample.odom_prev_stamp_s = match_info.odom_prev_stamp_s;
        sample.odom_curr_stamp_s = match_info.odom_curr_stamp_s;
        sample.odom_queue_size = match_info.odom_queue_size;
        sample.odom_samples_between = match_info.odom_samples_between;
        sample.odom_span_s = match_info.dt_complementary_s;
        sample.prev_match_abs_dt_s = match_info.prev_match_abs_dt_s;
        sample.curr_match_abs_dt_s = match_info.curr_match_abs_dt_s;
        sample.enabled = complementaryOdom.scaleEstimationEnabled;
        sample.gate_observable = gateObservable;
        sample.scale_instant_raw = scaleRatioRaw;
        sample.scale_instant_raw_clamped = 1.0f;
        sample.scale_instant_raw_safe = 1.0f;
        sample.scale_fallback_history = 1.0f;
        sample.scale_smooth = scaleApplied;
        sample.scale_smooth_ratio = 1.0f;
        sample.scale_smooth_legacy = 1.0f;
        sample.scale_applied = scaleApplied;
        sample.smooth_from_numden_enabled = false;
        sample.estimator_mode = estimator_mode_active ? 1 : 0;
        sample.scale_estimate_updated = false;
        sample.scale_applied_to_state = scale_applied_to_state && std::isfinite(scale_applied);
        sample.scale_ratio_raw = scaleRatioRaw;
        sample.scale_ratio_unprojected_raw = scaleRatioUnprojectedRaw;
        sample.scale_ls_raw = scaleLsRaw;
        sample.theta_deg = thetaDeg;
        sample.lidar_body_dx_m = t_lidar.x();
        sample.lidar_body_dy_m = t_lidar.y();
        sample.lidar_body_dz_m = t_lidar.z();
        sample.complementary_body_dx_m = t_complementary.x();
        sample.complementary_body_dy_m = t_complementary.y();
        sample.complementary_body_dz_m = t_complementary.z();
        sample.lidar_body_nondeg_dx_m = t_lidar_nondeg.x();
        sample.lidar_body_nondeg_dy_m = t_lidar_nondeg.y();
        sample.lidar_body_nondeg_dz_m = t_lidar_nondeg.z();
        sample.complementary_body_nondeg_dx_m = t_complementary_nondeg.x();
        sample.complementary_body_nondeg_dy_m = t_complementary_nondeg.y();
        sample.complementary_body_nondeg_dz_m = t_complementary_nondeg.z();
        sample.lidar_nondeg_unprojected_m = lidarNondegNormUnprojected;
        sample.lidar_nondeg_m = lidarNondegNorm;
        sample.complementary_nondeg_m = complementaryNondegNorm;
        sample.min_nondeg_m = minNondegNorm;
        sample.scale_smoothing_window = 0;
        sample.complementary_odom_lin_speed_orig_mps = complementaryOdomLinearSpeedOrig;
        sample.lidar_lin_speed_nondeg_mps = lidarLinearSpeedNondeg;
        sample.complementary_odom_lin_speed_nondeg_mps = complementaryOdomLinearSpeedNondeg;
        sample.lidar_lin_speed_proj_scale1_mps = lidarLinearSpeedProjScale1;
        sample.lidar_lin_speed_after_scale_mps = lidarLinearSpeedAfterScale;
        sample.dt_scan_s = dt_scan;
        sample.dt_scale_odom_interval_s = match_info.dt_complementary_s;
        sample.dt_scale_lidar_interval_s = match_info.lidar_curr_stamp_s - match_info.lidar_prev_stamp_s;
        sample.dt_scale_odom_pair_interval_s = match_info.dt_complementary_s;
        sample.dt_immediate_odom_pair_interval_s = dt_prediction;
        diagnostics->recordComplementaryOdomScaleCsv(sample);
    }

    return T_complementary_scaled_abs;
}

Eigen::Affine3f mapOptimization::projectDegenerateCorrection(const Eigen::Affine3f &T_optimized,
                                                             const Eigen::Affine3f &T_predicted,
                                                             const std::vector<TwistVector> &orthoBasis)
{
    const Eigen::Matrix4f T_diff = T_optimized.matrix().inverse() * T_predicted.matrix();
    const TwistVector xi_diff = matrixToTwist(T_diff);
    const TwistVector xi_proj = projectOntoBasis(xi_diff, orthoBasis);

    const Eigen::Matrix4f T_proj_motion = expMap(xi_proj);
    return Eigen::Affine3f(T_optimized.matrix() * T_proj_motion);
}

void mapOptimization::writeAffineToTransformTobeMapped(const Eigen::Affine3f &T_pose)
{
    float roll, pitch, yaw, x, y, z;
    pcl::getTranslationAndEulerAngles(T_pose, x, y, z, roll, pitch, yaw);
    transformTobeMapped[0] = roll;
    transformTobeMapped[1] = pitch;
    transformTobeMapped[2] = yaw;
    transformTobeMapped[3] = x;
    transformTobeMapped[4] = y;
    transformTobeMapped[5] = z;
}

mapOptimization::AdditionalOdomFusionResult mapOptimization::buildAdditionalOdomCorrectionResult(
    const Eigen::Affine3f &T_anchor,
    const Eigen::Affine3f &T_latest,
    const Eigen::Matrix4f &T_lidar_rel,
    const Eigen::Matrix4f &T_complementary_rel,
    const std::vector<TwistVector> &orthoBasis,
    bool ignore_dz,
    double dt_complementary_s)
{
    AdditionalOdomFusionResult result;
    result.T_anchor = T_anchor;
    result.T_latest = T_latest;

    Eigen::Vector3f t_lidar_rel_anchor = T_lidar_rel.block<3, 1>(0, 3);
    Eigen::Vector3f t_comp_rel_uncorrected_anchor = T_complementary_rel.block<3, 1>(0, 3);
    const Eigen::Matrix3f R_lidar_rel = T_lidar_rel.block<3, 3>(0, 0);
    const Eigen::Matrix3f R_comp_rel = T_complementary_rel.block<3, 3>(0, 0);
    result.R_orientation_drift = R_comp_rel * R_lidar_rel.transpose();

    Eigen::Vector3f t_comp_rel_corrected_anchor =
        result.R_orientation_drift.transpose() * t_comp_rel_uncorrected_anchor;

    if (ignore_dz)
    {
        t_lidar_rel_anchor.z() = 0.0f;
        t_comp_rel_uncorrected_anchor.z() = 0.0f;
        t_comp_rel_corrected_anchor.z() = 0.0f;
    }

    const Eigen::Matrix3f R_anchor = T_anchor.rotation();
    const Eigen::Matrix3f R_latest = T_latest.rotation();

    result.t_complementary_uncorrected_map = R_anchor * t_comp_rel_uncorrected_anchor;
    result.t_complementary_corrected_map = R_anchor * t_comp_rel_corrected_anchor;

    Eigen::Vector3f t_lidar_map = T_latest.translation() - T_anchor.translation();
    Eigen::Vector3f t_lidar_latest_local = R_latest.transpose() * t_lidar_map;
    Eigen::Vector3f t_comp_latest_local = R_latest.transpose() * result.t_complementary_corrected_map;

    if (ignore_dz)
    {
        t_lidar_latest_local.z() = 0.0f;
        t_comp_latest_local.z() = 0.0f;
    }

    const Eigen::Vector3f t_lidar_nondeg_latest_local =
        t_lidar_latest_local - projectOntoBasisTranslation(t_lidar_latest_local, orthoBasis);
    const Eigen::Vector3f t_comp_nondeg_latest_local =
        t_comp_latest_local - projectOntoBasisTranslation(t_comp_latest_local, orthoBasis);

    Eigen::Vector3f t_comp_nondeg_unit_latest_local = Eigen::Vector3f::Zero();
    const float comp_nondeg_norm = t_comp_nondeg_latest_local.norm();
    if (comp_nondeg_norm > 1e-6f)
        t_comp_nondeg_unit_latest_local = t_comp_nondeg_latest_local / comp_nondeg_norm;

    const Eigen::Vector3f t_lidar_nondeg_proj_latest_local =
        t_lidar_nondeg_latest_local.dot(t_comp_nondeg_unit_latest_local) * t_comp_nondeg_unit_latest_local;

    result.t_lidar_nondeg_map = R_latest * t_lidar_nondeg_latest_local;
    result.t_complementary_nondeg_map = R_latest * t_comp_nondeg_latest_local;
    result.t_lidar_nondeg_proj_on_complementary_map = R_latest * t_lidar_nondeg_proj_latest_local;

    result.scale_instant_raw =
        (comp_nondeg_norm > 1e-6f)
            ? static_cast<double>(t_lidar_nondeg_proj_latest_local.norm() / comp_nondeg_norm)
            : std::numeric_limits<double>::quiet_NaN();
    const double projected_nondeg_speed_mps =
        static_cast<double>(t_lidar_nondeg_proj_latest_local.norm()) / std::max(1e-5, dt_complementary_s);
    const double min_projected_nondeg_speed_mps =
        std::max(0.0, complementaryOdom.scaleMinNonDegenerateSpeed);
    const bool scale_observable =
        complementaryOdom.scaleEstimationEnabled &&
        std::isfinite(result.scale_instant_raw) &&
        projected_nondeg_speed_mps >= min_projected_nondeg_speed_mps;
    result.gate_observable = scale_observable ? 1.0 : 0.0;
    result.scale_filtered =
        scale_observable ? result.scale_instant_raw : std::numeric_limits<double>::quiet_NaN();

    const int smoothing_window = std::max(1, complementaryOdom.scaleSmoothingWindowSize);
    if (std::isfinite(result.scale_filtered))
    {
        laggedScaleFilteredHistory.push_back(result.scale_filtered);
    }
    while (static_cast<int>(laggedScaleFilteredHistory.size()) > smoothing_window)
    {
        laggedScaleFilteredHistory.pop_front();
    }
    if (!laggedScaleFilteredHistory.empty())
    {
        double sum = 0.0;
        for (const double sample : laggedScaleFilteredHistory)
            sum += sample;
        result.scale_smooth = sum / static_cast<double>(laggedScaleFilteredHistory.size());
    }
    else
    {
        result.scale_smooth = std::numeric_limits<double>::quiet_NaN();
    }

    result.lagged_complementary_lin_speed_mps =
        static_cast<double>(t_comp_rel_uncorrected_anchor.norm()) / std::max(1e-5, dt_complementary_s);
    result.lagged_relative_yaw_drift_deg =
        std::atan2(static_cast<double>(result.R_orientation_drift(1, 0)),
                   static_cast<double>(result.R_orientation_drift(0, 0))) *
        (180.0 / M_PI);
    result.valid = true;
    return result;
}

void mapOptimization::applyDegeneracyStateOverride(double dt_scan, bool degeneracyDetected)
{
    const Eigen::Affine3f T_optimized = trans2Affine3f(transformTobeMapped);
    const int baselineFrameLag = std::max(1, complementaryOdom.scaleBaselineFrameLag);
    const size_t lidarBufferSizeCap = static_cast<size_t>(baselineFrameLag + 1);

    if (degeneracyDetection.compensation_source == "none" || dt_scan <= 1e-5)
        return;

    std::vector<TwistVector> orthoBasis;
    const bool hasDegeneracyBasis = prepareDegeneracyOrthoBasis(orthoBasis);
    if (!hasDegeneracyBasis)
        orthoBasis.clear();

    const Eigen::Affine3f T_previous = incrementalOdometryAffineFront;

    Eigen::Affine3f T_complementary_odom = T_previous;
    Eigen::Affine3f T_lidar_lagged = T_previous;
    Eigen::Affine3f T_lidar_current = T_optimized;
    Eigen::Affine3f T_lagPathAnchor = T_previous;
    bool hasLagPathAnchor = false;
    bool hasLagVisualizationPair = false;
    bool hasLagPathVisualizationData = false;
    Eigen::Vector3f t_lidar_nondeg_map = Eigen::Vector3f::Zero();
    Eigen::Vector3f t_complementary_nondeg_map = Eigen::Vector3f::Zero();
    Eigen::Vector3f t_lidar_nondeg_proj_on_complementary_map = Eigen::Vector3f::Zero();
    Eigen::Vector3f t_complementary_uncorrected_map = Eigen::Vector3f::Zero();
    Eigen::Vector3f t_complementary_raw_map = Eigen::Vector3f::Zero();
    Eigen::Vector3f t_complementary_scaled_map = Eigen::Vector3f::Zero();
    Eigen::Affine3f T_complementary_unscaled_abs = T_previous;
    Eigen::Affine3f T_complementary_scaled_abs = T_previous;
    std::vector<Eigen::Vector3f> lagged_lidar_path_map;
    std::vector<Eigen::Vector3f> lagged_complementary_original_path_map;
    std::vector<Eigen::Vector3f> lagged_reconstructed_path_map;
    AdditionalOdomFusionResult lagFusionResult;
    bool hasLagVectorVisualizationData = false;
    ComplementaryOdomMatchInfo match_info;
    const bool estimatorModeActive = degeneracyDetected;
    const bool scaleAppliedToState = estimatorModeActive && hasDegeneracyBasis;
    const bool scaleApplyEnabled =
        complementaryOdom.scaleEstimationEnabled && complementaryOdom.scaleEstimationApply;
    double smoothedScaleForApply = std::numeric_limits<double>::quiet_NaN();
    if (scaleApplyEnabled && !laggedScaleFilteredHistory.empty())
    {
        double scale_sum = 0.0;
        for (const double sample : laggedScaleFilteredHistory)
            scale_sum += sample;
        smoothedScaleForApply = scale_sum / static_cast<double>(laggedScaleFilteredHistory.size());
    }

    // Core correction uses immediate LiDAR pair only.
    // Prefer the previous processed LiDAR timestamp from class state.
    // Fall back to dt-based synthesis only when the class state is invalid.
    TwistVector xi_complementary_lidar = TwistVector::Zero();
    double lidar_prev_stamp_s = std::numeric_limits<double>::quiet_NaN();
    if (std::isfinite(timeLastProcessing) && timeLastProcessing < timeLaserInfoCur)
    {
        lidar_prev_stamp_s = timeLastProcessing;
    }
    const bool hasComplementaryPrediction = inferComplementaryOdomTwist(
        dt_scan,
        xi_complementary_lidar,
        &match_info,
        lidar_prev_stamp_s,
        timeLaserInfoCur);
    if (hasComplementaryPrediction)
    {
        T_complementary_odom = buildScaledComplementaryPrediction(
            T_previous,
            T_optimized,
            orthoBasis,
            xi_complementary_lidar,
            match_info,
            estimatorModeActive,
            scaleAppliedToState,
            dt_scan,
            smoothedScaleForApply,
            t_lidar_nondeg_map,
            t_complementary_nondeg_map,
            t_lidar_nondeg_proj_on_complementary_map,
            t_complementary_raw_map,
            t_complementary_scaled_map,
            T_complementary_unscaled_abs,
            T_complementary_scaled_abs);
    }

    Eigen::Affine3f T_corrected_no_scale = T_optimized;
    Eigen::Affine3f T_corrected = T_optimized;
    if (estimatorModeActive && hasDegeneracyBasis)
    {
        T_corrected_no_scale = projectDegenerateCorrection(T_optimized, T_complementary_unscaled_abs, orthoBasis);
        T_corrected = projectDegenerateCorrection(T_optimized, T_complementary_odom, orthoBasis);
    }

    // Insert only the final effective LiDAR pose for this frame.
    const Eigen::Affine3f T_effective_current =
        (estimatorModeActive && hasDegeneracyBasis) ? T_corrected : T_optimized;
    complementaryOdomLidarPoseBuffer.emplace_back(timeLaserInfoCur, T_effective_current);
    while (complementaryOdomLidarPoseBuffer.size() > lidarBufferSizeCap)
    {
        complementaryOdomLidarPoseBuffer.pop_front();
    }

    double lidar_lagged_stamp_s = std::numeric_limits<double>::quiet_NaN();
    double lidar_current_stamp_s = std::numeric_limits<double>::quiet_NaN();
    if (complementaryOdomLidarPoseBuffer.size() >= 2)
    {
        T_lidar_lagged = complementaryOdomLidarPoseBuffer.front().second;
        T_lidar_current = complementaryOdomLidarPoseBuffer.back().second;
        lidar_lagged_stamp_s = complementaryOdomLidarPoseBuffer.front().first;
        lidar_current_stamp_s = complementaryOdomLidarPoseBuffer.back().first;
        hasLagVisualizationPair = std::isfinite(lidar_lagged_stamp_s) && std::isfinite(lidar_current_stamp_s);
    }

    // Lagged oldest/newest pair is used only for visualization vectors.
    if (hasLagVisualizationPair)
    {
        std::deque<nav_msgs::msg::Odometry> complementaryOdomQueueSnapshot;
        {
            std::lock_guard<std::mutex> lock(complementaryOdomMutex);
            complementaryOdomQueueSnapshot = complementaryOdomQueue;
        }

        if (complementaryOdomQueueSnapshot.size() >= 2 && complementaryOdomLidarPoseBuffer.size() >= 2)
        {
            struct LaggedPairSample
            {
                double lidar_stamp_s = std::numeric_limits<double>::quiet_NaN();
                double odom_stamp_s = std::numeric_limits<double>::quiet_NaN();
                Eigen::Affine3f lidar_pose = Eigen::Affine3f::Identity();
                Eigen::Matrix4f odom_pose = Eigen::Matrix4f::Identity();
            };

            const auto closestOdomIdx = [&complementaryOdomQueueSnapshot](double target_time_s,
                                                                            int min_idx,
                                                                            double *best_abs_dt_s) -> int
            {
                int best_idx = -1;
                double best_abs_dt = std::numeric_limits<double>::infinity();
                for (int i = std::max(0, min_idx); i < static_cast<int>(complementaryOdomQueueSnapshot.size()); ++i)
                {
                    const double msg_time = ROS_TIME(complementaryOdomQueueSnapshot[i].header.stamp);
                    const double abs_dt = std::abs(msg_time - target_time_s);
                    if (abs_dt < best_abs_dt)
                    {
                        best_abs_dt = abs_dt;
                        best_idx = i;
                    }
                }
                if (best_abs_dt_s)
                    *best_abs_dt_s = best_abs_dt;
                return best_idx;
            };

            std::vector<LaggedPairSample> lagged_pairs;
            lagged_pairs.reserve(complementaryOdomLidarPoseBuffer.size());
            int min_search_idx = 0;
            for (const auto &lidar_sample : complementaryOdomLidarPoseBuffer)
            {
                double best_abs_dt_s = std::numeric_limits<double>::infinity();
                const int odom_idx = closestOdomIdx(lidar_sample.first, min_search_idx, &best_abs_dt_s);
                if (odom_idx < 0 || best_abs_dt_s > 0.25)
                    continue;

                Eigen::Isometry3d iso_odom;
                tf2::fromMsg(complementaryOdomQueueSnapshot[odom_idx].pose.pose, iso_odom);

                LaggedPairSample sample;
                sample.lidar_stamp_s = lidar_sample.first;
                sample.odom_stamp_s = ROS_TIME(complementaryOdomQueueSnapshot[odom_idx].header.stamp);
                sample.lidar_pose = lidar_sample.second;
                sample.odom_pose = iso_odom.matrix().cast<float>();
                lagged_pairs.push_back(sample);
                min_search_idx = odom_idx;
            }

            if (lagged_pairs.size() >= 2)
            {
                if (!resolveComplementaryOdomExtrinsics(complementaryOdomQueueSnapshot.back().header.frame_id))
                {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                         "[COMPLEMENTARY_ODOM_LAGGED_PATH] extrinsics unresolved, skipping lagged path debug");
                }
                else
                {
                    const Eigen::Affine3f T_lidar_anchor = lagged_pairs.front().lidar_pose;
                    const Eigen::Matrix3f R_lidar_anchor = T_lidar_anchor.rotation();
                    const Eigen::Vector3f p_anchor = T_lidar_anchor.translation();
                    T_lagPathAnchor = T_lidar_anchor;
                    hasLagPathAnchor = true;

                    const Eigen::Matrix4f T_odom_to_lidar = T_complementary_to_lidar;
                    const Eigen::Matrix4f T_lidar_to_odom = T_complementary_to_lidar.inverse();
                    const Eigen::Matrix4f T_comp0_lidar = T_odom_to_lidar * lagged_pairs.front().odom_pose * T_lidar_to_odom;

                    lagged_lidar_path_map.clear();
                    lagged_complementary_original_path_map.clear();
                    lagged_reconstructed_path_map.clear();
                    lagged_lidar_path_map.reserve(lagged_pairs.size());
                    lagged_complementary_original_path_map.reserve(lagged_pairs.size());
                    lagged_reconstructed_path_map.reserve(lagged_pairs.size());

                    Eigen::Vector3f reconstructed_point = p_anchor;
                    int reconstructed_direction_sign = -1;
                    lagged_reconstructed_path_map.push_back(reconstructed_point);
                    Eigen::Matrix4f T_lidar_rel_last = Eigen::Matrix4f::Identity();
                    Eigen::Matrix4f T_comp_rel_last = Eigen::Matrix4f::Identity();

                    for (size_t i = 0; i < lagged_pairs.size(); ++i)
                    {
                        const Eigen::Affine3f &T_lidar_i = lagged_pairs[i].lidar_pose;
                        const Eigen::Matrix4f T_lidar_rel = T_lidar_anchor.matrix().inverse() * T_lidar_i.matrix();
                        const Eigen::Vector3f t_lidar_rel = T_lidar_rel.block<3, 1>(0, 3);
                        lagged_lidar_path_map.push_back(p_anchor + R_lidar_anchor * t_lidar_rel);

                        const Eigen::Matrix4f T_comp_i_lidar = T_odom_to_lidar * lagged_pairs[i].odom_pose * T_lidar_to_odom;
                        const Eigen::Matrix4f T_comp_rel = T_comp0_lidar.inverse() * T_comp_i_lidar;
                        const Eigen::Vector3f t_comp_rel = T_comp_rel.block<3, 1>(0, 3);
                        lagged_complementary_original_path_map.push_back(p_anchor + R_lidar_anchor * t_comp_rel);
                        T_lidar_rel_last = T_lidar_rel;
                        T_comp_rel_last = T_comp_rel;

                        if (i == 0)
                            continue;

                        const Eigen::Matrix4f T_comp_prev_lidar =
                            T_odom_to_lidar * lagged_pairs[i - 1].odom_pose * T_lidar_to_odom;
                        const Eigen::Matrix4f T_comp_delta = T_comp_prev_lidar.inverse() * T_comp_i_lidar;
                        Eigen::Vector3f t_comp_delta = T_comp_delta.block<3, 1>(0, 3);

                        if (complementaryOdom.ignore_dz)
                            t_comp_delta.z() = 0.0f;

                        const double dt_odom = std::max(1e-5, lagged_pairs[i].odom_stamp_s - lagged_pairs[i - 1].odom_stamp_s);
                        const double dt_lidar = std::max(1e-5, lagged_pairs[i].lidar_stamp_s - lagged_pairs[i - 1].lidar_stamp_s);
                        const double complementary_speed_mps = static_cast<double>(t_comp_delta.norm()) / dt_odom;
                        double forward_distance = complementary_speed_mps * dt_lidar;

                        Eigen::Vector3f forward_axis = lagged_pairs[i - 1].lidar_pose.rotation().col(0);
                        Eigen::Vector3f lidar_step =
                            lagged_pairs[i].lidar_pose.translation() - lagged_pairs[i - 1].lidar_pose.translation();
                        if (complementaryOdom.ignore_dz)
                        {
                            forward_axis.z() = 0.0f;
                            lidar_step.z() = 0.0f;
                        }

                        const float forward_axis_norm = forward_axis.norm();
                        if (forward_axis_norm > 1e-6f)
                            forward_axis /= forward_axis_norm;
                        else
                            forward_distance = 0.0;

                        const float lidar_step_signed_forward = lidar_step.dot(forward_axis);
                        constexpr float kForwardDirectionDeadbandM = 1e-4f;
                        if (lidar_step_signed_forward > kForwardDirectionDeadbandM)
                            reconstructed_direction_sign = 1;
                        else if (lidar_step_signed_forward < -kForwardDirectionDeadbandM)
                            reconstructed_direction_sign = -1;

                        Eigen::Vector3f forward_dir =
                            static_cast<float>(reconstructed_direction_sign) * forward_axis;
                        if (complementaryOdom.ignore_dz)
                        {
                            forward_dir.z() = 0.0f;
                            const float forward_norm = forward_dir.norm();
                            if (forward_norm > 1e-6f)
                                forward_dir /= forward_norm;
                            else
                                forward_distance = 0.0;
                        }

                        reconstructed_point += static_cast<float>(forward_distance) * forward_dir;
                        lagged_reconstructed_path_map.push_back(reconstructed_point);
                    }

                    hasLagPathVisualizationData = lagged_lidar_path_map.size() >= 2 &&
                                                  lagged_complementary_original_path_map.size() == lagged_lidar_path_map.size() &&
                                                  lagged_reconstructed_path_map.size() == lagged_lidar_path_map.size();

                    if (hasLagPathVisualizationData)
                    {
                        const Eigen::Affine3f T_lidar_latest = lagged_pairs.back().lidar_pose;
                        lagFusionResult = buildAdditionalOdomCorrectionResult(
                            T_lagPathAnchor,
                            T_lidar_latest,
                            T_lidar_rel_last,
                            T_comp_rel_last,
                            orthoBasis,
                            complementaryOdom.ignore_dz,
                            lagged_pairs.back().odom_stamp_s - lagged_pairs.front().odom_stamp_s);
                        hasLagVectorVisualizationData = lagFusionResult.valid;
                    }
                }
            }
        }

        TwistVector xi_complementary_lidar_lag = TwistVector::Zero();
        ComplementaryOdomMatchInfo match_info_lag;
        const bool hasLagComplementaryPrediction = inferComplementaryOdomTwist(
            dt_scan,
            xi_complementary_lidar_lag,
            &match_info_lag,
            lidar_lagged_stamp_s,
            lidar_current_stamp_s);

        if (hasLagComplementaryPrediction)
        {
            const double dt_prediction_lag = (match_info_lag.dt_complementary_s > 1e-5)
                                                 ? match_info_lag.dt_complementary_s
                                                 : dt_scan;
            const Eigen::Matrix4f T_lidar_complementary_predicted_lag =
                expMap(xi_complementary_lidar_lag, static_cast<float>(dt_prediction_lag));

            const Eigen::Matrix4f T_lidar_rel_lag = T_lidar_lagged.matrix().inverse() * T_lidar_current.matrix();
            if (!lagFusionResult.valid)
            {
                lagFusionResult = buildAdditionalOdomCorrectionResult(
                    T_lidar_lagged,
                    T_lidar_current,
                    T_lidar_rel_lag,
                    T_lidar_complementary_predicted_lag,
                    orthoBasis,
                    complementaryOdom.ignore_dz,
                    dt_prediction_lag);
                hasLagVectorVisualizationData = lagFusionResult.valid;
            }

            if (pubComplementaryOdomScaleDebug)
            {
                lili::msg::ComplementaryOdomScaleDebug msg;
                msg.header.stamp = timeLaserInfoStamp;
                msg.header.frame_id = lidarFrame;
                const bool hasAppliedScale = scaleApplyEnabled && std::isfinite(smoothedScaleForApply);
                msg.scale_applied = hasAppliedScale ? smoothedScaleForApply : 1.0;
                msg.scale_estimation_enabled = complementaryOdom.scaleEstimationEnabled ? 1.0 : 0.0;
                msg.scale_applied_to_state = hasAppliedScale ? 1.0 : 0.0;
                msg.estimator_mode = estimatorModeActive ? 1.0 : 0.0;
                msg.dt_scan_s = dt_scan;
                msg.dt_odom_s = match_info_lag.dt_complementary_s;
                msg.dt_scale_odom_interval_s = match_info_lag.dt_complementary_s;
                msg.dt_scale_lidar_interval_s =
                    match_info_lag.lidar_curr_stamp_s - match_info_lag.lidar_prev_stamp_s;
                msg.dt_scale_odom_pair_interval_s = match_info_lag.dt_complementary_s;
                msg.dt_immediate_odom_pair_interval_s = match_info_lag.dt_complementary_s;
                msg.scale_instant_raw = lagFusionResult.scale_instant_raw;
                msg.scale_filtered = lagFusionResult.scale_filtered;
                msg.scale_smooth = lagFusionResult.scale_smooth;
                msg.gate_observable = lagFusionResult.gate_observable;
                msg.lagged_complementary_lin_speed_mps = lagFusionResult.lagged_complementary_lin_speed_mps;
                msg.lagged_relative_yaw_drift_deg = lagFusionResult.lagged_relative_yaw_drift_deg;
                pubComplementaryOdomScaleDebug->publish(msg);
            }
        }
    }

    if (hasLagPathVisualizationData && !hasLagPathAnchor)
    {
        hasLagVectorVisualizationData = false;
    }

    if (hasLagVectorVisualizationData)
    {
        const Eigen::Affine3f T_arrow_base = hasLagPathAnchor
                                                 ? T_lagPathAnchor
                                                 : (hasLagVisualizationPair ? T_lidar_lagged : T_previous);
        publishComplementaryOdomDisplacementDebug(T_arrow_base,
                                                    T_complementary_unscaled_abs,
                                                    T_complementary_scaled_abs,
                                                    T_corrected_no_scale,
                                                    T_corrected,
                                                    lagFusionResult.t_complementary_nondeg_map,
                                                    lagFusionResult.t_lidar_nondeg_map,
                                                    lagFusionResult.t_lidar_nondeg_proj_on_complementary_map,
                                                    lagFusionResult.t_complementary_uncorrected_map,
                                                    lagFusionResult.t_complementary_corrected_map,
                                                    lagFusionResult.t_complementary_corrected_map);
    }

    if (hasLagPathVisualizationData)
    {
        publishComplementaryOdomLaggedPathsDebug(
            lagged_lidar_path_map,
            lagged_complementary_original_path_map,
            lagged_reconstructed_path_map);
    }

    // Per-frame replay record: everything the scaling logic consumes, so that
    // odometry-scaling experiments can be replayed offline without re-running SLAM.
    if (diagnostics)
    {
        ScaleReplayFrameSample replay;
        replay.stamp_sec = timeLaserInfoCur;
        replay.lidar_prev_stamp_s = lidar_prev_stamp_s;
        replay.dt_scan_s = dt_scan;
        replay.degeneracy_detected = degeneracyDetected;
        replay.has_degeneracy_basis = hasDegeneracyBasis;
        replay.has_complementary_twist = hasComplementaryPrediction;
        replay.override_applied_to_state = estimatorModeActive && hasDegeneracyBasis;
        replay.scale_applied = (scaleApplyEnabled && std::isfinite(smoothedScaleForApply))
                                   ? smoothedScaleForApply
                                   : 1.0;

        replay.basis_size = static_cast<int>(std::min<size_t>(orthoBasis.size(), 3));
        for (size_t i = 0; i < 3; ++i)
            for (size_t j = 0; j < 6; ++j)
                replay.basis_twists[i][j] = (i < orthoBasis.size())
                                                ? static_cast<double>(orthoBasis[i][j])
                                                : std::numeric_limits<double>::quiet_NaN();

        const TwistVector xi_lidar_increment =
            matrixToTwist(T_previous.matrix().inverse() * T_optimized.matrix());
        for (size_t j = 0; j < 6; ++j)
        {
            replay.lidar_increment_twist[j] = static_cast<double>(xi_lidar_increment[j]);
            replay.complementary_twist[j] = hasComplementaryPrediction
                                                ? static_cast<double>(xi_complementary_lidar[j])
                                                : std::numeric_limits<double>::quiet_NaN();
        }

        replay.dt_complementary_s = match_info.dt_complementary_s;
        replay.odom_prev_stamp_s = match_info.odom_prev_stamp_s;
        replay.odom_curr_stamp_s = match_info.odom_curr_stamp_s;

        const auto fillPose = [](const Eigen::Affine3f &T, std::array<double, 7> &out)
        {
            const Eigen::Vector3f t = T.translation();
            const Eigen::Quaternionf q(T.rotation());
            out = {static_cast<double>(t.x()), static_cast<double>(t.y()), static_cast<double>(t.z()),
                   static_cast<double>(q.x()), static_cast<double>(q.y()), static_cast<double>(q.z()),
                   static_cast<double>(q.w())};
        };
        fillPose(T_previous, replay.pose_prev);
        fillPose(T_optimized, replay.pose_optimized);
        fillPose(T_effective_current, replay.pose_effective);

        diagnostics->recordScaleReplayFrameCsv(replay);
    }

    if (!estimatorModeActive || !hasDegeneracyBasis)
        return;

    writeAffineToTransformTobeMapped(T_corrected);
}
