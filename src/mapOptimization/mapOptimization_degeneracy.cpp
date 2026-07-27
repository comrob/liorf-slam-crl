#include "mapOptimization/mapOptimization.hpp"

// This translation unit owns perturbation-based degeneracy detection
// orchestration and the complementary odometry aiding used to compensate
// degenerate directions (input handling, extrinsics resolution, and the
// state override). Related visualization lives in mapOptimization_publish.cpp.

void mapOptimization::complementaryOdomHandler(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(complementaryOdomMutex);
    complementaryOdomQueue.push_back(*msg);

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

    if (!complementaryOdom.autoLookupLidarToTf)
    {
        T_complementary_to_lidar = Eigen::Matrix4f::Identity();
        T_complementary_to_lidar.block<3, 3>(0, 0) = complementaryOdom.extRot.cast<float>();
        T_complementary_to_lidar.block<3, 1>(0, 3) = complementaryOdom.extTrans.cast<float>();
        complementaryOdomTfResolved = true;
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

    if (degeneracyDetected)
    {
        const double dt_scan = (curTimeDiff > 1e-5) ? curTimeDiff : 0.1;
        applyDegeneracyStateOverride(dt_scan);
    }

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
                                                  ComplementaryOdomMatchInfo *match_info)
{
    xi_complementary_lidar = TwistVector::Zero();
    if (degeneracyDetection.compensation_source != "complementary_odom")
        return false;

    std::lock_guard<std::mutex> lock(complementaryOdomMutex);
    if (complementaryOdomQueue.size() < 2)
        return false;

    const double prev_lidar_time = timeLaserInfoCur - ((curTimeDiff > 1e-5) ? curTimeDiff : dt_scan);
    const double curr_lidar_time = timeLaserInfoCur;

    const auto closestIdxToTime = [this](double target_time, int exclude_idx = -1) -> int
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
        return best_idx;
    };

    const int idx_prev_near = closestIdxToTime(prev_lidar_time);
    const int idx_curr_near = closestIdxToTime(curr_lidar_time, idx_prev_near);
    if (idx_prev_near < 0 || idx_curr_near < 0)
        return false;

    const double t_prev_near = ROS_TIME(complementaryOdomQueue[idx_prev_near].header.stamp);
    const double t_curr_near = ROS_TIME(complementaryOdomQueue[idx_curr_near].header.stamp);
    const double prev_abs_dt = std::abs(t_prev_near - prev_lidar_time);
    const double curr_abs_dt = std::abs(t_curr_near - curr_lidar_time);
    if (prev_abs_dt > 0.25 || curr_abs_dt > 0.25)
        return false;

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

float mapOptimization::smoothComplementaryOdomScale(float scaleInstant)
{
    const size_t windowSize = static_cast<size_t>(std::max(1, complementaryOdom.scaleSmoothingWindowSize));

    // Pre-fill with the first sample to avoid startup transients from an
    // undersized history window.
    if (complementaryOdomScaleHistory.empty())
    {
        complementaryOdomScaleHistory.assign(windowSize, scaleInstant);
        complementaryOdomScaleHistorySum = scaleInstant * static_cast<float>(windowSize);
        return scaleInstant;
    }

    complementaryOdomScaleHistory.push_back(scaleInstant);
    complementaryOdomScaleHistorySum += scaleInstant;
    while (complementaryOdomScaleHistory.size() > windowSize)
    {
        complementaryOdomScaleHistorySum -= complementaryOdomScaleHistory.front();
        complementaryOdomScaleHistory.pop_front();
    }

    return complementaryOdomScaleHistorySum / static_cast<float>(complementaryOdomScaleHistory.size());
}

float mapOptimization::smoothComplementaryOdomScaleFromNumDen(float numerator, float denominator, bool pushSample)
{
    const size_t windowSize = static_cast<size_t>(std::max(1, complementaryOdom.scaleSmoothingWindowSize));

    if (pushSample)
    {
        complementaryOdomScaleNumeratorHistory.push_back(numerator);
        complementaryOdomScaleNumeratorHistorySum += numerator;
        while (complementaryOdomScaleNumeratorHistory.size() > windowSize)
        {
            complementaryOdomScaleNumeratorHistorySum -= complementaryOdomScaleNumeratorHistory.front();
            complementaryOdomScaleNumeratorHistory.pop_front();
        }

        complementaryOdomScaleDenominatorHistory.push_back(denominator);
        complementaryOdomScaleDenominatorHistorySum += denominator;
        while (complementaryOdomScaleDenominatorHistory.size() > windowSize)
        {
            complementaryOdomScaleDenominatorHistorySum -= complementaryOdomScaleDenominatorHistory.front();
            complementaryOdomScaleDenominatorHistory.pop_front();
        }
    }

    if (complementaryOdomScaleDenominatorHistory.empty())
        return std::numeric_limits<float>::quiet_NaN();

    if (std::abs(complementaryOdomScaleDenominatorHistorySum) < 1e-6f)
        return std::numeric_limits<float>::quiet_NaN();

    return complementaryOdomScaleNumeratorHistorySum / complementaryOdomScaleDenominatorHistorySum;
}

float mapOptimization::smoothComplementaryOdomFallbackScale(float scaleInstant)
{
    constexpr size_t kFallbackWindowSize = 30;

    // Pre-fill with the first valid sample to avoid startup transients.
    if (complementaryOdomFallbackScaleHistory.empty())
    {
        complementaryOdomFallbackScaleHistory.assign(kFallbackWindowSize, scaleInstant);
        complementaryOdomFallbackScaleHistorySum = scaleInstant * static_cast<float>(kFallbackWindowSize);
        return scaleInstant;
    }

    complementaryOdomFallbackScaleHistory.push_back(scaleInstant);
    complementaryOdomFallbackScaleHistorySum += scaleInstant;
    while (complementaryOdomFallbackScaleHistory.size() > kFallbackWindowSize)
    {
        complementaryOdomFallbackScaleHistorySum -= complementaryOdomFallbackScaleHistory.front();
        complementaryOdomFallbackScaleHistory.pop_front();
    }

    return complementaryOdomFallbackScaleHistorySum / static_cast<float>(complementaryOdomFallbackScaleHistory.size());
}

Eigen::Affine3f mapOptimization::buildScaledComplementaryPrediction(const Eigen::Affine3f &T_previous,
                                                                    const Eigen::Affine3f &T_optimized,
                                                                    const std::vector<TwistVector> &orthoBasis,
                                                                    const TwistVector &xi_complementary_lidar,
                                                                    const ComplementaryOdomMatchInfo &match_info,
                                                                    double dt_scan,
                                                                    Eigen::Vector3f &t_lidar_nondeg_map,
                                                                    Eigen::Vector3f &t_complementary_nondeg_map,
                                                                    Eigen::Vector3f &t_lidar_nondeg_proj_on_complementary_map,
                                                                    Eigen::Vector3f &t_complementary_raw_map,
                                                                    Eigen::Vector3f &t_complementary_scaled_map,
                                                                    Eigen::Affine3f &T_complementary_unscaled_abs,
                                                                    Eigen::Affine3f &T_complementary_scaled_abs)
{
    Eigen::Matrix4f T_lidar_complementary_predicted_unscaled = expMap(xi_complementary_lidar, static_cast<float>(dt_scan));

    // Estimate the complementary-odom translation scale from the non-degenerate
    // translation subspace, where the scan matcher is trusted.
    const Eigen::Vector3f t_lidar =
        (T_previous.matrix().inverse() * T_optimized.matrix()).block<3, 1>(0, 3);
    const Eigen::Vector3f t_complementary = T_lidar_complementary_predicted_unscaled.block<3, 1>(0, 3);

    // Optional 2D mode for scale determination only.
    Eigen::Vector3f t_lidar_scale = t_lidar;
    Eigen::Vector3f t_complementary_scale = t_complementary;
    if (complementaryOdom.ignore_dz)
    {
        t_lidar_scale.z() = 0.0f;
        t_complementary_scale.z() = 0.0f;
    }

    const Eigen::Vector3f t_lidar_nondeg = t_lidar_scale - projectOntoBasisTranslation(t_lidar_scale, orthoBasis);
    const Eigen::Vector3f t_complementary_nondeg = t_complementary_scale - projectOntoBasisTranslation(t_complementary_scale, orthoBasis);
    // project lidar displacement onto complementary odometry displacement to get a scale estimate for the complementary odometry
    const float complementaryNondegNorm = t_complementary_nondeg.norm();
    const float lidarNondegNormUnprojected = t_lidar_nondeg.norm();
    Eigen::Vector3f t_complementary_nondeg_unit = Eigen::Vector3f::Zero();
    if (complementaryNondegNorm > 1e-6f)
        t_complementary_nondeg_unit = t_complementary_nondeg / complementaryNondegNorm;

    const Eigen::Vector3f t_lidar_nondeg_proj =
        t_lidar_nondeg.dot(t_complementary_nondeg_unit) * t_complementary_nondeg_unit;
    // const float complementaryOdomScale = t_complementary_nondeg.norm() > 1e-5f ? t_lidar_nondeg_proj.norm() / t_complementary_nondeg.norm() : 1.0f;

    constexpr float kMinScale = 0.2f;
    constexpr float kMaxScale = 2.0f;
    const float minNondegNorm =
        static_cast<float>(complementaryOdom.scaleMinNonDegenerateSpeed) * static_cast<float>(dt_scan);

    const float lidarNondegNorm = t_lidar_nondeg_proj.norm();
    const bool gateObservable = lidarNondegNorm > minNondegNorm && complementaryNondegNorm > minNondegNorm;

    float scaleInstantRaw = std::numeric_limits<float>::quiet_NaN();
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

        // Direct scale estimate from the current frame pair only.
        scaleInstantRaw = scaleRatioRaw;
    }

    const bool hasInstantRaw = std::isfinite(scaleInstantRaw);
    const float scaleInstantRawClamped = hasInstantRaw ? std::clamp(scaleInstantRaw, kMinScale, kMaxScale) : 1.0f;

    float scaleFallbackHistory = complementaryOdomFallbackScaleHistory.empty()
                                     ? 1.0f
                                     : (complementaryOdomFallbackScaleHistorySum /
                                        static_cast<float>(complementaryOdomFallbackScaleHistory.size()));
    if (gateObservable && hasInstantRaw)
        scaleFallbackHistory = smoothComplementaryOdomFallbackScale(scaleInstantRawClamped);

    const float scaleInstantRawSafe = (gateObservable && hasInstantRaw) ? scaleInstantRawClamped : scaleFallbackHistory;
    const float scaleSmoothLegacy = complementaryOdom.scaleEstimationEnabled ? smoothComplementaryOdomScale(scaleInstantRawSafe) : 1.0f;

    const float numeratorSample = lidarNondegNorm;
    const float denominatorSample = complementaryNondegNorm;
    const bool pushNumDenSample = gateObservable && hasInstantRaw;
    float scaleSmoothRatio = smoothComplementaryOdomScaleFromNumDen(numeratorSample, denominatorSample, pushNumDenSample);
    if (std::isfinite(scaleSmoothRatio))
        scaleSmoothRatio = std::clamp(scaleSmoothRatio, kMinScale, kMaxScale);

    const bool useRatioOfSums = complementaryOdom.smoothFromNumDen;
    const float scaleSmooth = (useRatioOfSums && std::isfinite(scaleSmoothRatio)) ? scaleSmoothRatio : scaleSmoothLegacy;
    const float scaleApplied = scaleSmooth;

    const Eigen::Matrix3f R_prev = T_previous.rotation();
    t_lidar_nondeg_map = R_prev * t_lidar_nondeg;
    t_complementary_nondeg_map = R_prev * t_complementary_nondeg;
    t_lidar_nondeg_proj_on_complementary_map = R_prev * t_lidar_nondeg_proj;
    t_complementary_raw_map = R_prev * t_complementary;
    t_complementary_scaled_map = scaleApplied * t_complementary_raw_map;

    if (pubComplementaryOdomScaleDebug)
    {
        liorf::msg::ComplementaryOdomScaleDebug msg;
        msg.header.stamp = timeLaserInfoStamp;
        msg.header.frame_id = lidarFrame;
        msg.scale_instant_raw = static_cast<double>(scaleInstantRaw);
        msg.scale_instant_raw_clamped = static_cast<double>(scaleInstantRawClamped);
        msg.scale_instant_raw_safe = static_cast<double>(scaleInstantRawSafe);
        msg.scale_fallback_history = static_cast<double>(scaleFallbackHistory);
        msg.scale_smooth = static_cast<double>(scaleSmooth);
        msg.scale_smooth_ratio = static_cast<double>(scaleSmoothRatio);
        msg.scale_smooth_legacy = static_cast<double>(scaleSmoothLegacy);
        msg.scale_applied = static_cast<double>(scaleApplied);
        msg.gate_observable = gateObservable ? 1.0 : 0.0;
        msg.scale_estimation_enabled = complementaryOdom.scaleEstimationEnabled ? 1.0 : 0.0;
        msg.smooth_from_numden_enabled = useRatioOfSums ? 1.0 : 0.0;
        msg.numerator_sample = static_cast<double>(numeratorSample);
        msg.denominator_sample = static_cast<double>(denominatorSample);
        msg.dt_scan_s = dt_scan;
        msg.dt_odom_s = match_info.dt_complementary_s;
        pubComplementaryOdomScaleDebug->publish(msg);
    }


    const float complementaryOdomLinearSpeedOrig = xi_complementary_lidar.head<3>().norm();
    const float lidarLinearSpeedNondeg = lidarNondegNorm / static_cast<float>(dt_scan);
    const float complementaryOdomLinearSpeedNondeg = complementaryNondegNorm / static_cast<float>(dt_scan);
    const float lidarLinearSpeedAfterScale =
        (complementaryNondegNorm * scaleApplied) / static_cast<float>(dt_scan);

    Eigen::Matrix4f T_lidar_complementary_predicted_scaled = T_lidar_complementary_predicted_unscaled;
    T_lidar_complementary_predicted_scaled.block<3, 1>(0, 3) *= scaleApplied;

    T_complementary_unscaled_abs = Eigen::Affine3f(T_previous.matrix() * T_lidar_complementary_predicted_unscaled);
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
        sample.scale_instant_raw = scaleInstantRaw;
        sample.scale_instant_raw_clamped = scaleInstantRawClamped;
        sample.scale_instant_raw_safe = scaleInstantRawSafe;
        sample.scale_fallback_history = scaleFallbackHistory;
        sample.scale_smooth = scaleSmooth;
        sample.scale_smooth_ratio = scaleSmoothRatio;
        sample.scale_smooth_legacy = scaleSmoothLegacy;
        sample.scale_applied = scaleApplied;
        sample.smooth_from_numden_enabled = useRatioOfSums;
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
        sample.scale_smoothing_window = complementaryOdom.scaleSmoothingWindowSize;
        sample.complementary_odom_lin_speed_orig_mps = complementaryOdomLinearSpeedOrig;
        sample.lidar_lin_speed_nondeg_mps = lidarLinearSpeedNondeg;
        sample.complementary_odom_lin_speed_nondeg_mps = complementaryOdomLinearSpeedNondeg;
        sample.lidar_lin_speed_proj_scale1_mps = lidarLinearSpeedProjScale1;
        sample.lidar_lin_speed_after_scale_mps = lidarLinearSpeedAfterScale;
        sample.dt_scan_s = dt_scan;
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

void mapOptimization::applyDegeneracyStateOverride(double dt_scan)
{
    if (degeneracyDetection.compensation_source == "none" || dt_scan <= 1e-5)
        return;

    std::vector<TwistVector> orthoBasis;
    if (!prepareDegeneracyOrthoBasis(orthoBasis))
        return;

    const Eigen::Affine3f T_optimized = trans2Affine3f(transformTobeMapped);
    const Eigen::Affine3f T_previous = incrementalOdometryAffineFront;

    Eigen::Affine3f T_complementary_odom = T_previous;
    Eigen::Vector3f t_lidar_nondeg_map = Eigen::Vector3f::Zero();
    Eigen::Vector3f t_complementary_nondeg_map = Eigen::Vector3f::Zero();
    Eigen::Vector3f t_lidar_nondeg_proj_on_complementary_map = Eigen::Vector3f::Zero();
    Eigen::Vector3f t_complementary_raw_map = Eigen::Vector3f::Zero();
    Eigen::Vector3f t_complementary_scaled_map = Eigen::Vector3f::Zero();
    Eigen::Affine3f T_complementary_unscaled_abs = T_previous;
    Eigen::Affine3f T_complementary_scaled_abs = T_previous;
    ComplementaryOdomMatchInfo match_info;

    TwistVector xi_complementary_lidar = TwistVector::Zero();
    const bool hasComplementaryPrediction = inferComplementaryOdomTwist(dt_scan, xi_complementary_lidar, &match_info);
    if (hasComplementaryPrediction)
    {
        T_complementary_odom = buildScaledComplementaryPrediction(
            T_previous,
            T_optimized,
            orthoBasis,
            xi_complementary_lidar,
            match_info,
            dt_scan,
            t_lidar_nondeg_map,
            t_complementary_nondeg_map,
            t_lidar_nondeg_proj_on_complementary_map,
            t_complementary_raw_map,
            t_complementary_scaled_map,
            T_complementary_unscaled_abs,
            T_complementary_scaled_abs);
    }

    const Eigen::Affine3f T_corrected_no_scale = projectDegenerateCorrection(T_optimized, T_complementary_unscaled_abs, orthoBasis);
    const Eigen::Affine3f T_corrected = projectDegenerateCorrection(T_optimized, T_complementary_odom, orthoBasis);

    publishComplementaryOdomDisplacementDebug(T_previous,
                                              T_complementary_unscaled_abs,
                                              T_complementary_scaled_abs,
                                              T_corrected_no_scale,
                                              T_corrected,
                                              t_complementary_nondeg_map,
                                              t_lidar_nondeg_map,
                                              t_lidar_nondeg_proj_on_complementary_map,
                                              t_complementary_raw_map,
                                              t_complementary_scaled_map);

    writeAffineToTransformTobeMapped(T_corrected);
}
