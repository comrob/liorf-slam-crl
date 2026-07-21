#include "mapOptimization/mapOptimization.hpp"

void mapOptimization::updateInitialGuess()
{
    if (!cloudKeyPoses3D) {
        RCLCPP_ERROR(get_logger(), "cloudKeyPoses3D is NULL! This should not happen.");
        return;
    }
    // save current transformation before any processing
    incrementalOdometryAffineFront = trans2Affine3f(transformTobeMapped);

    const auto clampTranslationPrediction = [this](Eigen::Affine3f &transIncre, const char *branch_name) {
        const double predNorm = static_cast<double>(transIncre.translation().norm());
        const double predSpeed = (curTimeDiff > 0.0)
                                     ? (predNorm / curTimeDiff)
                                     : std::numeric_limits<double>::infinity();
        const double wallNowSec = this->now().seconds();

        if (diagnostics)
            diagnostics->recordTranslationPrediction(predNorm);

        if (maxTranslationPrediction > 0.0 && predNorm > maxTranslationPrediction)
        {
            std::ostringstream err_ss;
            err_ss << "[TRANSLATION_PREDICTION_EXCEEDED]"
                   << " branch=" << branch_name
                   << " delta_m=" << std::fixed << std::setprecision(3) << predNorm
                   << " limit_m=" << maxTranslationPrediction
                     << " frame_stamp_s=" << std::fixed << std::setprecision(6) << timeLaserInfoCur
                     << " last_frame_stamp_s=" << timeLastProcessing
                     << " wall_now_s=" << wallNowSec
                   << " cur_dt_s=" << curTimeDiff
                   << " last_dt_s=" << lastTimeDiff;
            const std::string err_msg = err_ss.str();
            RCLCPP_ERROR_STREAM(this->get_logger(), err_msg);
            transIncre.translation().setZero();
            if (diagnostics)
                diagnostics->publishWarning(err_msg);
        }

        if (minTranslationPredictionSpeed > 0.0 && predSpeed < minTranslationPredictionSpeed)
        {
            std::ostringstream warn_ss;
            warn_ss << "[TRANSLATION_PREDICTION_SPEED_TOO_LOW]"
                    << " branch=" << branch_name
                    << " speed_mps=" << std::fixed << std::setprecision(3) << predSpeed
                    << " min_speed_mps=" << minTranslationPredictionSpeed
                    << " delta_m=" << predNorm
                    << " frame_stamp_s=" << std::fixed << std::setprecision(6) << timeLaserInfoCur
                    << " last_frame_stamp_s=" << timeLastProcessing
                    << " wall_now_s=" << wallNowSec
                    << " cur_dt_s=" << curTimeDiff
                    << " last_dt_s=" << lastTimeDiff;
            const std::string warn_msg = warn_ss.str();
            RCLCPP_WARN_STREAM(this->get_logger(), warn_msg);
            transIncre.translation().setZero();
            if (diagnostics)
                diagnostics->publishWarning(warn_msg);
        }
    };

    const auto applyConstantVelocityTranslationPrediction = [this](Eigen::Affine3f &transIncre, const char *branch_name) {
        constexpr double kMinLastDtSec = 1e-3;
        constexpr double kMaxScale = 10.0;

        if (!hasLastIncrementalDeltaPoseLocal || !std::isfinite(lastTimeDiff) || !std::isfinite(curTimeDiff) ||
            lastTimeDiff < kMinLastDtSec || curTimeDiff <= 0.0)
        {
            transIncre.translation().setZero();
            if (diagnostics)
            {
                const double wallNowSec = this->now().seconds();
                std::ostringstream ss;
                ss << "[TRANSLATION_PREDICTION_SKIPPED]"
                   << " branch=" << branch_name
                   << " reason=invalid_time_or_delta"
                   << " has_last_delta=" << (hasLastIncrementalDeltaPoseLocal ? 1 : 0)
                   << " frame_stamp_s=" << std::fixed << std::setprecision(6) << timeLaserInfoCur
                   << " last_frame_stamp_s=" << timeLastProcessing
                   << " wall_now_s=" << wallNowSec
                   << " cur_dt_s=" << std::fixed << std::setprecision(6) << curTimeDiff
                   << " last_dt_s=" << lastTimeDiff;
                diagnostics->logEventThrottle("translation_prediction_skipped_invalid_time_or_delta", 1.0, ss.str());
            }
            return;
        }

        const double scaleRaw = curTimeDiff / lastTimeDiff;
        const double scale = std::clamp(scaleRaw, 0.0, kMaxScale);

        if (diagnostics && std::abs(scale - scaleRaw) > 1e-9)
        {
                const double wallNowSec = this->now().seconds();
            std::ostringstream ss;
            ss << "[TRANSLATION_PREDICTION_SCALE_CLAMPED]"
               << " branch=" << branch_name
               << " scale_raw=" << std::fixed << std::setprecision(3) << scaleRaw
               << " scale_clamped=" << scale
                    << " frame_stamp_s=" << std::fixed << std::setprecision(6) << timeLaserInfoCur
                    << " last_frame_stamp_s=" << timeLastProcessing
                    << " wall_now_s=" << wallNowSec
               << " cur_dt_s=" << curTimeDiff
               << " last_dt_s=" << lastTimeDiff;
            diagnostics->logEventThrottle("translation_prediction_scale_clamped", 1.0, ss.str());
        }

        transIncre.translation() = lastIncrementalDeltaPoseLocal.translation() * static_cast<float>(scale);
    };

    static Eigen::Affine3f lastImuTransformation;
    // initialization
    if (cloudKeyPoses3D->points.empty())
    {
        transformTobeMapped[0] = cloudInfo.imurollinit;
        transformTobeMapped[1] = cloudInfo.imupitchinit;
        transformTobeMapped[2] = cloudInfo.imuyawinit;

        if (!useImuHeadingInitialization)
            transformTobeMapped[2] = 0;

        lastImuTransformation = pcl::getTransformation(0, 0, 0, cloudInfo.imurollinit, cloudInfo.imupitchinit, cloudInfo.imuyawinit); // save imu before return;
        return;
    }

    // use imu pre-integration estimation for pose guess
    static bool lastImuPreTransAvailable = false;
    static Eigen::Affine3f lastImuPreTransformation;
    if (cloudInfo.odomavailable == true)
    {
        Eigen::Affine3f transBack = pcl::getTransformation(cloudInfo.initialguessx,    cloudInfo.initialguessy,     cloudInfo.initialguessz, 
                                                           cloudInfo.initialguessroll, cloudInfo.initialguesspitch, cloudInfo.initialguessyaw);
        if (lastImuPreTransAvailable == false)
        {
            lastImuPreTransformation = transBack;
            lastImuPreTransAvailable = true;
        } else {
            Eigen::Affine3f transIncre = lastImuPreTransformation.inverse() * transBack;

            if (translationPredictionSource == TranslationPredictionSource::CONSTANT_VELOCITY)
            {
                applyConstantVelocityTranslationPrediction(transIncre, "imu_preintegration");
                clampTranslationPrediction(transIncre, "imu_preintegration");
                if (diagnostics)
                {
                    std::ostringstream diag_ss;
                    diag_ss << "Using constant velocity for translation prediction:" << std::endl
                            << transIncre.translation() << std::endl
                            << "curTimeDiff: " << curTimeDiff << std::endl
                            << "lastTimeDiff: " << lastTimeDiff;
                    diagnostics->logEventThrottle("constant_velocity_translation_prediction", 10.0, diag_ss.str());
                }
            }

            Eigen::Affine3f transTobe = trans2Affine3f(transformTobeMapped);
            Eigen::Affine3f transFinal = transTobe * transIncre;
            pcl::getTranslationAndEulerAngles(transFinal, transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5], 
                                                          transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);

            lastImuPreTransformation = transBack;

            lastImuTransformation = pcl::getTransformation(0, 0, 0, cloudInfo.imurollinit, cloudInfo.imupitchinit, cloudInfo.imuyawinit); // save imu before return;
            return;
        }
    }

    // use imu incremental estimation for pose guess (only rotation)
    if (cloudInfo.imuavailable == true && imuType)
    {
        Eigen::Affine3f transBack = pcl::getTransformation(0, 0, 0, cloudInfo.imurollinit, cloudInfo.imupitchinit, cloudInfo.imuyawinit);
        Eigen::Affine3f transIncre = lastImuTransformation.inverse() * transBack;

        if (translationPredictionSource == TranslationPredictionSource::CONSTANT_VELOCITY)
            {
                applyConstantVelocityTranslationPrediction(transIncre, "imu_incremental");
                clampTranslationPrediction(transIncre, "imu_incremental");
                if (diagnostics)
                {
                    std::ostringstream diag_ss;
                    diag_ss << "Using constant velocity for translation prediction:" << std::endl
                            << transIncre.translation() << std::endl
                            << "curTimeDiff: " << curTimeDiff << std::endl
                            << "lastTimeDiff: " << lastTimeDiff;
                    diagnostics->logEventThrottle("constant_velocity_translation_prediction", 10.0, diag_ss.str());
                }
            }

        Eigen::Affine3f transTobe = trans2Affine3f(transformTobeMapped);
        Eigen::Affine3f transFinal = transTobe * transIncre;
        pcl::getTranslationAndEulerAngles(transFinal, transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5], 
                                                    transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);

        lastImuTransformation = pcl::getTransformation(0, 0, 0, cloudInfo.imurollinit, cloudInfo.imupitchinit, cloudInfo.imuyawinit); // save imu before return;
        return;
    }
}


void mapOptimization::downsampleCurrentScan()
{
    laserCloudSurfLastDS->clear();
    downSizeFilterSurf.setInputCloud(laserCloudSurfLast);
    downSizeFilterSurf.filter(*laserCloudSurfLastDS);

    if (useSorFilter && !laserCloudSurfLastDS->empty())
    {
        pcl::StatisticalOutlierRemoval<PointType> sor;
        sor.setInputCloud(laserCloudSurfLastDS);
        sor.setMeanK(sorMeanK);
        sor.setStddevMulThresh(sorStddevMulThresh);
        sor.filter(*laserCloudSurfLastDS);
    }

    laserCloudSurfLastDSNum = laserCloudSurfLastDS->size();
}


void mapOptimization::scan2MapOptimization()
{
    if (cloudKeyPoses3D->points.empty())
        return;

    if (laserCloudSurfLastDSNum > 30)
    {
        auto alignOverrideConfig = mappingBackend->getAlignmentConfig();
        alignOverrideConfig.compute_jacobian_degeneracy = degeneracyDetection.jacobianBased.compute;
        alignOverrideConfig.jacobian_degeneracy_threshold = degeneracyDetection.jacobianBased.threshold;
        lio::AlignmentMetrics metrics = mappingBackend->align(laserCloudSurfLastDS, transformTobeMapped, alignOverrideConfig);
        this->isDegenerate = metrics.is_degenerate;

        if (degeneracyDetection.jacobianBased.log && diagnostics)
        {
            diagnostics->recordJacobianDegeneracyTelemetry(
                timeLaserInfoCur,
                backend_type,
                metrics.jacobian_degeneracy);
        }

        if (degeneracyDetection.enable)
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
                RCLCPP_WARN_STREAM_THROTTLE(get_logger(), *get_clock(), 1000, oss.str());
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
        
        if (true)
        {
            std::ostringstream oss;
            oss << "[SCAN2MAP_ITER] iter_used=" << metrics.iterations
                << " surf_total_ms=" << std::fixed << std::setprecision(3) << metrics.optimization_ms
                << " combine_total_ms=" << metrics.combine_ms
                << " lm_total_ms=" << metrics.lm_optimization_ms
                << " surf_input=" << metrics.input_count
                << " surf_knn_pass=" << metrics.knn_pass_count
                << " surf_plane_valid=" << metrics.plane_valid_count
                << " surf_matched=" << metrics.matched_count;
            diagnostics->logEventThrottle("scan2map_iter_summary", 1.0, oss.str());
        }
    } else {
        RCLCPP_WARN(get_logger(), "Not enough features! Only %d planar features available.", laserCloudSurfLastDSNum);
    }

    TicToc t_transformUpdate;
    transformUpdate();
    if (diagnostics)
        diagnostics->recordSlice("scan2MapOptimization.transformUpdate", t_transformUpdate.toc());
}

void mapOptimization::applyDegeneracyStateOverride(double dt_scan)
{
    if (degeneracyDetection.compensation_source == "none" || dt_scan <= 1e-5) return;

    // 1. Get the active basis and orthonormalize it
    auto basis = degeneracyDetector->getSparsifiedBasis();
    if (basis.empty()) return;
    
    auto orthoBasis = degeneracyDetector->orthonormalizeBasis(basis);
    if (orthoBasis.empty()) return;

    TwistVector xi_complementary_lidar = TwistVector::Zero();
    bool hasComplementaryPrediction = false;

    // 2. Infer complementary odometry body twist if requested.
    // If unavailable, we keep fallback prediction from incrementalOdometryAffineFront.
    if (degeneracyDetection.compensation_source == "complementary_odom")
    {
        std::lock_guard<std::mutex> lock(complementaryOdomMutex);
        if (complementaryOdomQueue.size() >= 2)
        {
            // Select complementary odometry samples closest to previous and current LiDAR stamps.
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

            if (idx_prev_near >= 0 && idx_curr_near >= 0)
            {
                const double t_prev_near = ROS_TIME(complementaryOdomQueue[idx_prev_near].header.stamp);
                const double t_curr_near = ROS_TIME(complementaryOdomQueue[idx_curr_near].header.stamp);
                const double prev_abs_dt = std::abs(t_prev_near - prev_lidar_time);
                const double curr_abs_dt = std::abs(t_curr_near - curr_lidar_time);

                // Keep a conservative staleness/association gate for both LiDAR-aligned picks.
                if (prev_abs_dt <= 0.25 && curr_abs_dt <= 0.25)
                {
                    int idx1 = idx_prev_near;
                    int idx2 = idx_curr_near;
                    if (ROS_TIME(complementaryOdomQueue[idx1].header.stamp) > ROS_TIME(complementaryOdomQueue[idx2].header.stamp))
                        std::swap(idx1, idx2);

                    // Resolve extrinsics
                    if (resolveComplementaryOdomExtrinsics(complementaryOdomQueue[idx2].header.frame_id))
                    {
                        double dt_add = ROS_TIME(complementaryOdomQueue[idx2].header.stamp) - ROS_TIME(complementaryOdomQueue[idx1].header.stamp);
                        if (dt_add >= 1e-3)
                        {
                            // Extract relative transform T_12 in complementary odometry frame
                            Eigen::Isometry3d iso1, iso2;
                            tf2::fromMsg(complementaryOdomQueue[idx1].pose.pose, iso1);
                            tf2::fromMsg(complementaryOdomQueue[idx2].pose.pose, iso2);
                            Eigen::Matrix4f T1 = iso1.matrix().cast<float>();
                            Eigen::Matrix4f T2 = iso2.matrix().cast<float>();
                            Eigen::Matrix4f T_add_delta = T1.inverse() * T2;

                            // Conjugate motion into LiDAR frame: T_l_delta = T_l_a * T_a_delta * T_a_l
                            Eigen::Matrix4f T_complementary_lidar_delta = T_complementary_to_lidar * T_add_delta * T_complementary_to_lidar.inverse();

                            // Convert to body twist (velocity)
                            xi_complementary_lidar = matrixToTwist(T_complementary_lidar_delta, static_cast<float>(dt_add));
                            hasComplementaryPrediction = true;

                            std::ostringstream oss;
                            oss << "[COMPLEMENTARY_ODOM_TWIST]"
                                << " dt_add_s=" << std::fixed << std::setprecision(4) << dt_add
                                << " prev_match_abs_dt_s=" << prev_abs_dt
                                << " curr_match_abs_dt_s=" << curr_abs_dt
                                << " lin_xyz=["
                                << xi_complementary_lidar[0] << ", " << xi_complementary_lidar[1] << ", " << xi_complementary_lidar[2] << "]"
                                << " ang_xyz=["
                                << xi_complementary_lidar[3] << ", " << xi_complementary_lidar[4] << ", " << xi_complementary_lidar[5] << "]"
                                << " lin_norm=" << xi_complementary_lidar.head<3>().norm()
                                << " ang_norm=" << xi_complementary_lidar.tail<3>().norm();
                            RCLCPP_INFO_STREAM_THROTTLE(get_logger(), *get_clock(), 1000, oss.str());
                        }
                    }
                }
            }
        }
    }

    // 3. Build prediction and preserve optimized solution in non-degenerate directions.
    const Eigen::Affine3f T_optimized = trans2Affine3f(transformTobeMapped);

    Eigen::Affine3f T_previous = incrementalOdometryAffineFront;
    Eigen::Affine3f T_complementary_odom = T_previous;

    bool hasNonDegenerateComponents = false;
    Eigen::Vector3f t_lidar_nondeg_map = Eigen::Vector3f::Zero();
    Eigen::Vector3f t_complementary_nondeg_map = Eigen::Vector3f::Zero();

    if (hasComplementaryPrediction)
    {
        Eigen::Matrix4f T_lidar_complementary_predicted = expMap(xi_complementary_lidar, static_cast<float>(dt_scan));

        // Estimate the complementary-odom translation scale from the non-degenerate
        // translation subspace, where the scan matcher is trusted. Both deltas
        // are local (body-frame) displacements relative to T_previous.
        const Eigen::Vector3f t_lidar =
            (T_previous.matrix().inverse() * T_optimized.matrix()).block<3, 1>(0, 3);
        const Eigen::Vector3f t_complementary = T_lidar_complementary_predicted.block<3, 1>(0, 3);

        const Eigen::Vector3f t_lidar_nondeg = t_lidar - projectOntoBasisTranslation(t_lidar, orthoBasis);
        const Eigen::Vector3f t_complementary_nondeg = t_complementary - projectOntoBasisTranslation(t_complementary, orthoBasis);

        // Observability gate: require a minimum linear speed in the
        // non-degenerate subspace for both displacements, otherwise the
        // scale ratio is noise-driven and we fall back to 1.0.
        constexpr float kMinScale = 0.2f;
        constexpr float kMaxScale = 5.0f;
        const float minNondegNorm =
            static_cast<float>(complementaryOdom.scaleMinNonDegenerateSpeed) * static_cast<float>(dt_scan);

        const float lidarNondegNorm = t_lidar_nondeg.norm();
        const float complementaryNondegNorm = t_complementary_nondeg.norm();
        const bool gatePassed = lidarNondegNorm > minNondegNorm && complementaryNondegNorm > minNondegNorm;

        float complementaryOdomScale = 1.0f;
        float scaleRatioRaw = std::numeric_limits<float>::quiet_NaN();
        float scaleLsRaw = std::numeric_limits<float>::quiet_NaN();
        float thetaDeg = std::numeric_limits<float>::quiet_NaN();

        if (gatePassed)
        {
            // Applied estimate: ratio of non-degenerate norms.
            scaleRatioRaw = lidarNondegNorm / complementaryNondegNorm;

            // Reference estimates for data analysis: least-squares scale and
            // direction mismatch angle between the non-degenerate components.
            const float dot = t_complementary_nondeg.dot(t_lidar_nondeg);
            scaleLsRaw = dot / (complementaryNondegNorm * complementaryNondegNorm);
            const float cosTheta = std::clamp(dot / (complementaryNondegNorm * lidarNondegNorm), -1.0f, 1.0f);
            thetaDeg = std::acos(cosTheta) * 180.0f / static_cast<float>(M_PI);

            if (complementaryOdom.scaleEstimationEnabled)
                complementaryOdomScale = std::clamp(scaleRatioRaw, kMinScale, kMaxScale);

            // Expose non-degenerate components (map frame) for visualization.
            hasNonDegenerateComponents = true;
            const Eigen::Matrix3f R_prev = T_previous.rotation();
            t_lidar_nondeg_map = R_prev * t_lidar_nondeg;
            t_complementary_nondeg_map = R_prev * t_complementary_nondeg;
        }

        const float complementaryOdomLinearSpeedOrig = xi_complementary_lidar.head<3>().norm();
        const float lidarLinearSpeedNondeg = lidarNondegNorm / static_cast<float>(dt_scan);
        const float complementaryOdomLinearSpeedNondeg = complementaryNondegNorm / static_cast<float>(dt_scan);
        const float lidarLinearSpeedAfterScale =
            (complementaryNondegNorm * complementaryOdomScale) / static_cast<float>(dt_scan);

        // Baseline first correction (scale fixed to 1): project optimized-to-
        // predicted displacement onto the degenerate subspace and report
        // equivalent linear speed from that projected motion.
        const Eigen::Affine3f T_complementary_odom_scale1(
            T_previous.matrix() * T_lidar_complementary_predicted);
        const Eigen::Matrix4f T_diff_scale1 =
            T_optimized.matrix().inverse() * T_complementary_odom_scale1.matrix();
        const TwistVector xi_diff_scale1 = matrixToTwist(T_diff_scale1);
        const TwistVector xi_proj_scale1 = projectOntoBasis(xi_diff_scale1, orthoBasis);
        const Eigen::Matrix4f T_proj_motion_scale1 = expMap(xi_proj_scale1);
        const float lidarLinearSpeedProjScale1 =
            T_proj_motion_scale1.block<3, 1>(0, 3).norm() / static_cast<float>(dt_scan);

        {
            std::ostringstream oss;
            oss << "[COMPLEMENTARY_ODOM_SCALE]"
                << " frame_stamp_s=" << std::fixed << std::setprecision(6) << timeLaserInfoCur
                << std::setprecision(4)
                << " enabled=" << (complementaryOdom.scaleEstimationEnabled ? 1 : 0)
                << " gate_passed=" << (gatePassed ? 1 : 0)
                << " scale_applied=" << complementaryOdomScale
                << " scale_ratio_raw=" << scaleRatioRaw
                << " scale_ls_raw=" << scaleLsRaw
                << " theta_deg=" << thetaDeg
                << " lidar_nondeg_m=" << lidarNondegNorm
                << " complementary_nondeg_m=" << complementaryNondegNorm
                << " min_nondeg_m=" << minNondegNorm
                << " complementary_odom_lin_speed_orig_mps=" << complementaryOdomLinearSpeedOrig
                << " lidar_lin_speed_nondeg_mps=" << lidarLinearSpeedNondeg
                << " complementary_odom_lin_speed_nondeg_mps=" << complementaryOdomLinearSpeedNondeg
                << " lidar_lin_speed_proj_scale1_mps=" << lidarLinearSpeedProjScale1
                << " lidar_lin_speed_after_scale_mps=" << lidarLinearSpeedAfterScale
                << " dt_scan_s=" << dt_scan;
            const std::string msg = oss.str();
            if (diagnostics)
                diagnostics->logEventThrottle("complementary_odom_scale", 0.0, msg); // every degeneracy frame
            RCLCPP_INFO_STREAM_THROTTLE(get_logger(), *get_clock(), 1000, msg);
        }

        // Rescale only the translation of the complementary-odom displacement; its
        // rotation is typically gyro-backed and kept as-is.
        T_lidar_complementary_predicted.block<3, 1>(0, 3) *= complementaryOdomScale;
        T_complementary_odom = Eigen::Affine3f(T_previous.matrix() * T_lidar_complementary_predicted);
    }

    // 4. Project only the optimized-to-predicted displacement onto degenerate subspace.
    const Eigen::Matrix4f T_diff = T_optimized.matrix().inverse() * T_complementary_odom.matrix();
    const TwistVector xi_diff = matrixToTwist(T_diff);
    const TwistVector xi_proj = projectOntoBasis(xi_diff, orthoBasis);

    const Eigen::Matrix4f T_proj_motion = expMap(xi_proj);
    Eigen::Affine3f T_corrected(T_optimized.matrix() * T_proj_motion);

    // 5. Publish debug arrows/poses from optimized base to predicted and projected endpoints.
    publishComplementaryOdomDisplacementDebug(T_previous, T_complementary_odom, T_corrected,
                                    hasNonDegenerateComponents, t_lidar_nondeg_map, t_complementary_nondeg_map);

    float roll, pitch, yaw, x, y, z;
    pcl::getTranslationAndEulerAngles(T_corrected, x, y, z, roll, pitch, yaw);
    transformTobeMapped[0] = roll;
    transformTobeMapped[1] = pitch;
    transformTobeMapped[2] = yaw;
    transformTobeMapped[3] = x;
    transformTobeMapped[4] = y;
    transformTobeMapped[5] = z;
}

void mapOptimization::publishComplementaryOdomDisplacementDebug(const Eigen::Affine3f &T_base_abs,
                                                      const Eigen::Affine3f &T_raw_abs,
                                                      const Eigen::Affine3f &T_proj_abs,
                                                      bool hasNonDegenerateComponents,
                                                      const Eigen::Vector3f &t_lidar_nondeg_map,
                                                      const Eigen::Vector3f &t_complementary_nondeg_map)
{
    if (!pubComplementaryOdomCorrectionDirection)
        return;

    const auto stamp = timeLaserInfoStamp;

    if (pubComplementaryOdomCorrectionDirection && pubComplementaryOdomCorrectionDirection->get_subscription_count() > 0)
    {
        visualization_msgs::msg::MarkerArray markers;

        visualization_msgs::msg::Marker delete_all;
        delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
        markers.markers.push_back(delete_all);

        geometry_msgs::msg::Point p_base;
        p_base.x = T_base_abs.translation().x();
        p_base.y = T_base_abs.translation().y();
        p_base.z = T_base_abs.translation().z();

        geometry_msgs::msg::Point p_raw;
        p_raw.x = T_raw_abs.translation().x();
        p_raw.y = T_raw_abs.translation().y();
        p_raw.z = T_raw_abs.translation().z();

        geometry_msgs::msg::Point p_proj;
        p_proj.x = T_proj_abs.translation().x();
        p_proj.y = T_proj_abs.translation().y();
        p_proj.z = T_proj_abs.translation().z();

        double additional_arrow_scale = 0.2;

        visualization_msgs::msg::Marker raw_arrow;
        raw_arrow.header.frame_id = mapFrameLocal;
        raw_arrow.header.stamp = stamp;
        raw_arrow.ns = "complementary_odom_correction_raw";
        raw_arrow.id = 0;
        raw_arrow.type = visualization_msgs::msg::Marker::ARROW;
        raw_arrow.action = visualization_msgs::msg::Marker::ADD;
        raw_arrow.scale.x = 0.08 * additional_arrow_scale;
        raw_arrow.scale.y = 0.16 * additional_arrow_scale;
        raw_arrow.scale.z = 0.16 * additional_arrow_scale;
        raw_arrow.color.r = 1.0;
        raw_arrow.color.g = 0.55;
        raw_arrow.color.b = 0.0;
        raw_arrow.color.a = 0.9;
        raw_arrow.points.push_back(p_base);
        raw_arrow.points.push_back(p_raw);
        markers.markers.push_back(raw_arrow);

        visualization_msgs::msg::Marker proj_arrow;
        proj_arrow.header.frame_id = mapFrameLocal;
        proj_arrow.header.stamp = stamp;
        proj_arrow.ns = "complementary_odom_correction_projected";
        proj_arrow.id = 1;
        proj_arrow.type = visualization_msgs::msg::Marker::ARROW;
        proj_arrow.action = visualization_msgs::msg::Marker::ADD;
        proj_arrow.scale.x = 0.10 * additional_arrow_scale;
        proj_arrow.scale.y = 0.20 * additional_arrow_scale;
        proj_arrow.scale.z = 0.20 * additional_arrow_scale;

        proj_arrow.color.r = 0.0;
        proj_arrow.color.g = 0.95;
        proj_arrow.color.b = 0.95;
        proj_arrow.color.a = 0.95;
        proj_arrow.points.push_back(p_base);
        proj_arrow.points.push_back(p_proj);
        markers.markers.push_back(proj_arrow);

        // Non-degenerate translation components (only when the observability
        // gate passed and the scale estimate was applied).
        if (hasNonDegenerateComponents)
        {
            const auto makeNondegArrow = [&](int id, const char *ns_name,
                                             const Eigen::Vector3f &v_map,
                                             float r, float g, float b) {
                visualization_msgs::msg::Marker arrow;
                arrow.header.frame_id = mapFrameLocal;
                arrow.header.stamp = stamp;
                arrow.ns = ns_name;
                arrow.id = id;
                arrow.type = visualization_msgs::msg::Marker::ARROW;
                arrow.action = visualization_msgs::msg::Marker::ADD;
                arrow.scale.x = 0.10 * additional_arrow_scale;
                arrow.scale.y = 0.20 * additional_arrow_scale;
                arrow.scale.z = 0.20 * additional_arrow_scale;
                arrow.color.r = r;
                arrow.color.g = g;
                arrow.color.b = b;
                arrow.color.a = 0.95;
                geometry_msgs::msg::Point p_end;
                p_end.x = p_base.x + v_map.x();
                p_end.y = p_base.y + v_map.y();
                p_end.z = p_base.z + v_map.z();
                arrow.points.push_back(p_base);
                arrow.points.push_back(p_end);
                return arrow;
            };

            markers.markers.push_back(makeNondegArrow(
                2, "nondeg_lidar_displacement", t_lidar_nondeg_map, 0.2f, 1.0f, 0.2f)); // Green
            markers.markers.push_back(makeNondegArrow(
                3, "nondeg_complementary_odom_displacement", t_complementary_nondeg_map, 1.0f, 0.0f, 1.0f)); // Magenta
        }

        pubComplementaryOdomCorrectionDirection->publish(markers);
    }
}

void mapOptimization::transformUpdate()
{
    if (cloudInfo.imuavailable == true && imuType)
    {
        if (std::abs(cloudInfo.imupitchinit) < 1.4)
        {
            double imuWeight = imuRPYWeight;
            tf2::Quaternion imuQuaternion;
            tf2::Quaternion transformQuaternion;
            double rollMid, pitchMid, yawMid;

            // slerp roll
            transformQuaternion.setRPY(transformTobeMapped[0], 0, 0);
            imuQuaternion.setRPY(cloudInfo.imurollinit, 0, 0);
            tf2::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);
            transformTobeMapped[0] = rollMid;

            // slerp pitch
            transformQuaternion.setRPY(0, transformTobeMapped[1], 0);
            imuQuaternion.setRPY(0, cloudInfo.imupitchinit, 0);
            tf2::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);
            transformTobeMapped[1] = pitchMid;
        }
    }

    transformTobeMapped[0] = constraintTransformation(transformTobeMapped[0], rotation_tollerance);
    transformTobeMapped[1] = constraintTransformation(transformTobeMapped[1], rotation_tollerance);
    transformTobeMapped[5] = constraintTransformation(transformTobeMapped[5], z_tollerance);

    incrementalOdometryAffineBack = trans2Affine3f(transformTobeMapped);
}

float mapOptimization::constraintTransformation(float value, float limit)
{
    if (value < -limit)
        value = -limit;
    if (value > limit)
        value = limit;

    return value;
}

bool mapOptimization::saveFrame()
{
    if (cloudKeyPoses3D->points.empty())
        return true;

    Eigen::Affine3f transStart = pclPointToAffine3f(cloudKeyPoses6D->back());
    Eigen::Affine3f transFinal = pcl::getTransformation(transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5],
                                                        transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
    Eigen::Affine3f transBetween = transStart.inverse() * transFinal;
    float x, y, z, roll, pitch, yaw;
    pcl::getTranslationAndEulerAngles(transBetween, x, y, z, roll, pitch, yaw);

    if (abs(roll)  < surroundingkeyframeAddingAngleThreshold &&
        abs(pitch) < surroundingkeyframeAddingAngleThreshold &&
        abs(yaw)   < surroundingkeyframeAddingAngleThreshold &&
        sqrt(x*x + y*y + z*z) < surroundingkeyframeAddingDistThreshold)
        return false;

    return true;
}

