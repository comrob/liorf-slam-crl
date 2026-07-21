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
            runDegeneracyDetectionAndCompensation();
        
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


