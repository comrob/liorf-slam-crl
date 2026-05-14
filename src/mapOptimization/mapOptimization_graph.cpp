#include "mapOptimization/mapOptimization.hpp"

#include <cctype>
#include <gtsam/linear/linearExceptions.h>

namespace
{
std::string formatGtsamKey(const gtsam::Key key)
{
    constexpr uint64_t kIndexMask = (uint64_t{1} << 56) - 1;
    const uint8_t chr = static_cast<uint8_t>((key >> 56) & 0xFF);
    const uint64_t index = key & kIndexMask;

    std::ostringstream oss;
    if (chr == 0)
    {
        oss << "pose_idx=" << key << " (plain integer key)";
        return oss.str();
    }

    if (std::isprint(chr))
    {
        oss << "symbol=" << static_cast<char>(chr) << index << " (raw_key=" << key << ")";
        return oss.str();
    }

    oss << "symbol_byte=" << static_cast<int>(chr) << ",index=" << index << " (raw_key=" << key << ")";
    return oss.str();
}

std::string summarizePendingFactors(const gtsam::NonlinearFactorGraph &graph, const size_t max_factors)
{
    std::ostringstream oss;
    const size_t total = graph.size();
    const size_t begin = (total > max_factors) ? (total - max_factors) : 0;
    oss << "pending_factors_total=" << total << " showing_last=" << (total - begin);

    for (size_t i = begin; i < total; ++i)
    {
        const auto &factor = graph.at(i);
        if (!factor)
        {
            oss << " | f" << i << "=<null>";
            continue;
        }

        oss << " | f" << i << " keys=[";
        const auto &keys = factor->keys();
        for (size_t k = 0; k < keys.size(); ++k)
        {
            if (k > 0)
                oss << ", ";
            oss << formatGtsamKey(keys[k]);
        }
        oss << "]";
    }

    return oss.str();
}

std::string summarizeInitialEstimateKeys(const gtsam::Values &values, const size_t max_keys)
{
    std::ostringstream oss;
    oss << "initial_estimate_count=" << values.size() << " keys=[";

    size_t printed = 0;
    for (auto it = values.begin(); it != values.end(); ++it)
    {
        if (printed > 0)
            oss << ", ";
        if (printed >= max_keys)
        {
            oss << "...";
            break;
        }
        oss << formatGtsamKey(it->key);
        ++printed;
    }
    oss << "]";
    return oss.str();
}
} // namespace

using gtsam::BetweenFactor;
using gtsam::Pose3;
using gtsam::PriorFactor;
namespace noiseModel = gtsam::noiseModel;

void mapOptimization::addOdomFactor()
{
    if (cloudKeyPoses3D->points.empty())
    {
        // Lock the local trajectory's origin completely to force T_GL to absorb all global drift
        noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6).finished());
        gtSAMgraph.add(PriorFactor<Pose3>(0, trans2gtsamPose(transformTobeMapped), priorNoise));
        initialEstimate.insert(0, trans2gtsamPose(transformTobeMapped));
    }
    else
    {
        noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
        gtsam::Pose3 poseFrom = pclPointTogtsamPose3(cloudKeyPoses6D->points.back());
        gtsam::Pose3 poseTo = trans2gtsamPose(transformTobeMapped);
        gtSAMgraph.add(BetweenFactor<Pose3>(cloudKeyPoses3D->size() - 1, cloudKeyPoses3D->size(), poseFrom.between(poseTo), odometryNoise));
        initialEstimate.insert(cloudKeyPoses3D->size(), poseTo);
    }
}

void mapOptimization::addLoopFactor()
{
    if (loopIndexQueue.empty())
        return;

    for (int i = 0; i < (int)loopIndexQueue.size(); ++i)
    {
        int indexFrom = loopIndexQueue[i].first;
        int indexTo = loopIndexQueue[i].second;
        gtsam::Pose3 poseBetween = loopPoseQueue[i];
        // gtsam::noiseModel::Diagonal::shared_ptr noiseBetween = loopNoiseQueue[i];
        auto noiseBetween = loopNoiseQueue[i];
        gtSAMgraph.add(BetweenFactor<Pose3>(indexFrom, indexTo, poseBetween, noiseBetween));
    }

    loopIndexQueue.clear();
    loopPoseQueue.clear();
    loopNoiseQueue.clear();

    if (rebuild_on_loop_closure)
        markMapRebuildTriggered("loop_closure_factor");

    aLoopIsClosed = true;
}

bool mapOptimization::saveKeyFramesAndFactor()
{
    if (saveFrame() == false)
        return false;

    // odom factor
    addOdomFactor();

    // gps factor
    addGPSFactor();

    // loop factor
    addLoopFactor();

    // cout << "****************************************************" << endl;
    // gtSAMgraph.print("GTSAM Graph:\n");

    // update iSAM
    try
    {
        isam->update(gtSAMgraph, initialEstimate);
        isam->update();

        if (aLoopIsClosed == true)
        {
            isam->update();
            isam->update();
            isam->update();
            isam->update();
            isam->update();
        }
    }
    catch (const gtsam::IndeterminantLinearSystemException &e)
    {
        const size_t latest_pose_candidate = cloudKeyPoses3D->size();
        std::ostringstream err_ss;
        err_ss << "[ISAM_UPDATE_FAIL] type=IndeterminantLinearSystemException"
               << " msg=\"" << e.what() << "\""
               << " ros_stamp_s=" << std::fixed << std::setprecision(6) << timeLaserInfoCur
               << " clock_now_s=" << this->now().seconds()
               << " latest_pose_candidate=" << latest_pose_candidate
               << " cloud_keyposes_3d_size=" << cloudKeyPoses3D->size()
               << " cloud_keyposes_6d_size=" << cloudKeyPoses6D->size()
               << " gps_queue_size=" << gpsQueue.size()
               << " gps_factors_accepted=" << gpsFactorsAccepted;

        RCLCPP_ERROR_STREAM(get_logger(), err_ss.str());
        RCLCPP_ERROR_STREAM(get_logger(), "[ISAM_UPDATE_FAIL_CONTEXT] " << summarizePendingFactors(gtSAMgraph, 10));
        RCLCPP_ERROR_STREAM(get_logger(), "[ISAM_UPDATE_FAIL_CONTEXT] " << summarizeInitialEstimateKeys(initialEstimate, 20));

        if (diagnostics)
        {
            diagnostics->logEvent(err_ss.str());
            diagnostics->logEvent("[ISAM_UPDATE_FAIL_CONTEXT] " + summarizePendingFactors(gtSAMgraph, 10));
            diagnostics->logEvent("[ISAM_UPDATE_FAIL_CONTEXT] " + summarizeInitialEstimateKeys(initialEstimate, 20));
        }

        gtSAMgraph.resize(0);
        initialEstimate.clear();
        aLoopIsClosed = false;
        return false;
    }
    catch (const std::exception &e)
    {
        std::ostringstream err_ss;
        err_ss << "[ISAM_UPDATE_FAIL] type=std::exception"
               << " msg=\"" << e.what() << "\""
               << " ros_stamp_s=" << std::fixed << std::setprecision(6) << timeLaserInfoCur
               << " clock_now_s=" << this->now().seconds();
        RCLCPP_ERROR_STREAM(get_logger(), err_ss.str());
        RCLCPP_ERROR_STREAM(get_logger(), "[ISAM_UPDATE_FAIL_CONTEXT] " << summarizePendingFactors(gtSAMgraph, 10));
        RCLCPP_ERROR_STREAM(get_logger(), "[ISAM_UPDATE_FAIL_CONTEXT] " << summarizeInitialEstimateKeys(initialEstimate, 20));

        if (diagnostics)
        {
            diagnostics->logEvent(err_ss.str());
            diagnostics->logEvent("[ISAM_UPDATE_FAIL_CONTEXT] " + summarizePendingFactors(gtSAMgraph, 10));
            diagnostics->logEvent("[ISAM_UPDATE_FAIL_CONTEXT] " + summarizeInitialEstimateKeys(initialEstimate, 20));
        }

        gtSAMgraph.resize(0);
        initialEstimate.clear();
        aLoopIsClosed = false;
        return false;
    }

    gtSAMgraph.resize(0);
    initialEstimate.clear();

    //save key poses
    PointType thisPose3D;
    PointTypePose thisPose6D;
    Pose3 latestEstimate;

    isamCurrentEstimate = isam->calculateEstimate();
    if (T_EL_initialized && isamCurrentEstimate.exists(T_EL_KEY))
    {
        T_EL_estimate = isamCurrentEstimate.at<Pose3>(T_EL_KEY);
    }
    const int latestPoseKey = cloudKeyPoses3D->size(); // capture before push_back
    latestEstimate = isamCurrentEstimate.at<Pose3>(latestPoseKey);
    // cout << "****************************************************" << endl;
    // isamCurrentEstimate.print("Current estimate: ");

    thisPose3D.x = latestEstimate.translation().x();
    thisPose3D.y = latestEstimate.translation().y();
    thisPose3D.z = latestEstimate.translation().z();
    thisPose3D.intensity = cloudKeyPoses3D->size(); // this can be used as index
    cloudKeyPoses3D->push_back(thisPose3D);

    thisPose6D.x = thisPose3D.x;
    thisPose6D.y = thisPose3D.y;
    thisPose6D.z = thisPose3D.z;
    thisPose6D.intensity = thisPose3D.intensity; // this can be used as index
    thisPose6D.roll = latestEstimate.rotation().roll();
    thisPose6D.pitch = latestEstimate.rotation().pitch();
    thisPose6D.yaw = latestEstimate.rotation().yaw();
    thisPose6D.time = timeLaserInfoCur;
    cloudKeyPoses6D->push_back(thisPose6D);

    // cout << "****************************************************" << endl;
    // cout << "Pose covariance:" << endl;
    // cout << isam->marginalCovariance(latestPoseKey) << endl << endl;
    poseCovariance = isam->marginalCovariance(latestPoseKey);

    // save updated transform
    transformTobeMapped[0] = latestEstimate.rotation().roll();
    transformTobeMapped[1] = latestEstimate.rotation().pitch();
    transformTobeMapped[2] = latestEstimate.rotation().yaw();
    transformTobeMapped[3] = latestEstimate.translation().x();
    transformTobeMapped[4] = latestEstimate.translation().y();
    transformTobeMapped[5] = latestEstimate.translation().z();

    // save all the received edge and surf points
    pcl::PointCloud<PointType>::Ptr thisSurfKeyFrame(new pcl::PointCloud<PointType>());
    pcl::copyPointCloud(*laserCloudSurfLastDS, *thisSurfKeyFrame);

    // save key frame cloud
    surfCloudKeyFrames.push_back(thisSurfKeyFrame);
    keyframeScanAdmissible.push_back(cloudInfo.scan_admission_ok ? 1 : 0);

    if (!cloudInfo.scan_admission_ok && diagnostics)
    {
        std::ostringstream oss;
        oss << "[KEYFRAME_LOCAL_MAP_SKIP]"
            << " keyframe_idx=" << (cloudKeyPoses3D->size() - 1)
            << " max_angular_speed_rad_s=" << cloudInfo.scan_max_angular_speed
            << " roll_span_rad=" << cloudInfo.scan_roll_span
            << " pitch_span_rad=" << cloudInfo.scan_pitch_span
            << " yaw_span_rad=" << cloudInfo.scan_yaw_span;
        diagnostics->logEventThrottle("keyframe_local_map_skip", 1.0, oss.str());
    }

    // The following code is copy from sc-lio-sam
    // Scan Context loop detector - giseop
    // - SINGLE_SCAN_FULL: using downsampled original point cloud (/full_cloud_projected + downsampling)
    // - SINGLE_SCAN_FEAT: using surface feature as an input point cloud for scan context (2020.04.01: checked it works.)
    // - MULTI_SCAN_FEAT: using NearKeyframes (because a MulRan scan does not have beyond region, so to solve this issue ... )
    const SCInputType sc_input_type = SCInputType::SINGLE_SCAN_FULL; // change this

    if (sc_input_type == SCInputType::SINGLE_SCAN_FULL)
    {
        pcl::PointCloud<PointType>::Ptr thisRawCloudKeyFrame(new pcl::PointCloud<PointType>());
        pcl::fromROSMsg(cloudInfo.cloud_deskewed, *thisRawCloudKeyFrame);

        scManager.makeAndSaveScancontextAndKeys(*thisRawCloudKeyFrame);
    }
    else if (sc_input_type == SCInputType::SINGLE_SCAN_FEAT)
    {
        scManager.makeAndSaveScancontextAndKeys(*thisSurfKeyFrame);
    }
    else if (sc_input_type == SCInputType::MULTI_SCAN_FEAT)
    {
        pcl::PointCloud<PointType>::Ptr multiKeyFrameFeatureCloud(new pcl::PointCloud<PointType>());
        loopFindNearKeyframes(multiKeyFrameFeatureCloud, cloudKeyPoses6D->size() - 1, historyKeyframeSearchNum, -1);
        scManager.makeAndSaveScancontextAndKeys(*multiKeyFrameFeatureCloud);
    }

    // save path for visualization
    updatePath(thisPose6D);

    return true;
}

void mapOptimization::correctPoses()
{
    if (cloudKeyPoses3D->points.empty())
        return;

    if (aLoopIsClosed == true)
    {
        // clear map cache
        laserCloudMapContainer.clear();
        // clear path
        globalPath.poses.clear();
        // update key poses
        int numPoses = cloudKeyPoses3D->size();
        for (int i = 0; i < numPoses; ++i)
        {
            cloudKeyPoses3D->points[i].x = isamCurrentEstimate.at<Pose3>(i).translation().x();
            cloudKeyPoses3D->points[i].y = isamCurrentEstimate.at<Pose3>(i).translation().y();
            cloudKeyPoses3D->points[i].z = isamCurrentEstimate.at<Pose3>(i).translation().z();

            cloudKeyPoses6D->points[i].x = cloudKeyPoses3D->points[i].x;
            cloudKeyPoses6D->points[i].y = cloudKeyPoses3D->points[i].y;
            cloudKeyPoses6D->points[i].z = cloudKeyPoses3D->points[i].z;
            cloudKeyPoses6D->points[i].roll = isamCurrentEstimate.at<Pose3>(i).rotation().roll();
            cloudKeyPoses6D->points[i].pitch = isamCurrentEstimate.at<Pose3>(i).rotation().pitch();
            cloudKeyPoses6D->points[i].yaw = isamCurrentEstimate.at<Pose3>(i).rotation().yaw();

            updatePath(cloudKeyPoses6D->points[i]);
        }

        aLoopIsClosed = false;
    }
}
