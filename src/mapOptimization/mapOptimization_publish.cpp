#include "mapOptimization/mapOptimization.hpp"


pcl::PointCloud<PointType>::Ptr mapOptimization::transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose* transformIn)
{
    pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

    int cloudSize = cloudIn->size();
    cloudOut->resize(cloudSize);

    Eigen::Affine3f transCur = pcl::getTransformation(transformIn->x, transformIn->y, transformIn->z, transformIn->roll, transformIn->pitch, transformIn->yaw);
    
    #pragma omp parallel for num_threads(numberOfCores)
    for (int i = 0; i < cloudSize; ++i)
    {
        const auto &pointFrom = cloudIn->points[i];
        cloudOut->points[i].x = transCur(0,0) * pointFrom.x + transCur(0,1) * pointFrom.y + transCur(0,2) * pointFrom.z + transCur(0,3);
        cloudOut->points[i].y = transCur(1,0) * pointFrom.x + transCur(1,1) * pointFrom.y + transCur(1,2) * pointFrom.z + transCur(1,3);
        cloudOut->points[i].z = transCur(2,0) * pointFrom.x + transCur(2,1) * pointFrom.y + transCur(2,2) * pointFrom.z + transCur(2,3);
        cloudOut->points[i].intensity = pointFrom.intensity;
    }
    return cloudOut;
}

gtsam::Pose3 mapOptimization::pclPointTogtsamPose3(PointTypePose thisPoint)
{
    return gtsam::Pose3(gtsam::Rot3::RzRyRx(double(thisPoint.roll), double(thisPoint.pitch), double(thisPoint.yaw)),
                              gtsam::Point3(double(thisPoint.x),    double(thisPoint.y),     double(thisPoint.z)));
}

gtsam::Pose3 mapOptimization::trans2gtsamPose(float transformIn[])
{
    return gtsam::Pose3(gtsam::Rot3::RzRyRx(transformIn[0], transformIn[1], transformIn[2]), 
                              gtsam::Point3(transformIn[3], transformIn[4], transformIn[5]));
}

Eigen::Affine3f mapOptimization::pclPointToAffine3f(PointTypePose thisPoint)
{ 
    return pcl::getTransformation(thisPoint.x, thisPoint.y, thisPoint.z, thisPoint.roll, thisPoint.pitch, thisPoint.yaw);
}

Eigen::Affine3f mapOptimization::trans2Affine3f(float transformIn[])
{
    return pcl::getTransformation(transformIn[3], transformIn[4], transformIn[5], transformIn[0], transformIn[1], transformIn[2]);
}

PointTypePose mapOptimization::trans2PointTypePose(float transformIn[])
{
    PointTypePose thisPose6D;
    thisPose6D.x = transformIn[3];
    thisPose6D.y = transformIn[4];
    thisPose6D.z = transformIn[5];
    thisPose6D.roll  = transformIn[0];
    thisPose6D.pitch = transformIn[1];
    thisPose6D.yaw   = transformIn[2];
    return thisPose6D;
}

nav_msgs::msg::Odometry mapOptimization::odometryMsgFromAffine(
    const Eigen::Affine3f &affine,
    const rclcpp::Time &stamp,
    const std::string &frameId,
    const std::string &childFrameId)
{
    float x, y, z, roll, pitch, yaw;
    pcl::getTranslationAndEulerAngles(affine, x, y, z, roll, pitch, yaw);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = frameId;
    odom.child_frame_id = childFrameId;
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = z;

    tf2::Quaternion quat_tf;
    quat_tf.setRPY(roll, pitch, yaw);
    geometry_msgs::msg::Quaternion quat_msg;
    tf2::convert(quat_tf, quat_msg);
    odom.pose.pose.orientation = quat_msg;
    return odom;
}

tf2::Transform mapOptimization::tfFromAffine(const Eigen::Affine3f &affine) const
{
    float x, y, z, roll, pitch, yaw;
    pcl::getTranslationAndEulerAngles(affine, x, y, z, roll, pitch, yaw);
    tf2::Quaternion quat_tf;
    quat_tf.setRPY(roll, pitch, yaw);
    return tf2::Transform(quat_tf, tf2::Vector3(x, y, z));
}

Eigen::Affine3f mapOptimization::affineFromTf(const tf2::Transform &transform) const
{
    double roll, pitch, yaw;
    tf2::Matrix3x3(transform.getRotation()).getRPY(roll, pitch, yaw);
    return pcl::getTransformation(
        transform.getOrigin().x(),
        transform.getOrigin().y(),
        transform.getOrigin().z(),
        roll,
        pitch,
        yaw);
}


void mapOptimization::updatePath(const PointTypePose& pose_in)
{
    geometry_msgs::msg::PoseStamped pose_stamped;
    rclcpp::Time t(static_cast<int64_t>(pose_in.time * 1e9));

    pose_stamped.header.stamp = t;
    pose_stamped.header.frame_id = mapFrameLocal;
    pose_stamped.pose.position.x = pose_in.x;
    pose_stamped.pose.position.y = pose_in.y;
    pose_stamped.pose.position.z = pose_in.z;
    tf2::Quaternion q;
    q.setRPY(pose_in.roll, pose_in.pitch, pose_in.yaw);
    pose_stamped.pose.orientation.x = q.x();
    pose_stamped.pose.orientation.y = q.y();
    pose_stamped.pose.orientation.z = q.z();
    pose_stamped.pose.orientation.w = q.w();

    globalPath.poses.push_back(pose_stamped);
}

void mapOptimization::publishMapOptimizationTFs(const rclcpp::Time &stamp)
{
    tf2::TimePoint time_point = tf2_ros::fromRclcpp(stamp);

    if (T_EM_initialized) {
        tf2::Quaternion q_ecef_map_enu;
        q_ecef_map_enu.setRPY(T_EM_estimate.rotation().roll(), T_EM_estimate.rotation().pitch(), T_EM_estimate.rotation().yaw());

        tf2::Transform t_ecef_to_map_enu = tf2::Transform(q_ecef_map_enu, tf2::Vector3(T_EM_estimate.translation().x(), T_EM_estimate.translation().y(), T_EM_estimate.translation().z()));
        tf2::Stamped<tf2::Transform> stamped_ecef_to_map_enu(t_ecef_to_map_enu, time_point, ECEFframe);
        geometry_msgs::msg::TransformStamped trans_ecef_to_map_enu;
        tf2::convert(stamped_ecef_to_map_enu, trans_ecef_to_map_enu);
        trans_ecef_to_map_enu.child_frame_id = mapFrameEnu;
        br->sendTransform(trans_ecef_to_map_enu);
    }

    // 0. Broadcast TF mapFrameEnu -> mapFrameNed (ENU->NED, pure rotation, no children)
    // R_ned_from_enu: North=ENU_Y, East=ENU_X, Down=-ENU_Z  =>  setRPY(pi, 0, pi/2)
    {
        tf2::Quaternion q_enu_to_ned;
        q_enu_to_ned.setRPY(M_PI, 0.0, M_PI / 2.0);
        tf2::Transform t_enu_to_ned(q_enu_to_ned, tf2::Vector3(0.0, 0.0, 0.0));
        tf2::Stamped<tf2::Transform> stamped_enu_to_ned(t_enu_to_ned, time_point, mapFrameEnu);
        geometry_msgs::msg::TransformStamped trans_enu_to_ned;
        tf2::convert(stamped_enu_to_ned, trans_enu_to_ned);
        trans_enu_to_ned.child_frame_id = mapFrameNed;
        br->sendTransform(trans_enu_to_ned);
    }

    // 1. Broadcast TF mapFrameEnu -> mapFrameLocal only after anchor is ready.
    if (gpsAnchorReady()) {
        tf2::Quaternion q_map_to_map_local;
        tf2::Transform t_map_to_map_local;
        q_map_to_map_local.setRPY(T_EL_estimate.rotation().roll(), T_EL_estimate.rotation().pitch(), T_EL_estimate.rotation().yaw());
        t_map_to_map_local = tf2::Transform(q_map_to_map_local, tf2::Vector3(T_EL_estimate.translation().x(), T_EL_estimate.translation().y(), T_EL_estimate.translation().z()));

        tf2::Stamped<tf2::Transform> stamped_map_to_map_local(t_map_to_map_local, time_point, mapFrameEnu);
        geometry_msgs::msg::TransformStamped trans_map_to_map_local;
        tf2::convert(stamped_map_to_map_local, trans_map_to_map_local);
        trans_map_to_map_local.child_frame_id = mapFrameLocal;
        br->sendTransform(trans_map_to_map_local);

        // 2. Broadcast PoseWithCovarianceStamped Topic
        if (pubGlobalOffset->get_subscription_count() != 0) {
            geometry_msgs::msg::PoseWithCovarianceStamped offset_msg;
            offset_msg.header.stamp = timeLaserInfoStamp;
            offset_msg.header.frame_id = mapFrameEnu;
            
            offset_msg.pose.pose.position.x = T_EL_estimate.translation().x();
            offset_msg.pose.pose.position.y = T_EL_estimate.translation().y();
            offset_msg.pose.pose.position.z = T_EL_estimate.translation().z();
            
            offset_msg.pose.pose.orientation.x = q_map_to_map_local.x();
            offset_msg.pose.pose.orientation.y = q_map_to_map_local.y();
            offset_msg.pose.pose.orientation.z = q_map_to_map_local.z();
            offset_msg.pose.pose.orientation.w = q_map_to_map_local.w();

            if (isamCurrentEstimate.exists(T_EL_KEY)) {
                gtsam::Matrix marginalCov = isam->marginalCovariance(T_EL_KEY);
                // Map GTSAM [Rot, Trans] to ROS [Trans, Rot]
                for (int i = 0; i < 3; i++) {
                    for (int j = 0; j < 3; j++) {
                        offset_msg.pose.covariance[(i)*6 + (j)] = marginalCov(i+3, j+3);
                        offset_msg.pose.covariance[(i+3)*6 + (j+3)] = marginalCov(i, j);
                    }
                }
            }
            pubGlobalOffset->publish(offset_msg);
        }
    }

    if (mapLocalToOdomInitialized) {
        float x, y, z, roll, pitch, yaw;
        pcl::getTranslationAndEulerAngles(mapLocalToOdomAffine, x, y, z, roll, pitch, yaw);
        tf2::Quaternion q_map_local_to_odom;
        q_map_local_to_odom.setRPY(roll, pitch, yaw);
        tf2::Transform t_map_local_to_odom = tf2::Transform(q_map_local_to_odom, tf2::Vector3(x, y, z));
        tf2::Stamped<tf2::Transform> stamped_map_local_to_odom(t_map_local_to_odom, time_point, mapFrameLocal);
        geometry_msgs::msg::TransformStamped trans_map_local_to_odom;
        tf2::convert(stamped_map_local_to_odom, trans_map_local_to_odom);
        trans_map_local_to_odom.child_frame_id = odometryFrame;
        br->sendTransform(trans_map_local_to_odom);
    }

    // ========== TRANSFORM 1: odom -> lidar_link (smooth incremental LiDAR pose) ==========
    tf2::Transform t_odom_to_lidar = tfFromAffine(odomToLidarAffine);
    const auto &odom_to_lidar_origin = t_odom_to_lidar.getOrigin();
    double odom_roll, odom_pitch, odom_yaw;
    tf2::Matrix3x3(t_odom_to_lidar.getRotation()).getRPY(odom_roll, odom_pitch, odom_yaw);

    tf2::Stamped<tf2::Transform> stamped_odom_to_lidar(t_odom_to_lidar, time_point, odometryFrame);
    geometry_msgs::msg::TransformStamped trans_odom_to_lidar;
    tf2::convert(stamped_odom_to_lidar, trans_odom_to_lidar);
    trans_odom_to_lidar.child_frame_id = "lidar_link";
    br->sendTransform(trans_odom_to_lidar);

    if (debugTFs)
    {
        RCLCPP_INFO_STREAM_THROTTLE(
            get_logger(), *get_clock(), 10000,
            "[TF_DEBUG] publish [1/2] " << odometryFrame << "->lidar_link (from smooth incremental LiDAR odometry)"
            << " xyz=(" << odom_to_lidar_origin.x() << ", " << odom_to_lidar_origin.y() << ", " << odom_to_lidar_origin.z() << ")"
            << " rpy=(" << odom_roll << ", " << odom_pitch << ", " << odom_yaw << ")"
        );
    }

    // ========== TRANSFORM 2: odom -> baselinkFrame (smooth incremental base-link pose) ==========
    if (lidarFrame != baselinkFrame)
    {
        // Frames differ: attempt lookup if we don't have it yet
        if (!hasLidar2Baselink)
        {
            const double nowWall = this->now().seconds();
            if (lastTfLookupAttemptWall < 0.0 || (nowWall - lastTfLookupAttemptWall) >= tfLookupRetryPeriodSec)
            {
                lastTfLookupAttemptWall = nowWall;
                if (debugTFs)
                {
                    RCLCPP_INFO_STREAM_THROTTLE(
                        get_logger(), *get_clock(), 10000,
                        "[TF_DEBUG] retry lookupTransform target='" << lidarFrame
                        << "' source='" << baselinkFrame << "'"
                    );
                }
                tryLookupLidarToBaselinkTf("publishMapOptimizationTFs/retry");
            }
        }

        // Publish to baselink if we have the transform
        if (hasLidar2Baselink)
        {
            tf2::Transform t_odom_to_baselink = tfFromAffine(odomToBaseAffine);
            tf2::Stamped<tf2::Transform> stamped_odom_to_baselink(t_odom_to_baselink, time_point, odometryFrame);
            geometry_msgs::msg::TransformStamped trans_odom_to_baselink;
            tf2::convert(stamped_odom_to_baselink, trans_odom_to_baselink);
            trans_odom_to_baselink.child_frame_id = baselinkFrame;
            br->sendTransform(trans_odom_to_baselink);

            if (debugTFs)
            {
                // Extract transform details for logging
                const auto &tr = lidar2Baselink.getOrigin();
                tf2::Quaternion q = lidar2Baselink.getRotation();
                double roll, pitch, yaw;
                tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

                RCLCPP_INFO_STREAM_THROTTLE(
                    get_logger(), *get_clock(), 10000,
                    "[TF_DEBUG] publish [2/2] " << odometryFrame << "->" << baselinkFrame
                    << " (from smooth incremental base-link odometry)"
                    << " lidar2baselink_applied: xyz=(" << tr.x() << ", " << tr.y() << ", " << tr.z() << ")"
                    << " rpy=(" << roll << ", " << pitch << ", " << yaw << ")"
                );
            }
        }
        else if (debugTFs)
        {
            RCLCPP_INFO_STREAM_THROTTLE(
                get_logger(), *get_clock(), 10000,
                "[TF_DEBUG] publish [2/2] SKIPPED " << odometryFrame << "->" << baselinkFrame
                << " (awaiting valid lidar2baselink lookup)"
            );
        }
    }
    else
    {
        // Frames are identical: base-link and lidar share the same smooth odometry state.
        tf2::Transform t_odom_to_baselink = tfFromAffine(odomToBaseAffine);
        tf2::Stamped<tf2::Transform> stamped_odom_to_baselink(t_odom_to_baselink, time_point, odometryFrame);
        geometry_msgs::msg::TransformStamped trans_odom_to_baselink;
        tf2::convert(stamped_odom_to_baselink, trans_odom_to_baselink);
        trans_odom_to_baselink.child_frame_id = baselinkFrame;
        br->sendTransform(trans_odom_to_baselink);

        if (debugTFs)
        {
            RCLCPP_INFO_STREAM_THROTTLE(
                get_logger(), *get_clock(), 10000,
                "[TF_DEBUG] publish [2/2] " << odometryFrame << "->" << baselinkFrame
                << " (from smooth incremental base-link odometry)"
                << " [frames identical: '" << lidarFrame << "' == '" << baselinkFrame << "']"
            );
        }
    }
}


void mapOptimization::publishOdometry()
{
    const Eigen::Affine3f mapLocalToLidarAffine = trans2Affine3f(transformTobeMapped);
    const Eigen::Affine3f lidarToBaselinkAffine = affineFromTf(lidar2Baselink);

    nav_msgs::msg::Odometry laserOdometryROS =
        odometryMsgFromAffine(mapLocalToLidarAffine, timeLaserInfoStamp, mapFrameLocal, "lidar_link");

    Eigen::Affine3f mapLocalToBaselinkAffine = mapLocalToLidarAffine;
    bool canPublishBaselinkPose = false;
    if (lidarFrame == baselinkFrame || hasLidar2Baselink)
    {
        mapLocalToBaselinkAffine = mapLocalToLidarAffine * lidarToBaselinkAffine;
        canPublishBaselinkPose = true;
    }

    if (save_dense_odom_trajectory)
    {
        std::lock_guard<std::mutex> history_lock(densePoseHistoryMutex);
        densePoseHistory.push_back(laserOdometryROS);
    }

    pubLaserOdometryGlobal->publish(laserOdometryROS);
    if (canPublishBaselinkPose)
    {
        pubBaselinkOdometryGlobal->publish(
            odometryMsgFromAffine(mapLocalToBaselinkAffine, timeLaserInfoStamp, mapFrameLocal, baselinkFrame));
    }

    if (lastIncreOdomPubFlag == false)
    {
        lastIncreOdomPubFlag = true;
        laserOdomIncremental = laserOdometryROS;
        poseAcumulatedIncremental = trans2Affine3f(transformTobeMapped);
        lastIncrementalDeltaPoseLocal = Eigen::Affine3f::Identity();
        hasLastIncrementalDeltaPoseLocal = false;
    }
    else
    {
        lastIncrementalDeltaPoseLocal = incrementalOdometryAffineFront.inverse() * incrementalOdometryAffineBack;
        hasLastIncrementalDeltaPoseLocal = true;
        poseAcumulatedIncremental = poseAcumulatedIncremental * lastIncrementalDeltaPoseLocal;
    }

    odomToLidarAffine = poseAcumulatedIncremental;
    laserOdomIncremental =
        odometryMsgFromAffine(odomToLidarAffine, timeLaserInfoStamp, odometryFrame, "lidar_link");
    if (isDegenerate)
        laserOdomIncremental.pose.covariance[0] = 1;
    else
        laserOdomIncremental.pose.covariance[0] = 0;

    pubLaserOdometryIncremental->publish(laserOdomIncremental);

    if (canPublishBaselinkPose)
    {
        odomToBaseAffine = odomToLidarAffine * lidarToBaselinkAffine;
        pubBaselinkOdometryIncremental->publish(
            odometryMsgFromAffine(odomToBaseAffine, timeLaserInfoStamp, odometryFrame, baselinkFrame));
    }
    else
    {
        odomToBaseAffine = odomToLidarAffine;
    }

    mapLocalToOdomAffine = mapLocalToLidarAffine * odomToLidarAffine.inverse();
    mapLocalToOdomInitialized = true;
}

void mapOptimization::publishFrames()
{
    if (cloudKeyPoses3D->points.empty())
        return;

    PointTypePose thisPose6D = trans2PointTypePose(transformTobeMapped);

    // publish key poses
    publishCloud(pubKeyPoses, cloudKeyPoses3D, timeLaserInfoStamp, mapFrameLocal);
    // Publish surrounding key frames
    publishCloud(pubRecentKeyFrames, laserCloudSurfFromMapDS, timeLaserInfoStamp, mapFrameLocal);

    if (pubSurfDebugColored->get_subscription_count() != 0)
    {
        pcl::PointCloud<PointType>::Ptr transformedInput = transformPointCloud(laserCloudSurfLastDS, &thisPose6D);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr coloredCloud(new pcl::PointCloud<pcl::PointXYZRGB>());

        const int codeBound = laserCloudSurfLastDSNum < static_cast<int>(scanAlignerPrimary->laserCloudSurfDebugCode.size())
                                  ? laserCloudSurfLastDSNum
                                  : static_cast<int>(scanAlignerPrimary->laserCloudSurfDebugCode.size());
        const int pointBound = codeBound < static_cast<int>(transformedInput->size())
                                   ? codeBound
                                   : static_cast<int>(transformedInput->size());

        coloredCloud->reserve(pointBound);
        for (int i = 0; i < pointBound; ++i)
        {
            pcl::PointXYZRGB point;
            point.x = transformedInput->points[i].x;
            point.y = transformedInput->points[i].y;
            point.z = transformedInput->points[i].z;

            switch (scanAlignerPrimary->laserCloudSurfDebugCode[i])
            {
                case ScanAligner::SURF_DEBUG_ACCEPTED:
                    point.r = 0; point.g = 255; point.b = 0;      // green
                    break;
                case ScanAligner::SURF_DEBUG_REJECTED_NEIGHBOR_COUNT:
                    point.r = 255; point.g = 0; point.b = 0;      // red
                    break;
                case ScanAligner::SURF_DEBUG_REJECTED_KNN_DISTANCE:
                    point.r = 255; point.g = 165; point.b = 0;    // orange
                    break;
                case ScanAligner::SURF_DEBUG_REJECTED_PLANE_INVALID:
                    point.r = 255; point.g = 255; point.b = 0;    // yellow
                    break;
                case ScanAligner::SURF_DEBUG_REJECTED_LOW_WEIGHT:
                    point.r = 255; point.g = 0; point.b = 255;    // magenta
                    break;
                default:
                    point.r = 128; point.g = 128; point.b = 128;  // gray (not optimized/unknown)
                    break;
            }

            coloredCloud->push_back(point);
        }

        publishCloud(pubSurfDebugColored, coloredCloud, timeLaserInfoStamp, mapFrameLocal);

        if (pubSurfDebugLegend->get_subscription_count() != 0)
        {
            std_msgs::msg::String legendMsg;
            legendMsg.data =
                "surf_debug_colored legend: "
                "green=accepted, "
                "red=rejected_neighbor_count, "
                "orange=rejected_knn_distance, "
                "yellow=rejected_plane_invalid, "
                "magenta=rejected_low_weight, "
                "gray=not_optimized";
            RCLCPP_INFO_STREAM_THROTTLE(get_logger(), *get_clock(), 5000, legendMsg.data);
            pubSurfDebugLegend->publish(legendMsg);
        }
        
    }

    // publish registered key frame
    if (pubRecentKeyFrame->get_subscription_count() != 0)
    {
        pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
        *cloudOut += *transformPointCloud(laserCloudSurfLastDS,    &thisPose6D);
        publishCloud(pubRecentKeyFrame, cloudOut, timeLaserInfoStamp, mapFrameLocal);
    }
    // publish matched surf features used in the final scan-to-map iteration
    if (pubMatchedSurfFeatures->get_subscription_count() != 0)
    {
        pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
        *cloudOut += *transformPointCloud(scanAlignerPrimary->getLaserCloudOri(), &thisPose6D);
        publishCloud(pubMatchedSurfFeatures, cloudOut, timeLaserInfoStamp, mapFrameLocal);
    }
    // publish registered high-res raw cloud
    if (pubCloudRegisteredRaw->get_subscription_count() != 0)
    {
        pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
        pcl::fromROSMsg(cloudInfo.cloud_deskewed, *cloudOut);
        PointTypePose thisPose6D = trans2PointTypePose(transformTobeMapped);
        *cloudOut = *transformPointCloud(cloudOut,  &thisPose6D);
        publishCloud(pubCloudRegisteredRaw, cloudOut, timeLaserInfoStamp, mapFrameLocal);
    }
    // publish path
    if (pubPath->get_subscription_count() != 0)
    {
        globalPath.header.stamp = timeLaserInfoStamp;
        globalPath.header.frame_id = mapFrameLocal;
        pubPath->publish(globalPath);
    }
}

void mapOptimization::publishDegeneracyMarkers(const std::vector<TwistVector> &twists, const rclcpp::Time &stamp)
{
    if (pubDegeneracyMarkers->get_subscription_count() == 0)
        return;

    visualization_msgs::msg::MarkerArray markerArray;

    // 1. Create a "Delete All" marker to clear previous frames' degeneracies
    visualization_msgs::msg::Marker deleteAllMarker;
    deleteAllMarker.action = visualization_msgs::msg::Marker::DELETEALL;
    markerArray.markers.push_back(deleteAllMarker);

    if (twists.empty()) {
        pubDegeneracyMarkers->publish(markerArray);
        return;
    }

    // 2. Get the current global pose to anchor and rotate the markers
    Eigen::Affine3f currentPose = trans2Affine3f(transformTobeMapped);
    Eigen::Vector3f robotPosition = currentPose.translation();
    Eigen::Matrix3f robotRotation = currentPose.rotation();

    int marker_id = 0;
    const float visualization_scale = 5.0f; // Scale up the arrows so they are easy to see in RViz

    for (size_t i = 0; i < twists.size(); ++i)
    {
        TwistVector twist = twists[i];
        Eigen::Vector3f localTranslation = twist.segment<3>(0);
        Eigen::Vector3f localRotationAxis = twist.segment<3>(3);

        // --- Translation Marker (Red) ---
        if (localTranslation.norm() > 1e-4f)
        {
            Eigen::Vector3f globalTranslationDir = robotRotation * localTranslation.normalized();
            
            visualization_msgs::msg::Marker transMarker;
            transMarker.header.frame_id = mapFrameLocal; // or odometryFrame
            transMarker.header.stamp = stamp;
            transMarker.ns = "degeneracy_translation";
            transMarker.id = marker_id++;
            transMarker.type = visualization_msgs::msg::Marker::ARROW;
            transMarker.action = visualization_msgs::msg::Marker::ADD;
            
            transMarker.scale.x = 0.2; // Shaft diameter
            transMarker.scale.y = 0.4; // Head diameter
            transMarker.scale.z = 0.0; // Head length (0 = default)
            
            transMarker.color.r = 1.0f;
            transMarker.color.g = 0.0f;
            transMarker.color.b = 0.0f;
            transMarker.color.a = 0.8f;

            geometry_msgs::msg::Point start_point, end_point;
            start_point.x = robotPosition.x();
            start_point.y = robotPosition.y();
            start_point.z = robotPosition.z();

            end_point.x = robotPosition.x() + (globalTranslationDir.x() * visualization_scale);
            end_point.y = robotPosition.y() + (globalTranslationDir.y() * visualization_scale);
            end_point.z = robotPosition.z() + (globalTranslationDir.z() * visualization_scale);

            transMarker.points.push_back(start_point);
            transMarker.points.push_back(end_point);
            
            markerArray.markers.push_back(transMarker);
        }

        // --- Rotation Axis Marker (Yellow) ---
        if (localRotationAxis.norm() > 1e-4f)
        {
            Eigen::Vector3f globalRotationDir = robotRotation * localRotationAxis.normalized();
            
            visualization_msgs::msg::Marker rotMarker;
            rotMarker.header.frame_id = mapFrameLocal; // or odometryFrame
            rotMarker.header.stamp = stamp;
            rotMarker.ns = "degeneracy_rotation";
            rotMarker.id = marker_id++;
            rotMarker.type = visualization_msgs::msg::Marker::ARROW;
            rotMarker.action = visualization_msgs::msg::Marker::ADD;
            
            rotMarker.scale.x = 0.2; 
            rotMarker.scale.y = 0.4; 
            rotMarker.scale.z = 0.0; 
            
            rotMarker.color.r = 1.0f;
            rotMarker.color.g = 1.0f;
            rotMarker.color.b = 0.0f;
            rotMarker.color.a = 0.8f;

            geometry_msgs::msg::Point start_point, end_point;
            start_point.x = robotPosition.x();
            start_point.y = robotPosition.y();
            start_point.z = robotPosition.z();

            end_point.x = robotPosition.x() + (globalRotationDir.x() * visualization_scale);
            end_point.y = robotPosition.y() + (globalRotationDir.y() * visualization_scale);
            end_point.z = robotPosition.z() + (globalRotationDir.z() * visualization_scale);

            rotMarker.points.push_back(start_point);
            rotMarker.points.push_back(end_point);
            
            markerArray.markers.push_back(rotMarker);
        }
    }

    pubDegeneracyMarkers->publish(markerArray);
}


