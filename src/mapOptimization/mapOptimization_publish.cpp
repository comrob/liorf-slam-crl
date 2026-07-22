#include "mapOptimization/mapOptimization.hpp"
#include "degeneracyDetection/TwistManipulation.hpp"
#include "scanAlignment/ScanAligner.hpp"
#include <cmath>

struct EIGEN_ALIGN16 PointTypeWithDebugCode
{
    PCL_ADD_POINT4D;
    float intensity;
    std::uint8_t debug_code;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(
    PointTypeWithDebugCode,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (std::uint8_t, debug_code, debug_code))

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

Eigen::Affine3f mapOptimization::pclPointToAffine3f(PointTypePose thisPoint) const
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

void mapOptimization::publishPredictionDebugClouds(const pcl::PointCloud<PointType>::Ptr &cloud)
{
    const bool publishPrevious = pubCloudPreviousPose && pubCloudPreviousPose->get_subscription_count() != 0;
    const bool publishPredicted = pubCloudPredictedPose && pubCloudPredictedPose->get_subscription_count() != 0;

    if (!publishPrevious && !publishPredicted)
        return;

    if (!cloud || cloud->empty())
        return;

    auto affineToPose = [](const Eigen::Affine3f &affine) {
        PointTypePose pose;
        pcl::getTranslationAndEulerAngles(
            affine,
            pose.x,
            pose.y,
            pose.z,
            pose.roll,
            pose.pitch,
            pose.yaw);
        return pose;
    };

    if (publishPrevious)
    {
        PointTypePose beforePose = affineToPose(poseBeforePredictionLocal);
        pcl::PointCloud<PointType>::Ptr cloudOut = transformPointCloud(cloud, &beforePose);
        publishCloud(pubCloudPreviousPose, cloudOut, timeLaserInfoStamp, mapFrameLocal);
    }

    if (publishPredicted)
    {
        PointTypePose predictedPose = affineToPose(poseAfterPredictionLocal);
        pcl::PointCloud<PointType>::Ptr cloudOut = transformPointCloud(cloud, &predictedPose);
        publishCloud(pubCloudPredictedPose, cloudOut, timeLaserInfoStamp, mapFrameLocal);
    }
}

void mapOptimization::publishPerturbationDebugProducts(
    const std::vector<pcl::PointCloud<PointType>::Ptr>& perturbedScans,
    const std::vector<pcl::PointCloud<PointType>::Ptr>& alignedScans,
    const std::vector<Eigen::Matrix4f>& perturbedPoses,
    const std::vector<Eigen::Matrix4f>& alignedPoses,
    const std::vector<std::vector<Eigen::Matrix4f>>& optimizationPaths,
    const rclcpp::Time& stamp,
    const Eigen::Affine3f& poseBeforeReanchor,
    const Eigen::Affine3f& poseAfterReanchor)
{
    if (perturbedScans.empty() && alignedScans.empty() && perturbedPoses.empty() && alignedPoses.empty())
        return;

    const Eigen::Matrix4f reanchorTransform =
        poseAfterReanchor.matrix() * poseBeforeReanchor.matrix().inverse();

    const auto reanchorPose = [&reanchorTransform](const Eigen::Matrix4f &pose) {
        return reanchorTransform * pose;
    };

    const std::array<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr, 3> perturbedCloudPubs = {
        pubDegeneracyPerturbedScan0,
        pubDegeneracyPerturbedScan1,
        pubDegeneracyPerturbedScan2
    };

    const std::array<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr, 3> alignedCloudPubs = {
        pubDegeneracyAlignedScan0,
        pubDegeneracyAlignedScan1,
        pubDegeneracyAlignedScan2
    };

    for (size_t i = 0; i < perturbedCloudPubs.size(); ++i)
    {
        if (!perturbedCloudPubs[i] || perturbedCloudPubs[i]->get_subscription_count() == 0)
            continue;

        if (i >= perturbedScans.size() || !perturbedScans[i] || perturbedScans[i]->empty())
            continue;

        publishCloud(perturbedCloudPubs[i], perturbedScans[i], stamp, mapFrameLocal);
    }

    for (size_t i = 0; i < alignedCloudPubs.size(); ++i)
    {
        if (!alignedCloudPubs[i] || alignedCloudPubs[i]->get_subscription_count() == 0)
            continue;

        if (i >= alignedScans.size() || !alignedScans[i] || alignedScans[i]->empty())
            continue;

        publishCloud(alignedCloudPubs[i], alignedScans[i], stamp, mapFrameLocal);
    }

    const std::array<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr, 3> perturbedPosePubs = {
        pubDegeneracyPerturbedPose0,
        pubDegeneracyPerturbedPose1,
        pubDegeneracyPerturbedPose2
    };

    const std::array<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr, 3> alignedPosePubs = {
        pubDegeneracyAlignedPose0,
        pubDegeneracyAlignedPose1,
        pubDegeneracyAlignedPose2
    };

    const auto makePoseStamped = [this, &stamp](const Eigen::Matrix4f &poseMatrix) {
        geometry_msgs::msg::PoseStamped msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = mapFrameLocal;

        Eigen::Affine3f affine(poseMatrix);
        msg.pose.position.x = affine.translation().x();
        msg.pose.position.y = affine.translation().y();
        msg.pose.position.z = affine.translation().z();

        Eigen::Quaternionf q(affine.rotation());
        msg.pose.orientation.x = q.x();
        msg.pose.orientation.y = q.y();
        msg.pose.orientation.z = q.z();
        msg.pose.orientation.w = q.w();
        return msg;
    };

    for (size_t i = 0; i < perturbedPosePubs.size(); ++i)
    {
        if (!perturbedPosePubs[i] || perturbedPosePubs[i]->get_subscription_count() == 0)
            continue;
        if (i >= perturbedPoses.size())
            continue;
        perturbedPosePubs[i]->publish(makePoseStamped(reanchorPose(perturbedPoses[i])));
    }

    for (size_t i = 0; i < alignedPosePubs.size(); ++i)
    {
        if (!alignedPosePubs[i] || alignedPosePubs[i]->get_subscription_count() == 0)
            continue;
        if (i >= alignedPoses.size())
            continue;
        alignedPosePubs[i]->publish(makePoseStamped(reanchorPose(alignedPoses[i])));
    }

    visualization_msgs::msg::MarkerArray markerArray;
    visualization_msgs::msg::Marker clearAll;
    clearAll.action = visualization_msgs::msg::Marker::DELETEALL;
    markerArray.markers.push_back(clearAll);

    const std::array<std::array<float, 3>, 3> colors = {{
        {{1.0f, 0.2f, 0.2f}},
        {{0.2f, 1.0f, 0.2f}},
        {{0.2f, 0.5f, 1.0f}}
    }};

    const size_t markerCount = std::min<size_t>(3, std::min(perturbedPoses.size(), alignedPoses.size()));
    if (pubDegeneracyDisplacements && pubDegeneracyDisplacements->get_subscription_count() > 0)
    {
        for (size_t i = 0; i < markerCount; ++i)
        {
            const Eigen::Matrix4f perturbedPose = reanchorPose(perturbedPoses[i]);
            const Eigen::Matrix4f alignedPose = reanchorPose(alignedPoses[i]);

            visualization_msgs::msg::Marker marker;
            marker.header.stamp = stamp;
            marker.header.frame_id = mapFrameLocal;
            marker.ns = "degeneracy_displacements";
            marker.id = static_cast<int>(i);
            marker.type = visualization_msgs::msg::Marker::ARROW;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.scale.x = 0.08;
            marker.scale.y = 0.16;
            marker.scale.z = 0.16;
            marker.color.r = colors[i][0];
            marker.color.g = colors[i][1];
            marker.color.b = colors[i][2];
            marker.color.a = 0.95f;

            geometry_msgs::msg::Point pFrom;
            pFrom.x = perturbedPose(0, 3);
            pFrom.y = perturbedPose(1, 3);
            pFrom.z = perturbedPose(2, 3);

            geometry_msgs::msg::Point pTo;
            pTo.x = alignedPose(0, 3);
            pTo.y = alignedPose(1, 3);
            pTo.z = alignedPose(2, 3);

            marker.points.push_back(pFrom);
            marker.points.push_back(pTo);
            markerArray.markers.push_back(marker);
        }
        pubDegeneracyDisplacements->publish(markerArray);
    }

    if (!pubDegeneracyOptimizationPaths || pubDegeneracyOptimizationPaths->get_subscription_count() == 0)
        return;

    visualization_msgs::msg::MarkerArray pathMarkers;
    visualization_msgs::msg::Marker clearPaths;
    clearPaths.action = visualization_msgs::msg::Marker::DELETEALL;
    pathMarkers.markers.push_back(clearPaths);

    int markerId = 0;
    const size_t pathCount = std::min<size_t>(3, optimizationPaths.size());
    for (size_t i = 0; i < pathCount; ++i)
    {
        const auto &path = optimizationPaths[i];
        if (path.size() < 2)
            continue;

        visualization_msgs::msg::Marker line;
        line.header.stamp = stamp;
        line.header.frame_id = mapFrameLocal;
        line.ns = "degeneracy_optimization_path";
        line.id = markerId++;
        line.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line.action = visualization_msgs::msg::Marker::ADD;
        line.scale.x = 0.04;
        line.color.r = colors[i][0];
        line.color.g = colors[i][1];
        line.color.b = colors[i][2];
        line.color.a = 0.9f;

        for (const auto &pose : path)
        {
            const Eigen::Matrix4f reanchoredPose = reanchorPose(pose);
            geometry_msgs::msg::Point p;
            p.x = reanchoredPose(0, 3);
            p.y = reanchoredPose(1, 3);
            p.z = reanchoredPose(2, 3);
            line.points.push_back(p);
        }
        pathMarkers.markers.push_back(line);

        for (size_t step = 0; step + 1 < path.size(); ++step)
        {
            const Eigen::Matrix4f reanchoredFrom = reanchorPose(path[step]);
            const Eigen::Matrix4f reanchoredTo = reanchorPose(path[step + 1]);

            visualization_msgs::msg::Marker stepArrow;
            stepArrow.header.stamp = stamp;
            stepArrow.header.frame_id = mapFrameLocal;
            stepArrow.ns = "degeneracy_optimization_steps";
            stepArrow.id = markerId++;
            stepArrow.type = visualization_msgs::msg::Marker::ARROW;
            stepArrow.action = visualization_msgs::msg::Marker::ADD;
            stepArrow.scale.x = 0.02;
            stepArrow.scale.y = 0.05;
            stepArrow.scale.z = 0.05;
            stepArrow.color.r = colors[i][0];
            stepArrow.color.g = colors[i][1];
            stepArrow.color.b = colors[i][2];
            stepArrow.color.a = 0.8f;

            geometry_msgs::msg::Point pFrom;
            pFrom.x = reanchoredFrom(0, 3);
            pFrom.y = reanchoredFrom(1, 3);
            pFrom.z = reanchoredFrom(2, 3);

            geometry_msgs::msg::Point pTo;
            pTo.x = reanchoredTo(0, 3);
            pTo.y = reanchoredTo(1, 3);
            pTo.z = reanchoredTo(2, 3);

            stepArrow.points.push_back(pFrom);
            stepArrow.points.push_back(pTo);
            pathMarkers.markers.push_back(stepArrow);
        }
    }

    pubDegeneracyOptimizationPaths->publish(pathMarkers);
}

void mapOptimization::publishMapOptimizationTFs(const rclcpp::Time &stamp)
{
    tf2::TimePoint time_point = tf2_ros::fromRclcpp(stamp);
    const bool canPublishBaselinkFrame = runtimeTfCoordinator->allowBaselinkFramePublishing("publishMapOptimizationTFs");

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
    if (canPublishBaselinkFrame && lidarFrame != baselinkFrame)
    {
        // Frames differ: attempt lookup if we don't have it yet
        if (!runtimeTfCoordinator->hasLidarToBaselinkTransform())
        {
            if (debugTFs)
            {
                RCLCPP_INFO_STREAM_THROTTLE(
                    get_logger(), *get_clock(), 10000,
                    "[TF_DEBUG] retry lookupTransform target='" << lidarFrame
                    << "' source='" << baselinkFrame << "'"
                );
            }
            runtimeTfCoordinator->tryLookupLidarToBaselinkTf("publishMapOptimizationTFs/retry");
        }

        // Publish to baselink if we have the transform
        if (runtimeTfCoordinator->hasLidarToBaselinkTransform())
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
                const tf2::Transform lidar2Baselink = runtimeTfCoordinator->lidarToBaselinkTransform();
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
    else if (canPublishBaselinkFrame)
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
    else if (debugTFs)
    {
        RCLCPP_INFO_STREAM_THROTTLE(
            get_logger(), *get_clock(), 10000,
            "[TF_DEBUG] publish [2/2] SKIPPED " << odometryFrame << "->" << baselinkFrame
            << " (baselink frame publishing suppressed due to existing external TF parent)"
        );
    }
}


void mapOptimization::publishOdometry()
{
    const Eigen::Affine3f mapLocalToLidarAffine = trans2Affine3f(transformTobeMapped);
    const Eigen::Affine3f lidarToBaselinkAffine = affineFromTf(runtimeTfCoordinator->lidarToBaselinkTransform());
    const bool canPublishBaselinkFrame = runtimeTfCoordinator->allowBaselinkFramePublishing("publishOdometry");

    nav_msgs::msg::Odometry laserOdometryROS =
        odometryMsgFromAffine(mapLocalToLidarAffine, timeLaserInfoStamp, mapFrameLocal, "lidar_link");

    Eigen::Affine3f mapLocalToBaselinkAffine = mapLocalToLidarAffine;
    bool canPublishBaselinkPose = false;
    if (canPublishBaselinkFrame && (lidarFrame == baselinkFrame || runtimeTfCoordinator->hasLidarToBaselinkTransform()))
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

    if (diagnostics)
    {
        const auto &stamp = laserOdomIncremental.header.stamp;
        const auto &p = laserOdomIncremental.pose.pose.position;
        const auto &q = laserOdomIncremental.pose.pose.orientation;

        TumPoseSample odomTumSample;
        odomTumSample.stamp_sec = static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
        odomTumSample.tx = p.x;
        odomTumSample.ty = p.y;
        odomTumSample.tz = p.z;
        odomTumSample.qx = q.x;
        odomTumSample.qy = q.y;
        odomTumSample.qz = q.z;
        odomTumSample.qw = q.w;
        diagnostics->recordOdomTrajectoryTum(odomTumSample);
    }

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

void mapOptimization::publishKeyframeDeskewedDownsampled(const pcl::PointCloud<PointType>::Ptr &cloud)
{
    if (!pubKeyframeDeskewedDownsampled || pubKeyframeDeskewedDownsampled->get_subscription_count() == 0)
        return;

    if (!cloud || cloud->empty())
        return;

    publishCloud(pubKeyframeDeskewedDownsampled, cloud, timeLaserInfoStamp, lidarFrame);
}

void mapOptimization::publishKeyframeDeskewedDownsampledDebug(const pcl::PointCloud<PointType>::Ptr &cloud)
{
    if (!pubKeyframeDeskewedDownsampledDebug || pubKeyframeDeskewedDownsampledDebug->get_subscription_count() == 0)
        return;

    if (!cloud || cloud->empty())
        return;

    const auto &codes = mappingBackend->getDebugCodes();
    pcl::PointCloud<PointTypeWithDebugCode>::Ptr cloudWithDebug(new pcl::PointCloud<PointTypeWithDebugCode>());
    cloudWithDebug->reserve(cloud->size());

    for (size_t i = 0; i < cloud->size(); ++i)
    {
        PointTypeWithDebugCode point;
        point.x = cloud->points[i].x;
        point.y = cloud->points[i].y;
        point.z = cloud->points[i].z;
        point.intensity = cloud->points[i].intensity;
        point.debug_code = (i < codes.size() && codes[i] >= 0) ? static_cast<std::uint8_t>(codes[i]) : 0;
        cloudWithDebug->push_back(point);
    }

    publishCloud(pubKeyframeDeskewedDownsampledDebug, cloudWithDebug, timeLaserInfoStamp, lidarFrame);
}

void mapOptimization::publishKdTreePlaneDebug()
{
    const bool wantsPoints = pubKdTreePlanePoints && pubKdTreePlanePoints->get_subscription_count() != 0;
    const bool wantsNormals = pubKdTreePlaneNormals && pubKdTreePlaneNormals->get_subscription_count() != 0;
    const bool wantsResiduals = pubKdTreePlaneResiduals && pubKdTreePlaneResiduals->get_subscription_count() != 0;

    if (!wantsPoints && !wantsNormals && !wantsResiduals)
        return;

    if (backend_type != "kdtree_lm")
        return;

    const auto &samples = mappingBackend->getLastPlaneNormalSamples();

    if (wantsPoints)
    {
        pcl::PointCloud<PointType>::Ptr points(new pcl::PointCloud<PointType>());
        points->reserve(samples.size());
        for (const auto &sample : samples)
            points->push_back(sample.point_map);

        publishCloud(pubKdTreePlanePoints, points, timeLaserInfoStamp, mapFrameLocal);
    }

    if (wantsNormals)
    {
        visualization_msgs::msg::MarkerArray markerArray;
        visualization_msgs::msg::Marker clearAll;
        clearAll.action = visualization_msgs::msg::Marker::DELETEALL;
        markerArray.markers.push_back(clearAll);

        constexpr float kArrowLength = 0.6f;
        constexpr float kArrowShaft = 0.03f;
        constexpr float kArrowHead = 0.08f;
        int markerId = 0;

        for (const auto &sample : samples)
        {
            if (!std::isfinite(sample.normal_map.x()) || !std::isfinite(sample.normal_map.y()) || !std::isfinite(sample.normal_map.z()))
                continue;

            if (!std::isfinite(sample.residual_vector_map.x()) || !std::isfinite(sample.residual_vector_map.y()) || !std::isfinite(sample.residual_vector_map.z()))
                continue;

            const float nNorm = sample.normal_map.norm();
            if (nNorm <= 1e-6f)
                continue;

            const Eigen::Vector3f nUnit = sample.normal_map / nNorm;
            const Eigen::Vector3f projectedPoint(
                sample.point_map.x + sample.residual_vector_map.x(),
                sample.point_map.y + sample.residual_vector_map.y(),
                sample.point_map.z + sample.residual_vector_map.z());

            visualization_msgs::msg::Marker marker;
            marker.header.stamp = timeLaserInfoStamp;
            marker.header.frame_id = mapFrameLocal;
            marker.ns = "kdtree_plane_normals";
            marker.id = markerId++;
            marker.type = visualization_msgs::msg::Marker::ARROW;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.scale.x = kArrowShaft;
            marker.scale.y = kArrowHead;
            marker.scale.z = kArrowHead;
            marker.color.r = 0.2f;
            marker.color.g = 0.9f;
            marker.color.b = 1.0f;
            marker.color.a = 0.9f;

            geometry_msgs::msg::Point p0;
            p0.x = static_cast<double>(projectedPoint.x());
            p0.y = static_cast<double>(projectedPoint.y());
            p0.z = static_cast<double>(projectedPoint.z());

            geometry_msgs::msg::Point p1;
            p1.x = p0.x + static_cast<double>(nUnit.x() * kArrowLength);
            p1.y = p0.y + static_cast<double>(nUnit.y() * kArrowLength);
            p1.z = p0.z + static_cast<double>(nUnit.z() * kArrowLength);

            marker.points.push_back(p0);
            marker.points.push_back(p1);
            markerArray.markers.push_back(marker);
        }

        pubKdTreePlaneNormals->publish(markerArray);
    }

    if (wantsResiduals)
    {
        visualization_msgs::msg::MarkerArray markerArray;
        visualization_msgs::msg::Marker clearAll;
        clearAll.action = visualization_msgs::msg::Marker::DELETEALL;
        markerArray.markers.push_back(clearAll);

        constexpr float kArrowShaft = 0.04f;
        constexpr float kArrowHead = 0.08f;
        int markerId = 0;

        for (const auto &sample : samples)
        {
            if (!std::isfinite(sample.residual_vector_map.x()) || !std::isfinite(sample.residual_vector_map.y()) || !std::isfinite(sample.residual_vector_map.z()))
                continue;

            const float residualNorm = sample.residual_vector_map.norm();
            if (residualNorm <= 1e-6f)
                continue;

            visualization_msgs::msg::Marker marker;
            marker.header.stamp = timeLaserInfoStamp;
            marker.header.frame_id = mapFrameLocal;
            marker.ns = "kdtree_plane_residuals";
            marker.id = markerId++;
            marker.type = visualization_msgs::msg::Marker::ARROW;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.scale.x = kArrowShaft;
            marker.scale.y = kArrowHead;
            marker.scale.z = kArrowHead;
            marker.color.r = 1.0f;
            marker.color.g = 0.25f;
            marker.color.b = 0.1f;
            marker.color.a = 0.95f;

            geometry_msgs::msg::Point p0;
            p0.x = sample.point_map.x;
            p0.y = sample.point_map.y;
            p0.z = sample.point_map.z;

            geometry_msgs::msg::Point p1;
            p1.x = sample.point_map.x + static_cast<double>(sample.residual_vector_map.x());
            p1.y = sample.point_map.y + static_cast<double>(sample.residual_vector_map.y());
            p1.z = sample.point_map.z + static_cast<double>(sample.residual_vector_map.z());

            marker.points.push_back(p0);
            marker.points.push_back(p1);
            markerArray.markers.push_back(marker);
        }

        pubKdTreePlaneResiduals->publish(markerArray);
    }
}

void mapOptimization::publishFrames()
{
    if (cloudKeyPoses3D->points.empty())
        return;

    PointTypePose thisPose6D = trans2PointTypePose(transformTobeMapped);

    // publish key poses
    publishCloud(pubKeyPoses, cloudKeyPoses3D, timeLaserInfoStamp, mapFrameLocal);
    // Publish surrounding key frames (local map)
    if (pubLocalMapCloud->get_subscription_count() != 0)
    {
        static double lastPublishTime = -1.0;
        if (timeLaserInfoCur - lastPublishTime >= 1.0) // Throttle to 1 Hz
        {
            pcl::PointCloud<PointType>::Ptr localMapCloud = mappingBackend->getLocalMapCloud();
            publishCloud(pubLocalMapCloud, localMapCloud, timeLaserInfoStamp, mapFrameLocal);
            lastPublishTime = timeLaserInfoCur;
        }
    }
    
    if (pubSurfDebugColored->get_subscription_count() != 0)
    {
        pcl::PointCloud<PointType>::Ptr transformedInput = transformPointCloud(laserCloudSurfLastDS, &thisPose6D);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr coloredCloud(new pcl::PointCloud<pcl::PointXYZRGB>());

        const auto& codes = mappingBackend->getDebugCodes();
        const int codeBound = laserCloudSurfLastDSNum < static_cast<int>(codes.size())
                                  ? laserCloudSurfLastDSNum
                                  : static_cast<int>(codes.size());
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

            switch (codes[i])
            {
                case SURF_DEBUG_ACCEPTED:
                    point.r = 0; point.g = 255; point.b = 0;      // green
                    break;
                case SURF_DEBUG_REJECTED_NEIGHBOR_COUNT:
                    point.r = 255; point.g = 0; point.b = 0;      // red
                    break;
                case SURF_DEBUG_REJECTED_KNN_DISTANCE:
                    point.r = 255; point.g = 165; point.b = 0;    // orange
                    break;
                case SURF_DEBUG_REJECTED_PLANE_INVALID:
                    point.r = 255; point.g = 255; point.b = 0;    // yellow
                    break;
                case SURF_DEBUG_REJECTED_LOW_WEIGHT:
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

    // publish registered cloud (per processed frame)
    if (pubRegisteredCloud->get_subscription_count() != 0)
    {
        pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
        *cloudOut += *transformPointCloud(laserCloudSurfLastDS,    &thisPose6D);
        publishCloud(pubRegisteredCloud, cloudOut, timeLaserInfoStamp, mapFrameLocal);
    }
    // publish matched surf features used in the final scan-to-map iteration
    if (pubMatchedSurfFeatures->get_subscription_count() != 0)
    {
        pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
        *cloudOut += *transformPointCloud(mappingBackend->getLaserCloudOri(), &thisPose6D);
        publishCloud(pubMatchedSurfFeatures, cloudOut, timeLaserInfoStamp, mapFrameLocal);
    }

    publishKdTreePlaneDebug();

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

void mapOptimization::publishTwistMarkers(
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub,
    const std::string& ns,
    const std::vector<TwistVector>& twists,
    const rclcpp::Time& stamp,
    float r_trans, float g_trans, float b_trans,
    float r_rot, float g_rot, float b_rot)
{
    if (pub->get_subscription_count() == 0) return;

    visualization_msgs::msg::MarkerArray markerArray;
    visualization_msgs::msg::Marker deleteAllMarker;
    deleteAllMarker.header.frame_id = odometryFrame;
    deleteAllMarker.header.stamp = stamp;
    deleteAllMarker.ns = "degeneracy";
    deleteAllMarker.id = 0;
    deleteAllMarker.action = visualization_msgs::msg::Marker::DELETEALL;
    markerArray.markers.push_back(deleteAllMarker);

    if (twists.empty()) {
        pub->publish(markerArray);
        return;
    }

    Eigen::Affine3f currentPose = trans2Affine3f(transformTobeMapped);
    Eigen::Vector3f robotPos = currentPose.translation();
    Eigen::Matrix3f robotRot = currentPose.rotation();

    int marker_id = 0;
    const float scale = 5.0f;

    for (const auto& twist : twists) {
        Eigen::Vector3f localTrans = twist.segment<3>(0);
        Eigen::Vector3f localRot = twist.segment<3>(3);

        if (localTrans.norm() > 1e-4f) {
            Eigen::Vector3f globalDir = robotRot * localTrans.normalized();
            visualization_msgs::msg::Marker m;
            m.header.frame_id = mapFrameLocal;
            m.header.stamp = stamp;
            m.ns = ns + "_trans";
            m.id = marker_id++;
            m.type = visualization_msgs::msg::Marker::ARROW;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.scale.x = 0.2; m.scale.y = 0.4;
            m.color.r = r_trans; m.color.g = g_trans; m.color.b = b_trans; m.color.a = 0.8f;
            
            geometry_msgs::msg::Point start, end;
            start.x = robotPos.x(); start.y = robotPos.y(); start.z = robotPos.z();
            end.x = robotPos.x() + globalDir.x() * scale;
            end.y = robotPos.y() + globalDir.y() * scale;
            end.z = robotPos.z() + globalDir.z() * scale;
            m.points.push_back(start); m.points.push_back(end);
            markerArray.markers.push_back(m);
        }

        if (localRot.norm() > 1e-4f) {
            Eigen::Vector3f globalDir = robotRot * localRot.normalized();
            visualization_msgs::msg::Marker m;
            m.header.frame_id = mapFrameLocal;
            m.header.stamp = stamp;
            m.ns = ns + "_rot";
            m.id = marker_id++;
            m.type = visualization_msgs::msg::Marker::ARROW;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.scale.x = 0.2; m.scale.y = 0.4;
            m.color.r = r_rot; m.color.g = g_rot; m.color.b = b_rot; m.color.a = 0.8f;
            
            geometry_msgs::msg::Point start, end;
            start.x = robotPos.x(); start.y = robotPos.y(); start.z = robotPos.z();
            end.x = robotPos.x() + globalDir.x() * scale;
            end.y = robotPos.y() + globalDir.y() * scale;
            end.z = robotPos.z() + globalDir.z() * scale;
            m.points.push_back(start); m.points.push_back(end);
            markerArray.markers.push_back(m);
        }
    }
    pub->publish(markerArray);
}


void mapOptimization::publishDegeneracyPaths(
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub,
    const std::string& ns,
    const std::vector<TwistVector>& twists,
    const rclcpp::Time& stamp)
{
    if (pub->get_subscription_count() == 0) return;

    visualization_msgs::msg::MarkerArray markerArray;
    visualization_msgs::msg::Marker deleteAllMarker;
    deleteAllMarker.action = visualization_msgs::msg::Marker::DELETEALL;
    markerArray.markers.push_back(deleteAllMarker);

    if (twists.empty()) {
        pub->publish(markerArray);
        return;
    }

    Eigen::Matrix4f currentPose = trans2Affine3f(transformTobeMapped).matrix();
    
    int marker_id = 0;
    
    std::vector<std::vector<float>> colors = {
        {1.0f, 0.0f, 0.0f}, // red
        {0.0f, 1.0f, 0.0f}, // green
        {0.0f, 0.0f, 1.0f}, // blue
        {1.0f, 1.0f, 0.0f}, // yellow
        {0.0f, 1.0f, 1.0f}, // cyan
        {1.0f, 0.0f, 1.0f}  // magenta
    };

    for (size_t i = 0; i < twists.size(); ++i) {
        const auto& twist = twists[i];
        const auto& color = colors[i % colors.size()];

        Eigen::Vector3f v = twist.head<3>();
        Eigen::Vector3f omega = twist.tail<3>();
        
        float v_norm = v.norm();
        float omega_norm = omega.norm();
        
        float target_trans = 2.0f; // 2 meters
        float target_rot = 36.0f * M_PI / 180.0f; // 36 degrees
        
        float w_step_trans = (v_norm > 1e-4f) ? target_trans / v_norm : 1e6f;
        float w_step_rot = (omega_norm > 1e-4f) ? target_rot / omega_norm : 1e6f;
        float w_step = std::min(w_step_trans, w_step_rot);
        if (w_step > 1e5f) w_step = 1.0f;

        visualization_msgs::msg::Marker pathMarker;
        pathMarker.header.frame_id = mapFrameLocal;
        pathMarker.header.stamp = stamp;
        pathMarker.ns = ns + "_path_line_" + std::to_string(i);
        pathMarker.id = marker_id++;
        pathMarker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        pathMarker.action = visualization_msgs::msg::Marker::ADD;
        pathMarker.scale.x = 0.1; // Thicker line
        pathMarker.color.r = color[0]; pathMarker.color.g = color[1]; pathMarker.color.b = color[2]; pathMarker.color.a = 0.8f;
        
        const int steps = 5; // 5 steps each side = 10 total segments
        
        for (int step = -steps; step <= steps; ++step) {
            float w = static_cast<float>(step) * w_step;
            
            Eigen::Matrix4f deltaT = expMap(w * twist);
            Eigen::Matrix4f pose_w = currentPose * deltaT;
            
            geometry_msgs::msg::Point pt;
            pt.x = pose_w(0, 3);
            pt.y = pose_w(1, 3);
            pt.z = pose_w(2, 3);
            pathMarker.points.push_back(pt);
            
            visualization_msgs::msg::Marker poseMarker;
            poseMarker.header.frame_id = mapFrameLocal;
            poseMarker.header.stamp = stamp;
            poseMarker.ns = ns + "_pose_" + std::to_string(i);
            poseMarker.id = marker_id++;
            poseMarker.type = visualization_msgs::msg::Marker::ARROW; 
            poseMarker.action = visualization_msgs::msg::Marker::ADD;
            poseMarker.pose.position.x = pose_w(0, 3);
            poseMarker.pose.position.y = pose_w(1, 3);
            poseMarker.pose.position.z = pose_w(2, 3);
            
            Eigen::Quaternionf q(pose_w.block<3, 3>(0, 0));
            poseMarker.pose.orientation.x = q.x();
            poseMarker.pose.orientation.y = q.y();
            poseMarker.pose.orientation.z = q.z();
            poseMarker.pose.orientation.w = q.w();
            
            // For arrows: scale.x is length, scale.y is width, scale.z is height
            poseMarker.scale.x = 1.0; poseMarker.scale.y = 0.2; poseMarker.scale.z = 0.2;
            poseMarker.color.r = color[0]; poseMarker.color.g = color[1]; poseMarker.color.b = color[2]; poseMarker.color.a = 0.8f;
            
            markerArray.markers.push_back(poseMarker);
        }
        markerArray.markers.push_back(pathMarker);
    }
    
    pub->publish(markerArray);
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

        double additional_arrow_scale = 0.08;

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
            double additional_arrow_scale2 = 0.06;
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
                arrow.scale.x = 0.10 * additional_arrow_scale2;
                arrow.scale.y = 0.20 * additional_arrow_scale2;
                arrow.scale.z = 0.20 * additional_arrow_scale2;
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
