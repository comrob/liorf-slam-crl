#include "mapOptimization/mapOptimization.hpp"

#include <cmath>

using gtsam::BetweenFactor;
using gtsam::Point3;
using gtsam::Pose3;
using gtsam::PriorFactor;
namespace noiseModel = gtsam::noiseModel;

void mapOptimization::timerCallbackPublishOrigin()
{
    std::lock_guard<std::mutex> lock(origin_mutex);
    if (!first_gps)
    {
        stored_origin_gps_msg.header.stamp = this->now();
        pubGpsOrigin->publish(stored_origin_gps_msg);
    }
}

void mapOptimization::initializeDatum(double lat, double lon, double alt, double heading_deg)
{
    std::lock_guard<std::mutex> lock(origin_mutex);
    if (!first_gps)
        return;

    stored_origin_gps_msg.header.stamp = this->now();
    stored_origin_gps_msg.latitude = lat;
    stored_origin_gps_msg.longitude = lon;
    stored_origin_gps_msg.altitude = alt;
    stored_origin_gps_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    first_gps = false;

    gps_trans_.Reset(lat, lon, alt);

    GeographicLib::Geocentric earth = GeographicLib::Geocentric::WGS84();
    double ecef_x, ecef_y, ecef_z;
    earth.Forward(lat, lon, alt, ecef_x, ecef_y, ecef_z);

    const double lat_rad = lat * M_PI / 180.0;
    const double lon_rad = lon * M_PI / 180.0;
    const double sin_lat = std::sin(lat_rad);
    const double cos_lat = std::cos(lat_rad);
    const double sin_lon = std::sin(lon_rad);
    const double cos_lon = std::cos(lon_rad);

    Eigen::Matrix3d R_ecef_enu;
    R_ecef_enu.col(0) = Eigen::Vector3d(-sin_lon, cos_lon, 0.0);
    R_ecef_enu.col(1) = Eigen::Vector3d(-sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat);
    R_ecef_enu.col(2) = Eigen::Vector3d(cos_lat * cos_lon, cos_lat * sin_lon, sin_lat);

    T_EM_estimate = gtsam::Pose3(gtsam::Rot3(R_ecef_enu), gtsam::Point3(ecef_x, ecef_y, ecef_z));
    T_EM_initialized = true;

    if (force_initial_gps)
    {
        double yaw_rad = (90.0 - heading_deg) * M_PI / 180.0;
        T_EL_estimate = gtsam::Pose3(gtsam::Rot3::Yaw(yaw_rad), gtsam::Point3(0, 0, 0));
        forced_anchor_active = true;
        T_EL_initialized = true;
        RCLCPP_INFO(get_logger(), "Manual GPS Datum initialized. Yaw: %.2f rad", yaw_rad);
    }
    else
    {
        RCLCPP_INFO(get_logger(), "GPS origin captured from sensor. Publishing will now begin.");
    }
}

void mapOptimization::gpsHandler(const sensor_msgs::msg::NavSatFix::SharedPtr gpsMsg)
{
    const double ros_stamp_sec = ROS_TIME(gpsMsg->header.stamp);
    const double clock_now_sec = this->now().seconds();

    if (gps_reject_on_invalid_status && gpsMsg->status.status < 0)
    {
        if (diagnostics)
        {
            std::ostringstream ss;
            ss << "[GPS_INPUT_REJECTED] reason=invalid_status"
               << " ros_stamp_s=" << std::fixed << std::setprecision(6) << ros_stamp_sec
               << " clock_now_s=" << clock_now_sec
               << " status=" << (int)gpsMsg->status.status;
            diagnostics->logEventThrottle("gps_input_rejected_status", 1.0, ss.str());
        }
        return;
    }

    if (!std::isfinite(gpsMsg->latitude) || !std::isfinite(gpsMsg->longitude) || !std::isfinite(gpsMsg->altitude))
    {
        if (diagnostics)
        {
            std::ostringstream ss;
            ss << "[GPS_INPUT_REJECTED] reason=non_finite_lla"
               << " ros_stamp_s=" << std::fixed << std::setprecision(6) << ros_stamp_sec
               << " clock_now_s=" << clock_now_sec
               << " lat=" << gpsMsg->latitude
               << " lon=" << gpsMsg->longitude
               << " alt=" << gpsMsg->altitude;
            diagnostics->logEventThrottle("gps_input_rejected_non_finite_lla", 1.0, ss.str());
        }
        return;
    }

    if (diagnostics)
    {
        std::ostringstream gps_input_ss;
        gps_input_ss << "[GPS_INPUT]"
                     << " ros_stamp_s=" << std::fixed << std::setprecision(6) << ros_stamp_sec
                     << " clock_now_s=" << clock_now_sec
                     << " lat=" << std::fixed << std::setprecision(9) << gpsMsg->latitude
                     << " lon=" << std::fixed << std::setprecision(9) << gpsMsg->longitude
                     << " alt=" << std::fixed << std::setprecision(3) << gpsMsg->altitude
                     << " status=" << (int)gpsMsg->status.status
                     << " cov_h=" << std::fixed << std::setprecision(3) << gpsMsg->position_covariance[0];
        diagnostics->logEventThrottle("gps_input_raw", 0.0, gps_input_ss.str());
    }

    if (save_dense_gps_trajectory)
    {
        std::lock_guard<std::mutex> history_lock(gpsHistoryMutex);
        gpsHistory.push_back(*gpsMsg);
    }

    if (diagnostics)
        diagnostics->markGpsUpdate(gpsMsg->header.stamp);

    Eigen::Vector3d trans_local_;
    
    if (first_gps) {
        initializeDatum(gpsMsg->latitude, gpsMsg->longitude, gpsMsg->altitude, 0.0);
    }

    // Before anchor is fully observable, forward raw GPS fix only (no fused orientation topics).
    if (!gpsAnchorReady() && pubLidarGpsFix->get_subscription_count() != 0)
        pubLidarGpsFix->publish(*gpsMsg);

    gps_trans_.Forward(gpsMsg->latitude, gpsMsg->longitude, gpsMsg->altitude, trans_local_[0], trans_local_[1], trans_local_[2]);

    if (!std::isfinite(trans_local_[0]) || !std::isfinite(trans_local_[1]) || !std::isfinite(trans_local_[2]))
    {
        if (diagnostics)
        {
            std::ostringstream ss;
            ss << "[GPS_INPUT_REJECTED] reason=non_finite_enu"
               << " ros_stamp_s=" << std::fixed << std::setprecision(6) << ros_stamp_sec
               << " clock_now_s=" << clock_now_sec
               << " enu_x=" << trans_local_[0]
               << " enu_y=" << trans_local_[1]
               << " enu_z=" << trans_local_[2];
            diagnostics->logEventThrottle("gps_input_rejected_non_finite_enu", 1.0, ss.str());
        }
        return;
    }

    gpsReceivedEnuQueue.emplace_back(trans_local_[0], trans_local_[1], trans_local_[2]);
    if (gpsReceivedEnuQueue.size() > kMaxGpsVizPoints)
        gpsReceivedEnuQueue.pop_front();

    if (diagnostics)
    {
        std::ostringstream gps_enu_ss;
        gps_enu_ss << "[GPS_ENU_CONVERTED]"
                   << " ros_stamp_s=" << std::fixed << std::setprecision(6) << ros_stamp_sec
                   << " clock_now_s=" << clock_now_sec
                   << " enu_x=" << std::fixed << std::setprecision(3) << trans_local_[0]
                   << " enu_y=" << std::fixed << std::setprecision(3) << trans_local_[1]
                   << " enu_z=" << std::fixed << std::setprecision(3) << trans_local_[2]
                   << " first_gps=" << (first_gps ? 1 : 0)
                   << " anchor_ready=" << (gpsAnchorReady() ? 1 : 0);
        diagnostics->logEventThrottle("gps_enu_converted", 0.0, gps_enu_ss.str());
    }

    nav_msgs::msg::Odometry gps_odom;
    gps_odom.header = gpsMsg->header;
    gps_odom.header.frame_id = mapFrameEnu;
    gps_odom.pose.pose.position.x = trans_local_[0];
    gps_odom.pose.pose.position.y = trans_local_[1];
    gps_odom.pose.pose.position.z = trans_local_[2];
    tf2::Quaternion quat_tf;
    quat_tf.setRPY(0.0, 0.0, 0.0);
    geometry_msgs::msg::Quaternion quat_msg;
    tf2::convert(quat_tf, quat_msg);
    gps_odom.pose.pose.orientation = quat_msg;
    if (gpsAnchorReady())
        pubGpsOdom->publish(gps_odom);
    gpsQueue.push_back(gps_odom);
}


void mapOptimization::visualizeGpsConstraints()
{
    if (pubGpsConstraintViz->get_subscription_count() == 0)
        return;

    visualization_msgs::msg::MarkerArray markerArray;

    visualization_msgs::msg::Marker markerGpsPoints;
    markerGpsPoints.header.frame_id = mapFrameEnu;
    markerGpsPoints.header.stamp = timeLaserInfoStamp;
    markerGpsPoints.action = visualization_msgs::msg::Marker::ADD;
    markerGpsPoints.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    markerGpsPoints.ns = "gps_received_points";
    markerGpsPoints.id = 0;
    markerGpsPoints.pose.orientation.w = 1.0;
    markerGpsPoints.scale.x = 0.25;
    markerGpsPoints.scale.y = 0.25;
    markerGpsPoints.scale.z = 0.25;
    markerGpsPoints.color.r = 1.0;
    markerGpsPoints.color.g = 0.2;
    markerGpsPoints.color.b = 0.2;
    markerGpsPoints.color.a = 0.9;

    for (const auto &p_gps : gpsReceivedEnuQueue)
    {
        geometry_msgs::msg::Point p;
        p.x = p_gps.x();
        p.y = p_gps.y();
        p.z = p_gps.z();
        markerGpsPoints.points.push_back(p);
    }

    visualization_msgs::msg::Marker markerLidarPoints;
    markerLidarPoints.header.frame_id = mapFrameEnu;
    markerLidarPoints.header.stamp = timeLaserInfoStamp;
    markerLidarPoints.action = visualization_msgs::msg::Marker::ADD;
    markerLidarPoints.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    markerLidarPoints.ns = "gps_associated_lidar_points";
    markerLidarPoints.id = 1;
    markerLidarPoints.pose.orientation.w = 1.0;
    markerLidarPoints.scale.x = 0.22;
    markerLidarPoints.scale.y = 0.22;
    markerLidarPoints.scale.z = 0.22;
    markerLidarPoints.color.r = 0.1;
    markerLidarPoints.color.g = 0.9;
    markerLidarPoints.color.b = 0.2;
    markerLidarPoints.color.a = 0.95;

    visualization_msgs::msg::Marker markerEdges;
    markerEdges.header.frame_id = mapFrameEnu;
    markerEdges.header.stamp = timeLaserInfoStamp;
    markerEdges.action = visualization_msgs::msg::Marker::ADD;
    markerEdges.type = visualization_msgs::msg::Marker::LINE_LIST;
    markerEdges.ns = "gps_lidar_links";
    markerEdges.id = 2;
    markerEdges.pose.orientation.w = 1.0;
    markerEdges.scale.x = 0.06;
    markerEdges.color.r = 1.0;
    markerEdges.color.g = 1.0;
    markerEdges.color.b = 0.0;
    markerEdges.color.a = 0.9;

    if (gpsAnchorReady())
    {
        for (const auto &assoc : gpsLidarAssociationQueue)
        {
            const int lidar_key = assoc.first;
            const gtsam::Point3 &p_gps_enu = assoc.second.first;
            const double gps_time = assoc.second.second;
            if (lidar_key < 0 || lidar_key >= static_cast<int>(cloudKeyPoses6D->points.size()))
                continue;

            const double keyframe_time = cloudKeyPoses6D->points[lidar_key].time;
            const double assoc_time_diff = std::abs(keyframe_time - gps_time);
            if (assoc_time_diff > gps_max_constraint_dt_sec)
            {
                RCLCPP_WARN_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    5000,
                "Visualized GPS-LiDAR association exceeds dt threshold: %.3f s (gps=%.3f, keyframe=%.3f, max=%.3f)",
                    assoc_time_diff,
                    gps_time,
                keyframe_time,
                gps_max_constraint_dt_sec);
            }

            const auto &lidar_local = cloudKeyPoses6D->points[lidar_key];
            gtsam::Point3 p_lidar_enu = T_EL_estimate.transformFrom(gtsam::Point3(lidar_local.x, lidar_local.y, lidar_local.z));

            geometry_msgs::msg::Point p_lidar_msg;
            p_lidar_msg.x = p_lidar_enu.x();
            p_lidar_msg.y = p_lidar_enu.y();
            p_lidar_msg.z = p_lidar_enu.z();
            markerLidarPoints.points.push_back(p_lidar_msg);

            geometry_msgs::msg::Point p_gps_msg;
            p_gps_msg.x = p_gps_enu.x();
            p_gps_msg.y = p_gps_enu.y();
            p_gps_msg.z = p_gps_enu.z();

            markerEdges.points.push_back(p_gps_msg);
            markerEdges.points.push_back(p_lidar_msg);
        }
    }

    markerArray.markers.push_back(markerGpsPoints);
    markerArray.markers.push_back(markerLidarPoints);
    markerArray.markers.push_back(markerEdges);
    pubGpsConstraintViz->publish(markerArray);
}


void mapOptimization::addGPSFactor()
{
    if (gpsQueue.empty())
        return;

    // wait for system initialized and settles down
    if (cloudKeyPoses3D->points.empty())
        return;
    else
    {
        if (common_lib_->pointDistance(cloudKeyPoses3D->front(), cloudKeyPoses3D->back()) < 5.0)
            return;
    }

    // pose covariance small, no need to correct
    // if (poseCovariance(3,3) < poseCovThreshold && poseCovariance(4,4) < poseCovThreshold)
    //     return;

    // last gps position
    static PointType lastGPSPoint;

    while (!gpsQueue.empty())
    {
        const double gps_stamp = ROS_TIME(gpsQueue.front().header.stamp);
        const double gps_eligible_time = gps_stamp + gps_processing_delay_sec;
        const double clock_now_sec = this->now().seconds();

        nav_msgs::msg::Odometry thisGPS = gpsQueue.front();
        float gps_x = thisGPS.pose.pose.position.x;
        float gps_y = thisGPS.pose.pose.position.y;
        float gps_z = thisGPS.pose.pose.position.z;
        float noise_x = thisGPS.pose.covariance[0];
        float noise_y = thisGPS.pose.covariance[7];
        float noise_z = thisGPS.pose.covariance[14];

        if (gps_eligible_time < timeLaserInfoCur - 0.2)
        {
            // message too old
            if (diagnostics)
            {
                std::ostringstream ss;
                ss << "[GPS_REJECTED] reason=too_old"
                   << " ros_stamp_s=" << std::fixed << std::setprecision(6) << gps_stamp
                         << " clock_now_s=" << clock_now_sec
                   << " eligible_s=" << gps_eligible_time
                   << " lidar_s=" << timeLaserInfoCur
                   << " dt_s=" << (gps_eligible_time - timeLaserInfoCur)
                   << " enu_xyz=(" << gps_x << "," << gps_y << "," << gps_z << ")";
                diagnostics->logEventThrottle("gps_rejected_too_old", 1.0, ss.str());
            }
            gpsQueue.pop_front();
        }
        else if (gps_eligible_time > timeLaserInfoCur + 0.2)
        {
            // message not yet eligible for delayed processing
            if (diagnostics)
            {
                std::ostringstream ss;
                ss << "[GPS_PENDING] reason=not_yet_eligible"
                   << " ros_stamp_s=" << std::fixed << std::setprecision(6) << gps_stamp
                         << " clock_now_s=" << clock_now_sec
                   << " eligible_s=" << gps_eligible_time
                   << " lidar_s=" << timeLaserInfoCur
                   << " dt_s=" << (gps_eligible_time - timeLaserInfoCur);
                diagnostics->logEventThrottle("gps_pending_not_yet_eligible", 1.0, ss.str());
            }
            break;
        }
        else
        {
            gpsQueue.pop_front();

            if (!useGpsElevation)
            {
                gps_z = transformTobeMapped[5];
                noise_z = 0.01;
            }

            if (!std::isfinite(gps_x) || !std::isfinite(gps_y) || !std::isfinite(gps_z) ||
                !std::isfinite(noise_x) || !std::isfinite(noise_y) || !std::isfinite(noise_z))
            {
                if (diagnostics)
                {
                    std::ostringstream ss;
                    ss << "[GPS_REJECTED] reason=non_finite_measurement"
                       << " ros_stamp_s=" << std::fixed << std::setprecision(6) << gps_stamp
                       << " clock_now_s=" << clock_now_sec
                       << " gps_xyz=(" << gps_x << "," << gps_y << "," << gps_z << ")"
                       << " cov_xyz=(" << noise_x << "," << noise_y << "," << noise_z << ")";
                    diagnostics->logEventThrottle("gps_rejected_non_finite_measurement", 1.0, ss.str());
                }
                continue;
            }

            // GPS too noisy, skip
            if (noise_x > gpsCovThreshold || noise_y > gpsCovThreshold)
            {
                if (diagnostics)
                {
                    std::ostringstream ss;
                    ss << "[GPS_REJECTED] reason=high_noise"
                       << " ros_stamp_s=" << std::fixed << std::setprecision(6) << gps_stamp
                              << " clock_now_s=" << clock_now_sec
                       << " noise_x=" << std::fixed << std::setprecision(3) << noise_x
                       << " noise_y=" << std::fixed << std::setprecision(3) << noise_y
                       << " noise_z=" << std::fixed << std::setprecision(3) << noise_z
                       << " threshold=" << gpsCovThreshold
                       << " enu_xyz=(" << gps_x << "," << gps_y << "," << gps_z << ")";
                    diagnostics->logEventThrottle("gps_rejected_high_noise", 1.0, ss.str());
                }
                continue;
            }

            // GPS not properly initialized (0,0,0)
            if (abs(gps_x) < 1e-6 && abs(gps_y) < 1e-6)
            {
                if (diagnostics)
                {
                    std::ostringstream ss;
                    ss << "[GPS_REJECTED] reason=uninitialized"
                       << " ros_stamp_s=" << std::fixed << std::setprecision(6) << gps_stamp
                              << " clock_now_s=" << clock_now_sec
                       << " enu_xyz=(" << gps_x << "," << gps_y << "," << gps_z << ")";
                    diagnostics->logEventThrottle("gps_rejected_uninitialized", 1.0, ss.str());
                }
                continue;
            }

            // Add GPS every a few meters
            PointType curGPSPoint;
            curGPSPoint.x = gps_x;
            curGPSPoint.y = gps_y;
            curGPSPoint.z = gps_z;
            if (common_lib_->pointDistance(curGPSPoint, lastGPSPoint) < 5.0)
            {
                if (diagnostics)
                {
                    std::ostringstream ss;
                    ss << "[GPS_REJECTED] reason=spatial_sparsity"
                       << " ros_stamp_s=" << std::fixed << std::setprecision(6) << gps_stamp
                              << " clock_now_s=" << clock_now_sec
                       << " dist_m=" << std::fixed << std::setprecision(3) << common_lib_->pointDistance(curGPSPoint, lastGPSPoint)
                       << " min_dist_m=5.0"
                       << " enu_xyz=(" << gps_x << "," << gps_y << "," << gps_z << ")";
                    diagnostics->logEventThrottle("gps_rejected_sparsity", 1.0, ss.str());
                }
                continue;
            }

            // 1. Initialize T_GL if this is the first GPS fusion
            if (!gtsam_anchor_inserted) {
                gtsam::Pose3 current_local_pose = pclPointTogtsamPose3(cloudKeyPoses6D->points.back());
                gtsam::Point3 initial_translation(gps_x - current_local_pose.x(), 
                                                  gps_y - current_local_pose.y(), 
                                                  gps_z - current_local_pose.z());

                if (!forced_anchor_active) {
                    T_EL_estimate = gtsam::Pose3(gtsam::Rot3::Identity(), initial_translation);
                } else {
                    // Keep manual rotation, but snap translation to the first real GPS.
                    T_EL_estimate = gtsam::Pose3(T_EL_estimate.rotation(), initial_translation);
                }

                initialEstimate.insert(T_EL_KEY, T_EL_estimate);

                gtsam::Vector6 prior_noise_vector;
                prior_noise_vector << 1e-2, 1e-2, M_PI, 1e8, 1e8, 1e8;
                gtSAMgraph.add(PriorFactor<Pose3>(T_EL_KEY, T_EL_estimate, noiseModel::Diagonal::Variances(prior_noise_vector)));

                T_EL_initialized = true;
                gtsam_anchor_inserted = true;
            }

            // 2. Create Noise Model and Custom Factor
            // Inflate GPS factor uncertainty to bound long-term drift when GPS/LiDAR timing can be de-synchronized.
            const double gps_covariance_inflation_var =
                std::max(0.0, gps_covariance_inflation_m) * std::max(0.0, gps_covariance_inflation_m);
            const double gps_var_x = std::max(static_cast<double>(noise_x), 1.0) + gps_covariance_inflation_var;
            const double gps_var_y = std::max(static_cast<double>(noise_y), 1.0) + gps_covariance_inflation_var;
            const double gps_var_z = std::max(static_cast<double>(noise_z), 1.0) + gps_covariance_inflation_var;
            gtsam::Vector3 gpsVarianceVec;
            gpsVarianceVec << gps_var_x, gps_var_y, gps_var_z;
            noiseModel::Diagonal::shared_ptr gps_noise = noiseModel::Diagonal::Variances(gpsVarianceVec);

            gtsam::Point3 measured_gps(gps_x, gps_y, gps_z);
            gtsam::Point3 antenna_offset(gpsAntennaOffsetX, gpsAntennaOffsetY, gpsAntennaOffsetZ);

            // Match this GPS measurement to the closest keyframe in time.
            // Include both recent saved keyframes and the current in-flight keyframe.
            const double gps_time = ROS_TIME(thisGPS.header.stamp);
            const int current_keyframe_idx = cloudKeyPoses3D->size();
            int best_keyframe_idx = current_keyframe_idx;
            double best_keyframe_time = timeLaserInfoCur;
            double best_time_diff = std::abs(best_keyframe_time - gps_time);

            const int search_start_idx = std::max(0, static_cast<int>(cloudKeyPoses6D->size()) - gps_keyframe_search_window);
            for (int i = search_start_idx; i < static_cast<int>(cloudKeyPoses6D->size()); ++i)
            {
                const double keyframe_time = cloudKeyPoses6D->points[i].time;
                const double time_diff = std::abs(keyframe_time - gps_time);
                if (time_diff < best_time_diff)
                {
                    best_time_diff = time_diff;
                    best_keyframe_idx = i;
                    best_keyframe_time = keyframe_time;
                }
            }

            // Diagnostic: Log keyframe timing pattern if enabled
            if (diagnostics && best_time_diff > gps_max_constraint_dt_sec)
            {
                int num_keyframes_checked = std::min((int)gps_keyframe_search_window, (int)cloudKeyPoses6D->size());
                double oldest_keyframe_time = (search_start_idx < (int)cloudKeyPoses6D->size()) ? 
                    cloudKeyPoses6D->points[search_start_idx].time : -1.0;
                double newest_keyframe_time = cloudKeyPoses6D->size() > 0 ? 
                    cloudKeyPoses6D->points[cloudKeyPoses6D->size()-1].time : -1.0;
                double keyframe_time_span = (newest_keyframe_time > 0 && oldest_keyframe_time > 0) ? 
                    (newest_keyframe_time - oldest_keyframe_time) : -1.0;
                
                std::ostringstream diag_ss;
                diag_ss << "[GPS_TIME_ALIGNMENT_DIAGNOSTIC]"
                       << " gps_t=" << std::fixed << std::setprecision(6) << gps_time
                       << " total_keyframes=" << cloudKeyPoses6D->size()
                       << " search_window=" << gps_keyframe_search_window
                       << " keyframes_checked=" << num_keyframes_checked
                       << " oldest_kf_t=" << oldest_keyframe_time
                       << " newest_kf_t=" << newest_keyframe_time
                       << " kf_span_s=" << keyframe_time_span
                       << " best_match_idx=" << best_keyframe_idx
                       << " best_match_t=" << best_keyframe_time
                       << " best_dt_s=" << best_time_diff
                       << " threshold_s=" << gps_max_constraint_dt_sec
                       << " (suggests: " << (keyframe_time_span > 0 ? 
                           (keyframe_time_span > best_time_diff * 2 ? "sparse_keyframes" : "timestamp_sync_issue")
                           : "unknown") << ")";
                diagnostics->logEvent(diag_ss.str());
            }

            if (best_time_diff > gps_max_constraint_dt_sec)
            {
                if (diagnostics)
                {
                    std::ostringstream ss;
                    ss << "[GPS_REJECTED] reason=time_alignment"
                       << " ros_stamp_s=" << std::fixed << std::setprecision(6) << gps_time
                              << " clock_now_s=" << clock_now_sec
                       << " keyframe_idx=" << best_keyframe_idx
                       << " gps_t=" << gps_time
                       << " keyframe_t=" << best_keyframe_time
                       << " dt_s=" << best_time_diff
                       << " max_dt_s=" << gps_max_constraint_dt_sec
                       << " enu_xyz=(" << gps_x << "," << gps_y << "," << gps_z << ")"
                       << " cov_xyz=(" << noise_x << "," << noise_y << "," << noise_z << ")";
                    diagnostics->logEventThrottle("gps_rejected_time_alignment", 1.0, ss.str());
                }
                RCLCPP_WARN_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    5000,
                    "Skipping GPS constraint due to dt threshold: %.3f s (gps=%.3f, keyframe=%.3f, max=%.3f)",
                    best_time_diff,
                    gps_time,
                    best_keyframe_time,
                    gps_max_constraint_dt_sec);

                if (diagnostics)
                {
                    std::ostringstream skippedConstraintOss;
                    skippedConstraintOss << "[GPS_CONSTRAINT_SKIPPED_TIME]"
                                         << " candidate_idx=" << best_keyframe_idx
                                         << " gps_t=" << std::fixed << std::setprecision(6) << gps_time
                                         << " keyframe_t=" << best_keyframe_time
                                         << " dt=" << best_time_diff
                                         << " max_dt=" << gps_max_constraint_dt_sec;
                    diagnostics->logEvent(skippedConstraintOss.str());
                }
                continue;
            }

            // Bypass Expression framework completely
            const int lidar_key = best_keyframe_idx;
            FloatingAnchorFactor gps_factor(T_EL_KEY, lidar_key, measured_gps, antenna_offset, gps_noise);
            gtSAMgraph.add(gps_factor);
            ++gpsFactorsAccepted;

            std::ostringstream gpsConstraintOss;
            gpsConstraintOss << "[GPS_CONSTRAINT_ADDED]"
                             << " pose_idx=" << lidar_key
                             << " factor_keys=(T0," << lidar_key << ")"
                             << " accepted=" << gpsFactorsAccepted
                             << " ros_stamp_s=" << std::fixed << std::setprecision(6) << gps_time
                             << " clock_now_s=" << clock_now_sec
                             << " keyframe_idx=" << best_keyframe_idx
                             << " gps_t=" << gps_time
                             << " keyframe_t=" << best_keyframe_time
                             << " dt=" << best_time_diff
                             << " gps_xyz=(" << measured_gps.x() << "," << measured_gps.y() << "," << measured_gps.z() << ")"
                             << " cov_xyz=(" << noise_x << "," << noise_y << "," << noise_z << ")";

            RCLCPP_INFO_STREAM(get_logger(), gpsConstraintOss.str());
            if (diagnostics)
                diagnostics->logEvent(gpsConstraintOss.str());

            gpsLidarAssociationQueue.emplace_back(lidar_key, std::make_pair(measured_gps, gps_time));
            const size_t kMaxGpsVizPoints = 2000;
            if (gpsLidarAssociationQueue.size() > kMaxGpsVizPoints)
                gpsLidarAssociationQueue.pop_front();

            if (rebuild_on_gps_jump && (last_gps_rebuild_time < 0.0 || (timeLaserInfoCur - last_gps_rebuild_time) > 60.0))
            {
                markMapRebuildTriggered("gps_periodic_60s");
                last_gps_rebuild_time = timeLaserInfoCur;
            }

            aLoopIsClosed = true;
            break;
        }
    }
}

void mapOptimization::publishLidarGpsFix()
{
    if (first_gps || !gpsAnchorReady())
        return;
    if (pubLidarGpsFix->get_subscription_count() == 0 &&
        pubLidarGpsEnuPose->get_subscription_count() == 0 &&
        pubLidarGpsNedPose->get_subscription_count() == 0 &&
        pubBaselinkGpsEnuOdometry->get_subscription_count() == 0 &&
        pubBaselinkGpsNedOdometry->get_subscription_count() == 0)
        return;

    // Transform LiDAR local pose into ENU frame via the floating-anchor T_GL
    gtsam::Point3 p_local(transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5]);
    gtsam::Point3 p_enu = T_EL_estimate.transformFrom(p_local);

    // Project ENU position back to geodetic lat/lon/alt
    double lat, lon, alt;
    gps_trans_.Reverse(p_enu.x(), p_enu.y(), p_enu.z(), lat, lon, alt);

    // Build local rotation identically to publishOdometry: setRPY(roll, pitch, yaw)
    tf2::Quaternion q_local_tf2;
    q_local_tf2.setRPY(transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
    gtsam::Rot3 R_local(Eigen::Quaterniond(
        q_local_tf2.w(), q_local_tf2.x(), q_local_tf2.y(), q_local_tf2.z()));

    // Rotate LiDAR orientation into ENU via the floating-anchor T_GL
    gtsam::Rot3 R_enu = T_EL_estimate.rotation().compose(R_local);
    gtsam::Quaternion q_enu = R_enu.toQuaternion();

    // Fixed rotation: ENU (East-North-Up) -> NED (North-East-Down)
    // North = ENU_Y, East = ENU_X, Down = -ENU_Z
    static const gtsam::Rot3 R_ned_from_enu(
         0.0, 1.0,  0.0,
         1.0, 0.0,  0.0,
         0.0, 0.0, -1.0);
    gtsam::Point3 p_ned = R_ned_from_enu.rotate(p_enu);
    gtsam::Rot3   R_ned = R_ned_from_enu.compose(R_enu);
    gtsam::Quaternion q_ned = R_ned.toQuaternion();

    if (pubLidarGpsFix->get_subscription_count() != 0)
    {
        sensor_msgs::msg::NavSatFix fix_msg;
        fix_msg.header.stamp = timeLaserInfoStamp;
        fix_msg.header.frame_id = mapFrameEnu;
        fix_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
        fix_msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
        fix_msg.latitude  = lat;
        fix_msg.longitude = lon;
        fix_msg.altitude  = alt;
        fix_msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
        pubLidarGpsFix->publish(fix_msg);
    }

    if (pubLidarGpsEnuPose->get_subscription_count() != 0)
    {
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = timeLaserInfoStamp;
        pose_msg.header.frame_id = mapFrameEnu;
        pose_msg.pose.position.x = p_enu.x();
        pose_msg.pose.position.y = p_enu.y();
        pose_msg.pose.position.z = p_enu.z();
        pose_msg.pose.orientation.x = q_enu.x();
        pose_msg.pose.orientation.y = q_enu.y();
        pose_msg.pose.orientation.z = q_enu.z();
        pose_msg.pose.orientation.w = q_enu.w();
        pubLidarGpsEnuPose->publish(pose_msg);
    }

    if (pubLidarGpsNedPose->get_subscription_count() != 0)
    {
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = timeLaserInfoStamp;
        pose_msg.header.frame_id = mapFrameNed;
        pose_msg.pose.position.x = p_ned.x();
        pose_msg.pose.position.y = p_ned.y();
        pose_msg.pose.position.z = p_ned.z();
        pose_msg.pose.orientation.x = q_ned.x();
        pose_msg.pose.orientation.y = q_ned.y();
        pose_msg.pose.orientation.z = q_ned.z();
        pose_msg.pose.orientation.w = q_ned.w();
        pubLidarGpsNedPose->publish(pose_msg);
    }

    if ((lidarFrame == baselinkFrame || hasLidar2Baselink) &&
        (pubBaselinkGpsEnuOdometry->get_subscription_count() != 0 ||
         pubBaselinkGpsNedOdometry->get_subscription_count() != 0))
    {
        tf2::Quaternion q_enu_tf(q_enu.x(), q_enu.y(), q_enu.z(), q_enu.w());
        tf2::Quaternion q_ned_tf(q_ned.x(), q_ned.y(), q_ned.z(), q_ned.w());

        tf2::Transform t_enu_to_lidar(q_enu_tf, tf2::Vector3(p_enu.x(), p_enu.y(), p_enu.z()));
        tf2::Transform t_ned_to_lidar(q_ned_tf, tf2::Vector3(p_ned.x(), p_ned.y(), p_ned.z()));
        tf2::Transform t_enu_to_baselink = t_enu_to_lidar * lidar2Baselink;
        tf2::Transform t_ned_to_baselink = t_ned_to_lidar * lidar2Baselink;

        if (pubBaselinkGpsEnuOdometry->get_subscription_count() != 0)
        {
            pubBaselinkGpsEnuOdometry->publish(
                odometryMsgFromAffine(affineFromTf(t_enu_to_baselink), timeLaserInfoStamp, mapFrameEnu, baselinkFrame));
        }

        if (pubBaselinkGpsNedOdometry->get_subscription_count() != 0)
        {
            pubBaselinkGpsNedOdometry->publish(
                odometryMsgFromAffine(affineFromTf(t_ned_to_baselink), timeLaserInfoStamp, mapFrameNed, baselinkFrame));
        }
    }
}

