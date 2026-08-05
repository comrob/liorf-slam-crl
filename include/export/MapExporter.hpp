#pragma once

#include "export/map_types.hpp"
#include "lili/srv/save_map.hpp"

#include <gtsam/geometry/Pose3.h>

#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <pcl/point_cloud.h>

#include <rclcpp/logger.hpp>
#include <rclcpp/time.hpp>

#include <string>
#include <vector>

class MapExporter
{
public:
    bool executeSave(
        const lili::srv::SaveMap::Request &req,
        lili::srv::SaveMap::Response &res,
        const std::string &savePCDDirectory,
        float mappingSurfLeafSize,
        bool saveDenseGpsTrajectory,
        bool saveDenseOdomTrajectory,
        const sensor_msgs::msg::NavSatFix &storedOriginGpsMsg,
        bool hasGpsOrigin,
        const pcl::PointCloud<PointType>::ConstPtr &cloudKeyPoses3D,
        const pcl::PointCloud<PointTypePose>::ConstPtr &cloudKeyPoses6D,
        const std::vector<pcl::PointCloud<PointType>::Ptr> &surfCloudKeyFrames,
        bool hasEnuLocal,
        const gtsam::Pose3 &tEnuLocal,
        const std::vector<sensor_msgs::msg::NavSatFix> &gpsHistory,
        const std::vector<nav_msgs::msg::Odometry> &densePoseHistory,
        const rclcpp::Logger &logger,
        const rclcpp::Time &saveTime) const;
};
