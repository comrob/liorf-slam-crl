#pragma once

#include <Eigen/Dense>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/logger.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <vector>

class RuntimeTfCoordinator
{
public:
    RuntimeTfCoordinator(
        const rclcpp::Logger &logger,
        const rclcpp::Clock::SharedPtr &clock,
        std::string &lidarFrame,
        std::string &baselinkFrame,
        bool autoLookupLidarToImuTf,
        Eigen::Matrix3d &extRot,
        Eigen::Matrix3d &extRPY,
        Eigen::Vector3d &extTrans,
        Eigen::Quaterniond &extQRPY,
        std::vector<double> &extRotV,
        std::vector<double> &extRPYV,
        std::vector<double> &extTransV);

    void noteImuMessageFrameId(const std::string &frameId);
    void noteLidarMessageFrameId(const std::string &frameId);
    bool ensureImuLidarExtrinsicsResolved(const char *context);

    void initializeLidarBaselinkTfRelationship(const char *context);
    bool tryLookupLidarToBaselinkTf(const char *context);
    bool allowBaselinkFramePublishing(const char *context);

    bool hasLidarToBaselinkTransform() const;
    tf2::Transform lidarToBaselinkTransform() const;

private:
    void logInfo(const std::string &message) const;
    void logWarn(const std::string &message) const;
    void logError(const std::string &message) const;
    double nowWallSec() const;
    void ensureBaselinkFrameDefaultedLocked();
    void setLidarToBaselinkIdentityLocked(const char *context);

    rclcpp::Logger logger_;
    rclcpp::Clock::SharedPtr clock_;

    std::string &lidarFrame_;
    std::string &baselinkFrame_;
    const bool autoLookupLidarToImuTf_;

    Eigen::Matrix3d &extRot_;
    Eigen::Matrix3d &extRPY_;
    Eigen::Vector3d &extTrans_;
    Eigen::Quaterniond &extQRPY_;
    std::vector<double> &extRotV_;
    std::vector<double> &extRPYV_;
    std::vector<double> &extTransV_;

    mutable std::mutex mutex_;
    std::string imuMessageFrameId_;
    std::string lidarMessageFrameId_;
    bool imuLidarExtrinsicsResolved_ = false;
    double lastImuLidarLookupAttemptWall_ = -1.0;
    double lastImuLidarWaitingLogWall_ = -1.0;
    const double imuLidarLookupRetryPeriodSec_ = 5.0;

    tf2::Stamped<tf2::Transform> lidar2Baselink_;
    bool hasLidar2Baselink_ = false;
    double lastLidarBaselinkLookupAttemptWall_ = -1.0;
    const double lidarBaselinkLookupRetryPeriodSec_ = 1.0;
    bool baselinkPublishDecisionMade_ = false;
    bool baselinkPublishAllowed_ = true;

    std::shared_ptr<tf2_ros::Buffer> tfBuffer_;
    std::shared_ptr<tf2_ros::TransformListener> tfListener_;
};