#include "tf_runtime.hpp"

#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <iomanip>
#include <sstream>

RuntimeTfCoordinator::RuntimeTfCoordinator(
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
    std::vector<double> &extTransV)
    : logger_(logger),
      clock_(clock),
      lidarFrame_(lidarFrame),
      baselinkFrame_(baselinkFrame),
      autoLookupLidarToImuTf_(autoLookupLidarToImuTf),
      extRot_(extRot),
      extRPY_(extRPY),
      extTrans_(extTrans),
      extQRPY_(extQRPY),
      extRotV_(extRotV),
      extRPYV_(extRPYV),
      extTransV_(extTransV)
{
    tfBuffer_ = std::make_shared<tf2_ros::Buffer>(clock_);
    tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);

    tf2::Transform identity;
    identity.setIdentity();
    lidar2Baselink_.setData(identity);

    std::lock_guard<std::mutex> lock(mutex_);
    ensureBaselinkFrameDefaultedLocked();
    if (autoLookupLidarToImuTf_)
    {
        logInfo("[IMU_LIDAR_TF_INIT] auto lookup enabled; awaiting IMU and LiDAR message frame ids");
    }
    else
    {
        imuLidarExtrinsicsResolved_ = true;
    }
}

void RuntimeTfCoordinator::logInfo(const std::string &message) const
{
    RCLCPP_INFO_STREAM(logger_, message);
}

void RuntimeTfCoordinator::logWarn(const std::string &message) const
{
    RCLCPP_WARN_STREAM(logger_, message);
}

void RuntimeTfCoordinator::logError(const std::string &message) const
{
    RCLCPP_ERROR_STREAM(logger_, message);
}

double RuntimeTfCoordinator::nowWallSec() const
{
    return clock_->now().seconds();
}

void RuntimeTfCoordinator::ensureBaselinkFrameDefaultedLocked()
{
    if (baselinkFrame_.empty() && !lidarFrame_.empty())
    {
        baselinkFrame_ = lidarFrame_;
        logInfo(
            "[BASELINK_FRAME_DEFAULTED] baselinkFrame was empty; using LiDAR frame '" +
            baselinkFrame_ + "'");
    }
}

void RuntimeTfCoordinator::noteImuMessageFrameId(const std::string &frameId)
{
    if (!autoLookupLidarToImuTf_ || frameId.empty())
        return;

    std::lock_guard<std::mutex> lock(mutex_);
    if (imuMessageFrameId_ == frameId)
        return;

    if (!imuMessageFrameId_.empty() && imuMessageFrameId_ != frameId)
    {
        logWarn(
            "[IMU_LIDAR_TF_FRAME_CHANGE] imu frame changed from '" + imuMessageFrameId_ +
            "' to '" + frameId + "' before extrinsics were resolved");
    }

    imuMessageFrameId_ = frameId;
}

void RuntimeTfCoordinator::noteLidarMessageFrameId(const std::string &frameId)
{
    if (frameId.empty())
        return;

    std::lock_guard<std::mutex> lock(mutex_);
    if (lidarMessageFrameId_ == frameId)
        return;

    if (!lidarMessageFrameId_.empty() && lidarMessageFrameId_ != frameId)
    {
        logWarn(
            "[IMU_LIDAR_TF_FRAME_CHANGE] lidar frame changed from '" + lidarMessageFrameId_ +
            "' to '" + frameId + "' before extrinsics were resolved");
    }

    lidarMessageFrameId_ = frameId;

    if (lidarFrame_.empty())
    {
        lidarFrame_ = frameId;
        logInfo(
            "[IMU_LIDAR_FRAME_DEFAULTED] lidarFrame was empty; using resolved LiDAR message frame '" +
            lidarFrame_ + "'");
    }

    ensureBaselinkFrameDefaultedLocked();
}

bool RuntimeTfCoordinator::ensureImuLidarExtrinsicsResolved(const char *context)
{
    if (!autoLookupLidarToImuTf_)
        return true;

    std::lock_guard<std::mutex> lock(mutex_);
    if (imuLidarExtrinsicsResolved_)
        return true;

    const auto logWaitingState = [&](const std::string &reason) {
        const double nowWall = nowWallSec();
        if (lastImuLidarWaitingLogWall_ >= 0.0 &&
            (nowWall - lastImuLidarWaitingLogWall_) < imuLidarLookupRetryPeriodSec_)
        {
            return;
        }
        lastImuLidarWaitingLogWall_ = nowWall;

        std::ostringstream ss;
        ss << "[IMU_LIDAR_TF_LOOKUP_WAIT] (" << context << ") " << reason;
        if (!imuMessageFrameId_.empty() || !lidarMessageFrameId_.empty())
        {
            ss << " imu_frame='" << (imuMessageFrameId_.empty() ? "<unset>" : imuMessageFrameId_) << "'"
               << " lidar_frame='" << (lidarMessageFrameId_.empty() ? "<unset>" : lidarMessageFrameId_) << "'";
        }
        logWarn(ss.str());
    };

    if (imuMessageFrameId_.empty() || lidarMessageFrameId_.empty())
    {
        logWaitingState("awaiting IMU and LiDAR message frame ids");
        return false;
    }

    const double nowWall = nowWallSec();
    if (lastImuLidarLookupAttemptWall_ >= 0.0 &&
        (nowWall - lastImuLidarLookupAttemptWall_) < imuLidarLookupRetryPeriodSec_)
    {
        logWaitingState("waiting before the next static TF lookup retry");
        return false;
    }
    lastImuLidarLookupAttemptWall_ = nowWall;

    try
    {
        geometry_msgs::msg::TransformStamped imuToLidarMsg =
            tfBuffer_->lookupTransform(lidarMessageFrameId_, imuMessageFrameId_, rclcpp::Time(0));

        tf2::Transform imuToLidarTf;
        tf2::fromMsg(imuToLidarMsg.transform, imuToLidarTf);
        tf2::Transform lidarToImuTf = imuToLidarTf.inverse();
        tf2::Matrix3x3 imuToLidarRot(imuToLidarTf.getRotation());

        for (int row = 0; row < 3; ++row)
        {
            for (int col = 0; col < 3; ++col)
            {
                extRot_(row, col) = imuToLidarRot[row][col];
                extRPY_(row, col) = imuToLidarRot[row][col];
            }
        }

        extTrans_ = Eigen::Vector3d(
            lidarToImuTf.getOrigin().x(),
            lidarToImuTf.getOrigin().y(),
            lidarToImuTf.getOrigin().z());
        extQRPY_ = Eigen::Quaterniond(extRPY_).inverse();

        extRotV_.clear();
        extRPYV_.clear();
        for (int row = 0; row < 3; ++row)
        {
            for (int col = 0; col < 3; ++col)
            {
                extRotV_.push_back(extRot_(row, col));
                extRPYV_.push_back(extRPY_(row, col));
            }
        }
        extTransV_ = {extTrans_.x(), extTrans_.y(), extTrans_.z()};

        imuLidarExtrinsicsResolved_ = true;

        const auto &rot = imuToLidarMsg.transform.rotation;
        std::ostringstream ss;
        ss << "[IMU_LIDAR_TF_LOOKUP_OK] (" << context << ") imu_frame='" << imuMessageFrameId_
           << "' lidar_frame='" << lidarMessageFrameId_ << "'"
           << " resolved_tf=lookupTransform(target='" << lidarMessageFrameId_
           << "', source='" << imuMessageFrameId_ << "')"
           << " -> imu_to_lidar"
           << " extTrans_lidar_to_imu=(" << extTrans_.x() << ", " << extTrans_.y() << ", " << extTrans_.z() << ")"
           << " extRot_imu_to_lidar_quat_xyzw=(" << rot.x << ", " << rot.y << ", " << rot.z << ", " << rot.w << ")";
        logInfo(ss.str());
        return true;
    }
    catch (const tf2::TransformException &ex)
    {
        logWaitingState(std::string("static TF lookup failed: ") + ex.what());
        std::ostringstream ss;
        ss << "[IMU_LIDAR_TF_LOOKUP_FAIL] (" << context << ")"
           << " imu_frame='" << imuMessageFrameId_ << "'"
           << " lidar_frame='" << lidarMessageFrameId_ << "'"
           << " reason=" << ex.what()
           << "; localization outputs remain disabled; specify manual extrinsicRot/extrinsicRPY/extrinsicTrans if this TF is not published";
        logError(ss.str());
        return false;
    }
}

void RuntimeTfCoordinator::setLidarToBaselinkIdentityLocked(const char *context)
{
    tf2::Transform identity;
    identity.setIdentity();
    lidar2Baselink_.setData(identity);
    hasLidar2Baselink_ = true;

    std::ostringstream ss;
    ss << "[TF_INIT] (" << context << ") lidarFrame == baselinkFrame ('" << lidarFrame_
       << "'): lidar2baselink is identity by definition, hasLidar2Baselink=true";
    logInfo(ss.str());
}

void RuntimeTfCoordinator::initializeLidarBaselinkTfRelationship(const char *context)
{
    std::lock_guard<std::mutex> lock(mutex_);
    ensureBaselinkFrameDefaultedLocked();

    if (lidarFrame_.empty())
    {
        logInfo("[TF_INIT] lidarFrame is empty: deferring lidar<->baselink initialization until the first LiDAR message establishes the frame");
        return;
    }

    if (lidarFrame_ == baselinkFrame_)
    {
        setLidarToBaselinkIdentityLocked(context);
        return;
    }

    std::ostringstream ss;
    ss << "[TF_INIT] lidarFrame != baselinkFrame ('" << lidarFrame_ << "' vs '" << baselinkFrame_
       << "'): attempting initial lookup";
    logInfo(ss.str());
    lastLidarBaselinkLookupAttemptWall_ = -1.0;
    hasLidar2Baselink_ = false;

    // Fall through to normal throttled lookup outside this locked section.
    // The next call will retry immediately.
}

bool RuntimeTfCoordinator::tryLookupLidarToBaselinkTf(const char *context)
{
    std::lock_guard<std::mutex> lock(mutex_);
    ensureBaselinkFrameDefaultedLocked();

    if (lidarFrame_.empty() || baselinkFrame_.empty())
        return false;

    if (lidarFrame_ == baselinkFrame_)
    {
        if (!hasLidar2Baselink_)
            setLidarToBaselinkIdentityLocked(context);
        return true;
    }

    const double nowWall = nowWallSec();
    if (lastLidarBaselinkLookupAttemptWall_ >= 0.0 &&
        (nowWall - lastLidarBaselinkLookupAttemptWall_) < lidarBaselinkLookupRetryPeriodSec_)
    {
        return hasLidar2Baselink_;
    }
    lastLidarBaselinkLookupAttemptWall_ = nowWall;

    try
    {
        geometry_msgs::msg::TransformStamped lidarToBaseMsg =
            tfBuffer_->lookupTransform(lidarFrame_, baselinkFrame_, rclcpp::Time(0));

        tf2::fromMsg(lidarToBaseMsg, lidar2Baselink_);
        hasLidar2Baselink_ = true;

        const auto &tr = lidarToBaseMsg.transform.translation;
        const auto &qr = lidarToBaseMsg.transform.rotation;
        double roll, pitch, yaw;
        tf2::Quaternion q(qr.x, qr.y, qr.z, qr.w);
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        std::ostringstream ss;
        ss << "[TF_LOOKUP_OK] (" << context << ") lookupTransform success target='" << lidarFrame_
           << "' source='" << baselinkFrame_ << "'"
           << " stamp=" << std::fixed << std::setprecision(6) << rclcpp::Time(lidarToBaseMsg.header.stamp).seconds()
           << " xyz=(" << tr.x << ", " << tr.y << ", " << tr.z << ")"
           << " quat_xyzw=(" << qr.x << ", " << qr.y << ", " << qr.z << ", " << qr.w << ")"
           << " rpy=(" << roll << ", " << pitch << ", " << yaw << ")";
        logInfo(ss.str());
        return true;
    }
    catch (const tf2::TransformException &ex)
    {
        hasLidar2Baselink_ = false;
        std::ostringstream ss;
        ss << "[TF_LOOKUP_FAIL] (" << context << ") lookupTransform failed target='" << lidarFrame_
           << "' source='" << baselinkFrame_ << "' reason=" << ex.what();
        logWarn(ss.str());
        return false;
    }
}

bool RuntimeTfCoordinator::allowBaselinkFramePublishing(const char *context)
{
    std::lock_guard<std::mutex> lock(mutex_);
    ensureBaselinkFrameDefaultedLocked();
    if (baselinkPublishDecisionMade_ || baselinkFrame_.empty())
        return baselinkPublishAllowed_;

    const std::string framesYaml = tfBuffer_->allFramesAsYAML();
    if (framesYaml.empty())
    {
        baselinkPublishDecisionMade_ = true;
        baselinkPublishAllowed_ = true;
        return true;
    }

    std::istringstream stream(framesYaml);
    std::string line;
    std::map<std::string, std::string> parentByFrame;
    std::string currentFrame;

    while (std::getline(stream, line))
    {
        if (line.empty())
            continue;

        if (line[0] != ' ')
        {
            const size_t colonPos = line.find(':');
            if (colonPos == std::string::npos)
                continue;

            currentFrame = line.substr(0, colonPos);
            continue;
        }

        if (currentFrame.empty())
            continue;

        const std::string parentKey = "  parent: ";
        if (line.rfind(parentKey, 0) != 0)
            continue;

        std::string parentFrameValue = line.substr(parentKey.size());
        const size_t firstContent = parentFrameValue.find_first_not_of(" '\"");
        if (firstContent == std::string::npos)
        {
            parentByFrame[currentFrame] = "";
            continue;
        }
        parentFrameValue.erase(0, firstContent);

        const size_t lastContent = parentFrameValue.find_last_not_of(" '\"\r\n");
        if (lastContent == std::string::npos)
        {
            parentByFrame[currentFrame] = "";
            continue;
        }
        parentFrameValue.erase(lastContent + 1);
        parentByFrame[currentFrame] = parentFrameValue;
    }

    const auto baselinkIt = parentByFrame.find(baselinkFrame_);
    if (baselinkIt == parentByFrame.end())
    {
        baselinkPublishDecisionMade_ = true;
        baselinkPublishAllowed_ = true;
        return true;
    }

    const std::string parentFrame = baselinkIt->second;
    baselinkPublishDecisionMade_ = true;
    if (parentFrame.empty() || parentFrame == "NO_PARENT")
    {
        baselinkPublishAllowed_ = true;
        return true;
    }

    std::string topLevelParent = parentFrame;
    std::set<std::string> visitedFrames{baselinkFrame_};
    while (!topLevelParent.empty() && topLevelParent != "NO_PARENT")
    {
        if (!visitedFrames.insert(topLevelParent).second)
            break;

        const auto parentIt = parentByFrame.find(topLevelParent);
        if (parentIt == parentByFrame.end() || parentIt->second.empty() || parentIt->second == "NO_PARENT")
            break;

        topLevelParent = parentIt->second;
    }

    std::ostringstream ss;
    ss << "[BASELINK_FRAME_PARENT_EXISTS] (" << context << ") baselinkFrame='" << baselinkFrame_
       << "' already has TF parent '" << parentFrame << "'"
       << " top_level_parent='" << topLevelParent << "'"
         << ". Baselink TF and baselink odometry publications will be suppressed to avoid creating an invalid multi-parent TF tree; choose a base_link-style child frame without an existing parent.";
    logWarn(ss.str());
     baselinkPublishAllowed_ = false;
     return false;
}

bool RuntimeTfCoordinator::hasLidarToBaselinkTransform() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return hasLidar2Baselink_;
}

tf2::Transform RuntimeTfCoordinator::lidarToBaselinkTransform() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return tf2::Transform(lidar2Baselink_.getRotation(), lidar2Baselink_.getOrigin());
}