#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>

#include "utility.h"
#include "export/map_types.hpp"
#include "export/MapExporter.hpp"
#include "liorf/msg/cloud_info.hpp"
#include "liorf/srv/save_map.hpp"
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <gtsam/nonlinear/ISAM2.h>

#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include <pcl/filters/statistical_outlier_removal.h>

#include "Scancontext.h"
#include "tictoc.h"
#include <fstream>
#include <iomanip>
#include <sstream>
#include <cstdint>
#include <filesystem>
#include <unordered_map>

enum class SCInputType
{
    SINGLE_SCAN_FULL,
    SINGLE_SCAN_FEAT,
    MULTI_SCAN_FEAT
};

struct VOXEL_LOC
{
    int64_t x;
    int64_t y;
    int64_t z;

    bool operator==(const VOXEL_LOC &other) const
    {
        return x == other.x && y == other.y && z == other.z;
    }
};

namespace std
{
template <>
struct hash<VOXEL_LOC>
{
    std::size_t operator()(const VOXEL_LOC &loc) const noexcept
    {
        return ((hash<int64_t>()(loc.x) ^ (hash<int64_t>()(loc.y) << 1)) >> 1) ^ (hash<int64_t>()(loc.z) << 1);
    }
};
} // namespace std

class FloatingAnchorFactor : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3>
{
private:
    gtsam::Point3 measured_gps_;
    gtsam::Point3 lever_arm_;

public:
    FloatingAnchorFactor(
        gtsam::Key T_EL_key,
        gtsam::Key x_t_key,
        const gtsam::Point3 &measured_gps,
        const gtsam::Point3 &lever_arm,
        gtsam::SharedNoiseModel model)
        : gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3>(model, T_EL_key, x_t_key),
          measured_gps_(measured_gps), lever_arm_(lever_arm)
    {
    }

    gtsam::Vector evaluateError(
        const gtsam::Pose3 &T_EL,
        const gtsam::Pose3 &x_t,
        boost::optional<gtsam::Matrix &> H1 = boost::none,
        boost::optional<gtsam::Matrix &> H2 = boost::none) const override
    {
        gtsam::Matrix36 H_xt_pt;
        gtsam::Point3 local_pt = x_t.transformFrom(lever_arm_, H2 ? &H_xt_pt : 0);

        gtsam::Matrix36 H_TGL_global;
        gtsam::Matrix33 H_local_global;
        gtsam::Point3 global_pt = T_EL.transformFrom(local_pt, H1 ? &H_TGL_global : 0, H2 ? &H_local_global : 0);

        if (H1)
            *H1 = H_TGL_global;
        if (H2)
            *H2 = H_local_global * H_xt_pt;

        return global_pt - measured_gps_;
    }
};

class mapOptimization : public ParamServer
{
public:
    MapExporter map_exporter_;

    gtsam::NonlinearFactorGraph gtSAMgraph;
    gtsam::Values initialEstimate;
    gtsam::Values optimizedEstimate;
    gtsam::ISAM2 *isam;
    gtsam::Values isamCurrentEstimate;
    Eigen::MatrixXd poseCovariance;

    rclcpp::Subscription<liorf::msg::CloudInfo>::SharedPtr subCloud;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subGPS;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subLoop;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudSurround;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubLaserOdometryGlobal;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubLaserOdometryIncremental;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubBaselinkOdometryGlobal;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubBaselinkOdometryIncremental;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubKeyPoses;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubHistoryKeyFrames;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubIcpKeyFrames;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubRecentKeyFrames;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubRecentKeyFrame;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubMatchedSurfFeatures;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubSurfDebugColored;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pubSurfDebugLegend;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubCloudRegisteredRaw;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubLoopConstraintEdge;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubGpsConstraintViz;
    rclcpp::Publisher<liorf::msg::CloudInfo>::SharedPtr pubSLAMInfo;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubGpsOdom;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pubGlobalOffset;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pubLidarGpsFix;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pubLidarGpsEnuPose;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pubLidarGpsNedPose;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubBaselinkGpsEnuOdometry;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubBaselinkGpsNedOdometry;

    const gtsam::Key T_EL_KEY = gtsam::Symbol('T', 0);
    bool T_EL_initialized = false;
    bool gtsam_anchor_inserted = false;
    bool forced_anchor_active = false;
    gtsam::Pose3 T_EL_estimate = gtsam::Pose3::Identity();
    size_t gpsFactorsAccepted = 0;
    bool T_EM_initialized = false;
    gtsam::Pose3 T_EM_estimate = gtsam::Pose3::Identity();
    Eigen::Affine3f poseAcumulatedIncremental = Eigen::Affine3f::Identity();
    Eigen::Affine3f odomToLidarAffine = Eigen::Affine3f::Identity();
    Eigen::Affine3f odomToBaseAffine = Eigen::Affine3f::Identity();
    Eigen::Affine3f mapLocalToOdomAffine = Eigen::Affine3f::Identity();
    bool mapLocalToOdomInitialized = false;
    bool lastIncreOdomPubFlag = false;
    nav_msgs::msg::Odometry laserOdomIncremental;

    double gpsAntennaOffsetX = 0.0;
    double gpsAntennaOffsetY = 0.0;
    double gpsAntennaOffsetZ = 0.0;

    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pubGpsOrigin;
    rclcpp::TimerBase::SharedPtr origin_publish_timer;

    std::mutex origin_mutex;
    sensor_msgs::msg::NavSatFix stored_origin_gps_msg;
    bool first_gps = true;

    bool gpsAnchorReady() const
    {
        return (T_EL_initialized && gpsFactorsAccepted >= 2) || forced_anchor_active;
    }

    rclcpp::Service<liorf::srv::SaveMap>::SharedPtr srvSaveMap;

    std::deque<nav_msgs::msg::Odometry> gpsQueue;
    std::deque<gtsam::Point3> gpsReceivedEnuQueue;
    std::deque<std::pair<int, std::pair<gtsam::Point3, double>>> gpsLidarAssociationQueue;
    std::vector<sensor_msgs::msg::NavSatFix> gpsHistory;
    std::vector<nav_msgs::msg::Odometry> densePoseHistory;
    std::mutex gpsHistoryMutex;
    std::mutex densePoseHistoryMutex;
    static constexpr size_t kMaxGpsVizPoints = 2000;
    liorf::msg::CloudInfo cloudInfo;

    vector<pcl::PointCloud<PointType>::Ptr> surfCloudKeyFrames;
    std::vector<uint8_t> keyframeScanAdmissible;

    pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D;
    pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D;
    pcl::PointCloud<PointType>::Ptr copy_cloudKeyPoses3D;
    pcl::PointCloud<PointTypePose>::Ptr copy_cloudKeyPoses6D;

    pcl::PointCloud<PointType>::Ptr laserCloudSurfLast;
    pcl::PointCloud<PointType>::Ptr laserCloudSurfLastDS;

    pcl::PointCloud<PointType>::Ptr laserCloudOri;
    pcl::PointCloud<PointType>::Ptr coeffSel;

    std::vector<PointType> laserCloudOriSurfVec;
    std::vector<PointType> coeffSelSurfVec;
    std::vector<bool> laserCloudOriSurfFlag;
    std::vector<uint8_t> laserCloudSurfKnnPassFlag;
    std::vector<uint8_t> laserCloudSurfPlaneValidFlag;
    std::vector<uint8_t> laserCloudSurfDebugCode;

    static constexpr uint8_t SURF_DEBUG_ACCEPTED = 0;
    static constexpr uint8_t SURF_DEBUG_REJECTED_NEIGHBOR_COUNT = 1;
    static constexpr uint8_t SURF_DEBUG_REJECTED_KNN_DISTANCE = 2;
    static constexpr uint8_t SURF_DEBUG_REJECTED_PLANE_INVALID = 3;
    static constexpr uint8_t SURF_DEBUG_REJECTED_LOW_WEIGHT = 4;
    static constexpr uint8_t SURF_DEBUG_NOT_OPTIMIZED = 5;

    uint32_t surfStageInputCount = 0;
    uint32_t surfStageKnnPassCount = 0;
    uint32_t surfStagePlaneValidCount = 0;
    uint32_t surfStageMatchedCount = 0;

    map<int, pair<pcl::PointCloud<PointType>, pcl::PointCloud<PointType>>> laserCloudMapContainer;
    pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMap;
    pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMapDS;
    std::unordered_map<VOXEL_LOC, PointType> voxelHashMap;
    bool require_map_rebuild = true;
    bool localMapDirty = true;
    bool kdtreeLocalMapDirty = true;
    double last_gps_rebuild_time = -1.0;

    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap;

    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurroundingKeyPoses;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeHistoryKeyPoses;

    pcl::VoxelGrid<PointType> downSizeFilterSurf;
    pcl::VoxelGrid<PointType> downSizeFilterLocalMapSurf;
    pcl::VoxelGrid<PointType> downSizeFilterICP;
    pcl::VoxelGrid<PointType> downSizeFilterSurroundingKeyPoses;

    rclcpp::Time timeLaserInfoStamp;
    double timeLastProcessing{-1};
    double timeLaserInfoCur{0};
    double newestCloudInfoStampSec{-1};
    double lastTimeDiff{0};
    double curTimeDiff{0};
    double lastOptimizedDeltaM{0};

    float transformTobeMapped[6];

    std::mutex mtx;
    std::mutex mtxLoopInfo;

    bool isDegenerate = false;
    cv::Mat matP;

    int laserCloudSurfFromMapDSNum = 0;
    int laserCloudSurfLastDSNum = 0;

    bool aLoopIsClosed = false;
    map<int, int> loopIndexContainer;
    vector<pair<int, int>> loopIndexQueue;
    vector<gtsam::Pose3> loopPoseQueue;
    vector<gtsam::SharedNoiseModel> loopNoiseQueue;
    deque<std_msgs::msg::Float64MultiArray> loopInfoVec;

    nav_msgs::msg::Path globalPath;

    Eigen::Affine3f transPointAssociateToMap;
    Eigen::Affine3f incrementalOdometryAffineFront;
    Eigen::Affine3f incrementalOdometryAffineBack;
    Eigen::Affine3f lastIncrementalDeltaPoseLocal{Eigen::Affine3f::Identity()};
    bool hasLastIncrementalDeltaPoseLocal{false};

    GeographicLib::LocalCartesian gps_trans_;

    SCManager scManager;

    std::unique_ptr<tf2_ros::TransformBroadcaster> br;
    std::shared_ptr<tf2_ros::Buffer> tfBuffer;
    std::shared_ptr<tf2_ros::TransformListener> tfListener;
    tf2::Stamped<tf2::Transform> lidar2Baselink;
    bool hasLidar2Baselink = false;
    double lastTfLookupAttemptWall = -1.0;
    const double tfLookupRetryPeriodSec = 1.0;

    mapOptimization(const rclcpp::NodeOptions &options);

    bool tryLookupLidarToBaselinkTf(const char *context);
    void allocateMemory();
    void laserCloudInfoHandler(const liorf::msg::CloudInfo::SharedPtr msgIn);

    void timerCallbackPublishOrigin();
    void initializeDatum(double lat, double lon, double alt, double heading_deg);
    void gpsHandler(const sensor_msgs::msg::NavSatFix::SharedPtr gpsMsg);

    void pointAssociateToMap(PointType const *const pi, PointType *const po);
    pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose *transformIn);
    gtsam::Pose3 pclPointTogtsamPose3(PointTypePose thisPoint);
    gtsam::Pose3 trans2gtsamPose(float transformIn[]);
    Eigen::Affine3f pclPointToAffine3f(PointTypePose thisPoint);
    Eigen::Affine3f trans2Affine3f(float transformIn[]);
    PointTypePose trans2PointTypePose(float transformIn[]);
    nav_msgs::msg::Odometry odometryMsgFromAffine(const Eigen::Affine3f &affine, const rclcpp::Time &stamp, const std::string &frameId, const std::string &childFrameId);
    tf2::Transform tfFromAffine(const Eigen::Affine3f &affine) const;
    Eigen::Affine3f affineFromTf(const tf2::Transform &transform) const;

    VOXEL_LOC voxelizePoint(const PointType &point, const float leafSize) const;
    void markMapRebuildTriggered(const std::string &reason);
    void logLocalMapStats(const std::string &stage);
    size_t pruneTransformedCloudCache();
    void manageLocalMap();
    void updateRollingMap();
    bool saveMapService(const std::shared_ptr<liorf::srv::SaveMap::Request> req, std::shared_ptr<liorf::srv::SaveMap::Response> res);
    void visualizeGlobalMapThread();
    void publishGlobalMap();

    void loopClosureThread();
    void loopInfoHandler(const std_msgs::msg::Float64MultiArray::SharedPtr loopMsg);
    void performRSLoopClosure();
    void performSCLoopClosure();
    bool detectLoopClosureDistance(int *latestID, int *closestID);
    bool detectLoopClosureExternal(int *latestID, int *closestID);
    void loopFindNearKeyframes(pcl::PointCloud<PointType>::Ptr &nearKeyframes, const int &key, const int &searchNum, const int &loop_index);
    void visualizeLoopClosure();

    void visualizeGpsConstraints();
    void updateInitialGuess();
    void extractForLoopClosure();
    void extractNearby();
    void extractCloud(pcl::PointCloud<PointType>::Ptr cloudToExtract);
    void extractSurroundingKeyFrames();
    void downsampleCurrentScan();
    void updatePointAssociateToMap();
    void surfOptimization();
    void combineOptimizationCoeffs();
    bool LMOptimization(int iterCount);
    void scan2MapOptimization();
    void transformUpdate();
    float constraintTransformation(float value, float limit);
    bool saveFrame();
    void addOdomFactor();
    void addGPSFactor();
    void addLoopFactor();
    bool saveKeyFramesAndFactor();
    void correctPoses();

    void updatePath(const PointTypePose &pose_in);
    void publishMapOptimizationTFs(const rclcpp::Time &stamp);
    void publishLidarGpsFix();
    void publishOdometry();
    void publishFrames();
};
