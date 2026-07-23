#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>

#include "utility.h"
#include "export/map_types.hpp"
#include "export/MapExporter.hpp"
#include "liorf/msg/complementary_odom_scale_debug.hpp"
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
#include "scanAlignment/IMappingBackend.hpp"
#include "degeneracyDetection/DegeneracyDetector.hpp"

#include <fstream>
#include <iomanip>
#include <sstream>
#include <cstdint>
#include <filesystem>
#include <unordered_map>
#include <limits>

enum class SCInputType
{
    SINGLE_SCAN_FULL,
    SINGLE_SCAN_FEAT,
    MULTI_SCAN_FEAT
};

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
    struct ComplementaryOdomMatchInfo
    {
        double lidar_prev_stamp_s = std::numeric_limits<double>::quiet_NaN();
        double lidar_curr_stamp_s = std::numeric_limits<double>::quiet_NaN();
        double odom_prev_stamp_s = std::numeric_limits<double>::quiet_NaN();
        double odom_curr_stamp_s = std::numeric_limits<double>::quiet_NaN();
        double dt_complementary_s = std::numeric_limits<double>::quiet_NaN();
        double prev_match_abs_dt_s = std::numeric_limits<double>::quiet_NaN();
        double curr_match_abs_dt_s = std::numeric_limits<double>::quiet_NaN();
        int odom_queue_size = 0;
        int odom_samples_between = 0;
    };

    MapExporter map_exporter_;
    std::shared_ptr<lio::IMappingBackend> mappingBackend;
    std::shared_ptr<DegeneracyDetector> degeneracyDetector;

    gtsam::NonlinearFactorGraph gtSAMgraph;
    gtsam::Values initialEstimate;
    gtsam::Values optimizedEstimate;
    gtsam::ISAM2 *isam;
    gtsam::Values isamCurrentEstimate;
    Eigen::MatrixXd poseCovariance;

    rclcpp::Subscription<liorf::msg::CloudInfo>::SharedPtr subCloud;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subGPS;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subLoop;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subComplementaryOdom;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudSurround;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubLaserOdometryGlobal;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubLaserOdometryIncremental;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubBaselinkOdometryGlobal;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubBaselinkOdometryIncremental;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubKeyPoses;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubHistoryKeyFrames;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubIcpKeyFrames;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLocalMapCloud;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubRegisteredCloud;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubCloudPreviousPose;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubCloudPredictedPose;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubKeyframeDeskewedDownsampled;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubKeyframeDeskewedDownsampledDebug;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubMatchedSurfFeatures;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubKdTreePlanePoints;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubKdTreePlaneNormals;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubKdTreePlaneResiduals;
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

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubDegeneracyRaw;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubDegeneracyPCA;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubDegeneracyBasis;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubDegeneracyPaths;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubDegeneracyPerturbedScan0;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubDegeneracyPerturbedScan1;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubDegeneracyPerturbedScan2;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubDegeneracyAlignedScan0;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubDegeneracyAlignedScan1;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubDegeneracyAlignedScan2;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pubDegeneracyPerturbedPose0;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pubDegeneracyPerturbedPose1;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pubDegeneracyPerturbedPose2;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pubDegeneracyAlignedPose0;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pubDegeneracyAlignedPose1;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pubDegeneracyAlignedPose2;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubDegeneracyDisplacements;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubDegeneracyOptimizationPaths;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubComplementaryOdomCorrectionDirection;
    rclcpp::Publisher<liorf::msg::ComplementaryOdomScaleDebug>::SharedPtr pubComplementaryOdomScaleDebug;

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

    bool require_map_rebuild = true;
    bool localMapDirty = true;
    bool kdtreeLocalMapDirty = true;
    double last_gps_rebuild_time = -1.0;

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

    int laserCloudSurfLastDSNum = 0;

    int temporal_filter_state = 0;

    bool aLoopIsClosed = false;
    map<int, int> loopIndexContainer;
    vector<pair<int, int>> loopIndexQueue;
    vector<gtsam::Pose3> loopPoseQueue;
    vector<gtsam::SharedNoiseModel> loopNoiseQueue;
    deque<std_msgs::msg::Float64MultiArray> loopInfoVec;

    nav_msgs::msg::Path globalPath;

    Eigen::Affine3f incrementalOdometryAffineFront;
    Eigen::Affine3f incrementalOdometryAffineBack;
    Eigen::Affine3f poseBeforePredictionLocal{Eigen::Affine3f::Identity()};
    Eigen::Affine3f poseAfterPredictionLocal{Eigen::Affine3f::Identity()};
    Eigen::Affine3f lastIncrementalDeltaPoseLocal{Eigen::Affine3f::Identity()};
    bool hasLastIncrementalDeltaPoseLocal{false};

    GeographicLib::LocalCartesian gps_trans_;

    SCManager scManager;

    std::unique_ptr<tf2_ros::TransformBroadcaster> br;

    mapOptimization(const rclcpp::NodeOptions &options);
    void allocateMemory();
    void laserCloudInfoHandler(const liorf::msg::CloudInfo::SharedPtr msgIn);

    void timerCallbackPublishOrigin();
    void initializeDatum(double lat, double lon, double alt, double heading_deg);
    void gpsHandler(const sensor_msgs::msg::NavSatFix::SharedPtr gpsMsg);

    // void pointAssociateToMap(PointType const *const pi, PointType *const po);
    pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose *transformIn);
    gtsam::Pose3 pclPointTogtsamPose3(PointTypePose thisPoint);
    gtsam::Pose3 trans2gtsamPose(float transformIn[]);
    Eigen::Affine3f pclPointToAffine3f(PointTypePose thisPoint) const;
    Eigen::Affine3f trans2Affine3f(float transformIn[]);
    PointTypePose trans2PointTypePose(float transformIn[]);
    nav_msgs::msg::Odometry odometryMsgFromAffine(const Eigen::Affine3f &affine, const rclcpp::Time &stamp, const std::string &frameId, const std::string &childFrameId);
    tf2::Transform tfFromAffine(const Eigen::Affine3f &affine) const;
    Eigen::Affine3f affineFromTf(const tf2::Transform &transform) const;

    void markMapRebuildTriggered(const std::string &reason);
    void logLocalMapStats(const std::string &stage);
    void manageLocalMap();
    void updateRollingMap();
    bool saveMapService(const std::shared_ptr<liorf::srv::SaveMap::Request> req, std::shared_ptr<liorf::srv::SaveMap::Response> res);
    void visualizeGlobalMapThread();
    void publishGlobalMap();
    void publishTwistMarkers(
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub,
        const std::string& ns,
        const std::vector<TwistVector>& twists,
        const rclcpp::Time& stamp,
        float r_trans, float g_trans, float b_trans,
        float r_rot, float g_rot, float b_rot);

    void publishDegeneracyPaths(
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub,
        const std::string& ns,
        const std::vector<TwistVector>& twists,
        const rclcpp::Time& stamp);
    void publishPerturbationDebugProducts(
        const std::vector<pcl::PointCloud<PointType>::Ptr>& perturbedScans,
        const std::vector<pcl::PointCloud<PointType>::Ptr>& alignedScans,
        const std::vector<Eigen::Matrix4f>& perturbedPoses,
        const std::vector<Eigen::Matrix4f>& alignedPoses,
        const std::vector<std::vector<Eigen::Matrix4f>>& optimizationPaths,
        const rclcpp::Time& stamp,
        const Eigen::Affine3f& poseBeforeReanchor,
        const Eigen::Affine3f& poseAfterReanchor);

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
    void downsampleCurrentScan();
    void scan2MapOptimization();
    void transformUpdate();
    float constraintTransformation(float value, float limit);
    [[nodiscard]] bool shouldSaveFrame() const;
    void addOdomFactor();
    void addGPSFactor();
    void addLoopFactor();
    bool saveKeyFramesAndFactor();
    void correctPoses();

    void updatePath(const PointTypePose &pose_in);
    void publishMapOptimizationTFs(const rclcpp::Time &stamp);
    void publishLidarGpsFix();
    void publishOdometry();
    void publishPredictionDebugClouds(const pcl::PointCloud<PointType>::Ptr &cloud);
    void publishKeyframeDeskewedDownsampled(const pcl::PointCloud<PointType>::Ptr &cloud);
    void publishKeyframeDeskewedDownsampledDebug(const pcl::PointCloud<PointType>::Ptr &cloud);
    void publishKdTreePlaneDebug();
    void publishFrames();

    // Degeneracy detection + complementary odometry compensation
    // (implemented in mapOptimization_degeneracy.cpp; visualization in
    // mapOptimization_publish.cpp).
    std::deque<nav_msgs::msg::Odometry> complementaryOdomQueue;
    std::mutex complementaryOdomMutex;

    std::deque<float> complementaryOdomScaleHistory;
    float complementaryOdomScaleHistorySum = 0.0f;
    std::deque<float> complementaryOdomFallbackScaleHistory;
    float complementaryOdomFallbackScaleHistorySum = 0.0f;

    bool complementaryOdomTfResolved = false;
    Eigen::Matrix4f T_complementary_to_lidar = Eigen::Matrix4f::Identity();

    void complementaryOdomHandler(const nav_msgs::msg::Odometry::SharedPtr msg);
    bool resolveComplementaryOdomExtrinsics(const std::string &msgFrameId);
    void publishComplementaryOdomDisplacementDebug(const Eigen::Affine3f &T_base_abs, const Eigen::Affine3f &T_raw_abs, const Eigen::Affine3f &T_proj_abs,
                                         bool hasNonDegenerateComponents = false,
                                         const Eigen::Vector3f &t_lidar_nondeg_map = Eigen::Vector3f::Zero(),
                                         const Eigen::Vector3f &t_complementary_nondeg_map = Eigen::Vector3f::Zero());
    void runDegeneracyDetectionAndCompensation();
    bool prepareDegeneracyOrthoBasis(std::vector<TwistVector> &orthoBasis);
    bool inferComplementaryOdomTwist(double dt_scan,
                                     TwistVector &xi_complementary_lidar,
                                     ComplementaryOdomMatchInfo *match_info = nullptr);
    float smoothComplementaryOdomScale(float scaleInstant);
    float smoothComplementaryOdomFallbackScale(float scaleInstant);
    Eigen::Affine3f buildScaledComplementaryPrediction(const Eigen::Affine3f &T_previous,
                                                       const Eigen::Affine3f &T_optimized,
                                                       const std::vector<TwistVector> &orthoBasis,
                                                       const TwistVector &xi_complementary_lidar,
                                                       const ComplementaryOdomMatchInfo &match_info,
                                                       double dt_scan,
                                                       bool &hasNonDegenerateComponents,
                                                       Eigen::Vector3f &t_lidar_nondeg_map,
                                                       Eigen::Vector3f &t_complementary_nondeg_map);
    Eigen::Affine3f projectDegenerateCorrection(const Eigen::Affine3f &T_optimized,
                                                const Eigen::Affine3f &T_predicted,
                                                const std::vector<TwistVector> &orthoBasis);
    void writeAffineToTransformTobeMapped(const Eigen::Affine3f &T_pose);
    void applyDegeneracyStateOverride(double dt_scan);
};
