#include <gtsam/nonlinear/NonlinearFactor.h>

#include "utility.h"
#include "liorf/msg/cloud_info.hpp"
#include "liorf/srv/save_map.hpp"
// <!-- liorf_yjz_lucky_boy -->
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


using namespace gtsam;

using symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
using symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)
using symbol_shorthand::G; // GPS pose

/*
    * A point cloud type that has 6D pose info ([x,y,z,roll,pitch,yaw] intensity is time stamp)
    */
struct PointXYZIRPYT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;                  // preferred way of adding a XYZ+padding
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRPYT,
                                   (float, x, x) (float, y, y)
                                   (float, z, z) (float, intensity, intensity)
                                   (float, roll, roll) (float, pitch, pitch) (float, yaw, yaw)
                                   (double, time, time))

typedef PointXYZIRPYT  PointTypePose;

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

class FloatingAnchorFactor : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3> {
private:
    gtsam::Point3 measured_gps_;
    gtsam::Point3 lever_arm_;

public:
    FloatingAnchorFactor(gtsam::Key T_GL_key, gtsam::Key x_t_key, const gtsam::Point3& measured_gps,
                         const gtsam::Point3& lever_arm, gtsam::SharedNoiseModel model)
        : gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3>(model, T_GL_key, x_t_key),
          measured_gps_(measured_gps), lever_arm_(lever_arm) {}

    gtsam::Vector evaluateError(const gtsam::Pose3& T_GL, const gtsam::Pose3& x_t,
                                boost::optional<gtsam::Matrix&> H1 = boost::none,
                                boost::optional<gtsam::Matrix&> H2 = boost::none) const override {
        // 1. Transform lever arm to local odometry frame
        gtsam::Matrix36 H_xt_pt;
        gtsam::Point3 local_pt = x_t.transformFrom(lever_arm_, H2 ? &H_xt_pt : 0);

        // 2. Transform local point to global GPS frame
        gtsam::Matrix36 H_TGL_global;
        gtsam::Matrix33 H_local_global;
        gtsam::Point3 global_pt = T_GL.transformFrom(local_pt, H1 ? &H_TGL_global : 0, H2 ? &H_local_global : 0);

        // 3. Assemble Jacobians using chain rule
        if (H1) *H1 = H_TGL_global;
        if (H2) *H2 = H_local_global * H_xt_pt;

        // 4. Return error vector
        return global_pt - measured_gps_;
    }
};

class mapOptimization : public ParamServer
{

public:
    // gtsam
    NonlinearFactorGraph gtSAMgraph;
    Values initialEstimate;
    Values optimizedEstimate;
    ISAM2 *isam;
    Values isamCurrentEstimate;
    Eigen::MatrixXd poseCovariance;

    rclcpp::Subscription<liorf::msg::CloudInfo>::SharedPtr subCloud;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subGPS;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subLoop;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudSurround;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubLaserOdometryGlobal;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubLaserOdometryIncremental;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubKeyPoses;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubHistoryKeyFrames;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubIcpKeyFrames;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubRecentKeyFrames;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubRecentKeyFrame;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubCloudRegisteredRaw;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubLoopConstraintEdge;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubGpsConstraintViz;
    rclcpp::Publisher<liorf::msg::CloudInfo>::SharedPtr pubSLAMInfo;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubGpsOdom;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pubGlobalOffset;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pubLidarGpsFix;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pubLidarGpsEnuPose;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pubLidarGpsNedPose;

    // Floating Anchor GPS Fusion Variables
    const gtsam::Key T_GL_KEY = gtsam::Symbol('T', 0);
    bool T_GL_initialized = false;
    gtsam::Pose3 T_GL_estimate = gtsam::Pose3::Identity();
    size_t gpsFactorsAccepted = 0;
    bool T_EM_initialized = false;
    gtsam::Pose3 T_EM_estimate = gtsam::Pose3::Identity();
    Eigen::Affine3f odomToBaseAffine = Eigen::Affine3f::Identity();
    Eigen::Affine3f mapLocalToOdomAffine = Eigen::Affine3f::Identity();
    bool mapLocalToOdomInitialized = false;
    
    // GPS Antenna Lever Arm (Offset from tracking frame)
    double gpsAntennaOffsetX = 0.0;
    double gpsAntennaOffsetY = 0.0;
    double gpsAntennaOffsetZ = 0.0;

    // Add the new publisher, timer, and state variables here
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pubGpsOrigin;
    rclcpp::TimerBase::SharedPtr origin_publish_timer;
    
    std::mutex origin_mutex;
    sensor_msgs::msg::NavSatFix stored_origin_gps_msg;
    bool first_gps = true;

    bool gpsAnchorReady() const
    {
        return T_GL_initialized && gpsFactorsAccepted >= 2;
    }

    rclcpp::Service<liorf::srv::SaveMap>::SharedPtr srvSaveMap;

    std::deque<nav_msgs::msg::Odometry> gpsQueue;
    std::deque<gtsam::Point3> gpsReceivedEnuQueue;
    std::deque<std::pair<int, std::pair<gtsam::Point3, double>>> gpsLidarAssociationQueue;
    static constexpr size_t kMaxGpsVizPoints = 2000;
    static constexpr int kGpsKeyframeSearchWindow = 10;
    static constexpr double kMaxGpsLidarConstraintDtSec = 0.30;
    liorf::msg::CloudInfo cloudInfo;

    vector<pcl::PointCloud<PointType>::Ptr> surfCloudKeyFrames;
    
    pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D;
    pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D;
    pcl::PointCloud<PointType>::Ptr copy_cloudKeyPoses3D;
    pcl::PointCloud<PointTypePose>::Ptr copy_cloudKeyPoses6D;

    pcl::PointCloud<PointType>::Ptr laserCloudSurfLast; // surf feature set from odoOptimization
    pcl::PointCloud<PointType>::Ptr laserCloudSurfLastDS; // downsampled surf feature set from odoOptimization

    pcl::PointCloud<PointType>::Ptr laserCloudOri;
    pcl::PointCloud<PointType>::Ptr coeffSel;

    std::vector<PointType> laserCloudOriSurfVec; // surf point holder for parallel computation
    std::vector<PointType> coeffSelSurfVec;
    std::vector<bool> laserCloudOriSurfFlag;

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
    pcl::VoxelGrid<PointType> downSizeFilterSurroundingKeyPoses; // for surrounding key poses of scan-to-map optimization
    
    rclcpp::Time timeLaserInfoStamp;
    double timeLastProcessing{-1};
    double timeLaserInfoCur{0};
    double newestCloudInfoStampSec{-1};
    double lastTimeDiff{0};
    double curTimeDiff{0};

    float transformTobeMapped[6];

    std::mutex mtx;
    std::mutex mtxLoopInfo;

    bool isDegenerate = false;
    cv::Mat matP;

    int laserCloudSurfFromMapDSNum = 0;
    int laserCloudSurfLastDSNum = 0;

    bool aLoopIsClosed = false;
    map<int, int> loopIndexContainer; // from new to old
    vector<pair<int, int>> loopIndexQueue;
    vector<gtsam::Pose3> loopPoseQueue;
    // vector<gtsam::noiseModel::Diagonal::shared_ptr> loopNoiseQueue;
    vector<gtsam::SharedNoiseModel> loopNoiseQueue;
    deque<std_msgs::msg::Float64MultiArray> loopInfoVec;

    nav_msgs::msg::Path globalPath;

    Eigen::Affine3f transPointAssociateToMap;
    Eigen::Affine3f incrementalOdometryAffineFront;
    Eigen::Affine3f incrementalOdometryAffineBack;
    Eigen::Affine3f lastLidarOdometryIncrement;

    GeographicLib::LocalCartesian gps_trans_;

    // scancontext loop closure
    SCManager scManager;

    std::unique_ptr<tf2_ros::TransformBroadcaster> br;
    std::shared_ptr<tf2_ros::Buffer> tfBuffer;
    std::shared_ptr<tf2_ros::TransformListener> tfListener;
    tf2::Stamped<tf2::Transform> lidar2Baselink;
    bool hasLidar2Baselink = false;
    double lastTfLookupAttemptWall = -1.0;
    const double tfLookupRetryPeriodSec = 1.0;

    mapOptimization(const rclcpp::NodeOptions & options) : ParamServer("liorf_mapOptimization", options)
    {
        ISAM2Params parameters;
        parameters.relinearizeThreshold = 0.1;
        parameters.relinearizeSkip = 1;
        isam = new ISAM2(parameters);

        diagnostics = std::make_shared<LiorfDiagnostics>(
            this,
            QosPolicy(history_policy, reliability_policy),
            history_policy,
            reliability_policy);

        auto cloudInfoQos = QosPolicy(history_policy, reliability_policy);
        cloudInfoQos.keep_last(std::max(1, cloud_info_queue_depth));

        subCloud = create_subscription<liorf::msg::CloudInfo>("liorf/deskew/cloud_info", cloudInfoQos,
                    std::bind(&mapOptimization::laserCloudInfoHandler, this, std::placeholders::_1));
        subGPS = create_subscription<sensor_msgs::msg::NavSatFix>(gpsTopic, QosPolicy(history_policy, reliability_policy),
                    std::bind(&mapOptimization::gpsHandler, this, std::placeholders::_1));
        subLoop = create_subscription<std_msgs::msg::Float64MultiArray>("lio_loop/loop_closure_detection", QosPolicy(history_policy, reliability_policy),
                    std::bind(&mapOptimization::loopInfoHandler, this, std::placeholders::_1));

        pubKeyPoses = create_publisher<sensor_msgs::msg::PointCloud2>("liorf/mapping/trajectory", QosPolicy(history_policy, reliability_policy));
        pubLaserCloudSurround = create_publisher<sensor_msgs::msg::PointCloud2>("liorf/mapping/map_global", QosPolicy(history_policy, reliability_policy));
        pubLaserOdometryGlobal = create_publisher<nav_msgs::msg::Odometry>("liorf/mapping/odometry", QosPolicy(history_policy, reliability_policy));
        pubLaserOdometryIncremental = create_publisher<nav_msgs::msg::Odometry>("liorf/mapping/odometry_incremental", QosPolicy(history_policy, reliability_policy));
        pubPath = create_publisher<nav_msgs::msg::Path>("liorf/mapping/path", QosPolicy(history_policy, reliability_policy));
        pubHistoryKeyFrames = create_publisher<sensor_msgs::msg::PointCloud2>("liorf/mapping/icp_loop_closure_history_cloud", QosPolicy(history_policy, reliability_policy));
        pubIcpKeyFrames = create_publisher<sensor_msgs::msg::PointCloud2>("liorf/mapping/icp_loop_closure_corrected_cloud", QosPolicy(history_policy, reliability_policy));
        pubLoopConstraintEdge = create_publisher<visualization_msgs::msg::MarkerArray>("/liorf/mapping/loop_closure_constraints", QosPolicy(history_policy, reliability_policy));
        pubGpsConstraintViz = create_publisher<visualization_msgs::msg::MarkerArray>("/liorf/mapping/gps_constraints", QosPolicy(history_policy, reliability_policy));
        pubRecentKeyFrames = create_publisher<sensor_msgs::msg::PointCloud2>("liorf/mapping/map_local", QosPolicy(history_policy, reliability_policy));
        pubRecentKeyFrame = create_publisher<sensor_msgs::msg::PointCloud2>("liorf/mapping/cloud_registered", QosPolicy(history_policy, reliability_policy));
        pubCloudRegisteredRaw = create_publisher<sensor_msgs::msg::PointCloud2>("liorf/mapping/cloud_registered_raw", QosPolicy(history_policy, reliability_policy));
        pubSLAMInfo = create_publisher<liorf::msg::CloudInfo>("liorf/mapping/slam_info", QosPolicy(history_policy, reliability_policy));
        pubGpsOdom = create_publisher<nav_msgs::msg::Odometry>("liorf/mapping/gps_odom", QosPolicy(history_policy, reliability_policy));
        pubGlobalOffset = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("liorf/enu_to_local_offset", QosPolicy(history_policy, reliability_policy));
        pubLidarGpsFix = create_publisher<sensor_msgs::msg::NavSatFix>("liorf/mapping/lidar_gps_fix", QosPolicy(history_policy, reliability_policy));
        pubLidarGpsEnuPose = create_publisher<geometry_msgs::msg::PoseStamped>("liorf/mapping/lidar_gps_enu_pose", QosPolicy(history_policy, reliability_policy));
        pubLidarGpsNedPose = create_publisher<geometry_msgs::msg::PoseStamped>("liorf/mapping/lidar_gps_ned_pose", QosPolicy(history_policy, reliability_policy));

        pubGpsOrigin = create_publisher<sensor_msgs::msg::NavSatFix>("liorf/gps_origin", QosPolicy(history_policy, reliability_policy));
        origin_publish_timer = this->create_wall_timer(std::chrono::seconds(1), std::bind(&mapOptimization::timerCallbackPublishOrigin, this));

        srvSaveMap = create_service<liorf::srv::SaveMap>("liorf/save_map", 
                        std::bind(&mapOptimization::saveMapService, this, std::placeholders::_1, std::placeholders::_2 ));

        downSizeFilterSurf.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);
        downSizeFilterLocalMapSurf.setLeafSize(surroundingKeyframeMapLeafSize, surroundingKeyframeMapLeafSize, surroundingKeyframeMapLeafSize);
        downSizeFilterICP.setLeafSize(loopClosureICPSurfLeafSize, loopClosureICPSurfLeafSize, loopClosureICPSurfLeafSize);
        downSizeFilterSurroundingKeyPoses.setLeafSize(surroundingKeyframeDensity, surroundingKeyframeDensity, surroundingKeyframeDensity); // for surrounding key poses of scan-to-map optimization

        br = std::make_unique<tf2_ros::TransformBroadcaster>(this);
        tfBuffer = std::make_shared<tf2_ros::Buffer>(get_clock());
        tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);

        tf2::Transform identity;
        identity.setIdentity();
        lidar2Baselink.setData(identity);

        // Initialize lidar<->baselink transform relationship
        if (lidarFrame == baselinkFrame)
        {
            // Frames are identical: lidar2baselink is identity by definition
            hasLidar2Baselink = true;
            RCLCPP_INFO_STREAM(
                get_logger(),
                "[TF_INIT] lidarFrame == baselinkFrame ('" << lidarFrame << "'): "
                << "lidar2baselink is identity by definition, hasLidar2Baselink=true"
            );
        }
        else
        {
            // Frames differ: need to lookup the actual transform
            RCLCPP_INFO_STREAM(
                get_logger(),
                "[TF_INIT] lidarFrame != baselinkFrame ('" << lidarFrame << "' vs '" << baselinkFrame << "'): "
                << "attempting initial lookup"
            );
            tryLookupLidarToBaselinkTf("ctor");
        }

        allocateMemory();
    }

    bool tryLookupLidarToBaselinkTf(const char *context)
    {
        try
        {
            geometry_msgs::msg::TransformStamped lidar_to_base_msg =
                tfBuffer->lookupTransform(lidarFrame, baselinkFrame, rclcpp::Time(0));

            tf2::fromMsg(lidar_to_base_msg, lidar2Baselink);
            hasLidar2Baselink = true;

            const auto &tr = lidar_to_base_msg.transform.translation;
            const auto &qr = lidar_to_base_msg.transform.rotation;
            double roll, pitch, yaw;
            tf2::Quaternion q(qr.x, qr.y, qr.z, qr.w);
            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

            RCLCPP_INFO_STREAM(
                get_logger(),
                "[TF_LOOKUP_OK] (" << context << ") lookupTransform success target='" << lidarFrame
                << "' source='" << baselinkFrame << "'"
                << " stamp=" << std::fixed << std::setprecision(6) << ROS_TIME(lidar_to_base_msg.header.stamp)
                << " xyz=(" << tr.x << ", " << tr.y << ", " << tr.z << ")"
                << " quat_xyzw=(" << qr.x << ", " << qr.y << ", " << qr.z << ", " << qr.w << ")"
                << " rpy=(" << roll << ", " << pitch << ", " << yaw << ")"
            );

            return true;
        }
        catch (tf2::TransformException &ex)
        {
            hasLidar2Baselink = false;
            RCLCPP_WARN_STREAM(
                get_logger(),
                "[TF_LOOKUP_FAIL] (" << context << ") lookupTransform failed target='" << lidarFrame
                << "' source='" << baselinkFrame << "' reason=" << ex.what()
            );
            return false;
        }
    }

    void allocateMemory()
    {
        cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
        cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());
        copy_cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
        copy_cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());

        kdtreeSurroundingKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());
        kdtreeHistoryKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());

        laserCloudSurfLast.reset(new pcl::PointCloud<PointType>()); // surf feature set from odoOptimization
        laserCloudSurfLastDS.reset(new pcl::PointCloud<PointType>()); // downsampled surf featuer set from odoOptimization

        laserCloudOri.reset(new pcl::PointCloud<PointType>());
        coeffSel.reset(new pcl::PointCloud<PointType>());

        laserCloudOriSurfVec.resize(N_SCAN * Horizon_SCAN);
        coeffSelSurfVec.resize(N_SCAN * Horizon_SCAN);
        laserCloudOriSurfFlag.resize(N_SCAN * Horizon_SCAN);

        std::fill(laserCloudOriSurfFlag.begin(), laserCloudOriSurfFlag.end(), false);

        laserCloudSurfFromMap.reset(new pcl::PointCloud<PointType>());
        laserCloudSurfFromMapDS.reset(new pcl::PointCloud<PointType>());

        kdtreeSurfFromMap.reset(new pcl::KdTreeFLANN<PointType>());

        for (int i = 0; i < 6; ++i){
            transformTobeMapped[i] = 0;
        }

        matP = cv::Mat(6, 6, CV_32F, cv::Scalar::all(0));
    }

    void laserCloudInfoHandler(const liorf::msg::CloudInfo::SharedPtr msgIn)
    {
        // extract time stamp
        timeLaserInfoStamp = msgIn->header.stamp;
        timeLaserInfoCur = ROS_TIME(msgIn->header.stamp);
        if (timeLaserInfoCur > newestCloudInfoStampSec)
            newestCloudInfoStampSec = timeLaserInfoCur;

        if (drop_stale_lidar_frames && max_lidar_processing_lag_sec > 0.0)
        {
            const double backlogLagSec = newestCloudInfoStampSec - timeLaserInfoCur;
            if (backlogLagSec > max_lidar_processing_lag_sec)
            {
                if (diagnostics)
                {
                    std::ostringstream oss;
                    oss << "[LIDAR_FRAME_DROP] reason=stale"
                        << " lag_s=" << std::fixed << std::setprecision(3) << backlogLagSec
                        << " max_lag_s=" << max_lidar_processing_lag_sec
                        << " cloud_stamp_s=" << timeLaserInfoCur
                        << " newest_cloud_stamp_s=" << newestCloudInfoStampSec;
                    diagnostics->logEventThrottle("lidar_frame_drop_stale", 1.0, oss.str());
                }
                return;
            }
        }

        if (diagnostics)
            diagnostics->markLidarUpdate(timeLaserInfoStamp);

        // extract info and feature cloud
        cloudInfo = *msgIn;
        pcl::fromROSMsg(msgIn->cloud_deskewed, *laserCloudSurfLast);

        // TODO
        // ......
        // remapping
        // ......
        // END

        std::lock_guard<std::mutex> lock(mtx);

        curTimeDiff = timeLaserInfoCur - timeLastProcessing;
        if (curTimeDiff >= mappingProcessInterval)
        {
            timeLastProcessing = timeLaserInfoCur;

            TicToc t_updateInitialGuess;
            updateInitialGuess();
            if (diagnostics)
                diagnostics->recordSlice("updateInitialGuess", t_updateInitialGuess.toc());

            if (require_map_rebuild || localMapDirty)
            {
                TicToc t_manageLocalMap;
                manageLocalMap();
                if (diagnostics)
                    diagnostics->recordSlice("manageLocalMap", t_manageLocalMap.toc());
            }

            TicToc t_downsampleCurrentScan;
            downsampleCurrentScan();
            if (diagnostics)
                diagnostics->recordSlice("downsampleCurrentScan", t_downsampleCurrentScan.toc());

            TicToc t_scan2MapOptimization;
            scan2MapOptimization();
            if (diagnostics)
                diagnostics->recordSlice("scan2MapOptimization", t_scan2MapOptimization.toc());

            TicToc t_saveKeyFramesAndFactor;
            bool newKeyframeSaved = saveKeyFramesAndFactor();
            if (diagnostics)
                diagnostics->recordSlice("saveKeyFramesAndFactor", t_saveKeyFramesAndFactor.toc());

            TicToc t_correctPoses;
            correctPoses();
            if (diagnostics)
                diagnostics->recordSlice("correctPoses", t_correctPoses.toc());

            if (newKeyframeSaved)
            {
                localMapDirty = true;
                TicToc t_updateRollingMap;
                updateRollingMap();
                if (diagnostics)
                    diagnostics->recordSlice("updateRollingMap", t_updateRollingMap.toc());
            }

            publishOdometry();
            publishLidarGpsFix();

            publishFrames();
            visualizeGpsConstraints();

            timeLastProcessing = timeLaserInfoCur;
            lastTimeDiff = curTimeDiff;
        }

        // Always publish TF at LiDAR callback rate (latest optimized pose with current LiDAR stamp).
        publishMapOptimizationTFs(timeLaserInfoStamp);
    }

    void timerCallbackPublishOrigin()
    {
        std::lock_guard<std::mutex> lock(origin_mutex);
        if (!first_gps)
        {
            stored_origin_gps_msg.header.stamp = this->now();
            pubGpsOrigin->publish(stored_origin_gps_msg);
        }
    }

    void gpsHandler(const sensor_msgs::msg::NavSatFix::SharedPtr gpsMsg)
    {
        if (gpsMsg->status.status < 0)
            return;

        if (diagnostics)
            diagnostics->markGpsUpdate(gpsMsg->header.stamp);

        Eigen::Vector3d trans_local_;
        
        if (first_gps) {
            std::lock_guard<std::mutex> lock(origin_mutex);
            stored_origin_gps_msg = *gpsMsg;
            first_gps = false;
            
            gps_trans_.Reset(gpsMsg->latitude, gpsMsg->longitude, gpsMsg->altitude);

            GeographicLib::Geocentric earth = GeographicLib::Geocentric::WGS84();
            double ecef_x, ecef_y, ecef_z;
            earth.Forward(gpsMsg->latitude, gpsMsg->longitude, gpsMsg->altitude, ecef_x, ecef_y, ecef_z);

            const double lat_rad = gpsMsg->latitude * M_PI / 180.0;
            const double lon_rad = gpsMsg->longitude * M_PI / 180.0;
            const double sin_lat = std::sin(lat_rad);
            const double cos_lat = std::cos(lat_rad);
            const double sin_lon = std::sin(lon_rad);
            const double cos_lon = std::cos(lon_rad);

            Eigen::Matrix3d R_ecef_enu;
            R_ecef_enu.col(0) = Eigen::Vector3d(-sin_lon,  cos_lon, 0.0);
            R_ecef_enu.col(1) = Eigen::Vector3d(-sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat);
            R_ecef_enu.col(2) = Eigen::Vector3d( cos_lat * cos_lon,  cos_lat * sin_lon, sin_lat);

            T_EM_estimate = gtsam::Pose3(gtsam::Rot3(R_ecef_enu), gtsam::Point3(ecef_x, ecef_y, ecef_z));
            T_EM_initialized = true;
            RCLCPP_INFO(get_logger(), "GPS origin captured. Publishing will now begin.");
        }

        // Before anchor is fully observable, forward raw GPS fix only (no fused orientation topics).
        if (!gpsAnchorReady() && pubLidarGpsFix->get_subscription_count() != 0)
            pubLidarGpsFix->publish(*gpsMsg);

        gps_trans_.Forward(gpsMsg->latitude, gpsMsg->longitude, gpsMsg->altitude, trans_local_[0], trans_local_[1], trans_local_[2]);

        gpsReceivedEnuQueue.emplace_back(trans_local_[0], trans_local_[1], trans_local_[2]);
        if (gpsReceivedEnuQueue.size() > kMaxGpsVizPoints)
            gpsReceivedEnuQueue.pop_front();

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

    void pointAssociateToMap(PointType const * const pi, PointType * const po)
    {
        po->x = transPointAssociateToMap(0,0) * pi->x + transPointAssociateToMap(0,1) * pi->y + transPointAssociateToMap(0,2) * pi->z + transPointAssociateToMap(0,3);
        po->y = transPointAssociateToMap(1,0) * pi->x + transPointAssociateToMap(1,1) * pi->y + transPointAssociateToMap(1,2) * pi->z + transPointAssociateToMap(1,3);
        po->z = transPointAssociateToMap(2,0) * pi->x + transPointAssociateToMap(2,1) * pi->y + transPointAssociateToMap(2,2) * pi->z + transPointAssociateToMap(2,3);
        po->intensity = pi->intensity;
    }

    pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose* transformIn)
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

    gtsam::Pose3 pclPointTogtsamPose3(PointTypePose thisPoint)
    {
        return gtsam::Pose3(gtsam::Rot3::RzRyRx(double(thisPoint.roll), double(thisPoint.pitch), double(thisPoint.yaw)),
                                  gtsam::Point3(double(thisPoint.x),    double(thisPoint.y),     double(thisPoint.z)));
    }

    gtsam::Pose3 trans2gtsamPose(float transformIn[])
    {
        return gtsam::Pose3(gtsam::Rot3::RzRyRx(transformIn[0], transformIn[1], transformIn[2]), 
                                  gtsam::Point3(transformIn[3], transformIn[4], transformIn[5]));
    }

    Eigen::Affine3f pclPointToAffine3f(PointTypePose thisPoint)
    { 
        return pcl::getTransformation(thisPoint.x, thisPoint.y, thisPoint.z, thisPoint.roll, thisPoint.pitch, thisPoint.yaw);
    }

    Eigen::Affine3f trans2Affine3f(float transformIn[])
    {
        return pcl::getTransformation(transformIn[3], transformIn[4], transformIn[5], transformIn[0], transformIn[1], transformIn[2]);
    }

    PointTypePose trans2PointTypePose(float transformIn[])
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

    VOXEL_LOC voxelizePoint(const PointType &point, const float leafSize) const
    {
        const float safeLeaf = std::max(leafSize, 1e-3f);
        VOXEL_LOC voxel;
        voxel.x = static_cast<int64_t>(std::floor(point.x / safeLeaf));
        voxel.y = static_cast<int64_t>(std::floor(point.y / safeLeaf));
        voxel.z = static_cast<int64_t>(std::floor(point.z / safeLeaf));
        return voxel;
    }

    void markMapRebuildTriggered(const std::string &reason)
    {
        if (!require_map_rebuild)
            require_map_rebuild = true;

        localMapDirty = true;
        kdtreeLocalMapDirty = true;

        if (diagnostics)
        {
            std::ostringstream oss;
            oss << "[MAP_REBUILD_TRIGGER] reason=" << reason
                << " t=" << std::fixed << std::setprecision(3) << timeLaserInfoCur
                << " keyposes=" << cloudKeyPoses3D->size()
                << " voxel_count=" << voxelHashMap.size();
            diagnostics->logEvent(oss.str());
        }
    }

    void logLocalMapStats(const std::string &stage)
    {
        if (!diagnostics)
            return;

        std::ostringstream oss;
        oss << "[LOCAL_MAP_STATS] stage=" << stage
            << " t=" << std::fixed << std::setprecision(3) << timeLaserInfoCur
            << " rebuild_pending=" << (require_map_rebuild ? 1 : 0)
            << " voxels=" << voxelHashMap.size()
            << " local_map_pts=" << laserCloudSurfFromMapDS->size()
            << " cached_clouds=" << laserCloudMapContainer.size()
            << " current_scan_pts=" << laserCloudSurfLastDSNum
            << " keyposes=" << cloudKeyPoses3D->size()
            << " radius=" << surroundingKeyframeSearchRadius
            << " leaf=" << surroundingKeyframeMapLeafSize
            << " cache_max_age_s=" << transformed_cloud_cache_max_age_sec;

        diagnostics->logEventThrottle("local_map_stats", 1.0, oss.str());
    }

    size_t pruneTransformedCloudCache()
    {
        if (laserCloudMapContainer.empty())
            return 0;

        const double oldestAllowed = timeLaserInfoCur - transformed_cloud_cache_max_age_sec;
        size_t removed = 0;

        for (auto it = laserCloudMapContainer.begin(); it != laserCloudMapContainer.end();)
        {
            const int key = it->first;
            bool erase = false;

            if (key < 0 || key >= static_cast<int>(cloudKeyPoses6D->size()))
            {
                erase = true;
            }
            else if (cloudKeyPoses6D->points[key].time < oldestAllowed)
            {
                erase = true;
            }

            if (erase)
            {
                it = laserCloudMapContainer.erase(it);
                ++removed;
            }
            else
            {
                ++it;
            }
        }

        return removed;
    }

    void manageLocalMap()
    {
        if (cloudKeyPoses3D->points.empty())
        {
            laserCloudSurfFromMapDS->clear();
            laserCloudSurfFromMapDSNum = 0;
            return;
        }

        if (!require_map_rebuild && !localMapDirty)
        {
            logLocalMapStats("manageLocalMap_reuse");
            return;
        }

        if (require_map_rebuild)
        {
            const size_t prev_voxel_count = voxelHashMap.size();

            TicToc t_rebuild_extract;
            voxelHashMap.clear();
            extractSurroundingKeyFrames();
            if (diagnostics)
                diagnostics->recordSlice("manageLocalMap.rebuild.extractSurroundingKeyFrames", t_rebuild_extract.toc());

            TicToc t_rebuild_insert;
            voxelHashMap.reserve(laserCloudSurfFromMapDS->size());

            for (const auto &pt : laserCloudSurfFromMapDS->points)
                voxelHashMap[voxelizePoint(pt, surroundingKeyframeMapLeafSize)] = pt;
            if (diagnostics)
                diagnostics->recordSlice("manageLocalMap.rebuild.hashInsert", t_rebuild_insert.toc());

            require_map_rebuild = false;

            if (diagnostics)
            {
                std::ostringstream oss;
                oss << "[MAP_REBUILD_DONE] mode=full"
                    << " t=" << std::fixed << std::setprecision(3) << timeLaserInfoCur
                    << " prev_voxels=" << prev_voxel_count
                    << " new_voxels=" << voxelHashMap.size()
                    << " local_map_pts=" << laserCloudSurfFromMapDS->size();
                diagnostics->logEvent(oss.str());
            }
        }
        else
        {
            const float cx = transformTobeMapped[3];
            const float cy = transformTobeMapped[4];
            const float cz = transformTobeMapped[5];
            const float radius2 = surroundingKeyframeSearchRadius * surroundingKeyframeSearchRadius;
            const size_t before_prune = voxelHashMap.size();

            TicToc t_incremental_prune;

            for (auto it = voxelHashMap.begin(); it != voxelHashMap.end();)
            {
                const auto &pt = it->second;
                const float dx = pt.x - cx;
                const float dy = pt.y - cy;
                const float dz = pt.z - cz;
                if ((dx * dx + dy * dy + dz * dz) > radius2)
                    it = voxelHashMap.erase(it);
                else
                    ++it;
            }
            if (diagnostics)
                diagnostics->recordSlice("manageLocalMap.incremental.prune", t_incremental_prune.toc());

            TicToc t_incremental_rebuildCloud;
            laserCloudSurfFromMapDS->clear();
            laserCloudSurfFromMapDS->reserve(voxelHashMap.size());
            for (const auto &entry : voxelHashMap)
                laserCloudSurfFromMapDS->push_back(entry.second);
            if (diagnostics)
                diagnostics->recordSlice("manageLocalMap.incremental.materializeCloud", t_incremental_rebuildCloud.toc());

            if (diagnostics)
            {
                std::ostringstream oss;
                oss << "[MAP_REBUILD_DONE] mode=incremental"
                    << " t=" << std::fixed << std::setprecision(3) << timeLaserInfoCur
                    << " voxels_before=" << before_prune
                    << " voxels_after=" << voxelHashMap.size()
                    << " local_map_pts=" << laserCloudSurfFromMapDS->size();
                diagnostics->logEventThrottle("map_incremental_update", 1.0, oss.str());
            }
        }

        localMapDirty = false;
        kdtreeLocalMapDirty = true;
        laserCloudSurfFromMapDSNum = laserCloudSurfFromMapDS->size();
        logLocalMapStats("manageLocalMap");
    }

    void updateRollingMap()
    {
        if (require_map_rebuild || laserCloudSurfLastDS->empty())
            return;

        PointTypePose poseForTransform = trans2PointTypePose(transformTobeMapped);
        TicToc t_transformCurrentScan;
        pcl::PointCloud<PointType>::Ptr transformedCurrentScan = transformPointCloud(laserCloudSurfLastDS, &poseForTransform);
        if (diagnostics)
            diagnostics->recordSlice("updateRollingMap.transformPointCloud", t_transformCurrentScan.toc());

        TicToc t_hashInsert;
        voxelHashMap.reserve(voxelHashMap.size() + transformedCurrentScan->size());

        for (const auto &pt : transformedCurrentScan->points)
        {
            VOXEL_LOC voxel = voxelizePoint(pt, surroundingKeyframeMapLeafSize);
            if (voxelHashMap.find(voxel) == voxelHashMap.end())
                voxelHashMap.emplace(voxel, pt);
        }
        if (diagnostics)
            diagnostics->recordSlice("updateRollingMap.hashInsert", t_hashInsert.toc());

        logLocalMapStats("updateRollingMap");
    }

    













    bool saveMapService(const std::shared_ptr<liorf::srv::SaveMap::Request> req,
                                std::shared_ptr<liorf::srv::SaveMap::Response> res)
    {
      // Resolve output directory.
      // savePCDDirectory convention: relative to HOME, stored with a leading "/" (e.g. "/Downloads/LOAM/").
      // req->destination semantics:
      //   starts with "/"  → absolute path, used as-is
      //   starts with "~/" → HOME-expanded
      //   otherwise        → relative to HOME
      const char* home_env = std::getenv("HOME");
      const std::string homeDir = (home_env != nullptr) ? home_env : "/tmp";
      namespace fs = std::filesystem;
      fs::path saveDir;
      if (req->destination.empty()) {
          saveDir = fs::path(homeDir) / fs::path(savePCDDirectory).relative_path();
      } else if (fs::path(req->destination).is_absolute()) {
          saveDir = req->destination;
      } else if (req->destination.rfind("~/", 0) == 0) {
          saveDir = fs::path(homeDir) / req->destination.substr(2);
      } else {
          saveDir = fs::path(homeDir) / req->destination;
      }
      saveDir = saveDir.lexically_normal();
      const std::string saveMapDirectory = saveDir.string();
    res->save_directory = saveMapDirectory;
    res->enu_map_saved = false;
    res->keyframes_used = static_cast<uint32_t>(cloudKeyPoses3D->size());
    res->surf_points_local = 0;
    res->surf_points_enu = 0;
    res->global_points_local = 0;
    res->global_points_enu = 0;
    res->message = "";

      cout << "****************************************************" << endl;
      cout << "Saving map to pcd files ..." << endl;
      cout << "Save destination: " << saveMapDirectory << endl;
      // Recreate output directory fresh (removes any previous run's artifacts).
      {
          std::error_code ec;
          fs::remove_all(saveDir, ec);
          fs::create_directories(saveDir, ec);
          if (ec) {
              RCLCPP_ERROR(get_logger(), "Failed to create save directory '%s': %s",
                           saveMapDirectory.c_str(), ec.message().c_str());
              res->success = false;
              res->message = std::string("failed to create save directory: ") + ec.message();
              return true;
          }
      }

      { // Lock guard scope for origin
          std::lock_guard<std::mutex> lock(origin_mutex);
          std::string metadata_file_path = saveMapDirectory + "/map_metadata.yaml";
          std::ofstream ofs(metadata_file_path);
          if (ofs.is_open())
          {
              ofs << std::fixed << std::setprecision(12);
              ofs << "global_datum:" << std::endl;
              
              if (!first_gps) {
                  ofs << "  latitude: " << stored_origin_gps_msg.latitude << std::endl;
                  ofs << "  longitude: " << stored_origin_gps_msg.longitude << std::endl;
                  ofs << "  altitude: " << stored_origin_gps_msg.altitude << std::endl;
              } else {
                  ofs << "  latitude: null\n  longitude: null\n  altitude: null" << std::endl;
              }

              ofs << "T_global_local:" << std::endl;
              if (T_GL_initialized) {
                  ofs << "  x: " << T_GL_estimate.translation().x() << std::endl;
                  ofs << "  y: " << T_GL_estimate.translation().y() << std::endl;
                  ofs << "  z: " << T_GL_estimate.translation().z() << std::endl;
                  ofs << "  roll: " << T_GL_estimate.rotation().roll() << std::endl;
                  ofs << "  pitch: " << T_GL_estimate.rotation().pitch() << std::endl;
                  ofs << "  yaw: " << T_GL_estimate.rotation().yaw() << std::endl;
              } else {
                  ofs << "  x: 0.0\n  y: 0.0\n  z: 0.0\n  roll: 0.0\n  pitch: 0.0\n  yaw: 0.0" << std::endl;
              }
              
              ofs.close();
              cout << "Map metadata (Datum + Transform) successfully saved to: " << metadata_file_path << endl;
          }
      }

      // save key frame transformations
      pcl::io::savePCDFileBinary(saveMapDirectory + "/trajectory_local.pcd", *cloudKeyPoses3D);
      pcl::io::savePCDFileBinary(saveMapDirectory + "/transformations_local.pcd", *cloudKeyPoses6D);
      // extract global point cloud map

      pcl::PointCloud<PointType>::Ptr globalSurfCloud(new pcl::PointCloud<PointType>());
      pcl::PointCloud<PointType>::Ptr globalSurfCloudDS(new pcl::PointCloud<PointType>());
      pcl::PointCloud<PointType>::Ptr globalMapCloud(new pcl::PointCloud<PointType>());
      for (int i = 0; i < (int)cloudKeyPoses3D->size(); i++) {
          *globalSurfCloud   += *transformPointCloud(surfCloudKeyFrames[i],    &cloudKeyPoses6D->points[i]);
          cout << "\r" << std::flush << "Processing feature cloud " << i << " of " << cloudKeyPoses6D->size() << " ...";
      }

      if(req->resolution != 0)
      {
        cout << "\n\nSave resolution: " << req->resolution << endl;
        // down-sample and save surf cloud
        downSizeFilterSurf.setInputCloud(globalSurfCloud);
        downSizeFilterSurf.setLeafSize(req->resolution, req->resolution, req->resolution);
        downSizeFilterSurf.filter(*globalSurfCloudDS);
        pcl::io::savePCDFileBinary(saveMapDirectory + "/SurfMap_local.pcd", *globalSurfCloudDS);
      }
      else
      {

        // save surf cloud
        pcl::io::savePCDFileBinary(saveMapDirectory + "/SurfMap_local.pcd", *globalSurfCloud);
      }

      // save global point cloud map
      *globalMapCloud += *globalSurfCloud;
    res->surf_points_local = static_cast<uint32_t>(globalSurfCloud->size());
    res->global_points_local = static_cast<uint32_t>(globalMapCloud->size());

      int ret = pcl::io::savePCDFileBinary(saveMapDirectory + "/GlobalMap_local.pcd", *globalMapCloud);

            // Save ENU-frame map products next to local-frame outputs when anchor is available.
            if (T_GL_initialized)
            {
                Eigen::Affine3f tGlobalLocalToEnu = pcl::getTransformation(
                        static_cast<float>(T_GL_estimate.translation().x()),
                        static_cast<float>(T_GL_estimate.translation().y()),
                        static_cast<float>(T_GL_estimate.translation().z()),
                        static_cast<float>(T_GL_estimate.rotation().roll()),
                        static_cast<float>(T_GL_estimate.rotation().pitch()),
                        static_cast<float>(T_GL_estimate.rotation().yaw()));

                pcl::PointCloud<PointType>::Ptr globalSurfCloudEnu(new pcl::PointCloud<PointType>());
                pcl::PointCloud<PointType>::Ptr globalMapCloudEnu(new pcl::PointCloud<PointType>());
                pcl::transformPointCloud(*globalSurfCloud, *globalSurfCloudEnu, tGlobalLocalToEnu);
                *globalMapCloudEnu += *globalSurfCloudEnu;
                res->enu_map_saved = true;
                res->surf_points_enu = static_cast<uint32_t>(globalSurfCloudEnu->size());
                res->global_points_enu = static_cast<uint32_t>(globalMapCloudEnu->size());

                pcl::io::savePCDFileBinary(saveMapDirectory + "/SurfMap_ENU.pcd", *globalSurfCloudEnu);
                pcl::io::savePCDFileBinary(saveMapDirectory + "/GlobalMap_ENU.pcd", *globalMapCloudEnu);

                pcl::PointCloud<PointType>::Ptr trajectoryEnu(new pcl::PointCloud<PointType>());
                trajectoryEnu->reserve(cloudKeyPoses3D->size());
                for (const auto &pt_local : cloudKeyPoses3D->points)
                {
                        gtsam::Point3 pt_enu = T_GL_estimate.transformFrom(gtsam::Point3(pt_local.x, pt_local.y, pt_local.z));
                        PointType out_pt;
                        out_pt.x = static_cast<float>(pt_enu.x());
                        out_pt.y = static_cast<float>(pt_enu.y());
                        out_pt.z = static_cast<float>(pt_enu.z());
                        out_pt.intensity = pt_local.intensity;
                        trajectoryEnu->push_back(out_pt);
                }
                pcl::io::savePCDFileBinary(saveMapDirectory + "/trajectory_ENU.pcd", *trajectoryEnu);
            }
            else
            {
                RCLCPP_WARN(get_logger(),
                                        "ENU map export skipped: T_global_local is not initialized yet.");
                res->message = "ENU map export skipped: T_global_local is not initialized yet.";
            }

      res->success = ret == 0;
      if (res->success)
      {
          const fs::path lastSavedMapPathFile = fs::path(homeDir) / ".liorf_last_saved_map_path";
          std::ofstream lastPathOut(lastSavedMapPathFile.string(), std::ios::trunc);
          if (lastPathOut.is_open())
          {
              lastPathOut << saveMapDirectory << std::endl;
              lastPathOut.close();
          }
          else
          {
              RCLCPP_WARN(get_logger(), "Failed to write last saved map path file: %s",
                          lastSavedMapPathFile.string().c_str());
          }
      }

      if (res->success && res->message.empty())
          res->message = "map saved successfully";
      if (!res->success && res->message.empty())
          res->message = "failed to save GlobalMap_local.pcd";

      downSizeFilterSurf.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);

      cout << "****************************************************" << endl;
      cout << "Saving map to pcd files completed\n" << endl;

      return true;
    }

    void visualizeGlobalMapThread()
    {
        rclcpp::Rate rate(0.2);
        while (rclcpp::ok()){
            rate.sleep();
            publishGlobalMap();
        }

        if (savePCD == false)
            return;

        std::shared_ptr<liorf::srv::SaveMap::Request> req = std::make_unique<liorf::srv::SaveMap::Request>();
        std::shared_ptr<liorf::srv::SaveMap::Response> res = std::make_unique<liorf::srv::SaveMap::Response>();

        if(!saveMapService(req, res)){
            cout << "Fail to save map" << endl;
        }
    }

    void publishGlobalMap()
    {
        if (pubLaserCloudSurround->get_subscription_count() == 0)
            return;

        if (cloudKeyPoses3D->points.empty() == true)
            return;

        pcl::KdTreeFLANN<PointType>::Ptr kdtreeGlobalMap(new pcl::KdTreeFLANN<PointType>());;
        pcl::PointCloud<PointType>::Ptr globalMapKeyPoses(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalMapKeyPosesDS(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalMapKeyFrames(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalMapKeyFramesDS(new pcl::PointCloud<PointType>());

        // kd-tree to find near key frames to visualize
        std::vector<int> pointSearchIndGlobalMap;
        std::vector<float> pointSearchSqDisGlobalMap;
        // search near key frames to visualize
        mtx.lock();
        kdtreeGlobalMap->setInputCloud(cloudKeyPoses3D);
        kdtreeGlobalMap->radiusSearch(cloudKeyPoses3D->back(), globalMapVisualizationSearchRadius, pointSearchIndGlobalMap, pointSearchSqDisGlobalMap, 0);
        mtx.unlock();

        for (int i = 0; i < (int)pointSearchIndGlobalMap.size(); ++i)
            globalMapKeyPoses->push_back(cloudKeyPoses3D->points[pointSearchIndGlobalMap[i]]);
        // downsample near selected key frames
        pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyPoses; // for global map visualization
        downSizeFilterGlobalMapKeyPoses.setLeafSize(globalMapVisualizationPoseDensity, globalMapVisualizationPoseDensity, globalMapVisualizationPoseDensity); // for global map visualization
        downSizeFilterGlobalMapKeyPoses.setInputCloud(globalMapKeyPoses);
        downSizeFilterGlobalMapKeyPoses.filter(*globalMapKeyPosesDS);
        for(auto& pt : globalMapKeyPosesDS->points)
        {
            kdtreeGlobalMap->nearestKSearch(pt, 1, pointSearchIndGlobalMap, pointSearchSqDisGlobalMap);
            pt.intensity = cloudKeyPoses3D->points[pointSearchIndGlobalMap[0]].intensity;
        }

        // extract visualized and downsampled key frames
        for (int i = 0; i < (int)globalMapKeyPosesDS->size(); ++i){
            if (common_lib_->pointDistance(globalMapKeyPosesDS->points[i], cloudKeyPoses3D->back()) > globalMapVisualizationSearchRadius)
                continue;
            int thisKeyInd = (int)globalMapKeyPosesDS->points[i].intensity;
            *globalMapKeyFrames += *transformPointCloud(surfCloudKeyFrames[thisKeyInd],    &cloudKeyPoses6D->points[thisKeyInd]);
        }
        // downsample visualized points
        pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyFrames; // for global map visualization
        downSizeFilterGlobalMapKeyFrames.setLeafSize(globalMapVisualizationLeafSize, globalMapVisualizationLeafSize, globalMapVisualizationLeafSize); // for global map visualization
        downSizeFilterGlobalMapKeyFrames.setInputCloud(globalMapKeyFrames);
        downSizeFilterGlobalMapKeyFrames.filter(*globalMapKeyFramesDS);
        publishCloud(pubLaserCloudSurround, globalMapKeyFramesDS, timeLaserInfoStamp, odometryFrame);
    }












    void loopClosureThread()
    {
        if (loopClosureEnableFlag == false)
            return;

        rclcpp::Rate rate(loopClosureFrequency);
        while (rclcpp::ok())
        {
            rate.sleep();
            performRSLoopClosure();
            performSCLoopClosure();
            visualizeLoopClosure();
        }
    }

    void loopInfoHandler(const std_msgs::msg::Float64MultiArray::SharedPtr loopMsg)
    {
        std::lock_guard<std::mutex> lock(mtxLoopInfo);
        if (loopMsg->data.size() != 2)
            return;

        loopInfoVec.push_back(*loopMsg);

        while (loopInfoVec.size() > 5)
            loopInfoVec.pop_front();
    }

    void performRSLoopClosure()
    {
        if (cloudKeyPoses3D->points.empty() == true)
            return;

        mtx.lock();
        *copy_cloudKeyPoses3D = *cloudKeyPoses3D;
        *copy_cloudKeyPoses6D = *cloudKeyPoses6D;
        mtx.unlock();

        // find keys
        int loopKeyCur;
        int loopKeyPre;
        if (detectLoopClosureExternal(&loopKeyCur, &loopKeyPre) == false)
            if (detectLoopClosureDistance(&loopKeyCur, &loopKeyPre) == false)
                return;

        // extract cloud
        pcl::PointCloud<PointType>::Ptr cureKeyframeCloud(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr prevKeyframeCloud(new pcl::PointCloud<PointType>());
        {
            loopFindNearKeyframes(cureKeyframeCloud, loopKeyCur, 0, -1);
            loopFindNearKeyframes(prevKeyframeCloud, loopKeyPre, historyKeyframeSearchNum, -1);
            if (cureKeyframeCloud->size() < 300 || prevKeyframeCloud->size() < 1000)
                return;
            if (pubHistoryKeyFrames->get_subscription_count() != 0)
                publishCloud(pubHistoryKeyFrames, prevKeyframeCloud, timeLaserInfoStamp, odometryFrame);
        }

        // ICP Settings
        pcl::IterativeClosestPoint<PointType, PointType> icp;
        icp.setMaxCorrespondenceDistance(historyKeyframeSearchRadius*2);
        icp.setMaximumIterations(100);
        icp.setTransformationEpsilon(1e-6);
        icp.setEuclideanFitnessEpsilon(1e-6);
        icp.setRANSACIterations(0);

        // Align clouds
        icp.setInputSource(cureKeyframeCloud);
        icp.setInputTarget(prevKeyframeCloud);
        pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
        icp.align(*unused_result);

        if (icp.hasConverged() == false || icp.getFitnessScore() > historyKeyframeFitnessScore)
            return;

        // publish corrected cloud
        if (pubIcpKeyFrames->get_subscription_count() != 0)
        {
            pcl::PointCloud<PointType>::Ptr closed_cloud(new pcl::PointCloud<PointType>());
            pcl::transformPointCloud(*cureKeyframeCloud, *closed_cloud, icp.getFinalTransformation());
            publishCloud(pubIcpKeyFrames, closed_cloud, timeLaserInfoStamp, odometryFrame);
        }

        // Get pose transformation
        float x, y, z, roll, pitch, yaw;
        Eigen::Affine3f correctionLidarFrame;
        correctionLidarFrame = icp.getFinalTransformation();
        // transform from world origin to wrong pose
        Eigen::Affine3f tWrong = pclPointToAffine3f(copy_cloudKeyPoses6D->points[loopKeyCur]);
        // transform from world origin to corrected pose
        Eigen::Affine3f tCorrect = correctionLidarFrame * tWrong;// pre-multiplying -> successive rotation about a fixed frame
        pcl::getTranslationAndEulerAngles (tCorrect, x, y, z, roll, pitch, yaw);
        gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
        gtsam::Pose3 poseTo = pclPointTogtsamPose3(copy_cloudKeyPoses6D->points[loopKeyPre]);
        gtsam::Vector Vector6(6);
        float noiseScore = icp.getFitnessScore();
        Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore, noiseScore;
        noiseModel::Diagonal::shared_ptr constraintNoise = noiseModel::Diagonal::Variances(Vector6);

        // Add pose constraint
        mtx.lock();
        loopIndexQueue.push_back(make_pair(loopKeyCur, loopKeyPre));
        loopPoseQueue.push_back(poseFrom.between(poseTo));
        loopNoiseQueue.push_back(constraintNoise);
        mtx.unlock();

        // add loop constriant
        loopIndexContainer[loopKeyCur] = loopKeyPre;
    }

    // copy from sc-lio-sam
    void performSCLoopClosure()
    {
        if (cloudKeyPoses3D->points.empty() == true)
            return;

        mtx.lock();
        *copy_cloudKeyPoses3D = *cloudKeyPoses3D;
        *copy_cloudKeyPoses6D = *cloudKeyPoses6D;
        mtx.unlock();

        // find keys
        // first: nn index, second: yaw diff 
        auto detectResult = scManager.detectLoopClosureID(); 
        int loopKeyCur    = copy_cloudKeyPoses3D->size() - 1;;
        int loopKeyPre    = detectResult.first;
        float yawDiffRad  = detectResult.second; // not use for v1 (because pcl icp withi initial somthing wrong...)
        if( loopKeyPre == -1)
            return;

        auto it = loopIndexContainer.find(loopKeyCur);
        if (it != loopIndexContainer.end())
            return;

        // std::cout << "SC loop found! between " << loopKeyCur << " and " << loopKeyPre << "." << std::endl; // giseop

        // extract cloud
        pcl::PointCloud<PointType>::Ptr cureKeyframeCloud(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr prevKeyframeCloud(new pcl::PointCloud<PointType>());
        {
            int base_key = 0;
            loopFindNearKeyframes(cureKeyframeCloud, loopKeyCur, 0, base_key); // giseop 
            loopFindNearKeyframes(prevKeyframeCloud, loopKeyPre, historyKeyframeSearchNum, base_key); // giseop 

            if (cureKeyframeCloud->size() < 300 || prevKeyframeCloud->size() < 1000)
                return;
            if (pubHistoryKeyFrames->get_subscription_count() != 0)
                publishCloud(pubHistoryKeyFrames, prevKeyframeCloud, timeLaserInfoStamp, odometryFrame);
        }

        // ICP Settings
        pcl::IterativeClosestPoint<PointType, PointType> icp;
        icp.setMaxCorrespondenceDistance(historyKeyframeSearchRadius*2);
        icp.setMaximumIterations(100);
        icp.setTransformationEpsilon(1e-6);
        icp.setEuclideanFitnessEpsilon(1e-6);
        icp.setRANSACIterations(0);

        // Align clouds
        icp.setInputSource(cureKeyframeCloud);
        icp.setInputTarget(prevKeyframeCloud);
        pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
        icp.align(*unused_result);

        if (icp.hasConverged() == false || icp.getFitnessScore() > historyKeyframeFitnessScore)
            return;

        // publish corrected cloud
        if (pubIcpKeyFrames->get_subscription_count() != 0)
        {
            pcl::PointCloud<PointType>::Ptr closed_cloud(new pcl::PointCloud<PointType>());
            pcl::transformPointCloud(*cureKeyframeCloud, *closed_cloud, icp.getFinalTransformation());
            publishCloud(pubIcpKeyFrames, closed_cloud, timeLaserInfoStamp, odometryFrame);
        }

        // Get pose transformation
        float x, y, z, roll, pitch, yaw;
        Eigen::Affine3f correctionLidarFrame;
        correctionLidarFrame = icp.getFinalTransformation();

        // // transform from world origin to wrong pose
        // Eigen::Affine3f tWrong = pclPointToAffine3f(copy_cloudKeyPoses6D->points[loopKeyCur]);
        // // transform from world origin to corrected pose
        // Eigen::Affine3f tCorrect = correctionLidarFrame * tWrong;// pre-multiplying -> successive rotation about a fixed frame
        // pcl::getTranslationAndEulerAngles (tCorrect, x, y, z, roll, pitch, yaw);
        // gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
        // gtsam::Pose3 poseTo = pclPointTogtsamPose3(copy_cloudKeyPoses6D->points[loopKeyPre]);

        // gtsam::Vector Vector6(6);
        // float noiseScore = icp.getFitnessScore();
        // Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore, noiseScore;
        // noiseModel::Diagonal::shared_ptr constraintNoise = noiseModel::Diagonal::Variances(Vector6);

        // giseop 
        pcl::getTranslationAndEulerAngles (correctionLidarFrame, x, y, z, roll, pitch, yaw);
        gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
        gtsam::Pose3 poseTo = Pose3(Rot3::RzRyRx(0.0, 0.0, 0.0), Point3(0.0, 0.0, 0.0));

        // giseop, robust kernel for a SC loop
        float robustNoiseScore = 0.5; // constant is ok...
        gtsam::Vector robustNoiseVector6(6); 
        robustNoiseVector6 << robustNoiseScore, robustNoiseScore, robustNoiseScore, robustNoiseScore, robustNoiseScore, robustNoiseScore;
        noiseModel::Base::shared_ptr robustConstraintNoise; 
        robustConstraintNoise = gtsam::noiseModel::Robust::Create(
            gtsam::noiseModel::mEstimator::Cauchy::Create(1), // optional: replacing Cauchy by DCS or GemanMcClure, but with a good front-end loop detector, Cauchy is empirically enough.
            gtsam::noiseModel::Diagonal::Variances(robustNoiseVector6)
        ); // - checked it works. but with robust kernel, map modification may be delayed (i.e,. requires more true-positive loop factors)

        // Add pose constraint
        mtx.lock();
        loopIndexQueue.push_back(make_pair(loopKeyCur, loopKeyPre));
        loopPoseQueue.push_back(poseFrom.between(poseTo));
        loopNoiseQueue.push_back(robustConstraintNoise);
        mtx.unlock();

        // add loop constriant
        loopIndexContainer[loopKeyCur] = loopKeyPre;
    }

    bool detectLoopClosureDistance(int *latestID, int *closestID)
    {
        int loopKeyCur = copy_cloudKeyPoses3D->size() - 1;
        int loopKeyPre = -1;

        // check loop constraint added before
        auto it = loopIndexContainer.find(loopKeyCur);
        if (it != loopIndexContainer.end())
            return false;

        // find the closest history key frame
        std::vector<int> pointSearchIndLoop;
        std::vector<float> pointSearchSqDisLoop;
        kdtreeHistoryKeyPoses->setInputCloud(copy_cloudKeyPoses3D);
        kdtreeHistoryKeyPoses->radiusSearch(copy_cloudKeyPoses3D->back(), historyKeyframeSearchRadius, pointSearchIndLoop, pointSearchSqDisLoop, 0);
        
        for (int i = 0; i < (int)pointSearchIndLoop.size(); ++i)
        {
            int id = pointSearchIndLoop[i];
            if (abs(copy_cloudKeyPoses6D->points[id].time - timeLaserInfoCur) > historyKeyframeSearchTimeDiff)
            {
                loopKeyPre = id;
                break;
            }
        }

        if (loopKeyPre == -1 || loopKeyCur == loopKeyPre)
            return false;

        *latestID = loopKeyCur;
        *closestID = loopKeyPre;

        return true;
    }

    bool detectLoopClosureExternal(int *latestID, int *closestID)
    {
        // this function is not used yet, please ignore it
        int loopKeyCur = -1;
        int loopKeyPre = -1;

        std::lock_guard<std::mutex> lock(mtxLoopInfo);
        if (loopInfoVec.empty())
            return false;

        double loopTimeCur = loopInfoVec.front().data[0];
        double loopTimePre = loopInfoVec.front().data[1];
        loopInfoVec.pop_front();

        if (abs(loopTimeCur - loopTimePre) < historyKeyframeSearchTimeDiff)
            return false;

        int cloudSize = copy_cloudKeyPoses6D->size();
        if (cloudSize < 2)
            return false;

        // latest key
        loopKeyCur = cloudSize - 1;
        for (int i = cloudSize - 1; i >= 0; --i)
        {
            if (copy_cloudKeyPoses6D->points[i].time >= loopTimeCur)
                loopKeyCur = round(copy_cloudKeyPoses6D->points[i].intensity);
            else
                break;
        }

        // previous key
        loopKeyPre = 0;
        for (int i = 0; i < cloudSize; ++i)
        {
            if (copy_cloudKeyPoses6D->points[i].time <= loopTimePre)
                loopKeyPre = round(copy_cloudKeyPoses6D->points[i].intensity);
            else
                break;
        }

        if (loopKeyCur == loopKeyPre)
            return false;

        auto it = loopIndexContainer.find(loopKeyCur);
        if (it != loopIndexContainer.end())
            return false;

        *latestID = loopKeyCur;
        *closestID = loopKeyPre;

        return true;
    }

    void loopFindNearKeyframes(pcl::PointCloud<PointType>::Ptr& nearKeyframes, const int& key, const int& searchNum, const int& loop_index)
    {
        // extract near keyframes
        nearKeyframes->clear();
        int cloudSize = copy_cloudKeyPoses6D->size();
        for (int i = -searchNum; i <= searchNum; ++i)
        {
            int keyNear = key + i;
            if (keyNear < 0 || keyNear >= cloudSize )
                continue;

            int select_loop_index = (loop_index != -1) ? loop_index : key + i;
            *nearKeyframes += *transformPointCloud(surfCloudKeyFrames[keyNear],   &copy_cloudKeyPoses6D->points[select_loop_index]);
        }

        if (nearKeyframes->empty())
            return;

        // downsample near keyframes
        pcl::PointCloud<PointType>::Ptr cloud_temp(new pcl::PointCloud<PointType>());
        downSizeFilterICP.setInputCloud(nearKeyframes);
        downSizeFilterICP.filter(*cloud_temp);
        *nearKeyframes = *cloud_temp;
    }

    void visualizeLoopClosure()
    {
        if (loopIndexContainer.empty())
            return;
        
        visualization_msgs::msg::MarkerArray markerArray;
        // loop nodes
        visualization_msgs::msg::Marker markerNode;
        markerNode.header.frame_id = odometryFrame;
        markerNode.header.stamp = timeLaserInfoStamp;
        markerNode.action = visualization_msgs::msg::Marker::ADD;
        markerNode.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        markerNode.ns = "loop_nodes";
        markerNode.id = 0;
        markerNode.pose.orientation.w = 1;
        markerNode.scale.x = 0.3; markerNode.scale.y = 0.3; markerNode.scale.z = 0.3; 
        markerNode.color.r = 0; markerNode.color.g = 0.8; markerNode.color.b = 1;
        markerNode.color.a = 1;
        // loop edges
        visualization_msgs::msg::Marker markerEdge;
        markerEdge.header.frame_id = odometryFrame;
        markerEdge.header.stamp = timeLaserInfoStamp;
        markerEdge.action = visualization_msgs::msg::Marker::ADD;
        markerEdge.type = visualization_msgs::msg::Marker::LINE_LIST;
        markerEdge.ns = "loop_edges";
        markerEdge.id = 1;
        markerEdge.pose.orientation.w = 1;
        markerEdge.scale.x = 0.1;
        markerEdge.color.r = 0.9; markerEdge.color.g = 0.9; markerEdge.color.b = 0;
        markerEdge.color.a = 1;

        for (auto it = loopIndexContainer.begin(); it != loopIndexContainer.end(); ++it)
        {
            int key_cur = it->first;
            int key_pre = it->second;
            geometry_msgs::msg::Point p;
            p.x = copy_cloudKeyPoses6D->points[key_cur].x;
            p.y = copy_cloudKeyPoses6D->points[key_cur].y;
            p.z = copy_cloudKeyPoses6D->points[key_cur].z;
            markerNode.points.push_back(p);
            markerEdge.points.push_back(p);
            p.x = copy_cloudKeyPoses6D->points[key_pre].x;
            p.y = copy_cloudKeyPoses6D->points[key_pre].y;
            p.z = copy_cloudKeyPoses6D->points[key_pre].z;
            markerNode.points.push_back(p);
            markerEdge.points.push_back(p);
        }

        markerArray.markers.push_back(markerNode);
        markerArray.markers.push_back(markerEdge);
        pubLoopConstraintEdge->publish(markerArray);
    }

    void visualizeGpsConstraints()
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
                if (assoc_time_diff > kMaxGpsLidarConstraintDtSec)
                {
                    RCLCPP_WARN_THROTTLE(
                        get_logger(),
                        *get_clock(),
                        5000,
                    "Visualized GPS-LiDAR association exceeds dt threshold: %.3f s (gps=%.3f, keyframe=%.3f, max=%.3f)",
                        assoc_time_diff,
                        gps_time,
                    keyframe_time,
                    kMaxGpsLidarConstraintDtSec);
                }

                const auto &lidar_local = cloudKeyPoses6D->points[lidar_key];
                gtsam::Point3 p_lidar_enu = T_GL_estimate.transformFrom(gtsam::Point3(lidar_local.x, lidar_local.y, lidar_local.z));

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

    void updateInitialGuess()
    {
        // save current transformation before any processing
        incrementalOdometryAffineFront = trans2Affine3f(transformTobeMapped);

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
                    transIncre.translation() = lastLidarOdometryIncrement.translation() * curTimeDiff / lastTimeDiff;
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
                    transIncre.translation() = lastLidarOdometryIncrement.translation() * curTimeDiff / lastTimeDiff;
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

    void extractForLoopClosure()
    {
        pcl::PointCloud<PointType>::Ptr cloudToExtract(new pcl::PointCloud<PointType>());
        int numPoses = cloudKeyPoses3D->size();
        for (int i = numPoses-1; i >= 0; --i)
        {
            if ((int)cloudToExtract->size() <= surroundingKeyframeSize)
                cloudToExtract->push_back(cloudKeyPoses3D->points[i]);
            else
                break;
        }

        extractCloud(cloudToExtract);
    }

    void extractNearby()
    {
        pcl::PointCloud<PointType>::Ptr surroundingKeyPoses(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr surroundingKeyPosesDS(new pcl::PointCloud<PointType>());
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;

        // extract all the nearby key poses and downsample them
        TicToc t_extractNearby_radiusSearch;
        kdtreeSurroundingKeyPoses->setInputCloud(cloudKeyPoses3D); // create kd-tree
        kdtreeSurroundingKeyPoses->radiusSearch(cloudKeyPoses3D->back(), (double)surroundingKeyframeSearchRadius, pointSearchInd, pointSearchSqDis);
        if (diagnostics)
            diagnostics->recordSlice("extractNearby.radiusSearch", t_extractNearby_radiusSearch.toc());

        TicToc t_extractNearby_collectPoses;
        for (int i = 0; i < (int)pointSearchInd.size(); ++i)
        {
            int id = pointSearchInd[i];
            surroundingKeyPoses->push_back(cloudKeyPoses3D->points[id]);
        }
        if (diagnostics)
            diagnostics->recordSlice("extractNearby.collectPoses", t_extractNearby_collectPoses.toc());

        TicToc t_extractNearby_downsamplePoses;
        downSizeFilterSurroundingKeyPoses.setInputCloud(surroundingKeyPoses);
        downSizeFilterSurroundingKeyPoses.filter(*surroundingKeyPosesDS);
        if (diagnostics)
            diagnostics->recordSlice("extractNearby.downsamplePoses", t_extractNearby_downsamplePoses.toc());

        TicToc t_extractNearby_remapIndices;
        for(auto& pt : surroundingKeyPosesDS->points)
        {
            kdtreeSurroundingKeyPoses->nearestKSearch(pt, 1, pointSearchInd, pointSearchSqDis);
            pt.intensity = cloudKeyPoses3D->points[pointSearchInd[0]].intensity;
        }
        if (diagnostics)
            diagnostics->recordSlice("extractNearby.remapIndices", t_extractNearby_remapIndices.toc());

        // also extract some latest key frames in case the robot rotates in one position
        TicToc t_extractNearby_addRecent;
        int numPoses = cloudKeyPoses3D->size();
        for (int i = numPoses-1; i >= 0; --i)
        {
            if (timeLaserInfoCur - cloudKeyPoses6D->points[i].time < 10.0)
                surroundingKeyPosesDS->push_back(cloudKeyPoses3D->points[i]);
            else
                break;
        }
        if (diagnostics)
            diagnostics->recordSlice("extractNearby.addRecent", t_extractNearby_addRecent.toc());

        TicToc t_extractNearby_extractCloud;
        extractCloud(surroundingKeyPosesDS);
        if (diagnostics)
            diagnostics->recordSlice("extractNearby.extractCloud", t_extractNearby_extractCloud.toc());
    }

    void extractCloud(pcl::PointCloud<PointType>::Ptr cloudToExtract)
    {
        // fuse the map
        laserCloudSurfFromMap->clear(); 
        TicToc t_extractCloud_fuse;
        for (int i = 0; i < (int)cloudToExtract->size(); ++i)
        {
            if (common_lib_->pointDistance(cloudToExtract->points[i], cloudKeyPoses3D->back()) > surroundingKeyframeSearchRadius)
                continue;

            int thisKeyInd = (int)cloudToExtract->points[i].intensity;
            if (laserCloudMapContainer.find(thisKeyInd) != laserCloudMapContainer.end()) 
            {
                // transformed cloud available
                *laserCloudSurfFromMap   += laserCloudMapContainer[thisKeyInd].second;
            } else {
                // transformed cloud not available
                pcl::PointCloud<PointType> laserCloudCornerTemp;
                pcl::PointCloud<PointType> laserCloudSurfTemp = *transformPointCloud(surfCloudKeyFrames[thisKeyInd],    &cloudKeyPoses6D->points[thisKeyInd]);
                *laserCloudSurfFromMap   += laserCloudSurfTemp;
                laserCloudMapContainer[thisKeyInd] = make_pair(laserCloudCornerTemp, laserCloudSurfTemp);
            }
            
        }
        if (diagnostics)
            diagnostics->recordSlice("extractCloud.fuseAndTransform", t_extractCloud_fuse.toc());

        // Downsample the surrounding surf key frames (or map)
        TicToc t_extractCloud_downsample;
        downSizeFilterLocalMapSurf.setInputCloud(laserCloudSurfFromMap);
        downSizeFilterLocalMapSurf.filter(*laserCloudSurfFromMapDS);
        laserCloudSurfFromMapDSNum = laserCloudSurfFromMapDS->size();
        if (diagnostics)
            diagnostics->recordSlice("extractCloud.downsampleLocalMap", t_extractCloud_downsample.toc());

        // clear map cache if too large
        TicToc t_extractCloud_cacheMaintenance;
        const size_t removedCacheEntries = pruneTransformedCloudCache();
        if (diagnostics)
        {
            diagnostics->recordSlice("extractCloud.cacheMaintenance", t_extractCloud_cacheMaintenance.toc());

            if (removedCacheEntries > 0)
            {
                std::ostringstream oss;
                oss << "[CACHE_PRUNE] removed=" << removedCacheEntries
                    << " remaining=" << laserCloudMapContainer.size()
                    << " max_age_s=" << transformed_cloud_cache_max_age_sec
                    << " t=" << std::fixed << std::setprecision(3) << timeLaserInfoCur;
                diagnostics->logEventThrottle("cache_prune", 1.0, oss.str());
            }
        }
    }

    void extractSurroundingKeyFrames()
    {
        if (cloudKeyPoses3D->points.empty() == true)
            return; 
        
        // if (loopClosureEnableFlag == true)
        // {
        //     extractForLoopClosure();    
        // } else {
        //     extractNearby();
        // }

        TicToc t_extractSurroundingKeyFrames_extractNearby;
        extractNearby();
        if (diagnostics)
            diagnostics->recordSlice("extractSurroundingKeyFrames.extractNearby", t_extractSurroundingKeyFrames_extractNearby.toc());
    }

    void downsampleCurrentScan()
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

    void updatePointAssociateToMap()
    {
        transPointAssociateToMap = trans2Affine3f(transformTobeMapped);
    }

    void surfOptimization()
    {
        updatePointAssociateToMap();

        #pragma omp parallel for num_threads(numberOfCores)
        for (int i = 0; i < laserCloudSurfLastDSNum; i++)
        {
            PointType pointOri, pointSel, coeff;
            std::vector<int> pointSearchInd;
            std::vector<float> pointSearchSqDis;

            pointOri = laserCloudSurfLastDS->points[i];
            pointAssociateToMap(&pointOri, &pointSel); 
            kdtreeSurfFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

            Eigen::Matrix<float, 5, 3> matA0;
            Eigen::Matrix<float, 5, 1> matB0;
            Eigen::Vector3f matX0;

            matA0.setZero();
            matB0.fill(-1);
            matX0.setZero();

            if (pointSearchSqDis[4] < 1.0) {
                for (int j = 0; j < 5; j++) {
                    matA0(j, 0) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].x;
                    matA0(j, 1) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].y;
                    matA0(j, 2) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].z;
                }

                matX0 = matA0.colPivHouseholderQr().solve(matB0);

                float pa = matX0(0, 0);
                float pb = matX0(1, 0);
                float pc = matX0(2, 0);
                float pd = 1;

                float ps = sqrt(pa * pa + pb * pb + pc * pc);
                pa /= ps; pb /= ps; pc /= ps; pd /= ps;

                bool planeValid = true;
                for (int j = 0; j < 5; j++) {
                    if (fabs(pa * laserCloudSurfFromMapDS->points[pointSearchInd[j]].x +
                             pb * laserCloudSurfFromMapDS->points[pointSearchInd[j]].y +
                             pc * laserCloudSurfFromMapDS->points[pointSearchInd[j]].z + pd) > 0.2) {
                        planeValid = false;
                        break;
                    }
                }

                if (planeValid) {
                    float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;

                    float s = 1 - 0.9 * fabs(pd2) / sqrt(sqrt(pointOri.x * pointOri.x
                            + pointOri.y * pointOri.y + pointOri.z * pointOri.z));

                    coeff.x = s * pa;
                    coeff.y = s * pb;
                    coeff.z = s * pc;
                    coeff.intensity = s * pd2;

                    if (s > 0.1) {
                        laserCloudOriSurfVec[i] = pointOri;
                        coeffSelSurfVec[i] = coeff;
                        laserCloudOriSurfFlag[i] = true;
                    }
                }
            }
        }
    }

    void combineOptimizationCoeffs()
    {
        // combine surf coeffs
        for (int i = 0; i < laserCloudSurfLastDSNum; ++i){
            if (laserCloudOriSurfFlag[i] == true){
                laserCloudOri->push_back(laserCloudOriSurfVec[i]);
                coeffSel->push_back(coeffSelSurfVec[i]);
            }
        }
        // reset flag for next iteration
        std::fill(laserCloudOriSurfFlag.begin(), laserCloudOriSurfFlag.end(), false);
    }

    bool LMOptimization(int iterCount)
    {
        // This optimization is from the original loam_velodyne by Ji Zhang, need to cope with coordinate transformation
        // lidar <- camera      ---     camera <- lidar
        // x = z                ---     x = y
        // y = x                ---     y = z
        // z = y                ---     z = x
        // roll = yaw           ---     roll = pitch
        // pitch = roll         ---     pitch = yaw
        // yaw = pitch          ---     yaw = roll

        // lidar -> camera
        float srx = sin(transformTobeMapped[2]);
        float crx = cos(transformTobeMapped[2]);
        float sry = sin(transformTobeMapped[1]);
        float cry = cos(transformTobeMapped[1]);
        float srz = sin(transformTobeMapped[0]);
        float crz = cos(transformTobeMapped[0]);

        int laserCloudSelNum = laserCloudOri->size();
        if (laserCloudSelNum < 50) {
            return false;
        }

        cv::Mat matA(laserCloudSelNum, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matAt(6, laserCloudSelNum, CV_32F, cv::Scalar::all(0));
        cv::Mat matAtA(6, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matB(laserCloudSelNum, 1, CV_32F, cv::Scalar::all(0));
        cv::Mat matAtB(6, 1, CV_32F, cv::Scalar::all(0));
        cv::Mat matX(6, 1, CV_32F, cv::Scalar::all(0));

        PointType pointOri, coeff;

        for (int i = 0; i < laserCloudSelNum; i++) {
            // lidar -> camera
            pointOri.x = laserCloudOri->points[i].x;
            pointOri.y = laserCloudOri->points[i].y;
            pointOri.z = laserCloudOri->points[i].z;
            // lidar -> camera
            coeff.x = coeffSel->points[i].x;
            coeff.y = coeffSel->points[i].y;
            coeff.z = coeffSel->points[i].z;
            coeff.intensity = coeffSel->points[i].intensity;
            // in camera
/*             float arx = (crx*sry*srz*pointOri.x + crx*crz*sry*pointOri.y - srx*sry*pointOri.z) * coeff.x
                      + (-srx*srz*pointOri.x - crz*srx*pointOri.y - crx*pointOri.z) * coeff.y
                      + (crx*cry*srz*pointOri.x + crx*cry*crz*pointOri.y - cry*srx*pointOri.z) * coeff.z;

            float ary = ((cry*srx*srz - crz*sry)*pointOri.x 
                      + (sry*srz + cry*crz*srx)*pointOri.y + crx*cry*pointOri.z) * coeff.x
                      + ((-cry*crz - srx*sry*srz)*pointOri.x 
                      + (cry*srz - crz*srx*sry)*pointOri.y - crx*sry*pointOri.z) * coeff.z;

            float arz = ((crz*srx*sry - cry*srz)*pointOri.x + (-cry*crz-srx*sry*srz)*pointOri.y)*coeff.x
                      + (crx*crz*pointOri.x - crx*srz*pointOri.y) * coeff.y
                      + ((sry*srz + cry*crz*srx)*pointOri.x + (crz*sry-cry*srx*srz)*pointOri.y)*coeff.z;
             */

            float arx = (-srx * cry * pointOri.x - (srx * sry * srz + crx * crz) * pointOri.y + (crx * srz - srx * sry * crz) * pointOri.z) * coeff.x
                      + (crx * cry * pointOri.x - (srx * crz - crx * sry * srz) * pointOri.y + (crx * sry * crz + srx * srz) * pointOri.z) * coeff.y;

            float ary = (-crx * sry * pointOri.x + crx * cry * srz * pointOri.y + crx * cry * crz * pointOri.z) * coeff.x
                      + (-srx * sry * pointOri.x + srx * sry * srz * pointOri.y + srx * cry * crz * pointOri.z) * coeff.y
                      + (-cry * pointOri.x - sry * srz * pointOri.y - sry * crz * pointOri.z) * coeff.z;

            float arz = ((crx * sry * crz + srx * srz) * pointOri.y + (srx * crz - crx * sry * srz) * pointOri.z) * coeff.x
                      + ((-crx * srz + srx * sry * crz) * pointOri.y + (-srx * sry * srz - crx * crz) * pointOri.z) * coeff.y
                      + (cry * crz * pointOri.y - cry * srz * pointOri.z) * coeff.z;
              
            // camera -> lidar
            matA.at<float>(i, 0) = arz;
            matA.at<float>(i, 1) = ary;
            matA.at<float>(i, 2) = arx;
            matA.at<float>(i, 3) = coeff.x;
            matA.at<float>(i, 4) = coeff.y;
            matA.at<float>(i, 5) = coeff.z;
            matB.at<float>(i, 0) = -coeff.intensity;
        }

        cv::transpose(matA, matAt);
        matAtA = matAt * matA;
        matAtB = matAt * matB;
        cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

        if (iterCount == 0) {

            cv::Mat matE(1, 6, CV_32F, cv::Scalar::all(0));
            cv::Mat matV(6, 6, CV_32F, cv::Scalar::all(0));
            cv::Mat matV2(6, 6, CV_32F, cv::Scalar::all(0));

            cv::eigen(matAtA, matE, matV);
            matV.copyTo(matV2);

            isDegenerate = false;
            float eignThre[6] = {100, 100, 100, 100, 100, 100};
            for (int i = 5; i >= 0; i--) {
                if (matE.at<float>(0, i) < eignThre[i]) {
                    for (int j = 0; j < 6; j++) {
                        matV2.at<float>(i, j) = 0;
                    }
                    isDegenerate = true;
                } else {
                    break;
                }
            }
            matP = matV.inv() * matV2;
        }

        if (isDegenerate)
        {
            cv::Mat matX2(6, 1, CV_32F, cv::Scalar::all(0));
            matX.copyTo(matX2);
            matX = matP * matX2;
        }

        transformTobeMapped[0] += matX.at<float>(0, 0);
        transformTobeMapped[1] += matX.at<float>(1, 0);
        transformTobeMapped[2] += matX.at<float>(2, 0);
        transformTobeMapped[3] += matX.at<float>(3, 0);
        transformTobeMapped[4] += matX.at<float>(4, 0);
        transformTobeMapped[5] += matX.at<float>(5, 0);

        float deltaR = sqrt(
                            pow(pcl::rad2deg(matX.at<float>(0, 0)), 2) +
                            pow(pcl::rad2deg(matX.at<float>(1, 0)), 2) +
                            pow(pcl::rad2deg(matX.at<float>(2, 0)), 2));
        float deltaT = sqrt(
                            pow(matX.at<float>(3, 0) * 100, 2) +
                            pow(matX.at<float>(4, 0) * 100, 2) +
                            pow(matX.at<float>(5, 0) * 100, 2));

        if (deltaR < 0.05 && deltaT < 0.05) {
            return true; // converged
        }
        return false; // keep optimizing
    }

    void scan2MapOptimization()
    {
        if (cloudKeyPoses3D->points.empty())
            return;

        if (laserCloudSurfLastDSNum > 30)
        {
            if (kdtreeLocalMapDirty)
            {
                TicToc t_setInputCloud;
                kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMapDS);
                kdtreeLocalMapDirty = false;
                if (diagnostics)
                    diagnostics->recordSlice("scan2MapOptimization.setInputCloud", t_setInputCloud.toc());
            }

            double surf_ms_total = 0.0;
            double combine_ms_total = 0.0;
            double lm_ms_total = 0.0;
            int iter_used = 0;

            for (int iterCount = 0; iterCount < 30; iterCount++)
            {
                iter_used++;
                laserCloudOri->clear();
                coeffSel->clear();

                TicToc t_surfOptimization;
                surfOptimization();
                surf_ms_total += t_surfOptimization.toc();

                TicToc t_combineOptimizationCoeffs;
                combineOptimizationCoeffs();
                combine_ms_total += t_combineOptimizationCoeffs.toc();

                TicToc t_lmOptimization;
                if (LMOptimization(iterCount) == true)
                {
                    lm_ms_total += t_lmOptimization.toc();
                    break;              
                }
                lm_ms_total += t_lmOptimization.toc();
            }

            if (diagnostics)
            {
                diagnostics->recordSlice("scan2MapOptimization.surfOptimization.total", surf_ms_total);
                diagnostics->recordSlice("scan2MapOptimization.combineOptimizationCoeffs.total", combine_ms_total);
                diagnostics->recordSlice("scan2MapOptimization.LMOptimization.total", lm_ms_total);

                std::ostringstream oss;
                oss << "[SCAN2MAP_ITER] iter_used=" << iter_used
                    << " surf_total_ms=" << std::fixed << std::setprecision(3) << surf_ms_total
                    << " combine_total_ms=" << combine_ms_total
                    << " lm_total_ms=" << lm_ms_total;
                diagnostics->logEventThrottle("scan2map_iter_summary", 1.0, oss.str());
            }

            TicToc t_transformUpdate;
            transformUpdate();
            if (diagnostics)
                diagnostics->recordSlice("scan2MapOptimization.transformUpdate", t_transformUpdate.toc());
        } else {
            RCLCPP_WARN(get_logger(), "Not enough features! Only %d planar features available.", laserCloudSurfLastDSNum);
        }
    }

    void transformUpdate()
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

    float constraintTransformation(float value, float limit)
    {
        if (value < -limit)
            value = -limit;
        if (value > limit)
            value = limit;

        return value;
    }

    bool saveFrame()
    {
        if (cloudKeyPoses3D->points.empty())
            return true;

        Eigen::Affine3f transStart = pclPointToAffine3f(cloudKeyPoses6D->back());
        Eigen::Affine3f transFinal = pcl::getTransformation(transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5], 
                                                            transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
        Eigen::Affine3f transBetween = transStart.inverse() * transFinal;
        float x, y, z, roll, pitch, yaw;
        pcl::getTranslationAndEulerAngles(transBetween, x, y, z, roll, pitch, yaw);

        if (abs(roll)  < surroundingkeyframeAddingAngleThreshold &&
            abs(pitch) < surroundingkeyframeAddingAngleThreshold && 
            abs(yaw)   < surroundingkeyframeAddingAngleThreshold &&
            sqrt(x*x + y*y + z*z) < surroundingkeyframeAddingDistThreshold)
            return false;

        return true;
    }

    void addOdomFactor()
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
            gtsam::Pose3 poseTo   = trans2gtsamPose(transformTobeMapped);
            gtSAMgraph.add(BetweenFactor<Pose3>(cloudKeyPoses3D->size()-1, cloudKeyPoses3D->size(), poseFrom.between(poseTo), odometryNoise));
            initialEstimate.insert(cloudKeyPoses3D->size(), poseTo);
        }
    }

    void addGPSFactor()
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

            if (gps_eligible_time < timeLaserInfoCur - 0.2)
            {
                // message too old
                gpsQueue.pop_front();
            }
            else if (gps_eligible_time > timeLaserInfoCur + 0.2)
            {
                // message not yet eligible for delayed processing
                break;
            }
            else
            {
                nav_msgs::msg::Odometry thisGPS = gpsQueue.front();
                gpsQueue.pop_front();

                // GPS too noisy, skip
                float noise_x = thisGPS.pose.covariance[0];
                float noise_y = thisGPS.pose.covariance[7];
                float noise_z = thisGPS.pose.covariance[14];
                if (noise_x > gpsCovThreshold || noise_y > gpsCovThreshold)
                    continue;

                float gps_x = thisGPS.pose.pose.position.x;
                float gps_y = thisGPS.pose.pose.position.y;
                float gps_z = thisGPS.pose.pose.position.z;
                if (!useGpsElevation)
                {
                    gps_z = transformTobeMapped[5];
                    noise_z = 0.01;
                }

                // GPS not properly initialized (0,0,0)
                if (abs(gps_x) < 1e-6 && abs(gps_y) < 1e-6)
                    continue;

                // Add GPS every a few meters
                PointType curGPSPoint;
                curGPSPoint.x = gps_x;
                curGPSPoint.y = gps_y;
                curGPSPoint.z = gps_z;
                if (common_lib_->pointDistance(curGPSPoint, lastGPSPoint) < 5.0)
                    continue;
                else
                    lastGPSPoint = curGPSPoint;

                // 1. Initialize T_GL if this is the first GPS fusion
                if (!T_GL_initialized) {
                    gtsam::Pose3 current_local_pose = pclPointTogtsamPose3(cloudKeyPoses6D->points.back());
                    gtsam::Point3 initial_translation(gps_x - current_local_pose.x(), 
                                                      gps_y - current_local_pose.y(), 
                                                      gps_z - current_local_pose.z());
                    
                    // Start with identity rotation (yaw=0). It will become observable with motion.
                    T_GL_estimate = gtsam::Pose3(gtsam::Rot3::Identity(), initial_translation);
                    initialEstimate.insert(T_GL_KEY, T_GL_estimate);
                    
                    // Add a weak prior to prevent singularity before heading is observable
                    gtsam::Vector6 prior_noise_vector;
                    prior_noise_vector << 1e-2, 1e-2, M_PI, 1e8, 1e8, 1e8; // Weak on yaw
                    gtSAMgraph.add(PriorFactor<Pose3>(T_GL_KEY, T_GL_estimate, noiseModel::Diagonal::Variances(prior_noise_vector)));
                    
                    T_GL_initialized = true;
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

                const int search_start_idx = std::max(0, static_cast<int>(cloudKeyPoses6D->size()) - kGpsKeyframeSearchWindow);
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

                if (best_time_diff > kMaxGpsLidarConstraintDtSec)
                {
                    RCLCPP_WARN_THROTTLE(
                        get_logger(),
                        *get_clock(),
                        5000,
                        "Skipping GPS constraint due to dt threshold: %.3f s (gps=%.3f, keyframe=%.3f, max=%.3f)",
                        best_time_diff,
                        gps_time,
                        best_keyframe_time,
                        kMaxGpsLidarConstraintDtSec);

                    if (diagnostics)
                    {
                        std::ostringstream skippedConstraintOss;
                        skippedConstraintOss << "[GPS_CONSTRAINT_SKIPPED_TIME]"
                                             << " candidate_idx=" << best_keyframe_idx
                                             << " gps_t=" << std::fixed << std::setprecision(6) << gps_time
                                             << " keyframe_t=" << best_keyframe_time
                                             << " dt=" << best_time_diff
                                             << " max_dt=" << kMaxGpsLidarConstraintDtSec;
                        diagnostics->logEvent(skippedConstraintOss.str());
                    }
                    continue;
                }

                // Bypass Expression framework completely
                const int lidar_key = best_keyframe_idx;
                FloatingAnchorFactor gps_factor(T_GL_KEY, lidar_key, measured_gps, antenna_offset, gps_noise);
                gtSAMgraph.add(gps_factor);
                ++gpsFactorsAccepted;

                std::ostringstream gpsConstraintOss;
                gpsConstraintOss << "[GPS_CONSTRAINT_ADDED]"
                                 << " idx=" << lidar_key
                                 << " accepted=" << gpsFactorsAccepted
                                 << " gps_t=" << std::fixed << std::setprecision(6) << gps_time
                                 << " keyframe_t=" << best_keyframe_time
                                 << " dt=" << best_time_diff
                                 << " gps_xyz=(" << measured_gps.x() << "," << measured_gps.y() << "," << measured_gps.z() << ")";

                RCLCPP_INFO_STREAM(get_logger(), gpsConstraintOss.str());
                if (diagnostics)
                    diagnostics->logEvent(gpsConstraintOss.str());

                gpsLidarAssociationQueue.emplace_back(lidar_key, std::make_pair(measured_gps, gps_time));
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

    void addLoopFactor()
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

    bool saveKeyFramesAndFactor()
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

        gtSAMgraph.resize(0);
        initialEstimate.clear();

        //save key poses
        PointType thisPose3D;
        PointTypePose thisPose6D;
        Pose3 latestEstimate;

        isamCurrentEstimate = isam->calculateEstimate();
        if (T_GL_initialized && isamCurrentEstimate.exists(T_GL_KEY)) {
            T_GL_estimate = isamCurrentEstimate.at<Pose3>(T_GL_KEY);
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
        thisPose6D.intensity = thisPose3D.intensity ; // this can be used as index
        thisPose6D.roll  = latestEstimate.rotation().roll();
        thisPose6D.pitch = latestEstimate.rotation().pitch();
        thisPose6D.yaw   = latestEstimate.rotation().yaw();
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
        pcl::copyPointCloud(*laserCloudSurfLastDS,    *thisSurfKeyFrame);

        // save key frame cloud
        surfCloudKeyFrames.push_back(thisSurfKeyFrame);

        // The following code is copy from sc-lio-sam
        // Scan Context loop detector - giseop
        // - SINGLE_SCAN_FULL: using downsampled original point cloud (/full_cloud_projected + downsampling)
        // - SINGLE_SCAN_FEAT: using surface feature as an input point cloud for scan context (2020.04.01: checked it works.)
        // - MULTI_SCAN_FEAT: using NearKeyframes (because a MulRan scan does not have beyond region, so to solve this issue ... )
        const SCInputType sc_input_type = SCInputType::SINGLE_SCAN_FULL; // change this 

        if( sc_input_type == SCInputType::SINGLE_SCAN_FULL )
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

    void correctPoses()
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
                cloudKeyPoses6D->points[i].roll  = isamCurrentEstimate.at<Pose3>(i).rotation().roll();
                cloudKeyPoses6D->points[i].pitch = isamCurrentEstimate.at<Pose3>(i).rotation().pitch();
                cloudKeyPoses6D->points[i].yaw   = isamCurrentEstimate.at<Pose3>(i).rotation().yaw();

                updatePath(cloudKeyPoses6D->points[i]);
            }

            aLoopIsClosed = false;
        }
    }

    void updatePath(const PointTypePose& pose_in)
    {
        geometry_msgs::msg::PoseStamped pose_stamped;
        rclcpp::Time t(static_cast<uint32_t>(pose_in.time * 1e9));
        pose_stamped.header.stamp = t;
        pose_stamped.header.frame_id = odometryFrame;
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

    void publishMapOptimizationTFs(const rclcpp::Time &stamp)
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
            q_map_to_map_local.setRPY(T_GL_estimate.rotation().roll(), T_GL_estimate.rotation().pitch(), T_GL_estimate.rotation().yaw());
            t_map_to_map_local = tf2::Transform(q_map_to_map_local, tf2::Vector3(T_GL_estimate.translation().x(), T_GL_estimate.translation().y(), T_GL_estimate.translation().z()));

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
                
                offset_msg.pose.pose.position.x = T_GL_estimate.translation().x();
                offset_msg.pose.pose.position.y = T_GL_estimate.translation().y();
                offset_msg.pose.pose.position.z = T_GL_estimate.translation().z();
                
                offset_msg.pose.pose.orientation.x = q_map_to_map_local.x();
                offset_msg.pose.pose.orientation.y = q_map_to_map_local.y();
                offset_msg.pose.pose.orientation.z = q_map_to_map_local.z();
                offset_msg.pose.pose.orientation.w = q_map_to_map_local.w();

                if (isamCurrentEstimate.exists(T_GL_KEY)) {
                    gtsam::Matrix marginalCov = isam->marginalCovariance(T_GL_KEY);
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

        // ========== TRANSFORM 1: odom -> lidar_link (direct optimized pose) ==========
        // This remains published from mapOptimization as requested
        float odom_x, odom_y, odom_z, odom_roll, odom_pitch, odom_yaw;
        pcl::getTranslationAndEulerAngles(odomToBaseAffine, odom_x, odom_y, odom_z, odom_roll, odom_pitch, odom_yaw);
        tf2::Quaternion quat_odom_to_base;
        quat_odom_to_base.setRPY(odom_roll, odom_pitch, odom_yaw);
        tf2::Transform t_odom_to_base = tf2::Transform(quat_odom_to_base, tf2::Vector3(odom_x, odom_y, odom_z));

        tf2::Transform t_odom_to_lidar;
        if (lidarFrame != baselinkFrame && hasLidar2Baselink)
            t_odom_to_lidar = t_odom_to_base * lidar2Baselink.inverse();
        else
            t_odom_to_lidar = t_odom_to_base;

        tf2::Stamped<tf2::Transform> stamped_odom_to_lidar(t_odom_to_lidar, time_point, odometryFrame);
        geometry_msgs::msg::TransformStamped trans_odom_to_lidar;
        tf2::convert(stamped_odom_to_lidar, trans_odom_to_lidar);
        trans_odom_to_lidar.child_frame_id = "lidar_link";
        br->sendTransform(trans_odom_to_lidar);

        if (debugTFs)
        {
            RCLCPP_INFO_STREAM_THROTTLE(
                get_logger(), *get_clock(), 10000,
                "[TF_DEBUG] publish [1/2] " << odometryFrame << "->lidar_link (direct optimization result)"
                << " xyz=(" << odom_x << ", " << odom_y << ", " << odom_z << ")"
                << " rpy=(" << odom_roll << ", " << odom_pitch << ", " << odom_yaw << ")"
            );
        }

        // ========== TRANSFORM 2: odom -> baselinkFrame (optimized pose * lidar2baselink) ==========
        // This applies the looked-up or identity lidar<->baselink transform
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
                tf2::Transform t_odom_to_baselink = t_odom_to_lidar * lidar2Baselink;
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
                        << " (optimized result * looked-up lidar2baselink)"
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
            // Frames are identical: lidar2baselink is identity, publish with explicit identity explanation
            tf2::Transform t_odom_to_baselink = t_odom_to_lidar * lidar2Baselink;  // multiplication by identity
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
                    << " (optimized result * identity lidar2baselink)"
                    << " [frames identical: '" << lidarFrame << "' == '" << baselinkFrame << "']"
                );
            }
        }
    }

    void publishLidarGpsFix()
    {
        if (first_gps || !gpsAnchorReady())
            return;
        if (pubLidarGpsFix->get_subscription_count() == 0 &&
            pubLidarGpsEnuPose->get_subscription_count() == 0 &&
            pubLidarGpsNedPose->get_subscription_count() == 0)
            return;

        // Transform LiDAR local pose into ENU frame via the floating-anchor T_GL
        gtsam::Point3 p_local(transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5]);
        gtsam::Point3 p_enu = T_GL_estimate.transformFrom(p_local);

        // Project ENU position back to geodetic lat/lon/alt
        double lat, lon, alt;
        gps_trans_.Reverse(p_enu.x(), p_enu.y(), p_enu.z(), lat, lon, alt);

        // Build local rotation identically to publishOdometry: setRPY(roll, pitch, yaw)
        tf2::Quaternion q_local_tf2;
        q_local_tf2.setRPY(transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
        gtsam::Rot3 R_local(Eigen::Quaterniond(
            q_local_tf2.w(), q_local_tf2.x(), q_local_tf2.y(), q_local_tf2.z()));

        // Rotate LiDAR orientation into ENU via the floating-anchor T_GL
        gtsam::Rot3 R_enu = T_GL_estimate.rotation().compose(R_local);
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
    }

    void publishOdometry()
    {
        // Publish odometry for ROS (global)
        nav_msgs::msg::Odometry laserOdometryROS;
        laserOdometryROS.header.stamp = timeLaserInfoStamp;
        laserOdometryROS.header.frame_id = odometryFrame;
        laserOdometryROS.child_frame_id = "lidar_link";
        laserOdometryROS.pose.pose.position.x = transformTobeMapped[3];
        laserOdometryROS.pose.pose.position.y = transformTobeMapped[4];
        laserOdometryROS.pose.pose.position.z = transformTobeMapped[5];
        // Ref: http://wiki.ros.org/tf2/Tutorials/Migration/DataConversions
        tf2::Quaternion quat_tf;
        quat_tf.setRPY(transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
        geometry_msgs::msg::Quaternion quat_msg;
        tf2::convert(quat_tf, quat_msg);
        laserOdometryROS.pose.pose.orientation = quat_msg;
        pubLaserOdometryGlobal->publish(laserOdometryROS);

        // Publish odometry for ROS (incremental)
        static bool lastIncreOdomPubFlag = false;
        static nav_msgs::msg::Odometry laserOdomIncremental; // incremental odometry msg
        static Eigen::Affine3f increOdomAffine; // incremental odometry in affine
        if (lastIncreOdomPubFlag == false)
        {
            lastIncreOdomPubFlag = true;
            laserOdomIncremental = laserOdometryROS;
            increOdomAffine = trans2Affine3f(transformTobeMapped);
        } else {
            lastLidarOdometryIncrement = incrementalOdometryAffineFront.inverse() * incrementalOdometryAffineBack;
            increOdomAffine = increOdomAffine * lastLidarOdometryIncrement;
            float x, y, z, roll, pitch, yaw;
            pcl::getTranslationAndEulerAngles (increOdomAffine, x, y, z, roll, pitch, yaw);
            if (cloudInfo.imuavailable == true && imuType)
            {
                if (std::abs(cloudInfo.imupitchinit) < 1.4)
                {
                    double imuWeight = 0.1;
                    tf2::Quaternion imuQuaternion;
                    tf2::Quaternion transformQuaternion;
                    double rollMid, pitchMid, yawMid;

                    // slerp roll
                    transformQuaternion.setRPY(roll, 0, 0);
                    imuQuaternion.setRPY(cloudInfo.imurollinit, 0, 0);
                    tf2::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);
                    roll = rollMid;

                    // slerp pitch
                    transformQuaternion.setRPY(0, pitch, 0);
                    imuQuaternion.setRPY(0, cloudInfo.imupitchinit, 0);
                    tf2::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);
                    pitch = pitchMid;
                }
            }
            laserOdomIncremental.header.stamp = timeLaserInfoStamp;
            laserOdomIncremental.header.frame_id = odometryFrame;
            laserOdomIncremental.child_frame_id = "odom_mapping";
            laserOdomIncremental.pose.pose.position.x = x;
            laserOdomIncremental.pose.pose.position.y = y;
            laserOdomIncremental.pose.pose.position.z = z;
            tf2::Quaternion quat_tf;
            quat_tf.setRPY(roll, pitch, yaw);
            geometry_msgs::msg::Quaternion quat_msg;
            tf2::convert(quat_tf, quat_msg);
            laserOdomIncremental.pose.pose.orientation = quat_msg;
            if (isDegenerate)
                laserOdomIncremental.pose.covariance[0] = 1;
            else
                laserOdomIncremental.pose.covariance[0] = 0;
        }
        pubLaserOdometryIncremental->publish(laserOdomIncremental);

        Eigen::Affine3f mapLocalToBase = trans2Affine3f(transformTobeMapped);
        Eigen::Affine3f odomToBase;

        tf2::Quaternion q_inc(
            laserOdomIncremental.pose.pose.orientation.x,
            laserOdomIncremental.pose.pose.orientation.y,
            laserOdomIncremental.pose.pose.orientation.z,
            laserOdomIncremental.pose.pose.orientation.w);
        double inc_roll, inc_pitch, inc_yaw;
        tf2::Matrix3x3(q_inc).getRPY(inc_roll, inc_pitch, inc_yaw);
        odomToBase = pcl::getTransformation(
            laserOdomIncremental.pose.pose.position.x,
            laserOdomIncremental.pose.pose.position.y,
            laserOdomIncremental.pose.pose.position.z,
            inc_roll,
            inc_pitch,
            inc_yaw
        );

        odomToBaseAffine = odomToBase;
        mapLocalToOdomAffine = mapLocalToBase * odomToBaseAffine.inverse();
        mapLocalToOdomInitialized = true;
    }

    void publishFrames()
    {
        if (cloudKeyPoses3D->points.empty())
            return;
        // publish key poses
        publishCloud(pubKeyPoses, cloudKeyPoses3D, timeLaserInfoStamp, odometryFrame);
        // Publish surrounding key frames
        publishCloud(pubRecentKeyFrames, laserCloudSurfFromMapDS, timeLaserInfoStamp, odometryFrame);
        // publish registered key frame
        if (pubRecentKeyFrame->get_subscription_count() != 0)
        {
            pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
            PointTypePose thisPose6D = trans2PointTypePose(transformTobeMapped);
            *cloudOut += *transformPointCloud(laserCloudSurfLastDS,    &thisPose6D);
            publishCloud(pubRecentKeyFrame, cloudOut, timeLaserInfoStamp, odometryFrame);
        }
        // publish registered high-res raw cloud
        if (pubCloudRegisteredRaw->get_subscription_count() != 0)
        {
            pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
            pcl::fromROSMsg(cloudInfo.cloud_deskewed, *cloudOut);
            PointTypePose thisPose6D = trans2PointTypePose(transformTobeMapped);
            *cloudOut = *transformPointCloud(cloudOut,  &thisPose6D);
            publishCloud(pubCloudRegisteredRaw, cloudOut, timeLaserInfoStamp, odometryFrame);
        }
        // publish path
        if (pubPath->get_subscription_count() != 0)
        {
            globalPath.header.stamp = timeLaserInfoStamp;
            globalPath.header.frame_id = odometryFrame;
            pubPath->publish(globalPath);
        }
        // publish SLAM infomation for 3rd-party usage
        static int lastSLAMInfoPubSize = -1;
        if (pubSLAMInfo->get_subscription_count() != 0)
        {
            // if (lastSLAMInfoPubSize != cloudKeyPoses6D->size())
            // {
            //     liorf::msg::CloudInfo slamInfo;
            //     slamInfo.header.stamp = timeLaserInfoStamp;
            //     pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
            //     *cloudOut += *laserCloudSurfLastDS;
            //     slamInfo.key_frame_cloud = publishCloud(rclcpp::Publisher(), cloudOut, timeLaserInfoStamp, lidarFrame);
            //     slamInfo.key_frame_poses = publishCloud(rclcpp::Publisher(), cloudKeyPoses6D, timeLaserInfoStamp, odometryFrame);
            //     pcl::PointCloud<PointType>::Ptr localMapOut(new pcl::PointCloud<PointType>());
            //     *localMapOut += *laserCloudSurfFromMapDS;
            //     slamInfo.key_frame_map = publishCloud(rclcpp::Publisher(), localMapOut, timeLaserInfoStamp, odometryFrame);
            //     pubSLAMInfo->publish(slamInfo);
            //     lastSLAMInfoPubSize = cloudKeyPoses6D->size();
            // }
        }
    }
};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);
    rclcpp::executors::SingleThreadedExecutor exec;

    auto MO = std::make_shared<mapOptimization>(options);
    exec.add_node(MO);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[1;32m----> Map Optimization Started.\033[0m");

    std::thread loopthread(&mapOptimization::loopClosureThread, MO);
    std::thread visualizeMapThread(&mapOptimization::visualizeGlobalMapThread, MO);

    exec.spin();

    rclcpp::shutdown();

    loopthread.join();
    visualizeMapThread.join();

    return 0;
}
