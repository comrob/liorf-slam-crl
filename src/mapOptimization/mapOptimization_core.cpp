#include "mapOptimization/mapOptimization.hpp"
#include "scanAlignment/VoxelPkoBackend.hpp"
#include "scanAlignment/KdTreeLmBackend.hpp"

using gtsam::ISAM2;
using gtsam::ISAM2Params;

mapOptimization::mapOptimization(const rclcpp::NodeOptions & options) : ParamServer("liorf_mapOptimization", options)
{
    ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.1;
    parameters.relinearizeSkip = 1;
    isam = new ISAM2(parameters);

    diagnostics = std::make_shared<LiorfDiagnostics>(
        this,
        QosPolicy(history_policy, reliability_policy),
        history_policy,
        reliability_policy,
        "~/.ros/liorf_logs",
        backend_type,
        "/liorf/debug/telemetry",
        1.0,
        diagnostics_write_files_master,
        diagnostics_write_timing_stats,
        diagnostics_write_event,
        diagnostics_write_warnings,
        diagnostics_write_telemetry,
        diagnostics_write_time_deltas,
        diagnostics_write_frame_metrics);

    auto cloudInfoQos = QosPolicy(history_policy, reliability_policy);
    cloudInfoQos.keep_last(std::max(1, cloud_info_queue_depth));

    subCloud = create_subscription<liorf::msg::CloudInfo>("liorf/deskew/cloud_info", cloudInfoQos,
                std::bind(&mapOptimization::laserCloudInfoHandler, this, std::placeholders::_1));
    subGPS = create_subscription<sensor_msgs::msg::NavSatFix>(gpsTopic, QosPolicy(history_policy, reliability_policy),
                std::bind(&mapOptimization::gpsHandler, this, std::placeholders::_1));
    subLoop = create_subscription<std_msgs::msg::Float64MultiArray>("lio_loop/loop_closure_detection", QosPolicy(history_policy, reliability_policy),
                std::bind(&mapOptimization::loopInfoHandler, this, std::placeholders::_1));
    if (!complementaryOdomTopic.empty() && degeneracyDetection.compensation_source == "complementary_odom")
    {
        subComplementaryOdom = create_subscription<nav_msgs::msg::Odometry>(
            complementaryOdomTopic, QosPolicy(history_policy, reliability_policy),
            std::bind(&mapOptimization::complementaryOdomHandler, this, std::placeholders::_1));
    }

    pubKeyPoses = create_publisher<sensor_msgs::msg::PointCloud2>("liorf/mapping/trajectory", QosPolicy(history_policy, reliability_policy));
    pubLaserCloudSurround = create_publisher<sensor_msgs::msg::PointCloud2>("liorf/mapping/map_global", QosPolicy(history_policy, reliability_policy));
    pubLaserOdometryGlobal = create_publisher<nav_msgs::msg::Odometry>("liorf/mapping/odometry", QosPolicy(history_policy, reliability_policy));
    pubLaserOdometryIncremental = create_publisher<nav_msgs::msg::Odometry>("liorf/mapping/odometry_incremental", QosPolicy(history_policy, reliability_policy));
    pubBaselinkOdometryGlobal = create_publisher<nav_msgs::msg::Odometry>("liorf/mapping/baselink_odometry", QosPolicy(history_policy, reliability_policy));
    pubBaselinkOdometryIncremental = create_publisher<nav_msgs::msg::Odometry>("liorf/mapping/baselink_odometry_incremental", QosPolicy(history_policy, reliability_policy));
    pubPath = create_publisher<nav_msgs::msg::Path>("liorf/mapping/path", QosPolicy(history_policy, reliability_policy));
    pubHistoryKeyFrames = create_publisher<sensor_msgs::msg::PointCloud2>("liorf/mapping/icp_loop_closure_history_cloud", QosPolicy(history_policy, reliability_policy));
    pubIcpKeyFrames = create_publisher<sensor_msgs::msg::PointCloud2>("liorf/mapping/icp_loop_closure_corrected_cloud", QosPolicy(history_policy, reliability_policy));
    pubLoopConstraintEdge = create_publisher<visualization_msgs::msg::MarkerArray>("/liorf/mapping/loop_closure_constraints", QosPolicy(history_policy, reliability_policy));
    pubGpsConstraintViz = create_publisher<visualization_msgs::msg::MarkerArray>("/liorf/mapping/gps_constraints", QosPolicy(history_policy, reliability_policy));
    pubLocalMapCloud = create_publisher<sensor_msgs::msg::PointCloud2>("liorf/mapping/map_local", QosPolicy(history_policy, reliability_policy));
    pubRegisteredCloud = create_publisher<sensor_msgs::msg::PointCloud2>("liorf/mapping/cloud_registered", QosPolicy(history_policy, reliability_policy));
    pubCloudPreviousPose = create_publisher<sensor_msgs::msg::PointCloud2>("liorf/mapping/debug/cloud_previous_pose", QosPolicy(history_policy, reliability_policy));
    pubCloudPredictedPose = create_publisher<sensor_msgs::msg::PointCloud2>("liorf/mapping/debug/cloud_predicted_pose", QosPolicy(history_policy, reliability_policy));
    pubKeyframeDeskewedDownsampled = create_publisher<sensor_msgs::msg::PointCloud2>("liorf/mapping/keyframes/cloud_deskewed_downsampled", QosPolicy(history_policy, reliability_policy));
    pubKeyframeDeskewedDownsampledDebug = create_publisher<sensor_msgs::msg::PointCloud2>("liorf/mapping/keyframes/cloud_deskewed_downsampled_debug", QosPolicy(history_policy, reliability_policy));
    pubMatchedSurfFeatures = create_publisher<sensor_msgs::msg::PointCloud2>("liorf/mapping/matched_surface_features", QosPolicy(history_policy, reliability_policy));
    pubKdTreePlanePoints = create_publisher<sensor_msgs::msg::PointCloud2>("liorf/mapping/kdtree_plane_points", QosPolicy(history_policy, reliability_policy));
    pubKdTreePlaneNormals = create_publisher<visualization_msgs::msg::MarkerArray>("liorf/mapping/kdtree_plane_normals", QosPolicy(history_policy, reliability_policy));
    pubKdTreePlaneResiduals = create_publisher<visualization_msgs::msg::MarkerArray>("liorf/mapping/kdtree_plane_residuals", QosPolicy(history_policy, reliability_policy));
    pubSurfDebugColored = create_publisher<sensor_msgs::msg::PointCloud2>("liorf/mapping/surf_debug_colored", QosPolicy(history_policy, reliability_policy));
    pubSurfDebugLegend = create_publisher<std_msgs::msg::String>("liorf/mapping/surf_debug_legend", QosPolicy(history_policy, reliability_policy));
    pubCloudRegisteredRaw = create_publisher<sensor_msgs::msg::PointCloud2>("liorf/mapping/cloud_registered_raw", QosPolicy(history_policy, reliability_policy));
    pubSLAMInfo = create_publisher<liorf::msg::CloudInfo>("liorf/mapping/slam_info", QosPolicy(history_policy, reliability_policy));
    pubGpsOdom = create_publisher<nav_msgs::msg::Odometry>("liorf/mapping/gps_odom", QosPolicy(history_policy, reliability_policy));
    pubGlobalOffset = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("liorf/enu_to_local_offset", QosPolicy(history_policy, reliability_policy));
    pubLidarGpsFix = create_publisher<sensor_msgs::msg::NavSatFix>("liorf/mapping/lidar_gps_fix", QosPolicy(history_policy, reliability_policy));
    pubLidarGpsEnuPose = create_publisher<geometry_msgs::msg::PoseStamped>("liorf/mapping/lidar_gps_enu_pose", QosPolicy(history_policy, reliability_policy));
    pubLidarGpsNedPose = create_publisher<geometry_msgs::msg::PoseStamped>("liorf/mapping/lidar_gps_ned_pose", QosPolicy(history_policy, reliability_policy));
    pubBaselinkGpsEnuOdometry = create_publisher<nav_msgs::msg::Odometry>("liorf/mapping/baselink_gps_enu_odometry", QosPolicy(history_policy, reliability_policy));
    pubBaselinkGpsNedOdometry = create_publisher<nav_msgs::msg::Odometry>("liorf/mapping/baselink_gps_ned_odometry", QosPolicy(history_policy, reliability_policy));

    pubDegeneracyRaw = create_publisher<visualization_msgs::msg::MarkerArray>("liorf/mapping/degeneracy_raw", 1);
    pubDegeneracyPCA = create_publisher<visualization_msgs::msg::MarkerArray>("liorf/mapping/degeneracy_pca", 1);
    pubDegeneracyBasis = create_publisher<visualization_msgs::msg::MarkerArray>("liorf/mapping/degeneracy_basis", 1);
    pubDegeneracyPaths = create_publisher<visualization_msgs::msg::MarkerArray>("liorf/mapping/degeneracy_paths", 1);
    pubDegeneracyPerturbedScan0 = create_publisher<sensor_msgs::msg::PointCloud2>("liorf/mapping/degeneracy/perturbed_scan_0", QosPolicy(history_policy, reliability_policy));
    pubDegeneracyPerturbedScan1 = create_publisher<sensor_msgs::msg::PointCloud2>("liorf/mapping/degeneracy/perturbed_scan_1", QosPolicy(history_policy, reliability_policy));
    pubDegeneracyPerturbedScan2 = create_publisher<sensor_msgs::msg::PointCloud2>("liorf/mapping/degeneracy/perturbed_scan_2", QosPolicy(history_policy, reliability_policy));
    pubDegeneracyAlignedScan0 = create_publisher<sensor_msgs::msg::PointCloud2>("liorf/mapping/degeneracy/aligned_scan_0", QosPolicy(history_policy, reliability_policy));
    pubDegeneracyAlignedScan1 = create_publisher<sensor_msgs::msg::PointCloud2>("liorf/mapping/degeneracy/aligned_scan_1", QosPolicy(history_policy, reliability_policy));
    pubDegeneracyAlignedScan2 = create_publisher<sensor_msgs::msg::PointCloud2>("liorf/mapping/degeneracy/aligned_scan_2", QosPolicy(history_policy, reliability_policy));
    pubDegeneracyPerturbedPose0 = create_publisher<geometry_msgs::msg::PoseStamped>("liorf/mapping/degeneracy/perturbed_pose_0", QosPolicy(history_policy, reliability_policy));
    pubDegeneracyPerturbedPose1 = create_publisher<geometry_msgs::msg::PoseStamped>("liorf/mapping/degeneracy/perturbed_pose_1", QosPolicy(history_policy, reliability_policy));
    pubDegeneracyPerturbedPose2 = create_publisher<geometry_msgs::msg::PoseStamped>("liorf/mapping/degeneracy/perturbed_pose_2", QosPolicy(history_policy, reliability_policy));
    pubDegeneracyAlignedPose0 = create_publisher<geometry_msgs::msg::PoseStamped>("liorf/mapping/degeneracy/aligned_pose_0", QosPolicy(history_policy, reliability_policy));
    pubDegeneracyAlignedPose1 = create_publisher<geometry_msgs::msg::PoseStamped>("liorf/mapping/degeneracy/aligned_pose_1", QosPolicy(history_policy, reliability_policy));
    pubDegeneracyAlignedPose2 = create_publisher<geometry_msgs::msg::PoseStamped>("liorf/mapping/degeneracy/aligned_pose_2", QosPolicy(history_policy, reliability_policy));
    pubDegeneracyDisplacements = create_publisher<visualization_msgs::msg::MarkerArray>("liorf/mapping/degeneracy/displacements", QosPolicy(history_policy, reliability_policy));
    pubDegeneracyOptimizationPaths = create_publisher<visualization_msgs::msg::MarkerArray>("liorf/mapping/degeneracy/optimization_paths", QosPolicy(history_policy, reliability_policy));
    pubComplementaryOdomCorrectionDirection = create_publisher<visualization_msgs::msg::MarkerArray>("liorf/mapping/complementary_odom/correction_direction", QosPolicy(history_policy, reliability_policy));

    pubGpsOrigin = create_publisher<sensor_msgs::msg::NavSatFix>("liorf/gps_origin", QosPolicy(history_policy, reliability_policy));
    origin_publish_timer = this->create_wall_timer(std::chrono::seconds(1), std::bind(&mapOptimization::timerCallbackPublishOrigin, this));

    srvSaveMap = create_service<liorf::srv::SaveMap>("liorf/save_map", 
                    std::bind(&mapOptimization::saveMapService, this, std::placeholders::_1, std::placeholders::_2 ));

    downSizeFilterSurf.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);
    downSizeFilterLocalMapSurf.setLeafSize(surroundingKeyframeMapLeafSize, surroundingKeyframeMapLeafSize, surroundingKeyframeMapLeafSize);
    downSizeFilterICP.setLeafSize(loopClosureICPSurfLeafSize, loopClosureICPSurfLeafSize, loopClosureICPSurfLeafSize);
    downSizeFilterSurroundingKeyPoses.setLeafSize(surroundingKeyframeDensity, surroundingKeyframeDensity, surroundingKeyframeDensity); // for surrounding key poses of scan-to-map optimization

    br = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    runtimeTfCoordinator->initializeLidarBaselinkTfRelationship("ctor");
    runtimeTfCoordinator->tryLookupLidarToBaselinkTf("ctor");

    allocateMemory();

    if (force_initial_gps && manual_gps_origin.size() == 3)
    {
        initializeDatum(
            manual_gps_origin[0],
            manual_gps_origin[1],
            manual_gps_origin[2],
            manual_global_heading);
    }
}

void mapOptimization::allocateMemory()
{
    cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
    cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());
    copy_cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
    copy_cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());

    kdtreeSurroundingKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());
    kdtreeHistoryKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());

    laserCloudSurfLast.reset(new pcl::PointCloud<PointType>());
    laserCloudSurfLastDS.reset(new pcl::PointCloud<PointType>());

    if (backend_type == "voxel_pko") {
        mappingBackend = std::make_shared<lio::VoxelPkoBackend>(
            voxel_map_config, 
            pko_config, 
            N_SCAN * Horizon_SCAN, 
            surfKnnMinDistance, 
            numberOfCores,
            localMapTruncationRadius
        );
        
    } else if (backend_type == "kdtree_lm") {
        mappingBackend = std::make_shared<lio::KdTreeLmBackend>(kdtree_lm_config);
    } else {
        RCLCPP_ERROR(get_logger(), "Unknown backend_type: %s. Falling back to kdtree_lm.", backend_type.c_str());
        mappingBackend = std::make_shared<lio::KdTreeLmBackend>(kdtree_lm_config);
    }

    DegeneracyParams dParams;
    dParams.n_multiplier = degeneracyDetection.perturbationBased.n_multiplier;
    dParams.max_icp_steps = degeneracyDetection.perturbationBased.max_icp_steps;
    dParams.max_perturbation_angle_deg = degeneracyDetection.perturbationBased.max_perturbation_angle_deg;
    dParams.descriptive_number_threshold = degeneracyDetection.perturbationBased.descriptive_number_threshold;
    dParams.eigen_value_threshold = degeneracyDetection.perturbationBased.eigen_value_threshold;
    dParams.verbose = degeneracyDetection.perturbationBased.verbose;
    degeneracyDetector = std::make_shared<DegeneracyDetector>(dParams);

    temporal_filter_state = 0;

    for (int i = 0; i < 6; ++i){
        transformTobeMapped[i] = 0;
    }

    lastIncrementalDeltaPoseLocal = Eigen::Affine3f::Identity();
    hasLastIncrementalDeltaPoseLocal = false;

    downSizeFilterSurf.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);
    downSizeFilterLocalMapSurf.setLeafSize(surroundingKeyframeMapLeafSize, surroundingKeyframeMapLeafSize, surroundingKeyframeMapLeafSize);
    downSizeFilterICP.setLeafSize(loopClosureICPSurfLeafSize, loopClosureICPSurfLeafSize, loopClosureICPSurfLeafSize);
    downSizeFilterSurroundingKeyPoses.setLeafSize(surroundingKeyframeDensity, surroundingKeyframeDensity, surroundingKeyframeDensity); // for surrounding key poses of scan-to-map optimization
}

void mapOptimization::laserCloudInfoHandler(const liorf::msg::CloudInfo::SharedPtr msgIn)
{
    runtimeTfCoordinator->noteLidarMessageFrameId(msgIn->cloud_deskewed.header.frame_id);
    if (!runtimeTfCoordinator->hasLidarToBaselinkTransform() && !lidarFrame.empty() && lidarFrame != baselinkFrame)
    {
        runtimeTfCoordinator->tryLookupLidarToBaselinkTf("laserCloudInfoHandler/frame_resolved");
    }

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
    if (curTimeDiff <= 0.0)
    {
        if (diagnostics)
        {
            std::ostringstream oss;
            oss << "[LIDAR_FRAME_SKIP] reason=non_positive_dt"
                << " dt_s=" << std::fixed << std::setprecision(6) << curTimeDiff
                << " stamp_s=" << timeLaserInfoCur
                << " last_processing_s=" << timeLastProcessing;
            diagnostics->logEventThrottle("lidar_frame_skip_non_positive_dt", 1.0, oss.str());
        }
    }
    else if (curTimeDiff >= mappingProcessInterval)
    {
        if (diagnostics)
            diagnostics->recordTimeDelta(curTimeDiff);

        timeLastProcessing = timeLaserInfoCur;
        poseBeforePredictionLocal = trans2Affine3f(transformTobeMapped);

        TicToc t_updateInitialGuess;
        updateInitialGuess();
        poseAfterPredictionLocal = trans2Affine3f(transformTobeMapped);
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
        publishPredictionDebugClouds(laserCloudSurfLastDS);
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

        // Record frame metrics: time delta, prediction delta, and optimized motion delta
        if (diagnostics)
        {
            const double optimized_delta_m = static_cast<double>(lastIncrementalDeltaPoseLocal.translation().norm());
            const double prediction_delta_m = diagnostics->getLastPredictionDelta();
            const double estimated_velocity_mps = (lastTimeDiff > 0.0)
                                                      ? (lastOptimizedDeltaM / lastTimeDiff)
                                                      : 0.0;
            diagnostics->recordFrameMetrics(
                timeLaserInfoCur,
                curTimeDiff,
                lastTimeDiff,
                prediction_delta_m,
                optimized_delta_m,
                estimated_velocity_mps);
            lastOptimizedDeltaM = optimized_delta_m;
        }

        publishFrames();
        visualizeGpsConstraints();

        timeLastProcessing = timeLaserInfoCur;
        lastTimeDiff = curTimeDiff;
    }

    // Always publish TF at LiDAR callback rate (latest optimized pose with current LiDAR stamp).
    publishMapOptimizationTFs(timeLaserInfoStamp);
}

