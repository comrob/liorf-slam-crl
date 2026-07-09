#pragma once
#ifndef _UTILITY_LIDAR_ODOMETRY_H_
#define _UTILITY_LIDAR_ODOMETRY_H_
#define PCL_NO_PRECOMPILE 
// <!-- liorf_yjz_lucky_boy -->
#include <rclcpp/rclcpp.hpp>
#include "liorf_diagnostics.h"

#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <common_lib.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h> 
#include <pcl_conversions/pcl_conversions.h>

#include <opencv2/opencv.hpp>
// #include <opencv/cv.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
 
#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>

#include "scanAlignment/ProbabilisticKernelOptimizer.hpp"
#include "mapOptimization/VoxelMapConfig.hpp"
#include "scanAlignment/KdTreeLmBackend.hpp"

using namespace std;

typedef pcl::PointXYZI PointType;

// <!-- liorf_localization_yjz_lucky_boy -->
inline std::shared_ptr<CommonLib::common_lib> common_lib_;

enum class SensorType { VELODYNE, OUSTER, LIVOX, ROBOSENSE, MULRAN};

enum class TranslationPredictionSource
{
    CONSTANT_VELOCITY,
    IMU
};

// enum TranslationPredictionSource to string
inline std::string TranslationPredictionSourceToString(TranslationPredictionSource v)
{
    switch (v)
    {
    case TranslationPredictionSource::CONSTANT_VELOCITY:
        return "CONSTANT_VELOCITY";
    case TranslationPredictionSource::IMU:
        return "IMU";
    default:
        return "UNKNOWN";
    }
}



class ParamServer : public rclcpp::Node
{
public:
    std::shared_ptr<LiorfDiagnostics> diagnostics;

    string history_policy;
    string reliability_policy;
    bool diagnostics_write_files_master;
    bool diagnostics_write_timing_stats;
    bool diagnostics_write_event;
    bool diagnostics_write_warnings;
    bool diagnostics_write_telemetry;
    bool diagnostics_write_time_deltas;
    bool diagnostics_write_frame_metrics;

    std::string robot_id;

    //Topics
    string pointCloudTopic;
    string imuTopic;
    string odomTopic;
    string gpsTopic;

    //Frames
    string lidarFrame;
    string baselinkFrame;
    string odometryFrame;
    string mapFrameLocal;
    string mapFrameEnu;
    string mapFrameNed;
    string ECEFframe;

    // Debug Flags
    bool debugTFs;

    // GPS Settings
    bool useImuHeadingInitialization;
    bool useGpsElevation;
    float gpsCovThreshold;
    float poseCovThreshold;
    double gps_processing_delay_sec;
    double gps_covariance_inflation_m;
    bool force_initial_gps;
    std::vector<double> manual_gps_origin;
    double manual_global_heading;
    bool gps_reject_on_invalid_status;

    // Save pcd
    bool savePCD;
    string savePCDDirectory;
    bool save_dense_gps_trajectory;
    bool save_dense_odom_trajectory;

    // Lidar Sensor Configuration
    SensorType sensor;
    TranslationPredictionSource translationPredictionSource;
    double maxTranslationPrediction;
    double minTranslationPredictionSpeed;
    bool reject_fast_turn_scans;
    double fast_turn_max_angular_speed_rad_s;

    int N_SCAN;
    int Horizon_SCAN;
    int downsampleRate;
    int point_filter_num;
    float lidarMinRange;
    float lidarMaxRange;

    // IMU
    int imuType;
    float imuRate;
    float imuAccNoise;
    float imuGyrNoise;
    float imuAccBiasN;
    float imuGyrBiasN;
    float imuGravity;
    float imuRPYWeight;
    vector<double> extRotV;
    vector<double> extRPYV;
    vector<double> extTransV;
    Eigen::Matrix3d extRot;
    Eigen::Matrix3d extRPY;
    Eigen::Vector3d extTrans;
    Eigen::Quaterniond extQRPY;
    bool autoLookupLidarToImuTf;
    std::string imuMessageFrameId;
    std::string lidarMessageFrameId;
    bool imuLidarExtrinsicsResolved = false;
    double lastImuLidarLookupAttemptWall = -1.0;
    double lastImuLidarWaitingLogWall = -1.0;
    const double imuLidarLookupRetryPeriodSec = 5.0;
    std::mutex imuLidarExtrinsicsMutex;
    std::shared_ptr<tf2_ros::Buffer> imuLidarTfBuffer;
    std::shared_ptr<tf2_ros::TransformListener> imuLidarTfListener;

    // voxel filter paprams
    float mappingSurfLeafSize ;
    float surroundingKeyframeMapLeafSize;
    float surfKnnMinDistance;
    float loopClosureICPSurfLeafSize ;

    bool useSorFilter;
    int sorMeanK;
    float sorStddevMulThresh;

    bool enableTemporalFiltering;
    float temporalFilterRadius;

    float z_tollerance; 
    float rotation_tollerance;

    // CPU Params
    int numberOfCores;
    double mappingProcessInterval;

    // Surrounding map
    float surroundingkeyframeAddingDistThreshold; 
    float surroundingkeyframeAddingAngleThreshold; 
    float surroundingKeyframeDensity;
    float surroundingKeyframeSearchRadius;
    float localMapTruncationRadius;
    
    // Loop closure
    bool  loopClosureEnableFlag;
    float loopClosureFrequency;
    int   surroundingKeyframeSize;
    float historyKeyframeSearchRadius;
    float historyKeyframeSearchTimeDiff;
    int   historyKeyframeSearchNum;
    float historyKeyframeFitnessScore;
    bool rebuild_on_loop_closure;
    bool rebuild_on_gps_jump;
    double transformed_cloud_cache_max_age_sec;
    int cloud_info_queue_depth;
    bool drop_stale_lidar_frames;
    double max_lidar_processing_lag_sec;

    // GPS-LiDAR alignment tuning
    int gps_keyframe_search_window;
    double gps_max_constraint_dt_sec;

    // global map visualization radius
    float globalMapVisualizationSearchRadius;
    float globalMapVisualizationPoseDensity;
    float globalMapVisualizationLeafSize;

    bool enableDegeneracyDetection;

    std::string backend_type;
    lio::PKOConfig pko_config;
    lio::VoxelMapConfig voxel_map_config;
    lio::KdTreeLmConfig kdtree_lm_config;

    ParamServer(std::string node_name, const rclcpp::NodeOptions & options) : Node(node_name, options)
    {   
        declare_parameter<string>("history_policy", "history_keep_last");
        get_parameter("history_policy", history_policy);
        declare_parameter<string>("reliability_policy", "reliability_reliable");
        get_parameter("reliability_policy", reliability_policy);
        declare_parameter<bool>("diagnostics_write_files_master", true);
        get_parameter("diagnostics_write_files_master", diagnostics_write_files_master);
        declare_parameter<bool>("diagnostics_write_timing_stats", true);
        get_parameter("diagnostics_write_timing_stats", diagnostics_write_timing_stats);
        declare_parameter<bool>("diagnostics_write_event", true);
        get_parameter("diagnostics_write_event", diagnostics_write_event);
        declare_parameter<bool>("diagnostics_write_warnings", true);
        get_parameter("diagnostics_write_warnings", diagnostics_write_warnings);
        declare_parameter<bool>("diagnostics_write_telemetry", true);
        get_parameter("diagnostics_write_telemetry", diagnostics_write_telemetry);
        declare_parameter<bool>("diagnostics_write_time_deltas", true);
        get_parameter("diagnostics_write_time_deltas", diagnostics_write_time_deltas);
        declare_parameter<bool>("diagnostics_write_frame_metrics", true);
        get_parameter("diagnostics_write_frame_metrics", diagnostics_write_frame_metrics);

        declare_parameter<string>("pointCloudTopic", "/points_raw");
        get_parameter("pointCloudTopic", pointCloudTopic);
        declare_parameter<string>("imuTopic", "/imu_correct");
        get_parameter("imuTopic", imuTopic);
        declare_parameter<string>("odomTopic", "/odometry/imu");
        get_parameter("odomTopic", odomTopic);
        declare_parameter<string>("gpsTopic", "/odometry/gps");
        get_parameter("gpsTopic", gpsTopic);

        declare_parameter<string>("lidarFrame", "base_link");
        get_parameter("lidarFrame", lidarFrame);
        declare_parameter<string>("baselinkFrame", "base_link");
        get_parameter("baselinkFrame", baselinkFrame);
        declare_parameter<string>("odometryFrame", "odom");
        get_parameter("odometryFrame", odometryFrame);
        declare_parameter<string>("mapFrameLocal", "map_local");
        get_parameter("mapFrameLocal", mapFrameLocal);
        declare_parameter<string>("mapFrameEnu", "map");
        get_parameter("mapFrameEnu", mapFrameEnu);
        declare_parameter<string>("mapFrameNed", "map_ned");
        get_parameter("mapFrameNed", mapFrameNed);
        declare_parameter<string>("ECEFframe", "earth");
        get_parameter("ECEFframe", ECEFframe);

        declare_parameter<bool>("debugTFs", false);
        get_parameter("debugTFs", debugTFs);

        declare_parameter<bool>("useImuHeadingInitialization", false);
        get_parameter("useImuHeadingInitialization", useImuHeadingInitialization);
        declare_parameter<bool>("useGpsElevation", false);
        get_parameter("useGpsElevation", useGpsElevation);
        declare_parameter<float>("gpsCovThreshold", 2.0f);
        get_parameter("gpsCovThreshold", gpsCovThreshold);
        declare_parameter<float>("poseCovThreshold", 25.0f);
        get_parameter("poseCovThreshold", poseCovThreshold);
        declare_parameter<double>("gps_processing_delay_sec", 0.0);
        get_parameter("gps_processing_delay_sec", gps_processing_delay_sec);
        declare_parameter<double>("gps_covariance_inflation_m", 2.0);
        get_parameter("gps_covariance_inflation_m", gps_covariance_inflation_m);
        declare_parameter<bool>("force_initial_gps", false);
        get_parameter("force_initial_gps", force_initial_gps);
        declare_parameter("manual_gps_origin", std::vector<double>{0.0, 0.0, 0.0});
        get_parameter("manual_gps_origin", manual_gps_origin);
        declare_parameter("manual_global_heading", 0.0);
        get_parameter("manual_global_heading", manual_global_heading);
        declare_parameter<bool>("gps_reject_on_invalid_status", false);
        get_parameter("gps_reject_on_invalid_status", gps_reject_on_invalid_status);

        declare_parameter<bool>("savePCD", false);
        get_parameter("savePCD", savePCD);
        declare_parameter<string>("savePCDDirectory", "/Downloads/LOAM/");
        get_parameter("savePCDDirectory", savePCDDirectory);
        declare_parameter<bool>("save_dense_gps_trajectory", true);
        get_parameter("save_dense_gps_trajectory", save_dense_gps_trajectory);
        declare_parameter<bool>("save_dense_odom_trajectory", true);
        get_parameter("save_dense_odom_trajectory", save_dense_odom_trajectory);

        // degeneracy detection and handling
        declare_parameter<bool>("enableDegeneracyDetection", false);
        get_parameter("enableDegeneracyDetection", enableDegeneracyDetection);

        std::string sensorStr;
        declare_parameter<string>("sensor", " ");
        get_parameter("sensor", sensorStr);
        if (sensorStr == "velodyne")
        {
            sensor = SensorType::VELODYNE;
        }
        else if (sensorStr == "ouster")
        {
            sensor = SensorType::OUSTER;
        }
        else if (sensorStr == "livox")
        {
            sensor = SensorType::LIVOX;
        } else if  (sensorStr == "robosense") {
            sensor = SensorType::ROBOSENSE;
        }
        else if (sensorStr == "mulran")
        {
            sensor = SensorType::MULRAN;
        } 
        else {
            RCLCPP_ERROR_STREAM(
                get_logger(),
                "Invalid sensor type (must be either 'velodyne' or 'ouster' or 'livox' or 'robosense' or 'mulran'): " << sensorStr);
            rclcpp::shutdown();
        }

        std::string translationPredictionSourceStrInput = declare_parameter<string>("translationPredictionSource", "");

        // 2. Logic remains largely the same
        if (translationPredictionSourceStrInput == "constant_velocity")
        {
            translationPredictionSource = TranslationPredictionSource::CONSTANT_VELOCITY;
        }
        else if (translationPredictionSourceStrInput == "imu")
        {
            translationPredictionSource = TranslationPredictionSource::IMU;
        }
        else
        {
            // Set to default(imu) and print error
            translationPredictionSource = TranslationPredictionSource::IMU;
            
            // ROS 2 Error Log
            RCLCPP_ERROR_STREAM(this->get_logger(),
                "Invalid translation prediction source (must be either 'constant_velocity' or 'imu'): " << translationPredictionSourceStrInput);
        }

        // 3. LOG result
        std::string translationPredictionSourceStr = TranslationPredictionSourceToString(translationPredictionSource);
        RCLCPP_INFO_STREAM(this->get_logger(), "Translation Prediction Source: " << translationPredictionSourceStr);

        declare_parameter<double>("maxTranslationPrediction", 5.0);
        get_parameter("maxTranslationPrediction", maxTranslationPrediction);
        declare_parameter<double>("minTranslationPredictionSpeed", 0.0);
        get_parameter("minTranslationPredictionSpeed", minTranslationPredictionSpeed);
        declare_parameter<bool>("reject_fast_turn_scans", false);
        get_parameter("reject_fast_turn_scans", reject_fast_turn_scans);
        declare_parameter<double>("fast_turn_max_angular_speed_rad_s", 3.0);
        get_parameter("fast_turn_max_angular_speed_rad_s", fast_turn_max_angular_speed_rad_s);


        declare_parameter<int>("N_SCAN", 16);
        get_parameter("N_SCAN", N_SCAN);
        declare_parameter<int>("Horizon_SCAN", 1800);
        get_parameter("Horizon_SCAN", Horizon_SCAN);
        declare_parameter<int>("downsampleRate", 1);
        get_parameter("downsampleRate", downsampleRate);
        declare_parameter<int>("point_filter_num", 3);
        get_parameter("point_filter_num", point_filter_num);
        declare_parameter<float>("lidarMinRange", 1.0f);
        get_parameter("lidarMinRange", lidarMinRange);
        declare_parameter<float>("lidarMaxRange", 1000.0f);
        get_parameter("lidarMaxRange", lidarMaxRange);

        declare_parameter<int>("imuType", 0);
        get_parameter("imuType", imuType);
        declare_parameter<float>("imuRate", 500.0f);
        get_parameter("imuRate", imuRate);
        declare_parameter<float>("imuAccNoise", 0.01f);
        get_parameter("imuAccNoise", imuAccNoise);
        declare_parameter<float>("imuGyrNoise", 0.001f);
        get_parameter("imuGyrNoise", imuGyrNoise);
        declare_parameter<float>("imuAccBiasN", 0.0002f);
        get_parameter("imuAccBiasN", imuAccBiasN);
        declare_parameter<float>("imuGyrBiasN", 0.00003f);
        get_parameter("imuGyrBiasN", imuGyrBiasN);
        declare_parameter<float>("imuGravity", 9.80511f);
        get_parameter("imuGravity", imuGravity);
        declare_parameter<float>("imuRPYWeight", 0.01f);
        get_parameter("imuRPYWeight", imuRPYWeight);
        declare_parameter<bool>("autoLookupLidarToImuTf", false);
        get_parameter("autoLookupLidarToImuTf", autoLookupLidarToImuTf);

        double ida[] = { 1.0,  0.0,  0.0,
                         0.0,  1.0,  0.0,
                         0.0,  0.0,  1.0};
        std::vector < double > id(ida, std::end(ida));
        declare_parameter("extrinsicRot", id);
        get_parameter("extrinsicRot", extRotV);
        declare_parameter("extrinsicRPY", id);
        get_parameter("extrinsicRPY", extRPYV);
        double zea[] = {0.0, 0.0, 0.0};
        std::vector < double > ze(zea, std::end(zea));
        declare_parameter("extrinsicTrans", ze);
        get_parameter("extrinsicTrans", extTransV);

        extRot = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRotV.data(), 3, 3);
        extRPY = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRPYV.data(), 3, 3);
        extTrans = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extTransV.data(), 3, 1);
        extQRPY = Eigen::Quaterniond(extRPY).inverse();

        if (autoLookupLidarToImuTf)
        {
            imuLidarTfBuffer = std::make_shared<tf2_ros::Buffer>(get_clock());
            imuLidarTfListener = std::make_shared<tf2_ros::TransformListener>(*imuLidarTfBuffer);
            RCLCPP_INFO_STREAM(
                get_logger(),
                "[IMU_LIDAR_TF_INIT] auto lookup enabled; awaiting IMU and LiDAR message frame ids");
        }
        else
        {
            imuLidarExtrinsicsResolved = true;
        }

        declare_parameter<float>("mappingSurfLeafSize", 0.2f);
        get_parameter("mappingSurfLeafSize", mappingSurfLeafSize);
        declare_parameter<float>("z_tollerance", 1000.0f);
        get_parameter("z_tollerance", z_tollerance);
        declare_parameter<float>("rotation_tollerance", 1000.0f);
        get_parameter("rotation_tollerance", rotation_tollerance);

        declare_parameter<int>("numberOfCores", 2);
        get_parameter("numberOfCores", numberOfCores);
        declare_parameter<double>("mappingProcessInterval", 0.15f);
        get_parameter("mappingProcessInterval", mappingProcessInterval);

        declare_parameter<float>("surroundingkeyframeAddingDistThreshold", 1.0f);
        get_parameter("surroundingkeyframeAddingDistThreshold", surroundingkeyframeAddingDistThreshold);
        declare_parameter<float>("surroundingkeyframeAddingAngleThreshold", 0.2f);
        get_parameter("surroundingkeyframeAddingAngleThreshold", surroundingkeyframeAddingAngleThreshold);
        declare_parameter<float>("surroundingKeyframeDensity", 1.0f);
        get_parameter("surroundingKeyframeDensity", surroundingKeyframeDensity);
        declare_parameter<float>("loopClosureICPSurfLeafSize", 0.3f);
        get_parameter("loopClosureICPSurfLeafSize", loopClosureICPSurfLeafSize);

        declare_parameter<bool>("useSorFilter", false);
        get_parameter("useSorFilter", useSorFilter);
        declare_parameter<int>("sorMeanK", 5);
        get_parameter("sorMeanK", sorMeanK);
        declare_parameter<float>("sorStddevMulThresh", 1.0f);
        get_parameter("sorStddevMulThresh", sorStddevMulThresh);

        declare_parameter("enable_temporal_filtering", false);
        get_parameter("enable_temporal_filtering", enableTemporalFiltering);

        declare_parameter("temporal_filter_radius", 0.2);
        get_parameter("temporal_filter_radius", temporalFilterRadius);

        declare_parameter<float>("surroundingKeyframeSearchRadius", 50.0f);
        get_parameter("surroundingKeyframeSearchRadius", surroundingKeyframeSearchRadius);
        declare_parameter<float>("localMapTruncationRadius", 50.0f);
        get_parameter("localMapTruncationRadius", localMapTruncationRadius);

        declare_parameter<bool>("loopClosureEnableFlag", false);
        get_parameter("loopClosureEnableFlag", loopClosureEnableFlag);
        declare_parameter<float>("loopClosureFrequency", 1.0f);
        get_parameter("loopClosureFrequency", loopClosureFrequency);
        declare_parameter<int>("surroundingKeyframeSize", 50);
        get_parameter("surroundingKeyframeSize", surroundingKeyframeSize);
        declare_parameter<float>("historyKeyframeSearchRadius", 10.0f);
        get_parameter("historyKeyframeSearchRadius", historyKeyframeSearchRadius);
        declare_parameter<float>("historyKeyframeSearchTimeDiff", 30.0f);
        get_parameter("historyKeyframeSearchTimeDiff", historyKeyframeSearchTimeDiff);
        declare_parameter<int>("historyKeyframeSearchNum", 25);
        get_parameter("historyKeyframeSearchNum", historyKeyframeSearchNum);
        declare_parameter<float>("historyKeyframeFitnessScore", 0.3f);
        get_parameter("historyKeyframeFitnessScore", historyKeyframeFitnessScore);
        declare_parameter<bool>("rebuild_on_loop_closure", true);
        get_parameter("rebuild_on_loop_closure", rebuild_on_loop_closure);
        declare_parameter<bool>("rebuild_on_gps_jump", true);
        get_parameter("rebuild_on_gps_jump", rebuild_on_gps_jump);
        declare_parameter<double>("transformed_cloud_cache_max_age_sec", 0.5);
        get_parameter("transformed_cloud_cache_max_age_sec", transformed_cloud_cache_max_age_sec);
        declare_parameter<int>("cloud_info_queue_depth", 5);
        get_parameter("cloud_info_queue_depth", cloud_info_queue_depth);
        declare_parameter<bool>("drop_stale_lidar_frames", true);
        get_parameter("drop_stale_lidar_frames", drop_stale_lidar_frames);
        declare_parameter<double>("max_lidar_processing_lag_sec", 0.5);
        get_parameter("max_lidar_processing_lag_sec", max_lidar_processing_lag_sec);

        declare_parameter<int>("gps_keyframe_search_window", 10);
        get_parameter("gps_keyframe_search_window", gps_keyframe_search_window);
        declare_parameter<double>("gps_max_constraint_dt_sec", 0.30);
        get_parameter("gps_max_constraint_dt_sec", gps_max_constraint_dt_sec);


        declare_parameter<float>("globalMapVisualizationSearchRadius", 1e3f);
        get_parameter("globalMapVisualizationSearchRadius", globalMapVisualizationSearchRadius);
        declare_parameter<float>("globalMapVisualizationPoseDensity", 10.0);
        get_parameter("globalMapVisualizationPoseDensity", globalMapVisualizationPoseDensity);
        declare_parameter<float>("globalMapVisualizationLeafSize", 1.0f);
        get_parameter("globalMapVisualizationLeafSize", globalMapVisualizationLeafSize);
        
        // ==========================================================
        // Backend and VoxelPko Parameters
        // ==========================================================
        declare_parameter<string>("mapping.backend_type", "voxel_pko");
        get_parameter("mapping.backend_type", backend_type);

        declare_parameter<int>("mapping.voxel_pko.hierarchy_factor", 3);
        get_parameter("mapping.voxel_pko.hierarchy_factor", voxel_map_config.hierarchy_factor);

        declare_parameter<float>("mapping.voxel_pko.planarity_threshold", 0.1f);
        get_parameter("mapping.voxel_pko.planarity_threshold", voxel_map_config.planarity_threshold);

        declare_parameter<float>("mapping.voxel_pko.point_to_surfel_threshold", 0.1f);
        get_parameter("mapping.voxel_pko.point_to_surfel_threshold", voxel_map_config.point_to_surfel_threshold);

        declare_parameter<int>("mapping.voxel_pko.min_surfel_inliers", 3);
        get_parameter("mapping.voxel_pko.min_surfel_inliers", voxel_map_config.min_surfel_inliers);

        declare_parameter<float>("mapping.voxel_pko.min_linearity_ratio", 0.3f);
        get_parameter("mapping.voxel_pko.min_linearity_ratio", voxel_map_config.min_linearity_ratio);

        declare_parameter<float>("mapping.voxel_pko.map_box_multiplier", 2.0f);
        get_parameter("mapping.voxel_pko.map_box_multiplier", voxel_map_config.map_box_multiplier);

        declare_parameter("mapping.voxel_pko.use_adaptive", true);
        declare_parameter("mapping.voxel_pko.min_scale_factor", 0.001);
        declare_parameter("mapping.voxel_pko.max_scale_factor", 10.0);
        declare_parameter("mapping.voxel_pko.num_alpha_segments", 25);
        declare_parameter("mapping.voxel_pko.truncated_threshold", 10.0);
        declare_parameter("mapping.voxel_pko.gmm_components", 2);
        declare_parameter("mapping.voxel_pko.gmm_sample_size", 100);

        get_parameter("mapping.voxel_pko.use_adaptive", pko_config.use_adaptive);
        get_parameter("mapping.voxel_pko.min_scale_factor", pko_config.min_scale_factor);
        get_parameter("mapping.voxel_pko.max_scale_factor", pko_config.max_scale_factor);
        get_parameter("mapping.voxel_pko.num_alpha_segments", pko_config.num_alpha_segments);
        get_parameter("mapping.voxel_pko.truncated_threshold", pko_config.truncated_threshold);
        get_parameter("mapping.voxel_pko.gmm_components", pko_config.gmm_components);
        get_parameter("mapping.voxel_pko.gmm_sample_size", pko_config.gmm_sample_size);
        
        // ==========================================================
        // KdTreeLm Parameters
        // ==========================================================
        declare_parameter<float>("mapping.kdtree_lm.surroundingKeyframeMapLeafSize", 0.4f);
        get_parameter("mapping.kdtree_lm.surroundingKeyframeMapLeafSize", kdtree_lm_config.surroundingKeyframeMapLeafSize);

        declare_parameter<int>("mapping.kdtree_lm.surroundingKeyframeSearchNum", 50);
        get_parameter("mapping.kdtree_lm.surroundingKeyframeSearchNum", kdtree_lm_config.surroundingKeyframeSearchNum);

        declare_parameter<float>("mapping.kdtree_lm.surfKnnMinDistance", 5.0f);
        get_parameter("mapping.kdtree_lm.surfKnnMinDistance", kdtree_lm_config.surfKnnMinDistance);

        declare_parameter<float>("mapping.kdtree_lm.edgeFeatureMinValidNum", 10.0f);
        get_parameter("mapping.kdtree_lm.edgeFeatureMinValidNum", kdtree_lm_config.edgeFeatureMinValidNum);

        declare_parameter<float>("mapping.kdtree_lm.surfFeatureMinValidNum", 100.0f);
        get_parameter("mapping.kdtree_lm.surfFeatureMinValidNum", kdtree_lm_config.surfFeatureMinValidNum);
        

        usleep(100);
    }

    void noteImuMessageFrameId(const std::string &frameId)
    {
        if (!autoLookupLidarToImuTf || frameId.empty())
            return;

        std::lock_guard<std::mutex> lock(imuLidarExtrinsicsMutex);
        if (imuMessageFrameId == frameId)
            return;

        if (!imuMessageFrameId.empty() && imuMessageFrameId != frameId)
        {
            RCLCPP_WARN_STREAM(
                get_logger(),
                "[IMU_LIDAR_TF_FRAME_CHANGE] imu frame changed from '" << imuMessageFrameId
                << "' to '" << frameId << "' before extrinsics were resolved");
        }

        imuMessageFrameId = frameId;
    }

    void noteLidarMessageFrameId(const std::string &frameId)
    {
        if (!autoLookupLidarToImuTf || frameId.empty())
            return;

        std::lock_guard<std::mutex> lock(imuLidarExtrinsicsMutex);
        if (lidarMessageFrameId == frameId)
            return;

        if (!lidarMessageFrameId.empty() && lidarMessageFrameId != frameId)
        {
            RCLCPP_WARN_STREAM(
                get_logger(),
                "[IMU_LIDAR_TF_FRAME_CHANGE] lidar frame changed from '" << lidarMessageFrameId
                << "' to '" << frameId << "' before extrinsics were resolved");
        }

        lidarMessageFrameId = frameId;
    }

    bool ensureImuLidarExtrinsicsResolved(const char *context)
    {
        if (!autoLookupLidarToImuTf)
            return true;

        std::lock_guard<std::mutex> lock(imuLidarExtrinsicsMutex);
        if (imuLidarExtrinsicsResolved)
            return true;

        const auto logWaitingState = [&](const std::string &reason) {
            const double nowWallSec = this->now().seconds();
            if (lastImuLidarWaitingLogWall >= 0.0 &&
                (nowWallSec - lastImuLidarWaitingLogWall) < imuLidarLookupRetryPeriodSec)
            {
                return;
            }
            lastImuLidarWaitingLogWall = nowWallSec;

            std::ostringstream ss;
            ss << "[IMU_LIDAR_TF_LOOKUP_WAIT] (" << context << ") " << reason;
            if (!imuMessageFrameId.empty() || !lidarMessageFrameId.empty())
            {
                ss << " imu_frame='" << (imuMessageFrameId.empty() ? "<unset>" : imuMessageFrameId) << "'"
                   << " lidar_frame='" << (lidarMessageFrameId.empty() ? "<unset>" : lidarMessageFrameId) << "'";
            }
            RCLCPP_WARN_STREAM(get_logger(), ss.str());
        };

        if (imuMessageFrameId.empty() || lidarMessageFrameId.empty())
        {
            logWaitingState("awaiting IMU and LiDAR message frame ids");
            return false;
        }

        const double nowWallSec = this->now().seconds();
        if (lastImuLidarLookupAttemptWall >= 0.0 &&
            (nowWallSec - lastImuLidarLookupAttemptWall) < imuLidarLookupRetryPeriodSec)
        {
            logWaitingState("waiting before the next static TF lookup retry");
            return false;
        }
        lastImuLidarLookupAttemptWall = nowWallSec;

        try
        {
            geometry_msgs::msg::TransformStamped imuToLidarMsg =
                imuLidarTfBuffer->lookupTransform(lidarMessageFrameId, imuMessageFrameId, rclcpp::Time(0));

            tf2::Transform imuToLidarTf;
            tf2::fromMsg(imuToLidarMsg.transform, imuToLidarTf);
            tf2::Transform lidarToImuTf = imuToLidarTf.inverse();
            tf2::Matrix3x3 imuToLidarRot(imuToLidarTf.getRotation());

            for (int row = 0; row < 3; ++row)
            {
                for (int col = 0; col < 3; ++col)
                {
                    extRot(row, col) = imuToLidarRot[row][col];
                    extRPY(row, col) = imuToLidarRot[row][col];
                }
            }

            extTrans = Eigen::Vector3d(
                lidarToImuTf.getOrigin().x(),
                lidarToImuTf.getOrigin().y(),
                lidarToImuTf.getOrigin().z());
            extQRPY = Eigen::Quaterniond(extRPY).inverse();

            extRotV.clear();
            extRPYV.clear();
            for (int row = 0; row < 3; ++row)
            {
                for (int col = 0; col < 3; ++col)
                {
                    extRotV.push_back(extRot(row, col));
                    extRPYV.push_back(extRPY(row, col));
                }
            }
            extTransV = {extTrans.x(), extTrans.y(), extTrans.z()};

            imuLidarExtrinsicsResolved = true;

            const auto &rot = imuToLidarMsg.transform.rotation;
            RCLCPP_INFO_STREAM(
                get_logger(),
                "[IMU_LIDAR_TF_LOOKUP_OK] (" << context << ") imu_frame='" << imuMessageFrameId
                << "' lidar_frame='" << lidarMessageFrameId << "'"
                << " resolved_tf=lookupTransform(target='" << lidarMessageFrameId
                << "', source='" << imuMessageFrameId << "')"
                << " -> imu_to_lidar"
                << " extTrans_lidar_to_imu=(" << extTrans.x() << ", " << extTrans.y() << ", " << extTrans.z() << ")"
                << " extRot_imu_to_lidar_quat_xyzw=(" << rot.x << ", " << rot.y << ", " << rot.z << ", " << rot.w << ")");
            return true;
        }
        catch (const tf2::TransformException &ex)
        {
            logWaitingState(std::string("static TF lookup failed: ") + ex.what());
            RCLCPP_ERROR_STREAM(
                get_logger(),
                "[IMU_LIDAR_TF_LOOKUP_FAIL] (" << context << ")"
                << " imu_frame='" << imuMessageFrameId << "'"
                << " lidar_frame='" << lidarMessageFrameId << "'"
                << " reason=" << ex.what()
                << "; localization outputs remain disabled; specify manual extrinsicRot/extrinsicRPY/extrinsicTrans if this TF is not published");
            return false;
        }
    }

    sensor_msgs::msg::Imu imuConverter(const sensor_msgs::msg::Imu& imu_in)
    {
        sensor_msgs::msg::Imu imu_out = imu_in;
        // rotate acceleration
        Eigen::Vector3d acc(imu_in.linear_acceleration.x, imu_in.linear_acceleration.y, imu_in.linear_acceleration.z);
        acc = extRot * acc;
        imu_out.linear_acceleration.x = acc.x();
        imu_out.linear_acceleration.y = acc.y();
        imu_out.linear_acceleration.z = acc.z();
        // rotate gyroscope
        Eigen::Vector3d gyr(imu_in.angular_velocity.x, imu_in.angular_velocity.y, imu_in.angular_velocity.z);
        gyr = extRot * gyr;
        imu_out.angular_velocity.x = gyr.x();
        imu_out.angular_velocity.y = gyr.y();
        imu_out.angular_velocity.z = gyr.z();

        if (imuType) {
            // rotate roll pitch yaw
            Eigen::Quaterniond q_from(imu_in.orientation.w, imu_in.orientation.x, imu_in.orientation.y, imu_in.orientation.z);
            Eigen::Quaterniond q_final = q_from * extQRPY;
            imu_out.orientation.x = q_final.x();
            imu_out.orientation.y = q_final.y();
            imu_out.orientation.z = q_final.z();
            imu_out.orientation.w = q_final.w();

            if (sqrt(q_final.x()*q_final.x() + q_final.y()*q_final.y() + q_final.z()*q_final.z() + q_final.w()*q_final.w()) < 0.1)
            {
                RCLCPP_ERROR(get_logger(), "Invalid quaternion, please use a 9-axis IMU!");
                rclcpp::shutdown();
            }
        }

        return imu_out;
    }
};

template<typename T>
sensor_msgs::msg::PointCloud2 publishCloud(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &thisPub, const T& thisCloud, rclcpp::Time thisStamp, std::string thisFrame)
{
    sensor_msgs::msg::PointCloud2 tempCloud;
    pcl::toROSMsg(*thisCloud, tempCloud);
    tempCloud.header.stamp = thisStamp;
    tempCloud.header.frame_id = thisFrame;
    if (thisPub->get_subscription_count() != 0)
        thisPub->publish(tempCloud);

    return tempCloud;
}

template<typename T>
double ROS_TIME(T msg)
{
    return rclcpp::Time(msg).seconds();
}


template<typename T>
void imuAngular2rosAngular(sensor_msgs::msg::Imu *thisImuMsg, T *angular_x, T *angular_y, T *angular_z)
{
    *angular_x = thisImuMsg->angular_velocity.x;
    *angular_y = thisImuMsg->angular_velocity.y;
    *angular_z = thisImuMsg->angular_velocity.z;
}


template<typename T>
void imuAccel2rosAccel(sensor_msgs::msg::Imu *thisImuMsg, T *acc_x, T *acc_y, T *acc_z)
{
    *acc_x = thisImuMsg->linear_acceleration.x;
    *acc_y = thisImuMsg->linear_acceleration.y;
    *acc_z = thisImuMsg->linear_acceleration.z;
}


template<typename T>
void imuRPY2rosRPY(sensor_msgs::msg::Imu *thisImuMsg, T *rosRoll, T *rosPitch, T *rosYaw)
{
    double imuRoll, imuPitch, imuYaw;
    tf2::Quaternion orientation;
    tf2::fromMsg(thisImuMsg->orientation, orientation);
    tf2::Matrix3x3(orientation).getRPY(imuRoll, imuPitch, imuYaw);

    *rosRoll = imuRoll;
    *rosPitch = imuPitch;
    *rosYaw = imuYaw;
}

inline rclcpp::QoS QosPolicy(const string &history_policy, const string &reliability_policy)
{
    rmw_qos_profile_t qos_profile = rmw_qos_profile_default;
    if (history_policy == "history_keep_last")
        qos_profile.history = rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    else if (history_policy == "history_keep_all")
        qos_profile.history = rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_KEEP_ALL;

    if (reliability_policy == "reliability_reliable")
        qos_profile.reliability = rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    else if (reliability_policy == "reliability_best_effort")
        qos_profile.reliability = rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;

    qos_profile.depth = 5;

    qos_profile.durability = rmw_qos_durability_policy_t::RMW_QOS_POLICY_DURABILITY_VOLATILE;
    qos_profile.deadline = RMW_QOS_DEADLINE_DEFAULT;
    qos_profile.lifespan = RMW_QOS_LIFESPAN_DEFAULT;
    qos_profile.liveliness = rmw_qos_liveliness_policy_t::RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT;
    qos_profile.liveliness_lease_duration = RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT;
    qos_profile.avoid_ros_namespace_conventions = false;

    return rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, qos_profile.depth), qos_profile);
}

#endif
