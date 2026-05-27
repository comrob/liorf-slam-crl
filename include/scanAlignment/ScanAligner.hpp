// File: include/scanAlignment/ScanAligner.hpp
#pragma once

#include "utility.h"
#include <pcl/common/transforms.h>
#include <opencv2/opencv.hpp>
#include <chrono>
#include "tictoc.h"
#include "mapOptimization/VoxelMap.hpp"
#include "scanAlignment/ProbabilisticKernelOptimizer.hpp" // Added PKO include

// Define the debug codes globally or inside the class
constexpr int SURF_DEBUG_NOT_OPTIMIZED = 0;
constexpr int SURF_DEBUG_ACCEPTED = 1;
constexpr int SURF_DEBUG_REJECTED_NEIGHBOR_COUNT = 2;
constexpr int SURF_DEBUG_REJECTED_KNN_DISTANCE = 3;
constexpr int SURF_DEBUG_REJECTED_PLANE_INVALID = 4;
constexpr int SURF_DEBUG_REJECTED_LOW_WEIGHT = 5;

struct AlignmentMetrics {
    double surf_optimization_ms = 0.0;
    double combine_ms = 0.0;
    double lm_optimization_ms = 0.0;
    int iterations = 0;
    int final_correspondences = 0;
    uint32_t surf_input_count = 0;
    uint32_t surf_knn_pass_count = 0;
    uint32_t surf_plane_valid_count = 0;
    uint32_t surf_matched_count = 0;
    bool is_degenerate = false;
};

class ScanAligner
{
public:
    std::vector<int> laserCloudSurfDebugCode; // Needs to be public for the publisher

    ScanAligner(int max_points, float knn_distance, int cores, const lio::PKOConfig& pko_config = lio::PKOConfig());
    ~ScanAligner() = default;

    void setMap(const std::shared_ptr<lio::VoxelMap>& map);
    
    // Legacy setMap (can be removed if no longer used)
    void setMap(const pcl::PointCloud<PointType>::Ptr &map_cloud, const pcl::KdTreeFLANN<PointType>::Ptr &map_kdtree);
    
    AlignmentMetrics align(const pcl::PointCloud<PointType>::Ptr &scan, float *transform);

    pcl::PointCloud<PointType>::Ptr getLaserCloudOri() const { return laserCloudOri; }
    const std::vector<int> &getDebugCodes() const { return laserCloudSurfDebugCode; }

private:
    std::shared_ptr<lio::VoxelMap> voxelMap;
    std::shared_ptr<lio::ProbabilisticKernelOptimizer> m_pko;

    int numberOfCores;
    float surfKnnMinDistance;
    float currentTransform[6];
    Eigen::Affine3f transPointAssociateToMap;

    pcl::PointCloud<PointType>::Ptr mapCloud;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeMap;
    pcl::PointCloud<PointType>::Ptr laserCloudOri;
    pcl::PointCloud<PointType>::Ptr coeffSel;

    std::vector<int> laserCloudSurfKnnPassFlag;
    std::vector<int> laserCloudSurfPlaneValidFlag;
    std::vector<PointType> laserCloudOriSurfVec;
    std::vector<PointType> coeffSelSurfVec;
    std::vector<bool> laserCloudOriSurfFlag;

    uint32_t surfStageInputCount = 0;
    uint32_t surfStageKnnPassCount = 0;
    uint32_t surfStagePlaneValidCount = 0;
    uint32_t surfStageMatchedCount = 0;

    bool isDegenerate = false;
    cv::Mat matP; // MUST be cv::Mat to match your CPP file

    void pointAssociateToMap(PointType const *const pi, PointType *const po);
    void updatePointAssociateToMap();
    void surfOptimization(const pcl::PointCloud<PointType>::Ptr &scan);
    void combineOptimizationCoeffs(int scanSize);
    bool LMOptimization(int iterCount);
    void cornerOptimization(const pcl::PointCloud<PointType>::Ptr &scan);
};