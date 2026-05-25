// File: include/scanAlignment/ScanAligner.hpp
#pragma once

#include "utility.h"
#include <pcl/common/transforms.h>
#include <opencv2/opencv.hpp>
#include <chrono>
#include "tictoc.h"

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

class ScanAligner {
private:
    pcl::PointCloud<PointType>::Ptr mapCloud;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeMap;

    pcl::PointCloud<PointType>::Ptr laserCloudOri;
    pcl::PointCloud<PointType>::Ptr coeffSel;

    std::vector<PointType> laserCloudOriSurfVec;
    std::vector<PointType> coeffSelSurfVec;
    std::vector<uint8_t> laserCloudOriSurfFlag;
    
    float currentTransform[6];
    Eigen::Affine3f transPointAssociateToMap;

    float surfKnnMinDistance;
    int numberOfCores;

    cv::Mat matP;
    bool isDegenerate;

    uint32_t surfStageInputCount = 0;
    uint32_t surfStageKnnPassCount = 0;
    uint32_t surfStagePlaneValidCount = 0;
    uint32_t surfStageMatchedCount = 0;

    void pointAssociateToMap(PointType const *const pi, PointType *const po);
    void updatePointAssociateToMap();
    void surfOptimization(const pcl::PointCloud<PointType>::Ptr &scan);
    void combineOptimizationCoeffs(int scanSize);
    bool LMOptimization(int iterCount);

public:
    std::vector<uint8_t> laserCloudSurfKnnPassFlag;
    std::vector<uint8_t> laserCloudSurfPlaneValidFlag;
    std::vector<uint8_t> laserCloudSurfDebugCode;

    static constexpr uint8_t SURF_DEBUG_ACCEPTED = 0;
    static constexpr uint8_t SURF_DEBUG_REJECTED_NEIGHBOR_COUNT = 1;
    static constexpr uint8_t SURF_DEBUG_REJECTED_KNN_DISTANCE = 2;
    static constexpr uint8_t SURF_DEBUG_REJECTED_PLANE_INVALID = 3;
    static constexpr uint8_t SURF_DEBUG_REJECTED_LOW_WEIGHT = 4;
    static constexpr uint8_t SURF_DEBUG_NOT_OPTIMIZED = 5;

    ScanAligner(int max_points, float knn_distance, int cores);
    void setMap(const pcl::PointCloud<PointType>::Ptr &map, const pcl::KdTreeFLANN<PointType>::Ptr &kdtree);
    AlignmentMetrics align(const pcl::PointCloud<PointType>::Ptr &scan, float transformIn[6]);
    pcl::PointCloud<PointType>::Ptr getLaserCloudOri() const { return laserCloudOri; }
};