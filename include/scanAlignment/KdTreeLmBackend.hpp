#pragma once

#include "scanAlignment/IMappingBackend.hpp"
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>
#include <opencv2/opencv.hpp>

namespace lio {

// Legacy parameter structure required for the old KD-Tree and LM approach
struct KdTreeLmConfig {
    float surroundingKeyframeSearchRadius = 50.0;
    float surroundingKeyframeMapLeafSize = 0.4;
    int surroundingKeyframeSearchNum = 50;
    
    // LM specific threshold parameters can be added here
    float edgeFeatureMinValidNum = 10;
    float surfFeatureMinValidNum = 100;
};

class KdTreeLmBackend : public IMappingBackend {
public:
    explicit KdTreeLmBackend(const KdTreeLmConfig& config);
    ~KdTreeLmBackend() override = default;

    void clearMap() override;

    void rebuildLocalMap(const std::vector<int>& keyframeIndices,
                         CloudRetriever getCloudFn,
                         const Eigen::Vector3d& currentSensorPos) override;

    void updateRollingMap(const pcl::PointCloud<PointType>::Ptr& alignedScan,
                          const Eigen::Vector3d& sensorPos) override;

    AlignmentMetrics align(const pcl::PointCloud<PointType>::Ptr& scan, 
                           float* transformTobeMapped,
                           bool isDegeneracyRun = false) override;

    pcl::PointCloud<PointType>::Ptr getLocalMapCloud() const override;
    
    size_t getMapPointCount() const override;

    const std::vector<int>& getDebugCodes() const override { return emptyDebugCodes; }
    pcl::PointCloud<PointType>::Ptr getLaserCloudOri() const override { return laserCloudOri; }

private:
    KdTreeLmConfig config_;
    std::vector<int> emptyDebugCodes;

    pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMap;
    pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMapDS;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap;

    pcl::VoxelGrid<PointType> downSizeFilterSurfMap;

    // Internal variables specific to LM Optimization ported from the old node
    pcl::PointCloud<PointType>::Ptr laserCloudOri;
    pcl::PointCloud<PointType>::Ptr coeffSel;
    cv::Mat matA, matAt, matAtA, matX, matB, matAtB;

    // Helper functions for the legacy LM alignment
    void pointAssociateToMap(PointType const * const pi, PointType * const po, float* transformTobeMapped);
    void updatePointAssociateToMap(float* transformTobeMapped);
    void surfOptimization(const pcl::PointCloud<PointType>::Ptr& scan, float* transformTobeMapped);
    void combineOptimizationCoeffs(int scanSize);
    bool LMOptimization(int iterCount, float* transformTobeMapped);
};

} // namespace lio
