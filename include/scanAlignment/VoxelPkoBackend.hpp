#pragma once

#include "scanAlignment/IMappingBackend.hpp"
#include "scanAlignment/ScanAligner.hpp"
#include "mapOptimization/VoxelMap.hpp"
#include "mapOptimization/VoxelMapConfig.hpp"
#include "scanAlignment/ProbabilisticKernelOptimizer.hpp"

namespace lio {

class VoxelPkoBackend : public IMappingBackend {
public:
    VoxelPkoBackend(const VoxelMapConfig& mapConfig, 
                    const PKOConfig& pkoConfig,
                    int max_points, 
                    float knn_distance, 
                    int cores,
                    float truncationRadius);

    ~VoxelPkoBackend() override = default;

    void clearMap() override;

    void rebuildLocalMap(const std::vector<int>& keyframeIndices,
                         CloudRetriever getCloudFn,
                         const Eigen::Vector3d& currentSensorPos) override;

    void updateRollingMap(const pcl::PointCloud<PointType>::Ptr& alignedScan,
                          const Eigen::Vector3d& sensorPos) override;

    AlignmentMetrics align(const pcl::PointCloud<PointType>::Ptr& scan, 
                           float* transformTobeMapped,
                           std::optional<AlignmentOverrideConfig> overrideConfig = std::nullopt) override;
    AlignmentOverrideConfig getAlignmentConfig() const override;

    pcl::PointCloud<PointType>::Ptr getLocalMapCloud() const override;
    
    size_t getMapPointCount() const override;

    const std::vector<int>& getDebugCodes() const override;
    pcl::PointCloud<PointType>::Ptr getLaserCloudOri() const override;
    const AlignmentTrace& getLastAlignmentTrace() const override { return lastAlignmentTrace; }

private:
    std::shared_ptr<lio::VoxelMap> voxelMap;
    
    std::shared_ptr<ScanAligner> scanAlignerPrimary;

    AlignmentTrace lastAlignmentTrace;
    
    float localMapTruncationRadius;
};

} // namespace lio
