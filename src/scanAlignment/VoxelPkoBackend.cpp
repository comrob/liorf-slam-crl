#include "scanAlignment/VoxelPkoBackend.hpp"
#include <pcl/common/transforms.h>
#include "common_lib.h"

namespace lio {

VoxelPkoBackend::VoxelPkoBackend(const VoxelMapConfig& mapConfig, 
                                 const PKOConfig& pkoConfig,
                                 int max_points, 
                                 float knn_distance, 
                                 int cores,
                                 float truncationRadius)
{
    localMapTruncationRadius = truncationRadius;
    voxelMap = std::make_shared<VoxelMap>(mapConfig);
    scanAlignerPrimary = std::make_shared<ScanAligner>(max_points, knn_distance, cores, pkoConfig);
    scanAlignerPrimary->setMap(voxelMap);
    
    // Create a secondary aligner for degeneracy checks
    scanAlignerDegeneracy = std::make_shared<ScanAligner>(max_points, knn_distance, cores, pkoConfig);
    scanAlignerDegeneracy->setMap(voxelMap);
}

void VoxelPkoBackend::clearMap()
{
    if (voxelMap) {
        voxelMap->Clear();
    }
}

void VoxelPkoBackend::rebuildLocalMap(const std::vector<int>& keyframeIndices,
                                      CloudRetriever getCloudFn,
                                      const Eigen::Vector3d& currentSensorPos)
{
    clearMap();
    if (keyframeIndices.empty()) {
        return;
    }
    
    for (int i : keyframeIndices)
    {
        auto transformedCurrentScan = getCloudFn(i);
        if (transformedCurrentScan && !transformedCurrentScan->empty()) {
            voxelMap->UpdateVoxelMap(transformedCurrentScan, currentSensorPos, localMapTruncationRadius, true);
        }
    }
}

void VoxelPkoBackend::updateRollingMap(const pcl::PointCloud<PointType>::Ptr& alignedScan,
                                       const Eigen::Vector3d& sensorPos)
{
    if (!alignedScan || alignedScan->empty() || !voxelMap) return;
    
    voxelMap->UpdateVoxelMap(alignedScan, sensorPos, localMapTruncationRadius, true);
}

AlignmentMetrics VoxelPkoBackend::align(const pcl::PointCloud<PointType>::Ptr& scan, 
                                        float* transformTobeMapped,
                                        bool isDegeneracyRun)
{
    if (isDegeneracyRun) {
        return scanAlignerDegeneracy->align(scan, transformTobeMapped);
    } else {
        return scanAlignerPrimary->align(scan, transformTobeMapped);
    }
}

pcl::PointCloud<PointType>::Ptr VoxelPkoBackend::getLocalMapCloud() const
{
    pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
    if (voxelMap) {
        auto centroids = voxelMap->GetL0Centroids();
        cloud->reserve(centroids.size());
        for (const auto& c : centroids) {
            PointType p;
            p.x = c.x(); p.y = c.y(); p.z = c.z();
            cloud->push_back(p);
        }
    }
    return cloud;
}

size_t VoxelPkoBackend::getMapPointCount() const
{
    if (voxelMap) {
        return voxelMap->GetVoxelCount();
    }
    return 0;
}

const std::vector<int>& VoxelPkoBackend::getDebugCodes() const
{
    return scanAlignerPrimary->getDebugCodes();
}

pcl::PointCloud<PointType>::Ptr VoxelPkoBackend::getLaserCloudOri() const
{
    return scanAlignerPrimary->getLaserCloudOri();
}

} // namespace lio
