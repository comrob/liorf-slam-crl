#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>
#include <functional>

#include "export/map_types.hpp"

namespace lio {

// Shared alignment metrics across different backends.
// Note: If this is also defined in ScanAligner.hpp, you may want to move it here 
// to avoid duplicate definitions.
struct AlignmentMetrics {
    double optimization_ms = 0.0;
    double combine_ms = 0.0;
    double lm_optimization_ms = 0.0;
    int iterations = 0;
    int final_correspondences = 0;
    uint32_t input_count = 0;
    uint32_t knn_pass_count = 0;
    uint32_t plane_valid_count = 0;
    uint32_t matched_count = 0;
    bool is_degenerate = false;
};

class IMappingBackend {
public:
    virtual ~IMappingBackend() = default;

    /**
     * @brief Completely clears the local map structures.
     */
    virtual void clearMap() = 0;

    /**
     * @brief Rebuilds the local map from a set of keyframes.
     * @param cloudKeyPoses3D The global poses of the keyframes
     * @param getCloudFn A callback or mechanism to retrieve the surface cloud for a specific keyframe index
     * @param currentSensorPos The current position to center the map (if applicable, e.g., KD-Tree radius bounds)
     */
    // To maintain loose coupling, we pass a functor that retrieves the keyframe cloud by index
    // so the backend doesn't need to know how mapOptimization stores its frames.
    using CloudRetriever = std::function<pcl::PointCloud<PointType>::Ptr(int)>;
    virtual void rebuildLocalMap(const std::vector<int>& keyframeIndices,
                                 CloudRetriever getCloudFn,
                                 const Eigen::Vector3d& currentSensorPos) = 0;

    /**
     * @brief Updates the rolling local map dynamically.
     * @param alignedScan The latest scan ready to be inserted.
     * @param sensorPos The origin of the sensor for this scan.
     */
    virtual void updateRollingMap(const pcl::PointCloud<PointType>::Ptr& alignedScan,
                                  const Eigen::Vector3d& sensorPos) = 0;

    /**
     * @brief Aligns the new scan to the current map.
     * @param scan The raw/downsampled surface scan to align.
     * @param transformTobeMapped Both the initial guess (input) and the optimized pose (output).
     * @param isDegeneracyRun Flag to indicate if this is a degeneracy check run (uses secondary optimizer if applicable).
     * @return AlignmentMetrics details of the optimization run.
     */
    virtual AlignmentMetrics align(const pcl::PointCloud<PointType>::Ptr& scan, 
                                   float* transformTobeMapped,
                                   bool isDegeneracyRun = false) = 0;

    /**
     * @brief Returns the local map point cloud for publishing/visualization.
     */
    virtual pcl::PointCloud<PointType>::Ptr getLocalMapCloud() const = 0;
    
    /**
     * @brief Returns the number of points in the map.
     */
    virtual size_t getMapPointCount() const = 0;

    /**
     * @brief Returns the debug codes from the last alignment (if available).
     */
    virtual const std::vector<int>& getDebugCodes() const = 0;

    /**
     * @brief Returns the original downsampled laser cloud from the last alignment.
     */
    virtual pcl::PointCloud<PointType>::Ptr getLaserCloudOri() const = 0;
};

} // namespace lio
