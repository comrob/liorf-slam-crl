#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>
#include <functional>
#include <optional>
#include <array>
#include <string>

#include "export/map_types.hpp"

namespace lio {

struct JacobianDegeneracyInfo {
    bool computed = false;
    bool is_degenerate = false;
    int selected_correspondences = 0;
    std::string reason;
    std::array<float, 6> eigenvalues = {0.f, 0.f, 0.f, 0.f, 0.f, 0.f};
    std::array<float, 6> thresholds = {0.f, 0.f, 0.f, 0.f, 0.f, 0.f};
    std::array<int, 6> zeroed_modes = {0, 0, 0, 0, 0, 0};
    // Row-major 6x6 matrix. For cv::eigen this stores eigenvectors row-wise.
    std::array<float, 36> eigenvectors = {
        0.f, 0.f, 0.f, 0.f, 0.f, 0.f,
        0.f, 0.f, 0.f, 0.f, 0.f, 0.f,
        0.f, 0.f, 0.f, 0.f, 0.f, 0.f,
        0.f, 0.f, 0.f, 0.f, 0.f, 0.f,
        0.f, 0.f, 0.f, 0.f, 0.f, 0.f,
        0.f, 0.f, 0.f, 0.f, 0.f, 0.f
    };
};

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
    JacobianDegeneracyInfo jacobian_degeneracy;
};

struct AlignmentTrace {
    std::vector<Eigen::Matrix4f> iteration_poses;
    bool converged = false;
};

struct PlaneNormalSample {
    PointType point_map;
    Eigen::Vector3f normal_map = Eigen::Vector3f::Zero();
    Eigen::Vector3f residual_vector_map = Eigen::Vector3f::Zero();
};

// Per-call alignment override configuration.
// Backends may honor only the fields they support.
struct AlignmentOverrideConfig {
    std::optional<int> max_iterations;
    std::optional<bool> capture_trace;
    std::optional<bool> compute_jacobian_degeneracy;
    std::optional<float> jacobian_degeneracy_threshold;
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
        * @param overrideConfig Optional per-call alignment override configuration.
     * @return AlignmentMetrics details of the optimization run.
     */
    virtual AlignmentMetrics align(const pcl::PointCloud<PointType>::Ptr& scan, 
                                   float* transformTobeMapped,
                                std::optional<AlignmentOverrideConfig> overrideConfig = std::nullopt) = 0;

        /**
        * @brief Returns the default alignment override config for this backend.
        */
        virtual AlignmentOverrideConfig getAlignmentConfig() const = 0;

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

    /**
     * @brief Returns the trace of optimization poses from the last align() call.
     * Default behavior for backends without trace support can return an empty trace.
     */
    virtual const AlignmentTrace& getLastAlignmentTrace() const = 0;

    /**
     * @brief Returns the accepted planar correspondence points and fitted normals
     * from the last surf optimization iteration.
     */
    virtual const std::vector<PlaneNormalSample>& getLastPlaneNormalSamples() const = 0;
};

} // namespace lio
