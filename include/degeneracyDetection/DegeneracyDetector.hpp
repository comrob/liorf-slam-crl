// File: include/degeneracyDetection/DegeneracyDetector.hpp
#pragma once

#include "utility.h"
#include "TwistManipulation.hpp"
#include "scanAlignment/IMappingBackend.hpp"
#include <pcl/common/transforms.h>
#include <vector>
#include <string>
#include <cstdint>

struct DegeneracyParams {
    float n_multiplier = 3.0f;
    int max_icp_steps = 4;
    float max_perturbation_angle_deg = 6.0f;
    float descriptive_number_threshold = 0.1f;
    float eigen_value_threshold = 0.05f;
    bool verbose = false;
};

class DegeneracyDetector {
private:
    DegeneracyParams params;
    std::vector<TwistVector> twists_perturbations_degeneration;
    std::vector<float> descriptive_numbers;
    bool failed;
    std::string fail_reason;

    std::vector<TwistVector> last_raw_twists;
    std::vector<TwistVector> last_pca_basis;
    std::vector<TwistVector> last_sparsified_basis;
    std::vector<pcl::PointCloud<PointType>::Ptr> last_perturbed_scans;
    std::vector<pcl::PointCloud<PointType>::Ptr> last_aligned_scans;
    std::vector<Eigen::Matrix4f> last_perturbed_poses;
    std::vector<Eigen::Matrix4f> last_aligned_poses;
    std::vector<std::vector<Eigen::Matrix4f>> last_optimization_paths;

public:
    DegeneracyDetector(const DegeneracyParams &parameters = DegeneracyParams());

    void evalDegeneracyPerturbation(
        const float poseEulerArray[6],
        pcl::PointCloud<PointType>::Ptr cloud_scan,
        pcl::PointCloud<PointType>::Ptr cloud_map,
        std::shared_ptr<lio::IMappingBackend> mappingBackend);

    std::vector<TwistVector> getTwistsPerturbationsDegeneracy() const { return twists_perturbations_degeneration; }
    
    std::string getDegeneracyDirectionsString() const;
    std::string getFinalBasisString() const;


    std::vector<TwistVector> extractBasisFromTwists(
        const std::vector<TwistVector> &twists,
        pcl::PointCloud<PointType>::Ptr cloud_scan);
    std::vector<TwistVector> orthonormalizeBasis(const std::vector<TwistVector> &basis) const;

    std::vector<TwistVector> getRawTwists() const { return last_raw_twists; }
    std::vector<TwistVector> getPcaBasis() const { return last_pca_basis; }
    std::vector<TwistVector> getSparsifiedBasis() const { return last_sparsified_basis; }
    const std::vector<pcl::PointCloud<PointType>::Ptr>& getPerturbedScans() const { return last_perturbed_scans; }
    const std::vector<pcl::PointCloud<PointType>::Ptr>& getAlignedScans() const { return last_aligned_scans; }
    const std::vector<Eigen::Matrix4f>& getPerturbedPoses() const { return last_perturbed_poses; }
    const std::vector<Eigen::Matrix4f>& getAlignedPoses() const { return last_aligned_poses; }
    const std::vector<std::vector<Eigen::Matrix4f>>& getOptimizationPaths() const { return last_optimization_paths; }

    bool isDegeneracyDetected() const { return !last_sparsified_basis.empty(); }

    bool isFailed() const { return failed; }
    std::string getFailReason() const { return fail_reason; }
};
