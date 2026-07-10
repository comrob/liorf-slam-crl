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
    float max_perturbation_angle_deg = 6.0f;
    float descriptive_number_threshold = 0.5f;
    float eigen_value_threshold = 0.05f;
    bool verbose = false;
};

struct DegeneracyConsistencyStats {
    uint64_t eval_count = 0;
    uint64_t hit_count = 0;
    uint64_t hit_streak = 0;
    uint64_t miss_streak = 0;
    bool detected = false;

    double hitRate() const {
        if (eval_count == 0)
            return 0.0;
        return static_cast<double>(hit_count) / static_cast<double>(eval_count);
    }
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
    DegeneracyConsistencyStats consistency_stats;

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
    
    std::vector<TwistVector> getRawTwists() const { return last_raw_twists; }
    std::vector<TwistVector> getPcaBasis() const { return last_pca_basis; }
    std::vector<TwistVector> getSparsifiedBasis() const { return last_sparsified_basis; }

    bool isDegeneracyDetected() const { return consistency_stats.detected; }
    const DegeneracyConsistencyStats& getConsistencyStats() const { return consistency_stats; }

    bool isFailed() const { return failed; }
    std::string getFailReason() const { return fail_reason; }
};
