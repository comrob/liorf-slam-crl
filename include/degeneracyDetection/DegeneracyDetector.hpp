// File: include/degeneracyDetection/DegeneracyDetector.hpp
#pragma once

#include "utility.h"
#include "TwistManipulation.hpp"
#include "scanAlignment/ScanAligner.hpp"
#include <pcl/common/transforms.h>
#include <vector>
#include <string>

struct DegeneracyParams {
    float n_multiplier = 3.0f;
    float max_perturbation_angle_deg = 6.0f;
    float descriptive_number_threshold = 0.5f;
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

public:
    DegeneracyDetector(const DegeneracyParams &parameters = DegeneracyParams());

    void evalDegeneracyPerturbation(
        const float poseEulerArray[6],
        pcl::PointCloud<PointType>::Ptr cloud_scan,
        pcl::PointCloud<PointType>::Ptr cloud_map,
        std::shared_ptr<ScanAligner> scanAlignerDegeneracy);

    std::vector<TwistVector> getTwistsPerturbationsDegeneracy() const { return twists_perturbations_degeneration; }
    std::string getDegeneracyDirectionsString() const;
    std::vector<TwistVector> extractBasisFromTwists(
        const std::vector<TwistVector> &twists,
        pcl::PointCloud<PointType>::Ptr cloud_scan);

    bool isFailed() const { return failed; }
    std::string getFailReason() const { return fail_reason; }
};
