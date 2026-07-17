// File: src/scanAlignment/ScanAligner.cpp
#include "scanAlignment/ScanAligner.hpp"
#include <omp.h>

ScanAligner::ScanAligner(int max_points, float knn_distance, int cores, const lio::PKOConfig& pko_config)
{
    numberOfCores = cores;    
    surfKnnMinDistance = knn_distance;

    m_pko = std::make_shared<lio::ProbabilisticKernelOptimizer>(pko_config);
    mapCloud = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>());
    kdtreeMap = pcl::KdTreeFLANN<PointType>::Ptr(new pcl::KdTreeFLANN<PointType>());
    laserCloudOri = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>());
    coeffSel = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>());

    laserCloudOriSurfVec.resize(max_points);
    coeffSelSurfVec.resize(max_points);
    laserCloudOriSurfFlag.resize(max_points, false);
    laserCloudSurfKnnPassFlag.resize(max_points, 0);
    laserCloudSurfPlaneValidFlag.resize(max_points, 0);
    laserCloudSurfDebugCode.resize(max_points, SURF_DEBUG_NOT_OPTIMIZED);

    for (int i = 0; i < 6; ++i) {
        currentTransform[i] = 0.0f;
    }
    
    matP = cv::Mat(6, 6, CV_32F, cv::Scalar::all(0));
    isDegenerate = false;
}

void ScanAligner::setMap(const std::shared_ptr<lio::VoxelMap>& map)
{
    voxelMap = map;
}

void ScanAligner::pointAssociateToMap(PointType const *const pi, PointType *const po)
{
    po->x = transPointAssociateToMap(0,0) * pi->x + transPointAssociateToMap(0,1) * pi->y + transPointAssociateToMap(0,2) * pi->z + transPointAssociateToMap(0,3);
    po->y = transPointAssociateToMap(1,0) * pi->x + transPointAssociateToMap(1,1) * pi->y + transPointAssociateToMap(1,2) * pi->z + transPointAssociateToMap(1,3);
    po->z = transPointAssociateToMap(2,0) * pi->x + transPointAssociateToMap(2,1) * pi->y + transPointAssociateToMap(2,2) * pi->z + transPointAssociateToMap(2,3);
    po->intensity = pi->intensity;
}

void ScanAligner::updatePointAssociateToMap()
{
    transPointAssociateToMap = pcl::getTransformation(currentTransform[3], currentTransform[4], currentTransform[5], currentTransform[0], currentTransform[1], currentTransform[2]);
}

lio::AlignmentMetrics ScanAligner::align(const pcl::PointCloud<PointType>::Ptr &scan,
                                         float transformIn[6],
                                         std::optional<lio::AlignmentOverrideConfig> overrideConfig)
{
    lio::AlignmentMetrics metrics;
    auto scanSize = scan->points.size();

    // Re-introduce the safety check:
    if (scanSize > laserCloudOriSurfVec.size()) {
        laserCloudOriSurfVec.resize(scanSize);
        coeffSelSurfVec.resize(scanSize);
        laserCloudOriSurfFlag.resize(scanSize, false);
        laserCloudSurfKnnPassFlag.resize(scanSize, 0);
        laserCloudSurfPlaneValidFlag.resize(scanSize, 0);
        laserCloudSurfDebugCode.resize(scanSize, SURF_DEBUG_NOT_OPTIMIZED);
    }

    for(int i=0; i<6; ++i) currentTransform[i] = transformIn[i];
    
    metrics.iterations = 0;

    surfStageInputCount = static_cast<uint32_t>(scanSize);
    surfStageKnnPassCount = 0;
    surfStagePlaneValidCount = 0;
    surfStageMatchedCount = 0;

    std::fill(laserCloudSurfDebugCode.begin(), laserCloudSurfDebugCode.end(), SURF_DEBUG_NOT_OPTIMIZED);

    int maxIters = 30;
    if (overrideConfig.has_value() && overrideConfig->max_iterations.has_value())
        maxIters = std::max(1, *overrideConfig->max_iterations);
    for (int iterCount = 0; iterCount < maxIters; iterCount++)
    {
        metrics.iterations++;
        laserCloudOri->clear();
        coeffSel->clear();

        TicToc t_surfOptimization;
        surfOptimization(scan);
        metrics.optimization_ms += t_surfOptimization.toc();

        TicToc t_combineOptimizationCoeffs;
        combineOptimizationCoeffs(scanSize);
        metrics.combine_ms += t_combineOptimizationCoeffs.toc();

        TicToc t_lmOptimization;
        if (LMOptimization(iterCount) == true)
        {
            metrics.lm_optimization_ms += t_lmOptimization.toc();
            break;              
        }
        metrics.lm_optimization_ms += t_lmOptimization.toc();
    }

    metrics.final_correspondences = laserCloudOri->size();
    metrics.input_count = surfStageInputCount;
    metrics.knn_pass_count = surfStageKnnPassCount;
    metrics.plane_valid_count = surfStagePlaneValidCount;
    metrics.matched_count = surfStageMatchedCount;
    metrics.is_degenerate = this->isDegenerate;

    for(int i=0; i<6; ++i) transformIn[i] = currentTransform[i];
    
    return metrics;
}

void ScanAligner::surfOptimization(const pcl::PointCloud<PointType>::Ptr &scan)
{
    if (!voxelMap) {
        throw std::runtime_error("ScanAligner::surfOptimization - voxelMap is not set.");
    }

    updatePointAssociateToMap();

    const int scanSize = scan->points.size();
    
    if (scanSize > (int)laserCloudOriSurfVec.size()) {
        laserCloudOriSurfVec.resize(scanSize);
        coeffSelSurfVec.resize(scanSize);
        laserCloudOriSurfFlag.resize(scanSize, false);
        laserCloudSurfKnnPassFlag.resize(scanSize, 0);
        laserCloudSurfPlaneValidFlag.resize(scanSize, 0);
        laserCloudSurfDebugCode.resize(scanSize, SURF_DEBUG_NOT_OPTIMIZED);
    }

    std::fill(laserCloudSurfKnnPassFlag.begin(), laserCloudSurfKnnPassFlag.begin() + scanSize, 0);
    std::fill(laserCloudSurfPlaneValidFlag.begin(), laserCloudSurfPlaneValidFlag.begin() + scanSize, 0);
    std::fill(laserCloudSurfDebugCode.begin(), laserCloudSurfDebugCode.begin() + scanSize, SURF_DEBUG_NOT_OPTIMIZED);
    std::fill(laserCloudOriSurfFlag.begin(), laserCloudOriSurfFlag.begin() + scanSize, false);

    if (m_pko) {
        m_pko->Reset(); // Reset GMM for each new LM iteration step
    }

    // ==========================================================
    // PASS 1: Parallel Correspondence & Residual Collection
    // ==========================================================
    
    // Pre-allocate to avoid thread locks during OpenMP parallel loop
    std::vector<double> raw_residuals(scanSize, 0.0);
    std::vector<Eigen::Vector3f> valid_normals(scanSize);
    std::vector<float> valid_pd2(scanSize);
    std::vector<bool> has_surfel(scanSize, false);

    int knnPassCount = 0;

    #pragma omp parallel for num_threads(numberOfCores) reduction(+:knnPassCount)
    for (int i = 0; i < scanSize; i++)
    {
        PointType pointOri = scan->points[i];
        PointType pointSel;
        pointAssociateToMap(&pointOri, &pointSel);

        Eigen::Vector3f surfel_normal, surfel_centroid;
        float planarity_score;

        // O(1) Surfel Lookup
        bool found = voxelMap->GetSurfelAtPoint(pointSel, surfel_normal, surfel_centroid, planarity_score);
        if (!found) {
            laserCloudSurfDebugCode[i] = SURF_DEBUG_REJECTED_NEIGHBOR_COUNT;
            continue;
        }

        // Calculate point-to-plane distance (residual)
        Eigen::Vector3f pt_vec(pointSel.x, pointSel.y, pointSel.z);
        float pd2 = surfel_normal.dot(pt_vec - surfel_centroid);

        // Store attributes for Pass 2 to avoid re-lookup
        valid_normals[i] = surfel_normal;
        valid_pd2[i] = pd2;
        raw_residuals[i] = static_cast<double>(pd2);
        has_surfel[i] = true;
        
        knnPassCount++;
    }

    // ==========================================================
    // PKO EVALUATION (Sequential Distribution Analysis)
    // ==========================================================
    
    double adaptive_huber_delta = 1.0;
    double residual_normalization_scale = 1.0;

    if (knnPassCount > 0) {
        // Compact the valid residuals into a dense vector
        std::vector<double> compacted_residuals;
        compacted_residuals.reserve(knnPassCount);
        for (int i = 0; i < scanSize; i++) {
            if (has_surfel[i]) {
                compacted_residuals.push_back(raw_residuals[i]);
            }
        }

        // Calculate normalization scale (StdDev / 3.0) 
        double mean = std::accumulate(compacted_residuals.begin(), compacted_residuals.end(), 0.0) / compacted_residuals.size();
        double variance = 0.0;
        for (double val : compacted_residuals) {
            variance += (val - mean) * (val - mean);
        }
        variance /= compacted_residuals.size();
        double std_dev = std::sqrt(variance);
        residual_normalization_scale = std::max(std_dev / 3.0, 1e-6);

        // Normalize residuals for PKO
        std::vector<double> normalized_residuals(compacted_residuals.size());
        for (size_t i = 0; i < compacted_residuals.size(); ++i) {
            normalized_residuals[i] = compacted_residuals[i] / residual_normalization_scale;
        }

        // Fit GMM and calculate best alpha
        if (m_pko) {
            adaptive_huber_delta = m_pko->CalculateScaleFactor(normalized_residuals);
        }
    }

    // ==========================================================
    // PASS 2: Parallel Robust Weighting & Coefficient Assignment
    // ==========================================================
    
    int planeValidCount = 0;
    int matchedCount = 0;
    float huber_threshold = static_cast<float>(adaptive_huber_delta);
    float norm_scale = static_cast<float>(residual_normalization_scale);

    #pragma omp parallel for num_threads(numberOfCores) reduction(+:planeValidCount,matchedCount)
    for (int i = 0; i < scanSize; i++)
    {
        if (!has_surfel[i]) continue;

        laserCloudSurfKnnPassFlag[i] = 1;
        laserCloudSurfPlaneValidFlag[i] = 1;
        planeValidCount++;

        float pd2 = valid_pd2[i];
        Eigen::Vector3f surfel_normal = valid_normals[i];

        // 1. Calculate normalized residual
        float abs_normalized_residual = std::abs(pd2) / norm_scale;
        
        // 2. Apply Huber robust weighting
        float weight = 1.0f;
        if (abs_normalized_residual > huber_threshold) {
            weight = huber_threshold / abs_normalized_residual;
        }

        // 3. IRLS requires sqrt(weight) because coefficients directly multiply Jacobian
        float s = std::sqrt(weight);

        PointType coeff;
        coeff.x = s * surfel_normal.x();
        coeff.y = s * surfel_normal.y();
        coeff.z = s * surfel_normal.z();
        coeff.intensity = s * pd2; // Store weighted residual

        // Reject structurally horrible outliers
        if (s > 0.1f) {
            laserCloudSurfDebugCode[i] = SURF_DEBUG_ACCEPTED;
            laserCloudOriSurfFlag[i] = true;
            laserCloudOriSurfVec[i] = scan->points[i];
            coeffSelSurfVec[i] = coeff;
            matchedCount++;
        } else {
            laserCloudSurfDebugCode[i] = SURF_DEBUG_REJECTED_LOW_WEIGHT;
        }
    }

    surfStageKnnPassCount += knnPassCount;
    surfStagePlaneValidCount += planeValidCount;
    surfStageMatchedCount += matchedCount;
}

void ScanAligner::combineOptimizationCoeffs(int scanSize)
{
    for (int i = 0; i < scanSize; ++i){
        if (laserCloudOriSurfFlag[i] == true){
            laserCloudOri->push_back(laserCloudOriSurfVec[i]);
            coeffSel->push_back(coeffSelSurfVec[i]);
        }
    }
    std::fill(laserCloudOriSurfFlag.begin(), laserCloudOriSurfFlag.begin() + scanSize, false);
}

bool ScanAligner::LMOptimization(int iterCount)
{
    float srx = sin(currentTransform[2]);
    float crx = cos(currentTransform[2]);
    float sry = sin(currentTransform[1]);
    float cry = cos(currentTransform[1]);
    float srz = sin(currentTransform[0]);
    float crz = cos(currentTransform[0]);

    int laserCloudSelNum = laserCloudOri->size();
    if (laserCloudSelNum < 50) {
        return false;
    }

    cv::Mat matA(laserCloudSelNum, 6, CV_32F, cv::Scalar::all(0));
    cv::Mat matAt(6, laserCloudSelNum, CV_32F, cv::Scalar::all(0));
    cv::Mat matAtA(6, 6, CV_32F, cv::Scalar::all(0));
    cv::Mat matB(laserCloudSelNum, 1, CV_32F, cv::Scalar::all(0));
    cv::Mat matAtB(6, 1, CV_32F, cv::Scalar::all(0));
    cv::Mat matX(6, 1, CV_32F, cv::Scalar::all(0));

    PointType pointOri, coeff;

    for (int i = 0; i < laserCloudSelNum; i++) {
        pointOri.x = laserCloudOri->points[i].x;
        pointOri.y = laserCloudOri->points[i].y;
        pointOri.z = laserCloudOri->points[i].z;
        coeff.x = coeffSel->points[i].x;
        coeff.y = coeffSel->points[i].y;
        coeff.z = coeffSel->points[i].z;
        coeff.intensity = coeffSel->points[i].intensity;

        float arx = (-srx * cry * pointOri.x - (srx * sry * srz + crx * crz) * pointOri.y + (crx * srz - srx * sry * crz) * pointOri.z) * coeff.x
                  + (crx * cry * pointOri.x - (srx * crz - crx * sry * srz) * pointOri.y + (crx * sry * crz + srx * srz) * pointOri.z) * coeff.y;

        float ary = (-crx * sry * pointOri.x + crx * cry * srz * pointOri.y + crx * cry * crz * pointOri.z) * coeff.x
                  + (-srx * sry * pointOri.x + srx * sry * srz * pointOri.y + srx * cry * crz * pointOri.z) * coeff.y
                  + (-cry * pointOri.x - sry * srz * pointOri.y - sry * crz * pointOri.z) * coeff.z;

        float arz = ((crx * sry * crz + srx * srz) * pointOri.y + (srx * crz - crx * sry * srz) * pointOri.z) * coeff.x
                  + ((-crx * srz + srx * sry * crz) * pointOri.y + (-srx * sry * srz - crx * crz) * pointOri.z) * coeff.y
                  + (cry * crz * pointOri.y - cry * srz * pointOri.z) * coeff.z;
          
        matA.at<float>(i, 0) = arz;
        matA.at<float>(i, 1) = ary;
        matA.at<float>(i, 2) = arx;
        matA.at<float>(i, 3) = coeff.x;
        matA.at<float>(i, 4) = coeff.y;
        matA.at<float>(i, 5) = coeff.z;
        matB.at<float>(i, 0) = -coeff.intensity;
    }

    cv::transpose(matA, matAt);
    matAtA = matAt * matA;
    matAtB = matAt * matB;
    cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

    if (iterCount == 0) {
        cv::Mat matE(1, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matV(6, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matV2(6, 6, CV_32F, cv::Scalar::all(0));

        cv::eigen(matAtA, matE, matV);
        matV.copyTo(matV2);

        isDegenerate = false;
        float eignThre[6] = {1e-3f, 1e-3f, 1e-3f, 1e-3f, 1e-3f, 1e-3f};
        for (int i = 5; i >= 0; i--) {
            if (matE.at<float>(0, i) < eignThre[i]) {
                for (int j = 0; j < 6; j++) {
                    matV2.at<float>(i, j) = 0;
                }
                isDegenerate = true;
            } else {
                break;
            }
        }
        matP = matV.inv() * matV2;
    }

    if (isDegenerate)
    {
        cv::Mat matX2(6, 1, CV_32F, cv::Scalar::all(0));
        matX.copyTo(matX2);
        matX = matP * matX2;
    }

    currentTransform[0] += matX.at<float>(0, 0);
    currentTransform[1] += matX.at<float>(1, 0);
    currentTransform[2] += matX.at<float>(2, 0);
    currentTransform[3] += matX.at<float>(3, 0);
    currentTransform[4] += matX.at<float>(4, 0);
    currentTransform[5] += matX.at<float>(5, 0);

    float deltaR = sqrt(
                        pow(pcl::rad2deg(matX.at<float>(0, 0)), 2) +
                        pow(pcl::rad2deg(matX.at<float>(1, 0)), 2) +
                        pow(pcl::rad2deg(matX.at<float>(2, 0)), 2));
    float deltaT = sqrt(
                        pow(matX.at<float>(3, 0) * 100, 2) +
                        pow(matX.at<float>(4, 0) * 100, 2) +
                        pow(matX.at<float>(5, 0) * 100, 2));

    if (deltaR < 0.05 && deltaT < 0.05) {
        return true; 
    }
    return false;
}

void ScanAligner::cornerOptimization(const pcl::PointCloud<PointType>::Ptr &scan)
{
    (void)scan;
}