// File: src/scanAlignment/ScanAligner.cpp
#include "scanAlignment/KdTreeLmBackend.hpp"
#include "tictoc.h"
#include "utility.h"
#include <omp.h>
namespace lio {

KdTreeLmBackend::KdTreeLmBackend(const KdTreeLmConfig& config)
    : config_(config)
{
    
    
    laserCloudOri = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>());
    coeffSel = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>());

    laserCloudOriSurfVec.resize(100000);
    coeffSelSurfVec.resize(100000);
    laserCloudOriSurfFlag.resize(100000, false);
    laserCloudSurfKnnPassFlag.resize(100000, 0);
    laserCloudSurfPlaneValidFlag.resize(100000, 0);
    laserCloudSurfDebugCode.resize(100000, SURF_DEBUG_NOT_OPTIMIZED);

    laserCloudSurfFromMap = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>());
    laserCloudSurfFromMapDS = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>());
    kdtreeSurfFromMap = pcl::KdTreeFLANN<PointType>::Ptr(new pcl::KdTreeFLANN<PointType>());
    downSizeFilterSurfMap.setLeafSize(config_.surroundingKeyframeMapLeafSize, config_.surroundingKeyframeMapLeafSize, config_.surroundingKeyframeMapLeafSize);

    matP = cv::Mat(6, 6, CV_32F, cv::Scalar::all(0));
    isDegenerate = false;
}

void KdTreeLmBackend::clearMap()
{
    laserCloudSurfFromMap->clear();
    laserCloudSurfFromMapDS->clear();
}

void KdTreeLmBackend::rebuildLocalMap(const std::vector<int>& keyframeIndices,
                         CloudRetriever getCloudFn,
                         const Eigen::Vector3d& currentSensorPos)
{
    laserCloudSurfFromMap->clear();
    for(int idx : keyframeIndices) {
        auto cloud = getCloudFn(idx);
        if (cloud != nullptr) {
            *laserCloudSurfFromMap += *cloud;
        }
    }
    laserCloudSurfFromMapDS->clear();
    downSizeFilterSurfMap.setInputCloud(laserCloudSurfFromMap);
    downSizeFilterSurfMap.filter(*laserCloudSurfFromMapDS);
    if (laserCloudSurfFromMapDS->size() > 10)
        kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMapDS);
}

void KdTreeLmBackend::updateRollingMap(const pcl::PointCloud<PointType>::Ptr& alignedScan,
                          const Eigen::Vector3d& sensorPos)
{
        *laserCloudSurfFromMapDS += *alignedScan;
    downSizeFilterSurfMap.setInputCloud(laserCloudSurfFromMapDS);
    pcl::PointCloud<PointType>::Ptr temp(new pcl::PointCloud<PointType>());
    downSizeFilterSurfMap.filter(*temp);
    
    // Spatial crop to prevent local map unbounded growth during extended traversals
    pcl::PointCloud<PointType>::Ptr cropped(new pcl::PointCloud<PointType>());
    cropped->reserve(temp->size());
    const float radius2 = 100.0f * 100.0f; // 100m bound
    for (const auto& p : temp->points) {
        float dx = p.x - sensorPos.x();
        float dy = p.y - sensorPos.y();
        float dz = p.z - sensorPos.z();
        if (dx*dx + dy*dy + dz*dz <= radius2) {
            cropped->points.push_back(p);
        }
    }
    cropped->width = cropped->points.size();
    cropped->height = 1;
    cropped->is_dense = false;
    
    laserCloudSurfFromMapDS = cropped;

    if (laserCloudSurfFromMapDS->size() > 10)
        kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMapDS);
}

size_t KdTreeLmBackend::getMapPointCount() const { return laserCloudSurfFromMapDS->size(); }

pcl::PointCloud<PointType>::Ptr KdTreeLmBackend::getLocalMapCloud() const { return laserCloudSurfFromMapDS; }


void KdTreeLmBackend::pointAssociateToMap(PointType const *const pi, PointType *const po, float* transformTobeMapped)
{
    po->x = transPointAssociateToMap(0,0) * pi->x + transPointAssociateToMap(0,1) * pi->y + transPointAssociateToMap(0,2) * pi->z + transPointAssociateToMap(0,3);
    po->y = transPointAssociateToMap(1,0) * pi->x + transPointAssociateToMap(1,1) * pi->y + transPointAssociateToMap(1,2) * pi->z + transPointAssociateToMap(1,3);
    po->z = transPointAssociateToMap(2,0) * pi->x + transPointAssociateToMap(2,1) * pi->y + transPointAssociateToMap(2,2) * pi->z + transPointAssociateToMap(2,3);
    po->intensity = pi->intensity;
}

void KdTreeLmBackend::updatePointAssociateToMap(float* transformTobeMapped)
{
    transPointAssociateToMap = pcl::getTransformation(transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5], transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
}

AlignmentMetrics KdTreeLmBackend::align(const pcl::PointCloud<PointType>::Ptr &scan, float* transformIn, bool isDegeneracyRun)
{
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

    // Use the transformIn pointer directly for optimization.
    // Or make a local working copy.

    
    AlignmentMetrics metrics;
    metrics.iterations = 0;
    if (laserCloudSurfFromMapDS->points.size() < 10) return metrics;

    surfStageInputCount = static_cast<uint32_t>(scanSize);
    surfStageKnnPassCount = 0;
    surfStagePlaneValidCount = 0;
    surfStageMatchedCount = 0;

    std::fill(laserCloudSurfDebugCode.begin(), laserCloudSurfDebugCode.end(), SURF_DEBUG_NOT_OPTIMIZED);

    for (int iterCount = 0; iterCount < 30; iterCount++)
    {
        metrics.iterations++;
        laserCloudOri->clear();
        coeffSel->clear();

        TicToc t_surfOptimization;
        surfOptimization(scan, transformIn);
        metrics.optimization_ms += t_surfOptimization.toc();

        TicToc t_combineOptimizationCoeffs;
        combineOptimizationCoeffs(scanSize);
        metrics.combine_ms += t_combineOptimizationCoeffs.toc();

        TicToc t_lmOptimization;
        if (LMOptimization(iterCount, transformIn) == true)
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

    
    
    return metrics;
}

void KdTreeLmBackend::surfOptimization(const pcl::PointCloud<PointType>::Ptr &scan, float* transformTobeMapped)
{
    // throw an error if kdtreeMap is not set
    if (kdtreeSurfFromMap->getInputCloud() == nullptr) {
        throw std::runtime_error("KdTreeLmBackend::surfOptimization - kdtreeMap input cloud is not set. Please call setMap() before surfOptimization().");
    }

    updatePointAssociateToMap(transformTobeMapped);

    const int scanSize = scan->points.size();
    std::fill(laserCloudSurfKnnPassFlag.begin(), laserCloudSurfKnnPassFlag.begin() + scanSize, 0);
    std::fill(laserCloudSurfPlaneValidFlag.begin(), laserCloudSurfPlaneValidFlag.begin() + scanSize, 0);
    std::fill(laserCloudSurfDebugCode.begin(), laserCloudSurfDebugCode.begin() + scanSize, SURF_DEBUG_NOT_OPTIMIZED);

    int knnPassCount = 0;
    int planeValidCount = 0;
    int matchedCount = 0;

    #pragma omp parallel for num_threads(omp_get_num_procs()) reduction(+:knnPassCount,planeValidCount,matchedCount)
    for (int i = 0; i < scanSize; i++)
    {
        PointType pointOri, pointSel, coeff;
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;

        pointOri = scan->points[i];
        pointAssociateToMap(&pointOri, &pointSel, transformTobeMapped); 

        laserCloudSurfDebugCode[i] = SURF_DEBUG_REJECTED_NEIGHBOR_COUNT;
        const int foundNeighbors = kdtreeSurfFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);
        if (foundNeighbors < 5)
            continue;

        Eigen::Matrix<float, 5, 3> matA0;
        Eigen::Matrix<float, 5, 1> matB0;
        Eigen::Vector3f matX0;

        matA0.setZero();
        matB0.fill(-1);
        matX0.setZero();

        const float knnGateDistanceSq = config_.surfKnnMinDistance * config_.surfKnnMinDistance;

        if (pointSearchSqDis[4] >= knnGateDistanceSq)
        {
            laserCloudSurfDebugCode[i] = SURF_DEBUG_REJECTED_KNN_DISTANCE;
            continue;
        }

        laserCloudSurfKnnPassFlag[i] = 1;
        knnPassCount++;

        for (int j = 0; j < 5; j++) {
            matA0(j, 0) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].x;
            matA0(j, 1) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].y;
            matA0(j, 2) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].z;
        }

        matX0 = matA0.colPivHouseholderQr().solve(matB0);

        float pa = matX0(0, 0);
        float pb = matX0(1, 0);
        float pc = matX0(2, 0);
        float pd = 1;

        float ps = sqrt(pa * pa + pb * pb + pc * pc);
        const float planeNormEpsilon = 1e-6f;
        if (ps <= planeNormEpsilon || std::isnan(ps) || std::isinf(ps)) {
            laserCloudSurfDebugCode[i] = SURF_DEBUG_REJECTED_PLANE_INVALID;
            continue;
        }
        pa /= ps; pb /= ps; pc /= ps; pd /= ps;

        bool planeValid = true;
        for (int j = 0; j < 5; j++) {
            if (fabs(pa * laserCloudSurfFromMapDS->points[pointSearchInd[j]].x +
                     pb * laserCloudSurfFromMapDS->points[pointSearchInd[j]].y +
                     pc * laserCloudSurfFromMapDS->points[pointSearchInd[j]].z + pd) > 0.2) {
                planeValid = false;
                break;
            }
        }

        if (!planeValid)
        {
            laserCloudSurfDebugCode[i] = SURF_DEBUG_REJECTED_PLANE_INVALID;
            continue;
        }

        laserCloudSurfPlaneValidFlag[i] = 1;
        planeValidCount++;

        float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;

        float s = 1 - 0.9 * fabs(pd2) / sqrt(pointOri.x * pointOri.x
                + pointOri.y * pointOri.y + pointOri.z * pointOri.z);

        coeff.x = s * pa;
        coeff.y = s * pb;
        coeff.z = s * pc;
        coeff.intensity = s * pd2;

        if (s > 0.1) {
            laserCloudSurfDebugCode[i] = SURF_DEBUG_ACCEPTED;
            laserCloudOriSurfVec[i] = pointOri;
            coeffSelSurfVec[i] = coeff;
            laserCloudOriSurfFlag[i] = true;
            matchedCount++;
        } else {
            laserCloudSurfDebugCode[i] = SURF_DEBUG_REJECTED_LOW_WEIGHT;
        }
    }

    surfStageKnnPassCount = static_cast<uint32_t>(knnPassCount);
    surfStagePlaneValidCount = static_cast<uint32_t>(planeValidCount);
    surfStageMatchedCount = static_cast<uint32_t>(matchedCount);
}

void KdTreeLmBackend::combineOptimizationCoeffs(int scanSize)
{
    for (int i = 0; i < scanSize; ++i){
        if (laserCloudOriSurfFlag[i] == true){
            laserCloudOri->push_back(laserCloudOriSurfVec[i]);
            coeffSel->push_back(coeffSelSurfVec[i]);
        }
    }
    std::fill(laserCloudOriSurfFlag.begin(), laserCloudOriSurfFlag.begin() + scanSize, false);
}

bool KdTreeLmBackend::LMOptimization(int iterCount, float* transformTobeMapped)
{
    float srx = sin(transformTobeMapped[2]);
    float crx = cos(transformTobeMapped[2]);
    float sry = sin(transformTobeMapped[1]);
    float cry = cos(transformTobeMapped[1]);
    float srz = sin(transformTobeMapped[0]);
    float crz = cos(transformTobeMapped[0]);

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
            if (matE.at<float>(i, 0) < eignThre[i]) {
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

    transformTobeMapped[0] += matX.at<float>(0, 0);
    transformTobeMapped[1] += matX.at<float>(1, 0);
    transformTobeMapped[2] += matX.at<float>(2, 0);
    transformTobeMapped[3] += matX.at<float>(3, 0);
    transformTobeMapped[4] += matX.at<float>(4, 0);
    transformTobeMapped[5] += matX.at<float>(5, 0);

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
} // namespace lio
