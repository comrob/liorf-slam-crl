#include "mapOptimization/mapOptimization.hpp"

void mapOptimization::updateInitialGuess()
{
    // save current transformation before any processing
    incrementalOdometryAffineFront = trans2Affine3f(transformTobeMapped);

    const auto clampTranslationPrediction = [this](Eigen::Affine3f &transIncre, const char *branch_name) {
        const double predNorm = static_cast<double>(transIncre.translation().norm());
        const double predSpeed = (curTimeDiff > 0.0)
                                     ? (predNorm / curTimeDiff)
                                     : std::numeric_limits<double>::infinity();

        if (diagnostics)
            diagnostics->recordTranslationPrediction(predNorm);

        if (maxTranslationPrediction > 0.0 && predNorm > maxTranslationPrediction)
        {
            std::ostringstream err_ss;
            err_ss << "[TRANSLATION_PREDICTION_EXCEEDED]"
                   << " branch=" << branch_name
                   << " delta_m=" << std::fixed << std::setprecision(3) << predNorm
                   << " limit_m=" << maxTranslationPrediction
                   << " cur_dt_s=" << curTimeDiff
                   << " last_dt_s=" << lastTimeDiff;
            const std::string err_msg = err_ss.str();
            RCLCPP_ERROR_STREAM(this->get_logger(), err_msg);
            transIncre.translation().setZero();
            if (diagnostics)
                diagnostics->publishWarning(err_msg);
        }

        if (minTranslationPredictionSpeed > 0.0 && predSpeed < minTranslationPredictionSpeed)
        {
            std::ostringstream warn_ss;
            warn_ss << "[TRANSLATION_PREDICTION_SPEED_TOO_LOW]"
                    << " branch=" << branch_name
                    << " speed_mps=" << std::fixed << std::setprecision(3) << predSpeed
                    << " min_speed_mps=" << minTranslationPredictionSpeed
                    << " delta_m=" << predNorm
                    << " cur_dt_s=" << curTimeDiff
                    << " last_dt_s=" << lastTimeDiff;
            const std::string warn_msg = warn_ss.str();
            RCLCPP_WARN_STREAM(this->get_logger(), warn_msg);
            transIncre.translation().setZero();
            if (diagnostics)
                diagnostics->publishWarning(warn_msg);
        }
    };

    static Eigen::Affine3f lastImuTransformation;
    // initialization
    if (cloudKeyPoses3D->points.empty())
    {
        transformTobeMapped[0] = cloudInfo.imurollinit;
        transformTobeMapped[1] = cloudInfo.imupitchinit;
        transformTobeMapped[2] = cloudInfo.imuyawinit;

        if (!useImuHeadingInitialization)
            transformTobeMapped[2] = 0;

        lastImuTransformation = pcl::getTransformation(0, 0, 0, cloudInfo.imurollinit, cloudInfo.imupitchinit, cloudInfo.imuyawinit); // save imu before return;
        return;
    }

    // use imu pre-integration estimation for pose guess
    static bool lastImuPreTransAvailable = false;
    static Eigen::Affine3f lastImuPreTransformation;
    if (cloudInfo.odomavailable == true)
    {
        Eigen::Affine3f transBack = pcl::getTransformation(cloudInfo.initialguessx,    cloudInfo.initialguessy,     cloudInfo.initialguessz, 
                                                           cloudInfo.initialguessroll, cloudInfo.initialguesspitch, cloudInfo.initialguessyaw);
        if (lastImuPreTransAvailable == false)
        {
            lastImuPreTransformation = transBack;
            lastImuPreTransAvailable = true;
        } else {
            Eigen::Affine3f transIncre = lastImuPreTransformation.inverse() * transBack;

            if (translationPredictionSource == TranslationPredictionSource::CONSTANT_VELOCITY)
            {
                transIncre.translation() = lastIncrementalDeltaPoseLocal.translation() * curTimeDiff / lastTimeDiff;
                clampTranslationPrediction(transIncre, "imu_preintegration");
                if (diagnostics)
                {
                    std::ostringstream diag_ss;
                    diag_ss << "Using constant velocity for translation prediction:" << std::endl
                            << transIncre.translation() << std::endl
                            << "curTimeDiff: " << curTimeDiff << std::endl
                            << "lastTimeDiff: " << lastTimeDiff;
                    diagnostics->logEventThrottle("constant_velocity_translation_prediction", 10.0, diag_ss.str());
                }
            }

            Eigen::Affine3f transTobe = trans2Affine3f(transformTobeMapped);
            Eigen::Affine3f transFinal = transTobe * transIncre;
            pcl::getTranslationAndEulerAngles(transFinal, transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5], 
                                                          transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);

            lastImuPreTransformation = transBack;

            lastImuTransformation = pcl::getTransformation(0, 0, 0, cloudInfo.imurollinit, cloudInfo.imupitchinit, cloudInfo.imuyawinit); // save imu before return;
            return;
        }
    }

    // use imu incremental estimation for pose guess (only rotation)
    if (cloudInfo.imuavailable == true && imuType)
    {
        Eigen::Affine3f transBack = pcl::getTransformation(0, 0, 0, cloudInfo.imurollinit, cloudInfo.imupitchinit, cloudInfo.imuyawinit);
        Eigen::Affine3f transIncre = lastImuTransformation.inverse() * transBack;

        if (translationPredictionSource == TranslationPredictionSource::CONSTANT_VELOCITY)
            {
                transIncre.translation() = lastIncrementalDeltaPoseLocal.translation() * curTimeDiff / lastTimeDiff;
                clampTranslationPrediction(transIncre, "imu_incremental");
                if (diagnostics)
                {
                    std::ostringstream diag_ss;
                    diag_ss << "Using constant velocity for translation prediction:" << std::endl
                            << transIncre.translation() << std::endl
                            << "curTimeDiff: " << curTimeDiff << std::endl
                            << "lastTimeDiff: " << lastTimeDiff;
                    diagnostics->logEventThrottle("constant_velocity_translation_prediction", 10.0, diag_ss.str());
                }
            }

        Eigen::Affine3f transTobe = trans2Affine3f(transformTobeMapped);
        Eigen::Affine3f transFinal = transTobe * transIncre;
        pcl::getTranslationAndEulerAngles(transFinal, transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5], 
                                                    transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);

        lastImuTransformation = pcl::getTransformation(0, 0, 0, cloudInfo.imurollinit, cloudInfo.imupitchinit, cloudInfo.imuyawinit); // save imu before return;
        return;
    }
}


void mapOptimization::downsampleCurrentScan()
{
    laserCloudSurfLastDS->clear();
    downSizeFilterSurf.setInputCloud(laserCloudSurfLast);
    downSizeFilterSurf.filter(*laserCloudSurfLastDS);

    if (useSorFilter && !laserCloudSurfLastDS->empty())
    {
        pcl::StatisticalOutlierRemoval<PointType> sor;
        sor.setInputCloud(laserCloudSurfLastDS);
        sor.setMeanK(sorMeanK);
        sor.setStddevMulThresh(sorStddevMulThresh);
        sor.filter(*laserCloudSurfLastDS);
    }

    laserCloudSurfLastDSNum = laserCloudSurfLastDS->size();
}

void mapOptimization::updatePointAssociateToMap()
{
    transPointAssociateToMap = trans2Affine3f(transformTobeMapped);
}

void mapOptimization::surfOptimization()
{
    updatePointAssociateToMap();

    #pragma omp parallel for num_threads(numberOfCores)
    for (int i = 0; i < laserCloudSurfLastDSNum; i++)
    {
        PointType pointOri, pointSel, coeff;
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;

        pointOri = laserCloudSurfLastDS->points[i];
        pointAssociateToMap(&pointOri, &pointSel); 
        kdtreeSurfFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

        Eigen::Matrix<float, 5, 3> matA0;
        Eigen::Matrix<float, 5, 1> matB0;
        Eigen::Vector3f matX0;

        matA0.setZero();
        matB0.fill(-1);
        matX0.setZero();

        if (pointSearchSqDis[4] < 1.0) {
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

            if (planeValid) {
                float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;

                float s = 1 - 0.9 * fabs(pd2) / sqrt(sqrt(pointOri.x * pointOri.x
                        + pointOri.y * pointOri.y + pointOri.z * pointOri.z));

                coeff.x = s * pa;
                coeff.y = s * pb;
                coeff.z = s * pc;
                coeff.intensity = s * pd2;

                if (s > 0.1) {
                    laserCloudOriSurfVec[i] = pointOri;
                    coeffSelSurfVec[i] = coeff;
                    laserCloudOriSurfFlag[i] = true;
                }
            }
        }
    }
}

void mapOptimization::combineOptimizationCoeffs()
{
    // combine surf coeffs
    for (int i = 0; i < laserCloudSurfLastDSNum; ++i){
        if (laserCloudOriSurfFlag[i] == true){
            laserCloudOri->push_back(laserCloudOriSurfVec[i]);
            coeffSel->push_back(coeffSelSurfVec[i]);
        }
    }
    // reset flag for next iteration
    std::fill(laserCloudOriSurfFlag.begin(), laserCloudOriSurfFlag.end(), false);
}

bool mapOptimization::LMOptimization(int iterCount)
{
    // This optimization is from the original loam_velodyne by Ji Zhang, need to cope with coordinate transformation
    // lidar <- camera      ---     camera <- lidar
    // x = z                ---     x = y
    // y = x                ---     y = z
    // z = y                ---     z = x
    // roll = yaw           ---     roll = pitch
    // pitch = roll         ---     pitch = yaw
    // yaw = pitch          ---     yaw = roll

    // lidar -> camera
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
        // lidar -> camera
        pointOri.x = laserCloudOri->points[i].x;
        pointOri.y = laserCloudOri->points[i].y;
        pointOri.z = laserCloudOri->points[i].z;
        // lidar -> camera
        coeff.x = coeffSel->points[i].x;
        coeff.y = coeffSel->points[i].y;
        coeff.z = coeffSel->points[i].z;
        coeff.intensity = coeffSel->points[i].intensity;
        // in camera
/*             float arx = (crx*sry*srz*pointOri.x + crx*crz*sry*pointOri.y - srx*sry*pointOri.z) * coeff.x
                  + (-srx*srz*pointOri.x - crz*srx*pointOri.y - crx*pointOri.z) * coeff.y
                  + (crx*cry*srz*pointOri.x + crx*cry*crz*pointOri.y - cry*srx*pointOri.z) * coeff.z;

        float ary = ((cry*srx*srz - crz*sry)*pointOri.x 
                  + (sry*srz + cry*crz*srx)*pointOri.y + crx*cry*pointOri.z) * coeff.x
                  + ((-cry*crz - srx*sry*srz)*pointOri.x 
                  + (cry*srz - crz*srx*sry)*pointOri.y - crx*sry*pointOri.z) * coeff.z;

        float arz = ((crz*srx*sry - cry*srz)*pointOri.x + (-cry*crz-srx*sry*srz)*pointOri.y)*coeff.x
                  + (crx*crz*pointOri.x - crx*srz*pointOri.y) * coeff.y
                  + ((sry*srz + cry*crz*srx)*pointOri.x + (crz*sry-cry*srx*srz)*pointOri.y)*coeff.z;
         */

        float arx = (-srx * cry * pointOri.x - (srx * sry * srz + crx * crz) * pointOri.y + (crx * srz - srx * sry * crz) * pointOri.z) * coeff.x
                  + (crx * cry * pointOri.x - (srx * crz - crx * sry * srz) * pointOri.y + (crx * sry * crz + srx * srz) * pointOri.z) * coeff.y;

        float ary = (-crx * sry * pointOri.x + crx * cry * srz * pointOri.y + crx * cry * crz * pointOri.z) * coeff.x
                  + (-srx * sry * pointOri.x + srx * sry * srz * pointOri.y + srx * cry * crz * pointOri.z) * coeff.y
                  + (-cry * pointOri.x - sry * srz * pointOri.y - sry * crz * pointOri.z) * coeff.z;

        float arz = ((crx * sry * crz + srx * srz) * pointOri.y + (srx * crz - crx * sry * srz) * pointOri.z) * coeff.x
                  + ((-crx * srz + srx * sry * crz) * pointOri.y + (-srx * sry * srz - crx * crz) * pointOri.z) * coeff.y
                  + (cry * crz * pointOri.y - cry * srz * pointOri.z) * coeff.z;
          
        // camera -> lidar
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
        float eignThre[6] = {100, 100, 100, 100, 100, 100};
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
        return true; // converged
    }
    return false; // keep optimizing
}

void mapOptimization::scan2MapOptimization()
{
    if (cloudKeyPoses3D->points.empty())
        return;

    if (laserCloudSurfLastDSNum > 30)
    {
        if (kdtreeLocalMapDirty)
        {
            TicToc t_setInputCloud;
            kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMapDS);
            kdtreeLocalMapDirty = false;
            if (diagnostics)
                diagnostics->recordSlice("scan2MapOptimization.setInputCloud", t_setInputCloud.toc());
        }

        double surf_ms_total = 0.0;
        double combine_ms_total = 0.0;
        double lm_ms_total = 0.0;
        int iter_used = 0;

        for (int iterCount = 0; iterCount < 30; iterCount++)
        {
            iter_used++;
            laserCloudOri->clear();
            coeffSel->clear();

            TicToc t_surfOptimization;
            surfOptimization();
            surf_ms_total += t_surfOptimization.toc();

            TicToc t_combineOptimizationCoeffs;
            combineOptimizationCoeffs();
            combine_ms_total += t_combineOptimizationCoeffs.toc();

            TicToc t_lmOptimization;
            if (LMOptimization(iterCount) == true)
            {
                lm_ms_total += t_lmOptimization.toc();
                break;              
            }
            lm_ms_total += t_lmOptimization.toc();
        }

        if (diagnostics)
        {
            diagnostics->recordSlice("scan2MapOptimization.surfOptimization.total", surf_ms_total);
            diagnostics->recordSlice("scan2MapOptimization.combineOptimizationCoeffs.total", combine_ms_total);
            diagnostics->recordSlice("scan2MapOptimization.LMOptimization.total", lm_ms_total);

            std::ostringstream oss;
            oss << "[SCAN2MAP_ITER] iter_used=" << iter_used
                << " surf_total_ms=" << std::fixed << std::setprecision(3) << surf_ms_total
                << " combine_total_ms=" << combine_ms_total
                << " lm_total_ms=" << lm_ms_total;
            diagnostics->logEventThrottle("scan2map_iter_summary", 1.0, oss.str());
        }

        TicToc t_transformUpdate;
        transformUpdate();
        if (diagnostics)
            diagnostics->recordSlice("scan2MapOptimization.transformUpdate", t_transformUpdate.toc());
    } else {
        RCLCPP_WARN(get_logger(), "Not enough features! Only %d planar features available.", laserCloudSurfLastDSNum);
    }
}

void mapOptimization::transformUpdate()
{
    if (cloudInfo.imuavailable == true && imuType)
    {
        if (std::abs(cloudInfo.imupitchinit) < 1.4)
        {
            double imuWeight = imuRPYWeight;
            tf2::Quaternion imuQuaternion;
            tf2::Quaternion transformQuaternion;
            double rollMid, pitchMid, yawMid;

            // slerp roll
            transformQuaternion.setRPY(transformTobeMapped[0], 0, 0);
            imuQuaternion.setRPY(cloudInfo.imurollinit, 0, 0);
            tf2::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);
            transformTobeMapped[0] = rollMid;

            // slerp pitch
            transformQuaternion.setRPY(0, transformTobeMapped[1], 0);
            imuQuaternion.setRPY(0, cloudInfo.imupitchinit, 0);
            tf2::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);
            transformTobeMapped[1] = pitchMid;
        }
    }

    transformTobeMapped[0] = constraintTransformation(transformTobeMapped[0], rotation_tollerance);
    transformTobeMapped[1] = constraintTransformation(transformTobeMapped[1], rotation_tollerance);
    transformTobeMapped[5] = constraintTransformation(transformTobeMapped[5], z_tollerance);

    incrementalOdometryAffineBack = trans2Affine3f(transformTobeMapped);
}

float mapOptimization::constraintTransformation(float value, float limit)
{
    if (value < -limit)
        value = -limit;
    if (value > limit)
        value = limit;

    return value;
}

bool mapOptimization::saveFrame()
{
    if (cloudKeyPoses3D->points.empty())
        return true;

    Eigen::Affine3f transStart = pclPointToAffine3f(cloudKeyPoses6D->back());
    Eigen::Affine3f transFinal = pcl::getTransformation(transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5],
                                                        transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
    Eigen::Affine3f transBetween = transStart.inverse() * transFinal;
    float x, y, z, roll, pitch, yaw;
    pcl::getTranslationAndEulerAngles(transBetween, x, y, z, roll, pitch, yaw);

    if (abs(roll)  < surroundingkeyframeAddingAngleThreshold &&
        abs(pitch) < surroundingkeyframeAddingAngleThreshold &&
        abs(yaw)   < surroundingkeyframeAddingAngleThreshold &&
        sqrt(x*x + y*y + z*z) < surroundingkeyframeAddingDistThreshold)
        return false;

    return true;
}

