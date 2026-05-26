#include "mapOptimization/mapOptimization.hpp"

void mapOptimization::markMapRebuildTriggered(const std::string &reason)
{
    if (!require_map_rebuild)
        require_map_rebuild = true;

    localMapDirty = true;
    kdtreeLocalMapDirty = true;

    if (diagnostics)
    {
        std::ostringstream oss;
        oss << "[MAP_REBUILD_TRIGGER] reason=" << reason
            << " t=" << std::fixed << std::setprecision(3) << timeLaserInfoCur
            << " keyposes=" << cloudKeyPoses3D->size()
            << " voxel_count=" << (voxelMap ? voxelMap->GetVoxelCount() : 0);
        diagnostics->logEvent(oss.str());
    }
}

void mapOptimization::logLocalMapStats(const std::string &stage)
{
    if (!diagnostics)
        return;

    std::ostringstream oss;
    oss << "[LOCAL_MAP_STATS] stage=" << stage
        << " t=" << std::fixed << std::setprecision(3) << timeLaserInfoCur
        << " rebuild_pending=" << (require_map_rebuild ? 1 : 0)
        << " voxels=" << (voxelMap ? voxelMap->GetVoxelCount() : 0)
        << " local_map_pts=" << (voxelMap ? voxelMap->GetPointCount() : 0)
        << " cached_clouds=" << 0
        << " current_scan_pts=" << laserCloudSurfLastDSNum
        << " keyposes=" << cloudKeyPoses3D->size()
        << " radius_search=" << surroundingKeyframeSearchRadius
        << " radius_truncation=" << localMapTruncationRadius
        << " leaf=" << surroundingKeyframeMapLeafSize
        << " cache_max_age_s=" << transformed_cloud_cache_max_age_sec;

    diagnostics->logEventThrottle("local_map_stats", 1.0, oss.str());
}

void mapOptimization::manageLocalMap()
{
    if (cloudKeyPoses3D->points.empty()) {
        voxelMap->Clear();
        require_map_rebuild = false;
        localMapDirty = false;
        return;
    }

    if (require_map_rebuild || localMapDirty)
    {
        voxelMap->Clear();
        int numPoses = cloudKeyPoses3D->size();
        
        Eigen::Vector3d current_sensor_pos(transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5]);

        // When rebuilding, we need to gather nearby or recent keyframes 
        // to re-populate the voxel map properly.
        for (int i = numPoses - 1; i >= 0; --i)
        {
            if (i >= 0 && i < static_cast<int>(keyframeScanAdmissible.size()) && !keyframeScanAdmissible[i])
                continue;

            // Simple distance check from the latest pose
            if (common_lib_->pointDistance(cloudKeyPoses3D->points[i], cloudKeyPoses3D->back()) > surroundingKeyframeSearchRadius)
                continue;

            pcl::PointCloud<PointType>::Ptr transformedCurrentScan = transformPointCloud(surfCloudKeyFrames[i], &cloudKeyPoses6D->points[i]);

            // Add straight to voxel map, keeping the bounding box centered at the latest position
            voxelMap->UpdateVoxelMap(transformedCurrentScan, current_sensor_pos, localMapTruncationRadius, true);
        }

        require_map_rebuild = false;
        localMapDirty = false;
        
    }
}

void mapOptimization::updateRollingMap()
{
    if (require_map_rebuild || laserCloudSurfLastDS->empty() || !cloudInfo.scan_admission_ok)
        return;

    Eigen::Vector3d sensor_pos(transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5]);
    PointTypePose poseForTransform = trans2PointTypePose(transformTobeMapped);
    pcl::PointCloud<PointType>::Ptr transformedCurrentScan = transformPointCloud(laserCloudSurfLastDS, &poseForTransform);

    // VoxelMap handles distance pruning and insertion in O(1)
    voxelMap->UpdateVoxelMap(transformedCurrentScan, sensor_pos, localMapTruncationRadius, true);

    logLocalMapStats("updateRollingMap");
}

bool mapOptimization::saveMapService(const std::shared_ptr<liorf::srv::SaveMap::Request> req,
                            std::shared_ptr<liorf::srv::SaveMap::Response> res)
{
  sensor_msgs::msg::NavSatFix originSnapshot;
  bool hasOrigin = false;
  {
      std::lock_guard<std::mutex> lock(origin_mutex);
      originSnapshot = stored_origin_gps_msg;
      hasOrigin = !first_gps;
  }

  std::vector<sensor_msgs::msg::NavSatFix> gpsHistorySnapshot;
  std::vector<nav_msgs::msg::Odometry> densePoseHistorySnapshot;
  if (save_dense_gps_trajectory)
  {
      std::lock_guard<std::mutex> history_lock(gpsHistoryMutex);
      gpsHistorySnapshot = gpsHistory;
  }
  if (save_dense_odom_trajectory)
  {
      std::lock_guard<std::mutex> history_lock(densePoseHistoryMutex);
      densePoseHistorySnapshot = densePoseHistory;
  }

  return map_exporter_.executeSave(
      *req,
      *res,
      savePCDDirectory,
      mappingSurfLeafSize,
      save_dense_gps_trajectory,
      save_dense_odom_trajectory,
      originSnapshot,
      hasOrigin,
      cloudKeyPoses3D,
      cloudKeyPoses6D,
      surfCloudKeyFrames,
      T_EL_initialized,
      T_EL_estimate,
      gpsHistorySnapshot,
      densePoseHistorySnapshot,
      get_logger(),
      this->now());
}

void mapOptimization::visualizeGlobalMapThread()
{
    rclcpp::Rate rate(0.2);
    while (rclcpp::ok()){
        rate.sleep();
        publishGlobalMap();
    }

    if (savePCD == false)
        return;

    std::shared_ptr<liorf::srv::SaveMap::Request> req = std::make_unique<liorf::srv::SaveMap::Request>();
    std::shared_ptr<liorf::srv::SaveMap::Response> res = std::make_unique<liorf::srv::SaveMap::Response>();

    if(!saveMapService(req, res)){
        cout << "Fail to save map" << endl;
    }
}

void mapOptimization::publishGlobalMap()
{
    if (pubLaserCloudSurround->get_subscription_count() == 0)
        return;

    if (cloudKeyPoses3D->points.empty() == true)
        return;

    pcl::KdTreeFLANN<PointType>::Ptr kdtreeGlobalMap(new pcl::KdTreeFLANN<PointType>());;
    pcl::PointCloud<PointType>::Ptr globalMapKeyPoses(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr globalMapKeyPosesDS(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr globalMapKeyFrames(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr globalMapKeyFramesDS(new pcl::PointCloud<PointType>());

    // kd-tree to find near key frames to visualize
    std::vector<int> pointSearchIndGlobalMap;
    std::vector<float> pointSearchSqDisGlobalMap;
    // search near key frames to visualize
    mtx.lock();
    kdtreeGlobalMap->setInputCloud(cloudKeyPoses3D);
    kdtreeGlobalMap->radiusSearch(cloudKeyPoses3D->back(), globalMapVisualizationSearchRadius, pointSearchIndGlobalMap, pointSearchSqDisGlobalMap, 0);
    mtx.unlock();

    for (int i = 0; i < (int)pointSearchIndGlobalMap.size(); ++i)
        globalMapKeyPoses->push_back(cloudKeyPoses3D->points[pointSearchIndGlobalMap[i]]);
    // downsample near selected key frames
    pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyPoses; // for global map visualization
    downSizeFilterGlobalMapKeyPoses.setLeafSize(globalMapVisualizationPoseDensity, globalMapVisualizationPoseDensity, globalMapVisualizationPoseDensity); // for global map visualization
    downSizeFilterGlobalMapKeyPoses.setInputCloud(globalMapKeyPoses);
    downSizeFilterGlobalMapKeyPoses.filter(*globalMapKeyPosesDS);
    for(auto& pt : globalMapKeyPosesDS->points)
    {
        kdtreeGlobalMap->nearestKSearch(pt, 1, pointSearchIndGlobalMap, pointSearchSqDisGlobalMap);
        pt.intensity = cloudKeyPoses3D->points[pointSearchIndGlobalMap[0]].intensity;
    }

    // extract visualized and downsampled key frames
    for (int i = 0; i < (int)globalMapKeyPosesDS->size(); ++i){
        if (common_lib_->pointDistance(globalMapKeyPosesDS->points[i], cloudKeyPoses3D->back()) > globalMapVisualizationSearchRadius)
            continue;
        int thisKeyInd = (int)globalMapKeyPosesDS->points[i].intensity;
        if (thisKeyInd >= 0 && thisKeyInd < static_cast<int>(keyframeScanAdmissible.size()) && !keyframeScanAdmissible[thisKeyInd])
            continue;
        *globalMapKeyFrames += *transformPointCloud(surfCloudKeyFrames[thisKeyInd],    &cloudKeyPoses6D->points[thisKeyInd]);
    }
    // downsample visualized points
    pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyFrames; // for global map visualization
    downSizeFilterGlobalMapKeyFrames.setLeafSize(globalMapVisualizationLeafSize, globalMapVisualizationLeafSize, globalMapVisualizationLeafSize); // for global map visualization
    downSizeFilterGlobalMapKeyFrames.setInputCloud(globalMapKeyFrames);
    downSizeFilterGlobalMapKeyFrames.filter(*globalMapKeyFramesDS);
    publishCloud(pubLaserCloudSurround, globalMapKeyFramesDS, timeLaserInfoStamp, mapFrameLocal);
}




