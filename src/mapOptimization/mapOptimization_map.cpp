#include "mapOptimization/mapOptimization.hpp"

VOXEL_LOC mapOptimization::voxelizePoint(const PointType &point, const float leafSize) const
{
    const float safeLeaf = std::max(leafSize, 1e-3f);
    VOXEL_LOC voxel;
    voxel.x = static_cast<int64_t>(std::floor(point.x / safeLeaf));
    voxel.y = static_cast<int64_t>(std::floor(point.y / safeLeaf));
    voxel.z = static_cast<int64_t>(std::floor(point.z / safeLeaf));
    return voxel;
}

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
            << " voxel_count=" << voxelHashMap.size();
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
        << " voxels=" << voxelHashMap.size()
        << " local_map_pts=" << laserCloudSurfFromMapDS->size()
        << " cached_clouds=" << laserCloudMapContainer.size()
        << " current_scan_pts=" << laserCloudSurfLastDSNum
        << " keyposes=" << cloudKeyPoses3D->size()
        << " radius=" << surroundingKeyframeSearchRadius
        << " leaf=" << surroundingKeyframeMapLeafSize
        << " cache_max_age_s=" << transformed_cloud_cache_max_age_sec;

    diagnostics->logEventThrottle("local_map_stats", 1.0, oss.str());
}

size_t mapOptimization::pruneTransformedCloudCache()
{
    if (laserCloudMapContainer.empty())
        return 0;

    const double oldestAllowed = timeLaserInfoCur - transformed_cloud_cache_max_age_sec;
    size_t removed = 0;

    for (auto it = laserCloudMapContainer.begin(); it != laserCloudMapContainer.end();)
    {
        const int key = it->first;
        bool erase = false;

        if (key < 0 || key >= static_cast<int>(cloudKeyPoses6D->size()))
        {
            erase = true;
        }
        else if (cloudKeyPoses6D->points[key].time < oldestAllowed)
        {
            erase = true;
        }

        if (erase)
        {
            it = laserCloudMapContainer.erase(it);
            ++removed;
        }
        else
        {
            ++it;
        }
    }

    return removed;
}

void mapOptimization::manageLocalMap()
{
    if (cloudKeyPoses3D->points.empty())
    {
        laserCloudSurfFromMapDS->clear();
        laserCloudSurfFromMapDSNum = 0;
        return;
    }

    if (!require_map_rebuild && !localMapDirty)
    {
        logLocalMapStats("manageLocalMap_reuse");
        return;
    }

    if (require_map_rebuild)
    {
        const size_t prev_voxel_count = voxelHashMap.size();

        TicToc t_rebuild_extract;
        voxelHashMap.clear();
        extractSurroundingKeyFrames();
        if (diagnostics)
            diagnostics->recordSlice("manageLocalMap.rebuild.extractSurroundingKeyFrames", t_rebuild_extract.toc());

        TicToc t_rebuild_insert;
        voxelHashMap.reserve(laserCloudSurfFromMapDS->size());

        for (const auto &pt : laserCloudSurfFromMapDS->points)
            voxelHashMap[voxelizePoint(pt, surroundingKeyframeMapLeafSize)] = pt;
        if (diagnostics)
            diagnostics->recordSlice("manageLocalMap.rebuild.hashInsert", t_rebuild_insert.toc());

        require_map_rebuild = false;

        if (diagnostics)
        {
            std::ostringstream oss;
            oss << "[MAP_REBUILD_DONE] mode=full"
                << " t=" << std::fixed << std::setprecision(3) << timeLaserInfoCur
                << " prev_voxels=" << prev_voxel_count
                << " new_voxels=" << voxelHashMap.size()
                << " local_map_pts=" << laserCloudSurfFromMapDS->size();
            diagnostics->logEvent(oss.str());
        }
    }
    else
    {
        const float cx = transformTobeMapped[3];
        const float cy = transformTobeMapped[4];
        const float cz = transformTobeMapped[5];
        const float radius2 = surroundingKeyframeSearchRadius * surroundingKeyframeSearchRadius;
        const size_t before_prune = voxelHashMap.size();

        TicToc t_incremental_prune;

        for (auto it = voxelHashMap.begin(); it != voxelHashMap.end();)
        {
            const auto &pt = it->second;
            const float dx = pt.x - cx;
            const float dy = pt.y - cy;
            const float dz = pt.z - cz;
            if ((dx * dx + dy * dy + dz * dz) > radius2)
                it = voxelHashMap.erase(it);
            else
                ++it;
        }
        if (diagnostics)
            diagnostics->recordSlice("manageLocalMap.incremental.prune", t_incremental_prune.toc());

        TicToc t_incremental_rebuildCloud;
        laserCloudSurfFromMapDS->clear();
        laserCloudSurfFromMapDS->reserve(voxelHashMap.size());
        for (const auto &entry : voxelHashMap)
            laserCloudSurfFromMapDS->push_back(entry.second);
        if (diagnostics)
            diagnostics->recordSlice("manageLocalMap.incremental.materializeCloud", t_incremental_rebuildCloud.toc());

        if (diagnostics)
        {
            std::ostringstream oss;
            oss << "[MAP_REBUILD_DONE] mode=incremental"
                << " t=" << std::fixed << std::setprecision(3) << timeLaserInfoCur
                << " voxels_before=" << before_prune
                << " voxels_after=" << voxelHashMap.size()
                << " local_map_pts=" << laserCloudSurfFromMapDS->size();
            diagnostics->logEventThrottle("map_incremental_update", 1.0, oss.str());
        }
    }

    localMapDirty = false;
    kdtreeLocalMapDirty = true;
    laserCloudSurfFromMapDSNum = laserCloudSurfFromMapDS->size();
    logLocalMapStats("manageLocalMap");
}

void mapOptimization::updateRollingMap()
{
    if (require_map_rebuild || laserCloudSurfLastDS->empty())
        return;

    PointTypePose poseForTransform = trans2PointTypePose(transformTobeMapped);
    TicToc t_transformCurrentScan;
    pcl::PointCloud<PointType>::Ptr transformedCurrentScan = transformPointCloud(laserCloudSurfLastDS, &poseForTransform);
    if (diagnostics)
        diagnostics->recordSlice("updateRollingMap.transformPointCloud", t_transformCurrentScan.toc());

    TicToc t_hashInsert;
    voxelHashMap.reserve(voxelHashMap.size() + transformedCurrentScan->size());

    for (const auto &pt : transformedCurrentScan->points)
    {
        VOXEL_LOC voxel = voxelizePoint(pt, surroundingKeyframeMapLeafSize);
        if (voxelHashMap.find(voxel) == voxelHashMap.end())
            voxelHashMap.emplace(voxel, pt);
    }
    if (diagnostics)
        diagnostics->recordSlice("updateRollingMap.hashInsert", t_hashInsert.toc());

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
        *globalMapKeyFrames += *transformPointCloud(surfCloudKeyFrames[thisKeyInd],    &cloudKeyPoses6D->points[thisKeyInd]);
    }
    // downsample visualized points
    pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyFrames; // for global map visualization
    downSizeFilterGlobalMapKeyFrames.setLeafSize(globalMapVisualizationLeafSize, globalMapVisualizationLeafSize, globalMapVisualizationLeafSize); // for global map visualization
    downSizeFilterGlobalMapKeyFrames.setInputCloud(globalMapKeyFrames);
    downSizeFilterGlobalMapKeyFrames.filter(*globalMapKeyFramesDS);
    publishCloud(pubLaserCloudSurround, globalMapKeyFramesDS, timeLaserInfoStamp, mapFrameLocal);
}













void mapOptimization::extractForLoopClosure()
{
    pcl::PointCloud<PointType>::Ptr cloudToExtract(new pcl::PointCloud<PointType>());
    int numPoses = cloudKeyPoses3D->size();
    for (int i = numPoses-1; i >= 0; --i)
    {
        if ((int)cloudToExtract->size() <= surroundingKeyframeSize)
            cloudToExtract->push_back(cloudKeyPoses3D->points[i]);
        else
            break;
    }

    extractCloud(cloudToExtract);
}

void mapOptimization::extractNearby()
{
    pcl::PointCloud<PointType>::Ptr surroundingKeyPoses(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr surroundingKeyPosesDS(new pcl::PointCloud<PointType>());
    std::vector<int> pointSearchInd;
    std::vector<float> pointSearchSqDis;

    // extract all the nearby key poses and downsample them
    TicToc t_extractNearby_radiusSearch;
    kdtreeSurroundingKeyPoses->setInputCloud(cloudKeyPoses3D); // create kd-tree
    kdtreeSurroundingKeyPoses->radiusSearch(cloudKeyPoses3D->back(), (double)surroundingKeyframeSearchRadius, pointSearchInd, pointSearchSqDis);
    if (diagnostics)
        diagnostics->recordSlice("extractNearby.radiusSearch", t_extractNearby_radiusSearch.toc());

    TicToc t_extractNearby_collectPoses;
    for (int i = 0; i < (int)pointSearchInd.size(); ++i)
    {
        int id = pointSearchInd[i];
        surroundingKeyPoses->push_back(cloudKeyPoses3D->points[id]);
    }
    if (diagnostics)
        diagnostics->recordSlice("extractNearby.collectPoses", t_extractNearby_collectPoses.toc());

    TicToc t_extractNearby_downsamplePoses;
    downSizeFilterSurroundingKeyPoses.setInputCloud(surroundingKeyPoses);
    downSizeFilterSurroundingKeyPoses.filter(*surroundingKeyPosesDS);
    if (diagnostics)
        diagnostics->recordSlice("extractNearby.downsamplePoses", t_extractNearby_downsamplePoses.toc());

    TicToc t_extractNearby_remapIndices;
    for(auto& pt : surroundingKeyPosesDS->points)
    {
        kdtreeSurroundingKeyPoses->nearestKSearch(pt, 1, pointSearchInd, pointSearchSqDis);
        pt.intensity = cloudKeyPoses3D->points[pointSearchInd[0]].intensity;
    }
    if (diagnostics)
        diagnostics->recordSlice("extractNearby.remapIndices", t_extractNearby_remapIndices.toc());

    // also extract some latest key frames in case the robot rotates in one position
    TicToc t_extractNearby_addRecent;
    int numPoses = cloudKeyPoses3D->size();
    for (int i = numPoses-1; i >= 0; --i)
    {
        if (timeLaserInfoCur - cloudKeyPoses6D->points[i].time < 10.0)
            surroundingKeyPosesDS->push_back(cloudKeyPoses3D->points[i]);
        else
            break;
    }
    if (diagnostics)
        diagnostics->recordSlice("extractNearby.addRecent", t_extractNearby_addRecent.toc());

    TicToc t_extractNearby_extractCloud;
    extractCloud(surroundingKeyPosesDS);
    if (diagnostics)
        diagnostics->recordSlice("extractNearby.extractCloud", t_extractNearby_extractCloud.toc());
}

void mapOptimization::extractCloud(pcl::PointCloud<PointType>::Ptr cloudToExtract)
{
    // fuse the map
    laserCloudSurfFromMap->clear(); 
    TicToc t_extractCloud_fuse;
    for (int i = 0; i < (int)cloudToExtract->size(); ++i)
    {
        if (common_lib_->pointDistance(cloudToExtract->points[i], cloudKeyPoses3D->back()) > surroundingKeyframeSearchRadius)
            continue;

        int thisKeyInd = (int)cloudToExtract->points[i].intensity;
        if (laserCloudMapContainer.find(thisKeyInd) != laserCloudMapContainer.end()) 
        {
            // transformed cloud available
            *laserCloudSurfFromMap   += laserCloudMapContainer[thisKeyInd].second;
        } else {
            // transformed cloud not available
            pcl::PointCloud<PointType> laserCloudCornerTemp;
            pcl::PointCloud<PointType> laserCloudSurfTemp = *transformPointCloud(surfCloudKeyFrames[thisKeyInd],    &cloudKeyPoses6D->points[thisKeyInd]);
            *laserCloudSurfFromMap   += laserCloudSurfTemp;
            laserCloudMapContainer[thisKeyInd] = make_pair(laserCloudCornerTemp, laserCloudSurfTemp);
        }
        
    }
    if (diagnostics)
        diagnostics->recordSlice("extractCloud.fuseAndTransform", t_extractCloud_fuse.toc());

    // Downsample the surrounding surf key frames (or map)
    TicToc t_extractCloud_downsample;
    downSizeFilterLocalMapSurf.setInputCloud(laserCloudSurfFromMap);
    downSizeFilterLocalMapSurf.filter(*laserCloudSurfFromMapDS);
    laserCloudSurfFromMapDSNum = laserCloudSurfFromMapDS->size();
    if (diagnostics)
        diagnostics->recordSlice("extractCloud.downsampleLocalMap", t_extractCloud_downsample.toc());

    // clear map cache if too large
    TicToc t_extractCloud_cacheMaintenance;
    const size_t removedCacheEntries = pruneTransformedCloudCache();
    if (diagnostics)
    {
        diagnostics->recordSlice("extractCloud.cacheMaintenance", t_extractCloud_cacheMaintenance.toc());

        if (removedCacheEntries > 0)
        {
            std::ostringstream oss;
            oss << "[CACHE_PRUNE] removed=" << removedCacheEntries
                << " remaining=" << laserCloudMapContainer.size()
                << " max_age_s=" << transformed_cloud_cache_max_age_sec
                << " t=" << std::fixed << std::setprecision(3) << timeLaserInfoCur;
            diagnostics->logEventThrottle("cache_prune", 1.0, oss.str());
        }
    }
}

void mapOptimization::extractSurroundingKeyFrames()
{
    if (cloudKeyPoses3D->points.empty() == true)
        return;

    TicToc t_extractSurroundingKeyFrames_extractNearby;
    extractNearby();
    if (diagnostics)
        diagnostics->recordSlice("extractSurroundingKeyFrames.extractNearby", t_extractSurroundingKeyFrames_extractNearby.toc());
}

