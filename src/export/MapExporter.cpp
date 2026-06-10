#include "export/MapExporter.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <builtin_interfaces/msg/time.hpp>

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>

#include <rclcpp/rclcpp.hpp>

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>

namespace
{
double stampToSec(const builtin_interfaces::msg::Time &stamp)
{
    return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

pcl::PointCloud<PointType>::Ptr transformPointCloud(
    const pcl::PointCloud<PointType>::Ptr &cloudIn,
    const PointTypePose &transformIn)
{
    pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
    const int cloudSize = static_cast<int>(cloudIn->size());
    cloudOut->resize(cloudSize);

    const Eigen::Affine3f transCur = pcl::getTransformation(
        transformIn.x,
        transformIn.y,
        transformIn.z,
        transformIn.roll,
        transformIn.pitch,
        transformIn.yaw);

    for (int i = 0; i < cloudSize; ++i)
    {
        const auto &pointFrom = cloudIn->points[i];
        cloudOut->points[i].x = transCur(0, 0) * pointFrom.x + transCur(0, 1) * pointFrom.y + transCur(0, 2) * pointFrom.z + transCur(0, 3);
        cloudOut->points[i].y = transCur(1, 0) * pointFrom.x + transCur(1, 1) * pointFrom.y + transCur(1, 2) * pointFrom.z + transCur(1, 3);
        cloudOut->points[i].z = transCur(2, 0) * pointFrom.x + transCur(2, 1) * pointFrom.y + transCur(2, 2) * pointFrom.z + transCur(2, 3);
        cloudOut->points[i].intensity = pointFrom.intensity;
    }

    return cloudOut;
}

std::filesystem::path resolveSaveDirectory(
    const liorf::srv::SaveMap::Request &req,
    const std::string &savePCDDirectory,
    const std::string &homeDir)
{
    namespace fs = std::filesystem;
    fs::path saveDir;
    if (req.destination.empty())
    {
        saveDir = fs::path(homeDir) / fs::path(savePCDDirectory).relative_path();
    }
    else if (fs::path(req.destination).is_absolute())
    {
        saveDir = req.destination;
    }
    else if (req.destination.rfind("~/", 0) == 0)
    {
        saveDir = fs::path(homeDir) / req.destination.substr(2);
    }
    else
    {
        saveDir = fs::path(homeDir) / req.destination;
    }
    return saveDir.lexically_normal();
}
} // namespace

bool MapExporter::executeSave(
    const liorf::srv::SaveMap::Request &req,
    liorf::srv::SaveMap::Response &res,
    const std::string &savePCDDirectory,
    float mappingSurfLeafSize,
    bool saveDenseGpsTrajectory,
    bool saveDenseOdomTrajectory,
    const sensor_msgs::msg::NavSatFix &storedOriginGpsMsg,
    bool hasGpsOrigin,
    const pcl::PointCloud<PointType>::ConstPtr &cloudKeyPoses3D,
    const pcl::PointCloud<PointTypePose>::ConstPtr &cloudKeyPoses6D,
    const std::vector<pcl::PointCloud<PointType>::Ptr> &surfCloudKeyFrames,
    bool hasEnuLocal,
    const gtsam::Pose3 &tEnuLocal,
    const std::vector<sensor_msgs::msg::NavSatFix> &gpsHistory,
    const std::vector<nav_msgs::msg::Odometry> &densePoseHistory,
    const rclcpp::Logger &logger,
    const rclcpp::Time &saveTime) const
{
    namespace fs = std::filesystem;

    const char *home_env = std::getenv("HOME");
    const std::string homeDir = (home_env != nullptr) ? home_env : "/tmp";
    const fs::path saveDir = resolveSaveDirectory(req, savePCDDirectory, homeDir);
    const std::string saveMapDirectory = saveDir.string();
    const fs::path mapsDir = saveDir / "maps";
    const fs::path trajectoriesDir = saveDir / "trajectories";
    const fs::path keyframeCloudsDir = trajectoriesDir / "keyframes_deskewed_ds";

    res.save_directory = saveMapDirectory;
    res.enu_map_saved = false;
    res.keyframes_used = static_cast<uint32_t>(cloudKeyPoses3D->size());
    res.surf_points_local = 0;
    res.surf_points_enu = 0;
    res.full_points_local = 0;
    res.full_points_enu = 0;
    res.trajectory_points_saved = 0;
    res.gps_points_saved = 0;
    res.message = "";

    std::cout << "****************************************************" << std::endl;
    std::cout << "Saving map to pcd files ..." << std::endl;
    std::cout << "Save destination: " << saveMapDirectory << std::endl;

    {
      std::error_code ec;
      fs::remove_all(saveDir, ec);
      fs::create_directories(mapsDir, ec);
      if (ec)
      {
          RCLCPP_ERROR(logger, "Failed to create maps directory '%s': %s", mapsDir.string().c_str(), ec.message().c_str());
          res.success = false;
          res.message = std::string("failed to create maps directory: ") + ec.message();
          return true;
      }

      fs::create_directories(trajectoriesDir, ec);
      if (ec)
      {
          RCLCPP_ERROR(logger, "Failed to create save directory '%s': %s", saveMapDirectory.c_str(), ec.message().c_str());
          res.success = false;
          res.message = std::string("failed to create save directory: ") + ec.message();
          return true;
      }

      fs::create_directories(keyframeCloudsDir, ec);
      if (ec)
      {
          RCLCPP_ERROR(logger, "Failed to create keyframe directory '%s': %s", keyframeCloudsDir.string().c_str(), ec.message().c_str());
          res.success = false;
          res.message = std::string("failed to create keyframe directory: ") + ec.message();
          return true;
      }
    }

    {
        std::string metadata_file_path = saveMapDirectory + "/goereference.yaml";
        std::ofstream ofs(metadata_file_path);
        if (ofs.is_open())
        {
            ofs << std::fixed << std::setprecision(12);
            ofs << "gps_origin_enu:" << std::endl;

            if (hasGpsOrigin)
            {
                ofs << "  latitude: " << storedOriginGpsMsg.latitude << std::endl;
                ofs << "  longitude: " << storedOriginGpsMsg.longitude << std::endl;
                ofs << "  altitude: " << storedOriginGpsMsg.altitude << std::endl;
            }
            else
            {
                ofs << "  latitude: null\n  longitude: null\n  altitude: null" << std::endl;
            }

            ofs << "T_enu_local:" << std::endl;
            if (hasEnuLocal)
            {
                const auto q = tEnuLocal.rotation().toQuaternion();
                ofs << "  x: " << tEnuLocal.translation().x() << std::endl;
                ofs << "  y: " << tEnuLocal.translation().y() << std::endl;
                ofs << "  z: " << tEnuLocal.translation().z() << std::endl;
                ofs << "  qx: " << q.x() << std::endl;
                ofs << "  qy: " << q.y() << std::endl;
                ofs << "  qz: " << q.z() << std::endl;
                ofs << "  qw: " << q.w() << std::endl;
            }
            else
            {
                ofs << "  x: 0.0\n  y: 0.0\n  z: 0.0\n  qx: 0.0\n  qy: 0.0\n  qz: 0.0\n  qw: 1.0" << std::endl;
            }

            ofs.close();
            std::cout << "Map metadata (Datum + Transform) successfully saved to: " << metadata_file_path << std::endl;
        }
    }

    std::vector<std::string> auxWarnings;
    try
    {
        const fs::path templateReadmePath =
            fs::path(ament_index_cpp::get_package_share_directory("liorf")) / "scripts" / "saved_map_output_README.md";
        std::error_code copyEc;
        fs::copy_file(templateReadmePath, saveDir / "README.md", fs::copy_options::overwrite_existing, copyEc);
        if (copyEc)
            auxWarnings.emplace_back(std::string("failed to copy README template: ") + copyEc.message());
    }
    catch (const std::exception &e)
    {
        auxWarnings.emplace_back(std::string("failed to resolve README template: ") + e.what());
    }

    pcl::io::savePCDFileBinary((trajectoriesDir / "trajectory_local.pcd").string(), *cloudKeyPoses3D);
    pcl::io::savePCDFileBinary((trajectoriesDir / "transformations_local.pcd").string(), *cloudKeyPoses6D);

    pcl::PointCloud<PointType>::Ptr fullSurfaceCloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr fullSurfaceCloudDS(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr fullMapCloud(new pcl::PointCloud<PointType>());

    for (int i = 0; i < static_cast<int>(cloudKeyPoses3D->size()); i++)
    {
        *fullSurfaceCloud += *transformPointCloud(surfCloudKeyFrames[i], cloudKeyPoses6D->points[i]);
        std::cout << "\r" << std::flush << "Processing feature cloud " << i << " of " << cloudKeyPoses6D->size() << " ...";
    }

    pcl::VoxelGrid<PointType> downSizeFilterSurf;
    downSizeFilterSurf.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);

    if (req.resolution != 0)
    {
        std::cout << "\n\nSave resolution: " << req.resolution << std::endl;
        downSizeFilterSurf.setInputCloud(fullSurfaceCloud);
        downSizeFilterSurf.setLeafSize(req.resolution, req.resolution, req.resolution);
        downSizeFilterSurf.filter(*fullSurfaceCloudDS);
        pcl::io::savePCDFileBinary((mapsDir / "SurfaceMap_local.pcd").string(), *fullSurfaceCloudDS);
    }
    else
    {
        pcl::io::savePCDFileBinary((mapsDir / "SurfaceMap_local.pcd").string(), *fullSurfaceCloud);
    }

    *fullMapCloud += *fullSurfaceCloud;
    res.surf_points_local = static_cast<uint32_t>(fullSurfaceCloud->size());
    res.full_points_local = static_cast<uint32_t>(fullMapCloud->size());

    const int ret = pcl::io::savePCDFileBinary((mapsDir / "FullMap_local.pcd").string(), *fullMapCloud);

    if (hasEnuLocal)
    {
        const Eigen::Affine3f tGlobalLocalToEnu = pcl::getTransformation(
            static_cast<float>(tEnuLocal.translation().x()),
            static_cast<float>(tEnuLocal.translation().y()),
            static_cast<float>(tEnuLocal.translation().z()),
            static_cast<float>(tEnuLocal.rotation().roll()),
            static_cast<float>(tEnuLocal.rotation().pitch()),
            static_cast<float>(tEnuLocal.rotation().yaw()));

        pcl::PointCloud<PointType>::Ptr fullSurfaceCloudEnu(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr fullMapCloudEnu(new pcl::PointCloud<PointType>());
        pcl::transformPointCloud(*fullSurfaceCloud, *fullSurfaceCloudEnu, tGlobalLocalToEnu);
        *fullMapCloudEnu += *fullSurfaceCloudEnu;
        res.enu_map_saved = true;
        res.surf_points_enu = static_cast<uint32_t>(fullSurfaceCloudEnu->size());
        res.full_points_enu = static_cast<uint32_t>(fullMapCloudEnu->size());

        pcl::io::savePCDFileBinary((mapsDir / "SurfaceMap_ENU.pcd").string(), *fullSurfaceCloudEnu);
        pcl::io::savePCDFileBinary((mapsDir / "FullMap_ENU.pcd").string(), *fullMapCloudEnu);
    }
    else
    {
        RCLCPP_WARN(logger, "ENU map export skipped: T_enu_local is not initialized yet.");
        res.message = "ENU map export skipped: T_enu_local is not initialized yet.";
    }

    std::vector<std::string> csvErrors;
    if (saveDenseGpsTrajectory)
    {
        std::string gpsCsvPath = (trajectoriesDir / "gps_raw_geodetic.csv").string();
        std::ofstream gps_ofs(gpsCsvPath);
        if (gps_ofs.is_open())
        {
            gps_ofs << std::fixed << std::setprecision(9);
            gps_ofs << "timestamp_sec,latitude,longitude,altitude,status\n";
            for (const auto &gps : gpsHistory)
            {
                gps_ofs << stampToSec(gps.header.stamp) << ","
                        << gps.latitude << ","
                        << gps.longitude << ","
                        << gps.altitude << ","
                        << static_cast<int>(gps.status.status) << "\n";
            }
            gps_ofs.close();
            res.gps_points_saved = static_cast<uint32_t>(gpsHistory.size());
        }
        else
        {
            csvErrors.emplace_back("failed to write trajectories/gps_raw_geodetic.csv");
        }
    }

    {
        std::string keyframeCsvPath = (trajectoriesDir / "trajectory_keyframes_local.csv").string();
        std::ofstream keyframe_ofs(keyframeCsvPath);
        if (keyframe_ofs.is_open())
        {
            keyframe_ofs << std::fixed << std::setprecision(9);
            keyframe_ofs << "timestamp_sec,index,x_local,y_local,z_local,roll,pitch,yaw,cloud_relpath,cloud_points\n";
            for (size_t i = 0; i < cloudKeyPoses6D->size(); ++i)
            {
                const auto &pose6D = cloudKeyPoses6D->points[i];

                std::ostringstream cloudName;
                cloudName << "kf_" << std::setw(6) << std::setfill('0') << i << ".pcd";
                const fs::path cloudPath = keyframeCloudsDir / cloudName.str();
                const fs::path cloudRelPath = fs::path("trajectories") / "keyframes_deskewed_ds" / cloudName.str();

                size_t cloudPointCount = 0;
                if (i < surfCloudKeyFrames.size() && surfCloudKeyFrames[i])
                {
                    pcl::io::savePCDFileBinary(cloudPath.string(), *surfCloudKeyFrames[i]);
                    cloudPointCount = surfCloudKeyFrames[i]->size();
                }

                keyframe_ofs << pose6D.time << ","
                             << i << ","
                             << pose6D.x << ","
                             << pose6D.y << ","
                             << pose6D.z << ","
                             << pose6D.roll << ","
                             << pose6D.pitch << ","
                             << pose6D.yaw << ","
                             << cloudRelPath.string() << ","
                             << cloudPointCount << "\n";
            }
            keyframe_ofs.close();
        }
        else
        {
            csvErrors.emplace_back("failed to write trajectories/trajectory_keyframes_local.csv");
        }
    }

    if (saveDenseOdomTrajectory)
    {
        std::string denseCsvPath = (trajectoriesDir / "trajectory_dense_local.csv").string();
        std::ofstream dense_ofs(denseCsvPath);
        if (dense_ofs.is_open())
        {
            dense_ofs << std::fixed << std::setprecision(9);
            dense_ofs << "timestamp_sec,index,x_local,y_local,z_local,qx,qy,qz,qw\n";
            for (size_t i = 0; i < densePoseHistory.size(); ++i)
            {
                const auto &pose = densePoseHistory[i];
                dense_ofs << stampToSec(pose.header.stamp) << ","
                          << i << ","
                          << pose.pose.pose.position.x << ","
                          << pose.pose.pose.position.y << ","
                          << pose.pose.pose.position.z << ","
                          << pose.pose.pose.orientation.x << ","
                          << pose.pose.pose.orientation.y << ","
                          << pose.pose.pose.orientation.z << ","
                          << pose.pose.pose.orientation.w << "\n";
            }
            dense_ofs.close();
            res.trajectory_points_saved = static_cast<uint32_t>(densePoseHistory.size());
        }
        else
        {
            csvErrors.emplace_back("failed to write trajectories/trajectory_dense_local.csv");
        }
    }

    {
        std::string summaryYamlPath = saveMapDirectory + "/save_summary.yaml";
        std::ofstream summary_ofs(summaryYamlPath);
        if (summary_ofs.is_open())
        {
            summary_ofs << std::fixed << std::setprecision(9);
            summary_ofs << "save_summary:" << std::endl;
            summary_ofs << "  map_creation_ros_time_sec: " << saveTime.seconds() << std::endl;
            summary_ofs << "  save_directory: \"" << saveMapDirectory << "\"" << std::endl;
            summary_ofs << "  resolution: " << req.resolution << std::endl;
            summary_ofs << "  enu_map_saved: " << (res.enu_map_saved ? "true" : "false") << std::endl;
            summary_ofs << "  keyframes_used: " << res.keyframes_used << std::endl;
            summary_ofs << "  trajectory_points_saved: " << res.trajectory_points_saved << std::endl;
            summary_ofs << "  gps_points_saved: " << res.gps_points_saved << std::endl;
            summary_ofs << "  surf_points_local: " << res.surf_points_local << std::endl;
            summary_ofs << "  surf_points_enu: " << res.surf_points_enu << std::endl;
            summary_ofs << "  full_points_local: " << res.full_points_local << std::endl;
            summary_ofs << "  full_points_enu: " << res.full_points_enu << std::endl;
            summary_ofs << "  save_dense_gps_trajectory: " << (saveDenseGpsTrajectory ? "true" : "false") << std::endl;
            summary_ofs << "  save_dense_odom_trajectory: " << (saveDenseOdomTrajectory ? "true" : "false") << std::endl;
            summary_ofs.close();
        }
        else
        {
            auxWarnings.emplace_back("failed to write save_summary.yaml");
        }
    }

    res.success = ret == 0;
    if (res.success)
    {
        const fs::path lastSavedMapPathFile = fs::path(homeDir) / ".liorf_last_saved_map_path";
        std::ofstream lastPathOut(lastSavedMapPathFile.string(), std::ios::trunc);
        if (lastPathOut.is_open())
        {
            lastPathOut << saveMapDirectory << std::endl;
            lastPathOut.close();
        }
        else
        {
            RCLCPP_WARN(logger, "Failed to write last saved map path file: %s", lastSavedMapPathFile.string().c_str());
        }
    }

    if (res.success && res.message.empty())
        res.message = "map saved successfully";
    if (!res.success && res.message.empty())
        res.message = "failed to save maps/FullMap_local.pcd";

    if (!csvErrors.empty())
    {
        std::ostringstream csvErr;
        for (size_t i = 0; i < csvErrors.size(); ++i)
        {
            if (i > 0)
                csvErr << "; ";
            csvErr << csvErrors[i];
        }

        if (!res.message.empty())
            res.message += "; ";
        res.message += csvErr.str();
    }

    if (!auxWarnings.empty())
    {
        std::ostringstream warnErr;
        for (size_t i = 0; i < auxWarnings.size(); ++i)
        {
            if (i > 0)
                warnErr << "; ";
            warnErr << auxWarnings[i];
        }

        if (!res.message.empty())
            res.message += "; ";
        res.message += warnErr.str();
    }

    std::cout << "****************************************************" << std::endl;
    std::cout << "Saving map to pcd files completed\n" << std::endl;

    return true;
}
