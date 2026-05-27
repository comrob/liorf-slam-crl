/**
 * @file      VoxelMap.cpp
 * @brief     Implementation of voxel-based hash map for efficient nearest neighbor search
 */

#include "mapOptimization/VoxelMap.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <rclcpp/rclcpp.hpp>

namespace lio {

VoxelMap::VoxelMap(const VoxelMapConfig& config) 
    : m_voxel_size(config.voxel_size)
    , m_max_hit_count(10) // default hardcoded
    , m_hierarchy_factor(config.hierarchy_factor)
    , m_planarity_threshold(config.planarity_threshold)
    , m_point_to_surfel_threshold(config.point_to_surfel_threshold)
    , m_min_surfel_inliers(config.min_surfel_inliers)
    , m_min_linearity_ratio(config.min_linearity_ratio)
    , m_map_box_multiplier(config.map_box_multiplier)
{
    
}

void VoxelMap::SetVoxelSize(float size) {
    if (size <= 0.0f) {
        throw std::invalid_argument("Voxel size must be positive");
    }
    if (std::abs(m_voxel_size - size) > 1e-6f) {
        m_voxel_size = size;
        Clear();
        RCLCPP_WARN(rclcpp::get_logger("VoxelMap"), "Voxel size changed - map cleared");
    }
}

void VoxelMap::SetHierarchyFactor(int factor) {
    if (factor <= 0 || factor % 2 == 0) {
        RCLCPP_ERROR(rclcpp::get_logger("VoxelMap"), "Hierarchy factor must be positive and odd. Got: %d", factor);
        return;
    }
    if (m_hierarchy_factor != factor) {
        m_hierarchy_factor = factor;
        Clear();
        RCLCPP_INFO(rclcpp::get_logger("VoxelMap"), "Hierarchy factor changed to %d - map cleared", factor);
    }
}

VoxelKey VoxelMap::PointToVoxelKey(const PointType& point, int level) const {
    float scale = m_voxel_size;
    if (level == 1) scale *= static_cast<float>(m_hierarchy_factor);
    
    int vx = static_cast<int>(std::floor(point.x / scale));
    int vy = static_cast<int>(std::floor(point.y / scale));
    int vz = static_cast<int>(std::floor(point.z / scale));
    return VoxelKey(vx, vy, vz);
}

VoxelKey VoxelMap::GetParentKey(const VoxelKey& key) const {
    int f = m_hierarchy_factor;
    return VoxelKey(
        key.x >= 0 ? key.x / f : (key.x - (f - 1)) / f,
        key.y >= 0 ? key.y / f : (key.y - (f - 1)) / f,
        key.z >= 0 ? key.z / f : (key.z - (f - 1)) / f
    );
}

void VoxelMap::RegisterToParent(const VoxelKey& key_L0) {
    std::lock_guard<std::recursive_mutex> lock(m_mutex);
    VoxelKey parent_L1 = GetParentKey(key_L0);
    m_voxels_L1[parent_L1].occupied_children.insert(key_L0);
}

void VoxelMap::UnregisterFromParent(const VoxelKey& key_L0) {
    std::lock_guard<std::recursive_mutex> lock(m_mutex);
    VoxelKey parent_L1 = GetParentKey(key_L0);
    
    auto it_L1 = m_voxels_L1.find(parent_L1);
    if (it_L1 == m_voxels_L1.end()) return;
    
    it_L1->second.occupied_children.erase(key_L0);
    if (it_L1->second.occupied_children.size() < 5) {
        it_L1->second.has_surfel = false;
    }
    if (it_L1->second.occupied_children.empty()) {
        m_voxels_L1.erase(it_L1);
    }
}

void VoxelMap::AddPoint(const PointType& point) {
    std::lock_guard<std::recursive_mutex> lock(m_mutex);
    VoxelKey key = PointToVoxelKey(point, 0);
    
    auto it = m_voxels_L0.find(key);
    bool was_empty = (it == m_voxels_L0.end());
    
    VoxelNode_L0& voxel_data = m_voxels_L0[key];
    Eigen::Vector3f point_vec(point.x, point.y, point.z);
    int n = voxel_data.point_count;
    
    if (n == 0) {
        voxel_data.centroid = point_vec;
        voxel_data.hit_count = m_init_hit_count;
        voxel_data.point_count = 1;
    } else {
        voxel_data.centroid = (voxel_data.centroid * n + point_vec) / (n + 1);
        voxel_data.point_count++;
    }
    
    if (was_empty) {
        RegisterToParent(key);
    }
}

void VoxelMap::AddPointCloud(const pcl::PointCloud<PointType>::Ptr& cloud) {
    std::lock_guard<std::recursive_mutex> lock(m_mutex);
    if (!cloud || cloud->empty()) {
        RCLCPP_WARN(rclcpp::get_logger("VoxelMap"), "UpdateVoxelMap called with empty point cloud");
        return;
    }

    for (size_t i = 0; i < cloud->size(); ++i) {
        AddPoint(cloud->at(i));
    }
}

std::vector<VoxelKey> VoxelMap::GetNeighborVoxels(const VoxelKey& center, float search_distance) const {
    std::vector<VoxelKey> neighbors;
    int search_range = static_cast<int>(std::ceil(search_distance / m_voxel_size));
    int grid_size = 2 * search_range + 1;
    neighbors.reserve(grid_size * grid_size * grid_size);
    
    for (int dx = -search_range; dx <= search_range; ++dx) {
        for (int dy = -search_range; dy <= search_range; ++dy) {
            for (int dz = -search_range; dz <= search_range; ++dz) {
                neighbors.emplace_back(center.x + dx, center.y + dy, center.z + dz);
            }
        }
    }
    
    return neighbors;
}

void VoxelMap::Clear() {
    std::lock_guard<std::recursive_mutex> lock(m_mutex);
    m_voxels_L0.clear();
    m_voxels_L1.clear();
}

void VoxelMap::UpdateVoxelMap(const pcl::PointCloud<PointType>::Ptr& new_cloud,
                               const Eigen::Vector3d& sensor_position,
                               double max_distance, bool is_keyframe) {
    std::lock_guard<std::recursive_mutex> lock(m_mutex);
    
    if (new_cloud->empty()) 
    {
        RCLCPP_WARN(rclcpp::get_logger("VoxelMap"), "UpdateVoxelMap called with empty point cloud");
        return;
    }
    
    if (!is_keyframe) {
        return; // Only add new points for keyframes
    }
    
    m_max_distance = static_cast<float>(max_distance);
    Eigen::Vector3f sensor_pos = sensor_position.cast<float>();
    
    float box_half_size = m_max_distance * m_map_box_multiplier;
    float recenter_threshold = m_max_distance / m_map_box_multiplier;
    
    if (!m_map_initialized) {
        m_map_center = sensor_pos;
        m_map_initialized = true;
        RCLCPP_INFO(rclcpp::get_logger("VoxelMap"), "Map initialized at center: (%.1f, %.1f, %.1f), box size: %.1fm x %.1fm", 
                     m_map_center.x(), m_map_center.y(), m_map_center.z(), 
                     box_half_size * 2.0f, box_half_size * 2.0f);
    }
    
    float dist_from_center = (sensor_pos - m_map_center).norm();
    bool need_recenter = dist_from_center > recenter_threshold;
    
    if (need_recenter) {
        // Removed unused variable 'old_center'
        m_map_center = sensor_pos;
        
        std::vector<VoxelKey> voxels_to_remove;
        for (const auto& pair : m_voxels_L0) {
            const VoxelKey& key = pair.first;
            Eigen::Vector3f voxel_center = VoxelKeyToCenter(key);
            
            bool outside_box = 
                std::abs(voxel_center.x() - m_map_center.x()) > box_half_size ||
                std::abs(voxel_center.y() - m_map_center.y()) > box_half_size ||
                std::abs(voxel_center.z() - m_map_center.z()) > box_half_size;
            
            if (outside_box) {
                voxels_to_remove.push_back(key);
            }
        }
        
        for (const auto& key : voxels_to_remove) {
            UnregisterFromParent(key);
            m_voxels_L0.erase(key);
        }
        
        std::vector<VoxelKey> L1_to_remove;
        for (const auto& pair : m_voxels_L1) {
            if (pair.second.occupied_children.empty()) {
                L1_to_remove.push_back(pair.first);
            }
        }
        for (const auto& key : L1_to_remove) {
            m_voxels_L1.erase(key);
        }
    }
    
    AddPointCloud(new_cloud);
    
    std::unordered_set<VoxelKey, VoxelKeyHash> affected_L1;
    for (const auto& pt : *new_cloud) {
        VoxelKey key_L1 = PointToVoxelKey(pt, 1);
        affected_L1.insert(key_L1);
    }
    
    for (const VoxelKey& key_L1 : affected_L1) {
        auto it_L1 = m_voxels_L1.find(key_L1);
        if (it_L1 == m_voxels_L1.end()) continue;
        
        VoxelNode_L1& node_L1 = it_L1->second;
        int current_child_count = node_L1.occupied_children.size();
        
        if (current_child_count < m_min_surfel_inliers) {
            node_L1.has_surfel = false;
            continue;
        }

        if (node_L1.has_surfel && node_L1.last_child_count == current_child_count) {
            continue;
        }

        std::vector<Eigen::Vector3f> collected_centroids;
        collected_centroids.reserve(node_L1.occupied_children.size());
        
        for (const VoxelKey& key_L0 : node_L1.occupied_children) {
            auto it_L0 = m_voxels_L0.find(key_L0);
            if (it_L0 == m_voxels_L0.end()) continue;
            collected_centroids.push_back(it_L0->second.centroid);
        }
        
        if (collected_centroids.size() < 3) {
            node_L1.has_surfel = false;
            continue;
        }
    
        Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
        for (const auto& pt : collected_centroids) {
            centroid += pt;
        }
        centroid /= static_cast<float>(collected_centroids.size());
        
        Eigen::Matrix3f covariance = Eigen::Matrix3f::Zero();
        for (const auto& pt : collected_centroids) {
            Eigen::Vector3f diff = pt - centroid;
            covariance += diff * diff.transpose();
        }
        covariance /= static_cast<float>(collected_centroids.size());
        
        Eigen::JacobiSVD<Eigen::Matrix3f> svd(covariance, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Vector3f singular_values = svd.singularValues();
        Eigen::Vector3f normal = svd.matrixU().col(2);
        float planarity = singular_values(2) / (singular_values(0) + 1e-6f);

        if (planarity > m_planarity_threshold) {
            node_L1.has_surfel = false;
            for (const VoxelKey& key_L0 : node_L1.occupied_children) {
                m_voxels_L0.erase(key_L0);
            }
            m_voxels_L1.erase(it_L1);
            continue;
        }
        
        node_L1.has_surfel = true;
        node_L1.surfel_normal = normal;
        node_L1.surfel_centroid = centroid;
        node_L1.surfel_covariance = covariance;
        node_L1.planarity_score = planarity;
        node_L1.last_child_count = current_child_count;
    }
}

std::vector<VoxelKey> VoxelMap::GetOccupiedVoxels() const {
    std::lock_guard<std::recursive_mutex> lock(m_mutex);
    std::vector<VoxelKey> occupied_voxels;
    occupied_voxels.reserve(m_voxels_L0.size());
    
    for (const auto& pair : m_voxels_L0) {
        if (pair.second.point_count > 0) {  
            occupied_voxels.push_back(pair.first);
        }
    }
    return occupied_voxels;
}

Eigen::Vector3f VoxelMap::VoxelKeyToCenter(const VoxelKey& key) const {
    float center_x = (key.x + 0.5f) * m_voxel_size;
    float center_y = (key.y + 0.5f) * m_voxel_size;
    float center_z = (key.z + 0.5f) * m_voxel_size;
    return Eigen::Vector3f(center_x, center_y, center_z);
}

Eigen::Vector3f VoxelMap::GetVoxelCentroid(const VoxelKey& key) const {
    auto it = m_voxels_L0.find(key);
    if (it == m_voxels_L0.end()) {
        return VoxelKeyToCenter(key);
    }
    return it->second.centroid;
}

PointType VoxelMap::GetCentroidPoint(const VoxelKey& key) const {
    auto it = m_voxels_L0.find(key);
    Eigen::Vector3f center;
    if (it == m_voxels_L0.end()) {
        center = VoxelKeyToCenter(key);
    } else {
        center = it->second.centroid;
    }
    PointType pt;
    pt.x = center.x();
    pt.y = center.y();
    pt.z = center.z();
    pt.intensity = 0.0f;
    return pt;
}

int VoxelMap::GetVoxelHitCount(const VoxelKey& key) const {
    std::lock_guard<std::recursive_mutex> lock(m_mutex);
    auto it = m_voxels_L0.find(key);
    if (it == m_voxels_L0.end()) {
        return 0;
    }
    return it->second.hit_count;
}

std::vector<std::pair<Eigen::Vector3f, int>> VoxelMap::GetOccupiedVoxelsWithHitCount() const {
    std::lock_guard<std::recursive_mutex> lock(m_mutex);
    std::vector<std::pair<Eigen::Vector3f, int>> result;
    result.reserve(m_voxels_L0.size());
    
    for (const auto& pair : m_voxels_L0) {
        if (pair.second.point_count > 0) {
            Eigen::Vector3f center = VoxelKeyToCenter(pair.first);
            result.emplace_back(center, pair.second.hit_count);
        }
    }
    return result;
}

void VoxelMap::MarkVoxelAsHit(const VoxelKey& key) {
    std::lock_guard<std::recursive_mutex> lock(m_mutex);
    m_hit_voxels[key] = true;
}

void VoxelMap::ClearHitMarkers() {
    std::lock_guard<std::recursive_mutex> lock(m_mutex);
    m_hit_voxels.clear();
}

bool VoxelMap::IsVoxelHit(const VoxelKey& key) const {
    std::lock_guard<std::recursive_mutex> lock(m_mutex);
    return m_hit_voxels.find(key) != m_hit_voxels.end();
}

std::vector<VoxelKey> VoxelMap::GetHitVoxels() const {
    std::lock_guard<std::recursive_mutex> lock(m_mutex);
    std::vector<VoxelKey> hit_voxels;
    hit_voxels.reserve(m_hit_voxels.size());
    for (const auto& pair : m_hit_voxels) {
        hit_voxels.push_back(pair.first);
    }
    return hit_voxels;
}

std::vector<std::tuple<Eigen::Vector3f, Eigen::Vector3f, float, VoxelKey, int>> VoxelMap::GetL1Surfels() const {
    std::lock_guard<std::recursive_mutex> lock(m_mutex);
    std::vector<std::tuple<Eigen::Vector3f, Eigen::Vector3f, float, VoxelKey, int>> surfels;
    
    for (const auto& pair : m_voxels_L1) {
        const VoxelKey& key = pair.first;
        const VoxelNode_L1& node = pair.second;
        
        if (node.has_surfel) {
            surfels.emplace_back(node.surfel_centroid, node.surfel_normal, node.planarity_score, key, node.hit_count);
        }
    }
    return surfels;
}

bool VoxelMap::GetSurfelAtPoint(const PointType& point,
                                 Eigen::Vector3f& normal,
                                 Eigen::Vector3f& centroid,
                                 float& planarity_score) const {
    VoxelKey key_L1 = PointToVoxelKey(point, 1);
    
    auto it = m_voxels_L1.find(key_L1);
    if (it == m_voxels_L1.end()) {
        return false;
    }
    
    const VoxelNode_L1& node = it->second;
    if (!node.has_surfel) {
        return false;
    }
    
    normal = node.surfel_normal;
    centroid = node.surfel_centroid;
    planarity_score = node.planarity_score;
    return true;
}

} // namespace lio