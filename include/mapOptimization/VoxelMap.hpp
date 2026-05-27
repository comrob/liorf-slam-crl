/**
 * @file      VoxelMap.hpp
 * @brief     Voxel-based hash map for efficient nearest neighbor search
 */

#ifndef VOXEL_MAP_HPP
#define VOXEL_MAP_HPP

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <mutex>
#include <Eigen/Dense>

#include "mapOptimization/VoxelMapConfig.hpp"

using PointType = pcl::PointXYZI;

namespace lio {

/**
 * @brief Voxel key for spatial hashing
 * Represents a 3D grid cell by integer indices (x, y, z)
 */
struct VoxelKey {
    int x, y, z;
    
    VoxelKey() : x(0), y(0), z(0) {}
    VoxelKey(int x_, int y_, int z_) : x(x_), y(y_), z(z_) {}
    
    bool operator==(const VoxelKey& other) const {
        return x == other.x && y == other.y && z == other.z;
    }
    
    // For debugging
    std::string ToString() const {
        return "(" + std::to_string(x) + ", " + std::to_string(y) + ", " + std::to_string(z) + ")";
    }
};

/**
 * @brief Z-order (Morton code) hash function for VoxelKey
 */
struct VoxelKeyHash {
private:
    static inline uint64_t ExpandBits(int32_t v) {
        uint64_t x = static_cast<uint64_t>(v + (1 << 20)) & 0x1fffff;  // 21 bits
        x = (x | (x << 32)) & 0x1f00000000ffffULL;
        x = (x | (x << 16)) & 0x1f0000ff0000ffULL;
        x = (x | (x << 8))  & 0x100f00f00f00f00fULL;
        x = (x | (x << 4))  & 0x10c30c30c30c30c3ULL;
        x = (x | (x << 2))  & 0x1249249249249249ULL;
        return x;
    }
    
public:
    std::size_t operator()(const VoxelKey& key) const {
        uint64_t morton = ExpandBits(key.x) | (ExpandBits(key.y) << 1) | (ExpandBits(key.z) << 2);
        return static_cast<std::size_t>(morton);
    }
};

class VoxelMap {
public:
    // explicit VoxelMap(float voxel_size = 0.5f);
    explicit VoxelMap(const VoxelMapConfig& config);
    void SetVoxelSize(float size);
    void SetMaxHitCount(int max_count) { m_max_hit_count = max_count; }
    int GetMaxHitCount() const { return m_max_hit_count; }
    void SetInitHitCount(int count) { m_init_hit_count = count; }
    int GetInitHitCount() const { return m_init_hit_count; }
    void SetHierarchyFactor(int factor);
    void SetPlanarityThreshold(float threshold) { m_planarity_threshold = threshold; }
    void SetPointToSurfelThreshold(float threshold) { m_point_to_surfel_threshold = threshold; }
    void SetMinSurfelInliers(int count) { m_min_surfel_inliers = count; }
    void SetMinLinearityRatio(float ratio) { m_min_linearity_ratio = ratio; }
    void SetMapBoxMultiplier(float multiplier) { m_map_box_multiplier = multiplier; }
    float GetMapBoxMultiplier() const { return m_map_box_multiplier; }
    Eigen::Vector3f GetMapCenter() const { return m_map_center; }
    bool IsMapInitialized() const { return m_map_initialized; }
    int GetHierarchyFactor() const { return m_hierarchy_factor; }
    float GetVoxelSize() const { return m_voxel_size; }
    
    void AddPoint(const PointType& point);
    void AddPointCloud(const pcl::PointCloud<PointType>::Ptr& cloud);
    void Clear();
    void UpdateVoxelMap(const pcl::PointCloud<PointType>::Ptr& new_cloud,
                        const Eigen::Vector3d& sensor_position,
                        double max_distance, bool is_keyframe);
    
    size_t GetPointCount() const {
        size_t total = 0;
        for (const auto& pair : m_voxels_L0) {
            total += pair.second.point_count;
        }
        return total;
    }
    
    size_t GetVoxelCount() const { return m_voxels_L0.size(); }
    
    std::vector<Eigen::Vector3f> GetL0Centroids() const {
        std::vector<Eigen::Vector3f> centroids;
        centroids.reserve(m_voxels_L0.size());
        for (const auto& pair : m_voxels_L0) {
            centroids.push_back(pair.second.centroid);
        }
        return centroids;
    }
    
    PointType GetCentroidPoint(const VoxelKey& key) const;
    std::vector<VoxelKey> GetOccupiedVoxels() const;
    Eigen::Vector3f VoxelKeyToCenter(const VoxelKey& key) const;
    Eigen::Vector3f GetVoxelCentroid(const VoxelKey& key) const;
    int GetVoxelHitCount(const VoxelKey& key) const;
    std::vector<std::pair<Eigen::Vector3f, int>> GetOccupiedVoxelsWithHitCount() const;
    void MarkVoxelAsHit(const VoxelKey& key);
    void ClearHitMarkers();
    bool IsVoxelHit(const VoxelKey& key) const;
    std::vector<VoxelKey> GetHitVoxels() const;
    std::vector<std::tuple<Eigen::Vector3f, Eigen::Vector3f, float, VoxelKey, int>> GetL1Surfels() const;
    
    bool GetSurfelAtPoint(const PointType& point,
                          Eigen::Vector3f& normal,
                          Eigen::Vector3f& centroid,
                          float& planarity_score) const;
    
private:
    VoxelKey PointToVoxelKey(const PointType& point, int level = 0) const;
    VoxelKey GetParentKey(const VoxelKey& key) const;
    void RegisterToParent(const VoxelKey& key_L0);
    void UnregisterFromParent(const VoxelKey& key_L0);
    std::vector<VoxelKey> GetNeighborVoxels(const VoxelKey& center, float search_distance) const;
    
    float m_voxel_size;
    int m_max_hit_count;
    int m_init_hit_count = 1;
    int m_hierarchy_factor;
    float m_planarity_threshold = 0.01f;
    float m_point_to_surfel_threshold = 0.1f;
    int m_min_surfel_inliers = 5;
    float m_min_linearity_ratio = 0.3f;
    
    float m_map_box_multiplier = 2.0f;
    Eigen::Vector3f m_map_center = Eigen::Vector3f::Zero();
    bool m_map_initialized = false;
    float m_max_distance = 100.0f;
    
    struct VoxelNode_L0 {
        Eigen::Vector3f centroid;
        int hit_count;
        int point_count;
        bool centroid_dirty;
        
        VoxelNode_L0() : centroid(Eigen::Vector3f::Zero()), hit_count(1), point_count(0), centroid_dirty(true) {}
    };
    std::unordered_map<VoxelKey, VoxelNode_L0, VoxelKeyHash> m_voxels_L0;
    
    struct VoxelNode_L1 {
        int hit_count;
        std::unordered_set<VoxelKey, VoxelKeyHash> occupied_children;
        
        bool has_surfel;
        Eigen::Vector3f surfel_normal;
        Eigen::Vector3f surfel_centroid;
        Eigen::Matrix3f surfel_covariance;
        float planarity_score;
        int last_child_count;
        
        VoxelNode_L1() 
            : hit_count(0)
            , has_surfel(false)
            , surfel_normal(Eigen::Vector3f::Zero())
            , surfel_centroid(Eigen::Vector3f::Zero())
            , surfel_covariance(Eigen::Matrix3f::Zero())
            , planarity_score(1.0f)
            , last_child_count(0) {}
    };
    std::unordered_map<VoxelKey, VoxelNode_L1, VoxelKeyHash> m_voxels_L1;
    
    std::unordered_map<VoxelKey, bool, VoxelKeyHash> m_hit_voxels;
    
    mutable std::recursive_mutex m_mutex;
};

} // namespace lio

#endif // VOXEL_MAP_HPP