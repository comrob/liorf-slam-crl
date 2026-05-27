/**
 * @file      VoxelMapConfig.hpp
 * @brief     Lightweight configuration struct for VoxelMap
 */

#ifndef VOXEL_MAP_CONFIG_HPP
#define VOXEL_MAP_CONFIG_HPP

namespace lio {

struct VoxelMapConfig {
    float voxel_size = 0.5f;
    int hierarchy_factor = 3;
    float planarity_threshold = 0.1f;
    float point_to_surfel_threshold = 0.1f;
    int min_surfel_inliers = 3;
    float min_linearity_ratio = 0.3f;
    float map_box_multiplier = 2.0f;
};

} // namespace lio

#endif // VOXEL_MAP_CONFIG_HPP