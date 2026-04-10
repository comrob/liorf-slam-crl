#pragma once

#include <pcl/point_types.h>

typedef pcl::PointXYZI PointType;

struct PointXYZIRPYT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRPYT,
                                  (float, x, x)(float, y, y)(float, z, z)
                                  (float, intensity, intensity)
                                  (float, roll, roll)(float, pitch, pitch)(float, yaw, yaw)
                                  (double, time, time))

typedef PointXYZIRPYT PointTypePose;
