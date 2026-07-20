// File: include/degeneracyDetection/TwistManipulation.hpp
#pragma once

 #include <cmath>
 #include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

using TwistVector = Eigen::Matrix<float, 6, 1>;

inline Eigen::Matrix4f expMap(const TwistVector &twist, float delta_time = 1.0f)
{
    Eigen::Matrix4f exp_twist = Eigen::Matrix4f::Identity();

    Eigen::Vector3f v = twist.segment<3>(0);
    Eigen::Vector3f omega = twist.segment<3>(3);

    Eigen::Matrix3f omega_hat;
    omega_hat << 0.0f, -omega(2), omega(1),
                 omega(2), 0.0f, -omega(0),
                -omega(1), omega(0), 0.0f;

    float theta = omega.norm();

    if (theta < 1e-3f)
    {
        Eigen::Matrix3f R = Eigen::Matrix3f::Identity();
        Eigen::Matrix3f J = Eigen::Matrix3f::Identity() * delta_time;
        Eigen::Vector3f t = J * v;

        exp_twist.block<3, 3>(0, 0) = R;
        exp_twist.block<3, 1>(0, 3) = t;
        return exp_twist;
    }

    Eigen::Matrix3f R = Eigen::Matrix3f::Identity() +
                        (sin(theta * delta_time) / theta) * omega_hat +
                        ((1.0f - cos(theta * delta_time)) / (theta * theta)) * (omega_hat * omega_hat);

    Eigen::Matrix3f I = Eigen::Matrix3f::Identity();
    Eigen::Matrix3f J = I * delta_time +
                        ((1.0f - cos(theta * delta_time)) / (theta * theta)) * omega_hat +
                        ((theta * delta_time - sin(theta * delta_time)) / (theta * theta * theta)) * (omega_hat * omega_hat);

    Eigen::Vector3f t = J * v;

    exp_twist.block<3, 3>(0, 0) = R;
    exp_twist.block<3, 1>(0, 3) = t;

    return exp_twist;
}

inline TwistVector matrixToTwist(const Eigen::Matrix4f &T_delta, float time = 1.0f)
{
    TwistVector twist;

    Eigen::Matrix3f R_delta = T_delta.block<3, 3>(0, 0);
    Eigen::Vector3f t_delta = T_delta.block<3, 1>(0, 3);

    Eigen::AngleAxisf angle_axis(R_delta);
    float theta = angle_axis.angle();
    Eigen::Vector3f axis = angle_axis.axis();

    Eigen::Vector3f angular_velocity = (theta / time) * axis;
    Eigen::Vector3f linear_velocity;

    if (theta < 1e-5f)
    {
        linear_velocity = t_delta / time;
    }
    else
    {
        Eigen::Matrix3f omega_hat;
        omega_hat << 0.0f, -angular_velocity(2), angular_velocity(1),
                     angular_velocity(2), 0.0f, -angular_velocity(0),
                    -angular_velocity(1), angular_velocity(0), 0.0f;

        Eigen::Matrix3f I = Eigen::Matrix3f::Identity();
        Eigen::Matrix3f J = I + (1.0f - cos(theta)) / (theta * theta) * omega_hat +
                            (theta - sin(theta)) / (theta * theta * theta) * (omega_hat * omega_hat);

        linear_velocity = J.inverse() * t_delta / time;
    }

    twist.segment<3>(0) = linear_velocity;
    twist.segment<3>(3) = angular_velocity;

    return twist;
}

// Projection utility
inline TwistVector projectOntoBasis(const TwistVector &twist, const std::vector<TwistVector> &basis)
{
    TwistVector projectedTwist = TwistVector::Zero();
    for (const auto &basisVector : basis)
    {
        float projectionScalar = twist.dot(basisVector) / basisVector.dot(basisVector);
        projectedTwist += projectionScalar * basisVector;
    }
    return projectedTwist;
}

// Project a 3D translation onto the translation subspace spanned by the
// linear parts of the given twist basis. Gram-Schmidt is applied because
// orthonormality of the full 6D twists does not imply orthonormality of
// their linear parts.
inline Eigen::Vector3f projectOntoBasisTranslation(const Eigen::Vector3f &t,
                                                   const std::vector<TwistVector> &basis)
{
    std::vector<Eigen::Vector3f> lin;
    for (const auto &b : basis)
    {
        Eigen::Vector3f v = b.head<3>();
        for (const auto &u : lin)
            v -= v.dot(u) * u;
        if (v.norm() > 1e-6f)
            lin.push_back(v.normalized());
    }
    Eigen::Vector3f proj = Eigen::Vector3f::Zero();
    for (const auto &u : lin)
        proj += t.dot(u) * u;
    return proj;
}
