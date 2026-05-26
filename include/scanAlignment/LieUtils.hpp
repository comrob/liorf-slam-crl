/**
 * @file      LieUtils.hpp
 * @brief     Lie group utilities for SO(3) and SE(3) operations without Sophus dependency
 */

#ifndef LIE_UTILS_HPP
#define LIE_UTILS_HPP

#include <Eigen/Dense>
#include <cmath>

namespace lio {

constexpr float kEpsilon = 1e-6f;
constexpr float kPi = 3.14159f;

class SO3 {
public:
    SO3() = default;
    explicit SO3(const Eigen::Matrix3f& R);
    void Normalize();
    static SO3 Exp(const Eigen::Vector3f& omega);
    Eigen::Vector3f Log() const;
    const Eigen::Matrix3f& Matrix() const { return m_matrix; }
    Eigen::Matrix3f& Matrix() { return m_matrix; }
    SO3 operator*(const SO3& other) const {
        return SO3(m_matrix * other.m_matrix);
    }
    Eigen::Vector3f operator*(const Eigen::Vector3f& v) const {
        return m_matrix * v;
    }
    SO3 Inverse() const {
        return SO3(m_matrix.transpose());
    }
    static SO3 Identity() {
        return SO3(Eigen::Matrix3f::Identity());
    }
    
private:
    Eigen::Matrix3f m_matrix = Eigen::Matrix3f::Identity();
};

class SE3 {
public:
    SE3() = default;
    SE3(const SO3& rotation, const Eigen::Vector3f& translation) 
        : m_rotation(rotation), m_translation(translation) {}
    SE3(const Eigen::Matrix3f& R, const Eigen::Vector3f& t)
        : m_rotation(R), m_translation(t) {}
    explicit SE3(const Eigen::Matrix4f& matrix);
    
    static SE3 FromMatrix(const Eigen::Matrix4f& T);
    static SE3 Exp(const Eigen::Matrix<float, 6, 1>& xi);
    Eigen::Matrix<float, 6, 1> Log() const;
    Eigen::Matrix4f Matrix() const;
    
    const SO3& Rotation() const { return m_rotation; }
    SO3& Rotation() { return m_rotation; }
    Eigen::Matrix3f RotationMatrix() const { return m_rotation.Matrix(); }
    const Eigen::Vector3f& Translation() const { return m_translation; }
    Eigen::Vector3f& Translation() { return m_translation; }
    
    SE3 operator*(const SE3& other) const {
        return SE3(m_rotation * other.m_rotation,
                   m_translation + m_rotation * other.m_translation);
    }
    Eigen::Vector3f operator*(const Eigen::Vector3f& p) const {
        return m_rotation * p + m_translation;
    }
    SE3 Inverse() const {
        SO3 R_inv = m_rotation.Inverse();
        return SE3(R_inv, R_inv * (-m_translation));
    }
    static SE3 Identity() {
        return SE3(SO3::Identity(), Eigen::Vector3f::Zero());
    }
    
private:
    SO3 m_rotation;
    Eigen::Vector3f m_translation = Eigen::Vector3f::Zero();
};

Eigen::Matrix3f Hat(const Eigen::Vector3f& v);
Eigen::Vector3f Vee(const Eigen::Matrix3f& S);

} // namespace lio

#endif // LIE_UTILS_HPP