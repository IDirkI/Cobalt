#pragma once

#include <cmath>

#include "quaternion.hpp"

#include "../../algebra/complex/complex.hpp"

namespace cobalt::math::geometry {

// ---------------- Non-member Overloads ----------------
inline Quaternion operator+(Quaternion lhs, const Quaternion &rhs) { lhs += rhs; return lhs; }
template<typename T>
    inline Quaternion operator+(Quaternion lhs, T c) { lhs += Quaternion(c); return lhs; }
template<typename T>
    inline Quaternion operator+(T c, Quaternion rhs) { return (rhs + c); }

inline Quaternion operator-(Quaternion lhs, const Quaternion &rhs) { lhs -= rhs; return lhs; }
template<typename T>
    inline Quaternion operator-(Quaternion lhs, T c) { lhs -= Quaternion(c); return lhs; }
template<typename T>
    inline Quaternion operator-(T c, const Quaternion &rhs) { return (Quaternion(c) - rhs); }

inline Quaternion operator*(Quaternion lhs, const Quaternion &rhs) { lhs *= rhs; return lhs; }
template<typename T>
    inline Quaternion operator*(Quaternion lhs, T c) { lhs *= c; return lhs; }
template<typename T>
    inline Quaternion operator*(T c, Quaternion rhs) { rhs *= c; return rhs; }

template<typename T>
    inline Quaternion operator/(Quaternion lhs, T c) { lhs /= c; return lhs; }
template<typename T>
    inline Quaternion operator/(float c, const Quaternion &rhs) { return (Quaternion(c) / rhs); }

inline const Quaternion operator-(Quaternion q) { q *= -1; return q; } 

inline bool operator==(const Quaternion &lhs, const Quaternion &rhs) { 
    if(std::abs(lhs.w() - rhs.w()) > QUATERNION_EQUAL_THRESHOLD) { return false; }
    if(std::abs(lhs.x() - rhs.x()) > QUATERNION_EQUAL_THRESHOLD) { return false; }
    if(std::abs(lhs.y() - rhs.y()) > QUATERNION_EQUAL_THRESHOLD) { return false; }
    if(std::abs(lhs.z() - rhs.z()) > QUATERNION_EQUAL_THRESHOLD) { return false; }
    return true;
}

inline bool operator!=(const Quaternion &lhs, const Quaternion &rhs) { return !(lhs == rhs); }

// ---------------- Non-member Functions ----------------
/**
 * @brief Norm of the quaternion
 * @return |q|
 */
constexpr float norm(const Quaternion &q) {
    return std::sqrt(q.w()*q.w() + q.x()*q.x() + q.y()*q.y() + q.z()*q.z());
}

/**
 * @brief Squared Norm of the quaternion
 * @return |q|^2
 */
constexpr float normSqr(const Quaternion &q) {
    return q.w()*q.w() + q.x()*q.x() + q.y()*q.y() + q.z()*q.z();
}

/**
 * @brief Conjugate of the quaternion
 * @return q̄
 */
inline Quaternion conj(const Quaternion &q) {
    return Quaternion(q.w(), -q.x(), -q.y(), -q.z());
}

/**
 * @brief Inverse of the quaternion
 * @return q^-1
 */
inline Quaternion inv(const Quaternion &q) {
    return conj(q) / normSqr(q);
}

/**
 * @brief Dot product of two quaternions
 * @return q · q
 */
constexpr float dot(const Quaternion &q, const Quaternion &p) {
    return q.w()*p.w() + q.x()*p.x() + q.y()*p.y() + q.z()*p.z();
}

/**
 * @brief Normalize the quaternion to unit length
 * @return q / |q|
 */
inline Quaternion normalize(const Quaternion &q) {
    return (q / norm(q));
}

/**
 * @brief Natural logarithm of the quaternion
 * @return log(q)
 */
inline Quaternion log(const Quaternion &q) {
    float qNorm = norm(q);
    cobalt::math::linear_algebra::Vector<3, float> v = q.vector();
    float vNorm = norm(v);

    if(vNorm < QUATERNION_EQUAL_THRESHOLD) {
        return Quaternion(std::log(qNorm), 0.0f, 0.0f, 0.0f);
    }

    float t = std::acos(q.w()/qNorm);
    cobalt::math::linear_algebra::Vector<3, float> vDir = v/vNorm;

    return Quaternion(std::log(qNorm), vDir.x()*t, vDir.y()*t, vDir.z()*t);
}

/**
 *  @brief Exponential of the quaternion
 *  @return exp(q)
 */
inline Quaternion exp(const Quaternion &q) {
    cobalt::math::linear_algebra::Vector<3, float> v = q.vector();
    float vNorm = norm(v);
    float expW = std::exp(q.w());

    if(vNorm < QUATERNION_EQUAL_THRESHOLD) {
        return Quaternion(expW, 0.0f, 0.0f, 0.0f);
    }

    cobalt::math::linear_algebra::Vector<3, float> vDir = v/vNorm;
    float cosV = std::cos(vNorm);
    float sinV = std::sin(vNorm);

    return Quaternion(
        expW*cosV,
        expW*vDir.x()*sinV,
        expW*vDir.y()*sinV,
        expW*vDir.z()*sinV
    );
}

/**
 *  @brief Power of the quaternion to a real exponent
 *  @return q^n
 */
inline Quaternion pow(const Quaternion &q, float n) {
    return exp(log(q) * n);
}

/**
 *  @brief Rotate a 3-Vector by a quaternion
 *  @return q * v * q^-1
 *  @note The quaternion is assumed to be normalized
 */
inline cobalt::math::linear_algebra::Vector<3, float> rotate(const Quaternion &q, const cobalt::math::linear_algebra::Vector<3> &v) {
    Quaternion p = Quaternion::pure(v);
    Quaternion qInv = inv(q);
    Quaternion result = q * p * qInv;

    return result.vector();
}


} // cobalt::math::geometry