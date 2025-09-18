#pragma once

#include <math.h>

#include "quaternion.hpp"

namespace cobalt::math::geometry {

// ---------------- Non-member Overloads ----------------
inline const Quaternion operator+(Quaternion lhs, const Quaternion &rhs) { lhs += rhs; return lhs; }

inline const Quaternion operator-(Quaternion lhs, const Quaternion &rhs) { lhs -= rhs; return lhs; }

inline const Quaternion operator*(Quaternion lhs, const Quaternion &rhs) { lhs *= rhs; return lhs; }
inline const Quaternion operator*(Quaternion lhs, float c) { lhs *= c; return lhs; }
inline const Quaternion operator*(float c, Quaternion lhs) { lhs *= c; return lhs; }

inline const Quaternion operator/(Quaternion lhs, float c) { lhs /= c; return lhs; }
inline const Quaternion operator/(float c, Quaternion lhs) { lhs /= c; return lhs; }

inline const Quaternion operator-(Quaternion q) { q *= -1; return q; } 

inline bool operator==(const Quaternion &lhs, const Quaternion &rhs) { 
    if(fabsf(lhs.w() - rhs.w()) > QUATERNION_EQUAL_THRESHOLD) { return false; }
    if(fabsf(lhs.x() - rhs.x()) > QUATERNION_EQUAL_THRESHOLD) { return false; }
    if(fabsf(lhs.y() - rhs.y()) > QUATERNION_EQUAL_THRESHOLD) { return false; }
    if(fabsf(lhs.z() - rhs.z()) > QUATERNION_EQUAL_THRESHOLD) { return false; }
    return true;
}

inline bool operator!=(const Quaternion &lhs, const Quaternion &rhs) { return !(lhs == rhs); }

// ---------------- Non-member Functions ----------------
/**
 * @brief Conjugate the quaternion
 */
inline Quaternion conj(const Quaternion &q) {
    return Quaternion(q.w(), -q.x(), -q.y(), -q.z());
}

/**
 * @brief Norm of the quaternion
 */
inline float norm(const Quaternion &q) {
    return sqrtf(q.w()*q.w() + q.x()*q.x() + q.y()*q.y() + q.z()*q.z());
}

/**
 * @brief Normalize the quaternion in the same orientation
 */
inline Quaternion normalize(Quaternion q) {
    float n = norm(q);

    q = q/n;
    return q;
}


} // cobalt::math::geometry