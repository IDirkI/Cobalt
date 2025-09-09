#pragma once

#include "quaternion.hpp"
#include "quaternion_ops.hpp"

#include "../../linear_algebra/vector/vector.hpp"

namespace cobalt::math::geometry {


// ---------------- Conversions ----------------
/**
 *  @brief Convert a quaternion to a axis-angle(vector) representation
 */
inline cobalt::math::linear_algebra::Vector<3> toVector(const Quaternion &q) {
    float angle = 2*static_cast<float>(std::acos(q.w()));
    cobalt::math::linear_algebra::Vector<3> u {q.x(), q.y(), q.z()};
    u /= static_cast<float>(std::sin(angle/2.0f));

    u *= angle;
    return u;
}

// ---------------- Checks ----------------
/**
 *  @brief Check if a quaternion is zero (0 + 0i + 0j + 0k)
 */
bool isZero(const Quaternion &q) { 
    if(static_cast<float>(std::abs(q.w())) > QUATERNION_ZERO_THRESHOLD) { return false; }
    if(static_cast<float>(std::abs(q.x())) > QUATERNION_ZERO_THRESHOLD) { return false; }
    if(static_cast<float>(std::abs(q.y())) > QUATERNION_ZERO_THRESHOLD) { return false; }
    if(static_cast<float>(std::abs(q.z())) > QUATERNION_ZERO_THRESHOLD) { return false; }
    return true;
}

/**
 *  @brief Check if a quaternion is unitary. (norm = 1)
 */
bool isNormalized(const Quaternion &q) { 
    float n = norm(q);
    return (static_cast<float>(std::abs(n - 1.0f)) > QUATERNION_EQUAL_THRESHOLD);
}

} // cobalt::math::geometry 