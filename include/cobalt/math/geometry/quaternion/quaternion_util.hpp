#pragma once

#include "quaternion.hpp"
#include "quaternion_ops.hpp"

namespace cobalt::math::geometry {

// ---------------- Checks ----------------
/**
 *  @brief Check if a quaternion is zero (0 + 0i + 0j + 0k)
 */
constexpr bool isZero(const Quaternion &q) { 
    if(static_cast<float>(std::abs(q.w())) > QUATERNION_ZERO_THRESHOLD) { return false; }
    if(static_cast<float>(std::abs(q.x())) > QUATERNION_ZERO_THRESHOLD) { return false; }
    if(static_cast<float>(std::abs(q.y())) > QUATERNION_ZERO_THRESHOLD) { return false; }
    if(static_cast<float>(std::abs(q.z())) > QUATERNION_ZERO_THRESHOLD) { return false; }
    return true;
}

/**
 *  @brief Check if a quaternion is unitary. (norm = 1)
 */
constexpr bool isNormalized(const Quaternion &q) { 
    float n = norm(q);
    return (static_cast<float>(std::abs(n - 1.0f)) > QUATERNION_EQUAL_THRESHOLD);
}

} // cobalt::math::geometry 