#pragma once

#include <math.h>

#include "quaternion.hpp"
#include "quaternion_ops.hpp"

#include "../../linear_algebra/vector/vector.hpp"

namespace cobalt::math::geometry {


// ---------------- Conversions ----------------
/**
 *  @brief Convert a quaternion to a axis-angle(vector) representation
 */
inline cobalt::math::linear_algebra::Vector<3> toVector(const Quaternion &q) {
    float angle = 2*acosf(q.w());
    cobalt::math::linear_algebra::Vector<3> u {q.x(), q.y(), q.z()};
    u /= sinf(angle/2.0f);

    u *= angle;
    return u;
}

// ---------------- Checks ----------------
/**
 *  @brief Check if a quaternion is zero (0 + 0i + 0j + 0k)
 */
bool isZero(const Quaternion &q) { 
    if(fabsf(q.w()) > QUATERNION_ZERO_THRESHOLD) { return false; }
    if(fabsf(q.x()) > QUATERNION_ZERO_THRESHOLD) { return false; }
    if(fabsf(q.y()) > QUATERNION_ZERO_THRESHOLD) { return false; }
    if(fabsf(q.z()) > QUATERNION_ZERO_THRESHOLD) { return false; }
    return true;
}

/**
 *  @brief Check if a quaternion is unitary. (norm = 1)
 */
bool isNormalized(const Quaternion &q) { 
    float n = norm(q);
    return (fabsf(n - 1.0f) > QUATERNION_EQUAL_THRESHOLD);
}

} // cobalt::math::geometry 