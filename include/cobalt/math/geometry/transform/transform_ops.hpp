#pragma once

#include "transform.hpp"
#include "transform_util.hpp"

#include "../../linear_algebra/vector/vector.hpp"
#include "../../linear_algebra/matrix/matrix.hpp"
#include "../../linear_algebra/matrix/matrix_ops.hpp"
#include "../../linear_algebra/matrix/matrix_util.hpp"

namespace cobalt::math::geometry {

// ---------------- Non-member Arithmetic Overloads ----------------
template<typename T = def_floating, typename = std::enable_if_t<Floating<T>>>
    constexpr Transform<T> operator*(Transform<T> lhs, const Transform<T> &rhs) { lhs *= rhs; return lhs; }

template<typename T = def_floating, typename = std::enable_if_t<Floating<T>>>
    constexpr cobalt::math::linear_algebra::Vector<3, T> operator*(const Transform<T> &lhs, const cobalt::math::linear_algebra::Vector<3, T> &v) {
        return lhs.apply(v);
    }

template<typename T = def_floating, typename = std::enable_if_t<Floating<T>>>
    constexpr bool operator==(const Transform<T> &lhs, const Transform<T> &rhs) {
        return ((lhs.rotation() == rhs.rotation()) && (lhs.translation() == rhs.translation()));
    }

template<typename T = def_floating, typename = std::enable_if_t<Floating<T>>>
    constexpr bool operator!=(const Transform<T> &lhs, const Transform<T> &rhs) {
        return !(rhs == lhs);
    }

// ---------------- Non-member Functions ----------------
/**
 *  @brief Compute the inverse transformation of a given transformation
 *  @param H Transformation to invert
 *  @return H^-1
 */
template<typename T = def_floating, typename = std::enable_if_t<Floating<T>>>
    constexpr Transform<T> inv(const Transform<T> &H) {
        cobalt::math::geometry::Quaternion<T> qinv = inv(H.rotation());
        cobalt::math::linear_algebra::Vector<3, T> qinvt = static_cast<T>(-1) * (qinv*H.translation());

        return Transform<T>(qinv, qinvt);
    }

} // cobalt::math::geometry