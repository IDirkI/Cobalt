#pragma once

#include "transform.hpp"
#include "transform_util.hpp"

#include "../../linear_algebra/vector/vector.hpp"
#include "../../linear_algebra/matrix/matrix.hpp"
#include "../../linear_algebra/matrix/matrix_ops.hpp"
#include "../../linear_algebra/matrix/matrix_util.hpp"

namespace cobalt::math::geometry {

// ---------------- Non-member Arithmetic Overloads ----------------
template<typename T = float>
    constexpr Transform<T> operator*(Transform<T> lhs, const Transform<T> &rhs) { lhs *= rhs; return lhs; }

template<typename T = float>
    constexpr cobalt::math::linear_algebra::Vector<4, T> operator*(const Transform<T> &lhs, cobalt::math::linear_algebra::Vector<4, T> v) {
        cobalt::math::linear_algebra::Vector<4, T> temp = v;

        for(uint8_t i = 0; i < 4; i++) {
            v[i] = static_cast<T>(0);

            for(uint8_t j = 0; j < 4; j++) {
                v[i] += lhs(i, j) * temp[j];
            }
        }

        return v;
    }

template<typename T = float>
    constexpr cobalt::math::linear_algebra::Vector<4, T> operator*(cobalt::math::linear_algebra::Vector<4, T> v, const Transform<T> &lhs) {
        cobalt::math::linear_algebra::Vector<4, T> temp = v;

        for(uint8_t i = 0; i < 4; i++) {
            v[i] = static_cast<T>(0);

            for(uint8_t j = 0; j < 4; j++) {
                v[i] += lhs(j, i) * temp[i];
            }
        }

        return v;
    }


// ---------------- Non-member Functions ----------------

template<typename T = float>
    constexpr Transform<T> inv(const Transform<T> &H) {
        cobalt::math::linear_algebra::Matrix<3, 3, T> RT = cobalt::math::linear_algebra::transpose(H.rotation());
        cobalt::math::linear_algebra::Vector<3, T> RTt = H.translation();
        RTt = -RT*RTt;

        Transform<T> output(RT, RTt);
        return output;
    }

} // cobalt::math::geometry