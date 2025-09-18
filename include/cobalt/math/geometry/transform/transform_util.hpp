#pragma once

#include "transform.hpp"

#include "../../linear_algebra/matrix/matrix.hpp"

namespace cobalt::math::geometry {

template<typename T = float>
    constexpr cobalt::math::linear_algebra::Matrix<4, 4, T> toMatrix(const Transform<T> &H) {
        cobalt::math::linear_algebra::Matrix<4, 4, T> output{};

        for(uint8_t i = 0; i < 4; i++) {
            for(uint8_t j = 0; j < 4; j++) {
                output(i, j) = H(i, j);
            }
        }

        return output;
    }

} // cobalt::math::geometry