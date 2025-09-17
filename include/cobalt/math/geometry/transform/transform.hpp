#pragma once

#include <array>

#include "../../linear_algebra/matrix/matrix.hpp"
#include "../../linear_algebra/vector/vector.hpp"

namespace cobalt::math::geometry {

// --------------------------------------
//      Homogeneous Transformations    
// --------------------------------------

/**
 *  @brief Homogeneous transformation matrix.
 *  @tparam T Element type (default float).
 */
template<typename T = float>
struct Transform {
    private:
        std::array<T, 16> data_{};

    public: 
        // ---------------- Constructors ----------------
        /**
         *  @brief Construct an identity transformation matrix.
         */
        constexpr Transform() noexcept : data_{} {
            for(uint8_t i = 0; i < 4; i++) {
                data_[4*i + i] = static_cast<T>(1);
            }
        }

        /**
         *  @brief Construct an transformation matrix.
         *  @param R Rotation matrix associated with the transform
         *  @param t Translation vector associated with the transformn
         */
        constexpr Transform(const cobalt::math::linear_algebra::Matrix<3, 3, T> &R, const cobalt::math::linear_algebra::Vector<3, T> &t) noexcept {
            for(uint8_t i = 0; i < 4; i++) {
                for(uint8_t j = 0; j < 4; j++) {
                    if(i < 3 && j < 3) { data_[4*i + j] = R(i, j); }
                    else if(i < 3 && j == 3) { data_[4*i + j] = t[i]; }
                    else if(i == 3 && j < 3) { data_[4*i + j] = static_cast<T>(0); }
                    else { data_[4*i + j] = static_cast<T>(1); }
                }
            }
        }

        // ---------------- Static Factories ----------------
        constexpr Transform<T> eye() {
            Transform<T> out{};

            for(uint8_t i = 0; i < 4; i++) {
                out(i, i) = static_cast<T>(1);
            }

            return out;
        }

        // ---------------- Getters ----------------
        /**
         *  @brief Return the row number of the transformation matrix.
         */
        constexpr uint8_t rows() const { return 4; }

        /**
         *  @brief Return the column number of the transformation matrix.
         */
        constexpr uint8_t cols() const { return 4; }

        // ---------------- Element Accessors ----------------
        /**
         *  @brief Access element at the given row/column.
         *  @param r Row of the accessed element.
         *  @param c Column of the accessed element.
         *  @return Reference to element.
         */
        constexpr T &operator()(uint8_t r, uint8_t c) { if(r >= 4) { r = 3; } if(c >= 4) { c = 3; } return data_[r*4 + c]; }

        /**
         *  @brief Const access to element at the given row/column.
         *  @param r Row of the accessed element.
         *  @param c Column of the accessed element.
         *  @return Const reference to element.
         */
        const T &operator()(uint8_t r, uint8_t c) const { if(r >= 4) { r = 3; } if(c >= 4) { c = 3; } return data_[r*4 + c]; }

        // ---------------- Member Overloads ----------------
        /**
         *  @brief Right-multiply another transform matrix(4x4) to this transform matrix(4x4).
         *  @return Resulting transform matrix (4x4)
         */
        const Transform<T> &operator*=(const Transform<T> &rhs) {
            Transform<T> output{};

                for(uint8_t i = 0; i < 4; i++) {
                    for(uint8_t j = 0; j < 4; j++) {
                        output(i, j) = static_cast<T>(0);

                        for(uint8_t k = 0; k < 4; k++) {
                            output(i, j) += data_[i*4 + k] * rhs(k, j);
                        }
                    }
                }

                *this = output;

                return *this;
        }

        // ---------------- Member Functions ----------------
        /**
         *  @brief Apply a homogeneous transformation to a 3-Vector
         *  @return Transformed 3-Vector
         */
        const cobalt::math::linear_algebra::Vector<3, T> apply(cobalt::math::linear_algebra::Vector<3, T> v) const {
            cobalt::math::linear_algebra::Vector<4, T> temp = {v[0], v[1], v[2], static_cast<T>(1)};

            for(uint8_t i = 0; i < 3; i++) {
                v[i] = static_cast<T>(0);

                for(uint8_t j = 0; j < 4; j++) {
                    v[i] += data_[i*4 + j] * temp[j];
                }
            }

            return v;
        }


};

} // cobalt::math::geometry