#pragma once

#include <stdint.h>
#include <cmath>
#include <array>
#include <string>

#include "../vector/vector.hpp"

namespace cobalt::math::linear_algebra {

constexpr uint8_t MATRIX_MAX_ROW_SIZE = 12;
constexpr uint8_t MATRIX_MAX_COL_SIZE = 12;

constexpr float   MATRIX_EQUAL_THRESHOLD = 1e-6;
constexpr float   MATRIX_ZERO_THRESHOLD = 1e-12;

constexpr uint8_t MATRIX_DEFAULT_PRECISION = 3;
constexpr uint8_t MATRIX_DEFAULT_SVD_ITERATIONS = 100;

// --------------------------------------
//          NxM - Matrix    
// --------------------------------------

/**
 *  @brief Fixed-size matrix.
 *  @tparam R Row count of the matrix.
 *  @tparam C Column count of the matrix.
 *  @tparam T Element type (default float).
 */
template<uint8_t R, uint8_t C, typename T = float>
struct Matrix {
    static_assert(R > 0                     , "[MATRIX Error] : Matrix rows must be positive.");
    static_assert(C > 0                     , "[MATRIX Error] : Matrix columns must be positive");
    static_assert(R <= MATRIX_MAX_ROW_SIZE  , "[MATRIX Error] : Matrix rows exceeds maximum size.");
    static_assert(C <= MATRIX_MAX_COL_SIZE  , "[MATRIX Error] : Matrix columns exceeds maximum size.");

    private:
        std::array<T, R*C> data_{};

    public:
        // ---------------- Constructors ----------------
        /**
         *  @brief Construct a zero-initialized matrix.
         */
        constexpr Matrix() noexcept : data_{} {}

        /**
         *  @brief Construct a matrix from an 2d initializer list.
         *  @param list2d 2D-Initializer list to copy values from.
         */ 
        constexpr Matrix(std::initializer_list<std::initializer_list<T>> list2d) noexcept {
            uint8_t i = 0, j = 0;

            for(std::initializer_list<T> list : list2d) {
                j = 0;
                for(T val : list) {
                    if(i < R && j < C) { data_[i*C + j] = val; }
                    j++;
                }

                for(; j < C; j++) { data_[i*C + j] = static_cast<T>(0); }

                i++;
                if(i >= R) break;
            }

            for(; i < R; i++) { 
                for(; j < C; j++) { data_[i*C + j] = static_cast<T>(0); }
            }
        }

        // ---------------- Static Factories ----------------

        /**
         *  @brief Construct a zero matrix.
         */
        static constexpr Matrix zero() noexcept { return Matrix(); }

        /**
         *  @brief Construct an identity matrix.
         */
        static constexpr Matrix eye() noexcept { 
            Matrix<R, C, T> out{};
            uint8_t d = (R <= C) ?R :C;

            for(uint8_t i = 0; i < d; i++) {
                out(i, i) = static_cast<T>(1);
            }

            return out;
        }

        /**
         *  @brief Convert a vector into a diagonal matrix
         * 
         *  Puts the vector elements along the diagonal of a matrix. If all the diagonals are not filled up, they are set to zero.
         * 
         *  @tparam R Diagonal matrix row count
         *  @tparam C Diagonal matrix column count
         *  @tparam N Vector size
         *  @tparam T Vector/Matrix element type
         *  @param d Vector with the diagonal elements
         * 
         *  @note The output matrix should, at the minimum, be able to contain the vector diagonals. `N <= R` and `N <= C`.
         * 
         */
        template<uint8_t N>
            static constexpr Matrix<R, C, T> diagonal(const Vector<N, T> &d) {
                static_assert(R >= N, "[MATRIX Error] : Matrix row count is too small to contain the diagonal vector.");
                static_assert(C >= N, "[MATRIX Error] : Matrix column count is too small to contain the diagonal vector.");

                Matrix<R, C, T> output;

                uint8_t minLength = (R < C) ?R :C;

                for(uint8_t i = 0; i < minLength; i++) {
                    output(i,i) = (i < N) ?d[i] :static_cast<T>(0);
                }

                return output;
            }
        
        // ---------------- Getters ----------------
        /**
         *  @brief Return the row number of the vector.
         */
        constexpr uint8_t rows() const { return R; }

        /**
         *  @brief Return the column number of the vector.
         */
        constexpr uint8_t cols() const { return C; }

        // ---------------- Element Accessors ----------------
            /**
             *  @brief Access element at the given row/column.
             *  @param r Row of the accessed element.
             *  @param c Column of the accessed element.
             *  @return Reference to element.
             */
            constexpr T &operator()(uint8_t r, uint8_t c) { if(r >= R) { r = R-1; } if(c >= C) { c = C-1; } return data_[r*C + c]; }

            /**
             *  @brief Const access to element at the given row/column.
             *  @param r Row of the accessed element.
             *  @param c Column of the accessed element.
             *  @return Const reference to element.
             */
            const T &operator()(uint8_t r, uint8_t c) const { if(r >= R) { r = R-1; } if(c >= C) { c = C-1; } return data_[r*C + c]; }
        
        // ---------------- Arithmetic Overloads ----------------
        /**
         *  @brief Add another matrix to this matrix.
         */
        constexpr Matrix &operator+=(const Matrix &rhs) {
            for(uint8_t i = 0; i < R; i++) { 
                for(uint8_t j = 0; j < C; j++) {
                    data_[i*C + j] += rhs.data_[i*C + j]; 
                }
            }
            return *this;
        }

        /**
         *  @brief Subtract another matrix from this matrix.
         */
        constexpr Matrix &operator-=(const Matrix &rhs) {
            for(uint8_t i = 0; i < R; i++) { 
                for(uint8_t j = 0; j < C; j++) {
                    data_[i*C + j] -= rhs.data_[i*C + j]; 
                }
            }
            return *this;
        }

        /**
         *  @brief Right-multiply another matrix(CxL) to this matrix(RxC).
         *  @return (RxL) right-multiplied matrix
         */
        template<uint8_t L>
            constexpr Matrix<R, L> &operator*=(const Matrix<C, L, T> &rhs) {
                Matrix<R, L, T> output{};

                for(uint8_t i = 0; i < R; i++) {
                    for(uint8_t j = 0; j < L; j++) {
                        output(i, j) = static_cast<T>(0);

                        for(uint8_t k = 0; k < C; k++) {
                            output(i, j) += data_[i*C + k] * rhs(k, j);
                        }
                    }
                }

                *this = output;

                return *this;
            }

        /**
         *  @brief Scalar multiplication of this matrix
         */
        constexpr Matrix &operator*=(float c) {
            for(uint8_t i = 0; i < R; i++) {
                for(uint8_t j = 0; j < C; j++) {
                    data_[i*C + j] *= c;
                }
            }

            return *this;
        }

        /**
         *  @brief Scalar divison of this matrix
         */
        constexpr Matrix &operator/=(float c) {
            for(uint8_t i = 0; i < R; i++) {
                for(uint8_t j = 0; j < C; j++) {
                    data_[i*C + j] /= c;
                }
            }

            return *this;
        }

        /**
         *  @brief Element wise negative to this matrix
         */
        constexpr Matrix &operator-() {
            for(uint8_t i = 0; i < R; i++) {
                for(uint8_t j = 0; j < C; j++) {
                    data_[i*C + j] *= -1.0f;
                }
            }

            return *this;
        }

        // ------------ Member Functions  ------------
        template<uint8_t N, uint8_t M>
            constexpr inline Matrix<N, M, T> block(uint8_t r0 = 0, uint8_t c0 = 0) const;

        // ---------------- Utility  ----------------
        
        std::string toString(uint8_t percision = MATRIX_DEFAULT_PRECISION) const;
};

} // cobalt::math::linear_algebra