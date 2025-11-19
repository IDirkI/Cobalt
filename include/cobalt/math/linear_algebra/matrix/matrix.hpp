#pragma once

#include <stdint.h>
#include <array>

#include "../vector/vector.hpp"

namespace cobalt::math::linear_algebra {

constexpr uint8_t MATRIX_MAX_ROW_SIZE = 12;
constexpr uint8_t MATRIX_MAX_COL_SIZE = 12;

constexpr float   MATRIX_EQUAL_THRESHOLD = 1e-6;

constexpr float MATRIX_PSEUDO_K= 0.5;

constexpr uint8_t MATRIX_DEFAULT_EXP_TERMS = 20;
constexpr uint8_t MATRIX_DEFAULT_SVD_ITERATIONS = 100;

// --------------------------------------
//          NxM - Matrix    
// --------------------------------------

/**
 *  @brief Fixed-size matrix.
 *  @tparam N Row count of the matrix.
 *  @tparam M Column count of the matrix.
 *  @tparam T Element type (default float).
 */
template<uint8_t N, uint8_t M, typename T = float>
struct Matrix {
    static_assert(N <= MATRIX_MAX_ROW_SIZE  , "[MATRIX Error] : Matrix rows exceeds maximum size.");
    static_assert(M <= MATRIX_MAX_COL_SIZE  , "[MATRIX Error] : Matrix columns exceeds maximum size.");

    private:
        std::array<T, N*M> data_{};

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
                    if(i < N && j < M) { data_[i*M + j] = val; }
                    j++;
                }

                for(; j < M; j++) { data_[i*M + j] = static_cast<T>(0); }

                i++;
                if(i >= N) break;
            }

            for(; i < N; i++) { 
                for(; j < M; j++) { data_[i*M + j] = static_cast<T>(0); }
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
            Matrix<N, M, T> out{};
            uint8_t d = (N <= M) ?N :M;

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
         *  @tparam N Diagonal matrix row count
         *  @tparam M Diagonal matrix column count
         *  @tparam K Vector size
         *  @tparam T Vector/Matrix element type
         *  @param  d Vector with the diagonal elements
         * 
         *  @note The output matrix should, at the minimum, be able to contain the vector diagonals. `K <= N` and `K <= M`.
         * 
         */
        template<uint8_t K>
            static constexpr Matrix<N, M, T> diagonal(const Vector<K, T> &d) {
                static_assert(N >= K, "[MATRIX Error] : Matrix row count is too small to contain the diagonal vector.");
                static_assert(M >= K, "[MATRIX Error] : Matrix column count is too small to contain the diagonal vector.");

                Matrix<N, M, T> output;

                uint8_t minLength = (N < M) ?N :M;

                for(uint8_t i = 0; i < minLength; i++) {
                    output(i,i) = (i < K) ?d[i] :static_cast<T>(0);
                }

                return output;
            }
        
        // ---------------- Getters ----------------
        /**
         *  @brief Return the row number of the vector.
         */
        constexpr uint8_t rows() const { return N; }

        /**
         *  @brief Return the column number of the vector.
         */
        constexpr uint8_t cols() const { return M; }

        // ---------------- Element Accessors ----------------
            /**
             *  @brief Access element at the given row/column.
             *  @param r Row of the accessed element.
             *  @param c Column of the accessed element.
             *  @warning No bounds checking. Use at() for safe access
             *  @note In debug mode, asserts if `r` >= `N` or `c` >= `M`
             *  @return Reference to element.
             */
            constexpr T &operator()(uint8_t r, uint8_t c) { 
                assert(r < N && "[MATRIX Error] : Accessed element must be within matrix row size.");
                assert(c < M && "[MATRIX Error] : Accessed element must be within matrix column size.");
                return data_[r*M + c]; 
            }

            /**
             *  @brief Const access to element at the given row/column.
             *  @param r Row of the accessed element.
             *  @param c Column of the accessed element.
             *  @warning No bounds checking. Use at() for safe access
             *  @note In debug mode, asserts if `r` >= `R` or `c` >= `C`
             *  @return Const reference to element.
             */
            const T &operator()(uint8_t r, uint8_t c) const {
                assert(r < N && "[MATRIX Error] : Accessed element must be within matrix row size.");
                assert(c < M && "[MATRIX Error] : Accessed element must be within matrix column size.");
                 return data_[r*M + c];
            }
        
            /**
             *  @brief Access element at the given row/column.
             *  @param r Row of the accessed element.
             *  @param c Column of the accessed element.
             *  @note Clamps the output to the last element if the asked index is out of bounds
             *  @return Reference to element.
             */
            constexpr T &at(uint8_t r, uint8_t c) { if(r >= N) { r = N-1; } if(c >= M) { c = M-1; } return data_[r*M + c]; }

            /**
             *  @brief Const access to element at the given row/column.
             *  @param r Row of the accessed element.
             *  @param c Column of the accessed element.
             *  @note Clamps the output to the last element if the asked index is out of bounds
             *  @return Const reference to element.
             */
            const T &at(uint8_t r, uint8_t c) const { if(r >= N) { r = N-1; } if(c >= M) { c = M-1; } return data_[r*M + c]; }
        // ---------------- Arithmetic Overloads ----------------
        /**
         *  @brief Add another matrix to this matrix.
         */
        constexpr Matrix &operator+=(const Matrix &rhs) {
            for(uint8_t i = 0; i < N; i++) { 
                for(uint8_t j = 0; j < M; j++) {
                    data_[i*M + j] += rhs.data_[i*M + j]; 
                }
            }
            return *this;
        }

        /**
         *  @brief Subtract another matrix from this matrix.
         */
        constexpr Matrix &operator-=(const Matrix &rhs) {
            for(uint8_t i = 0; i < N; i++) { 
                for(uint8_t j = 0; j < M; j++) {
                    data_[i*M + j] -= rhs.data_[i*M + j]; 
                }
            }
            return *this;
        }

        /**
         *  @brief Right-multiply another matrix(NxN) to this matrix(NxN).
         *  @return (NxN) right-multiplied matrix
         */
        constexpr Matrix<N, N> &operator*=(const Matrix<N, N, T> &rhs) {
            Matrix<N, N, T> output{};

            for(uint8_t i = 0; i < N; i++) {
                for(uint8_t j = 0; j < N; j++) {
                    output(i, j) = static_cast<T>(0);

                    for(uint8_t k = 0; k < N; k++) {
                        output(i, j) += data_[i*N + k] * rhs(k, j);
                    }
                }
            }

            *this = output;

            return *this;
        }

        /**
         *  @brief Scalar multiplication of this matrix
         */
        constexpr Matrix &operator*=(T c) {
            for(uint8_t i = 0; i < N; i++) {
                for(uint8_t j = 0; j < M; j++) {
                    data_[i*M + j] *= c;
                }
            }

            return *this;
        }

        /**
         *  @brief Scalar divison of this matrix
         */
        constexpr Matrix &operator/=(T c) {
            for(uint8_t i = 0; i < N; i++) {
                for(uint8_t j = 0; j < M; j++) {
                    data_[i*M + j] /= c;
                }
            }

            return *this;
        }

        // ------------ Member Functions  ------------

        /**
         *  @brief Get a RxC matrix block out of a NxM matrix.
         *  
         *  Extracts a RxC matrix from the elements `(r0, c0)` --> `(r0 + R, c0 + C)`.
         * 
         * 
         *  @tparam R Row count of the output matrix.
         *  @tparam C Column count of the output matrix.
         *  @param r0 (optional) Starting row of the block
         *  @param c0 (optional) Starting column of the block
         * 
         *  @note If the original matrix isn't defined inside the output block the missing entried are set to zero.
         */
        template<uint8_t R, uint8_t C>
            constexpr Matrix<R, C, T> block(uint8_t r0 = 0, uint8_t c0 = 0) const  {
                Matrix<R, C, T> output = Matrix<R, C, T>::zero();

                for(uint8_t i = 0; i < R; i++) {
                    for(uint8_t j = 0; j < C; j++) {
                        output(i, j) = data_[(i+r0)*M + (j+c0)];
                    }
                }

                return output;
            }
};

} // cobalt::math::linear_algebra