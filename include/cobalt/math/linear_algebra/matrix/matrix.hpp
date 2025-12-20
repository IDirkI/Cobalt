#pragma once

#include <stdint.h>
#include <array>

#include "../../config.hpp"

#include "../vector/vector.hpp"

namespace cobalt::math::linear_algebra {

constexpr index_t MATRIX_MAX_ROW_SIZE = 12;
constexpr index_t MATRIX_MAX_COL_SIZE = 12;

constexpr float MATRIX_PSEUDO_K= 0.5;

constexpr index_t MATRIX_DEFAULT_EXP_TERMS = 20;
constexpr index_t MATRIX_DEFAULT_LOG_TERMS = 20;
constexpr index_t MATRIX_DEFAULT_POW_TERMS = 20;
constexpr index_t MATRIX_DEFAULT_SVD_ITERATIONS = 100;

// --------------------------------------
//          NxM - Matrix    
// --------------------------------------
/**
 *  @brief Fixed-size matrix
 *  @tparam N Row count of the matrix
 *  @tparam M Column count of the matrix
 *  @tparam T Element type (default float)
 */
template<index_t N, index_t M, typename T = float, typename = std::enable_if_t<Scalar<T>>>
struct Matrix {
    static_assert(N <= MATRIX_MAX_ROW_SIZE  , "[MATRIX Error] : Matrix rows exceeds maximum size.");
    static_assert(M <= MATRIX_MAX_COL_SIZE  , "[MATRIX Error] : Matrix columns exceeds maximum size.");

    private:
        std::array<T, N*M> data_{};

    public:
        // ---------------- Constructors ----------------
        /**
         *  @brief Default constructor (zero-initializes elements)
         */
        constexpr Matrix() noexcept : data_{} {}

        /**
         *  @brief Construct a matrix from an initializer list of initializer lists
         *  @param list2d 2D Initializer list of elements
         */ 
        constexpr Matrix(std::initializer_list<std::initializer_list<T>> list2d) noexcept {
            index_t i = 0, j = 0;

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
         *  @brief Create a zero-matrix
         */
        static constexpr Matrix zero() noexcept { return Matrix(); }

        /**
         *  @brief Create an identity matrix
         */
        static constexpr Matrix eye() noexcept { 
            Matrix<N, M, T> out{};
            index_t d = (N <= M) ?N :M;

            for(index_t i = 0; i < d; i++) {
                out(i, i) = static_cast<T>(1);
            }

            return out;
        }

        /**
         *  @brief Create a diagonal matrix from a vector
         *  @param d Vector containing the diagonal elements
         *  @return Diagonal matrix with elements from vector d
         *  @note If the vector size is less than min(N, M), remaining diagonal elements are set to zero
         *  @note If the vector size is greater than min(N, M), excess elements are ignored
         *  @tparam K Size of the input vector
         *  @warning Asserts if K > min(N, M)
         */
        template<index_t K>
            static constexpr Matrix<N, M, T> diagonal(const Vector<K, T> &d) {
                static_assert(N >= K, "[MATRIX Error] : Matrix row count is too small to contain the diagonal vector.");
                static_assert(M >= K, "[MATRIX Error] : Matrix column count is too small to contain the diagonal vector.");

                Matrix<N, M, T> output;

                index_t minLength = (N < M) ?N :M;

                for(index_t i = 0; i < minLength; i++) {
                    output(i,i) = (i < K) ?d[i] :static_cast<T>(0);
                }

                return output;
            }
        
        // ---------------- Getters ----------------
        /**
         *  @brief Return the row number of the vector
         *  @return Number of rows N
         */
        static constexpr index_t rows() noexcept { return N; }

        /**
         *  @brief Return the column number of the vector
         *  @return Number of columns M
         */
        static constexpr index_t cols() noexcept { return M; }

        // ---------------- Element Accessors ----------------
        /**
         *  @brief Access element at the given row/column
         *  @param r Row of the accessed element
         *  @param c Column of the accessed element
         *  @warning No bounds checking. Use at() for safe access
         *  @note Asserts if r >= N or c >= M
         *  @return Reference to element
         */
        constexpr T &operator()(index_t r, index_t c) noexcept { 
            assert(r < N && "[MATRIX Error] : Accessed element must be within matrix row size.");
            assert(c < M && "[MATRIX Error] : Accessed element must be within matrix column size.");
            return data_[r*M + c]; 
        }

        /**
         *  @brief Const access to element at the given row/column
         *  @param r Row of the accessed element
         *  @param c Column of the accessed element
         *  @warning No bounds checking. Use at() for safe access
         *  @note Asserts if r >= N or c >= M
         *  @return Const reference to element
         */
        const T &operator()(index_t r, index_t c) const noexcept {
            assert(r < N && "[MATRIX Error] : Accessed element must be within matrix row size.");
            assert(c < M && "[MATRIX Error] : Accessed element must be within matrix column size.");
                return data_[r*M + c];
        }
    
        /**
         *  @brief Access element at the given row/column
         *  @param r Row of the accessed element
         *  @param c Column of the accessed element
         *  @note Clamps the output to the last element if the asked index is out of bounds
         *  @return Reference to element
         */
        constexpr T &at(index_t r, index_t c) noexcept { if(r >= N) { r = N-1; } if(c >= M) { c = M-1; } return data_[r*M + c]; }

        /**
         *  @brief Const access to element at the given row/column
         *  @param r Row of the accessed element
         *  @param c Column of the accessed element
         *  @note Clamps the output to the last element if the asked index is out of bounds
         *  @return Const reference to element
         */
        const T &at(index_t r, index_t c) const noexcept { if(r >= N) { r = N-1; } if(c >= M) { c = M-1; } return data_[r*M + c]; }

        /**
         *  @brief Access to raw data of the matrix
         *  @return Raw data array
         */
        constexpr T* data() noexcept { return data_.data(); }

        /**
         *  @brief Const access to raw data of the matrix
         *  @return Const raw data array
         */
        constexpr const T* data() const noexcept { return data_.data(); }

        /**
         *  @brief Access to the start of the data in memory
         *  @return Iterator to the start of the raw data array
         */
        constexpr auto begin() noexcept { return data_.begin(); }

        /**
         *  @brief Access to the end of the data in memory
         *  @return Iterator to the end of the raw data array
         */
        constexpr auto end() noexcept { return data_.end(); }

        // ---------------- Arithmetic Overloads ----------------
        /**
         *  @brief Add another matrix to this matrix
         */
        constexpr Matrix &operator+=(const Matrix &rhs) noexcept {
            for(index_t i = 0; i < N; i++) { 
                for(index_t j = 0; j < M; j++) {
                    data_[i*M + j] += rhs.data_[i*M + j]; 
                }
            }
            return *this;
        }

        /**
         *  @brief Subtract another matrix from this matrix
         */
        constexpr Matrix &operator-=(const Matrix &rhs) noexcept {
            for(index_t i = 0; i < N; i++) { 
                for(index_t j = 0; j < M; j++) {
                    data_[i*M + j] -= rhs.data_[i*M + j]; 
                }
            }
            return *this;
        }

        /**
         *  @brief Right-multiply another matrix(NxN) to this matrix(NxN)
         *  @return (NxN) right-multiplied matrix
         */
        constexpr Matrix<N, N> &operator*=(const Matrix<N, N, T> &rhs) noexcept {
            Matrix<N, N, T> output{};

            for(index_t i = 0; i < N; i++) {
                for(index_t j = 0; j < N; j++) {
                    output(i, j) = static_cast<T>(0);

                    for(index_t k = 0; k < N; k++) {
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
        constexpr Matrix &operator*=(T c) noexcept {
            for(index_t i = 0; i < N; i++) {
                for(index_t j = 0; j < M; j++) {
                    data_[i*M + j] *= c;
                }
            }

            return *this;
        }

        /**
         *  @brief Scalar divison of this matrix
         */
        constexpr Matrix &operator/=(T c) noexcept {
            for(index_t i = 0; i < N; i++) {
                for(index_t j = 0; j < M; j++) {
                    data_[i*M + j] /= c;
                }
            }

            return *this;
        }

        // ------------ Member Functions  ------------

        /**
         *  @brief Extract a sub-block of the matrix
         *  @tparam R Row count of the sub-block
         *  @tparam C Column count of the sub-block
         *  @param r0 Starting row index of the sub-block
         *  @param c0 Starting column index of the sub-block
         *  @return Sub-block matrix of size RxC
         *  @note If the specified block exceeds the matrix dimensions, remaining elements are filled with zeros
         */
        template<index_t R, index_t C>
            constexpr Matrix<R, C, T> block(index_t r0 = 0, index_t c0 = 0) const noexcept {
                Matrix<R, C, T> output = Matrix<R, C, T>::zero();

                for(index_t i = 0; i < R; i++) {
                    for(index_t j = 0; j < C; j++) {
                        output(i, j) = ((i+r0 < N) && (j+c0 < M)) ?data_[(i+r0)*M + (j+c0)] :static_cast<T>(0.0f);
                    }
                }

                return output;
            }
};

} // cobalt::math::linear_algebra