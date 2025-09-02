#pragma once

#include "matrix.hpp"

#include "../vector/vector_util.hpp"

namespace cobalt::math::linear_algebra {

// ---------------- Non-member Utility ----------------
/**
 *  @brief Compute the LU-decomposion of a matrix.
 * 
 *  Decomposes A into L & U matricies such that P*A = L*U. 
 * 
 *  @param A Matrix to LU-decompose.
 *  @param L Lower triangular matrix (NxN) decomposion output.
 *  @param U Upper triangular matrix (NxN) decomposion output.
 *  @param P Permutation vector output.
 *  @return `true` if decomposision succeeds, `false` if A is signular.
 *  @note Return value should not be ignored and handled properly if A is singular
 */
template<uint8_t N, typename T>
    [[nodiscard]] bool decompLU(const Matrix<N, N, T> &A, Matrix<N, N, T> &L, Matrix<N, N, T> &U, Vector<N, T> &P) {
        L = Matrix<N, N, T>::eye();
        U = A;
        for(uint8_t i = 0; i < N; i++) P[i] = i;
        
        for(uint8_t k = 0; k < N; k++) {
            // Get pivot
            T maxVal = static_cast<T>(std::fabs(U(k, k)));
            uint8_t pivot = k;

            for(uint8_t i = k+1; i < N; i++) {
                T val = static_cast<T>(std::fabs(U(i, k)));
                if(val > maxVal) {
                    maxVal = val;
                    pivot = i;
                }
            }

            if(maxVal < static_cast<T>(MATRIX_EQUAL_THRESHOLD)) { return false; } // Singular matrix

            // Swap rows
            if(pivot != k) {
                for(uint8_t j = 0; j < N; j++) {
                    std::swap(U(k, j), U(pivot, j));
                }

                for(uint8_t j = 0; j < k; j++) {
                    std::swap(L(k, j), L(pivot, j));
                }
                
                std::swap(P[k], P[pivot]);
            }

            // Elimination
            for(uint8_t i = k+1; i<N; i++) {
                T factor = U(i, k) / U(k, k);
                L(i, k) = factor;

                for(uint8_t j = k; j < N; j++) {
                    U(i, j) -= factor * U(k, j);
                }
            }
        }

        return true;    // Non-Singular
    }

} // cobalt::math::linear_algebra