#pragma once

#include <math.h>
#include <sstream>
#include <iomanip>

#include "matrix.hpp"

#include "../vector/vector_ops.hpp"
#include "../vector/vector_util.hpp"

namespace cobalt::math::linear_algebra {

// ---------------- Non-member Utility ----------------
/**
 *  @brief Extract eigenvalue and eigenvectors of a symmetric matrix
 * 
 * 
 *  @param A Matrix to extract eigenvalue and eigenvectors of.
 *  @param e Vector with eigenvalue elements in decreasing order.
 *  @param V Eigen vector V matrix(NxN) output.
 *  @param maxIterations (optional) The maximum number of iterations to compute for.
 *  @return `iterations` The number of iterations it ran to converge.
 * 
 *  @note `A` has to be symmetric otherwise the results are not correct.
 */
template<uint8_t N, typename T>
    size_t jacobi(Matrix<N, N, T> &A, Vector<N, T> &e, Matrix<N, N, T> &V,  size_t maxIterations = MATRIX_DEFAULT_SVD_ITERATIONS) {
        int iteration = 0;
        V = Matrix<N, N>::eye();

        for(uint8_t i = 0; i < maxIterations; i++) {
            iteration++;
            bool converged = true;

            for(uint8_t p = 0; p < N; p++) {
                for(uint8_t q = p+1; q < N; q++) {
                    T A_pp = A(p, p);
                    T A_pq = A(p, q);
                    T A_qq = A(q, q);

                    if(fabsf(A_pq) > MATRIX_EQUAL_THRESHOLD) {
                        converged = false;

                        T phi = static_cast<T>( 0.5f * atan2f(static_cast<T>(2)*A_pq, A_qq - A_pp));
                        T c = static_cast<T>(cosf(phi));
                        T s = static_cast<T>(sinf(phi));

                        for(uint8_t k = 0; k < N; k++) {
                            T V_kp = V(k, p);
                            T V_kq = V(k, q);

                            V(k, p) = c*V_kp - s*V_kq;
                            V(k, q) = s*V_kp + c*V_kq;
                        }

                        for(uint8_t k = 0; k < N; k++) {
                            if(k != p && k != q) {
                                T A_kp = A(k, p);
                                T A_kq = A(k, q);

                                A(k, p) = c*A_kp - s*A_kq;
                                A(p, k) = A(k, p);
                                A(k, q) = s*A_kp + c*A_kq;
                                A(q, k) = A(k, q);
                            }
                        }

                        A(p, p) = c*c*A_pp - 2*s*c*A_pq + s*s*A_qq;
                        A(q, q) = s*s*A_pp + 2*s*c*A_pq + c*c*A_qq;
                        A(p, q) = A(q, p) = static_cast<T>(0);
                    }
                }
            }

            if(converged) { break; }
        }

        // Eigenvalue extraction
        for(uint8_t i = 0; i < N; i++) {
            e[i] = A(i, i);
        }

        // Eigenvalue/vector ordering      high --> low
        for(uint8_t i = 0; i < N; i++) {
            uint8_t index = i;

            for(uint8_t j = i+1; j < N; j++) {
                if(e[j] > e[index]) { index = j; }
            }

        
            if(index != i) {
                std::swap(e[i], e[index]);

                for(uint8_t k = 0; k < N; k++) { std::swap(V(k, i), V(k, index)); }
            }
        }

        return iteration;
    }

/**
 *  @brief Compute the Single Value Decomposion of a matrix.
 * 
 *  Decomposes A into U, Σ & V  matricies such that A = U*Σ*Vᵀ. 
 * 
 *  @param A Matrix to single value decompose.
 *  @param U Orthogonal U matrix(NxN) output
 *  @param S Diagonal eigenvalue Σ matrix(NxM) output
 *  @param V Eigen vector V matrix(MxM) output.
 *  @param maxIterations (optional) The maximum number of iterations to compute for
 * 
 *  @return `iterations` The number of iterations it ran to converge.
 *  @note SVD always converges so it cannot fail.
 */
template<uint8_t N, uint8_t M, typename T = float>
    size_t svd(const Matrix<N, M, T> &A, Matrix<N, N, T> &U, Matrix<N, M, T> &S, Matrix<M, M, T> &V, size_t maxIterations = MATRIX_DEFAULT_SVD_ITERATIONS) {
        static_assert(N >= M, "[MATRIX Error] : SVD only exists for matricies(NxM) with N >= M.");

        Matrix<M, M, T> AtA = transpose(A)*A;

        for(uint8_t i = 0; i < N; i++) {
            for(uint8_t j = 0; j < N; j++) {
                U(i, j) = (j < M) ?A(i, j) :static_cast<T>(0);
            }
        }
        Vector<M> eigen{};
        S = Matrix<N, M, T>::zero();
        V = Matrix<M, M, T>::eye();

        size_t iterations = jacobi(AtA, eigen, V, maxIterations);


        // Compute S, singular values
        Vector<M, T> sig{};
        for(uint8_t i = 0; i < M; i++) {
            sig[i] = static_cast<T>(sqrtf(fmaxf(eigen[i], static_cast<T>(0))));
        }
        S = Matrix<N, M, T>::diagonal(sig);


        // Compute U, A*V*S_inv
        Matrix<N, M, T> AV = A * V;

        for(uint8_t j = 0; j < M; j++) {
            if(static_cast<float>(sig[j]) > MATRIX_ZERO_THRESHOLD) {
                for(uint8_t i = 0; i < N; i++) {
                    U(i, j) = AV(i, j) / sig[j];
                }
            }
        }

        return iterations;    
    }

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
template<uint8_t N, typename T = float>
    [[nodiscard]] bool decompLU(const Matrix<N, N, T> &A, Matrix<N, N, T> &L, Matrix<N, N, T> &U, Vector<N, T> &P) {
        L = Matrix<N, N, T>::eye();
        U = A;
        for(uint8_t i = 0; i < N; i++) P[i] = i;
        
        for(uint8_t k = 0; k < N; k++) {
            // Get pivot
            T maxVal = static_cast<T>(fabsf(U(k, k)));
            uint8_t pivot = k;

            for(uint8_t i = k+1; i < N; i++) {
                T val = static_cast<T>(fabsf(U(i, k)));
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

/**
 *  @brief Compute the QR-decomposion of a matrix.
 * 
 *  Decomposes `A` into orthonormal `Q` & upper triangular `R` matricies such that A = QR. 
 * 
 *  @param A Matrix to QR-decompose.
 *  @param Q Orthonormal matrix Q (NxN) decomposion output.
 *  @param R Upper triangular matrix R (NxN) decomposion output.
 *  @return `true` if A's columns were independent, `false` otherwise. Returning false indicates `R` will be singular.
 */
template<uint8_t N, uint8_t M, typename T = float>
    bool decompQR(const Matrix<N, M, T> &A, Matrix<N, N, T> &Q, Matrix<N, M, T> &R) {
        
        bool isIndependent = gramSchmidt(A, Q);

        if(!isIndependent) { return false; }

        R =  transpose(Q) * A;

        return true;
    }

/**
 *  @brief Extract a orthonormal set of vectors from a column vector matrix
 * 
 *  Creates a orthonormal set of vectors from the column vectors of `A` as the column vectors of the output matrix `Q`
 * 
 *  @param A Matrix with column vectors to orthonormalize.
 *  @param Q Output matrix with orthonormal column vectors.
 *  @return `true` if input vectors were linearly independent, `false` otherwise. Returning false indicates `Q` has zero column(s)
 */
template<uint8_t N, uint8_t M, typename T = float>
    bool gramSchmidt(const Matrix<N, M, T> &A, Matrix<N, M, T> &Q) {
        Q = Matrix<N, M, T>::zero();
        bool isIndependent = true;

        for(uint8_t j = 0; j < M; j++) {
            Vector<N, T> vec = toVector(A, j);

            for(uint8_t i = 0; i < j; i++) {
                vec = rejectFrom(vec, toVector(Q, i));
            }

            vec = normalize(vec);

            if(norm(vec) < MATRIX_ZERO_THRESHOLD) { return isIndependent = false; } // Zero colummn

            for(uint8_t i = 0; i < N; i++) { Q(i, j) = vec[i]; }
        }

        return isIndependent;
    }

// ---------------- Conversions ----------------
/**
 *  @brief Convert a matrix into a vector
 * 
 *  Creates a vector out of a matrix(Nx1) or the colummn of a matrix(NxM)
 * 
 *  @param A Matrix to convert to a vector
 *  @param d (optional) Matrix column to convert. Defaults to 0.
 *  @return Vector of size `R`
 * 
 */
template<uint8_t N, uint8_t M, typename T = float>
    constexpr inline Vector<N, T> toVector(const Matrix<N, M, T> &A, uint8_t column = 0) {
        Vector<N, T> output;

        for(uint8_t i = 0; i < N; i++) {
            output[i] = A(i, column);
        }

        return output;
    }

// ---------------- Checks ----------------
/**
 *  @brief Check if a matrix is singular (det = 0)
 */
template<uint8_t N, typename T = float>
constexpr inline bool isSingular(const Matrix<N, N, T> &A) {
    return (fabsf(det(A)) < VECTOR_EQUAL_THRESHOLD);
}

} // cobalt::math::linear_algebra