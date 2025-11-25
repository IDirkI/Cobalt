#pragma once

#include <cmath>

#include "matrix.hpp"

#include "../vector/vector_ops.hpp"
#include "../vector/vector_util.hpp"

namespace cobalt::math::linear_algebra {

// ---------------- Non-member Utility ----------------

/**
 *  @brief Compute the hadamard (element-wise) product of two matrices
 */
template<uint8_t N, uint8_t M, typename T = float>
    constexpr Matrix<N, M, T> hadamard(const Matrix<N, M, T> &A, const Matrix<N, M, T> &B) {
        Matrix<N, M, T> output = Matrix<N, M, T>::zero();
        
        for(uint8_t i = 0; i < N; i++) {
            for(uint8_t j = 0; j < M; j++) {
                output(i,j) = A(i,j) * B(i,j);
            }
        }

        return output;
    }


/**
 *  @brief Compute the matrix exponential approximation of a matrix
 *  @param A Matrix to exponentiate
 *  @param terms Number of terms to approximate with
 */
template<uint8_t N, typename T = float>
    constexpr Matrix<N, N, T> exp(const Matrix<N, N, T> &A, uint8_t terms = MATRIX_DEFAULT_EXP_TERMS) {
        Matrix<N, N, T> output = Matrix<N, N, T>::zero();
        float powA = Matrix<N, N, T>::eye();
        float fact = 1;

        for(uint8_t i = 0; i < terms; i++) {
            output += static_cast<T>(1.0f/fact) * powA;
            powA *= A;
            fact *= (i+1);
        }

        return output;
    }

/**
 *  @brief Compute the left moore-penrose psuedo inverse of a matrix
 *  @param A Matrix to pseudo-invert
 *  @param Apinv Inverted output Matrix
 *  @return `true` if inversion succeeds, `false` if A is not-full column rank.
 * 
 *  DLS method is used to handle possible singular value A.
 *  
 *  @note Return value should not be ignored and handled properly if A is not-full column rank
 */
template<uint8_t N, uint8_t M, typename T = float>
    [[nodiscard]] constexpr bool pseudoL(const Matrix<N, M, T> &A, Matrix<M, N, T> &Apinv) {
        static_assert(N >= M, "Left pseudo-inverse onlt works for 'tall' matricies, not 'wide'.");

        Matrix<M,N,T> At = transpose(A);
        Matrix<M, M, T> AtA = At * A;

        Matrix<M, M, T> AtAinv;
        if(!inv(AtA, AtAinv)) { return false; } // Singular

        Apinv = AtAinv * At;

        return true;
    }

/**
 *  @brief Compute the right moore-penrose psuedo inverse of a matrix
 *  @param A Matrix to pseudo-invert
 *  @param Apinv Inverted output Matrix
 *  @return `true` if inversion succeeds, `false` if A is not-full row rank.
 * 
 *  DLS method is used to handle possible singular value A.
 *  
 *  @note Return value should not be ignored and handled properly if A is not-full row rank
 */
template<uint8_t N, uint8_t M, typename T = float>
    [[nodiscard]] constexpr bool pseudoR(const Matrix<N, M, T> &A, Matrix<M, N, T> &Apinv) {
        static_assert(M >= N, "Right pseudo-inverse onlt works for 'wide' matricies, not 'tall'.");

        Matrix<M,N,T> At = transpose(A);
        Matrix<N, N, T> AAt = A * At;

        Matrix<M, M, T> AAtinv;
        if(!inv(AAt, AAtinv)) { return false; } // Singular

        Apinv = At * AAtinv;

        return true;
    }

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

                    if(std::abs(A_pq) > MATRIX_EQUAL_THRESHOLD) {
                        converged = false;

                        T phi = static_cast<T>( 0.5f * std::atan2(static_cast<T>(2)*A_pq, A_qq - A_pp));
                        T c = static_cast<T>(std::cos(phi));
                        T s = static_cast<T>(std::sin(phi));

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
            sig[i] = static_cast<T>(std::sqrt(std::max(eigen[i], static_cast<T>(0))));
        }
        S = Matrix<N, M, T>::diagonal(sig);


        // Compute U, A*V*S_inv
        Matrix<N, M, T> AV = A * V;

        for(uint8_t j = 0; j < M; j++) {
            if(static_cast<float>(sig[j]) > MATRIX_EQUAL_THRESHOLD) {
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
 *  @param P Permutation matrix output.
 *  @return `true` if decomposision succeeds, `false` if A is signular.
 *  @note Return value should not be ignored and handled properly if A is singular
 */
template<uint8_t N, typename T = float>
    [[nodiscard]] bool lu(const Matrix<N, N, T> &A, Matrix<N, N, T> &L, Matrix<N, N, T> &U, Matrix<N, N, T> &P, uint8_t &swapCount) {
        P = Matrix<N, N, T>::eye();
        L = Matrix<N, N, T>::eye();
        U = A;

        swapCount = 0;
        
        for(uint8_t k = 0; k < N; k++) {
            // Get pivot
            T maxVal = static_cast<T>(std::abs(U(k, k)));
            uint8_t pivot = k;

            for(uint8_t i = k+1; i < N; i++) {
                T val = static_cast<T>(std::abs(U(i, k)));
                if(val > maxVal) {
                    maxVal = val;
                    pivot = i;
                }
            }

            if(maxVal < static_cast<T>(MATRIX_EQUAL_THRESHOLD)) { return false; } // Singular matrix

            // Swap rows
            if(pivot != k) {
                swapCount++;

                for(uint8_t j = 0; j < N; j++) {
                    std::swap(U(k, j), U(pivot, j));
                    std::swap(P(k, j), P(pivot, j));
                }

                for(uint8_t j = 0; j < k; j++) {
                    std::swap(L(k, j), L(pivot, j));
                }
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
 *  @brief Compute the QR-decomposition of a matrix.
 * 
 *  Decomposes `A` into orthonormal `Q` (NxN) & upper triangular `R` (NxM) such that A = QR. 
 * 
 *  @param A Matrix to QR-decompose (NxM).
 *  @param Q Orthonormal matrix Q (NxN) decomposition output.
 *  @param R Upper triangular matrix R (NxM) decomposition output.
 *  @return `true` if A's columns were independent, `false` otherwise.
 */
template<uint8_t N, uint8_t M, typename T = float>
    bool qr(const Matrix<N, M, T> &A, Matrix<N, N, T> &Q, Matrix<N, M, T> &R) {
        
        bool isIndependent = gramSchmidt(A, Q);

        R = transpose(Q) * A;

        return isIndependent;
    }

/**
 *  @brief Extract a full orthonormal basis from a column vector matrix
 * 
 *  Creates an orthonormal set of vectors from the column vectors of `A` as the 
 *  first M columns of Q. If M < N, extends to a full orthonormal basis.
 * 
 *  @param A Matrix with column vectors to orthonormalize (NxM).
 *  @param Q Output matrix with orthonormal columns (NxN).
 *  @return `true` if input vectors were linearly independent, `false` otherwise.
 */
template<uint8_t N, uint8_t M, typename T = float>
    bool gramSchmidt(const Matrix<N, M, T> &A, Matrix<N, N, T> &Q) {
        Q = Matrix<N, N, T>::zero();
        bool isIndependent = true;

        for(uint8_t j = 0; j < M; j++) {
            Vector<N, T> vec = getColumn(A, j);

            for(uint8_t i = 0; i < j; i++) {
                Vector<N, T> qi = getColumn(Q, i);
                vec = ortho(vec, qi);
            }

            vec = normalize(vec);

            if(norm(vec) < MATRIX_EQUAL_THRESHOLD) { isIndependent = false; } // Zero colummn

            for(uint8_t i = 0; i < N; i++) { 
                Q(i, j) = vec[i];
            }
        }

        if(M < N) {
            for(uint8_t j = M; j < N; j++) {
                Vector<N, T> vec = Vector<N, T>::zero();
                vec[j] = static_cast<T>(1);

                for(uint8_t i = 0; i < j; i++) {
                    Vector<N, T> qi = getColumn(Q, i);
                    vec = ortho(vec, qi);
                }

                uint8_t attempt = 0;
                while((norm(vec) < static_cast<T>(MATRIX_EQUAL_THRESHOLD)) && (attempt < N)) {
                    vec = Vector<N, T>::zero();
                    vec[(j + attempt) % N] = static_cast<T>(1);
                    
                    for(uint8_t i = 0; i < j; i++) {
                        Vector<N, T> qi = getColumn(Q, i);
                        vec = ortho(vec, qi);
                    }
                    attempt++;
                }

                vec = normalize(vec);

                for(uint8_t i = 0; i < N; i++) { 
                    Q(i, j) = vec[i]; 
                }
            }
        }

        return isIndependent;
    }

/**
 *  @brief Orthonormalize the columns of a matrix (reduced QR)
 * 
 *  Creates orthonormal columns from A's columns (NxM output).
 * 
 *  @param A Matrix with column vectors to orthonormalize (NxM).
 *  @param Q Output matrix with orthonormal columns (NxM).
 *  @return `true` if input vectors were linearly independent.
 */
template<uint8_t N, uint8_t M, typename T = float>
    bool gramSchmidtReduced(const Matrix<N, M, T> &A, Matrix<N, M, T> &Q) {
        Q = Matrix<N, M, T>::zero();
        bool isIndependent = true;

        for(uint8_t j = 0; j < M; j++) {
            Vector<N, T> vec = getColumn(A, j);

            for(uint8_t i = 0; i < j; i++) {
                Vector<N, T> qi = getColumn(Q, i);
                vec = ortho(vec, qi);
            }

            vec = normalize(vec);

            if(norm(vec) < MATRIX_EQUAL_THRESHOLD) { isIndependent = false; } // Zero colummn

            for(uint8_t i = 0; i < N; i++) { 
                Q(i, j) = vec[i];
            }
        }

        return isIndependent;
    }

// ---------------- Conversions ----------------
/**
 *  @brief Convert a matrix into a vector. Row first order
 * 
 *  @param A Matrix to convert to a vector
 *  @return Vector of size `N*M`
 * 
 */
template<uint8_t N, uint8_t M, typename T = float>
    constexpr Vector<N*M, T> vectorize(const Matrix<N, M, T> &A) {
        Vector<N*M, T> output;

        for(uint8_t i = 0; i < N; i++) {
            for(uint8_t j = 0; j < M; j++) {
                output[i*M + j] = A(i, j);
            }   
        }
    }

/**
 *  @brief Convert a vector into a matrix. Row first order
 * 
 *  @param v Vector to convert to a matrix
 *  @return Matrix of size `NxM`
 * 
 */
template<uint8_t N, uint8_t M, typename T = float>
    constexpr Matrix<N, M, T> reshape(const Vector<N*M, T> &v) {
        Matrix<N, M, T> output;

        for(uint8_t i = 0; i < N; i++) {
            for(uint8_t j = 0; j < M; j++) {
                output(i, j) = v[i*M + j];
            }   
        }
    }

/**
 *  @brief Get the column of a matrix as a vector
 */
template<uint8_t N, uint8_t M, typename T = float>
    constexpr Vector<N, T> getColumn(const Matrix<N, M, T> &A, uint8_t column = 0) {
        Vector<N, T> output;

        for(uint8_t i = 0; i < N; i++) {
            output[i] = A(i, column);
        }

        return output;
    }

/**
 *  @brief Get the row of a matrix as a vector
 */
template<uint8_t N, uint8_t M, typename T = float>
    constexpr Vector<M, T> getRow(const Matrix<N, M, T> &A, uint8_t row = 0) {
        Vector<M, T> output;

        for(uint8_t j = 0; j < M; j++) {
            output[j] = A(row, j);
        }

        return output;
    }

/**
 *  @brief Get the main diagonal of a matrix as a vector
 */
template<uint8_t N, uint8_t M, typename T = float>
    constexpr Vector<(N < M) ?N :M, T> getDiagonal(const Matrix<N, M, T> &A) {
        constexpr uint8_t minLength = (N < M) ?N :M;
        Vector<minLength, T> output;

        for(uint8_t i = 0; i < minLength; i++) {
            output[i] = A(i, i);
        }

        return output;
    }

// ---------------- Checks ----------------
/**
 *  @brief Check if a matrix is the zero matrix
 */
template<uint8_t N, uint8_t M, typename T = float>
constexpr bool isZero(const Matrix<N, M, T> &A) {
    for(uint8_t i = 0; i < N; i++) {
        for(uint8_t j = 0; j < M; j++) {
            if(std::abs(A(i,j)) > MATRIX_EQUAL_THRESHOLD) { return false; }
        }
    }

    return true;
}

/**
 *  @brief Check if a matrix is the identity matrix
 */
template<uint8_t N, typename T = float>
constexpr bool isIdentity(const Matrix<N, N, T> &A) {
    return isZero(A - Matrix<N, N, T>::eye());
}

/**
 *  @brief Check if a matrix is symmetric
 */
template<uint8_t N, typename T = float>
constexpr bool isSymmetric(const Matrix<N, N, T> &A) {
    return isZero(A - transpose(A));
}

/**
 *  @brief Check if a matrix is orthogonal
 */
template<uint8_t N, typename T = float>
constexpr bool isOrthogonal(const Matrix<N, N, T> &A) {
    return isZero(A*transpose(A));
}

/**
 *  @brief Check if a matrix is diagonal
 */
template<uint8_t N, typename T = float>
constexpr bool isDiagonal(const Matrix<N, N, T> &A) {
    return isZero(A - Matrix<N, N, T>::diagonal(getDiagonal(A)));
}

/**
 *  @brief Check if a matrix is singular
 */
template<uint8_t N, typename T = float>
constexpr bool isSingular(const Matrix<N, N, T> &A) {
    return (std::abs(det(A)) < MATRIX_EQUAL_THRESHOLD);
}

} // cobalt::math::linear_algebra