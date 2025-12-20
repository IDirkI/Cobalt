#pragma once

#include <cmath>

#include "matrix.hpp"

#include "../vector/vector_ops.hpp"
#include "../vector/vector_util.hpp"

namespace cobalt::math::linear_algebra {

// ---------------- Clamping ----------------
/**
 *  @brief Clamp the elements of a maxtrix between an interval
 *  @param A Matrix to clamp.
 *  @param min Lower clamp bound.
 *  @param max Upper clamp bound.
 *  @return Element wise clamped matrix A between [min, max]
 */
template<index_t N, index_t M, typename T = float, typename = std::enable_if_t<Scalar<T>>>
    constexpr Matrix<N, M, T> clamp(const Matrix<N, M, T> &A, T minVal, T maxVal) {
        Matrix<N, M, T> output;
        for(index_t i = 0; i < N; i++) {
            for(index_t j = 0; j < M; j++) {
                output(i, j) = (A(i, j) > maxVal) ?maxVal :((A(i, j) < minVal) ?minVal :A(i, j));
            }
        }

        return output;
    }

/**
 *  @brief Clamp the elements of a maxtrix between the absolute value of max
 *  @param A Matrix to clamp.
 *  @param max Upper clamp bound.
 *  @return Element wise clamped matrix A between [-max, max]
 */
template<index_t N, index_t M, typename T = float, typename = std::enable_if_t<Scalar<T>>>
    constexpr Matrix<N, M, T> clamp(const Matrix<N, M, T> &A, T maxVal) {
        return clamp(A, -maxVal, maxVal);
    }

// ---------------- Element Wise ----------------
/**
 *  @brief Compute smallest element of a matrix
 *  @param A Matrix to check.
 *  @return Smallest matrix element
 */
template<index_t N, index_t M, typename T = float, typename = std::enable_if_t<Scalar<T>>>
    constexpr T min(const Matrix<N, M, T> &A) {
        T output = A(0,0);

        for(index_t i = 0; i < N; i++) {
            for(index_t j = 0; j < M; j++) {
                if(A(i, j) < output ) { output = A(i, j); }
            }
        }

        return output;
    }

/**
 *  @brief Compute largest element of a matrix
 *  @param A Matrix to check.
 *  @return Largest matrix element
 */
template<index_t N, index_t M, typename T = float, typename = std::enable_if_t<Scalar<T>>>
    constexpr T max(const Matrix<N, M, T> &A) {
        T output = A(0,0);

        for(index_t i = 0; i < N; i++) {
            for(index_t j = 0; j < M; j++) {
                if(A(i, j) > output ) { output = A(i, j); }
            }
        }

        return output;
    }

/**
 *  @brief Compute matrix's element wise absolute value
 *  @param A Matrix to abs.
 *  @return Matrix with absolute value of each element of A
 */
template<index_t N, index_t M, typename T = float, typename = std::enable_if_t<Scalar<T>>>
    constexpr Matrix<N, M, T> abs(const Matrix<N, M, T> &A) {
        Matrix<N, M, T> output;
        for(index_t i = 0; i < N; i++) {
            for(index_t j = 0; j < M; j++) {
                output(i, j) = std::abs(A(i, j));
            }
        }

        return output;
    }

/**
 *  @brief Compute matrix's element wise sign
 *  @param A Matrix to sign.
 *  @return Matrix with sign of each element of A
 */
template<index_t N, index_t M, typename T = float, typename = std::enable_if_t<Scalar<T>>>
    constexpr Matrix<N, M, T> sign(const Matrix<N, M, T> &A) {
        Matrix<N, M, T> output;
        for(index_t i = 0; i < N; i++) {
            for(index_t j = 0; j < M; j++) {
                output(i, j) = (std::abs(A(i,j)) < epsilon_<T>) ?static_cast<T>(0.0f) :((A(i,j) > 0) ?static_cast<T>(1.0f) :static_cast<T>(-1.0f));
            }
        }

        return output;
    }

/**
 *  @brief Compute matrix's element wise floored value
 *  @param A Matrix to floor.
 *  @return Matrix with floored value of each element of A
 */
template<index_t N, index_t M, typename T = float, typename = std::enable_if_t<Scalar<T>>>
    constexpr Matrix<N, M, T> floor(const Matrix<N, M, T> &A) {
        Matrix<N, M, T> output;
        for(index_t i = 0; i < N; i++) {
            for(index_t j = 0; j < M; j++) {
                output(i, j) = std::floor(A(i, j));
            }
        }

        return output;
    }

/**
 *  @brief Compute matrix's element wise ceiled value
 *  @param A Matrix to ceil.
 *  @return Matrix with ceiled value of each element of A
 */
template<index_t N, index_t M, typename T = float, typename = std::enable_if_t<Scalar<T>>>
    constexpr Matrix<N, M, T> ceil(const Matrix<N, M, T> &A) {
        Matrix<N, M, T> output;
        for(index_t i = 0; i < N; i++) {
            for(index_t j = 0; j < M; j++) {
                output(i, j) = std::ceil(A(i, j));
            }
        }

        return output;
    }

/**
 *  @brief Compute matrix's element wise rounded value
 *  @param A Matrix to round.
 *  @return Matrix with rounded value of each element of A
 */
template<index_t N, index_t M, typename T = float, typename = std::enable_if_t<Scalar<T>>>
    constexpr Matrix<N, M, T> round(const Matrix<N, M, T> &A) {
        Matrix<N, M, T> output;
        for(index_t i = 0; i < N; i++) {
            for(index_t j = 0; j < M; j++) {
                output(i, j) = std::round(A(i, j));
            }
        }

        return output;
    }

/**
 *  @brief Sets the components of a matrix to a clean zero if they are very close to zero
 *  @param A Matrix to clean
 */
template<index_t N, index_t M, typename T = float, typename = std::enable_if_t<Scalar<T>>>
    constexpr Matrix<N, M, T> cleanZero(const Matrix<N, M, T> &A) {
        Matrix<N, M, T> output = Matrix<N, M, T>::zero();
        
        for(index_t i = 0; i < N; i++) {
            for(index_t j = 0; j < M; j++) {
                output(i, j) = (std::abs(A(i, j)) < epsilon_<T>) ?static_cast<T>(0.0f) :A(i, j);
            }
        }

        return output;
    }

// ---------------- Pseudo-Inverse ----------------
/**
 *  @brief Compute the left moore-penrose psuedo inverse of a matrix
 *  @param A Matrix to pseudo-invert
 *  @param Apinv Inverted output Matrix
 *  @return `true` if inversion succeeds, `false` otherwise
 *  @warning If function returns `false`, Apinv is not modified and is not a valid pseudo-inverse. Return value should be handled properly
 */
template<index_t N, index_t M, typename T = float, typename = std::enable_if_t<Scalar<T>>>
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
 *  @return `true` if inversion succeeds, `false` otherwise
 *  @warning If function returns `false`, Apinv is not modified and is not a valid pseudo-inverse. Return value should be handled properly
 */
template<index_t N, index_t M, typename T = float, typename = std::enable_if_t<Scalar<T>>>
    [[nodiscard]] constexpr bool pseudoR(const Matrix<N, M, T> &A, Matrix<M, N, T> &Apinv) {
        static_assert(M >= N, "Right pseudo-inverse onlt works for 'wide' matricies, not 'tall'.");

        Matrix<M,N,T> At = transpose(A);
        Matrix<N, N, T> AAt = A * At;

        Matrix<N, N, T> AAtinv;
        if(!inv(AAt, AAtinv)) { return false; } // Singular

        Apinv = At * AAtinv;

        return true;
    }

// ---------------- Eigen Val & Vector ----------------
/**
 *  @brief Compute eigenvalues and eigenvectors of a symmetric matrix using the Jacobi method
 *  @param A Symmetric matrix to compute eigenvalues/vectors of (modified in-place)
 *  @param e Eigenvalue vector output
 *  @param V Eigenvector matrix output
 *  @param maxIterations (optional) The maximum number of iterations to compute for
 *  @return `iterations` The number of iterations it ran to converge
 *  @note A must be symmetric
 *  @note Eigenvalues are stored in descending order in e, with corresponding eigenvectors in V
 *  @warning Computationally expensive for large matrices
 */
template<index_t N, typename T>
    size_t jacobi(Matrix<N, N, T> &A, Vector<N, T> &e, Matrix<N, N, T> &V,  size_t maxIterations = MATRIX_DEFAULT_SVD_ITERATIONS) {
        int iteration = 0;
        V = Matrix<N, N>::eye();

        for(index_t i = 0; i < maxIterations; i++) {
            iteration++;
            bool converged = true;

            for(index_t p = 0; p < N; p++) {
                for(index_t q = p+1; q < N; q++) {
                    T A_pp = A(p, p);
                    T A_pq = A(p, q);
                    T A_qq = A(q, q);

                    if(std::abs(A_pq) > epsilon_<T>) {
                        converged = false;

                        T phi = static_cast<T>( 0.5f * std::atan2(static_cast<T>(2)*A_pq, A_qq - A_pp));
                        T c = static_cast<T>(std::cos(phi));
                        T s = static_cast<T>(std::sin(phi));

                        for(index_t k = 0; k < N; k++) {
                            T V_kp = V(k, p);
                            T V_kq = V(k, q);

                            V(k, p) = c*V_kp - s*V_kq;
                            V(k, q) = s*V_kp + c*V_kq;
                        }

                        for(index_t k = 0; k < N; k++) {
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
        for(index_t i = 0; i < N; i++) {
            e[i] = A(i, i);
        }

        // Eigenvalue/vector ordering      high --> low
        for(index_t i = 0; i < N; i++) {
            index_t index = i;

            for(index_t j = i+1; j < N; j++) {
                if(e[j] > e[index]) { index = j; }
            }

        
            if(index != i) {
                std::swap(e[i], e[index]);

                for(index_t k = 0; k < N; k++) { std::swap(V(k, i), V(k, index)); }
            }
        }

        return iteration;
    }

// ---------------- Decompositions ----------------
/**
 *  @brief Compute the Singular Value Decomposition (SVD) of a matrix
 *  @param A Matrix to compute SVD of
 *  @param U Left singular vectors (NxN) output
 *  @param S Singular values (NxM) output
 *  @param V Right singular vectors (MxM) output
 *  @param maxIterations (optional) The maximum number of iterations to compute for
 *  @return `iterations` The number of iterations it ran to converge
 *  @warning Computationally expensive for large matrices
 */
template<index_t N, index_t M, typename T = float, typename = std::enable_if_t<Scalar<T>>>
    size_t svd(const Matrix<N, M, T> &A, Matrix<N, N, T> &U, Matrix<N, M, T> &S, Matrix<M, M, T> &V, size_t maxIterations = MATRIX_DEFAULT_SVD_ITERATIONS) {
        static_assert(N >= M, "[MATRIX Error] : SVD only exists for matricies(NxM) with N >= M.");

        Matrix<M, M, T> AtA = transpose(A)*A;

        for(index_t i = 0; i < N; i++) {
            for(index_t j = 0; j < N; j++) {
                U(i, j) = (j < M) ?A(i, j) :static_cast<T>(0);
            }
        }
        Vector<M> eigen{};
        S = Matrix<N, M, T>::zero();
        V = Matrix<M, M, T>::eye();

        size_t iterations = jacobi(AtA, eigen, V, maxIterations);


        // Compute S, singular values
        Vector<M, T> sig{};
        for(index_t i = 0; i < M; i++) {
            sig[i] = static_cast<T>(std::sqrt(std::max(eigen[i], static_cast<T>(0))));
        }
        S = Matrix<N, M, T>::diagonal(sig);


        // Compute U, A*V*S_inv
        Matrix<N, M, T> AV = A * V;

        for(index_t j = 0; j < M; j++) {
            if(static_cast<float>(sig[j]) > epsilon_<T>) {
                for(index_t i = 0; i < N; i++) {
                    U(i, j) = AV(i, j) / sig[j];
                }
            }
        }

        return iterations;    
    }

/**
 *  @brief Compute the LU-decomposition of a matrix with partial pivoting
 *  Decomposes `A` into lower triangular `L` and upper triangular `U` matrices such that PA = LU,
 *  where `P` is a permutation matrix representing row swaps
 *  @param A Matrix to LU-decompose (NxN)
 *  @param L Lower triangular matrix L (NxN) decomposition output
 *  @param U Upper triangular matrix U (NxN) decomposition output
 *  @param P Permutation matrix P (NxN) decomposition output
 *  @param swapCount Output number of row swaps performed during decomposition
 *  @return `true` if A is non-singular and decomposition succeeded, `false` otherwise
 *  @note Only defined for square matrices
 *  @warning If function returns `false`, L, U, and P are not modified and do not represent a valid decomposition. Return value should be handled properly
 */
template<index_t N, typename T = float, typename = std::enable_if_t<Scalar<T>>>
    [[nodiscard]] bool lu(const Matrix<N, N, T> &A, Matrix<N, N, T> &L, Matrix<N, N, T> &U, Matrix<N, N, T> &P, index_t &swapCount) {
        P = Matrix<N, N, T>::eye();
        L = Matrix<N, N, T>::eye();
        U = A;

        swapCount = 0;
        
        for(index_t k = 0; k < N; k++) {
            // Get pivot
            T maxVal = static_cast<T>(std::abs(U(k, k)));
            index_t pivot = k;

            for(index_t i = k+1; i < N; i++) {
                T val = static_cast<T>(std::abs(U(i, k)));
                if(val > maxVal) {
                    maxVal = val;
                    pivot = i;
                }
            }

            if(maxVal < epsilon_<T>) { return false; } // Singular matrix

            // Swap rows
            if(pivot != k) {
                swapCount++;

                for(index_t j = 0; j < N; j++) {
                    std::swap(U(k, j), U(pivot, j));
                    std::swap(P(k, j), P(pivot, j));
                }

                for(index_t j = 0; j < k; j++) {
                    std::swap(L(k, j), L(pivot, j));
                }
            }

            // Elimination
            for(index_t i = k+1; i<N; i++) {
                T factor = U(i, k) / U(k, k);
                L(i, k) = factor;

                for(index_t j = k; j < N; j++) {
                    U(i, j) -= factor * U(k, j);
                }
            }
        }

        return true;    // Non-Singular
    }

/**
 *  @brief Compute the QR-decomposition of a matrix using the Gram-Schmidt process
 *  Decomposes `A` into orthogonal matrix `Q` and upper triangular matrix `R` such that A = QR
 *  @param A Matrix to QR-decompose (NxM)
 *  @param Q Orthogonal matrix Q (NxN) decomposition output
 *  @param R Upper triangular matrix R (NxM) decomposition output
 *  @return `true` if A's columns are linearly independent, `false` otherwise
 */
template<index_t N, index_t M, typename T = float, typename = std::enable_if_t<Scalar<T>>>
    bool qr(const Matrix<N, M, T> &A, Matrix<N, N, T> &Q, Matrix<N, M, T> &R) {
        
        bool isIndependent = gramSchmidt(A, Q);

        R = transpose(Q) * A;

        return isIndependent;
    }

/**
 *  @brief Extract a full orthonormal basis from a column vector matrix
 * 
 *  Creates an orthonormal set of vectors from the column vectors of `A` as the 
 *  first M columns of Q. If M < N, extends to a full orthonormal basis
 *
 *  @param A Matrix with column vectors to orthonormalize (NxM)
 *  @param Q Output matrix with orthonormal columns (NxN)
 *  @return `true` if input vectors were linearly independent, `false` otherwise
 */
template<index_t N, index_t M, typename T = float, typename = std::enable_if_t<Scalar<T>>>
    bool gramSchmidt(const Matrix<N, M, T> &A, Matrix<N, N, T> &Q) {
        Q = Matrix<N, N, T>::zero();
        bool isIndependent = true;

        for(index_t j = 0; j < M; j++) {
            Vector<N, T> vec = getColumn(A, j);

            for(index_t i = 0; i < j; i++) {
                Vector<N, T> qi = getColumn(Q, i);
                vec = ortho(vec, qi);
            }

            vec = normalize(vec);

            if(norm(vec) < epsilon_<T>) { isIndependent = false; } // Zero colummn

            for(index_t i = 0; i < N; i++) { 
                Q(i, j) = vec[i];
            }
        }

        if(M < N) {
            for(index_t j = M; j < N; j++) {
                Vector<N, T> vec = Vector<N, T>::zero();
                vec[j] = static_cast<T>(1);

                for(index_t i = 0; i < j; i++) {
                    Vector<N, T> qi = getColumn(Q, i);
                    vec = ortho(vec, qi);
                }

                index_t attempt = 0;
                while((norm(vec) < epsilon_<T>) && (attempt < N)) {
                    vec = Vector<N, T>::zero();
                    vec[(j + attempt) % N] = static_cast<T>(1);
                    
                    for(index_t i = 0; i < j; i++) {
                        Vector<N, T> qi = getColumn(Q, i);
                        vec = ortho(vec, qi);
                    }
                    attempt++;
                }

                vec = normalize(vec);

                for(index_t i = 0; i < N; i++) { 
                    Q(i, j) = vec[i]; 
                }
            }
        }

        return isIndependent;
    }

/**
 *  @brief Orthonormalize the columns of a matrix (reduced QR)
 * 
 *  Creates orthonormal columns from A's columns (NxM output)
 * 
 *  @param A Matrix with column vectors to orthonormalize (NxM)
 *  @param Q Output matrix with orthonormal columns (NxM)
 *  @return `true` if input vectors were linearly independent
 */
template<index_t N, index_t M, typename T = float, typename = std::enable_if_t<Scalar<T>>>
    bool gramSchmidtReduced(const Matrix<N, M, T> &A, Matrix<N, M, T> &Q) {
        Q = Matrix<N, M, T>::zero();
        bool isIndependent = true;

        for(index_t j = 0; j < M; j++) {
            Vector<N, T> vec = getColumn(A, j);

            for(index_t i = 0; i < j; i++) {
                Vector<N, T> qi = getColumn(Q, i);
                vec = ortho(vec, qi);
            }

            vec = normalize(vec);

            if(norm(vec) < epsilon_<T>) { isIndependent = false; } // Zero colummn

            for(index_t i = 0; i < N; i++) { 
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
 *  @return Vector of size N*M
 */
template<index_t N, index_t M, typename T = float, typename = std::enable_if_t<Scalar<T>>>
    constexpr Vector<N*M, T> vectorize(const Matrix<N, M, T> &A) {
        Vector<N*M, T> output;

        for(index_t i = 0; i < N; i++) {
            for(index_t j = 0; j < M; j++) {
                output[i*M + j] = A(i, j);
            }   
        }
    }

/**
 *  @brief Convert a vector into a matrix. Row first order
 * 
 *  @param v Vector to convert to a matrix
 *  @return Matrix of size NxM
 */
template<index_t N, index_t M, typename T = float, typename = std::enable_if_t<Scalar<T>>>
    constexpr Matrix<N, M, T> reshape(const Vector<N*M, T> &v) {
        Matrix<N, M, T> output;

        for(index_t i = 0; i < N; i++) {
            for(index_t j = 0; j < M; j++) {
                output(i, j) = v[i*M + j];
            }   
        }
    }

/**
 *  @brief Get the column of a matrix as a vector
 */
template<index_t N, index_t M, typename T = float, typename = std::enable_if_t<Scalar<T>>>
    constexpr Vector<N, T> getColumn(const Matrix<N, M, T> &A, index_t column = 0) {
        Vector<N, T> output;

        for(index_t i = 0; i < N; i++) {
            output[i] = A(i, column);
        }

        return output;
    }

/**
 *  @brief Get the row of a matrix as a vector
 */
template<index_t N, index_t M, typename T = float, typename = std::enable_if_t<Scalar<T>>>
    constexpr Vector<M, T> getRow(const Matrix<N, M, T> &A, index_t row = 0) {
        Vector<M, T> output;

        for(index_t j = 0; j < M; j++) {
            output[j] = A(row, j);
        }

        return output;
    }

/**
 *  @brief Get the main diagonal of a matrix as a vector
 */
template<index_t N, index_t M, typename T = float, typename = std::enable_if_t<Scalar<T>>>
    constexpr Vector<(N < M) ?N :M, T> getDiagonal(const Matrix<N, M, T> &A) {
        constexpr index_t minLength = (N < M) ?N :M;
        Vector<minLength, T> output;

        for(index_t i = 0; i < minLength; i++) {
            output[i] = A(i, i);
        }

        return output;
    }

// ---------------- Checks ----------------
/**
 *  @brief Check if a matrix is the zero matrix
 */
template<index_t N, index_t M, typename T = float, typename = std::enable_if_t<Scalar<T>>>
constexpr bool isZero(const Matrix<N, M, T> &A) {
    for(index_t i = 0; i < N; i++) {
        for(index_t j = 0; j < M; j++) {
            if(std::abs(A(i,j)) > epsilon_<T>) { return false; }
        }
    }

    return true;
}

/**
 *  @brief Check if a matrix is the identity matrix
 */
template<index_t N, typename T = float, typename = std::enable_if_t<Scalar<T>>>
constexpr bool isIdentity(const Matrix<N, N, T> &A) {
    return isZero(A - Matrix<N, N, T>::eye());
}

/**
 *  @brief Check if a matrix is symmetric
 */
template<index_t N, typename T = float, typename = std::enable_if_t<Scalar<T>>>
constexpr bool isSymmetric(const Matrix<N, N, T> &A) {
    return isZero(A - transpose(A));
}

/**
 *  @brief Check if a matrix is orthogonal
 */
template<index_t N, typename T = float, typename = std::enable_if_t<Scalar<T>>>
constexpr bool isOrthogonal(const Matrix<N, N, T> &A) {
    return isIdentity(A*transpose(A));
}

/**
 *  @brief Check if a matrix is diagonal
 */
template<index_t N, typename T = float, typename = std::enable_if_t<Scalar<T>>>
constexpr bool isDiagonal(const Matrix<N, N, T> &A) {
    return isZero(A - Matrix<N, N, T>::diagonal(getDiagonal(A)));
}

/**
 *  @brief Check if a matrix is singular
 */
template<index_t N, typename T = float, typename = std::enable_if_t<Scalar<T>>>
constexpr bool isSingular(const Matrix<N, N, T> &A) {
    return (std::abs(det(A)) < epsilon_<T>);
}

} // cobalt::math::linear_algebra