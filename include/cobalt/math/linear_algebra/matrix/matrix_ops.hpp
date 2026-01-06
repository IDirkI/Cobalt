#pragma once

#include <cmath>

#include "matrix.hpp"
#include "matrix_util.hpp"

#include "../vector/vector.hpp"

namespace cobalt::math::linear_algebra {

// ---------------- Non-member Arithmetic Overloads ----------------
/**
 *  @brief Matrix addition.
 */
template<index_t N, index_t M, typename T = def_scalar, typename = std::enable_if_t<Scalar<T>>>
    constexpr Matrix<N, M, T> operator+(Matrix<N, M, T> lhs, const Matrix<N, M, T> &rhs) noexcept { lhs += rhs; return lhs; }

/**
 *  @brief Matrix subtraction.
 */
template<index_t N, index_t M, typename T = def_scalar, typename = std::enable_if_t<Scalar<T>>>
    constexpr Matrix<N, M, T> operator-(Matrix<N, M, T> lhs, const Matrix<N, M, T> &rhs) noexcept { lhs -= rhs; return lhs; }

/**
 *  @brief Scalar matrix multiplication.
 */
template<index_t N, index_t M, typename T = def_scalar, typename = std::enable_if_t<Scalar<T>>>
    constexpr Matrix<N, M, T> operator*(Matrix<N, M, T> lhs, T c) noexcept { lhs *= c; return lhs; }

/**
 *  @brief Scalar matrix multiplication.
 */
template<index_t N, index_t M, typename T = def_scalar, typename = std::enable_if_t<Scalar<T>>>
    constexpr Matrix<N, M, T> operator*(T c, Matrix<N, M, T> lhs) noexcept { lhs *= c; return lhs; }

/**
 *  @brief Matrix multiplication.
 */
template<index_t N, index_t M, index_t K, typename T = def_scalar, typename = std::enable_if_t<Scalar<T>>>
    constexpr Matrix<N, K, T> operator*(Matrix<N, M, T> lhs, const Matrix<M, K, T> &rhs) noexcept { 
        Matrix<N, K, T> output{};

        for(index_t i = 0; i < N; i++) {
            for(index_t j = 0; j < K; j++) {
                output(i, j) = static_cast<T>(0);

                for(index_t k = 0; k < M; k++) {
                    output(i, j) += lhs(i, k) * rhs(k, j);
                }
            }
        }

        return output;
    }

/**
 *  @brief Vector right-multiplication.
 */
template<index_t N, index_t M, typename T = def_scalar, typename = std::enable_if_t<Scalar<T>>>
    constexpr Vector<N, T> operator*(const Matrix<N, M, T> &A, const Vector<M, T> &v) noexcept {
        Vector<N, T> output{};
        for(index_t i = 0; i < N; i++) {
            output[i] = static_cast<T>(0);
            for(index_t j = 0; j < M; j++) {
                output[i] += v[j]*A(i, j);
            }
        }
        return output;
    }

/**
 *  @brief Unary negation
 */
template<index_t N, index_t M, typename T = def_scalar, typename = std::enable_if_t<Scalar<T>>>
    constexpr Matrix<N, M, T> operator-(Matrix<N, M, T> A) noexcept { A *= -1; return A; }

/**
 *  @brief Scalar matrix divison.
 */
template<index_t N, index_t M, typename T = def_scalar, typename = std::enable_if_t<Scalar<T>>>
    constexpr Matrix<N, M, T> operator/(Matrix<N, M, T> lhs, T c) noexcept { lhs /= c; return lhs; }

/**
 *  @brief Matrix equality.
 */
template<index_t N, index_t M, typename T = def_scalar, typename = std::enable_if_t<Scalar<T>>>
    constexpr inline bool operator==(const Matrix<N, M, T> &lhs, const Matrix<N, M, T> &rhs) { 
        for(index_t i = 0; i < N; i++) {
            for(index_t j = 0; j < M; j++) {
                if(std::abs(lhs(i, j) - rhs(i, j)) > epsilon_<T>) return false;
            }
        }

        return true;
    }

/**
 *  @brief Matrix inequality.
 */
template<index_t N, index_t M, typename T = def_scalar, typename = std::enable_if_t<Scalar<T>>>
    constexpr inline bool operator!=(const Matrix<N, M, T> &lhs, const Matrix<N, M, T> &rhs) noexcept { return (!(lhs == rhs)); }

// ---------------- Non-member Functions ----------------
/**
 *  @brief Compute determinant of a matrix
 *  @param A Matrix to compute determinant of
 *  @return Determinant of A
 *  @note Only defined for square matrices
 */
template<index_t N, typename T = def_scalar, typename = std::enable_if_t<Scalar<T>>>
    constexpr T det(const Matrix<N, N, T> &A) {
        T output = static_cast<T>(0);

        switch(N) {
            case 1: { output = static_cast<T>(A(0, 0))  ; break; }
            case 2: { 
                output += static_cast<T>(A(0, 0)*A(1, 1));
                output -= static_cast<T>(A(0, 1)*A(1, 0));
                break;
            }
            case 3: {
                output += static_cast<T>(A(0, 0)*A(1, 1)*A(2, 2) + A(0, 1)*A(1, 2)*A(2, 0) + A(0, 2)*A(1, 0)*A(2, 1));
                output -= static_cast<T>(A(0, 2)*A(1, 1)*A(2, 0) + A(0, 0)*A(1, 2)*A(2, 1) + A(0, 1)*A(1, 0)*A(2, 2));
                break;
            }
            default: {
                Matrix<N, N, T> L, U, P;
                index_t swapCount = 0;
                if(!lu(A, L, U, P, swapCount)) { return static_cast<T>(0.0f); } // Singualr => det(A) = 0

                output = (swapCount % 2 == 0) ?static_cast<T>(1) :static_cast<T>(-1);
                output *= static_cast<T>(traceProduct(U));
                break;
            }
        }

        return output;
    }

/**
 *  @brief Compute transpose of a matrix
 *  @param A Matrix to transpose
 *  @return Transposed matrix A^T 
 */
template<index_t N, index_t M, typename T = def_scalar, typename = std::enable_if_t<Scalar<T>>>
    constexpr Matrix<M, N, T> transpose(const Matrix<N, M, T> &A) noexcept {
        Matrix<M, N, T> output{};

        for(index_t i = 0; i < M; i++) {
            for(index_t j = 0; j < N; j++) {
                output(i, j) = A(j, i);
            }
        }
        
        return output;
    }

/**
 *  @brief Compute inverse of a matrix
 *  @param A Matrix to compute inverse of
 *  @param Ainv Inverse matrix A^-1 output
 *  @return `true` if A is invertible, `false` otherwise
 *  @note Only defined for square matrices
 *  @warning If function returns `false`, Ainv is not modified and is not a valid matrix. Return value should be handled properly.
 */
template<index_t N, typename T = def_scalar, typename = std::enable_if_t<Scalar<T>>>
    [[nodiscard]] constexpr bool inv(const Matrix<N, N, T> &A, Matrix<N, N, T> &Ainv) {
        switch(N) {
            case 1: { 
                if(std::abs(A(0, 0)) < epsilon_<T>) { return false; }    // Singular
                Ainv(0, 0) = 1.0f / A(0, 0); 

                return true; 
            }
            case 2: { 
                T denom = static_cast<T>(det(A));
                if(std::abs(denom) < epsilon_<T>) { return false; }    // Singular
                Ainv(0, 0) = A(1, 1) / denom;
                Ainv(0, 1) = -A(0, 1) / denom;
                Ainv(1, 0) = -A(1, 0) / denom;
                Ainv(1, 1) = A(0, 0) / denom;

                return true;
            }
            case 3: { 
                T denom = static_cast<T>(det(A));
                if(std::abs(denom) < epsilon_<T>) { return false; }    // Singular

                Ainv(0, 0) =  (A(1,1)*A(2,2) - A(1,2)*A(2,1)) / denom;
                Ainv(0, 1) = -(A(0,1)*A(2,2) - A(0,2)*A(2,1)) / denom;
                Ainv(0, 2) =  (A(0,1)*A(1,2) - A(0,2)*A(1,1)) / denom;

                Ainv(1, 0) = -(A(1,0)*A(2,2) - A(1,2)*A(2,0)) / denom;
                Ainv(1, 1) =  (A(0,0)*A(2,2) - A(0,2)*A(2,0)) / denom;
                Ainv(1, 2) = -(A(0,0)*A(1,2) - A(0,2)*A(1,0)) / denom;

                Ainv(2, 0) =  (A(1,0)*A(2,1) - A(1,1)*A(2,0)) / denom;
                Ainv(2, 1) = -(A(0,0)*A(2,1) - A(0,1)*A(2,0)) / denom;
                Ainv(2, 2) =  (A(0,0)*A(1,1) - A(0,1)*A(1,0)) / denom;
                
                return true;
            }
            default: { 
                Matrix<N, N, T> L, U, P;
                index_t swapCount;
                if(!lu(A, L, U, P, swapCount)) { return false; } // Singular

                for(index_t j = 0; j < N; j++) {
                    Vector<N, T> e{}, x{};
                    e[j] = static_cast<T>(1);

                    if(!solve(A, e, x)) { return false; }   // Singular

                    for(index_t i = 0; i < N; i++) {
                        Ainv(i, j) = x[i];
                    }
                }

                return true;
            }
        }
    }

/**
 *  @brief Compute the rank of a matrix
 *  @param A Matrix to compute rank of
 *  @return Rank of A
 */
template<index_t N, index_t M, typename T = def_scalar, typename = std::enable_if_t<Scalar<T>>>
    constexpr index_t rank(const Matrix<N, M, T> &A) {
        Matrix<N, M, T> Q;
        gramSchmidt(A, Q);
        
        index_t rankNum = 0;
        for(index_t j = 0; j < M; j++) {
            Vector<N, T> colVec = getColumn(Q, j);
            
            if(norm(colVec) > epsilon_<T>) {
                rankNum++;
            }
        }
        
        return rankNum;
    }

/**
 *  @brief Compute matrix trace (sum of diagonal elements)
 *  @param A Matrix to compute trace of
 *  @return Trace of A
 */
template<index_t N, index_t M, typename T = def_scalar, typename = std::enable_if_t<Scalar<T>>>
    constexpr T trace(const Matrix<N, M, T> &A) noexcept {
        index_t minLength = (N < M) ?N :M;

        T output = static_cast<T>(0);

        for(index_t i = 0; i < minLength; i++) {
            output += A(i, i);
        }
        
        return output;
    }

/**
 *  @brief Compute product of a matrix's diagonal elements (trace product)
 *  @param A Matrix to compute trace product of
 *  @return Trace product of A
 */
template<index_t N, index_t M, typename T = def_scalar, typename = std::enable_if_t<Scalar<T>>>
    constexpr T traceProduct(const Matrix<N, M, T> &A) noexcept {
        index_t minLength = (N < M) ?N :M;

        T output = static_cast<T>(1);

        for(index_t i = 0; i < minLength; i++) {
            output *= A(i, i);
        }
        
        return output;
    }

/**
 *  @brief Compute the hadamard (element-wise) product of two matrices
 */
template<index_t N, index_t M, typename T = def_scalar, typename = std::enable_if_t<Scalar<T>>>
    constexpr Matrix<N, M, T> hadamard(const Matrix<N, M, T> &A, const Matrix<N, M, T> &B) noexcept {
        Matrix<N, M, T> output = Matrix<N, M, T>::zero();
        
        for(index_t i = 0; i < N; i++) {
            for(index_t j = 0; j < M; j++) {
                output(i,j) = A(i,j) * B(i,j);
            }
        }

        return output;
    }

/**
 *  @brief Compute the matrix logarithm approximation of a matrix
 *  @param A Matrix to logarithmize
 *  @param terms Number of terms to approximate with
 *  @return Matrix logarithm of A
 *  @note Only defined for square matrices
 *  @warning Converges only for matrices where ||A - I|| < 1
 */
template<index_t N, typename T = def_scalar, typename = std::enable_if_t<Scalar<T>>>
    constexpr Matrix<N, N, T> log(const Matrix<N, N, T> &A, uint16_t terms = MATRIX_DEFAULT_LOG_TERMS) {
        Matrix<N, N, T> output = Matrix<N, N, T>::zero();
        Matrix<N, N, T> AmI = A - Matrix<N, N, T>::eye();;
        Matrix<N, N, T> powAmI = AmI;

        for(index_t i = 1; i <= terms; i++) {
            T coeff = (i % 2 == 1) ?static_cast<T>(1.0f/i) :static_cast<T>(-1.0f/i);
            output += coeff * powAmI;
            powAmI *= AmI;
        }

        return output;
    }

/**
 *  @brief Compute the matrix exponential approximation of a matrix
 *  @param A Matrix to exponentiate
 *  @param terms Number of terms to approximate with
 *  @return Matrix exponential of A
 *  @note Only defined for square matrices
 */
template<index_t N, typename T = def_scalar, typename = std::enable_if_t<Scalar<T>>>
    constexpr Matrix<N, N, T> exp(const Matrix<N, N, T> &A, uint16_t terms = MATRIX_DEFAULT_EXP_TERMS) {
        Matrix<N, N, T> output = Matrix<N, N, T>::zero();
        Matrix<N, N, T> powA = Matrix<N, N, T>::eye();
        float fact = 1;

        for(index_t i = 0; i < terms; i++) {
            output += static_cast<T>(1.0f/fact) * powA;
            powA *= A;
            fact *= (i+1);
        }

        return output;
    }


/**
 *  @brief Compute the matrix power of a matrix raised to a scalar exponent
 *  @param A Matrix to exponentiate
 *  @param c Scalar exponent
 *  @param terms Number of terms to approximate with
 *  @return Matrix A raised to the power of c
 *  @note Only defined for square matrices
 *  @warning Converges only for matrices where ||A - I|| < 1
 */
template<index_t N, typename T = def_scalar, typename = std::enable_if_t<Scalar<T>>>
    constexpr Matrix<N, N, T> pow(const Matrix<N, N, T> &A, float c, uint16_t terms = MATRIX_DEFAULT_POW_TERMS) {
        return exp(log(A, terms) * static_cast<T>(c), terms);
    }

/**
 *  @brief Compute the matrix power of a matrix raised to an integer exponent
 *  @param A Matrix to exponentiate
 *  @param n Integer exponent (>= 0)
 *  @param terms Number of terms to approximate with
 *  @return Matrix A raised to the power of n
 *  @note Only defined for square matrices
 */
template<index_t N, typename T = def_scalar, typename = std::enable_if_t<Scalar<T>>>
    constexpr Matrix<N, N, T> powInt(const Matrix<N, N, T> &A, uint16_t n, uint16_t terms = MATRIX_DEFAULT_POW_TERMS) noexcept {
        Matrix<N, N, T> output = Matrix<N, N, T>::eye();
        Matrix<N, N, T> powA = A;
        
        for(index_t i = 0; i < n; i++) {
            output *= powA;
        }

        return output;
    }

/**
 *  @brief Compute the Frobenius norm of a matrix
 *  @param A Matrix to compute Frobenius norm of
 *  @return Frobenius norm of A
 */
template<index_t N, index_t M, typename T = def_scalar, typename = std::enable_if_t<Scalar<T>>>
    constexpr T normFrobenius(const Matrix<N, M, T> &A) noexcept {
        T sum = static_cast<T>(0.0f);

        for(index_t i = 0; i < N; i++) {
            for(index_t j = 0; j < M; j++) {
                sum += A(i, j) * A(i, j);
            }
        }
        
        return std::sqrt(sum);
    }

/**
 *  @brief Compute the infinity norm of a matrix (Largest row sum)
 *  @param A Matrix to compute infinity norm of
 *  @return Infinity norm of A
 */
template<index_t N, index_t M, typename T = def_scalar, typename = std::enable_if_t<Scalar<T>>>
    constexpr T normInf(const Matrix<N, M, T> &A) noexcept {
        T max = static_cast<T>(0.0f);

        for(index_t i = 0; i < N; i++) {
            T sum = static_cast<T>(0.0f);
            for(index_t j = 0; j < M; j++) {
                sum += A(i,j);
            }
            max = (max < sum) ?sum :max;
        }
        
        return max;
    }

/**
 *  @brief Compute the infinity norm of a matrix (Largest row sum)
 *  @param A Matrix to compute infinity norm of
 *  @return Infinity norm of A
 */
template<index_t N, index_t M, typename T = def_scalar, typename = std::enable_if_t<Scalar<T>>>
    constexpr T norm1(const Matrix<N, M, T> &A) noexcept {
        T max = static_cast<T>(0.0f);

        for(index_t j = 0; j < M; j++) {
            T sum = static_cast<T>(0.0f);
            for(index_t i = 0; i < N; i++) {
                sum += A(i,j);
            }
            max = (max < sum) ?sum :max;
        }
        
        return max;
    }

/**
 *  @brief Compute the 2-norm (spectral norm) of a matrix
 *  @param A Matrix to compute 2-norm of
 *  @return 2-norm of A
 *  @note Computed via singular value decomposition (SVD)
 *  @warning Computationally expensive for large matrices
 */
template<index_t N, index_t M, typename T = def_scalar, typename = std::enable_if_t<Scalar<T>>>
    constexpr T norm2(const Matrix<N, M, T> &A) noexcept {
        Matrix<N, N, T> U;
        Matrix<N, M, T> S;
        Matrix<M, M, T> V;
        svd(A, U, S, V);

        return S(0,0);
    }

/**
 *  @brief Compute the condition number of a matrix
 *  @param A Matrix to compute condition number of
 *  @return Condition number of A
 *  @note Computed via singular value decomposition (SVD)
 *  @warning Computationally expensive for large matrices
 */
template<index_t N, index_t M, typename T = def_scalar, typename = std::enable_if_t<Scalar<T>>>
    constexpr T conditionNum(const Matrix<N, M, T> &A) noexcept {
        Matrix<N, N, T> U;
        Matrix<N, M, T> S;
        Matrix<M, M, T> V;
        svd(A, U, S, V);

        return S(0,0)/S(M-1,M-1);
    }


/**
 *  @brief Solve the linear system Ax = b using LU decomposition
 *  @param A Coefficient matrix
 *  @param b Right-hand side vector
 *  @param x Solution vector output
 *  @return `true` if the system has a unique solution, `false` otherwise
 *  @note Only defined for square matrices
 *  @warning If function returns `false`, x is not modified and is not a valid solution. Return value should be handled properly.
 */
template<index_t N, typename T = def_scalar, typename = std::enable_if_t<Scalar<T>>>
    [[nodiscard]] inline bool solve(const Matrix<N, N, T> &A, const Vector<N, T> &b, Vector<N, T> &x) {
        Matrix<N, N, T> L, U, P;
        index_t swapCount;
        
        if(!lu(A, L, U, P, swapCount)) { return false; }    // Singular

        Vector<N, T> pb = P*b;
        
        // Forward sub, Ly = Pb
        Vector<N, T> y;
        for(index_t i = 0; i < N; i++) {
            T sum = static_cast<T>(0.0f);
            for(index_t j = 0; j < i; j++) {
                sum += L(i, j) * y[j];
            } 
            y[i] = pb[i] - sum;
        }

        // Back sub, Ux = y
        for(int8_t i = N-1; i >= 0; i--) {
            T sum = static_cast<T>(0.0f);
            for(index_t j = i+1; j < N; j++) {
                sum += U(i, j) * x[j];
            } 
            x[i] = (y[i] - sum) / U(i, i);
        }

        return true;
    }

} // cobalt::math::linear_algebra