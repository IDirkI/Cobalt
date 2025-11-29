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
template<uint8_t N, uint8_t M, typename T = float>
    constexpr Matrix<N, M, T> operator+(Matrix<N, M, T> lhs, const Matrix<N, M, T> &rhs) { lhs += rhs; return lhs; }

/**
 *  @brief Matrix subtraction.
 */
template<uint8_t N, uint8_t M, typename T = float>
    constexpr Matrix<N, M, T> operator-(Matrix<N, M, T> lhs, const Matrix<N, M, T> &rhs) { lhs -= rhs; return lhs; }

/**
 *  @brief Scalar matrix multiplication.
 */
template<uint8_t N, uint8_t M, typename T = float>
    constexpr Matrix<N, M, T> operator*(Matrix<N, M, T> lhs, T c) { lhs *= c; return lhs; }

/**
 *  @brief Scalar matrix multiplication.
 */
template<uint8_t N, uint8_t M, typename T = float>
    constexpr Matrix<N, M, T> operator*(T c, Matrix<N, M, T> lhs) { lhs *= c; return lhs; }

/**
 *  @brief Matrix multiplication.
 */
template<uint8_t N, uint8_t M, uint8_t K, typename T = float>
    constexpr Matrix<N, K, T> operator*(Matrix<N, M, T> lhs, const Matrix<M, K, T> &rhs) { 
        Matrix<N, K, T> output{};

        for(uint8_t i = 0; i < N; i++) {
            for(uint8_t j = 0; j < K; j++) {
                output(i, j) = static_cast<T>(0);

                for(uint8_t k = 0; k < M; k++) {
                    output(i, j) += lhs(i, k) * rhs(k, j);
                }
            }
        }

        return output;
    }

/**
 *  @brief Vector right-multiplication.
 */
template<uint8_t N, uint8_t M, typename T = float>
    constexpr Vector<N, T> operator*(const Matrix<N, M, T> &A, const Vector<M, T> &v) {
        Vector<N, T> output{};
        for(uint8_t i = 0; i < N; i++) {
            output[i] = static_cast<T>(0);
            for(uint8_t j = 0; j < M; j++) {
                output[i] += v[j]*A(i, j);
            }
        }
        return output;
    }

/**
 *  @brief Unary negation
 */
template<uint8_t N, uint8_t M, typename T = float>
    constexpr Matrix<N, M, T> operator-(Matrix<N, M, T> A) { A *= -1; return A; }

/**
 *  @brief Scalar matrix divison.
 */
template<uint8_t N, uint8_t M, typename T = float>
    constexpr Matrix<N, M, T> operator/(Matrix<N, M, T> lhs, T c) { lhs /= c; return lhs; }

/**
 *  @brief Matrix equality.
 */
template<uint8_t N, uint8_t M, typename T = float>
    constexpr inline bool operator==(const Matrix<N, M, T> &lhs, const Matrix<N, M, T> &rhs) { 
        for(uint8_t i = 0; i < N; i++) {
            for(uint8_t j = 0; j < M; j++) {
                if(std::abs(lhs(i, j) - rhs(i, j)) > static_cast<T>(MATRIX_EQUAL_THRESHOLD)) return false;
            }
        }

        return true;
    }

/**
 *  @brief Matrix inequality.
 */
template<uint8_t N, uint8_t M, typename T = float>
    constexpr inline bool operator!=(const Matrix<N, M, T> &lhs, const Matrix<N, M, T> &rhs) { return (!(lhs == rhs)); }

// ---------------- Non-member Functions ----------------
/**
 *  @brief Compute determinant of a matrix(NxN)
 *  @param A Matrix to get determinant of
 */
template<uint8_t N, typename T = float>
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
                uint8_t swapCount = 0;
                if(!lu(A, L, U, P, swapCount)) { return static_cast<T>(0.0f); } // Singualr => det(A) = 0

                output = (swapCount % 2 == 0) ?static_cast<T>(1) :static_cast<T>(-1);
                output *= static_cast<T>(traceProduct(U));
                break;
            }
        }

        return output;
    }

/**
 *  @brief Compute transpose of a matrix(NxN)
 *  @param A Matrix to transpose
 */
template<uint8_t N, uint8_t M, typename T = float>
    constexpr Matrix<M, N, T> transpose(const Matrix<N, M, T> &A) {
        Matrix<M, N, T> output{};

        for(uint8_t i = 0; i < M; i++) {
            for(uint8_t j = 0; j < N; j++) {
                output(i, j) = A(j, i);
            }
        }
        
        return output;
    }

/**
 *  @brief Compute inverse of a matrix
 *  @param A Matrix to invert
 *  @param Ainv Inverted output Matrix
 *  @return `true` if inversion succeeds, `false` if A is signular.
 *  @note Return value should not be ignored and handled properly if A is singular
 */
template<uint8_t N, typename T = float>
    [[nodiscard]] constexpr bool inv(const Matrix<N, N, T> &A, Matrix<N, N, T> &Ainv) {
        switch(N) {
            case 1: { 
                if(std::abs(A(0, 0)) < MATRIX_EQUAL_THRESHOLD) { return false; }    // Singular
                Ainv(0, 0) = 1.0f / A(0, 0); 

                return true; 
            }
            case 2: { 
                T denom = static_cast<T>(det(A));
                if(std::abs(denom) < MATRIX_EQUAL_THRESHOLD) { return false; }    // Singular
                Ainv(0, 0) = A(1, 1) / denom;
                Ainv(0, 1) = -A(0, 1) / denom;
                Ainv(1, 0) = -A(1, 0) / denom;
                Ainv(1, 1) = A(0, 0) / denom;

                return true;
            }
            case 3: { 
                T denom = static_cast<T>(det(A));
                if(std::abs(denom) < MATRIX_EQUAL_THRESHOLD) { return false; }    // Singular

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
                uint8_t swapCount;
                if(!lu(A, L, U, P, swapCount)) { return false; } // Singular

                for(uint8_t j = 0; j < N; j++) {
                    Vector<N, T> e{}, x{};
                    e[j] = static_cast<T>(1);

                    if(!solve(A, e, x)) { return false; }   // Singular

                    for(uint8_t i = 0; i < N; i++) {
                        Ainv(i, j) = x[i];
                    }
                }

                return true;
            }
        }
    }

/**
 *  @brief Compute rank of matrix
 *  @param A Matrix to compute rank of 
 */
template<uint8_t N, uint8_t M, typename T = float>
    constexpr uint8_t rank(const Matrix<N, M, T> &A) {
        Matrix<N, M, T> Q;
        gramSchmidt(A, Q);
        
        uint8_t rankNum = 0;
        for(uint8_t j = 0; j < M; j++) {
            Vector<N, T> colVec = getColumn(Q, j);
            
            if(norm(colVec) > static_cast<T>(MATRIX_EQUAL_THRESHOLD)) {
                rankNum++;
            }
        }
        
        return rankNum;
    }

/**
 *  @brief Compute matrix trace
 *  @param A Matrix to compute trace of 
 */
template<uint8_t N, uint8_t M, typename T = float>
    constexpr T trace(const Matrix<N, M, T> &A) {
        T output = static_cast<T>(0);

        for(uint8_t i = 0; i < M; i++) {
            output += A(i, i);
        }
        
        return output;
    }

/**
 *  @brief Compute matrix trace product/geometric trace/diagonal product
 *  @param A Matrix to compute trace product of 
 */
template<uint8_t N, uint8_t M, typename T = float>
    constexpr T traceProduct(const Matrix<N, M, T> &A) {
        T output = static_cast<T>(1);

        for(uint8_t i = 0; i < M; i++) {
            output *= A(i, i);
        }
        
        return output;
    }

/**
 *  @brief Compute the matrix logarithm approximation of a matrix
 *  @param A Matrix to logarithmize
 *  @param terms Number of terms to approximate with
 */
template<uint8_t N, typename T = float>
    constexpr Matrix<N, N, T> log(const Matrix<N, N, T> &A, uint16_t terms = MATRIX_DEFAULT_LOG_TERMS) {
        Matrix<N, N, T> output = Matrix<N, N, T>::zero();
        Matrix<N, N, T> AmI = A - Matrix<N, N, T>::eye();;
        Matrix<N, N, T> powAmI = AmI;

        for(uint8_t i = 1; i <= terms; i++) {
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
 */
template<uint8_t N, typename T = float>
    constexpr Matrix<N, N, T> exp(const Matrix<N, N, T> &A, uint16_t terms = MATRIX_DEFAULT_EXP_TERMS) {
        Matrix<N, N, T> output = Matrix<N, N, T>::zero();
        Matrix<N, N, T> powA = Matrix<N, N, T>::eye();
        float fact = 1;

        for(uint8_t i = 0; i < terms; i++) {
            output += static_cast<T>(1.0f/fact) * powA;
            powA *= A;
            fact *= (i+1);
        }

        return output;
    }


/**
 *  @brief Compute the power of a matrix
 *  @param A Matrix to exponentiate
 *  @param c Power to raise matrix to
 */
template<uint8_t N, typename T = float>
    constexpr Matrix<N, N, T> pow(const Matrix<N, N, T> &A, float c, uint16_t terms = MATRIX_DEFAULT_POW_TERMS) {
        return exp(log(A, terms) * static_cast<T>(c), terms);
    }

/**
 *  @brief Compute the Frobenius norm of a matrix
 */
template<uint8_t N, uint8_t M, typename T = float>
    constexpr T normFrobenius(const Matrix<N, M, T> &A) {
        T sum = static_cast<T>(0.0f);

        for(uint8_t i = 0; i < N; i++) {
            for(uint8_t j = 0; j < M; j++) {
                sum += A(i, j) * A(i, j);
            }
        }
        
        return std::sqrt(sum);
    }

/**
 *  @brief Compute the inf-norm of a matrix (Largest row sum)
 */
template<uint8_t N, uint8_t M, typename T = float>
    constexpr T normInf(const Matrix<N, M, T> &A) {
        T max = static_cast<T>(0.0f);

        for(uint8_t i = 0; i < N; i++) {
            T sum = static_cast<T>(0.0f);
            for(uint8_t j = 0; j < M; j++) {
                sum += A(i,j);
            }
            max = (max < sum) ?sum :max;
        }
        
        return max;
    }

/**
 *  @brief Compute the 1-norm of a matrix (Largest column sum)
 */
template<uint8_t N, uint8_t M, typename T = float>
    constexpr T norm1(const Matrix<N, M, T> &A) {
        T max = static_cast<T>(0.0f);

        for(uint8_t j = 0; j < M; j++) {
            T sum = static_cast<T>(0.0f);
            for(uint8_t i = 0; i < N; i++) {
                sum += A(i,j);
            }
            max = (max < sum) ?sum :max;
        }
        
        return max;
    }

/**
 *  @brief Compute the 2-norm of a matrix (Largest singular value)
 */
template<uint8_t N, uint8_t M, typename T = float>
    constexpr T norm2(const Matrix<N, M, T> &A) {
        Matrix<N, N, T> U;
        Matrix<N, M, T> S;
        Matrix<M, M, T> V;
        svd(A, U, S, V);

        return S(0,0);
    }

/**
 *  @brief Compute the condition number of a matrix
 */
template<uint8_t N, uint8_t M, typename T = float>
    constexpr T conditionNum(const Matrix<N, M, T> &A) {
        Matrix<N, N, T> U;
        Matrix<N, M, T> S;
        Matrix<M, M, T> V;
        svd(A, U, S, V);

        return S(0,0)/S(M-1,M-1);
    }


/**
 *  @brief Solve the linear system A * x = b.
 *  @param A Coefficient matrix.
 *  @param b Right-hand side vector.
 *  @param x Output, solution vector.
 *  @return Whether the solution was successful. If not, A is singular thus no composion is possible.
 *  @note Return value should not be ignored and handled properly if A is singular.
 */
template<uint8_t N, typename T = float>
    [[nodiscard]] inline bool solve(const Matrix<N, N, T> &A, const Vector<N, T> &b, Vector<N, T> &x) {
        Matrix<N, N, T> L, U, P;
        uint8_t swapCount;
        
        if(!lu(A, L, U, P, swapCount)) { return false; }    // Singular

        Vector<N, T> pb = P*b;
        
        // Forward sub, Ly = Pb
        Vector<N, T> y;
        for(uint8_t i = 0; i < N; i++) {
            T sum = static_cast<T>(0.0f);
            for(uint8_t j = 0; j < i; j++) {
                sum += L(i, j) * y[j];
            } 
            y[i] = pb[i] - sum;
        }

        // Back sub, Ux = y
        for(int8_t i = N-1; i >= 0; i--) {
            T sum = static_cast<T>(0.0f);
            for(uint8_t j = i+1; j < N; j++) {
                sum += U(i, j) * x[j];
            } 
            x[i] = (y[i] - sum) / U(i, i);
        }

        return true;
    }

} // cobalt::math::linear_algebra