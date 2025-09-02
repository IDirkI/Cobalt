#pragma once

#include "matrix.hpp"
#include "matrix_util.hpp"

#include "../vector/vector.hpp"

namespace cobalt::math::linear_algebra {

// ---------------- Non-member Arithmetic Overloads ----------------
/**
 *  @brief Matrix addition.
 */
template<uint8_t R, uint8_t C, typename T = float>
    constexpr inline Matrix<R, C, T> operator+(Matrix<R, C, T> lhs, const Matrix<R, C, T> &rhs) { lhs += rhs; return lhs; }

/**
 *  @brief Matrix subtraction.
 */
template<uint8_t R, uint8_t C, typename T = float>
    constexpr inline Matrix<R, C, T> operator-(Matrix<R, C, T> lhs, const Matrix<R, C, T> &rhs) { lhs -= rhs; return lhs; }

/**
 *  @brief Scalar matrix multiplication.
 */
template<uint8_t R, uint8_t C, typename T = float>
    constexpr inline Matrix<R, C, T> operator*(Matrix<R, C, T> lhs, float c) { lhs *= c; return lhs; }

/**
 *  @brief Scalar matrix multiplication.
 */
template<uint8_t R, uint8_t C, typename T = float>
    constexpr inline Matrix<R, C, T> operator*(float c, Matrix<R, C, T> lhs) { lhs *= c; return lhs; }

/**
 *  @brief Matrix multiplication.
 */
template<uint8_t R, uint8_t C, uint8_t K, typename T = float>
    constexpr inline Matrix<R, K, T> operator*(Matrix<R, C, T> lhs, const Matrix<C, K, T> &rhs) { lhs *= rhs; return lhs; }

/**
 *  @brief Vector right-multiplication.
 */
template<uint8_t R, uint8_t C, typename T = float>
    constexpr inline Vector<R, T> operator*(const Matrix<R, C, T> &A, const Vector<C, T> &v) {
        Vector<R, T> output{};
        for(uint8_t i = 0; i < R; i++) {
            output[i] = static_cast<T>(0);
            for(uint8_t j = 0; j < C; j++) {
                output[i] += v[j]*A(i, j);
            }
        }
        return output;
    }

/**
 *  @brief Scalar matrix divison.
 */
template<uint8_t R, uint8_t C, typename T = float>
    constexpr inline Matrix<R, C, T> operator/(Matrix<R, C, T> lhs, float c) { lhs /= c; return lhs; }

/**
 *  @brief Matrix equality.
 */
template<uint8_t R, uint8_t C, typename T = float>
    constexpr inline bool operator==(const Matrix<R, C, T> &lhs, const Matrix<R, C, T> &rhs) { 
        for(uint8_t i = 0; i < R; i++) {
            for(uint8_t j = 0; j < C; j++) {
                if(std::abs(lhs(i, j) - rhs(i, j)) > static_cast<T>(MATRIX_EQUAL_THRESHOLD)) return false;
            }
        }

        return true;
    }

/**
 *  @brief Matrix inequality.
 */
template<uint8_t R, uint8_t C, typename T = float>
    constexpr inline bool operator!=(const Matrix<R, C, T> &lhs, const Matrix<R, C, T> &rhs) { return (!(lhs == rhs)); }

// ---------------- Non-member Functions ----------------
/**
 *  @brief Determinant of a matrix
 *  @param A Matrix to get determinant of
 */
template<uint8_t N, typename T = float>
    constexpr inline T det(const Matrix<N, N, T> &A) {
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
                Matrix<N, N, T> L, U;
                Vector<N, T> P;

                if(!decompLU(A, L, U, P)) { output = static_cast<T>(0); break; } // Singualr => det(A) = 0

                uint8_t swapCount = 0;
                std::array<bool, N> visited{false};

                for(uint8_t i = 0; i < N; i++) {
                    if(!visited[i]) {
                        uint8_t cycle = 0;
                        uint8_t j = i;
                        while(!visited[j]) {
                            visited[j] = true;
                            j = P[j];
                            cycle++;
                        }

                        if(cycle > 0) { swapCount += cycle-1; }
                    }
                }

                output = (swapCount % 2 == 0) ?static_cast<T>(1) :static_cast<T>(-1);
                output *= static_cast<T>(traceProduct(U));
                break;
            }
        }

        return output;
    }

/**
 *  @brief Matrix transposition
 *  @param A Matrix to transpose
 */
template<uint8_t R, uint8_t C, typename T = float>
    constexpr inline Matrix<C, R, T> transpose(const Matrix<R, C, T> &A) {
        Matrix<C, R, T> output{};

        for(uint8_t i = 0; i < C; i++) {
            for(uint8_t j = 0; j < R; j++) {
                output(i, j) = A(j, i);
            }
        }
        
        return output;
    }

/**
 *  @brief Compute inverse of a matrix
 *  @param A Matrix to invert
 *  @param Ainv Inverted Matrix
 */
template<uint8_t N, typename T = float>
    [[nodiscard]] constexpr inline bool inv(const Matrix<N, N, T> &A, Matrix<N, N, T> &Ainv) {
        switch(N) {
            case 1: { Ainv(0, 0) = 1.0f / A(0, 0); return true; }   //TODO check A(0,0) = 0 case
            case 2: { 
                float denom = static_cast<float>(det(A));
                if(std::fabs(denom) < MATRIX_EQUAL_THRESHOLD) { return false; }    // Singular
                Ainv(0, 0) = A(1, 1) / denom;
                Ainv(0, 1) = -A(0, 1) / denom;
                Ainv(1, 0) = -A(1, 0) / denom;
                Ainv(1, 1) = -A(0, 0) / denom;

                return true;
            }
            case 3: { 
                float denom = static_cast<float>(det(A));
                if(std::fabs(denom) < MATRIX_EQUAL_THRESHOLD) { return false; }    // Singular

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
                Matrix<N, N, T> L, U;
                Vector<N, T> P;

                if(!decompLU(A, L, U, P)) { return false; } // Singular

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
 *  @brief Compute matrix trace
 *  @param A Matrix to compute trace of 
 */
template<uint8_t R, uint8_t C, typename T = float>
    constexpr inline T trace(const Matrix<R, C, T> &A) {
        T output = static_cast<T>(0);

        for(uint8_t i = 0; i < C; i++) {
            output += A(i, i);
        }
        
        return output;
    }

/**
 *  @brief Compute matrix trace product/geometric trace/diagonal product
 *  @param A Matrix to compute trace product of 
 */
template<uint8_t R, uint8_t C, typename T = float>
    constexpr inline T traceProduct(const Matrix<R, C, T> &A) {
        T output = static_cast<T>(1);

        for(uint8_t i = 0; i < C; i++) {
            output *= A(i, i);
        }
        
        return output;
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
        Matrix<N, N, T> L, U;
        Vector<N, T> P;
        
        if(!decompLU(A, L, U, P)) { return false; }    // Singular

        // Solve for Pb = b'
        Vector<N, T> bp{};
        for(uint8_t i = 0; i < N; i++) { bp[i] = b[P[i]]; }

        Vector<N, T> y{}; 
        for(uint8_t i = 0; i < N; i++) { // Solve for L*y = b', frwd sub
            T sum = static_cast<T>(bp[i]);

            for(uint8_t j = 0; j < i; j++) {
                sum -= L(i,j) * y[j];
            }

            y[i] = sum;
        }

         
        for(int i = N-1; i >= 0; i--) { // Solve for U*x = y, bcwd sub
            T sum = y[i];

            for(uint8_t j = i+1; j < N; j++) {
                sum -= U(i,j) * x[j];
            }

            if(static_cast<T>(std::fabs(U(i, i))) < static_cast<T>(MATRIX_EQUAL_THRESHOLD)) { return false; } // Singular

            x[i] = sum / U(i, i);
        }

        return true;
    }

} // cobalt::math::linear_algebra