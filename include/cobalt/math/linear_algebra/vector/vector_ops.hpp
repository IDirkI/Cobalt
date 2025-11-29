#pragma once

#include <cmath>

#include "vector.hpp"

namespace cobalt::math::linear_algebra {

// ---------------- Non-member Arithmetic Overloads ----------------
/**
 *  @brief Vector addition
 */
template<uint8_t N, typename T = float>
    constexpr Vector<N, T> operator+(Vector<N, T> lhs, const Vector<N, T> &rhs) { lhs += rhs; return lhs; }

/**
 *  @brief Vector subtraction
 */
template<uint8_t N, typename T = float>
    constexpr Vector<N, T> operator-(Vector<N, T> lhs, const Vector<N, T> &rhs) { lhs -= rhs; return lhs; }

/**
 *  @brief Vector subtraction
 */
template<uint8_t N, typename T = float>
    constexpr Vector<N, T> operator*(Vector<N, T> v, T c) { v *= c; return v; }

/**
 *  @brief Vector scalar multiplication
 */
template<uint8_t N, typename T = float>
    constexpr Vector<N, T> operator*(T c, Vector<N, T> v) { return v * c; }

/**
 *  @brief Vector scalar divison
 */
template<uint8_t N, typename T = float>
    constexpr Vector<N, T> operator/(Vector<N, T> v, T c) { v /= c; return v; }

/**
 *  @brief Flip the vector. Element wise negation
 */
template<uint8_t N, typename T = float>
    constexpr Vector<N, T> operator-(Vector<N, T> v) { v *= -1; return v; }

/**
 *  @brief Check vector equality within a threshold (default 1e-5)
 */
template<uint8_t N, typename T = float>
    constexpr bool operator==(const Vector<N, T> &lhs, const Vector<N, T> &rhs) {
        for(uint8_t i = 0; i < N; i++) {
            if(std::abs(lhs[i] - rhs[i]) > static_cast<T>(VECTOR_EQUAL_THRESHOLD)) return false;
        }
        return true;
    }

/**
 *  @brief Check vector non-equality within a threshold (default 1e-5)
 */
template<uint8_t N, typename T = float>
    constexpr bool operator!=(const Vector<N, T> &lhs, const Vector<N, T> &rhs) { return !(lhs == rhs); }


// ---------------- Non-member Functions ----------------
/**
 *  @brief Dot product between two vectors
 *  @return Scalar dot product
 */
template<uint8_t N, typename T = float>
    constexpr T dot(const Vector<N, T> &v, const Vector<N, T> &u) { 
        T output = static_cast<T>(0.0f);
        for(uint8_t i = 0; i < N; i++) {
            output += v[i] * u[i];
        }
        return output;
    }

/**
 *  @brief Cross product between two 3-vectors
 *  @return Vector cross product
 */
template<typename T = float>
    constexpr Vector<3, T> cross(const Vector<3, T> &v, const Vector<3, T> &u) { 
        return Vector<3, T> {
            v.y()*u.z() - v.z()*u.y(),
            v.z()*u.x() - v.x()*u.z(),
            v.x()*u.y() - v.y()*u.x()
        };
    }

/**
 *  @brief Hadamard (element wise) product between two vectors
 *  @return Vector Hadamard product
 */
template<uint8_t N, typename T = float>
    constexpr Vector<N, T> hadamard(const Vector<N, T> &v, const Vector<N, T> &u) { 
        Vector<N, T> output{};
        for(uint8_t i = 0; i < N; i++) {
            output[i] = v[i]*u[i];
        }
        return output;
    }

/**
 *  @brief Triple product between three 3-vectors
 *  @return Scalar triple product (dot(v, cross(u, w)))
 */
template<typename T = float>
    constexpr T tripleProduct(const Vector<3, T> &v, const Vector<3, T> &u, const Vector<3, T> &w) { 
        return dot(v, cross(u, w));
    }


/**
 *  @brief Compute vector norm/magnitude
 *  @return Norm/magnitude of vector
 */
template<uint8_t N, typename T = float>
    constexpr float norm(const Vector<N, T> &v) { return std::sqrt(dot(v, v)); }

/**
 *  @brief Compute squared vector norm/magnitude
 *  @return Squared norm/magnitude of vector
 */
template<uint8_t N, typename T = float>
    constexpr float normSqr(const Vector<N, T> &v) { return dot(v, v); }

/**
 *  @brief Normalize a vector to unit length
 *  @return Normalized vector
 */
template<uint8_t N, typename T = float>
    constexpr Vector<N, T> normalize(const Vector<N, T> &v) {
        float mag = norm(v);
        Vector<N, T> output = v;
        output = (mag > VECTOR_EQUAL_THRESHOLD) ?(output /= mag) :(Vector<N, T>::zero());
        return output;
    }

/**
 *  @brief Compute the distance between two vectors
 *  @return Scalar distance between the tips of the vectors
 */
template<uint8_t N, typename T = float>
    constexpr float distance(const Vector<N, T> &v, const Vector<N, T> &u) { return norm(v - u); }

/**
 *  @brief Compute the squared distance between two vectors
 *  @return Squared distance between the tips of the vectors
 */
template<uint8_t N, typename T = float>
    constexpr float distanceSqr(const Vector<N, T> &v, const Vector<N, T> &u) { return dot(v - u, v - u); }

/**
 *  @brief Compute the angle between two vectors in radians
 *  @return Angle between the vectors in radians
 */
template<uint8_t N, typename T = float>
    constexpr float angle(const Vector<N, T> &v, const Vector<N, T> &u) { return std::acos(dot(v, u)/(norm(v)*norm(u))); }

/**
 *  @brief Project vector v onto vector u
 *  @return Projected vector
 *  @note If u is the zero vector, returns the zero vector
 */
template<uint8_t N, typename T = float>
    constexpr Vector<N, T> project(const Vector<N, T> &v, const Vector<N, T> &u) {
        T denom = dot(u, u);
        if(std::abs(denom) < static_cast<T>(VECTOR_EQUAL_THRESHOLD)) { return Vector<N, T>::zero(); }

        T scale = dot(v, u) / denom;
        return u * scale;
    }

} // cobalt::math::linear_algebra