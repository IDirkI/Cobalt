#pragma once

#include <math.h>

#include "vector.hpp"

namespace cobalt::math::linear_algebra {

// ---------------- Non-member Arithmetic Overloads ----------------
/**
 *  @brief Vector addition.
 */
template<uint8_t N, typename T = float>
    constexpr inline Vector<N, T> operator+(Vector<N, T> lhs, const Vector<N, T> &rhs) { lhs += rhs; return lhs; }

/**
 *  @brief Vector subtraction.
 */
template<uint8_t N, typename T = float>
    constexpr inline Vector<N, T> operator-(Vector<N, T> lhs, const Vector<N, T> &rhs) { lhs -= rhs; return lhs; }

/**
 *  @brief Vector subtraction.
 */
template<uint8_t N, typename T = float>
    constexpr inline Vector<N, T> operator*(Vector<N, T> v, float c) { v *= c; return v; }

/**
 *  @brief Vector scalar multiplication.
 */
template<uint8_t N, typename T = float>
    constexpr inline Vector<N, T> operator*(float c, Vector<N, T> v) { return v * c; }

/**
 *  @brief Vector scalar divison.
 */
template<uint8_t N, typename T = float>
    constexpr inline Vector<N, T> operator/(Vector<N, T> v, float c) { v /= c; return v; }

/**
 *  @brief Flip the vector. Element wise negation.
 */
template<uint8_t N, typename T = float>
    constexpr Vector<N, T> operator-(Vector<N, T> v) { v *= -1; return v; }

/**
 *  @brief Check vector equality within a threshold (default 1e-5)
 */
template<uint8_t N, typename T = float>
    constexpr bool operator==(const Vector<N, T> &lhs, const Vector<N, T> &rhs) {
        for(uint8_t i = 0; i < N; i++) {
            if(fabsf(lhs[i] - rhs[i]) > static_cast<T>(VECTOR_EQUAL_THRESHOLD)) return false;
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
 *  @brief Dot product between two vectors.
 *  @return Scalar dot product.
 */
template<uint8_t N, typename T = float>
    constexpr inline float dot(const Vector<N, T> &v, const Vector<N, T> &u) { 
        float output = 0.0f;
        for(uint8_t i = 0; i < N; i++) {
            output += v[i] * u[i];
        }
        return output;
    }

/**
 *  @brief Cross product between two 3D-vectors.
 *  @return Vector cross product.
 */
template<typename T = float>
    constexpr inline Vector<3, T> cross(const Vector<3, T> &v, const Vector<3, T> &u) { 
        return Vector<3, T> {
            v.y()*u.z() - v.z()*u.y(),
            v.z()*u.x() - v.x()*u.z(),
            v.x()*u.y() - v.y()*u.x()
        };
    }

/**
 *  @brief Hadamard product between two vectors (element wise).
 *  @return Vector hadamard product.
 */
template<uint8_t N, typename T = float>
    constexpr inline Vector<N, T> hadamard(const Vector<N, T> &v, const Vector<N, T> &u) { 
        Vector<N, T> output{};
        for(uint8_t i = 0; i < N; i++) {
            output[i] = v[i]*u[i];
        }
        return output;
    }


/**
 *  @brief Compute vector norm.
 *  @return Norm/Magnitude of vector.
 */
template<uint8_t N, typename T = float>
    constexpr inline float norm(const Vector<N, T> &v) { return sqrtf(dot(v, v)); }

/**
 *  @brief Compute normalized vector.
 *  @return Normalized (unit) vector in the same direction as the vector.
 */
template<uint8_t N, typename T = float>
    constexpr inline Vector<N, T> normalize(const Vector<N, T> &v) {
        float mag = norm(v);
        Vector<N, T> output = v;
        output = (mag > VECTOR_ZERO_THRESHOLD) ?(output /= mag) :(Vector<N, T>::zero());
        return output;
    }

/**
 *  @brief Compute distance between two vectors.
 *  @return Scalar distance between the tips of the vectors.
 */
template<uint8_t N, typename T = float>
    constexpr inline float getDistance(const Vector<N, T> &v, const Vector<N, T> &u) { return norm(v - u); }

/**
 *  @brief Compute the distance squared between two vectors.
 *  @return Scalar squared distance between the tips of the vectors.
 */
template<uint8_t N, typename T = float>
    constexpr inline float getDistanceSqr(const Vector<N, T> &v, const Vector<N, T> &u) { return dot(v - u, v - u); }

/**
 *  @brief Compute angle between two vectors.
 *  @return Angle between two vectors in their shared plane (radians)
 */
template<uint8_t N, typename T = float>
    constexpr inline float getAngle(const Vector<N, T> &v, const Vector<N, T> &u) { return static_cast<T>(acosf(dot(v, u)/(norm(v)*norm(u)))); }

/**
 *  @brief Compute projection of one vector onto another.
 *  @param v Vector to project.
 *  @param u Vector to project onto.
 *  @return Projected component of v.
 */
template<uint8_t N, typename T = float>
    constexpr inline Vector<N, T> projectOnto(const Vector<N, T> &v, const Vector<N, T> &u) {
        float denom = dot(u, u);
        if(denom == static_cast<T>(0.0f)) { return Vector<N, T>{}; }

        float newLength = dot(v, u) / denom;
        return u * newLength;
    }

} // cobalt::math::linear_algebra