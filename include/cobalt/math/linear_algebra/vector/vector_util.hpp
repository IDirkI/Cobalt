#pragma once

#include <algorithm>
#include <sstream>
#include <iomanip>

#include "vector.hpp"

namespace cobalt::math::linear_algebra {

// ---------------- Member Utility ----------------
/**
 *  @brief Convert the vector to a string representation.
 *  @param percision Number of decimal places.
 *  @return String representation of the vector.
 */
template<uint8_t N, typename T>
    std::string Vector<N, T>::toString(uint8_t percision) const {
        std::ostringstream oss;
        oss << "[ ";
        for(uint8_t i = 0; i < N; i++) {
            oss << std::fixed << std::setprecision(percision) << data_[i];
            if(i != N-1) { oss << ", "; }
        }
        oss << " ]ᵀ";

        return oss.str();
    }

// ---------------- Non-member Utility ----------------
/**
 *  @brief Clamp the elements of a vector between an interval
 *  @param v Vector to project.
 *  @param min Lower clamp bound.
 *  @param max Upper clamp bound.
 *  @return Element wise clamped vector v between [min, max]
 */
template<uint8_t N, typename T = float>
    constexpr inline Vector<N, T> clamp(const Vector<N, T> &v, float min, float max) {
        Vector<N, T> output;
        for(uint8_t i = 0; i < N; i++) {
            if(v[i] > max)      { output[i] = max; }
            else if(v[i] < min) { output[i] = min; }
            else                { output[i] = v[i]; }
        }

        return output;
    }

/**
 *  @brief Compute vector's element wise sign
 *  @param v Vector to sign check.
 *  @return Vector with sign of each element of v
 */
template<uint8_t N, typename T = float>
    constexpr inline Vector<N, T> sign(const Vector<N, T> &v) {
        Vector<N, T> output;
        for(uint8_t i = 0; i < N; i++) {
            if(v[i] > 0.0f)         { output[i] = static_cast<T>(1); }
            else if(v[i] < 0.0f)    { output[i] = static_cast<T>(-1); }
            else                    { output[i] = static_cast<T>(0); }
        }

        return output;
    }

/**
 *  @brief Compute element wise absolute value on vector
 *  @param v Vector to absolute value.
 *  @return Vector with absolute valued elements
 */
template<uint8_t N, typename T = float>
    constexpr inline Vector<N, T> abs(const Vector<N, T> &v) {
        Vector<N, T> output;
        for(uint8_t i = 0; i < N; i++) {
            output[i] = (v[i] >= 0.0f) ?v[i] :-v[i];
        }

        return output;
    }

/**
 *  @brief Compute smallest element of a vector
 *  @param v Vector to check.
 *  @return Smallest vector element
 */
template<uint8_t N, typename T = float>
    constexpr inline T min(const Vector<N, T> &v) {
        T output = v[0];

        for(uint8_t i = 0; i < N; i++) {
            if(v[i] < output ) { output = v[i]; }
        }

        return output;
    }

/**
 *  @brief Compute largest element of a vector
 *  @param v Vector to check.
 *  @return Largest vector element
 */
template<uint8_t N, typename T = float>
    constexpr inline T max(const Vector<N, T> &v) {
        T output = v[0];

        for(uint8_t i = 0; i < N; i++) {
            if(v[i] > output ) { output = v[i]; }
        }

        return output;
    }

/**
 *  @brief Compute projection of a 3D-vector on to a plane (plane normal).
 *  @param v Vector to project.
 *  @param n Unit plane normal.
 *  @return Vector v projected onto the plane of n.
 */
template<uint8_t N, typename T = float>
    constexpr inline Vector<N, T> projectOntoPlane(const Vector<N, T> &v, const Vector<N, T> &n) {
        return v - dot(v, n)*n;
    }

/**
 *  @brief Compute component of one vector orthogonal to another.
 *  @param v Vector to reject.
 *  @param u Vector to reject from.
 *  @return Orthogonal component of v to u.
 */
template<uint8_t N, typename T = float>
    constexpr inline Vector<N, T> rejectFrom(const Vector<N, T> &v, const Vector<N, T> &u) {
        return v - projectOnto(v, u);
    }

/**
 *  @brief Compute reflection of a vector across a plane normal.
 *  @param v Vector to reflect.
 *  @param n Unit plane normal.
 *  @return Reflected vector v relative to the plane of n
 */
template<uint8_t N, typename T = float>
    constexpr inline Vector<N, T> reflect(const Vector<N, T> &v, const Vector<N, T> &n) {
        return v - 2.0f*dot(v, n)*n;
    }

/**
 *  @brief Linear interpolation between two vectors.
 *  @param v Start vector.
 *  @param u End vector.
 *  @param t Interpolation factor (0.0 → v, 1.0 → u).
 *  @return Interpolated vector between v and u.
 */
template<uint8_t N, typename T = float>
constexpr inline Vector<N, T> lerp(const Vector<N, T> &v,const Vector<N, T> &u, float t) {
    return v*(1.0f - t) + u*t;
}

/**
 *  @brief Spherical interpolation between two vectors.
 *  @param v Start vector.
 *  @param u End vector.
 *  @param t Interpolation factor (0.0 → v, 1.0 → u)
 *  @return Spherically interpolated vector between v and u.
 */
template<uint8_t N, typename T = float>
constexpr inline Vector<N, T> slerp(Vector<N, T> v, Vector<N, T> u, float t) {
    v = normalize(v);
    u = normalize(u);

    float dotVU = dot(v, u);
    dotVU = std::clamp(dotVU, -1.0f, 1.0f);

    float theta = static_cast<float>(std::acos(dotVU)) * t;

    Vector<N, T> relative = normalize(u - v*dotVU);

    return v*static_cast<float>(std::cos(theta)) + relative*static_cast<float>(std::sin(theta));
}

// ---------------- Conversions ----------------
/**
 *  @brief Convert a vector to std::array.
 */
template<uint8_t N, typename T = float>
constexpr inline std::array<T, N> toArray(const Vector<N, T> &v) {
    std::array<T, N> arr{};
    for(uint8_t i = 0; i < N; i++) {
        arr[i] = v[i];
    }

    return arr;
}

/**
 *  @brief Construct a vector from std::array.
 */
template<uint8_t N, typename T = float>
constexpr inline Vector<N, T> fromArray(const std::array<T, N> &arr) {
    Vector<N, T> v;
    for(uint8_t i = 0; i < N; i++) {
        v[i] = arr[i];
    }

    return v;
}

} // cobalt::math::linear_algebra