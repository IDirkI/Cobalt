#pragma once

#include <cmath>
#include <algorithm>

#include "vector.hpp"

#include "../matrix/matrix.hpp"

namespace cobalt::math::linear_algebra {

// ---------------- Non-member Utility ----------------
/**
 *  @brief Compute smallest element of a vector
 *  @param v Vector to check.
 *  @return Smallest vector element
 */
template<uint8_t N, typename T = float>
    constexpr T min(const Vector<N, T> &v) {
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
    constexpr T max(const Vector<N, T> &v) {
        T output = v[0];

        for(uint8_t i = 0; i < N; i++) {
            if(v[i] > output ) { output = v[i]; }
        }

        return output;
    }

/**
 *  @brief Compute element wise absolute value on vector
 *  @param v Vector to absolute value.
 *  @return Vector with absolute valued elements
 */
template<uint8_t N, typename T = float>
    constexpr Vector<N, T> abs(const Vector<N, T> &v) {
        Vector<N, T> output;
        for(uint8_t i = 0; i < N; i++) {
            output[i] = (v[i] >= static_cast<T>(0.0f)) ?v[i] :-v[i];
        }

        return output;
    }

/**
 *  @brief Compute vector's element wise sign
 *  @param v Vector to sign check.
 *  @return Vector with sign of each element of v
 */
template<uint8_t N, typename T = float>
    constexpr Vector<N, T> sign(const Vector<N, T> &v) {
        Vector<N, T> output;
        for(uint8_t i = 0; i < N; i++) {
            if(v[i] > static_cast<T>(0.0f))         { output[i] = static_cast<T>(1); }
            else if(v[i] < static_cast<T>(0.0f))    { output[i] = static_cast<T>(-1); }
            else                                    { output[i] = static_cast<T>(0); }
        }

        return output;
    }

/**
 *  @brief Clamp the elements of a vector between an interval
 *  @param v Vector to project.
 *  @param min Lower clamp bound.
 *  @param max Upper clamp bound.
 *  @return Element wise clamped vector v between [min, max]
 */
template<uint8_t N, typename T = float>
    constexpr Vector<N, T> clamp(const Vector<N, T> &v, T minVal, T maxVal) {
        Vector<N, T> output;
        for(uint8_t i = 0; i < N; i++) {
            if(v[i] > maxVal)       { output[i] = maxVal; }
            else if(v[i] < minVal)  { output[i] = minVal; }
            else                    { output[i] = v[i]; }
        }

        return output;
    }

/**
 *  @brief Element wise flooring of a vector
 */
template<uint8_t N, typename T = float>
    constexpr Vector<N, T> floor(const Vector<N, T> &v) {
        Vector<N, T> res;
        for(uint8_t i = 0; i < N; i++) {
            res[i] = std::floor(v[i]);
        }
        return res;
    }

/**
 *  @brief Element wise ceiling of a vector
 */
template<uint8_t N, typename T = float>
    constexpr Vector<N, T> ceil(const Vector<N, T> &v) {
        Vector<N, T> res;
        for(uint8_t i = 0; i < N; i++) {
            res[i] = std::ceil(v[i]);
        }
        return res;
    }

/**
 *  @brief Element wise rounding of a vector
 */
template<uint8_t N, typename T = float>
    constexpr Vector<N, T> round(const Vector<N, T> &v) {
        Vector<N, T> res;
        for(uint8_t i = 0; i < N; i++) {
            res[i] = std::round(v[i]);
        }
        return res;
    }


/**
 *  @brief Element wise minimum of two vectors
 */
template<uint8_t N, typename T = float>
    constexpr Vector<N, T> minElements(const Vector<N, T> &v, const Vector<N, T> &u) {
        Vector<N, T> res;
        for(uint8_t i = 0; i < N; i++) {
            res[i] = (v[i] < u[i]) ?v[i] :u[i];
        }
        return res;
    }

/**
 *  @brief Element wise maximum of two vectors
 */
template<uint8_t N, typename T = float>
    constexpr Vector<N, T> maxElements(const Vector<N, T> &v, const Vector<N, T> &u) {
        Vector<N, T> res;
        for(uint8_t i = 0; i < N; i++) {
            res[i] = (v[i] > u[i]) ?v[i] :u[i];
        }
        return res;
    }

/**
 *  @brief Compute index of the minimum element
 */
template<uint8_t N, typename T = float>
    constexpr uint8_t argmin(const Vector<N, T> &v) {
        uint8_t index = 0;
        T minVal = v[0];
        for(uint8_t i = 1; i < N; i++) {
            if(v[i] < minVal) {
                minVal = v[i];
                index = i;
            }
        }
        return index;
    }

/**
 *  @brief Compute index of the maximum element
 */
template<uint8_t N, typename T = float>
    constexpr uint8_t argmax(const Vector<N, T> &v) {
        uint8_t index = 0;
        T maxVal = v[0];
        for(uint8_t i = 1; i < N; i++) {
            if(v[i] > maxVal) {
                maxVal = v[i];
                index = i;
            }
        }
        return index;
    }

/**
 *  @brief Compute projection of a 3D-vector on to a plane (plane normal).
 *  @param v Vector to project.
 *  @param n Unit plane normal.
 *  @return Vector v projected onto the plane of n.
 */
template<uint8_t N, typename T = float>
    constexpr Vector<N, T> projectPlane(const Vector<N, T> &v, const Vector<N, T> &n) {
        return v - dot(v, n)*n;
    }

/**
 *  @brief Compute component of one vector orthogonal to another.
 *  @param v Vector to reject.
 *  @param u Vector to reject from.
 *  @return Orthogonal component of v to u.
 */
template<uint8_t N, typename T = float>
    constexpr Vector<N, T> ortho(const Vector<N, T> &v, const Vector<N, T> &u) {
        return v - project(v, u);
    }

/**
 *  @brief Compute reflection of a vector across a plane normal.
 *  @param v Vector to reflect.
 *  @param n Unit plane normal.
 *  @return Reflected vector v relative to the plane of n
 */
template<uint8_t N, typename T = float>
    constexpr Vector<N, T> reflect(const Vector<N, T> &v, const Vector<N, T> &n) {
        return v - static_cast<T>(2.0f)*dot(v, n)*n;
    }

/**
 *  @brief Compute sum of all elements of a vector.
 *  @param v Vector to sum across.
 */
template<uint8_t N, typename T = float>
    constexpr T sum(const Vector<N, T> &v) {
        T output = static_cast<T>(0);
        for(uint8_t i = 0; i < N; i++) {
            output += v[i];
        } 
        return output;
    }

/**
 *  @brief Compute product of all elements of a vector.
 *  @param v Vector to multiply across.
 */
template<uint8_t N, typename T = float>
    constexpr T product(const Vector<N, T> &v) {
        T output = static_cast<T>(1);
        for(uint8_t i = 0; i < N; i++) {
            output *= v[i];
        } 
        return output;
    }

/**
 *  @brief Compute mean of all elements of a vector.
 */
template<uint8_t N, typename T = float>
    constexpr T mean(const Vector<N, T> &v) {
        return sum(v)/(static_cast<T>(N));
    }

/**
 *  @brief Compute variance in all elements of a vector.
 */
template<uint8_t N, typename T = float>
    constexpr T variance(const Vector<N, T> &v) {
        T sqrSum = static_cast<T>(0);
        T m = mean(v);
        for(uint8_t i = 0; i < N; i++) {
            T diff = (v[i] - m);
            sqrSum += diff * diff;
        }
        return sqrSum/(static_cast<T>(N));
    }

/**
 *  @brief Compute the standard deviation in all elements of a vector.
 */
template<uint8_t N, typename T = float>
    constexpr T stdDev(const Vector<N, T> &v) {
        return std::sqrt(variance(v));
    }

/**
 *  @brief Compute covariance between two vectors.
 */
template<uint8_t N, typename T = float>
    constexpr T covariance(const Vector<N, T> &v, const Vector<N, T> &u) {
        T sum = static_cast<T>(0);
        T vm = mean(v);
        T um = mean(u);
        for(uint8_t i = 0; i < N; i++) {
            sqrSum += (v[i] - vm)*(u[i] - um)
        }
        return sqrSum/(static_cast<T>(N));
    }

/**
 *  @brief Linear interpolation between two vectors.
 *  @param v Start vector.
 *  @param u End vector.
 *  @param t Interpolation factor
 *  @return Interpolated vector between v and u.
 */
template<uint8_t N, typename T = float>
constexpr Vector<N, T> lerp(const Vector<N, T> &v, const Vector<N, T> &u, T t) {
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
constexpr Vector<N, T> slerp(Vector<N, T> v, Vector<N, T> u, T t) {
    v = normalize(v);
    u = normalize(u);

    float dotVU = dot(v, u);
    dotVU = std::clamp(dotVU, -1.0f, 1.0f);

    float theta = std::acos(dotVU) * t;

    Vector<N, T> relative = normalize(u - v*dotVU);

    return v*std::cos(theta) + relative*std::sin(theta);
}

// ---------------- Conversions ----------------
/**
 *  @brief Convert a vector to std::array.
 */
template<uint8_t N, typename T = float>
constexpr std::array<T, N> toArray(const Vector<N, T> &v) {
    std::array<T, N> arr{};
    for(uint8_t i = 0; i < N; i++) {
        arr[i] = v[i];
    }

    return arr;
}

/**
 *  @brief Construct a skew-symmetric matrix(3x3)from a vector(3).
 *  @param v Vector to turn into a skew-symmetric matrix.
 */
template<typename T = float>
constexpr Matrix<3, 3, T> toSkew(const Vector<3, T> &v) {
    Matrix<3, 3, T> output = Matrix<3, 3, T>::zero();

    output(0, 1) = -v.z();
    output(0, 2) =  v.y();
    output(1, 2) = -v.x();

    output(1, 0) =  v.z();
    output(2, 0) = -v.y();
    output(2, 1) =  v.x();

    return output;
}

// ---------------- Checks ----------------
/**
 *  @brief Check if a vector is normalized
 */
template<uint8_t N, typename T = float>
constexpr bool isNormalized(const Vector<N, T> &v) {
    return (std::abs(norm(v) - static_cast<T>(1.0f)) < VECTOR_EQUAL_THRESHOLD);
}

/**
 *  @brief Check if a vector is zero
 */
template<uint8_t N, typename T = float>
constexpr bool isZero(const Vector<N, T> &v) {
    return (normSqr(v) < (VECTOR_EQUAL_THRESHOLD * VECTOR_EQUAL_THRESHOLD));
}

/**
 *  @brief Check if two vectors are parallel
 */
template<uint8_t N, typename T = float>
constexpr bool isParallel(const Vector<N, T> &v, const Vector<N, T> &u) {
    Vector<N, T> vn = normalize(v);
    Vector<N, T> un = normalize(u);
    T prod = std::abs(dot(vn, un));
    return (std::abs(prod - static_cast<T>(1.0f)) < VECTOR_EQUAL_THRESHOLD);
}

/**
 *  @brief Check if two vectors are orthogonal
 */
template<uint8_t N, typename T = float>
constexpr bool isOrthogonal(const Vector<N, T> &v, const Vector<N, T> &u) {
    return (std::abs(dot(v, u)) < VECTOR_EQUAL_THRESHOLD);
}

/**
 *  @brief Check if a vector's elements are finite numebrs
 */
template<uint8_t N, typename T = float>
constexpr bool isFinite(const Vector<N, T> &v) {
    for(uint8_t i = 0; i < N; i++) {
        if(!std::isfinite(v[i])) return false;
    }
    return true;
}


} // cobalt::math::linear_algebra