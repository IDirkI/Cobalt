#pragma once

#include <cmath>
#include <algorithm>

#include "vector.hpp"

#include "../matrix/matrix.hpp"

namespace cobalt::math::linear_algebra {

// ---------------- Clamping ----------------
/**
 *  @brief Clamp the elements of a vector between an interval
 *  @param v Vector to clamp.
 *  @param min Lower clamp bound.
 *  @param max Upper clamp bound.
 *  @return Element wise clamped vector v between [min, max]
 */
template<index_t N, typename T = float>
    constexpr Vector<N, T> clamp(const Vector<N, T> &v, T minVal, T maxVal) noexcept {
        Vector<N, T> output;
        for(index_t i = 0; i < N; i++) {
            output[i] = (v[i] > maxVal) ?maxVal :((v[i] < minVal) ?minVal :v[i]);
        }

        return output;
    }

/**
 *  @brief Clamp the elements of a vector between the absolute value of max
 *  @param v Vector to clamp.
 *  @param max Upper clamp bound.
 *  @return Element wise clamped vector v between [-max, max]
 */
template<index_t N, typename T = float>
    constexpr Vector<N, T> clamp(const Vector<N, T> &v, T maxVal) noexcept {
        return clamp(v, -maxVal, maxVal);
    }

// ---------------- Element Wise ----------------
/**
 *  @brief Compute smallest element of a vector
 *  @param v Vector to check.
 *  @return Smallest vector element
 */
template<index_t N, typename T = float>
    constexpr T min(const Vector<N, T> &v) noexcept {
        T output = v[0];

        for(index_t i = 0; i < N; i++) {
            if(v[i] < output ) { output = v[i]; }
        }

        return output;
    }

/**
 *  @brief Compute largest element of a vector
 *  @param v Vector to check.
 *  @return Largest vector element
 */
template<index_t N, typename T = float>
    constexpr T max(const Vector<N, T> &v) noexcept {
        T output = v[0];

        for(index_t i = 0; i < N; i++) {
            if(v[i] > output ) { output = v[i]; }
        }

        return output;
    }

/**
 *  @brief Compute vector's element wise absolute value
 *  @param v Vector to abs.
 *  @return Vector with absolute value of each element of v
 */
template<index_t N, typename T = float>
    constexpr Vector<N, T> abs(const Vector<N, T> &v) {
        Vector<N, T> output;
        for(index_t i = 0; i < N; i++) {
            output[i] = std::abs(v[i]);
        }

        return output;
    }

/**
 *  @brief Compute vector's element wise sign
 *  @param v Vector to sign.
 *  @return Vector with sign of each element of v
 */
template<index_t N, typename T = float>
    constexpr Vector<N, T> sign(const Vector<N, T> &v) {
        Vector<N, T> output;
        for(index_t i = 0; i < N; i++) {
            output[i] = (std::abs(v[i]) < epsilon<T>) ?static_cast<T>(0) :((v[i] > 0) ?static_cast<T>(1) :static_cast<T>(-1));
        }

        return output;
    }

/**
 *  @brief Compute vector's element wise floored value
 *  @param v Vector to floor.
 *  @return Vector with floored value of each element of v
 */
template<index_t N, typename T = float>
    constexpr Vector<N, T> floor(const Vector<N, T> &v) {
        Vector<N, T> output;
        for(index_t i = 0; i < N; i++) {
            output[i] = std::floor(v[i]);
        }
        return output;
    }

/**
 *  @brief Compute vector's element wise ceiled value
 *  @param v Vector to ceil.
 *  @return Vector with ceiled value of each element of v
 */
template<index_t N, typename T = float>
    constexpr Vector<N, T> ceil(const Vector<N, T> &v) {
        Vector<N, T> output;
        for(index_t i = 0; i < N; i++) {
            output[i] = std::ceil(v[i]);
        }
        return output;
    }

/**
 *  @brief Element wise rounding of a vector
 *  @param v Vector to round.
 *  @return Vector with rounded value of each element of v
 */
template<index_t N, typename T = float>
    constexpr Vector<N, T> round(const Vector<N, T> &v) {
        Vector<N, T> output;
        for(index_t i = 0; i < N; i++) {
            output[i] = std::round(v[i]);
        }
        return output;
    }


/**
 *  @brief Element wise minimum of two vectors
 *  @param v First vector.
 *  @param u Second vector.
 *  @return Element wise minimum vector.
 */
template<index_t N, typename T = float>
    constexpr Vector<N, T> minElements(const Vector<N, T> &v, const Vector<N, T> &u) {
        Vector<N, T> output;
        for(index_t i = 0; i < N; i++) {
            output[i] = (v[i] < u[i]) ?v[i] :u[i];
        }
        return output;
    }

/**
 *  @brief Element wise maximum of two vectors
 *  @param v First vector.
 *  @param u Second vector.
 *  @return Element wise maximum vector.
 */
template<index_t N, typename T = float>
    constexpr Vector<N, T> maxElements(const Vector<N, T> &v, const Vector<N, T> &u) {
        Vector<N, T> output;
        for(index_t i = 0; i < N; i++) {
            output[i] = (v[i] > u[i]) ?v[i] :u[i];
        }
        return output;
    }

/**
 *  @brief Compute index of the minimum element
 *  @param v Vector to check.
 *  @return Index of smallest vector element
 */
template<index_t N, typename T = float>
    constexpr index_t argmin(const Vector<N, T> &v) {
        index_t index = 0;
        T minVal = v[0];
        for(index_t i = 1; i < N; i++) {
            if(v[i] < minVal) {
                minVal = v[i];
                index = i;
            }
        }
        return index;
    }

/**
 *  @brief Compute index of the maximum element
 *  @param v Vector to check.
 *  @return Index of largest vector element
 */
template<index_t N, typename T = float>
    constexpr index_t argmax(const Vector<N, T> &v) {
        index_t index = 0;
        T maxVal = v[0];
        for(index_t i = 1; i < N; i++) {
            if(v[i] > maxVal) {
                maxVal = v[i];
                index = i;
            }
        }
        return index;
    }

/**
 *  @brief Project vector v onto plane defined by normal n.
 *  @param v Vector to project.
 *  @param n Plane normal.
 *  @return Projected vector of v onto the plane defined by n.
 *  @note n should be a unit vector.
 */
template<index_t N, typename T = float>
    constexpr Vector<N, T> projectPlane(const Vector<N, T> &v, const Vector<N, T> &n) {
        return v - dot(v, n)*n;
    }

/**
 *  @brief Compute orthogonal component of v relative to u.
 *  @param v Vector to compute orthogonal component of.
 *  @param u Vector to compute orthogonal component relative to.
 *  @return Orthogonal component of v relative to u.
 *  @note If u is the zero vector, returns v.
 */
template<index_t N, typename T = float>
    constexpr Vector<N, T> ortho(const Vector<N, T> &v, const Vector<N, T> &u) {
        return v - project(v, u);
    }

/**
 *  @brief Reflect vector v about normal n.
 *  @param v Vector to reflect.
 *  @param n Normal to reflect about.
 *  @return Reflected vector.
 *  @note n should be a unit vector.
 */
template<index_t N, typename T = float>
    constexpr Vector<N, T> reflect(const Vector<N, T> &v, const Vector<N, T> &n) noexcept {
        return v - static_cast<T>(2)*dot(v, n)*n;
    }

/**
 *  @brief Compute sum of all elements of a vector.
 *  @param v Vector to sum across.
 *  @return Sum of all elements of v
 */
template<index_t N, typename T = float>
    constexpr T sum(const Vector<N, T> &v) noexcept {
        T output = static_cast<T>(0);
        for(index_t i = 0; i < N; i++) {
            output += v[i];
        } 
        return output;
    }

/**
 *  @brief Compute product of all elements of a vector.
 *  @param v Vector to multiply across.
 *  @return Product of all elements of v
 */
template<index_t N, typename T = float>
    constexpr T product(const Vector<N, T> &v) noexcept {
        T output = static_cast<T>(1);
        for(index_t i = 0; i < N; i++) {
            output *= v[i];
        } 
        return output;
    }

/**
 *  @brief Compute mean/average of all elements of a vector.
 *  @param v Vector to average across.
 *  @return Mean/average of all elements of v
 */
template<index_t N, typename T = float>
    constexpr T mean(const Vector<N, T> &v) noexcept {
        return sum(v)/(static_cast<T>(N));
    }

/**
 *  @brief Compute variance in all elements of a vector.
 *  @param v Vector to compute variance of.
 *  @return Variance of all elements of v
 *  @note Uses population variance.
 */
template<index_t N, typename T = float>
    constexpr T variance(const Vector<N, T> &v) noexcept {
        T sqrSum = static_cast<T>(0);
        T m = mean(v);
        for(index_t i = 0; i < N; i++) {
            T diff = (v[i] - m);
            sqrSum += diff * diff;
        }
        return sqrSum/(static_cast<T>(N));
    }

/**
 *  @brief Compute standard deviation in all elements of a vector.
 *  @param v Vector to compute standard deviation of.
 *  @return Standard deviation of all elements of v
 *  @note Uses population standard.
 */
template<index_t N, typename T = float>
    constexpr T stdDev(const Vector<N, T> &v) noexcept {
        return std::sqrt(variance(v));
    }

/**
 *  @brief Compute covariance between two vectors.
 *  @param v First vector.
 *  @param u Second vector.
 *  @return Covariance between v and u
 *  @note Uses population covariance.
 */
template<index_t N, typename T = float>
    constexpr T covariance(const Vector<N, T> &v, const Vector<N, T> &u) noexcept {
        T sum = static_cast<T>(0);
        T vm = mean(v);
        T um = mean(u);
        for(index_t i = 0; i < N; i++) {
            sum += (v[i] - vm)*(u[i] - um);
        }
        return sum/(static_cast<T>(N));
    }

/**
 *  @brief Sets the components of a vector to a clean zero if they are very close to zero
 *  @param v Vector to clean
 */
template<index_t N, typename T = float>
    constexpr Vector<N, T> cleanZero(const Vector<N, T> &v) {
        Vector<N, T> output = Vector<N, T>::zero();
        
        for(index_t i = 0; i < N; i++) {
            output[i] = (std::abs(v[i]) < epsilon<T>) ?static_cast<T>(0) :v[i];
        }

        return output;
    }

// ---------------- Interpolation ----------------
/**
 *  @brief Linear interpolation between two vectors.
 *  @param v Start vector.
 *  @param u End vector.
 *  @param t Interpolation factor (0.0 → v, 1.0 → u)
 *  @return Linearly interpolated vector between v and u.
 *  @note t is not clamped between 0.0 and 1.0.
 */
template<index_t N, typename T = float>
constexpr Vector<N, T> lerp(const Vector<N, T> &v, const Vector<N, T> &u, T t) {
    return v*(1.0f - t) + u*t;
}

/**
 *  @brief Spherical linear interpolation between two vectors.
 *  @param v Start vector.
 *  @param u End vector.
 *  @param t Interpolation factor (0.0 → v, 1.0 → u)
 *  @return Spherically interpolated vector between v and u.
 *  @note Both input vectors are normalized before interpolation.
 */
template<index_t N, typename T = float>
constexpr Vector<N, T> slerp(Vector<N, T> v, Vector<N, T> u, T t) {
    v = normalize(v);
    u = normalize(u);

    T dotVU = dot(v, u);
    dotVU = std::clamp(dotVU, -1.0f, 1.0f);

    T theta = std::acos(dotVU) * t;

    Vector<N, T> relative = normalize(u - v*dotVU);

    return v*std::cos(theta) + relative*std::sin(theta);
}

// ---------------- Conversions ----------------
/**
 *  @brief Convert a vector to a std::array.
 *  @param v Vector to convert.
 *  @return std::array containing the vector elements.
 */
template<index_t N, typename T = float>
constexpr std::array<T, N> toArray(const Vector<N, T> &v) noexcept {
    std::array<T, N> arr{};
    for(index_t i = 0; i < N; i++) {
        arr[i] = v[i];
    }

    return arr;
}

/**
 *  @brief Convert a 3D vector to a skew-symmetric matrix.
 *  @param v 3D vector to convert.
 *  @return 3x3 skew-symmetric matrix corresponding to v.
 *  @note Only defined for 3D vectors.
 */
template<typename T = float>
constexpr Matrix<3, 3, T> toSkew(const Vector<3, T> &v) noexcept {
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
 *  @return `true` if the vector is normalized, `false` otherwise.
 */
template<index_t N, typename T = float>
constexpr bool isNormalized(const Vector<N, T> &v) noexcept {
    return (std::abs(norm(v) - static_cast<T>(1)) < epsilon<T>);
}

/**
 *  @brief Check if a vector is the zero vector
 *  @return `true` if the vector is the zero vector, `false` otherwise.
 */
template<index_t N, typename T = float>
constexpr bool isZero(const Vector<N, T> &v) noexcept {
    return (normSqr(v) < (epsilon<T> * epsilon<T>));
}

/**
 *  @brief Check if two vectors are parallel
 *  @return `true` if the vectors are parallel, `false` otherwise.
 */
template<index_t N, typename T = float>
constexpr bool isParallel(const Vector<N, T> &v, const Vector<N, T> &u) noexcept {
    Vector<N, T> vn = normalize(v);
    Vector<N, T> un = normalize(u);
    T prod = std::abs(dot(vn, un));
    return (std::abs(prod - static_cast<T>(1)) < epsilon<T>);
}

/**
 *  @brief Check if two vectors are orthogonal
 *  @return `true` if the vectors are orthogonal, `false` otherwise.
 */
template<index_t N, typename T = float>
constexpr bool isOrthogonal(const Vector<N, T> &v, const Vector<N, T> &u) noexcept {
    return (std::abs(dot(v, u)) < epsilon<T>);
}

/**
 *  @brief Check if all elements of a vector are finite numbers
 *  @return `true` if all elements are finite, `false` otherwise.
 */
template<index_t N, typename T = float>
constexpr bool isFinite(const Vector<N, T> &v) noexcept {
    for(index_t i = 0; i < N; i++) {
        if(!std::isfinite(v[i])) return false;
    }
    return true;
}


} // cobalt::math::linear_algebra