#pragma once

#include <cmath>
#include <algorithm>

#include "vector.hpp"

namespace cobalt::math::linear_algebra {

// ---------------- Non-member Arithmetic Overloads ----------------
/**
 *  @brief Vector addition
 */
template<index_t N, typename T = float, typename = std::enable_if_t<Scalar<T>>, typename = std::enable_if_t<Scalar<T>>>
    constexpr Vector<N, T> operator+(Vector<N, T> lhs, const Vector<N, T> &rhs) noexcept { lhs += rhs; return lhs; }

/**
 *  @brief Vector subtraction
 */
template<index_t N, typename T = float, typename = std::enable_if_t<Scalar<T>>, typename = std::enable_if_t<Scalar<T>>>
    constexpr Vector<N, T> operator-(Vector<N, T> lhs, const Vector<N, T> &rhs) noexcept { lhs -= rhs; return lhs; }

/**
 *  @brief Vector subtraction
 */
template<index_t N, typename T = float, typename = std::enable_if_t<Scalar<T>>, typename = std::enable_if_t<Scalar<T>>>
    constexpr Vector<N, T> operator*(Vector<N, T> v, T c) { v *= c; return v; }

/**
 *  @brief Vector scalar multiplication
 */
template<index_t N, typename T = float, typename = std::enable_if_t<Scalar<T>>, typename = std::enable_if_t<Scalar<T>>>
    constexpr Vector<N, T> operator*(T c, Vector<N, T> v) { return v * c; }

/**
 *  @brief Vector scalar divison
 */
template<index_t N, typename T = float, typename = std::enable_if_t<Scalar<T>>, typename = std::enable_if_t<Scalar<T>>>
    constexpr Vector<N, T> operator/(Vector<N, T> v, T c) { v /= c; return v; }

/**
 *  @brief Flip the vector. Element wise negation
 */
template<index_t N, typename T = float, typename = std::enable_if_t<Scalar<T>>, typename = std::enable_if_t<Scalar<T>>>
    constexpr Vector<N, T> operator-(Vector<N, T> v) noexcept { v *= -1; return v; }

/**
 *  @brief Check vector equality within a threshold (default 1e-5)
 */
template<index_t N, typename T = float, typename = std::enable_if_t<Scalar<T>>, typename = std::enable_if_t<Scalar<T>>>
    constexpr bool operator==(const Vector<N, T> &lhs, const Vector<N, T> &rhs) noexcept {
        for(index_t i = 0; i < N; i++) {
            if(std::abs(lhs[i] - rhs[i]) > epsilon<T>) return false;
        }
        return true;
    }

/**
 *  @brief Check vector non-equality within a threshold (default 1e-5)
 */
template<index_t N, typename T = float, typename = std::enable_if_t<Scalar<T>>, typename = std::enable_if_t<Scalar<T>>>
    constexpr bool operator!=(const Vector<N, T> &lhs, const Vector<N, T> &rhs) noexcept { return !(lhs == rhs); }


// ---------------- Non-member Functions ----------------
/**
 *  @brief Dot product between two vectors
 *  @return Scalar dot product
 */
template<index_t N, typename T = float, typename = std::enable_if_t<Scalar<T>>, typename = std::enable_if_t<Scalar<T>>>  
    constexpr T dot(const Vector<N, T> &v, const Vector<N, T> &u) noexcept { 
        T output = static_cast<T>(0);
        for(index_t i = 0; i < N; i++) {
            output += v[i] * u[i];
        }
        return output;
    }

/**
 *  @brief Cross product between two 3-vectors
 *  @return Vector cross product
 */
template<typename T = float, typename = std::enable_if_t<Scalar<T>>, typename = std::enable_if_t<Scalar<T>>>
    constexpr Vector<3, T> cross(const Vector<3, T> &v, const Vector<3, T> &u) noexcept { 
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
template<index_t N, typename T = float, typename = std::enable_if_t<Scalar<T>>, typename = std::enable_if_t<Scalar<T>>>
    constexpr Vector<N, T> hadamard(const Vector<N, T> &v, const Vector<N, T> &u) noexcept { 
        Vector<N, T> output{};
        for(index_t i = 0; i < N; i++) {
            output[i] = v[i]*u[i];
        }
        return output;
    }

/**
 *  @brief Triple product between three 3-vectors
 *  @return Scalar triple product (dot(v, cross(u, w)))
 */
template<typename T = float, typename = std::enable_if_t<Scalar<T>>>
    constexpr T tripleProduct(const Vector<3, T> &v, const Vector<3, T> &u, const Vector<3, T> &w) noexcept { 
        return dot(v, cross(u, w));
    }


/**
 *  @brief Compute vector norm/magnitude
 *  @return Norm/magnitude of vector
 */
template<index_t N, typename T = float, typename = std::enable_if_t<Scalar<T>>>
    constexpr T norm(const Vector<N, T> &v) noexcept { return std::sqrt(dot(v, v)); }

/**
 *  @brief Compute squared vector norm/magnitude
 *  @return Squared norm/magnitude of vector
 */
template<index_t N, typename T = float, typename = std::enable_if_t<Scalar<T>>>
    constexpr T normSqr(const Vector<N, T> &v) noexcept { return dot(v, v); }

/**
 *  @brief Normalize a vector to unit length
 *  @return Normalized vector
 */
template<index_t N, typename T = float, typename = std::enable_if_t<Scalar<T>>>
    constexpr Vector<N, T> normalize(const Vector<N, T> &v) {
        T mag = norm(v);
        Vector<N, T> output = v;
        output = (mag > epsilon<T>) ?(output /= mag) :(Vector<N, T>::zero());
        return output;
    }

/**
 *  @brief Compute the distance between two vectors
 *  @return Scalar distance between the tips of the vectors
 */
template<index_t N, typename T = float, typename = std::enable_if_t<Scalar<T>>>
    constexpr T distance(const Vector<N, T> &v, const Vector<N, T> &u) noexcept  { return norm(v - u); }

/**
 *  @brief Compute the squared distance between two vectors
 *  @return Squared distance between the tips of the vectors
 */
template<index_t N, typename T = float, typename = std::enable_if_t<Scalar<T>>>
    constexpr T distanceSqr(const Vector<N, T> &v, const Vector<N, T> &u) noexcept { return dot(v - u, v - u); }

/**
 *  @brief Compute the angle between two vectors in radians
 *  @return Angle between the vectors in radians
 */
template<index_t N, typename T = float, typename = std::enable_if_t<Scalar<T>>>
    constexpr T angle(const Vector<N, T> &v, const Vector<N, T> &u) { 
        T nv = norm(v);
        T nu = norm(u);

        if(nv == static_cast<T>(0) || nu == static_cast<T>(0)) return static_cast<T>(0);

        T cosang = dot(v, u)/(nv*nu);
        cosang = std::clamp(cosang, static_cast<T>(-1), static_cast<T>(1));
        return std::acos(cosang);
    }

/**
 *  @brief Project vector v onto vector u
 *  @return Projected vector
 *  @note If u is the zero vector, returns the zero vector
 */
template<index_t N, typename T = float, typename = std::enable_if_t<Scalar<T>>>
    constexpr Vector<N, T> project(const Vector<N, T> &v, const Vector<N, T> &u) {
        T denom = dot(u, u);
        if(std::abs(denom) < epsilon<T>) { return Vector<N, T>::zero(); }

        T scale = dot(v, u) / denom;
        return u * scale;
    }

} // cobalt::math::linear_algebra