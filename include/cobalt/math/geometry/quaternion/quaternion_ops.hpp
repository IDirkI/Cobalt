#pragma once

#include <cmath>

#include "quaternion.hpp"

#include "../../algebra/complex/complex.hpp"

namespace cobalt::math::geometry {

// ---------------- Non-member Overloads ----------------
template<typename T = def_floating, typename = std::enable_if_t<Floating<T>>>
    inline Quaternion<T> operator+(Quaternion<T> lhs, const Quaternion<T> &rhs) noexcept { lhs += rhs; return lhs; }
template<typename T = def_floating, typename = std::enable_if_t<Floating<T>>>
    inline Quaternion<T> operator+(Quaternion<T> lhs, T c) noexcept { lhs += Quaternion<T>(c); return lhs; }
template<typename T = def_floating, typename = std::enable_if_t<Floating<T>>>
    inline Quaternion<T> operator+(T c, Quaternion<T> rhs) noexcept { return (rhs + c); }

template<typename T = def_floating, typename = std::enable_if_t<Floating<T>>>
    inline Quaternion<T> operator-(Quaternion<T> lhs, const Quaternion<T> &rhs) noexcept { lhs -= rhs; return lhs; }
template<typename T = def_floating, typename = std::enable_if_t<Floating<T>>>
    inline Quaternion<T> operator-(Quaternion<T> lhs, T c) noexcept { lhs -= Quaternion<T>(c); return lhs; }
template<typename T = def_floating, typename = std::enable_if_t<Floating<T>>>
    inline Quaternion<T> operator-(T c, const Quaternion<T> &rhs) noexcept { return (Quaternion<T>(c) - rhs); }

template<typename T = def_floating, typename = std::enable_if_t<Floating<T>>>
    inline Quaternion<T> operator*(Quaternion<T> lhs, const Quaternion<T> &rhs) noexcept { lhs *= rhs; return lhs; }
template<typename T = def_floating, typename = std::enable_if_t<Floating<T>>>
    inline Quaternion<T> operator*(Quaternion<T> lhs, T c) noexcept { lhs *= c; return lhs; }
template<typename T = def_floating, typename = std::enable_if_t<Floating<T>>>
    inline Quaternion<T> operator*(T c, Quaternion<T> rhs) noexcept { rhs *= c; return rhs; }
template<typename T = def_floating, typename = std::enable_if_t<Floating<T>>>
    inline cobalt::math::linear_algebra::Vector<3, T> operator*(const Quaternion<T> &rhs, cobalt::math::linear_algebra::Vector<3, T> v) noexcept { return rotate(rhs, v); }

template<typename T = def_floating, typename = std::enable_if_t<Floating<T>>>
    inline Quaternion<T> operator/(Quaternion<T> lhs, const Quaternion<T> &rhs) noexcept { lhs /= rhs; return lhs; }
template<typename T = def_floating, typename = std::enable_if_t<Floating<T>>>
    inline Quaternion<T> operator/(Quaternion<T> lhs, T c) noexcept { lhs /= c; return lhs; }
template<typename T = def_floating, typename = std::enable_if_t<Floating<T>>>
    inline Quaternion<T> operator/(T c, const Quaternion<T> &rhs) noexcept { return (Quaternion<T>(c) / rhs); }

template<typename T = def_floating, typename = std::enable_if_t<Floating<T>>>
    inline Quaternion<T> operator-(Quaternion<T> q) noexcept { q *= -1; return q; } 

template<typename T = def_floating, typename = std::enable_if_t<Floating<T>>>
    inline bool operator==(const Quaternion<T> &lhs, const Quaternion<T> &rhs) noexcept { 
        if(std::abs(lhs.w() - rhs.w()) > epsilon_<T>) { return false; }
        if(std::abs(lhs.x() - rhs.x()) > epsilon_<T>) { return false; }
        if(std::abs(lhs.y() - rhs.y()) > epsilon_<T>) { return false; }
        if(std::abs(lhs.z() - rhs.z()) > epsilon_<T>) { return false; }
        return true;
    }

template<typename T = def_floating, typename = std::enable_if_t<Floating<T>>>
    inline bool operator!=(const Quaternion<T> &lhs, const Quaternion<T> &rhs) noexcept { return !(lhs == rhs); }

// ---------------- Non-member Functions ----------------
/**
 * @brief Norm of the quaternion
 * @return |q|
 */
template<typename T = def_floating, typename = std::enable_if_t<Floating<T>>>
    constexpr T norm(const Quaternion<T> &q) noexcept {
        return std::sqrt(q.w()*q.w() + q.x()*q.x() + q.y()*q.y() + q.z()*q.z());
    }

/**
 * @brief Squared norm of the quaternion
 * @return |q|^2
 */
template<typename T = def_floating, typename = std::enable_if_t<Floating<T>>>
    constexpr T normSqr(const Quaternion<T> &q) noexcept {
        return q.w()*q.w() + q.x()*q.x() + q.y()*q.y() + q.z()*q.z();
    }

/**
 * @brief Conjugate of the quaternion
 * @return q*
 */
template<typename T = def_floating, typename = std::enable_if_t<Floating<T>>>
    inline Quaternion<T> conj(const Quaternion<T> &q) noexcept {
        return Quaternion<T>(q.w(), -q.x(), -q.y(), -q.z());
    }

/**
 * @brief Inverse of the quaternion
 * @return q^-1
 */
template<typename T = def_floating, typename = std::enable_if_t<Floating<T>>>
    inline Quaternion<T> inv(const Quaternion<T> &q) noexcept {
        return conj(q) / normSqr(q);
    }

/**
 * @brief Dot product between two quaternions
 * @return Scalar dot product
 */
template<typename T = def_floating, typename = std::enable_if_t<Floating<T>>>
    constexpr T dot(const Quaternion<T> &q, const Quaternion<T> &p) noexcept {
        return (q.w()*p.w() + q.x()*p.x() + q.y()*p.y() + q.z()*p.z());
    }

/**
 * @brief Normalize the quaternion
 * @return q / |q|
 */
template<typename T = def_floating, typename = std::enable_if_t<Floating<T>>>
    inline Quaternion<T> normalize(const Quaternion<T> &q) noexcept {
        T n = norm(q);
        if (n < epsilon_<T>) {
            return Quaternion<T>::zero();
        }
        return (q / n);
    }

/**
 * @brief Logarithm of the quaternion
 * @return log(q)
 */
template<typename T = def_floating, typename = std::enable_if_t<Floating<T>>>
    inline Quaternion<T> log(const Quaternion<T> &q) noexcept {;
        cobalt::math::linear_algebra::Vector<3, T> v = q.vector();
        T vNorm = norm(v);
        T qNorm = norm(q);

        if(vNorm < epsilon_<T>) {
            return Quaternion<T>(std::log(qNorm), static_cast<T>(0), static_cast<T>(0), static_cast<T>(0));
        }

        T t = std::acos(q.w()/qNorm);
        cobalt::math::linear_algebra::Vector<3, T> vDir = v/vNorm;

        return Quaternion<T>(std::log(qNorm), vDir.x()*t, vDir.y()*t, vDir.z()*t);
    }

/**
 *  @brief Exponential of the quaternion
 *  @return exp(q)
 */
template<typename T = def_floating, typename = std::enable_if_t<Floating<T>>>
    inline Quaternion<T> exp(const Quaternion<T> &q) noexcept {
        cobalt::math::linear_algebra::Vector<3, T> v = q.vector();
        T vNorm = norm(v);
        T expW = std::exp(q.w());

        if(vNorm < epsilon_<T>) {
            return Quaternion<T>(expW, 0.0f, 0.0f, 0.0f);
        }

        cobalt::math::linear_algebra::Vector<3, T> vDir = v/vNorm;
        T cosV = std::cos(vNorm);
        T sinV = std::sin(vNorm);

        return Quaternion<T>(
            expW*cosV,
            expW*vDir.x()*sinV,
            expW*vDir.y()*sinV,
            expW*vDir.z()*sinV
        );
    }

/**
 *  @brief Raise a quaternion to a scalar power
 *  @return q^n
 */
template<typename T = def_floating, typename = std::enable_if_t<Floating<T>>>
    inline Quaternion<T> pow(const Quaternion<T> &q, T n) noexcept {
        return exp(log(q) * n);
    }

/**
 *  @brief Rotate a 3-Vector by a quaternion
 *  @return q * v * q^-1
 *  @note The quaternion is assumed to be normalized
 */
template<typename T = def_floating, typename = std::enable_if_t<Floating<T>>>
    inline cobalt::math::linear_algebra::Vector<3, T> rotate(const Quaternion<T> &q, const cobalt::math::linear_algebra::Vector<3, T> &v) noexcept {
        Quaternion<T> p = Quaternion<T>::fromPureVector(v);
        Quaternion<T> qInv = inv(q);
        Quaternion<T> result = q * p * qInv;

        return result.vector();
    }


} // cobalt::math::geometry