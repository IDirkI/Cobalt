#pragma once

#define _USE_MATH_DEFINES 
#include <cmath>

#include "complex.hpp"

namespace cobalt::math::algebra {
// ---------------- Non-member Overloads ----------------

inline Complex operator+(Complex lhs, const Complex &rhs) noexcept { lhs += rhs; return lhs; }
inline Complex operator+(Complex lhs, float c) noexcept { lhs += c; return lhs; }
inline Complex operator+(float c, Complex lhs) noexcept { lhs += c; return lhs; }

inline Complex operator-(Complex lhs, const Complex &rhs) noexcept { lhs -= rhs; return lhs; }
inline Complex operator-(Complex lhs, float c) noexcept { lhs -= c; return lhs; }
inline Complex operator-(float c, Complex lhs) noexcept { return Complex(c) - lhs; }

inline Complex operator*(Complex lhs, const Complex &rhs) noexcept { lhs *= rhs; return lhs; }
inline Complex operator*(Complex lhs, float c) noexcept { lhs *= c; return lhs; }
inline Complex operator*(float c, Complex lhs) noexcept { lhs *= c; return lhs; }

inline Complex operator/(Complex lhs, const Complex &rhs) { lhs /= rhs; return lhs; }
inline Complex operator/(Complex lhs, float c) { lhs /= c; return lhs; }
inline Complex operator/(float c, Complex rhs) { 
    float denom = rhs.real()*rhs.real() + rhs.imag()*rhs.imag();
    return Complex((c*rhs.real())/denom, (-c*rhs.imag())/denom);
}

inline Complex operator-(Complex z) noexcept { z *= -1; return z; }

inline bool operator==(Complex lhs, const Complex &rhs) noexcept { 
    if(std::abs(lhs.real() - rhs.real()) > epsilon<>) { return false; }
    if(std::abs(lhs.imag() - rhs.imag()) > epsilon<>) { return false; }
    return true;
}
inline bool operator==(Complex lhs, float c) noexcept { return (lhs == Complex(c)); }
inline bool operator==(float c, Complex lhs) noexcept { return (lhs == c); }

inline bool operator!=(Complex lhs, const Complex &rhs) noexcept { return !(lhs == rhs); }
inline bool operator!=(Complex lhs, float c) noexcept { return !(lhs == c); }
inline bool operator!=(float c, Complex lhs) noexcept { return (lhs != c); }


// ---------------- Non-member Functions ----------------
/**
 *  @brief Get the norm/absolute value of a complex number
 *  @return `|z|` The norm of the complex number
 */
constexpr inline float norm(const Complex &z) noexcept { return std::sqrt(z.real()*z.real() + z.imag()*z.imag()); }

/**
 *  @brief Get the square of the norm/absolute value of a complex number
 *  @return `|z|²` The squared norm of the complex number
 */
constexpr inline float normSqr(const Complex &z) noexcept { return (z.real()*z.real() + z.imag()*z.imag()); }

/**
 *  @brief Get the argument/angle of a complex number
 *  @return `∠z` The argument of the complex number
 */
constexpr inline float arg(const Complex &z) noexcept { return std::atan2(z.imag(), z.real()); }

/**
 *  @brief Get the conjugate of a complex number
 *  @return `z̄` Conjugate of the compex number
 */
inline Complex conj(const Complex &z) noexcept { return Complex(z.real(), -z.imag()); }

/**
 *  @brief Get the multiplicative inverse of the complex number
 *  @return `1/z` The inverse of the complex number
 */
inline Complex inv(const Complex &z) { 
    float d = normSqr(z);
    return Complex(z.real()/d, -z.imag()/d);
 }

 /**
 *  @brief Get the exponential power of the complex number to e
 *  @return `eᶻ` The exponentited complex number
 */
inline Complex exp(const Complex &z) {
    return Complex::polar(std::pow(M_E, z.real()), z.imag());
 }

  /**
 *  @brief Get the natural logarithm of the complex number  
 *  @return `ln(z)` The natural log of the complex number
 */
inline Complex log(const Complex &z) {
    return Complex(std::log(norm(z)), arg(z));
}


  /**
 *  @brief Get the n-th power of the complex number
 *  @return `zⁿ` The n-th power of the complex number
 */
inline Complex pow(const Complex &z, float n) {
    return Complex::polar(std::pow(norm(z), n), arg(z)*n);
}

  /**
 *  @brief Get the square root of the complex number
 *  @return `√z` The sqrt of the complex number
 */
inline Complex sqrt(const Complex &z) {
    return pow(z, 0.5f);
}

} // cobalt::math::algebra
