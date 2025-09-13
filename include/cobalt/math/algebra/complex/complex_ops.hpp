#pragma once

#include <cmath>

#include "complex.hpp"

namespace cobalt::math::algebra {
// ---------------- Non-member Overloads ----------------

inline Complex operator+(Complex lhs, const Complex &rhs) { lhs += rhs; return lhs; }
inline Complex operator+(Complex lhs, float c) { lhs += c; return lhs; }
inline Complex operator+(float c, Complex lhs) { lhs += c; return lhs; }

inline Complex operator-(Complex lhs, const Complex &rhs) { lhs -= rhs; return lhs; }
inline Complex operator-(Complex lhs, float c) { lhs -= c; return lhs; }
inline Complex operator-(float c, Complex lhs) { lhs -= c; return lhs; }

inline Complex operator*(Complex lhs, const Complex &rhs) { lhs *= rhs; return lhs; }
inline Complex operator*(Complex lhs, float c) { lhs *= c; return lhs; }
inline Complex operator*(float c, Complex lhs) { lhs *= c; return lhs; }

inline Complex operator/(Complex lhs, const Complex &rhs) { lhs /= rhs; return lhs; }
inline Complex operator/(Complex lhs, float c) { lhs /= c; return lhs; }
inline Complex operator/(float c, Complex lhs) { lhs = (Complex::one() / lhs)*c; return lhs; }

inline bool operator==(Complex lhs, const Complex &rhs) { 
    if(static_cast<float>(lhs.real() - rhs.real()) > COMPLEX_EQUAL_THRESHOLD) { return false; }
    if(static_cast<float>(lhs.real() - rhs.real()) > COMPLEX_EQUAL_THRESHOLD) { return false; }
    return true;
}
inline bool operator==(Complex lhs, float c) { return ((static_cast<float>(lhs.real() - c) < COMPLEX_EQUAL_THRESHOLD) && (lhs.imag() < COMPLEX_ZERO_THRESHOLD)); }
inline bool operator==(float c, Complex lhs) { return (lhs == c); }

inline bool operator!=(Complex lhs, const Complex &rhs) { 
    if(static_cast<float>(lhs.real() - rhs.real()) > COMPLEX_EQUAL_THRESHOLD) { return true; }
    if(static_cast<float>(lhs.real() - rhs.real()) > COMPLEX_EQUAL_THRESHOLD) { return true; }
    return false;
}
inline bool operator!=(Complex lhs, float c) { return ((static_cast<float>(lhs.real() - c) > COMPLEX_EQUAL_THRESHOLD) || (lhs.imag() > COMPLEX_ZERO_THRESHOLD)); }
inline bool operator!=(float c, Complex lhs) { return (lhs != c); }


// ---------------- Non-member Functions ----------------
/**
 *  @brief Get the norm/absolute value of a complex number
 *  @return `|z|` The norm of the complex number
 */
constexpr inline float abs(const Complex &z) { return sqrtf(z.real()*z.real() + z.imag()*z.imag()); }

/**
 *  @brief Get the square of the norm/absolute value of a complex number
 *  @return `|z|²` The squared norm of the complex number
 */
constexpr inline float normSqr(const Complex &z) { return (z.real()*z.real() + z.imag()*z.imag()); }

/**
 *  @brief Get the argument/angle of a complex number
 *  @return `∠z` The argument of the complex number
 */
constexpr inline float arg(const Complex &z) { return atan2f(z.imag(), z.real()); }

/**
 *  @brief Get the conjugate of a complex number
 *  @return `z̄` Conjugate of the compex number
 */
inline Complex conj(const Complex &z) { return Complex(z.real(), -z.imag()); }

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
    return Complex::polar(powf(M_E, z.real()), z.imag());
 }

  /**
 *  @brief Get the natural logarithm of the complex number  
 *  @return `ln(z)` The natural log of the complex number
 */
inline Complex log(const Complex &z) {
    return Complex(logf(abs(z)), arg(z));

} // cobalt::math::algebra


  /**
 *  @brief Get the n-th power of the complex number
 *  @return `zⁿ` The n-th power of the complex number
 */
inline Complex pow(const Complex &z, float n) {
    return Complex::polar(powf(abs(z), n), arg(z)*n);
}

} // cobalt::math::algebra
