#pragma once

#define _USE_MATH_DEFINES
#include <stdint.h>
#include <cmath>
#include <array>

#include "complex.hpp"

namespace cobalt::math::algebra {

// ---------------- Non-member Utility ----------------
/**
 * @brief Clamp the magnitude/norm of a complex number to a maximum value
 * @param z Complex number to clamp
 * @param maxMagnitude Maximum allowed magnitude/norm
 */
inline Complex clampMagnitude(const Complex &z, float maxMagnitude) {
    float currentNormSqr = z.real()*z.real() + z.imag()*z.imag();
    float maxNormSqr = maxMagnitude * maxMagnitude;

    if(currentNormSqr < maxNormSqr) {
        return z;
    }

    float scale = maxMagnitude / std::sqrt(currentNormSqr);
    return Complex(z.real() * scale, z.imag() * scale);
}

/**
 *  @brief Round the real and imaginary parts of a complex number
 *  @param z Complex number to round
 */
inline Complex round(const Complex &z) {
    return Complex(std::round(z.real()), std::round(z.imag()));
}

/**
 *  @brief Ceil the real and imaginary parts of a complex number
 *  @param z Complex number to ceil
 */
inline Complex ceil(const Complex &z) {
    return Complex(std::ceil(z.real()), std::ceil(z.imag()));
}

/**
 *  @brief Floor the real and imaginary parts of a complex number
 *  @param z Complex number to floor
 */
inline Complex floor(const Complex &z) {
    return Complex(std::floor(z.real()), std::floor(z.imag()));
}

/**
 *  @brief Sets the components of a complex number to a clean zero if they are very close to zero
 *  @param z Complex number to clean
 */
inline Complex cleanZero(const Complex &z) {
    return Complex(
        (std::abs(z.real()) < COMPLEX_EQUAL_THRESHOLD) ?0.0f :z.real(),
        (std::abs(z.imag()) < COMPLEX_EQUAL_THRESHOLD) ?0.0f :z.imag()
    );
}

// ---------------- Math Extention ----------------
/**
 *  @brief Calculate the n-th roots of a complex number
 *  @param z Complex number
 *  @param n Number of roots
 *  @param roots Output array of roots (must have size n)
 * 
 *  @tparam N Number of roots to compute
 */
template<size_t N>
    inline void nthRoots(const Complex &z, std::array<Complex, N> &roots) {
        float r = std::sqrt(z.real()*z.real() + z.imag()*z.imag());
        float theta = std::atan2(z.imag(), z.real());
        
        float rootR = std::pow(r, 1.0f / N);
        
        for(uint8_t k = 0; k < N; k++) {
            float rootTheta = (theta + 2.0f * M_PI * k) / N;
            roots[k] = Complex::polar(rootR, rootTheta);
        }
    }

/**
 *  @brief Compute the distance between two complex numbers
 */
inline float distance(const Complex &z, const Complex &w) {
    float dRe = z.real() - w.real();
    float dIm = z.imag() - w.imag();
    return std::sqrt(dRe*dRe + dIm*dIm);
} 

/**
 *  @brief Project complex number onto real axis
 */
inline Complex projectReal(const Complex &z) {
    return Complex(z.real(), 0.0f);
}

/**
 *  @brief Project complex number onto imaginary axis
 */
inline Complex projectImag(const Complex &z) {
    return Complex(0.0f, z.imag());
}

/**
 *  @brief Project complex number onto unit circle
 */
inline Complex projectUnit(const Complex &z) {
    float mag = std::sqrt(z.real()*z.real() + z.imag()*z.imag());
    
    if(mag < COMPLEX_EQUAL_THRESHOLD) {
        return Complex::one(); // Default to 1 if magnitude is zero
    }
    
    return Complex(z.real() / mag, z.imag() / mag);
}

// ---------------- Rotation ----------------

/**
 *  @brief Rotate complex number by angle theta (counterclockwise)
 *  @param z Complex number to rotate
 *  @param theta Rotation angle in radians
 *  @return Rotated complex number
 */
inline Complex rotate(const Complex &z, float theta) {
    Complex rotation = Complex::polar(1.0f, theta);
    Complex result = z;
    result *= rotation;
    return result;
}

/**
 *  @brief Rotate complex number 90 degrees counterclockwise (multiply by i)
 */
inline Complex rotate90(const Complex &z) {
    return Complex(-z.imag(), z.real());
}

/**
 *  @brief Rotate complex number 180 degrees (negate)
 */
inline Complex rotate180(const Complex &z) {
    return Complex(-z.real(), -z.imag());
}

/**
 *  @brief Rotate complex number 270 degrees counterclockwise (multiply by -i)
 */
inline Complex rotate270(const Complex &z) {
    return Complex(z.imag(), -z.real());
}

// ---------------- Conversion ----------------
/**
 *  @brief Convert a complex number to its polar representation
 * 
 *  @param z Complex number to convert
 *  @param r Output norm/magnitude
 *  @param theta Output argument/angle
 */
inline void toPolar(const Complex &z, float &r, float &theta) {
    r = std::sqrt(z.real()*z.real() + z.imag()*z.imag());
    theta = std::atan2(z.imag(), z.real());
}

// ---------------- Interpolation ----------------
/**
 *  @brief Interpolate between two complex numbers
 *  @param z Start complex number
 *  @param w End complex number
 *  @param t Interpolation parameter [0, 1]
 *  @return Interpolated complex number
 */
inline Complex lerp(const Complex &z, const Complex &w, float t) {
    return Complex(
        z.real() + t * (w.real() - z.real()),
        z.imag() + t * (w.imag() - z.imag())
    );
}

/**
 *  @brief Spherically interpolate on unit circle
 *  @param z Start complex number
 *  @param w End complex number
 *  @param t Interpolation factor
 *  @return Interpolated complex number on unit circle
 */
inline Complex slerp(const Complex &z, const Complex &w, float t) {
    float r1;
    float r2;
    float theta1;
    float theta2;
    
    toPolar(z, r1, theta1);
    toPolar(w, r2, theta2);
    
    // Interpolate in polar coordinates
    float r = r1 + t * (r2 - r1);
    
    // Handle angle wrapping
    float dtheta = theta2 - theta1;
    if(dtheta > M_PI) {
        dtheta -= 2.0f * M_PI;
    } else if(dtheta < -M_PI) {
        dtheta += 2.0f * M_PI;
    }
    
    float theta = theta1 + t * dtheta;
    
    return Complex::polar(r, theta);
}

// ---------------- Checks ----------------
/**
 *  @brief Check if a complex number is zero (0 + 0j)
 */
bool isZero(const Complex &z) { 
    if(std::abs(z.real()) > COMPLEX_EQUAL_THRESHOLD) { return false; }
    if(std::abs(z.imag()) > COMPLEX_EQUAL_THRESHOLD) { return false; }
    return true;
}

/**
 *  @brief Check if a complex number is purely real
 */
bool isReal(const Complex &z) { 
    if(std::abs(z.imag()) > COMPLEX_EQUAL_THRESHOLD) { return false; }
    return true;
}

/**
 *  @brief Check if a complex number is purely imaginary
 */
bool isImag(const Complex &z) { 
    if(std::abs(z.real()) > COMPLEX_EQUAL_THRESHOLD) { return false; }
    if(std::abs(z.imag()) < COMPLEX_EQUAL_THRESHOLD) { return false; }
    return true;
}

/**
 *  @brief Check if a complex number is has a norm of 1
 */
bool isUnit(const Complex &z) { 
    return (std::abs(z.real()*z.real() + z.imag()*z.imag() - 1.0f) < COMPLEX_EQUAL_THRESHOLD);
}

/**
 *  @brief Check if two complex number are conjugates of eachother
 */
bool isConjugate(const Complex &z, const Complex &w) { 
    if(std::abs(z.real() - w.real()) > COMPLEX_EQUAL_THRESHOLD) { return false; }
    if(std::abs(z.imag() + w.imag()) > COMPLEX_EQUAL_THRESHOLD) { return false; }
    return true;
}
    
} // cobalt::math::algebra
