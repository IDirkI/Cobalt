#pragma once

#include <stdint.h>
#include <string>
#include <sstream>
#include <iomanip>

#include "complex.hpp"

namespace cobalt::math::algebra {

constexpr bool isZero(const Complex &z);
constexpr bool isReal(const Complex &z);
constexpr bool isImag(const Complex &z);


// ---------------- Member Utility ----------------
std::string Complex::toString(uint8_t percision) const {
    std::ostringstream oss;
    if(isZero(*this)) { oss << std::fixed << std::setprecision(percision) << 0          ; return oss.str(); }
    if(isReal(*this)) { oss << std::fixed << std::setprecision(percision) << re_        ; return oss.str(); }
    if(isImag(*this)) { oss << std::fixed << std::setprecision(percision) << im_ << "j" ; return oss.str(); }

    oss << std::fixed << std::setprecision(percision) << re_;

    oss << std::fixed << std::setprecision(percision) << ((im_ > 0.0f) ?" + " :" - ");

    oss << std::fixed << std::setprecision(percision) << std::abs(im_) << "j";

    return oss.str();
}

// ---------------- Checks ----------------
/**
 *  @brief Check if a complex number is zero (0 + 0j)
 */
constexpr bool isZero(const Complex &z) { 
    if(static_cast<float>(std::abs(z.real())) > COMPLEX_ZERO_THRESHOLD) { return false; }
    if(static_cast<float>(std::abs(z.imag())) > COMPLEX_ZERO_THRESHOLD) { return false; }
    return true;
}

/**
 *  @brief Check if a complex number is purely real
 */
constexpr bool isReal(const Complex &z) { 
    if(static_cast<float>(std::abs(z.real())) < COMPLEX_ZERO_THRESHOLD) { return false; }
    if(static_cast<float>(std::abs(z.imag())) > COMPLEX_ZERO_THRESHOLD) { return false; }
    return true;
}

/**
 *  @brief Check if a complex number is purely imaginary
 */
constexpr bool isImag(const Complex &z) { 
    if(static_cast<float>(std::abs(z.real())) > COMPLEX_ZERO_THRESHOLD) { return false; }
    if(static_cast<float>(std::abs(z.imag())) < COMPLEX_ZERO_THRESHOLD) { return false; }
    return true;
}
    
} // cobalt::math::algebra
