#pragma once

#include <stdint.h>
#include <string>
#include <sstream>
#include <iomanip>

#include "complex.hpp"

namespace cobalt::math::algebra {

// ---------------- Checks ----------------
/**
 *  @brief Check if a complex number is zero (0 + 0j)
 */
bool isZero(const Complex &z) { 
    if(fabsf(z.real()) > COMPLEX_ZERO_THRESHOLD) { return false; }
    if(fabsf(z.imag()) > COMPLEX_ZERO_THRESHOLD) { return false; }
    return true;
}

/**
 *  @brief Check if a complex number is purely real
 */
bool isReal(const Complex &z) { 
    if(fabsf(z.real()) < COMPLEX_ZERO_THRESHOLD) { return false; }
    if(fabsf(z.imag()) > COMPLEX_ZERO_THRESHOLD) { return false; }
    return true;
}

/**
 *  @brief Check if a complex number is purely imaginary
 */
bool isImag(const Complex &z) { 
    if(fabsf(z.real()) > COMPLEX_ZERO_THRESHOLD) { return false; }
    if(fabsf(z.imag()) < COMPLEX_ZERO_THRESHOLD) { return false; }
    return true;
}
    
} // cobalt::math::algebra
