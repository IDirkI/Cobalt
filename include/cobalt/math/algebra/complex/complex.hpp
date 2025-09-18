#pragma once

#include <stdint.h>
#include <cmath>
#include <string>

namespace cobalt::math::algebra {

constexpr float COMPLEX_EQUAL_THRESHOLD = 1e-5;
constexpr float COMPLEX_ZERO_THRESHOLD = 1e-12;

constexpr float COMPLEX_DEFAULT_PRECISION = 3;

// --------------------------------------
//          Complex Number    
// --------------------------------------

struct Complex {
    private:
        float re_;
        float im_;

    public:
        // ---------------- Constructors ----------------
        /**
         *  @brief Default, zero-constructor. (0 + 0j)
         */
        Complex() noexcept: re_(0.0f), im_(0.0f) {}

        /**
         *  @brief Construct from real and imaginary parts
         *  @param real Real part
         *  @param imag Imaginary part
         */
        Complex(float real, float imag = 0.0f) noexcept : re_(real), im_(imag) {}


        // ---------------- Static Factories ----------------
        /**
         *  @brief Construct 0 + 0j.
         */
        static Complex zero() noexcept { return Complex(); }

        /**
         *  @brief Construct 1 + 0j.
         */
        static Complex one() noexcept { return Complex(1.0f); }

        /**
         *  @brief Construct 0 + 1j.
         */
        static Complex oneIm() noexcept { return Complex(0.0f, 1.0f); }

        /**
         *  @brief Construct reᶦθ
         *  @param r The norm/magnitude of the complex number
         *  @param theta The argument/angle of the complex number
         */
        static Complex polar(float r, float theta) noexcept { return Complex(r*cosf(theta), r*sinf(theta)); }


        // ---------------- Accessors ----------------
        /**
         *  @brief Const access to the real part.
         *  @return Const reference to real part.
         */
        constexpr float real() const { return re_; }

        /**
         *  @brief Const access to the imaginary part.
         *  @return Const reference to imaginary part.
         */
        constexpr float imag() const { return im_; }

        /**
         *  @brief Sets the real element
         */
        void real(float re) { re_ = re; }

        /**
         *  @brief Sets the imaginary element
         */
        void imag(float im) { im_ = im; }


        // ---------------- Operator Overloads ----------------
        /**
         *  @brief Add another complex number to this complex number.
         */
        constexpr Complex &operator+=(const Complex &rhs) {
            re_ += rhs.re_;
            im_ += rhs.im_;

            return *this;
        }

        /**
         *  @brief Add a real number to this complex number.
         */
        constexpr Complex &operator+=(float c) {
            re_ += c;

            return *this;
        }

        /**
         *  @brief Subtract another complex number from this complex number.
         */
        constexpr Complex &operator-=(const Complex &rhs) {
            re_ -= rhs.re_;
            im_ -= rhs.im_;

            return *this;
        }

        /**
         *  @brief Subtract a real number from this complex number.
         */
        constexpr Complex &operator-=(float c) {
            re_ -= c;

            return *this;
        }

        /**
         *  @brief Multiply this complex number by another complex number.
         */
        constexpr Complex &operator*=(const Complex &rhs) {
            re_ = re_*rhs.re_ - im_*rhs.im_;
            im_ = re_*rhs.im_ + im_*rhs.re_;

            return *this;
        }

        /**
         *  @brief Multiply this complex number by a scalar
         */
        constexpr Complex &operator*=(float c) {
            re_ *= c;
            im_ *= c;

            return *this;
        }

        /**
         *  @brief Divide this complex number by another complex number.
         */
        constexpr Complex &operator/=(const Complex &rhs) {
            float denom = rhs.re_*rhs.re_ + rhs.im_*rhs.im_;
            
            re_ = (re_*rhs.re_ - im_*rhs.im_) / denom;
            im_ = (re_*rhs.im_ - im_*rhs.re_) / denom;

            return *this;
        }

        /**
         *  @brief Divide this complex number by a scalar
         */
        constexpr Complex &operator/=(float c) {
            re_ /= c;
            im_ /= c;

            return *this;
        }
};

} // cobalt::math::algebra