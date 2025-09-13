#pragma once

#include <math.h>

#include "../../linear_algebra/vector/vector.hpp"
#include "../../linear_algebra/vector/vector_ops.hpp"


namespace cobalt::math::geometry {

constexpr float QUATERNION_EQUAL_THRESHOLD = 1e-5;
constexpr float QUATERNION_ZERO_THRESHOLD = 1e-12;

// --------------------------------------
//             Quaternion    
// --------------------------------------

struct Quaternion {
    private:
        float w_;
        float x_;
        float y_;
        float z_;
    public:
        // ---------------- Constructors ----------------
        /**
         * @brief Default, zero-constructor
         */
        Quaternion() : w_(0.0f), x_(0.0f), y_(0.0f), z_(0.0f) {}

        /**
         * @brief Default constructor
         */
        Quaternion(float w, float x = 0.0f, float y = 0.0f, float z = 0.0f) : w_(w), x_(x), y_(y), z_(z) {}

        // ---------------- Static Factories ----------------
        /**
         * @brief Create a zero-quaternion
         */
        static inline Quaternion zero() { return Quaternion(); }

        /**
         * @brief Create a unit quaternion
         */
        static inline Quaternion unit() { return Quaternion(1.0f); }

        /**
         * @brief Create a quaternion from an 3-Vector
         * @param v Vector to turn into quaternion
         * @note `|v|` represents the turning angle with the normalized v representing a rotation axis
         */
        static inline Quaternion fromVector(const cobalt::math::linear_algebra::Vector<3> &v) {
            cobalt::math::linear_algebra::Vector<3> u = normalize(v);
            float half = norm(v) * 0.5f;
            float s = sinf(half);
            return Quaternion(cosf(half), u.x() * s, u.y() * s, u.z() * s);
        }

        // ---------------- Accessors ----------------
        /**
         * @brief Const access to w element
         */
        const inline float w() const { return w_; }

        /**
         * @brief Const access to x element
         */
        const inline float x() const { return x_; }

        /**
         * @brief Const access to y element
         */
        const inline float y() const { return y_; }

        /**
         * @brief Const access to z element
         */
        const inline float z() const { return z_; }


        /**
         * @brief Set w element
         */
        constexpr inline void w(float wNew) { w_ = wNew; }

        /**
         * @brief Set x element
         */
        constexpr inline void x(float xNew) { x_ = xNew; }

        /**
         * @brief Set y element
         */
        constexpr inline void y(float yNew) { y_ = yNew; }

        /**
         * @brief Set z element
         */
        constexpr inline void z(float zNew) { z_ = zNew; }


        // ---------------- Overloads ----------------

        inline Quaternion &operator+=(const Quaternion &rhs) {
            w_ += rhs.w_;
            x_ += rhs.x_;
            y_ += rhs.y_;
            z_ += rhs.z_;
            return *this;
        }

        inline Quaternion &operator-=(const Quaternion &rhs) {
            w_ -= rhs.w_;
            x_ -= rhs.x_;
            y_ -= rhs.y_;
            z_ -= rhs.z_;
            return *this;
        }

        inline Quaternion &operator*=(const Quaternion &rhs) {
            w_ = w_*rhs.w_ - x_*rhs.x_ - y_*rhs.y_ - z_*rhs.z_;
            x_ = w_*rhs.x_ + x_*rhs.w_ + y_*rhs.z_ - z_*rhs.y_;
            y_ = w_*rhs.y_ - x_*rhs.z_ + y_*rhs.w_ + z_*rhs.x_;
            z_ = w_*rhs.z_ + x_*rhs.y_ - y_*rhs.x_ + z_*rhs.w_;
            return *this;
        }

        inline Quaternion &operator*=(float c) {
            w_ *= c;
            x_ *= c;
            y_ *= c;
            z_ *= c;
            return *this;
        }

        inline Quaternion &operator/=(float c) {
            w_ /= c;
            x_ /= c;
            y_ /= c;
            z_ /= c;
            return *this;
        }

        inline Quaternion &operator-() {
            w_ *= -1.0f;
            x_ *= -1.0f;
            y_ *= -1.0f;
            z_ *= -1.0f;
            return *this;
        }
        
};
} // cobalt::math::geometry