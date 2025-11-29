#pragma once

#include <math.h>

#include "../../linear_algebra/vector/vector.hpp"
#include "../../linear_algebra/vector/vector_ops.hpp"

#include "../../algebra/complex/complex.hpp"


namespace cobalt::math::geometry {

constexpr float QUATERNION_EQUAL_THRESHOLD = 1e-5;

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
         *  @brief Zero-constructor
         */
        Quaternion() noexcept : w_(0.0f), x_(0.0f), y_(0.0f), z_(0.0f) {}

        /**
         *  @brief Default constructor from scalars
         *  @param w Real part
         *  @param x First imaginary part
         *  @param y Second imaginary part
         */
        template<typename T = float>
        Quaternion(T w, T x = 0.0f, T y = 0.0f, T z = 0.0f) noexcept : w_(static_cast<float>(w)), x_(static_cast<float>(x)), y_(static_cast<float>(y)), z_(static_cast<float>(z)) {}

        /**
         *  @brief Constructor from a complex number (sets j and k components to zero)
         *  @param z Complex number to convert
         */
        Quaternion(cobalt::math::algebra::Complex z) noexcept : w_(z.real()), x_(z.imag()), y_(0.0f), z_(0.0f) {}

        // ---------------- Static Factories ----------------
        /**
         *  @brief Create a zero-quaternion
         */
        static inline Quaternion zero() { return Quaternion(0.0f, 0.0f, 0.0f, 0.0f); }

        /**
         *  @brief Create the identity quaternion
         */
        static inline Quaternion eye() { return Quaternion(1.0f, 0.0f, 0.0f, 0.0f); }

        /**
            *  @brief Create a purely imaginary quaternion from a 3-Vector
            *   @param v 3-Vector to convert
         */
        static inline Quaternion pure(const cobalt::math::linear_algebra::Vector<3> &v) {
            return Quaternion(0.0f, v.x(), v.y(), v.z());
        }

        /**
         *  @brief Create a quaternion from an axis-angle representation
         *  @param axis  Rotation axis
         *  @param angle Rotation angle in radians
         *  @note `axis` is assumed to be normalized
         */
        static inline Quaternion axisAngle(const cobalt::math::linear_algebra::Vector<3> &axis, float angle) {
            cobalt::math::linear_algebra::Vector<3> u = normalize(axis);
            float halfAngle = angle * 0.5f;
            float s = std::sin(halfAngle);
            return Quaternion(std::cos(halfAngle), u.x() * s, u.y() * s, u.z() * s);
        }

        /**
         *  @brief Create a quaternion from an 3-Vector
         *  @param v 3-Vector representing axis-angle rotation
         *  @note The direction of `v` is the rotation axis, and `|v|` is the rotation angle in radians
         */
        static inline Quaternion rotationVector(const cobalt::math::linear_algebra::Vector<3> &v) {
            cobalt::math::linear_algebra::Vector<3> axis = normalize(v);
            float angle = norm(v);

            if(angle < QUATERNION_EQUAL_THRESHOLD) { return Quaternion::eye(); }

            return axisAngle(axis, angle);
        }

        /**
         *  @brief Create a quaternion from Euler angles (roll, pitch, yaw)
         *  @param roll  Rotation around the x-axis in radians
         *  @param pitch Rotation around the y-axis in radians
         *  @param yaw   Rotation around the z-axis in radians
         *  @note Uses the ZYX rotation order (yaw-pitch-roll)
         */
        static inline Quaternion euler(float roll, float pitch, float yaw) {
            float cr = std::cos(roll * 0.5f);
            float sr = std::sin(roll * 0.5f);
            float cp = std::cos(pitch * 0.5f);
            float sp = std::sin(pitch * 0.5f);
            float cy = std::cos(yaw * 0.5f);
            float sy = std::sin(yaw * 0.5f);

            return Quaternion(
                cr*cp*cy + sr*sp*sy,
                sr*cp*cy - cr*sp*sy,
                cr*sp*cy + sr*cp*sy,
                cr*cp*sy - sr*sp*cy
            );
        }

        // ---------------- Accessors ----------------
        /**
         * @brief Const access to w element
         */
        constexpr float w() const { return w_; }

        /**
         * @brief Const access to x element
         */
        constexpr float x() const { return x_; }

        /**
         * @brief Const access to y element
         */
        constexpr float y() const { return y_; }

        /**
         * @brief Const access to z element
         */
        constexpr float z() const { return z_; }

        /**
         * @brief Const access to the vector part as a 3-Vector
         */
        constexpr cobalt::math::linear_algebra::Vector<3, float> vector() const { return cobalt::math::linear_algebra::Vector<3, float>(x_, y_, z_); }


        /**
         * @brief Set w element
         */
        constexpr void w(float wNew) { w_ = wNew; }

        /**
         * @brief Set x element
         */
        constexpr void x(float xNew) { x_ = xNew; }

        /**
         * @brief Set y element
         */
        constexpr void y(float yNew) { y_ = yNew; }

        /**
         * @brief Set z element
         */
        constexpr void z(float zNew) { z_ = zNew; }

        /**
         * @brief Set the vector part from a 3-Vector
         */
        constexpr void vector(cobalt::math::linear_algebra::Vector<3, float> &v) { 
            x_ = v.x();
            y_ = v.y();
            z_ = v.z();
        }

        // ---------------- Overloads ----------------

        constexpr Quaternion &operator+=(const Quaternion &rhs) {
            w_ += rhs.w_;
            x_ += rhs.x_;
            y_ += rhs.y_;
            z_ += rhs.z_;
            return *this;
        }

        constexpr Quaternion &operator-=(const Quaternion &rhs) {
            w_ -= rhs.w_;
            x_ -= rhs.x_;
            y_ -= rhs.y_;
            z_ -= rhs.z_;
            return *this;
        }

        constexpr Quaternion &operator*=(const Quaternion &rhs) {
            float tempW = w_*rhs.w_ - x_*rhs.x_ - y_*rhs.y_ - z_*rhs.z_;
            float tempX = w_*rhs.x_ + x_*rhs.w_ + y_*rhs.z_ - z_*rhs.y_;
            float tempY = w_*rhs.y_ - x_*rhs.z_ + y_*rhs.w_ + z_*rhs.x_;
            float tempZ = w_*rhs.z_ + x_*rhs.y_ - y_*rhs.x_ + z_*rhs.w_;

            w_ = tempW;
            x_ = tempX;
            y_ = tempY;
            z_ = tempZ;
            
            return *this;
        }

        constexpr Quaternion &operator/=(const Quaternion &rhs) {
            float normSquared = rhs.x_*rhs.x_ + rhs.y_*rhs.y_ + rhs.z_*rhs.z_ + rhs.w_*rhs.w_;
            float tempW = (w_*rhs.w_ + x_*rhs.x_ + y_*rhs.y_ + z_*rhs.z_) / normSquared;
            float tempX = (x_*rhs.w_ - w_*rhs.x_ - y_*rhs.z_ + z_*rhs.y_) / normSquared;
            float tempY = (y_*rhs.w_ + w_*rhs.y_ + x_*rhs.z_ - z_*rhs.x_) / normSquared;
            float tempZ = (z_*rhs.w_ - w_*rhs.z_ - x_*rhs.y_ + y_*rhs.x_) / normSquared;

            w_ = tempW;
            x_ = tempX;
            y_ = tempY;
            z_ = tempZ;
            
            return *this;
        }

        constexpr Quaternion &operator*=(float c) {
            w_ *= c;
            x_ *= c;
            y_ *= c;
            z_ *= c;
            return *this;
        }

        constexpr Quaternion &operator/=(float c) {
            w_ /= c;
            x_ /= c;
            y_ /= c;
            z_ /= c;
            return *this;
        }
        
};
} // cobalt::math::geometry