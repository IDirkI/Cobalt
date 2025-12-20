#pragma once

#include <math.h>

#include "../../linear_algebra/vector/vector.hpp"
#include "../../linear_algebra/vector/vector_ops.hpp"

#include "../../linear_algebra/matrix/matrix.hpp"
#include "../../linear_algebra/matrix/matrix_ops.hpp"

#include "../../algebra/complex/complex.hpp"

namespace cobalt::math::geometry {

// --------------------------------------
//             Quaternion    
// --------------------------------------
/**
 *  @brief Quaternion representing a 3D rotation/orientation
 *  @tparam T Element type (default float)
 */
template<typename T = def_floating, typename = std::enable_if_t<Floating<T>>>
struct Quaternion {
    private:
        T w_;
        T x_;
        T y_;
        T z_;
    public:
        // ---------------- Constructors ----------------
        /**
         *  @brief Zero-constructor
         */
        Quaternion() noexcept : w_(static_cast<T>(0)), x_(static_cast<T>(0)), y_(static_cast<T>(0)), z_(static_cast<T>(0)) {}

        /**
         *  @brief Default constructor from scalars
         *  @param w Real part
         *  @param x First imaginary part
         *  @param y Second imaginary part
         */
        Quaternion(T w, T x = static_cast<T>(0), T y = static_cast<T>(0), T z = static_cast<T>(0)) noexcept : w_(static_cast<T>(w)), x_(static_cast<T>(x)), y_(static_cast<T>(y)), z_(static_cast<T>(z)) {}

        /**
         *  @brief Constructor from a complex number (sets j and k components to zero)
         *  @param z Complex number to convert
         */
        Quaternion(cobalt::math::algebra::Complex z) noexcept : w_(z.real()), x_(z.imag()), y_(static_cast<T>(0)), z_(static_cast<T>(0)) {}

        // ---------------- Static Factories ----------------
        /**
         *  @brief Create a zero-quaternion
         */
        static inline Quaternion zero() noexcept { return Quaternion(static_cast<T>(0), static_cast<T>(0), static_cast<T>(0), static_cast<T>(0)); }

        /**
         *  @brief Create the identity quaternion
         */
        static inline Quaternion eye() noexcept { return Quaternion(static_cast<T>(1), static_cast<T>(0), static_cast<T>(0), static_cast<T>(0)); }

        /**
            *  @brief Create a purely imaginary quaternion from a 3-Vector
            *   @param v 3-Vector to convert
         */
        static inline Quaternion fromPureVector(const cobalt::math::linear_algebra::Vector<3, T> &v) noexcept {
            return Quaternion(static_cast<T>(0), static_cast<T>(v.x()), static_cast<T>(v.y()), static_cast<T>(v.z()));
        }

        /**
         *  @brief Create a quaternion from an axis-angle representation
         *  @param axis  Rotation axis
         *  @param angle Rotation angle in radians
         *  @note `axis` is assumed to be normalized
         */
        static inline Quaternion fromAxisAngle(const cobalt::math::linear_algebra::Vector<3, T> &axis, T angle) noexcept {
            cobalt::math::linear_algebra::Vector<3, T> u = normalize(axis);
            T halfAngle = angle * static_cast<T>(0.5);
            T s = std::sin(halfAngle);
            return Quaternion(std::cos(halfAngle), u.x() * s, u.y() * s, u.z() * s);
        }

        /**
         *  @brief Create a quaternion from an 3-Vector
         *  @param v 3-Vector representing axis-angle rotation
         *  @note The direction of `v` is the rotation axis, and `|v|` is the rotation angle in radians
         */
        static inline Quaternion fromRotationVector(const cobalt::math::linear_algebra::Vector<3, T> &v) noexcept {
            cobalt::math::linear_algebra::Vector<3, T> axis = normalize(v);
            T angle = norm(v);

            if(angle < epsilon_<T>) { return Quaternion::eye(); }

            return fromAxisAngle(axis, angle);
        }

        /**
         *  @brief Create a quaternion from an 3x3 Rotation Matrix
         *  @param R 3x3 Rotation matrix 
         *  @note R must be a proper rotation matrix
         */
        static inline Quaternion fromRotationMatrix(const cobalt::math::linear_algebra::Matrix<3, 3, T> &R) noexcept {
            T diag = trace(R);
            T s1 = (R(2,1) - R(1, 2) > 0) ?1.0f :-1.0f;
            T s2 = (R(0,2) - R(2, 0) > 0) ?1.0f :-1.0f;
            T s3 = (R(1,0) - R(0, 1) > 0) ?1.0f :-1.0f;

            return Quaternion(
                0.5*std::sqrt(static_cast<T>(1) + diag),
                0.5*std::sqrt(static_cast<T>(1) + diag)*s1,
                0.5*std::sqrt(static_cast<T>(1) - diag)*s2,
                0.5*std::sqrt(static_cast<T>(1) - diag)*s3
            );

        }

        /**
         *  @brief Create a quaternion from Euler angles (roll, pitch, yaw)
         *  @param roll  Rotation around the x-axis in radians
         *  @param pitch Rotation around the y-axis in radians
         *  @param yaw   Rotation around the z-axis in radians
         *  @note Uses the ZYX rotation order (yaw-pitch-roll)
         */
        static inline Quaternion fromEuler(T roll, T pitch, T yaw) noexcept {
            T cr = std::cos(roll * 0.5f);
            T sr = std::sin(roll * 0.5f);
            T cp = std::cos(pitch * 0.5f);
            T sp = std::sin(pitch * 0.5f);
            T cy = std::cos(yaw * 0.5f);
            T sy = std::sin(yaw * 0.5f);

            return Quaternion(
                cr*cp*cy + sr*sp*sy,
                sr*cp*cy - cr*sp*sy,
                cr*sp*cy + sr*cp*sy,
                cr*cp*sy - sr*sp*cy
            );
        }

        // ---------------- Accessors ----------------
        /**
         *  @brief Access the w component
         *  @return Reference to component
         */
        constexpr T &w() noexcept { return w_; }

        /**
         *  @brief Access the x component
         *  @return Reference to component
         */
        constexpr T &x() noexcept { return x_; }

        /**
         *  @brief Access the y component
         *  @return Reference to component
         */
        constexpr T &y() noexcept { return y_; }

        /**
         *  @brief Access the z component
         *  @return Reference to component
         */
        constexpr T &z() noexcept { return z_; }

        /**
         *  @brief Const access the w component
         *  @return Const reference to component
         */
        constexpr const T &w() const noexcept { return w_; }

        /**
         *  @brief Const access the x component
         *  @return Const reference to component
         */
        constexpr const T &x() const noexcept { return x_; }

        /**
         *  @brief Const access the y component
         *  @return Const reference to component
         */
        constexpr const T &y() const noexcept { return y_; }

        /**
         *  @brief Const access the z component
         *  @return Const reference to component
         */
        constexpr const T &z() const noexcept { return z_; }

        /**
         *  @brief Const access the imaginary vector component
         *  @return Const reference to imaginary vector
         */
        constexpr const cobalt::math::linear_algebra::Vector<3, T> vector() const { return cobalt::math::linear_algebra::Vector<3, T>(x_, y_, z_); }

        // ---------------- Overloads ----------------

        constexpr Quaternion &operator+=(const Quaternion &rhs) noexcept {
            w_ += rhs.w_;
            x_ += rhs.x_;
            y_ += rhs.y_;
            z_ += rhs.z_;
            return *this;
        }

        constexpr Quaternion &operator-=(const Quaternion &rhs) noexcept {
            w_ -= rhs.w_;
            x_ -= rhs.x_;
            y_ -= rhs.y_;
            z_ -= rhs.z_;
            return *this;
        }

        constexpr Quaternion &operator*=(const Quaternion &rhs) noexcept {
            T tempW = w_*rhs.w_ - x_*rhs.x_ - y_*rhs.y_ - z_*rhs.z_;
            T tempX = w_*rhs.x_ + x_*rhs.w_ + y_*rhs.z_ - z_*rhs.y_;
            T tempY = w_*rhs.y_ - x_*rhs.z_ + y_*rhs.w_ + z_*rhs.x_;
            T tempZ = w_*rhs.z_ + x_*rhs.y_ - y_*rhs.x_ + z_*rhs.w_;

            w_ = tempW;
            x_ = tempX;
            y_ = tempY;
            z_ = tempZ;
            
            return *this;
        }

        constexpr Quaternion &operator/=(const Quaternion &rhs) noexcept {
            T normSquared = rhs.x_*rhs.x_ + rhs.y_*rhs.y_ + rhs.z_*rhs.z_ + rhs.w_*rhs.w_;
            T tempW = (w_*rhs.w_ + x_*rhs.x_ + y_*rhs.y_ + z_*rhs.z_) / normSquared;
            T tempX = (x_*rhs.w_ - w_*rhs.x_ - y_*rhs.z_ + z_*rhs.y_) / normSquared;
            T tempY = (y_*rhs.w_ + w_*rhs.y_ + x_*rhs.z_ - z_*rhs.x_) / normSquared;
            T tempZ = (z_*rhs.w_ - w_*rhs.z_ - x_*rhs.y_ + y_*rhs.x_) / normSquared;

            w_ = tempW;
            x_ = tempX;
            y_ = tempY;
            z_ = tempZ;
            
            return *this;
        }

        constexpr Quaternion &operator*=(float c) noexcept {
            w_ *= c;
            x_ *= c;
            y_ *= c;
            z_ *= c;
            return *this;
        }

        constexpr Quaternion &operator/=(float c) noexcept {
            w_ /= c;
            x_ /= c;
            y_ /= c;
            z_ /= c;
            return *this;
        }
        
};
} // cobalt::math::geometry