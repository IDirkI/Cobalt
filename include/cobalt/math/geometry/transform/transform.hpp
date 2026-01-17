#pragma once

#include <array>

#include "../../config.hpp"

#include "../quaternion/quaternion.hpp"
#include "../quaternion/quaternion_ops.hpp"

#include "../../linear_algebra/matrix/matrix.hpp"
#include "../../linear_algebra/vector/vector.hpp"

namespace cobalt::math::geometry {

template<typename T = def_floating, typename = std::enable_if_t<Floating<T>>>
    const cobalt::math::linear_algebra::Vector<3, T> TRANSFORM_DEFAULT_TRANSLATION = cobalt::math::linear_algebra::Vector<3, T>::zero();
template<typename T = def_floating, typename = std::enable_if_t<Floating<T>>>
    const cobalt::math::geometry::Quaternion<T> TRANSFORM_DEFAULT_ROTATION = cobalt::math::geometry::Quaternion<T>::eye();

// --------------------------------------
//      Homogeneous Transformations    
// --------------------------------------
/**
 *  @brief Homogeneous transformation matrix.
 *  @tparam T Element type (default float).
 */
template<typename T = def_floating, typename = std::enable_if_t<Floating<T>>>
struct Transform {
    private:
        cobalt::math::geometry::Quaternion<T> q_;
        cobalt::math::linear_algebra::Vector<3, T> t_;

    public: 
        // ---------------- Constructors ----------------
        /**
         *  @brief Construct an identity transformation matrix.
         */
        constexpr Transform() noexcept 
            : q_(cobalt::math::geometry::Quaternion<T>::eye()), t_(cobalt::math::linear_algebra::Vector<3, T>::zero()) {}

        /**
         *  @brief Construct an transformation from rotation quaternion and translation.
         *  @param q Rotation quaternion associated with the transform
         *  @param t Translation vector associated with the transform
         */
        constexpr Transform(const cobalt::math::geometry::Quaternion<T> &q, const cobalt::math::linear_algebra::Vector<3, T> &t = TRANSFORM_DEFAULT_TRANSLATION<T>) noexcept
            : q_(q), t_(t){}

        /**
         *  @brief Construct an transformation from rotation quaternion and translation.
         *  @param q Rotation quaternion associated with the transform
         *  @param t Translation vector associated with the transform
         */
        constexpr Transform(const cobalt::math::linear_algebra::Vector<3, T> &t, const cobalt::math::geometry::Quaternion<T> &q = TRANSFORM_DEFAULT_ROTATION<T>) noexcept
            : q_(q), t_(t){}

        /**
         *  @brief Construct an transformation from rotation matrix and translation.
         *  @param R Rotation matrix associated with the transform
         *  @param t Translation vector associated with the transformn
         */
        constexpr Transform(const cobalt::math::linear_algebra::Matrix<3, 3, T> &R, const cobalt::math::linear_algebra::Vector<3, T> &t = TRANSFORM_DEFAULT_TRANSLATION<T>) noexcept
            : q_(cobalt::math::geometry::Quaternion<T>::fromRotationMatrix(R)), t_(t){}

        // ---------------- Static Factories ----------------
        /**
         *  @brief Construct an identity transformation.
         */
        static constexpr Transform<T> eye() {
            return Transform<T>();
        }

        /**
         *  @brief Construct a pure rotation transformation from given rotation vector.
         *  @param v Vector representing the rotation/orientation
         * 
         *  The direction of the vector is the axis while the norm of the vector is the angle
         */
        static constexpr Transform<T> fromRotationVector(const cobalt::math::linear_algebra::Vector<3, T> &v) {
            Quaternion<T> q = Quaternion<T>::fromRotationVector(v);

            return Transform<T>(q);
        }

        /**
         *  @brief Construct a rotation-transformation along the X-axis from a given angle.
         *  @param angle Angle to rotate around X-axis
         */
        static constexpr Transform<T> fromRotationX(T angle) {
            cobalt::math::linear_algebra::Matrix<3, 3, T> R = cobalt::math::linear_algebra::Matrix<3, 3, T>::eye();

            R(1,1) = std::cos(angle);
            R(1,2) = -std::sin(angle);
            R(2,1) = std::sin(angle);
            R(2,2) = std::cos(angle);
        
            return Transform<T>(R, cobalt::math::linear_algebra::Vector<3, T>::zero());
        }

        /**
         *  @brief Construct a rotation-transformation along the Y-axis from a given angle.
         *  @param angle Angle to rotate around Y-axis
         */
        static constexpr Transform<T> fromRotationY(T angle) {
             cobalt::math::linear_algebra::Matrix<3, 3, T> R = cobalt::math::linear_algebra::Matrix<3, 3, T>::eye();

            R(0,0) = std::cos(angle);
            R(0,2) = std::sin(angle);
            R(2,0) = -std::sin(angle);
            R(2,2) = std::cos(angle);
        
            return Transform<T>(R, cobalt::math::linear_algebra::Vector<3, T>::zero());
        }

        /**
         *  @brief Construct a rotation-transformation along the Z-axis from a given angle.
         *  @param angle Angle to rotate around Z-axis
         */
        static constexpr Transform<T> fromRotationZ(T angle) {
            cobalt::math::linear_algebra::Matrix<3, 3, T> R = cobalt::math::linear_algebra::Matrix<3, 3, T>::eye();

            R(0,0) = std::cos(angle);
            R(0,1) = -std::sin(angle);
            R(1,0) = std::sin(angle);
            R(1,1) = std::cos(angle);
        
            return Transform<T>(R, cobalt::math::linear_algebra::Vector<3, T>::zero());
        }

        // ---------------- Chain Operations ----------------
        /**
         *  @brief Apply a rotation around the X-axis to the transformation.
         *  @param angle Angle to rotate around X-axis
         *  @return Reference to this transformation after rotation for chaining
         */
        constexpr Transform<T> &rotateX(T angle) {
            Quaternion<T> Q = Quaternion<T>::fromAxisAngle(
                cobalt::math::linear_algebra::Vector<3>(1,0,0), angle
            );
            q_ *= Q;
            return *this;
        }

        /**
         *  @brief Apply a rotation around the Y-axis to the transformation.
         *  @param angle Angle to rotate around Y-axis
         *  @return Reference to this transformation after rotation for chaining
         */
        constexpr Transform<T> &rotateY(T angle) {
            Quaternion<T> Q = Quaternion<T>::fromAxisAngle(
                cobalt::math::linear_algebra::Vector<3>(0,1,0), angle
            );
            q_ *= Q;
            return *this;
        }

        /**
         *  @brief Apply a rotation around the Z-axis to the transformation.
         *  @param angle Angle to rotate around Z-axis
         *  @return Reference to this transformation after rotation for chaining
         */
        constexpr Transform<T> &rotateZ(T angle) {
            Quaternion<T> Q = Quaternion<T>::fromAxisAngle(
                cobalt::math::linear_algebra::Vector<3>(0,0,1), angle
            );
            q_ *= Q;
            return *this;
        }

        /**
         *  @brief Apply a rotation to the transformation.
         *  @param q Rotation quaternion to apply
         *  @return Reference to this transformation after rotation for chaining
         */
        constexpr Transform<T> &rotate(cobalt::math::geometry::Quaternion<T> rotation) {
            q_ *= rotation;
            return *this;
        } 

        /**
         *  @brief Apply a translation to the transformation.
         *  @param translation Translation vector to apply
         *  @return Reference to this transformation after translation for chaining
         */
        constexpr Transform<T> &translate(cobalt::math::linear_algebra::Vector<3, T> translation) {
            t_ += cobalt::math::geometry::rotate(q_, translation);
            return *this;
        }

        // ---------------- Accessors ----------------
        /**
         *  @brief Access to the rotation matrix part of the transformation
         *  @return Reference to rotation matrix `R` associated with the transformation
         */
        constexpr cobalt::math::geometry::Quaternion<T> &rotation() {
            return q_;
        }

        /**
         *  @brief Access to the translation vector part of the transformation
         *  @return Reference to translation vector `t` associated with the transformation
         */
        constexpr cobalt::math::linear_algebra::Vector<3, T> &translation() {
            return t_;
        }

        /**
         *  @brief Const access to the rotation matrix part of the transformation
         *  @return Const reference to rotation matrix `q` associated with the transformation
         */
        const cobalt::math::geometry::Quaternion<T> &rotation() const {
            return q_;
        }

        /**
         *  @brief Const access to the translation vector part of the transformation
         *  @return Const reference to translation vector `t` associated with the transformation
         */
        const cobalt::math::linear_algebra::Vector<3, T> &translation() const {
            return t_;
        }

        // ---------------- Member Overloads ----------------
        /**
         *  @brief In-place multiplication of two transformations (concatenation)
         *  @param rhs Right-hand side transformation
         *  @return Reference to this transformation after multiplication
         *  @note The resulting transformation is equivalent to first applying `this`, then `rhs`
         */
        Transform<T> &operator*=(const Transform<T> &rhs) {
            t_ += cobalt::math::geometry::rotate(q_, rhs.t_);
            q_ *= rhs.q_;

            return *this;
        }

        // ---------------- Member Functions ----------------
        /**
         *  @brief Apply the transformation to a 3-Vector
         *  @param v 3-Vector to transform
         *  @return Transformed 3-Vector
         *  @note The transformation is applied as `q*v + t`
         */
        inline cobalt::math::linear_algebra::Vector<3, T> apply(const cobalt::math::linear_algebra::Vector<3, T> &v) const {
            cobalt::math::linear_algebra::Vector<3, T> qv = cobalt::math::geometry::rotate(q_, v);
            return (qv + t_);
        }
};

} // cobalt::math::geometry