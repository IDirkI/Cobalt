#pragma once

#include <array>

#include "../../config.hpp"

#include "../quaternion/quaternion.hpp"
#include "../quaternion/quaternion_ops.hpp"

#include "../../linear_algebra/matrix/matrix.hpp"
#include "../../linear_algebra/vector/vector.hpp"

namespace cobalt::math::geometry {

// --------------------------------------
//      Homogeneous Transformations    
// --------------------------------------
/**
 *  @brief Homogeneous transformation matrix.
 *  @tparam T Element type (default float).
 */
template<typename T = def_floating, typename = std::enable_if_t<Scalar<T>>>
struct Transform {
    private:
        cobalt::math::linear_algebra::Matrix<3, 3, T> R_;
        cobalt::math::linear_algebra::Vector<3, T> t_;

    public: 
        // ---------------- Constructors ----------------
        /**
         *  @brief Construct an identity transformation matrix.
         */
        constexpr Transform() noexcept 
            : R_(cobalt::math::linear_algebra::Matrix<3, 3, T>::eye()), t_(cobalt::math::linear_algebra::Vector<3, T>::zero()) {}

        /**
         *  @brief Construct an transformation matrix.
         *  @param R Rotation matrix associated with the transform
         *  @param t Translation vector associated with the transformn
         */
        constexpr Transform(const cobalt::math::linear_algebra::Matrix<3, 3, T> &R, const cobalt::math::linear_algebra::Vector<3, T> &t) noexcept
            : R_(R), t_(t){}

        // ---------------- Static Factories ----------------
        /**
         *  @brief Construct an identity transformation.
         */
        static constexpr Transform<T> eye() {
            return Transform<T>();
        }

        /**
         *  @brief Construct a rotation-transformation from given unit quaternion.
         *  @param q Unit quaternion representing the rotation/orientation
         */
        static constexpr Transform<T> fromQuaternion(const Quaternion &q) {
            cobalt::math::geometry::Quaternion qn = normalize(q);
            cobalt::math::linear_algebra::Matrix<3, 3, T> R;
            
            R(0,0) = (1 - 2*(qn.y()*qn.y() + qn.z()*qn.z()));
            R(1,0) = 2*(qn.x()*qn.y() + qn.z()*qn.w());
            R(2,0) = 2*(qn.x()*qn.z() - qn.y()*qn.w());

            R(0,1) = 2*(qn.x()*qn.y() - qn.z()*qn.w());
            R(1,1) = (1 - 2*(qn.x()*qn.x() + qn.z()*qn.z()));
            R(2,1) = 2*(qn.y()*qn.z() + qn.x()*qn.w());

            R(0,2) = 2*(qn.x()*qn.z() + qn.y()*qn.w());
            R(1,2) = 2*(qn.y()*qn.z() - qn.x()*qn.w());
            R(2,2) = (1 - 2*(qn.x()*qn.x() + qn.y()*qn.y()));
        
            return Transform<T>(R, cobalt::math::linear_algebra::Vector<3, T>::zero());
        }

        /**
         *  @brief Construct a pure rotation transformation from given rotation vector.
         *  @param v Vector representing the rotation/orientation
         * 
         *  The direction of the vector is the axis while the norm of the vector is the angle
         */
        static constexpr Transform<T> fromPose(const cobalt::math::linear_algebra::Vector<3, T> &position, const cobalt::math::geometry::Quaternion &orientation) {
            Transform<T> H = fromQuaternion(orientation);
            H.t_ = position;

            return H;
        }

        /**
         *  @brief Construct a pure rotation transformation from given rotation vector.
         *  @param v Vector representing the rotation/orientation
         * 
         *  The direction of the vector is the axis while the norm of the vector is the angle
         */
        static constexpr Transform<T> fromRotationVector(const cobalt::math::linear_algebra::Vector<3, T> &v) {
            Quaternion q = Quaternion::fromRotationVector(v);

            return fromQuaternion(q);
        }

        /**
         *  @brief Construct a translation-transformation from given vector.
         *  @param v Vector representing the translation in the transformation
         */
        static constexpr Transform<T> fromTranslationVector(const cobalt::math::linear_algebra::Vector<3, T> &t) {
            return Transform<T>(cobalt::math::linear_algebra::Matrix<3, 3, T>::eye(), t);
        }

        /**
         *  @brief Construct a rotation-transformation along the X-axis from a given angle.
         *  @param angle Angle to rotate around X-axis
         */
        static constexpr Transform<T> fromRotationX(float angle) {
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
        static constexpr Transform<T> fromRotationY(float angle) {
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
        static constexpr Transform<T> fromRotationZ(float angle) {
            cobalt::math::linear_algebra::Matrix<3, 3, T> R = cobalt::math::linear_algebra::Matrix<3, 3, T>::eye();

            R(0,0) = std::cos(angle);
            R(0,1) = -std::sin(angle);
            R(1,0) = std::sin(angle);
            R(1,1) = std::cos(angle);
        
            return Transform<T>(R, cobalt::math::linear_algebra::Vector<3, T>::zero());
        }

        // ---------------- Accessors ----------------
        /**
         *  @brief Access to the rotation matrix part of the transformation
         *  @return Reference to rotation matrix `R` associated with the transformation
         */
        constexpr cobalt::math::linear_algebra::Matrix<3, 3, T> &rotation() {
            return R_;
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
         *  @return Const reference to rotation matrix `R` associated with the transformation
         */
        const cobalt::math::linear_algebra::Matrix<3, 3, T> &rotation() const {
            return R_;
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
         *  @brief Right-multiply another transform matrix(4x4) to this transform matrix(4x4).
         *  @return Resulting transform matrix (4x4)
         */
        Transform<T> &operator*=(const Transform<T> &rhs) {
            t_ += R_*rhs.t_;
            R_ = R_*rhs.R_;

            return *this;
        }

        // ---------------- Member Functions ----------------
        /**
         *  @brief Apply a homogeneous transformation (rotate + translate) to a 3-Vector
         *  @return Transformed 3-Vector
         */
        cobalt::math::linear_algebra::Vector<3, T> apply(cobalt::math::linear_algebra::Vector<3, T> v) const {
            cobalt::math::linear_algebra::Vector<3, T> Rv = R_*v;
            return (Rv + t_);
        }
};

} // cobalt::math::geometry