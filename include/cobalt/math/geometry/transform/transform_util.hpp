#pragma once

#include "transform.hpp"

#include "../../linear_algebra/matrix/matrix.hpp"

#include "../quaternion/quaternion.hpp"
#include "../quaternion/quaternion_util.hpp"

namespace cobalt::math::geometry {

// ---------------- Element Wise ----------------
/**
 *  @brief Sets the components of a transform to a clean zero if they are very close to zero
 *  @param H Quaternion to clean
 */
template<typename T = float, typename = std::enable_if_t<Scalar<T>>>
    inline Transform<T> cleanZero(const Transform<T> &H) {
        return  Transform<T>(cleanZero(H.rotation()), cleanZero(H.translation()));
    }


// ---------------- Conversions ----------------
/**
 *  @brief Convert a transformation into a 4x4 Matrix form
 *  @param H Transformation to convert
 *  @return 4x4 Homogeneous transformation matrix
 */
template<typename T = float, typename = std::enable_if_t<Scalar<T>>>
    constexpr cobalt::math::linear_algebra::Matrix<4, 4, T> toMatrix(const Transform<T> &H) {
        cobalt::math::linear_algebra::Matrix<4, 4, T> output = cobalt::math::linear_algebra::Matrix<4, 4, T>::eye();

        for(index_t i = 0; i < 3; i++) {
            for(index_t j = 0; j < 3; j++) {
                output(i, j) = (H.rotation())(i, j);
            }
        }

        output(0, 3) = H.translation()[0];
        output(1, 3) = H.translation()[1];
        output(2, 3) = H.translation()[2];

        return output;
    }

/**
 *  @brief Convert a rotational transformation into a unit quaternion
 *  @param H Transformation to convert
 *  @return Unit quaternion of the rotation
 */
template<typename T = float, typename = std::enable_if_t<Scalar<T>>>
    constexpr cobalt::math::geometry::Quaternion toQuaternion(const Transform<T> &H) {
        return cobalt::math::geometry::Quaternion::fromRotationMatrix(H.rotation());
    }

/**
 *  @brief Convert a rotational transformation into a unit quaternion
 *  @param H Transformation to convert
 *  @param rotationVector Rotation axis + angle of the transformation
 *  @param translationVector Translation vector of the transformation
 */
template<typename T = float, typename = std::enable_if_t<Scalar<T>>>
    constexpr void toVector(const Transform<T> &H, cobalt::math::linear_algebra::Vector<3, T> &rotationVector, cobalt::math::linear_algebra::Vector<3, T> &translationVector) {
        rotationVector = toVector(toQuaternion(H));
        translationVector = H.translation();
    }

/**
 *  @brief Convert a rotational transformation to its euler angles
 *  @param H Transformation to convert
*   @note (r,p,y) in ZYX sequence 
 */
template<typename T = float, typename = std::enable_if_t<Scalar<T>>>
    inline void toEuler(const Transform<T> &H, float &roll, float &pitch, float &yaw) {
        toEuler(toQuaternion(H), roll, pitch, yaw);
    }

// ---------------- Checks ----------------
/**
 *  @brief Check if a transformation is the identity transformation
 */
template<typename T = float, typename = std::enable_if_t<Scalar<T>>>
    inline void isIdentity(const Transform<T> &H) {
        return ((H.rotation() == cobalt::math::linear_algebra::Matrix<3, 3, T>::eye()) &&
                (H.translation() == cobalt::math::linear_algebra::Vector<3, T>::zero())); 
    }

/**
 *  @brief Check if a transformation is the purely rotational
 */
template<typename T = float, typename = std::enable_if_t<Scalar<T>>>
    inline void isPureRotation(const Transform<T> &H) {
        return (H.translation() == cobalt::math::linear_algebra::Vector<3, T>::zero());
    }

/**
 *  @brief Check if a transformation is the purely translational
 */
template<typename T = float, typename = std::enable_if_t<Scalar<T>>>
    inline void isPureTranslation(const Transform<T> &H) {
        return (H.rotation() == cobalt::math::linear_algebra::Matrix<3, 3, T>::eye());
    }

} // cobalt::math::geometry