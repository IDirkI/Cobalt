#pragma once

#include "transform.hpp"

#include "../../linear_algebra/matrix/matrix.hpp"

#include "../quaternion/quaternion.hpp"
#include "../quaternion/quaternion_util.hpp"

namespace cobalt::math::geometry {

// ---------------- Element Wise ----------------
/**
 *  @brief Sets the components of a vector to a clean zero if they are very close to zero
 *  @param H Transformation to clean
 *  @return Cleaned transformation
 */
template<typename T = def_floating, typename = std::enable_if_t<Floating<T>>>
    inline Transform<T> cleanZero(const Transform<T> &H) {
        return  Transform<T>(cleanZero(H.rotation()), cleanZero(H.translation()));
    }

// ---------------- Interpolation ----------------
/**
 *  @brief Linearly interpolate between two transformations
 *  @param H1 First transformation
 *  @param H2 Second transformation
 *  @param t Interpolation factor (0.0 to 1.0)
 *  @return Interpolated transformation
 */
template<typename T = def_floating, typename = std::enable_if_t<Floating<T>>>
    constexpr Transform<T> lerp(const Transform<T> &H1, const Transform<T> &H2, T t) {
        return Transform<T>(slerp(H1.rotation(), H2.rotation(), t), lerp(H1.translation(), H2.translation(), t));
    }

// ---------------- Conversions ----------------
/**
 *  @brief Convert a transformation into a homogeneous transformation matrix
 *  @param H Transformation to convert
 *  @return 4x4 Homogeneous transformation matrix
 */
template<typename T = def_floating, typename = std::enable_if_t<Floating<T>>>
    constexpr cobalt::math::linear_algebra::Matrix<4, 4, T> toMatrix(const Transform<T> &H) {
        cobalt::math::linear_algebra::Matrix<4, 4, T> output = cobalt::math::linear_algebra::Matrix<4, 4, T>::eye();

        cobalt::math::linear_algebra::Matrix<3, 3, T> R = toMatrix(H.rotation());

        for(index_t i = 0; i < 3; i++) {
            for(index_t j = 0; j < 3; j++) {
                output(i, j) = R(i, j);
            }
        }

        output(0, 3) = H.translation()[0];
        output(1, 3) = H.translation()[1];
        output(2, 3) = H.translation()[2];

        return output;
    }

/**
 *  @brief Convert a transformation into a rotation matrix form
 *  @param H Transformation to convert
 *  @return 3x3 Rotation matrix with translation
 */
template<typename T = def_floating, typename = std::enable_if_t<Floating<T>>>
    constexpr cobalt::math::linear_algebra::Matrix<3, 3, T> toRotationMatrix(const Transform<T> &H) {
        return toMatrix(H.rotation());
    }

/**
 *  @brief Convert a rotational transformation into a unit quaternion
 *  @param H Transformation to convert
 *  @param rotationVector Rotation axis + angle of the transformation
 *  @param translationVector Translation vector of the transformation
 */
template<typename T = def_floating, typename = std::enable_if_t<Floating<T>>>
    constexpr void toVector(const Transform<T> &H, cobalt::math::linear_algebra::Vector<3, T> &rotationVector, cobalt::math::linear_algebra::Vector<3, T> &translationVector) {
        rotationVector = toRotationVector(H.rotation());
        translationVector = H.translation();
    }

/**
 *  @brief Convert a transformation into a unit quaternion
 *  @param H Transformation to convert
 *  @param roll Output roll angle
 *  @param pitch Output pitch angle
 *  @param yaw Output yaw angle
 *  @return Unit quaternion representing the rotation of the transformation in ZYX order
 */
template<typename T = def_floating, typename = std::enable_if_t<Floating<T>>>
    inline void toEuler(const Transform<T> &H, T &roll, T &pitch, T &yaw) {
        toEuler(H.rotation(), roll, pitch, yaw);
    }

// ---------------- Checks ----------------
/**
 *  @brief Check if a transformation is the identity transformation
 *  @return `true` if the transformation is identity, `false` otherwise
 */
template<typename T = def_floating, typename = std::enable_if_t<Floating<T>>>
    inline bool isIdentity(const Transform<T> &H) {
        return (isIdentity(H.rotation()) && isZero(H.translation()));
    }

/**
 *  @brief Check if a transformation is the purely rotational
 *  @return `true` if the transformation is purely rotational, `false` otherwise
 */
template<typename T = def_floating, typename = std::enable_if_t<Floating<T>>>
    inline bool isPureRotation(const Transform<T> &H) {
        return isZero(H.translation());
    }

/**
 *  @brief Check if a transformation is the purely translational
 *  @return `true` if the transformation is purely translational, `false` otherwise
 */
template<typename T = def_floating, typename = std::enable_if_t<Floating<T>>>
    inline bool isPureTranslation(const Transform<T> &H) {
        return isIdentity(H.rotation());
    }

/**
 *  @brief Check if a transformation is valid (i.e., has a normalized rotation quaternion)
 *  @return `true` if the transformation is valid, `false` otherwise
 */
template<typename T = def_floating, typename = std::enable_if_t<Floating<T>>>
    inline bool isFinite(const Transform<T> &H) {
        return (isFinite(H.rotation()) && isFinite(H.translation()));
    }

/**
 *  @brief Check if a transformation is valid (i.e., has a normalized rotation quaternion and is finite)
 *  @return `true` if the transformation is valid, `false` otherwise
 */
template<typename T = def_floating, typename = std::enable_if_t<Floating<T>>>
    inline bool isValid(const Transform<T> &H) {
        return (isNormalized(H.rotation()) && isFinite(H));
    }

} // cobalt::math::geometry