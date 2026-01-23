#pragma once

#include <math.h>

#include "quaternion.hpp"
#include "quaternion_ops.hpp"

#include "../../linear_algebra/vector/vector.hpp"

#include "../../linear_algebra/matrix/matrix.hpp"

namespace cobalt::math::geometry {

// ---------------- Clamps ----------------
/**
 *  @brief Clamp the components of a quaternion between min and max values
 *  @param q Quaternion to clamp
 *  @param minVal Lower clamp bound
 *  @param maxVal Upper clamp bound
 *  @return Clamped quaternion
 *  @note Asserts if minVal > maxVal. If this occurs, the identity quaternion is returned.
 */
template<typename T = def_floating, typename = std::enable_if_t<Floating<T>>>
    inline Quaternion<T> clamp(const Quaternion<T> &q, T minVal, T maxVal) noexcept {
        assert(minVal <= maxVal && "[QUATERNION Error] : Minimum clamp value is greater than maximum clamp value.");

        if(minVal > maxVal) { return Quaternion<T>::eye(); }

        return Quaternion<T>(
            (q.w() > maxVal) ?maxVal :((q.w() < minVal) ?minVal :q.w()),
            (q.x() > maxVal) ?maxVal :((q.x() < minVal) ?minVal :q.x()),
            (q.y() > maxVal) ?maxVal :((q.y() < minVal) ?minVal :q.y()),
            (q.z() > maxVal) ?maxVal :((q.z() < minVal) ?minVal :q.z())
        );
    }

/**
 *  @brief Clamp the components of a quaternion between an absolute max value
 *  @param q Quaternion to clamp
 *  @param maxVal Upper/Lower absolute clamp bound
 *  @return Clamped quaternion
 */
template<typename T = def_floating, typename = std::enable_if_t<Floating<T>>>
    inline Quaternion<T> clamp(const Quaternion<T> &q, T maxVal) noexcept {
        return clamp(q, -maxVal, maxVal);
    }

/**
 *  @brief Clamp the rotation angle of a quaternion between min and max angles (in radians)
 *  @param q Quaternion to clamp
 *  @param minAngle Minimum rotation angle in radians
 *  @param maxAngle Maximum rotation angle in radians
 *  @return Clamped quaternion
 *  @note Asserts if minAngle > maxAngle. If this occurs, the identity quaternion is returned.
 */
template<typename T = def_floating, typename = std::enable_if_t<Floating<T>>>
    inline Quaternion<T> clampRotation(const Quaternion<T> &q, T minAngle, T maxAngle) noexcept {
        assert(minAngle <= maxAngle && "[QUATERNION Error] : Minimum clamp angle is greater than maximum clamp angle.");

        if(minAngle > maxAngle) { Quaternion<T>::eye(); }

        Quaternion<T> qn = normalize(q);

        T s = std::sqrt(1 - qn.w()*qn.w());
        

        T angle = 2.0f*std::acos(qn.w());
        cobalt::math::linear_algebra::Vector<3,T> axis;

        if(s < epsilon_<T>) {
            axis = cobalt::math::linear_algebra::Vector<3,T>::unitX();
        }
        else {
            axis = qn.vector() / s;
        }

        T clampedAngle = (angle > maxAngle) ?maxAngle :((angle < minAngle) ?minAngle :angle);

        return Quaternion<T>::fromAxisAngle(axis, clampedAngle);
        
    }

/**
 *  @brief Clamp the rotation angle of a quaternion between an abolute max angle (in radians)
 *  @param q Quaternion to clamp
 *  @param maxAngle Maximum rotation angle in radians
 *  @return Clamped quaternion
 */
template<typename T = def_floating, typename = std::enable_if_t<Floating<T>>>
    inline Quaternion<T> clampRotation(const Quaternion<T> &q, T maxAngle) noexcept {
        return clampRotation(q, -maxAngle, maxAngle);
    }

// ---------------- Element Wise ----------------
/**
 *  @brief Round the components of a quaternion to the nearest integer
 *  @param q Quaternion<T> to round
 *  @return Rounded quaternion
 */
template<typename T = def_floating, typename = std::enable_if_t<Floating<T>>>
    inline Quaternion<T> round(const Quaternion<T> &q) noexcept {
        return Quaternion<T>(std::round(q.w()),
                        std::round(q.x()),
                        std::round(q.y()),
                        std::round(q.z())
                        );
    }

/**
 *  @brief Ceil the components of a quaternion to the nearest integer
 *  @param q Quaternion<T> to ceil
 *  @return Ceiled quaternion
 */
template<typename T = def_floating, typename = std::enable_if_t<Floating<T>>>
    inline Quaternion<T> ceil(const Quaternion<T> &q) noexcept {
        return Quaternion<T>(std::ceil(q.w()),
                        std::ceil(q.x()),
                        std::ceil(q.y()),
                        std::ceil(q.z())
                        );
    }

/**
 *  @brief Floor the components of a quaternion to the nearest integer
 *  @param q Quaternion<T> to floor
 *  @return Floored quaternion
 */
template<typename T = def_floating, typename = std::enable_if_t<Floating<T>>>
    inline Quaternion<T> floor(const Quaternion<T> &q) noexcept {
        return Quaternion<T>(std::floor(q.w()),
                        std::floor(q.x()),
                        std::floor(q.y()),
                        std::floor(q.z())
                        );
    }

/**
 *  @brief Sets the components of a quaternion to a clean zero if they are very close to zero
 *  @param q Quaternion<T> to clean
 *  @return Cleaned quaternion
 */
template<typename T = def_floating, typename = std::enable_if_t<Floating<T>>>
    inline Quaternion<T> cleanZero(const Quaternion<T> &q) noexcept {
        return Quaternion<T>(
            (std::abs(q.w()) < epsilon_<T>) ?0.0f :q.w(),
            (std::abs(q.x()) < epsilon_<T>) ?0.0f :q.x(),
            (std::abs(q.y()) < epsilon_<T>) ?0.0f :q.y(),
            (std::abs(q.z()) < epsilon_<T>) ?0.0f :q.z()
        );
    }

// ---------------- Angles & Difference ----------------
/** 
 *  @brief Calculate the angled distance between two quaternions
 *  @param q First quaternion
 *  @param p Second quaternion
 *  @return Angled distance in radians
*/
template<typename T = def_floating, typename = std::enable_if_t<Floating<T>>>
    inline T angledDistance(const Quaternion<T> &q, const Quaternion<T> &p) noexcept {
        Quaternion<T> qn = normalize(q);
        Quaternion<T> pn = normalize(p);

        T d = std::abs(dot(qn, pn));
        
        if(d > 1.0f) { d = 1.0f; }
        if(d < -1.0f) { d = -1.0f; }

        return 2.0f * std::acos(d);
    }

/** 
 *  @brief Calculate the difference quaternion from q to p
 *  @param q First quaternion
 *  @param p Second quaternion
 *  @return Difference quaternion q^-1 * p
 *  @note The difference represents the rotation needed to go from q to p
*/
template<typename T = def_floating, typename = std::enable_if_t<Floating<T>>>
    inline Quaternion<T> difference(const Quaternion<T> &q, const Quaternion<T> &p) noexcept {
        Quaternion<T> qn = normalize(q);
        Quaternion<T> pn = normalize(p);

        return pn * inv(qn);
    }

/**
 *  @brief Get the shortest path quaternion between two quaternions
 *  @param q First quaternion
 *  @param p Second quaternion
 *  @return Shortest path quaternion
 */
template<typename T = def_floating, typename = std::enable_if_t<Floating<T>>>
    inline Quaternion<T> shortestPath(const Quaternion<T> &q, const Quaternion<T> &p) noexcept {
        Quaternion<T> dq = difference(q, p);

        if(dq.w() < static_cast<T>(0)) {
            dq = -dq;
        }

        return dq;
    }

// ---------------- Dynamics & Motion ----------------
/** 
 *  @brief Calculate the angular velocity vector from a quaternion and its time derivative
 *  @param q Quaternion<T>
 *  @param qDot Time derivative of the quaternion
 *  @return Angular velocity vector in radians per second
*/
template<typename T = def_floating, typename = std::enable_if_t<Floating<T>>>
inline cobalt::math::linear_algebra::Vector<3, T> angularVelocity(const Quaternion<T> &q, const Quaternion<T> &qDot) noexcept {
    Quaternion<T> qConj = conj(q);
    Quaternion<T> angVel = static_cast<T>(2)*qDot*qConj;

    return cobalt::math::linear_algebra::Vector<3, T>(angVel.x(), angVel.y(), angVel.z());
}

/**
 *  @brief Decompose a quaternion into its swing and twist components around a specified axis
 *  @param q Quaternion to decompose
 *  @param axis Axis to decompose around (must be normalized)
 *  @param swing Output swing component
 *  @param twist Output twist component
 */
template<typename T = def_floating, typename = std::enable_if_t<Floating<T>>>
    inline void swingTwist(const Quaternion<T> &q, const cobalt::math::linear_algebra::Vector<3, T> &axis, Quaternion<T> &swing, Quaternion<T> &twist) noexcept {
        cobalt::math::linear_algebra::Vector<3, T> p = q.vector();
        cobalt::math::linear_algebra::Vector<3, T> proj = dot(p, axis) * axis;

        twist = Quaternion<T>(q.w(), proj.x(), proj.y(), proj.z());
        T normTwist = norm(twist);

        if(normTwist < epsilon_<T>) {
            twist = Quaternion<T>::eye();
            swing = q;
        } else {
            twist = twist / normTwist;
            swing = q * conj(twist);
        }
    }

// ---------------- Interpolation ----------------
/**
 *  @brief Spherical linear interpolation (slerp) between two quaternions
 *  @param q Start quaternion
 *  @param p End quaternion
 *  @param t Interpolation factor (0.0 → q, 1.0 → p)
 *  @return Spherically interpolated quaternion between q and p at t
 *  @note Both input quaternions are normalized before interpolation
 */
template<typename T = def_floating, typename = std::enable_if_t<Floating<T>>>
    inline Quaternion<T> slerp(const Quaternion<T> &q, const Quaternion<T> &p, T t) {
        Quaternion<T> qn = normalize(q);
        Quaternion<T> pn = normalize(p);

        T dotProd = dot(qn, pn);

        if(dotProd < epsilon_<T>) {
            pn = -pn;
            dotProd = -dotProd;
        }

        if(dotProd > 1 - epsilon_<T>) {
            Quaternion<T> result = qn + t * (pn - qn);
            return normalize(result);
        }

        T theta0 = std::acos(dotProd);
        T theta = theta0*t;

        Quaternion<T> qPerp = pn - dotProd * qn;
        qPerp = normalize(qPerp);

        return qn * std::cos(theta) + qPerp * std::sin(theta);
    }

/**
 *  @brief Normalized linear interpolation (nlerp) between two quaternions
 *  @param q Start quaternion
 *  @param p End quaternion
 *  @param t Interpolation factor (0.0 → q, 1.0 → p)
 *  @return Normalized linearly interpolated quaternion between q and p at t
 *  @note Both input quaternions are normalized before interpolation
 */
template<typename T = def_floating, typename = std::enable_if_t<Floating<T>>>
    inline Quaternion<T> nlerp(const Quaternion<T> &q, const Quaternion<T> &p, T t) {
        Quaternion<T> qn = normalize(q);
        Quaternion<T> pn = normalize(p);

        if(dot(qn, pn) < epsilon_<T>) {
            pn = -pn;
        }

        Quaternion<T> result = (static_cast<T>(1) - t) * qn + t * pn;
        return normalize(result);
    }

/**
 *  @brief Linear interpolation (lerp) between two quaternions
 *  @param q Start quaternion
 *  @param p End quaternion
 *  @param t Interpolation factor (0.0 → q, 1.0 → p)
 *  @return (Non-normalized) Linearly interpolated quaternion between q and p at t
 */
template<typename T = def_floating, typename = std::enable_if_t<Floating<T>>>
    inline Quaternion<T> lerp(const Quaternion<T> &q, const Quaternion<T> &p, T t) {
        return (static_cast<T>(1) - t) * q + t * p;
    }

// ---------------- Conversions ----------------
/**
 *  @brief Convert a quaternion to a std::array
 *  @param q Quaternion<T> to convert
 *  @return std::array containing the quaternion components in the order [w, x, y, z]
 */
template<typename T = def_floating, typename = std::enable_if_t<Floating<T>>>
    inline std::array<T, 4> toArray(const Quaternion<T> &q) noexcept {
        return { q.w(), q.x(), q.y(), q.z() };
    }

/**
 *  @brief Convert a quaternion to a rotation matrix
 *  @param q Quaternion<T> to convert
 *  @return 3x3 rotation matrix
 *  @note The input quaternion is normalized before conversion
 */
template<typename T = def_floating, typename = std::enable_if_t<Floating<T>>>
    inline cobalt::math::linear_algebra::Matrix<3, 3, T> toMatrix(const Quaternion<T> &q) noexcept {
        Quaternion<T> qn = normalize(q);

        T w = qn.w();
        T x = qn.x();
        T y = qn.y();
        T z = qn.z();

        cobalt::math::linear_algebra::Matrix<3, 3, T> R;
        R(0,0) = 1 - 2*(y*y + z*z);
        R(0,1) = 2*(x*y - z*w);
        R(0,2) = 2*(x*z + y*w);

        R(1,0) = 2*(x*y + z*w);
        R(1,1) = 1 - 2*(x*x + z*z);
        R(1,2) = 2*(y*z - x*w);

        R(2,0) = 2*(x*z - y*w);
        R(2,1) = 2*(y*z + x*w);
        R(2,2) = 1 - 2*(x*x + y*y);

        return R;
    }

/**
 *  @brief Convert a quaternion to its axis-angle representation as a 3-Vector
 *  @param q Quaternion<T> to convert
 *  @return 3-Vector representing the rotation axis scaled by the rotation angle in radians
 *  @note The input quaternion is normalized before conversion
 *  @note If the quaternion represents no rotation, a zero vector is returned 
 */
template<typename T = def_floating, typename = std::enable_if_t<Floating<T>>>
    inline cobalt::math::linear_algebra::Vector<3, T> toRotationVector(const Quaternion<T> &q) noexcept {
        Quaternion<T> qn = normalize(q);

        T angle = 2*std::acos(qn.w());
        T s = std::sqrt(1 - qn.w()*qn.w());
        if(s < epsilon_<T>) {
            return cobalt::math::linear_algebra::Vector<3, T>::zero();
        }

        cobalt::math::linear_algebra::Vector<3, T> axis = {qn.x()/s, qn.y()/s, qn.z()/s};

        return axis * angle;
    }

/**
 *  @brief Convert a quaternion to its rotation angle
 *  @param q Quaternion to convert
 *  @return Rotation angle in radians
 *  @note The input quaternion is normalized before conversion
 */
template<typename T = def_floating, typename = std::enable_if_t<Floating<T>>>
    inline T toAngle(const Quaternion<T> &q) noexcept {
        Quaternion<T> qn = normalize(q);

        return static_cast<T>(2)*std::acos(qn.w());
    }

/**
 *  @brief Convert a quaternion to its rotation axis
 *  @param q Quaternion to convert
 *  @return Normalized rotation axis as a 3-Vector
 *  @note The input quaternion is normalized before conversion
 *  @note If the quaternion represents no rotation, the +x axis is returned
 */
template<typename T = def_floating, typename = std::enable_if_t<Floating<T>>>
    inline cobalt::math::linear_algebra::Vector<3, T> toAxis(const Quaternion<T> &q) noexcept {
        Quaternion<T> qn = normalize(q);

        T s = std::sqrt(1 - qn.w()*qn.w());
        if(s < epsilon_<T>) {
            return cobalt::math::linear_algebra::Vector<3, T>::unitX();
        }

        return cobalt::math::linear_algebra::Vector<3, T>{ qn.x()/s, qn.y()/s, qn.z()/s };
    }

/**
 *  @brief Convert a quaternion to Euler angles (roll, pitch, yaw) in ZYX order
 *  @param q Quaternion to convert
 *  @param roll Output roll angle in radians
 *  @param pitch Output pitch angle in radians
 *  @param yaw Output yaw angle in radians
 *  @note The input quaternion is normalized before conversion
 */
template<typename T = def_floating, typename = std::enable_if_t<Floating<T>>>
    inline void toEuler(const Quaternion<T> &q, T &roll, T &pitch, T &yaw) noexcept {
        Quaternion<T> qn = normalize(q);

        roll = std::atan2(2*(q.w()*q.x() + q.y()*q.z()), 1 - 2*(q.x()*q.x() + q.y()*q.y()));
        pitch = std::asin(2*(q.w()*q.y() - q.x()*q.z()));
        yaw = std::atan2(2*(q.w()*q.z() + q.x()*q.y()), 1 - 2*(q.y()*q.y() + q.z()*q.z()));
    }

// ---------------- Checks ----------------
/**
 *  @brief Check if a quaternion is a zero-quaternion (0 + 0i + 0j + 0k)
 *  @return `true` if the quaternion is zero, `false` otherwise
 */
template<typename T = def_floating, typename = std::enable_if_t<Floating<T>>>
    inline bool isZero(const Quaternion<T> &q) noexcept { 
        if(std::abs(q.w()) > epsilon_<T>) { return false; }
        if(std::abs(q.x()) > epsilon_<T>) { return false; }
        if(std::abs(q.y()) > epsilon_<T>) { return false; }
        if(std::abs(q.z()) > epsilon_<T>) { return false; }
        return true;
    }

/**
 *  @brief Check if a quaternion is normalized (unit length)
 *  @return `true` if the quaternion is normalized, `false` otherwise
 */
template<typename T = def_floating, typename = std::enable_if_t<Floating<T>>>
    inline bool isNormalized(const Quaternion<T> &q) noexcept { 
        float n = norm(q);
        return (std::abs(n - static_cast<T>(1)) < epsilon_<T>);
    }


/**
 *  @brief Check if a quaternion is the identity quaternion (1 + 0i + 0j + 0k)
 *  @return `true` if the quaternion is the identity, `false` otherwise
 */
template<typename T = def_floating, typename = std::enable_if_t<Floating<T>>>
    inline bool isIdentity(const Quaternion<T> &q) noexcept { 
        if(std::abs(q.w() - static_cast<T>(1)) > epsilon_<T>) { return false; }
        if(std::abs(q.x()) > epsilon_<T>) { return false; }
        if(std::abs(q.y()) > epsilon_<T>) { return false; }
        if(std::abs(q.z()) > epsilon_<T>) { return false; }
        return true;
    }

/**
 *  @brief Check if a quaternion is purely real (w + 0i + 0j + 0k)
 *  @return `true` if the quaternion is real, `false` otherwise
 */
template<typename T = def_floating, typename = std::enable_if_t<Floating<T>>>
    inline bool isReal(const Quaternion<T> &q) noexcept { 
        if(std::abs(q.x()) > epsilon_<T>) { return false; }
        if(std::abs(q.y()) > epsilon_<T>) { return false; }
        if(std::abs(q.z()) > epsilon_<T>) { return false; }
        return true;
    }

/**
 *  @brief Check if a quaternion is purely imaginary (0 + xi + yj + zk)
 *  @return `true` if the quaternion is purely imaginary, `false` otherwise
 */
template<typename T = def_floating, typename = std::enable_if_t<Floating<T>>>
    inline bool isPure(const Quaternion<T> &q) noexcept { 
        if(std::abs(q.w()) > epsilon_<T>) { return false; }
        return true;
    }

/**
 *  @brief Check if all components of a quaternion are finite numbers
 *  @return `true` if all components are finite, `false` otherwise
 */
template<typename T = def_floating, typename = std::enable_if_t<Floating<T>>>
    inline bool isFinite(const Quaternion<T> &q) noexcept { 
        return std::isfinite(q.w()) &&
               std::isfinite(q.x()) &&
               std::isfinite(q.y()) &&
               std::isfinite(q.z());
    }

/**
 *  @brief Check if two quaternions represent the same rotation
 *  @param q First quaternion
 *  @param p Second quaternion
 *  @return `true` if they represent the same rotation, `false` otherwise
 */
template<typename T = def_floating, typename = std::enable_if_t<Floating<T>>>
    inline bool isSameRotation(const Quaternion<T> &q, const Quaternion<T> &p) noexcept { 
        Quaternion<T> qn = normalize(q);
        Quaternion<T> pn = normalize(p);

        T d = dot(qn, pn);
        return (std::abs(std::abs(d) - static_cast<T>(1)) < epsilon_<T>);
    }

} // cobalt::math::geometry 