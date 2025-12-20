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
inline Quaternion clamp(const Quaternion &q, float minVal, float maxVal) noexcept {
    assert(minVal <= maxVal && "[QUATERNION Error] : Minimum clamp value is greater than maximum clamp value.");

    if(minVal > maxVal) { return Quaternion::eye(); }

    return Quaternion(
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
inline Quaternion clamp(const Quaternion &q, float maxVal) noexcept {
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
inline Quaternion clampRotation(const Quaternion &q, float minAngle, float maxAngle) noexcept {
    assert(minAngle <= maxAngle && "[QUATERNION Error] : Minimum clamp angle is greater than maximum clamp angle.");

    if(minAngle > maxAngle) { Quaternion::eye(); }

    Quaternion qn = normalize(q);

    float s = std::sqrt(1 - qn.w()*qn.w());
    

    float angle = 2.0f*std::acos(qn.w());
    cobalt::math::linear_algebra::Vector<3> axis;

    if(s < epsilon_<>) {
        axis = cobalt::math::linear_algebra::Vector<3>::unitX();
    }
    else {
        axis = qn.vector() / s;
    }

    float clampedAngle = (angle > maxAngle) ?maxAngle :((angle < minAngle) ?minAngle :angle);

    return Quaternion::fromAxisAngle(axis, clampedAngle);
    
}

/**
 *  @brief Clamp the rotation angle of a quaternion between an abolute max angle (in radians)
 *  @param q Quaternion to clamp
 *  @param maxAngle Maximum rotation angle in radians
 *  @return Clamped quaternion
 */
inline Quaternion clampRotation(const Quaternion &q, float maxAngle) noexcept {
    return clampRotation(q, -maxAngle, maxAngle);
}

// ---------------- Element Wise ----------------
/**
 *  @brief Round the components of a quaternion to the nearest integer
 *  @param q Quaternion to round
 */
inline Quaternion round(const Quaternion &q) noexcept {
    return Quaternion(std::round(q.w()),
                      std::round(q.x()),
                      std::round(q.y()),
                      std::round(q.z())
                    );
}

/**
 *  @brief Ceil the components of a quaternion to the nearest integer
 *  @param q Quaternion to ceil
 */
inline Quaternion ceil(const Quaternion &q) noexcept {
    return Quaternion(std::ceil(q.w()),
                      std::ceil(q.x()),
                      std::ceil(q.y()),
                      std::ceil(q.z())
                    );
}

/**
 *  @brief Floor the components of a quaternion to the nearest integer
 *  @param q Quaternion to floor
 */
inline Quaternion floor(const Quaternion &q) noexcept {
    return Quaternion(std::floor(q.w()),
                      std::floor(q.x()),
                      std::floor(q.y()),
                      std::floor(q.z())
                    );
}

/**
 *  @brief Sets the components of a quaternion to a clean zero if they are very close to zero
 *  @param q Quaternion to clean
 */
inline Quaternion cleanZero(const Quaternion &q) noexcept {
    return Quaternion(
        (std::abs(q.w()) < epsilon_<>) ?0.0f :q.w(),
        (std::abs(q.x()) < epsilon_<>) ?0.0f :q.x(),
        (std::abs(q.y()) < epsilon_<>) ?0.0f :q.y(),
        (std::abs(q.z()) < epsilon_<>) ?0.0f :q.z()
    );
}

// ---------------- Angles & Difference ----------------
/** 
 *  @brief Calculate the angled distance between two quaternions
 *  @param q First quaternion
 *  @param p Second quaternion
 *  @return Angled distance in radians
*/
inline float angledDistance(const Quaternion &q, const Quaternion &p) noexcept {
    Quaternion qn = normalize(q);
    Quaternion pn = normalize(p);

    float d = std::abs(dot(qn, pn));
    
    if(d > 1.0f) { d = 1.0f; }
    if(d < -1.0f) { d = -1.0f; }

    return 2.0f * std::acos(d);
}

/** 
 *  @brief Calculate the difference quaternion from q to p (i.e., the rotation needed to go from q to p)
 *  @param q Source quaternion
 *  @param p Destination quaternion
 *  @return Difference quaternion (p * q^-1)
*/
inline Quaternion difference(const Quaternion &q, const Quaternion &p) noexcept {
    return p*inv(q);
}

/**
 *  @brief Choose the shortest rotation between two quaternions
 *  @param q First quaternion
 *  @param p Second quaternion
 */
inline Quaternion shortestPath(const Quaternion &q, const Quaternion &p) noexcept {
    if(dot(q, p) < 0.0f) { return -p; } 
    else { return p; }
}

// ---------------- Dynamics & Motion ----------------
/** 
 *  @brief Calculate the angular velocity represented by a quaternion and its time derivative
 *  @param q Quaternion
 *  @param qDot Time derivative of the quaternion
 *  @return 3-Vector representing angular velocity in radians per second
*/
inline cobalt::math::linear_algebra::Vector<3, float> angularVelocity(const Quaternion &q, const Quaternion &qDot) noexcept {
    Quaternion omega = 2.0f * qDot * conj(q);
    return omega.vector();
}

/**
 *  @brief Decompose a quaternion into its swing and twist components around a specified axis
 *  @param q Quaternion to decompose
 *  @param axis Axis to decompose around (must be normalized)
 *  @param swing Output swing component
 *  @param twist Output twist component
 */
template<typename T>
    inline void swingTwist(const Quaternion &q, const cobalt::math::linear_algebra::Vector<3, T> &axis, Quaternion &swing, Quaternion &twist) noexcept {
        cobalt::math::linear_algebra::Vector<3, T> p = q.vector();
        cobalt::math::linear_algebra::Vector<3, T> proj = dot(p, axis) * axis;

        twist = Quaternion(q.w(), proj.x(), proj.y(), proj.z());
        float normTwist = norm(twist);

        if(normTwist < epsilon_<>) {
            twist = Quaternion::eye();
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
 *  @return Spherically interpolated quaternion between q and p
 *  @note Both input quaternions are normalized before interpolation
 */
inline Quaternion slerp(const Quaternion &q, const Quaternion &p, float t) {
    Quaternion qn = normalize(q);
    Quaternion pn = normalize(p);

    float dotProd = dot(qn, pn);

    if(dotProd < 0.0f) {
        pn = -pn;
        dotProd = -dotProd;
    }

    if(dotProd > QUATERNION_SLERP_THRESHOLD) {
        Quaternion result = qn + t * (pn - qn);
        return normalize(result);
    }

    float theta_0 = std::acos(dotProd);
    float theta = theta_0 * t;

    Quaternion qPerp = pn - dotProd * qn;
    qPerp = normalize(qPerp);

    return qn * std::cos(theta) + qPerp * std::sin(theta);
}

/**
 *  @brief Normalized linear interpolation (nlerp) between two quaternions
 *  @param q Start quaternion
 *  @param p End quaternion
 *  @param t Interpolation factor (0.0 → q, 1.0 → p)
 *  @return Normalized linearly interpolated quaternion between q and p
 *  @note Both input quaternions are normalized before interpolation
 */
inline Quaternion nlerp(const Quaternion &q, const Quaternion &p, float t) {
    Quaternion qn = normalize(q);
    Quaternion pn = normalize(p);

    if(dot(qn, pn) < 0.0f) {
        pn = -pn;
    }

    Quaternion result = (1.0f - t) * qn + t * pn;
    return normalize(result);
}

/**
 *  @brief Llinear interpolation (lerp) between two quaternions
 *  @param q Start quaternion
 *  @param p End quaternion
 *  @param t Interpolation factor (0.0 → q, 1.0 → p)
 *  @return Normalized linearly interpolated quaternion between q and p
 */
inline Quaternion lerp(const Quaternion &q, const Quaternion &p, float t) {
    Quaternion tempQ = q;
    Quaternion tempP = p;

    if(dot(tempQ, tempP) < 0.0f) {
        tempQ = -tempP;
    }

    Quaternion result = (1.0f - t) * tempQ + t * tempP;
    return normalize(result);
}

// ---------------- Conversions ----------------
/**
 *  @brief Convert a quaternion to a rotation matrix
 *  @param q Quaternion to convert
 *  @return 3x3 rotation matrix
 */
inline cobalt::math::linear_algebra::Matrix<3, 3, float> toMatrix(const Quaternion &q) noexcept {
    Quaternion qn = normalize(q);

    float w = qn.w();
    float x = qn.x();
    float y = qn.y();
    float z = qn.z();

    cobalt::math::linear_algebra::Matrix<3, 3> R;
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
 *  @brief Convert a quaternion to a rotation vector (axis-angle representation)
 *  @param q Quaternion to convert
 *  @return 3-Vector representing axis-angle rotation
 */
inline cobalt::math::linear_algebra::Vector<3, float> toVector(const Quaternion &q) noexcept {
    Quaternion qn = normalize(q);

    float angle = 2*std::acos(qn.w());
    float s = std::sqrt(1 - qn.w()*qn.w());
    if(s < epsilon_<>) {
        return cobalt::math::linear_algebra::Vector<3, float>::zero();
    }

    cobalt::math::linear_algebra::Vector<3, float> axis = {qn.x()/s, qn.y()/s, qn.z()/s};

    return axis * angle;
}

/**
 *  @brief Convert a quaternion to its rotation angle in radians
 *  @param q Quaternion to convert
 */
inline float toAngle(const Quaternion &q) noexcept {
    Quaternion qn = normalize(q);

    return 2.0f*std::acos(qn.w());
}

/**
 *  @brief Convert a quaternion to its rotation axis
 *  @param q Quaternion to convert
 *  @return 3-Vector representing the rotation axis
 *  @note If the quaternion represents no rotation, an arbitrary unit-x axis is returned
 */
inline cobalt::math::linear_algebra::Vector<3, float> toAxis(const Quaternion &q) noexcept {
    Quaternion qn = normalize(q);

    float s = std::sqrt(1 - qn.w()*qn.w());
    if(s < epsilon_<>) {
        return cobalt::math::linear_algebra::Vector<3 ,float>::unitX();
    }

    return cobalt::math::linear_algebra::Vector<3, float>{qn.x()/s, qn.y()/s, qn.z()/s};
}

/**
 *  @brief Convert a quaternion to its euler angles
 *  @param q Unit quaternion to convert
*   @note (r,p,y) in ZYX sequence 
 */
inline void toEuler(const Quaternion &q, float &roll, float &pitch, float &yaw) noexcept {
    Quaternion qn = normalize(q);

    roll = std::atan2(2*(q.w()*q.x() + q.y()*q.z()), 1 - 2*(q.x()*q.x() + q.y()*q.y()));
    pitch = std::asin(2*(q.w()*q.y() - q.x()*q.z()));
    yaw = std::atan2(2*(q.w()*q.z() + q.x()*q.y()), 1 - 2*(q.y()*q.y() + q.z()*q.z()));
}

// ---------------- Checks ----------------
/**
 *  @brief Check if a quaternion is zero (0 + 0i + 0j + 0k)
 */
inline bool isZero(const Quaternion &q) noexcept { 
    if(std::abs(q.w()) > epsilon_<>) { return false; }
    if(std::abs(q.x()) > epsilon_<>) { return false; }
    if(std::abs(q.y()) > epsilon_<>) { return false; }
    if(std::abs(q.z()) > epsilon_<>) { return false; }
    return true;
}

/**
 *  @brief Check if a quaternion is unitary. (norm = 1)
 */
inline bool isNormalized(const Quaternion &q) noexcept { 
    float n = norm(q);
    return (std::abs(n - 1.0f) < epsilon_<>);
}


/**
 *  @brief Check if a quaternion is the identity quaternion (1 + 0i + 0j + 0k)
 */
inline bool isIdentity(const Quaternion &q) noexcept { 
    if(std::abs(q.w() - 1.0f) > epsilon_<>) { return false; }
    if(std::abs(q.x()) > epsilon_<>) { return false; }
    if(std::abs(q.y()) > epsilon_<>) { return false; }
    if(std::abs(q.z()) > epsilon_<>) { return false; }
    return true;
}

/**
 *  @brief Check if a quaternion is purely real (w + 0i + 0j + 0k)
 */
inline bool isReal(const Quaternion &q) noexcept { 
    if(std::abs(q.x()) > epsilon_<>) { return false; }
    if(std::abs(q.y()) > epsilon_<>) { return false; }
    if(std::abs(q.z()) > epsilon_<>) { return false; }
    return true;
}

/**
 *  @brief Check if a quaternion is purely imaginary (0 + xi + yj + zk)
 */
inline bool isPure(const Quaternion &q) noexcept { 
    if(std::abs(q.w()) > epsilon_<>) { return false; }
    return true;
}

/**
 *  @brief Check if two quaternions represent the same rotation
 *  @param q First quaternion
 *  @param p Second quaternion
 *  @return `true` if they represent the same rotation, `false` otherwise
 */
inline bool isSameRotation(const Quaternion &q, const Quaternion &p) noexcept { 
    Quaternion qn = normalize(q);
    Quaternion pn = normalize(p);

    float d = dot(qn, pn);
    return (std::abs(std::abs(d) - 1.0f) < epsilon_<>);
}

} // cobalt::math::geometry 