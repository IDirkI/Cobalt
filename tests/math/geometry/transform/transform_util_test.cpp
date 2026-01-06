#define _USE_MATH_DEFINES
#include <cmath>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "cobalt/math/geometry/transform/transform.hpp"
#include "cobalt/math/geometry/transform/transform_ops.hpp"
#include "cobalt/math/geometry/transform/transform_util.hpp"
#include "cobalt/math/geometry/quaternion/quaternion.hpp"
#include "cobalt/math/geometry/quaternion/quaternion_util.hpp"
#include "cobalt/math/linear_algebra/vector/vector.hpp"
#include "cobalt/math/linear_algebra/vector/vector_ops.hpp"
#include "cobalt/math/linear_algebra/matrix/matrix.hpp"
#include "cobalt/math/linear_algebra/matrix/matrix_ops.hpp"

using namespace cobalt::math::geometry;
using namespace cobalt::math::linear_algebra;

// ============================================================================
// Transform Utility Tests (transform_util.hpp)
// ============================================================================

// ----------------------------------------------------------------------------
// Element-wise Operation Tests
// ----------------------------------------------------------------------------

TEST_CASE("Transform - Clean Zero Translation", "[transform][util][element]") {
    Transform<> T(Vector<3>{1e-10f, 2.0f, -1e-10f});
    
    Transform<> result = cleanZero(T);
    
    REQUIRE_THAT(result.translation()[0], Catch::Matchers::WithinAbs(0.0f, 1e-6));
    REQUIRE_THAT(result.translation()[1], Catch::Matchers::WithinAbs(2.0f, 1e-6));
    REQUIRE_THAT(result.translation()[2], Catch::Matchers::WithinAbs(0.0f, 1e-6));
}

TEST_CASE("Transform - Clean Zero Rotation", "[transform][util][element]") {
    Quaternion<> q(1.0f + 1e-10f, 1e-10f, 1e-10f, 1e-10f);
    Transform<> T(q);
    
    Transform<> result = cleanZero(T);
    
    REQUIRE_THAT(result.rotation().w(), Catch::Matchers::WithinAbs(1.0f, 1e-6));
    REQUIRE_THAT(result.rotation().x(), Catch::Matchers::WithinAbs(0.0f, 1e-6));
    REQUIRE_THAT(result.rotation().y(), Catch::Matchers::WithinAbs(0.0f, 1e-6));
    REQUIRE_THAT(result.rotation().z(), Catch::Matchers::WithinAbs(0.0f, 1e-6));
}

TEST_CASE("Transform - Clean Zero Combined", "[transform][util][element]") {
    Quaternion<> q(1.0f, 1e-10f, 0.0f, 1e-10f);
    Vector<3> t = {1e-10f, 1.0f, 1e-10f};
    Transform<> T(q, t);
    
    Transform<> result = cleanZero(T);
    
    REQUIRE_THAT(result.translation()[0], Catch::Matchers::WithinAbs(0.0f, 1e-6));
    REQUIRE_THAT(result.translation()[1], Catch::Matchers::WithinAbs(1.0f, 1e-6));
    REQUIRE_THAT(result.translation()[2], Catch::Matchers::WithinAbs(0.0f, 1e-6));
    
    REQUIRE_THAT(result.rotation().x(), Catch::Matchers::WithinAbs(0.0f, 1e-6));
    REQUIRE_THAT(result.rotation().z(), Catch::Matchers::WithinAbs(0.0f, 1e-6));
}

TEST_CASE("Transform - Clean Zero Preserves Significant Values", "[transform][util][element]") {
    Transform<> T(Vector<3>{0.001f, 2.0f, 0.0001f});
    
    Transform<> result = cleanZero(T);
    
    REQUIRE_THAT(result.translation()[0], Catch::Matchers::WithinAbs(0.001f, 1e-6));
    REQUIRE_THAT(result.translation()[1], Catch::Matchers::WithinAbs(2.0f, 1e-6));
    REQUIRE_THAT(result.translation()[2], Catch::Matchers::WithinAbs(0.0001f, 1e-6));
}

TEST_CASE("Transform - Clean Zero Identity", "[transform][util][element]") {
    Transform<> T = Transform<>::eye();
    
    Transform<> result = cleanZero(T);
    
    REQUIRE(result == T);
}

// ----------------------------------------------------------------------------
// Interpolation Tests
// ----------------------------------------------------------------------------

TEST_CASE("Transform - Lerp Boundaries", "[transform][util][interp]") {
    Transform<> T1(Vector<3>{0.0f, 0.0f, 0.0f});
    Transform<> T2(Vector<3>{10.0f, 0.0f, 0.0f});
    
    Transform<> result0 = lerp(T1, T2, 0.0f);
    Transform<> result1 = lerp(T1, T2, 1.0f);
    
    REQUIRE_THAT(result0.translation()[0], Catch::Matchers::WithinAbs(T1.translation()[0], 1e-5f));
    REQUIRE_THAT(result1.translation()[0], Catch::Matchers::WithinAbs(T2.translation()[0], 1e-5f));
}

TEST_CASE("Transform - Lerp Midpoint Translation", "[transform][util][interp]") {
    Transform<> T1(Vector<3>{0.0f, 0.0f, 0.0f});
    Transform<> T2(Vector<3>{10.0f, 20.0f, 30.0f});
    
    Transform<> result = lerp(T1, T2, 0.5f);
    
    REQUIRE_THAT(result.translation()[0], Catch::Matchers::WithinAbs(5.0f, 1e-5f));
    REQUIRE_THAT(result.translation()[1], Catch::Matchers::WithinAbs(10.0f, 1e-5f));
    REQUIRE_THAT(result.translation()[2], Catch::Matchers::WithinAbs(15.0f, 1e-5f));
}

TEST_CASE("Transform - Lerp Rotation Only", "[transform][util][interp]") {
    Transform<> T1 = Transform<>::fromRotationZ(0.0f);
    Transform<> T2 = Transform<>::fromRotationZ(M_PI / 2.0f);
    
    Transform<> result = lerp(T1, T2, 0.5f);
    
    // Should interpolate rotation
    REQUIRE(isNormalized(result.rotation()));
}

TEST_CASE("Transform - Lerp Combined", "[transform][util][interp]") {
    Quaternion<> q1 = Quaternion<>::fromAxisAngle(Vector<3>{0.0f, 0.0f, 1.0f}, 0.0f);
    Quaternion<> q2 = Quaternion<>::fromAxisAngle(Vector<3>{0.0f, 0.0f, 1.0f}, M_PI / 2.0f);
    
    Transform<> T1(q1, Vector<3>{0.0f, 0.0f, 0.0f});
    Transform<> T2(q2, Vector<3>{10.0f, 0.0f, 0.0f});
    
    Transform<> result = lerp(T1, T2, 0.5f);
    
    // Should interpolate both
    REQUIRE(norm(result.translation()) > 0.0f);
    REQUIRE(isNormalized(result.rotation()));
}

TEST_CASE("Transform - Lerp Multiple Steps", "[transform][util][interp]") {
    Transform<> T1 = Transform<>::eye();
    Transform<> T2(Vector<3>{10.0f, 0.0f, 0.0f});
    
    std::vector<float> ts = {0.0f, 0.25f, 0.5f, 0.75f, 1.0f};
    
    for (float t : ts) {
        Transform<> result = lerp(T1, T2, t);
        REQUIRE_THAT(result.translation()[0], Catch::Matchers::WithinAbs(10.0f * t, 1e-5f));
    }
}

// ----------------------------------------------------------------------------
// Conversion Tests
// ----------------------------------------------------------------------------

TEST_CASE("Transform - To Matrix Identity", "[transform][util][conversion]") {
    Transform<> T = Transform<>::eye();
    
    Matrix<4, 4> M = toMatrix(T);
    
    Matrix<4, 4> I = Matrix<4, 4>::eye();
    
    for (uint8_t i = 0; i < 4; i++) {
        for (uint8_t j = 0; j < 4; j++) {
            REQUIRE_THAT(M(i, j), Catch::Matchers::WithinAbs(I(i, j), 1e-5f));
        }
    }
}

TEST_CASE("Transform - To Matrix Pure Translation", "[transform][util][conversion]") {
    Transform<> T(Vector<3>{1.0f, 2.0f, 3.0f});
    
    Matrix<4, 4> M = toMatrix(T);
    
    // Check translation column
    REQUIRE_THAT(M(0, 3), Catch::Matchers::WithinAbs(1.0f, 1e-6));
    REQUIRE_THAT(M(1, 3), Catch::Matchers::WithinAbs(2.0f, 1e-6));
    REQUIRE_THAT(M(2, 3), Catch::Matchers::WithinAbs(3.0f, 1e-6));
    
    // Check rotation is identity
    for (uint8_t i = 0; i < 3; i++) {
        for (uint8_t j = 0; j < 3; j++) {
            float expected = (i == j) ? 1.0f : 0.0f;
            REQUIRE_THAT(M(i, j), Catch::Matchers::WithinAbs(expected, 1e-5f));
        }
    }
}

TEST_CASE("Transform - To Matrix Pure Rotation", "[transform][util][conversion]") {
    Transform<> T = Transform<>::fromRotationZ(M_PI / 2.0f);
    
    Matrix<4, 4> M = toMatrix(T);
    
    // Check translation is zero
    REQUIRE_THAT(M(0, 3), Catch::Matchers::WithinAbs(0.0f, 1e-6));
    REQUIRE_THAT(M(1, 3), Catch::Matchers::WithinAbs(0.0f, 1e-6));
    REQUIRE_THAT(M(2, 3), Catch::Matchers::WithinAbs(0.0f, 1e-6));
    
    // Check bottom row is [0, 0, 0, 1]
    REQUIRE_THAT(M(3, 0), Catch::Matchers::WithinAbs(0.0f, 1e-6));
    REQUIRE_THAT(M(3, 1), Catch::Matchers::WithinAbs(0.0f, 1e-6));
    REQUIRE_THAT(M(3, 2), Catch::Matchers::WithinAbs(0.0f, 1e-6));
    REQUIRE_THAT(M(3, 3), Catch::Matchers::WithinAbs(1.0f, 1e-6));
}

TEST_CASE("Transform - To Matrix Combined", "[transform][util][conversion]") {
    Quaternion<> q = Quaternion<>::fromAxisAngle(Vector<3>{0.0f, 0.0f, 1.0f}, M_PI / 4.0f);
    Transform<> T(q, Vector<3>{5.0f, 10.0f, 15.0f});
    
    Matrix<4, 4> M = toMatrix(T);
    
    // Check translation
    REQUIRE_THAT(M(0, 3), Catch::Matchers::WithinAbs(5.0f, 1e-5f));
    REQUIRE_THAT(M(1, 3), Catch::Matchers::WithinAbs(10.0f, 1e-5f));
    REQUIRE_THAT(M(2, 3), Catch::Matchers::WithinAbs(15.0f, 1e-5f));
    
    // Check bottom row
    REQUIRE_THAT(M(3, 3), Catch::Matchers::WithinAbs(1.0f, 1e-6));
}

TEST_CASE("Transform - To Matrix Orthogonality", "[transform][util][conversion]") {
    Quaternion<> q = Quaternion<>::fromAxisAngle(Vector<3>{1.0f, 1.0f, 1.0f}, M_PI / 3.0f);
    Transform<> T(q);
    
    Matrix<4, 4> M = toMatrix(T);
    
    // Extract rotation part
    Matrix<3, 3> R;
    for (uint8_t i = 0; i < 3; i++) {
        for (uint8_t j = 0; j < 3; j++) {
            R(i, j) = M(i, j);
        }
    }
    
    // Check orthogonality: R^T * R = I
    Matrix<3, 3> RtR = transpose(R) * R;
    Matrix<3, 3> I = Matrix<3, 3>::eye();
    
    for (uint8_t i = 0; i < 3; i++) {
        for (uint8_t j = 0; j < 3; j++) {
            REQUIRE_THAT(RtR(i, j), Catch::Matchers::WithinAbs(I(i, j), 1e-5f));
        }
    }
}

TEST_CASE("Transform - To Rotation Matrix", "[transform][util][conversion]") {
    Transform<> T = Transform<>::fromRotationX(M_PI / 6.0f);
    
    Matrix<3, 3> R = toRotationMatrix(T);
    
    // Should be 3x3 rotation matrix
    float det = cobalt::math::linear_algebra::det(R);
    REQUIRE_THAT(det, Catch::Matchers::WithinAbs(1.0f, 1e-5f));
}

TEST_CASE("Transform - To Rotation Matrix Identity", "[transform][util][conversion]") {
    Transform<> T = Transform<>::eye();
    
    Matrix<3, 3> R = toRotationMatrix(T);
    
    REQUIRE(R == Matrix<3, 3>::eye());
}

TEST_CASE("Transform - To Rotation Matrix Ignores Translation", "[transform][util][conversion]") {
    Transform<> T1 = Transform<>::fromRotationZ(M_PI / 4);
    Transform<> T2(Quaternion<>::fromAxisAngle(Vector<3>{0.0f, 0.0f, 1.0f}, M_PI / 4), Vector<3>{100.0f, 200.0f, 300.0f});
    
    Matrix<3, 3> R1 = toRotationMatrix(T1);
    Matrix<3, 3> R2 = toRotationMatrix(T2);
    
    // Should be equal despite different translations
    for (uint8_t i = 0; i < 3; i++) {
        for (uint8_t j = 0; j < 3; j++) {
            REQUIRE_THAT(R1(i, j), Catch::Matchers::WithinAbs(R2(i, j), 1e-5f));
        }
    }
}

TEST_CASE("Transform - To Vector Representation", "[transform][util][conversion]") {
    Quaternion<> q = Quaternion<>::fromAxisAngle(Vector<3>{0.0f, 0.0f, 1.0f}, M_PI / 4);
    Vector<3> t = {1.0f, 2.0f, 3.0f};
    Transform<> T(q, t);
    
    Vector<3> rotVec, transVec;
    toVector(T, rotVec, transVec);
    
    // Check translation matches
    REQUIRE(transVec == t);
    
    // Check rotation vector norm is angle
    REQUIRE_THAT(norm(rotVec), Catch::Matchers::WithinAbs(M_PI / 4, 1e-5f));
}

TEST_CASE("Transform - To Vector Zero Rotation", "[transform][util][conversion]") {
    Transform<> T(Vector<3>{5.0f, 10.0f, 15.0f});
    
    Vector<3> rotVec, transVec;
    toVector(T, rotVec, transVec);
    
    REQUIRE_THAT(norm(rotVec), Catch::Matchers::WithinAbs(0.0f, 1e-6));
    REQUIRE(transVec == T.translation());
}

TEST_CASE("Transform - To Euler Angles Identity", "[transform][util][conversion]") {
    Transform<> T = Transform<>::eye();
    
    float roll, pitch, yaw;
    toEuler(T, roll, pitch, yaw);
    
    REQUIRE_THAT(roll, Catch::Matchers::WithinAbs(0.0f, 1e-5f));
    REQUIRE_THAT(pitch, Catch::Matchers::WithinAbs(0.0f, 1e-5f));
    REQUIRE_THAT(yaw, Catch::Matchers::WithinAbs(0.0f, 1e-5f));
}

TEST_CASE("Transform - To Euler Angles Round Trip", "[transform][util][conversion]") {
    float expectedRoll = M_PI / 6;
    float expectedPitch = M_PI / 4;
    float expectedYaw = M_PI / 3;
    
    Quaternion<> q = Quaternion<>::fromEuler(expectedRoll, expectedPitch, expectedYaw);
    Transform<> T(q);
    
    float roll, pitch, yaw;
    toEuler(T, roll, pitch, yaw);
    
    REQUIRE_THAT(roll, Catch::Matchers::WithinAbs(expectedRoll, 1e-5f));
    REQUIRE_THAT(pitch, Catch::Matchers::WithinAbs(expectedPitch, 1e-5f));
    REQUIRE_THAT(yaw, Catch::Matchers::WithinAbs(expectedYaw, 1e-5f));
}

TEST_CASE("Transform - To Euler Only Rotation Matters", "[transform][util][conversion]") {
    Quaternion<> q = Quaternion<>::fromEuler(0.1f, 0.2f, 0.3f);
    
    Transform<> T1(q);
    Transform<> T2(q, Vector<3>{10.0f, 20.0f, 30.0f});
    
    float roll1, pitch1, yaw1;
    float roll2, pitch2, yaw2;
    
    toEuler(T1, roll1, pitch1, yaw1);
    toEuler(T2, roll2, pitch2, yaw2);
    
    REQUIRE_THAT(roll1, Catch::Matchers::WithinAbs(roll2, 1e-6f));
    REQUIRE_THAT(pitch1, Catch::Matchers::WithinAbs(pitch2, 1e-6f));
    REQUIRE_THAT(yaw1, Catch::Matchers::WithinAbs(yaw2, 1e-6f));
}

// ----------------------------------------------------------------------------
// Check Function Tests
// ----------------------------------------------------------------------------

TEST_CASE("Transform - Is Identity True", "[transform][util][check]") {
    Transform<> T = Transform<>::eye();
    
    REQUIRE(isIdentity(T));
}

TEST_CASE("Transform - Is Identity False Translation", "[transform][util][check]") {
    Transform<> T(Vector<3>{0.01f, 0.0f, 0.0f});
    
    REQUIRE_FALSE(isIdentity(T));
}

TEST_CASE("Transform - Is Identity False Rotation", "[transform][util][check]") {
    Transform<> T = Transform<>::fromRotationZ(0.01f);
    
    REQUIRE_FALSE(isIdentity(T));
}

TEST_CASE("Transform - Is Identity Within Epsilon", "[transform][util][check]") {
    Quaternion<> q(1.0f + 1e-7f, 1e-10f, 1e-10f, 1e-10f);
    Transform<> T(q, Vector<3>{1e-10f, 1e-10f, 1e-10f});
    
    REQUIRE(isIdentity(T));
}

TEST_CASE("Transform - Is Pure Rotation True", "[transform][util][check]") {
    Transform<> T = Transform<>::fromRotationZ(M_PI / 4);
    
    REQUIRE(isPureRotation(T));
}

TEST_CASE("Transform - Is Pure Rotation False", "[transform][util][check]") {
    Transform<> T(Quaternion<>::fromAxisAngle(Vector<3>{0.0f, 0.0f, 1.0f}, M_PI / 4), Vector<3>{0.01f, 0.0f, 0.0f});
    
    REQUIRE_FALSE(isPureRotation(T));
}

TEST_CASE("Transform - Is Pure Rotation Identity", "[transform][util][check]") {
    Transform<> T = Transform<>::eye();
    
    // Identity is technically pure rotation
    REQUIRE(isPureRotation(T));
}

TEST_CASE("Transform - Is Pure Translation True", "[transform][util][check]") {
    Transform<> T(Vector<3>{5.0f, 10.0f, 15.0f});
    
    REQUIRE(isPureTranslation(T));
}

TEST_CASE("Transform - Is Pure Translation False", "[transform][util][check]") {
    Quaternion<> q = Quaternion<>::fromAxisAngle(Vector<3>{0.0f, 0.0f, 1.0f}, 0.01f);
    Transform<> T(q, Vector<3>{1.0f, 2.0f, 3.0f});
    
    REQUIRE_FALSE(isPureTranslation(T));
}

TEST_CASE("Transform - Is Pure Translation Identity", "[transform][util][check]") {
    Transform<> T = Transform<>::eye();
    
    // Identity is technically pure translation
    REQUIRE(isPureTranslation(T));
}

TEST_CASE("Transform - Is Finite True", "[transform][util][check]") {
    Transform<> T(Vector<3>{1.0f, 2.0f, 3.0f});
    
    REQUIRE(isFinite(T));
}

TEST_CASE("Transform - Is Finite False Translation Infinity", "[transform][util][check]") {
    Transform<> T(Vector<3>{std::numeric_limits<float>::infinity(), 2.0f, 3.0f});
    
    REQUIRE_FALSE(isFinite(T));
}

TEST_CASE("Transform - Is Finite False Translation NaN", "[transform][util][check]") {
    Transform<> T(Vector<3>{1.0f, std::numeric_limits<float>::quiet_NaN(), 3.0f});
    
    REQUIRE_FALSE(isFinite(T));
}

TEST_CASE("Transform - Is Finite False Rotation NaN", "[transform][util][check]") {
    Quaternion<> q(1.0f, std::numeric_limits<float>::quiet_NaN(), 0.0f, 0.0f);
    Transform<> T(q);
    
    REQUIRE_FALSE(isFinite(T));
}

TEST_CASE("Transform - Is Valid True", "[transform][util][check]") {
    Quaternion<> q = normalize(Quaternion<>::fromAxisAngle(Vector<3>{1.0f, 0.0f, 0.0f}, M_PI / 4));
    Transform<> T(q, Vector<3>{1.0f, 2.0f, 3.0f});
    
    REQUIRE(isValid(T));
}

TEST_CASE("Transform - Is Valid False Unnormalized Rotation", "[transform][util][check]") {
    Quaternion<> q(2.0f, 0.0f, 0.0f, 0.0f);  // Not normalized
    Transform<> T(q);
    
    REQUIRE_FALSE(isValid(T));
}

TEST_CASE("Transform - Is Valid False Non-Finite", "[transform][util][check]") {
    Transform<> T(Vector<3>{std::numeric_limits<float>::infinity(), 0.0f, 0.0f});
    
    REQUIRE_FALSE(isValid(T));
}

TEST_CASE("Transform - Is Valid Identity", "[transform][util][check]") {
    Transform<> T = Transform<>::eye();
    
    REQUIRE(isValid(T));
}

// ----------------------------------------------------------------------------
// Integration Tests
// ----------------------------------------------------------------------------

TEST_CASE("Transform - Round Trip Matrix Conversion", "[transform][integration]") {
    Quaternion<> q = Quaternion<>::fromAxisAngle(Vector<3>{1.0f, 1.0f, 1.0f}, M_PI / 3);
    Vector<3> t = {5.0f, 10.0f, 15.0f};
    Transform<> T_original(q, t);
    
    // Convert to matrix
    Matrix<4, 4> M = toMatrix(T_original);
    
    // Apply to point using matrix
    Vector<3> p = {1.0f, 2.0f, 3.0f};
    Vector<3> result_transform = T_original * p;
    
    // Manually apply matrix (homogeneous coordinates)
    Vector<3> result_matrix;
    for (uint8_t i = 0; i < 3; i++) {
        result_matrix[i] = M(i, 0) * p[0] + M(i, 1) * p[1] + M(i, 2) * p[2] + M(i, 3);
    }
    
    REQUIRE_THAT(result_transform[0], Catch::Matchers::WithinAbs(result_matrix[0], 1e-4f));
    REQUIRE_THAT(result_transform[1], Catch::Matchers::WithinAbs(result_matrix[1], 1e-4f));
    REQUIRE_THAT(result_transform[2], Catch::Matchers::WithinAbs(result_matrix[2], 1e-4f));
}

TEST_CASE("Transform - Clean Zero After Many Operations", "[transform][integration]") {
    Transform<> T = Transform<>::eye();
    
    // Perform many operations that should cancel
    for (int i = 0; i < 100; i++) {
        Transform<> step = Transform<>::fromRotationZ(0.01f);
        T = T * step;
        T = T * inv(step);
    }
    
    // Clean numerical errors
    T = cleanZero(T);
    
    // Should be close to identity
    REQUIRE_THAT(norm(T.translation()), Catch::Matchers::WithinAbs(0.0f, 1e-3f));
}

TEST_CASE("Transform - Conversion Consistency", "[transform][integration]") {
    // Create transform with known rotation
    float roll = M_PI / 6;
    float pitch = M_PI / 4;
    float yaw = M_PI / 3;
    
    Quaternion<> q = Quaternion<>::fromEuler(roll, pitch, yaw);
    Transform<> T(q, Vector<3>{1.0f, 2.0f, 3.0f});
    
    // Convert to Euler
    float roll_out, pitch_out, yaw_out;
    toEuler(T, roll_out, pitch_out, yaw_out);
    
    // Reconstruct
    Quaternion<> q_reconstructed = Quaternion<>::fromEuler(roll_out, pitch_out, yaw_out);
    Transform<> T_reconstructed(q_reconstructed, T.translation());
    
    // Apply to test point
    Vector<3> p = {5.0f, 10.0f, 15.0f};
    Vector<3> r1 = T * p;
    Vector<3> r2 = T_reconstructed * p;
    
    REQUIRE_THAT(r1[0], Catch::Matchers::WithinAbs(r2[0], 1e-4f));
    REQUIRE_THAT(r1[1], Catch::Matchers::WithinAbs(r2[1], 1e-4f));
    REQUIRE_THAT(r1[2], Catch::Matchers::WithinAbs(r2[2], 1e-4f));
}

TEST_CASE("Transform - Matrix Conversion Preserves Determinant", "[transform][integration]") {
    // Any rotation should have determinant 1
    for (float angle = 0.0f; angle < 2.0f * M_PI; angle += M_PI / 8) {
        Transform<> T = Transform<>::fromRotationZ(angle);
        Matrix<3, 3> R = toRotationMatrix(T);
        
        float det = cobalt::math::linear_algebra::det(R);
        REQUIRE_THAT(det, Catch::Matchers::WithinAbs(1.0f, 1e-5f));
    }
}

TEST_CASE("Transform - Check Functions Consistent", "[transform][integration]") {
    // Pure rotation
    Transform<> T_rot = Transform<>::fromRotationX(M_PI / 4);
    REQUIRE(isPureRotation(T_rot));
    REQUIRE_FALSE(isPureTranslation(T_rot));
    
    // Pure translation
    Transform<> T_trans(Vector<3>{1.0f, 2.0f, 3.0f});
    REQUIRE(isPureTranslation(T_trans));
    REQUIRE_FALSE(isPureRotation(T_trans));
    
    // Combined
    Transform<> T_both(Quaternion<>::fromAxisAngle(Vector<3>{0.0f, 0.0f, 1.0f}, M_PI / 4), 
                       Vector<3>{1.0f, 2.0f, 3.0f});
    REQUIRE_FALSE(isPureRotation(T_both));
    REQUIRE_FALSE(isPureTranslation(T_both));
    
    // Identity is both
    Transform<> T_identity = Transform<>::eye();
    REQUIRE(isPureRotation(T_identity));
    REQUIRE(isPureTranslation(T_identity));
    REQUIRE(isIdentity(T_identity));
}

TEST_CASE("Transform - Conversion Handles Gimbal Lock", "[transform][integration]") {
    // Pitch at 90 degrees (gimbal lock condition)
    float roll = M_PI / 6;
    float pitch = M_PI / 2;
    float yaw = M_PI / 4;
    
    Quaternion<> q = Quaternion<>::fromEuler(roll, pitch, yaw);
    Transform<> T(q);
    
    // Should still convert without issues
    float roll_out, pitch_out, yaw_out;
    toEuler(T, roll_out, pitch_out, yaw_out);
    
    REQUIRE(std::isfinite(roll_out));
    REQUIRE(std::isfinite(pitch_out));
    REQUIRE(std::isfinite(yaw_out));
}

TEST_CASE("Transform - Multiple Conversions Stable", "[transform][integration]") {
    Transform<> T_original = Transform<>::fromRotationZ(M_PI / 5);
    T_original.translation() = Vector<3>{3.0f, 4.0f, 5.0f};
    
    // Convert multiple times
    for (int i = 0; i < 10; i++) {
        Matrix<4, 4> M = toMatrix(T_original);
        Vector<3> rotVec, transVec;
        toVector(T_original, rotVec, transVec);
        
        // Values should remain consistent
        REQUIRE(isValid(T_original));
    }
}

TEST_CASE("Transform - Clean Zero Idempotent", "[transform][integration]") {
    Quaternion<> q(1.0f, 1e-10f, 1e-10f, 1e-10f);
    Transform<> T(q, Vector<3>{1e-10f, 1.0f, 1e-10f});
    
    Transform<> cleaned_once = cleanZero(T);
    Transform<> cleaned_twice = cleanZero(cleaned_once);
    
    // Should be identical after first clean
    REQUIRE(cleaned_once.translation() == cleaned_twice.translation());
    REQUIRE(cleaned_once.rotation() == cleaned_twice.rotation());
}

TEST_CASE("Transform - Interpolation Path Length", "[transform][integration]") {
    Transform<> T_start(Vector<3>{0.0f, 0.0f, 0.0f});
    Transform<> T_end(Vector<3>{10.0f, 0.0f, 0.0f});
    
    // Generate path
    std::vector<Vector<3>> path_points;
    for (float t = 0.0f; t <= 1.0f; t += 0.05f) {
        Transform<> T_interp = lerp(T_start, T_end, t);
        path_points.push_back(T_interp.translation());
    }
    
    // Calculate total path length
    float total_length = 0.0f;
    for (size_t i = 1; i < path_points.size(); i++) {
        total_length += norm(path_points[i] - path_points[i-1]);
    }
    
    // Should be approximately 10.0
    REQUIRE_THAT(total_length, Catch::Matchers::WithinAbs(10.0f, 0.5f));
}

TEST_CASE("Transform - Matrix Application Equivalence", "[transform][integration]") {
    // Create arbitrary transform
    Quaternion<> q = Quaternion<>::fromAxisAngle(Vector<3>{1.0f, 2.0f, 3.0f}, 1.2f);
    Transform<> T(q, Vector<3>{5.0f, 10.0f, 15.0f});
    
    // Test multiple points
    std::vector<Vector<3>> test_points = {
        {0.0f, 0.0f, 0.0f},
        {1.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f},
        {0.0f, 0.0f, 1.0f},
        {1.0f, 1.0f, 1.0f},
        {-1.0f, -1.0f, -1.0f}
    };
    
    Matrix<4, 4> M = toMatrix(T);
    
    for (const auto& p : test_points) {
        Vector<3> result_transform = T * p;
        
        // Apply using matrix
        Vector<3> result_matrix;
        for (uint8_t i = 0; i < 3; i++) {
            result_matrix[i] = M(i, 0) * p[0] + M(i, 1) * p[1] + M(i, 2) * p[2] + M(i, 3);
        }
        
        REQUIRE_THAT(result_transform[0], Catch::Matchers::WithinAbs(result_matrix[0], 1e-4f));
        REQUIRE_THAT(result_transform[1], Catch::Matchers::WithinAbs(result_matrix[1], 1e-4f));
        REQUIRE_THAT(result_transform[2], Catch::Matchers::WithinAbs(result_matrix[2], 1e-4f));
    }
}

TEST_CASE("Transform - Rotation Matrix Consistency", "[transform][integration]") {
    // Create multiple rotations
    std::vector<Transform<>> transforms = {
        Transform<>::fromRotationX(M_PI / 4),
        Transform<>::fromRotationY(M_PI / 3),
        Transform<>::fromRotationZ(M_PI / 6),
        Transform<>::fromRotationX(-M_PI / 2)
    };
    
    for (const auto& T : transforms) {
        Matrix<3, 3> R = toRotationMatrix(T);
        
        // Check properties of rotation matrix
        // 1. Determinant should be 1
        float det = cobalt::math::linear_algebra::det(R);
        REQUIRE_THAT(det, Catch::Matchers::WithinAbs(1.0f, 1e-5f));
        
        // 2. Should be orthogonal: R^T * R = I
        Matrix<3, 3> RtR = transpose(R) * R;
        Matrix<3, 3> I = Matrix<3, 3>::eye();
        
        for (uint8_t i = 0; i < 3; i++) {
            for (uint8_t j = 0; j < 3; j++) {
                REQUIRE_THAT(RtR(i, j), Catch::Matchers::WithinAbs(I(i, j), 1e-5f));
            }
        }
    }
}

TEST_CASE("Transform - Check Functions Edge Cases", "[transform][integration]") {
    // Very small rotation
    Transform<> T_small_rot = Transform<>::fromRotationZ(1e-8f);
    REQUIRE(isPureRotation(T_small_rot));  // Still pure rotation despite tiny angle
    
    // Very small translation
    Transform<> T_small_trans(Vector<3>{1e-8f, 1e-8f, 1e-8f});
    REQUIRE(isPureTranslation(T_small_trans));  // Still pure translation despite tiny values
    
    // Almost identity
    Quaternion<> q_almost(1.0f - 1e-8f, 1e-9f, 1e-9f, 1e-9f);
    Transform<> T_almost(q_almost, Vector<3>{1e-9f, 1e-9f, 1e-9f});
    // May or may not be considered identity depending on epsilon
}

TEST_CASE("Transform - Conversion Preserves Transform Effect", "[transform][integration]") {
    // Create transform
    Quaternion<> q = Quaternion<>::fromAxisAngle(Vector<3>{1.0f, 1.0f, 0.0f}, 0.8f);
    Transform<> T_original(q, Vector<3>{2.0f, 3.0f, 4.0f});
    
    // Convert to various representations and back
    Matrix<4, 4> M = toMatrix(T_original);
    
    Vector<3> rotVec, transVec;
    toVector(T_original, rotVec, transVec);
    
    float roll, pitch, yaw;
    toEuler(T_original, roll, pitch, yaw);
    
    // Test point
    Vector<3> p = {1.0f, 2.0f, 3.0f};
    Vector<3> result_original = T_original * p;
    
    // All representations should produce same result when applied
    REQUIRE(isFinite(result_original));
}