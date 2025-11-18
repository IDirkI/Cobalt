#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "cobalt/math/linear_algebra/vector/vector.hpp"

using namespace cobalt::math::linear_algebra;

// ============================================================================
// Vector Core Tests (vector.hpp)
// ============================================================================

TEST_CASE("Vector - Default Construction", "[vector][core]") {
    Vector<3> v;
    
    REQUIRE_THAT(v[0], Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(v[1], Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(v[2], Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Vector - Initializer List Construction", "[vector][core]") {
    Vector<3> v = {1.0f, 2.0f, 3.0f};
    
    REQUIRE_THAT(v[0], Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(v[1], Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(v[2], Catch::Matchers::WithinAbs(3.0, 1e-6));
}

TEST_CASE("Vector - Initializer List Partial", "[vector][core]") {
    Vector<4> v = {1.0f, 2.0f};
    
    REQUIRE_THAT(v[0], Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(v[1], Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(v[2], Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(v[3], Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Vector - Variadic Construction", "[vector][core]") {
    Vector<3> v(1.0f, 2.0f, 3.0f);
    
    REQUIRE_THAT(v[0], Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(v[1], Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(v[2], Catch::Matchers::WithinAbs(3.0, 1e-6));
}

TEST_CASE("Vector - Zero Factory", "[vector][core]") {
    Vector<3> v = Vector<3>::zero();
    
    REQUIRE_THAT(v[0], Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(v[1], Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(v[2], Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Vector - Unit X Factory 2D", "[vector][core]") {
    Vector<2> v = Vector<2>::unitX();
    
    REQUIRE_THAT(v[0], Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(v[1], Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Vector - Unit X Factory 3D", "[vector][core]") {
    Vector<3> v = Vector<3>::unitX();
    
    REQUIRE_THAT(v[0], Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(v[1], Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(v[2], Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Vector - Unit Y Factory 2D", "[vector][core]") {
    Vector<2> v = Vector<2>::unitY();
    
    REQUIRE_THAT(v[0], Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(v[1], Catch::Matchers::WithinAbs(1.0, 1e-6));
}

TEST_CASE("Vector - Unit Y Factory 3D", "[vector][core]") {
    Vector<3> v = Vector<3>::unitY();
    
    REQUIRE_THAT(v[0], Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(v[1], Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(v[2], Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Vector - Unit Z Factory", "[vector][core]") {
    Vector<3> v = Vector<3>::unitZ();
    
    REQUIRE_THAT(v[0], Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(v[1], Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(v[2], Catch::Matchers::WithinAbs(1.0, 1e-6));
}

TEST_CASE("Vector - From Array Factory", "[vector][core]") {
    std::array<float, 3> arr = {4.0f, 5.0f, 6.0f};
    Vector<3> v = Vector<3>::fromArray(arr);
    
    REQUIRE_THAT(v[0], Catch::Matchers::WithinAbs(4.0, 1e-6));
    REQUIRE_THAT(v[1], Catch::Matchers::WithinAbs(5.0, 1e-6));
    REQUIRE_THAT(v[2], Catch::Matchers::WithinAbs(6.0, 1e-6));
}

TEST_CASE("Vector - Size Getter", "[vector][core]") {
    Vector<5> v;
    REQUIRE(v.size() == 5);
}

TEST_CASE("Vector - X/Y Accessors 2D", "[vector][core]") {
    Vector<2> v = {3.0f, 4.0f};
    
    REQUIRE_THAT(v.x(), Catch::Matchers::WithinAbs(3.0, 1e-6));
    REQUIRE_THAT(v.y(), Catch::Matchers::WithinAbs(4.0, 1e-6));
    
    v.x() = 10.0f;
    v.y() = 20.0f;
    
    REQUIRE_THAT(v.x(), Catch::Matchers::WithinAbs(10.0, 1e-6));
    REQUIRE_THAT(v.y(), Catch::Matchers::WithinAbs(20.0, 1e-6));
}

TEST_CASE("Vector - X/Y/Z Accessors 3D", "[vector][core]") {
    Vector<3> v = {1.0f, 2.0f, 3.0f};
    
    REQUIRE_THAT(v.x(), Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(v.y(), Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(v.z(), Catch::Matchers::WithinAbs(3.0, 1e-6));
    
    v.x() = 7.0f;
    v.y() = 8.0f;
    v.z() = 9.0f;
    
    REQUIRE_THAT(v.x(), Catch::Matchers::WithinAbs(7.0, 1e-6));
    REQUIRE_THAT(v.y(), Catch::Matchers::WithinAbs(8.0, 1e-6));
    REQUIRE_THAT(v.z(), Catch::Matchers::WithinAbs(9.0, 1e-6));
}

TEST_CASE("Vector - Element Access Valid Index", "[vector][core]") {
    Vector<3> v = {1.0f, 2.0f, 3.0f};
    
    REQUIRE_THAT(v[0], Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(v[1], Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(v[2], Catch::Matchers::WithinAbs(3.0, 1e-6));
}

TEST_CASE("Vector - Safe Element Access Out of Bounds", "[vector][core]") {
    Vector<3> v = {1.0f, 2.0f, 3.0f};
    
    // at() should clamp to last element
    REQUIRE_THAT(v.at(10), Catch::Matchers::WithinAbs(3.0, 1e-6));
}

TEST_CASE("Vector - Element Modification", "[vector][core]") {
    Vector<3> v;
    
    v[0] = 5.0f;
    v[1] = 6.0f;
    v[2] = 7.0f;
    
    REQUIRE_THAT(v[0], Catch::Matchers::WithinAbs(5.0, 1e-6));
    REQUIRE_THAT(v[1], Catch::Matchers::WithinAbs(6.0, 1e-6));
    REQUIRE_THAT(v[2], Catch::Matchers::WithinAbs(7.0, 1e-6));
}

TEST_CASE("Vector - Addition Assignment", "[vector][core]") {
    Vector<3> v1 = {1.0f, 2.0f, 3.0f};
    Vector<3> v2 = {4.0f, 5.0f, 6.0f};
    
    v1 += v2;
    
    REQUIRE_THAT(v1[0], Catch::Matchers::WithinAbs(5.0, 1e-6));
    REQUIRE_THAT(v1[1], Catch::Matchers::WithinAbs(7.0, 1e-6));
    REQUIRE_THAT(v1[2], Catch::Matchers::WithinAbs(9.0, 1e-6));
}

TEST_CASE("Vector - Subtraction Assignment", "[vector][core]") {
    Vector<3> v1 = {5.0f, 7.0f, 9.0f};
    Vector<3> v2 = {1.0f, 2.0f, 3.0f};
    
    v1 -= v2;
    
    REQUIRE_THAT(v1[0], Catch::Matchers::WithinAbs(4.0, 1e-6));
    REQUIRE_THAT(v1[1], Catch::Matchers::WithinAbs(5.0, 1e-6));
    REQUIRE_THAT(v1[2], Catch::Matchers::WithinAbs(6.0, 1e-6));
}

TEST_CASE("Vector - Scalar Multiplication Assignment", "[vector][core]") {
    Vector<3> v = {1.0f, 2.0f, 3.0f};
    
    v *= 2.0f;
    
    REQUIRE_THAT(v[0], Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(v[1], Catch::Matchers::WithinAbs(4.0, 1e-6));
    REQUIRE_THAT(v[2], Catch::Matchers::WithinAbs(6.0, 1e-6));
}

TEST_CASE("Vector - Scalar Division Assignment", "[vector][core]") {
    Vector<3> v = {2.0f, 4.0f, 6.0f};
    
    v /= 2.0f;
    
    REQUIRE_THAT(v[0], Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(v[1], Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(v[2], Catch::Matchers::WithinAbs(3.0, 1e-6));
}

TEST_CASE("Vector - Different Types", "[vector][core]") {
    Vector<3, double> vd = {1.0, 2.0, 3.0};
    Vector<3, int> vi = {1, 2, 3};
    
    REQUIRE(vd[0] == 1.0);
    REQUIRE(vi[0] == 1);
}