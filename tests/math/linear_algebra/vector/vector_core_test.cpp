#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "cobalt/math/linear_algebra/vector/vector.hpp"
#include "cobalt/math/linear_algebra/vector/vector_ops.hpp"

using namespace cobalt::math::linear_algebra;

// ============================================================================
// Vector Core Tests (vector.hpp)
// ============================================================================

// ----------------------------------------------------------------------------
// Construction Tests
// ----------------------------------------------------------------------------

TEST_CASE("Vector - Default Construction Zeros All Elements", "[vector][core][construction]") {
    Vector<3> v;
    
    REQUIRE_THAT(v[0], Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(v[1], Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(v[2], Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Vector - Default Construction Various Sizes", "[vector][core][construction]") {
    Vector<1> v1;
    Vector<2> v2;
    Vector<5> v5;
    
    REQUIRE_THAT(v1[0], Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(v2[1], Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(v5[4], Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Vector - Initializer List Full Construction", "[vector][core][construction]") {
    Vector<3> v = {1.0f, 2.0f, 3.0f};
    
    REQUIRE_THAT(v[0], Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(v[1], Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(v[2], Catch::Matchers::WithinAbs(3.0, 1e-6));
}

TEST_CASE("Vector - Initializer List Partial Construction", "[vector][core][construction]") {
    Vector<4> v = {1.0f, 2.0f};
    
    REQUIRE_THAT(v[0], Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(v[1], Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(v[2], Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(v[3], Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Vector - Initializer List Single Element", "[vector][core][construction]") {
    Vector<3> v = {5.0f};
    
    REQUIRE_THAT(v[0], Catch::Matchers::WithinAbs(5.0, 1e-6));
    REQUIRE_THAT(v[1], Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(v[2], Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Vector - Initializer List Excess Elements Ignored", "[vector][core][construction]") {
    Vector<2> v = {1.0f, 2.0f, 99.0f, 88.0f};
    
    REQUIRE_THAT(v[0], Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(v[1], Catch::Matchers::WithinAbs(2.0, 1e-6));
}

TEST_CASE("Vector - Variadic Construction 2D", "[vector][core][construction]") {
    Vector<2> v(1.0f, 2.0f);
    
    REQUIRE_THAT(v[0], Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(v[1], Catch::Matchers::WithinAbs(2.0, 1e-6));
}

TEST_CASE("Vector - Variadic Construction 3D", "[vector][core][construction]") {
    Vector<3> v(1.0f, 2.0f, 3.0f);
    
    REQUIRE_THAT(v[0], Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(v[1], Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(v[2], Catch::Matchers::WithinAbs(3.0, 1e-6));
}

TEST_CASE("Vector - Variadic Construction 4D", "[vector][core][construction]") {
    Vector<4> v(1.0f, 2.0f, 3.0f, 4.0f);
    
    REQUIRE_THAT(v[0], Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(v[1], Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(v[2], Catch::Matchers::WithinAbs(3.0, 1e-6));
    REQUIRE_THAT(v[3], Catch::Matchers::WithinAbs(4.0, 1e-6));
}

TEST_CASE("Vector - Variadic Construction With Negative Values", "[vector][core][construction]") {
    Vector<3> v(-1.0f, 2.5f, -3.7f);
    
    REQUIRE_THAT(v[0], Catch::Matchers::WithinAbs(-1.0, 1e-6));
    REQUIRE_THAT(v[1], Catch::Matchers::WithinAbs(2.5, 1e-6));
    REQUIRE_THAT(v[2], Catch::Matchers::WithinAbs(-3.7, 1e-6));
}

// ----------------------------------------------------------------------------
// Factory Method Tests
// ----------------------------------------------------------------------------

TEST_CASE("Vector - Zero Factory Creates All Zeros", "[vector][core][factory]") {
    Vector<3> v = Vector<3>::zero();
    
    REQUIRE_THAT(v[0], Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(v[1], Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(v[2], Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Vector - Zero Factory Various Sizes", "[vector][core][factory]") {
    Vector<1> v1 = Vector<1>::zero();
    Vector<5> v5 = Vector<5>::zero();
    
    REQUIRE_THAT(v1[0], Catch::Matchers::WithinAbs(0.0, 1e-6));
    for(uint8_t i = 0; i < 5; i++) {
        REQUIRE_THAT(v5[i], Catch::Matchers::WithinAbs(0.0, 1e-6));
    }
}

TEST_CASE("Vector - Unit X Factory 2D", "[vector][core][factory]") {
    Vector<2> v = Vector<2>::unitX();
    
    REQUIRE_THAT(v[0], Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(v[1], Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Vector - Unit X Factory 3D", "[vector][core][factory]") {
    Vector<3> v = Vector<3>::unitX();
    
    REQUIRE_THAT(v[0], Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(v[1], Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(v[2], Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Vector - Unit Y Factory 2D", "[vector][core][factory]") {
    Vector<2> v = Vector<2>::unitY();
    
    REQUIRE_THAT(v[0], Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(v[1], Catch::Matchers::WithinAbs(1.0, 1e-6));
}

TEST_CASE("Vector - Unit Y Factory 3D", "[vector][core][factory]") {
    Vector<3> v = Vector<3>::unitY();
    
    REQUIRE_THAT(v[0], Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(v[1], Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(v[2], Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Vector - Unit Z Factory 3D", "[vector][core][factory]") {
    Vector<3> v = Vector<3>::unitZ();
    
    REQUIRE_THAT(v[0], Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(v[1], Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(v[2], Catch::Matchers::WithinAbs(1.0, 1e-6));
}

TEST_CASE("Vector - From Array Factory", "[vector][core][factory]") {
    std::array<float, 3> arr = {4.0f, 5.0f, 6.0f};
    Vector<3> v = Vector<3>::fromArray(arr);
    
    REQUIRE_THAT(v[0], Catch::Matchers::WithinAbs(4.0, 1e-6));
    REQUIRE_THAT(v[1], Catch::Matchers::WithinAbs(5.0, 1e-6));
    REQUIRE_THAT(v[2], Catch::Matchers::WithinAbs(6.0, 1e-6));
}

TEST_CASE("Vector - From Array Factory 2D", "[vector][core][factory]") {
    std::array<float, 2> arr = {-1.5f, 2.7f};
    Vector<2> v = Vector<2>::fromArray(arr);
    
    REQUIRE_THAT(v[0], Catch::Matchers::WithinAbs(-1.5, 1e-6));
    REQUIRE_THAT(v[1], Catch::Matchers::WithinAbs(2.7, 1e-6));
}

TEST_CASE("Vector - From Array Factory Large Vector", "[vector][core][factory]") {
    std::array<float, 6> arr = {1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f};
    Vector<6> v = Vector<6>::fromArray(arr);
    
    for(uint8_t i = 0; i < 6; i++) {
        REQUIRE_THAT(v[i], Catch::Matchers::WithinAbs(arr[i], 1e-6));
    }
}

// ----------------------------------------------------------------------------
// Getter Tests
// ----------------------------------------------------------------------------

TEST_CASE("Vector - Size Getter", "[vector][core][getter]") {
    Vector<5> v;
    REQUIRE(v.size() == 5);
}

TEST_CASE("Vector - Size Getter Various Sizes", "[vector][core][getter]") {
    REQUIRE(Vector<1>::zero().size() == 1);
    REQUIRE(Vector<2>::zero().size() == 2);
    REQUIRE(Vector<3>::zero().size() == 3);
    REQUIRE(Vector<4>::zero().size() == 4);
    REQUIRE(Vector<12>::zero().size() == 12);
}

// ----------------------------------------------------------------------------
// Special Accessor Tests
// ----------------------------------------------------------------------------

TEST_CASE("Vector - X Accessor 2D", "[vector][core][accessor]") {
    Vector<2> v = {3.0f, 4.0f};
    
    REQUIRE_THAT(v.x(), Catch::Matchers::WithinAbs(3.0, 1e-6));
}

TEST_CASE("Vector - X Accessor 3D", "[vector][core][accessor]") {
    Vector<3> v = {1.0f, 2.0f, 3.0f};
    
    REQUIRE_THAT(v.x(), Catch::Matchers::WithinAbs(1.0, 1e-6));
}

TEST_CASE("Vector - Y Accessor 2D", "[vector][core][accessor]") {
    Vector<2> v = {3.0f, 4.0f};
    
    REQUIRE_THAT(v.y(), Catch::Matchers::WithinAbs(4.0, 1e-6));
}

TEST_CASE("Vector - Y Accessor 3D", "[vector][core][accessor]") {
    Vector<3> v = {1.0f, 2.0f, 3.0f};
    
    REQUIRE_THAT(v.y(), Catch::Matchers::WithinAbs(2.0, 1e-6));
}

TEST_CASE("Vector - Z Accessor 3D", "[vector][core][accessor]") {
    Vector<3> v = {1.0f, 2.0f, 3.0f};
    
    REQUIRE_THAT(v.z(), Catch::Matchers::WithinAbs(3.0, 1e-6));
}

TEST_CASE("Vector - X Accessor Modification 2D", "[vector][core][accessor]") {
    Vector<2> v = {3.0f, 4.0f};
    
    v.x() = 10.0f;
    
    REQUIRE_THAT(v.x(), Catch::Matchers::WithinAbs(10.0, 1e-6));
    REQUIRE_THAT(v[0], Catch::Matchers::WithinAbs(10.0, 1e-6));
}

TEST_CASE("Vector - Y Accessor Modification 3D", "[vector][core][accessor]") {
    Vector<3> v = {1.0f, 2.0f, 3.0f};
    
    v.y() = 20.0f;
    
    REQUIRE_THAT(v.y(), Catch::Matchers::WithinAbs(20.0, 1e-6));
    REQUIRE_THAT(v[1], Catch::Matchers::WithinAbs(20.0, 1e-6));
}

TEST_CASE("Vector - Z Accessor Modification", "[vector][core][accessor]") {
    Vector<3> v = {1.0f, 2.0f, 3.0f};
    
    v.z() = 9.0f;
    
    REQUIRE_THAT(v.z(), Catch::Matchers::WithinAbs(9.0, 1e-6));
    REQUIRE_THAT(v[2], Catch::Matchers::WithinAbs(9.0, 1e-6));
}

TEST_CASE("Vector - XYZ Accessors Combined Modification", "[vector][core][accessor]") {
    Vector<3> v;
    
    v.x() = 7.0f;
    v.y() = 8.0f;
    v.z() = 9.0f;
    
    REQUIRE_THAT(v.x(), Catch::Matchers::WithinAbs(7.0, 1e-6));
    REQUIRE_THAT(v.y(), Catch::Matchers::WithinAbs(8.0, 1e-6));
    REQUIRE_THAT(v.z(), Catch::Matchers::WithinAbs(9.0, 1e-6));
}

TEST_CASE("Vector - Const X Accessor 3D", "[vector][core][accessor]") {
    const Vector<3> v = {1.0f, 2.0f, 3.0f};
    
    REQUIRE_THAT(v.x(), Catch::Matchers::WithinAbs(1.0, 1e-6));
}

TEST_CASE("Vector - Const Y Accessor 3D", "[vector][core][accessor]") {
    const Vector<3> v = {1.0f, 2.0f, 3.0f};
    
    REQUIRE_THAT(v.y(), Catch::Matchers::WithinAbs(2.0, 1e-6));
}

TEST_CASE("Vector - Const Z Accessor 3D", "[vector][core][accessor]") {
    const Vector<3> v = {1.0f, 2.0f, 3.0f};
    
    REQUIRE_THAT(v.z(), Catch::Matchers::WithinAbs(3.0, 1e-6));
}

// ----------------------------------------------------------------------------
// Element Access Tests
// ----------------------------------------------------------------------------

TEST_CASE("Vector - Element Access Operator", "[vector][core][access]") {
    Vector<3> v = {1.0f, 2.0f, 3.0f};
    
    REQUIRE_THAT(v[0], Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(v[1], Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(v[2], Catch::Matchers::WithinAbs(3.0, 1e-6));
}

TEST_CASE("Vector - Element Access First and Last", "[vector][core][access]") {
    Vector<5> v = {1.0f, 2.0f, 3.0f, 4.0f, 5.0f};
    
    REQUIRE_THAT(v[0], Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(v[4], Catch::Matchers::WithinAbs(5.0, 1e-6));
}

TEST_CASE("Vector - Element Modification", "[vector][core][access]") {
    Vector<3> v;
    
    v[0] = 5.0f;
    v[1] = 6.0f;
    v[2] = 7.0f;
    
    REQUIRE_THAT(v[0], Catch::Matchers::WithinAbs(5.0, 1e-6));
    REQUIRE_THAT(v[1], Catch::Matchers::WithinAbs(6.0, 1e-6));
    REQUIRE_THAT(v[2], Catch::Matchers::WithinAbs(7.0, 1e-6));
}

TEST_CASE("Vector - Element Modification Chain", "[vector][core][access]") {
    Vector<3> v;
    
    v[0] = 1.0f;
    v[0] += 2.0f;
    v[0] *= 3.0f;
    
    REQUIRE_THAT(v[0], Catch::Matchers::WithinAbs(9.0, 1e-6));
}

TEST_CASE("Vector - Const Element Access", "[vector][core][access]") {
    const Vector<3> v = {1.0f, 2.0f, 3.0f};
    
    REQUIRE_THAT(v[0], Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(v[1], Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(v[2], Catch::Matchers::WithinAbs(3.0, 1e-6));
}

TEST_CASE("Vector - Safe At Method In Bounds", "[vector][core][access]") {
    Vector<3> v = {1.0f, 2.0f, 3.0f};
    
    REQUIRE_THAT(v.at(0), Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(v.at(2), Catch::Matchers::WithinAbs(3.0, 1e-6));
}

TEST_CASE("Vector - Safe At Method Clamps Out of Bounds", "[vector][core][access]") {
    Vector<3> v = {1.0f, 2.0f, 3.0f};
    
    REQUIRE_THAT(v.at(10), Catch::Matchers::WithinAbs(3.0, 1e-6));
    REQUIRE_THAT(v.at(100), Catch::Matchers::WithinAbs(3.0, 1e-6));
}

TEST_CASE("Vector - Safe At Method Modification", "[vector][core][access]") {
    Vector<3> v;
    
    v.at(0) = 5.0f;
    v.at(1) = 6.0f;
    
    REQUIRE_THAT(v.at(0), Catch::Matchers::WithinAbs(5.0, 1e-6));
    REQUIRE_THAT(v.at(1), Catch::Matchers::WithinAbs(6.0, 1e-6));
}

TEST_CASE("Vector - Safe At Method Const Version", "[vector][core][access]") {
    const Vector<3> v = {1.0f, 2.0f, 3.0f};
    
    REQUIRE_THAT(v.at(0), Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(v.at(10), Catch::Matchers::WithinAbs(3.0, 1e-6));
}

TEST_CASE("Vector - Data Pointer Access", "[vector][core][access]") {
    Vector<3> v = {1.0f, 2.0f, 3.0f};
    
    float* ptr = v.data();
    REQUIRE(ptr != nullptr);
    REQUIRE_THAT(ptr[0], Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(ptr[2], Catch::Matchers::WithinAbs(3.0, 1e-6));
}

TEST_CASE("Vector - Data Pointer Modification", "[vector][core][access]") {
    Vector<3> v;
    float* ptr = v.data();
    
    ptr[0] = 5.0f;
    ptr[1] = 6.0f;
    ptr[2] = 7.0f;
    
    REQUIRE_THAT(v[0], Catch::Matchers::WithinAbs(5.0, 1e-6));
    REQUIRE_THAT(v[1], Catch::Matchers::WithinAbs(6.0, 1e-6));
    REQUIRE_THAT(v[2], Catch::Matchers::WithinAbs(7.0, 1e-6));
}

TEST_CASE("Vector - Const Data Pointer Access", "[vector][core][access]") {
    const Vector<3> v = {1.0f, 2.0f, 3.0f};
    
    const float* ptr = v.data();
    REQUIRE(ptr != nullptr);
    REQUIRE_THAT(ptr[0], Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(ptr[1], Catch::Matchers::WithinAbs(2.0, 1e-6));
}

TEST_CASE("Vector - Iterator Begin and End", "[vector][core][access]") {
    Vector<3> v = {1.0f, 2.0f, 3.0f};
    
    auto it = v.begin();
    REQUIRE(it != v.end());
    REQUIRE_THAT(*it, Catch::Matchers::WithinAbs(1.0, 1e-6));
}

TEST_CASE("Vector - Iterator Range Loop", "[vector][core][access]") {
    Vector<4> v = {1.0f, 2.0f, 3.0f, 4.0f};
    
    float sum = 0.0f;
    for(auto val : v) {
        sum += val;
    }
    
    REQUIRE_THAT(sum, Catch::Matchers::WithinAbs(10.0, 1e-6));
}

TEST_CASE("Vector - Iterator Modification", "[vector][core][access]") {
    Vector<3> v = {1.0f, 2.0f, 3.0f};
    
    for(auto it = v.begin(); it != v.end(); ++it) {
        *it *= 2.0f;
    }
    
    REQUIRE_THAT(v[0], Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(v[1], Catch::Matchers::WithinAbs(4.0, 1e-6));
    REQUIRE_THAT(v[2], Catch::Matchers::WithinAbs(6.0, 1e-6));
}

// ----------------------------------------------------------------------------
// Arithmetic Assignment Operator Tests
// ----------------------------------------------------------------------------

TEST_CASE("Vector - Addition Assignment", "[vector][core][arithmetic]") {
    Vector<3> v1 = {1.0f, 2.0f, 3.0f};
    Vector<3> v2 = {4.0f, 5.0f, 6.0f};
    
    v1 += v2;
    
    REQUIRE_THAT(v1[0], Catch::Matchers::WithinAbs(5.0, 1e-6));
    REQUIRE_THAT(v1[1], Catch::Matchers::WithinAbs(7.0, 1e-6));
    REQUIRE_THAT(v1[2], Catch::Matchers::WithinAbs(9.0, 1e-6));
}

TEST_CASE("Vector - Addition Assignment With Zero", "[vector][core][arithmetic]") {
    Vector<3> v1 = {1.0f, 2.0f, 3.0f};
    Vector<3> v2 = Vector<3>::zero();
    
    Vector<3> original = v1;
    v1 += v2;
    
    REQUIRE(v1 == original);
}

TEST_CASE("Vector - Addition Assignment Chain", "[vector][core][arithmetic]") {
    Vector<3> v1 = {1.0f, 2.0f, 3.0f};
    Vector<3> v2 = {1.0f, 1.0f, 1.0f};
    
    v1 += v2;
    v1 += v2;
    v1 += v2;
    
    REQUIRE_THAT(v1[0], Catch::Matchers::WithinAbs(4.0, 1e-6));
    REQUIRE_THAT(v1[1], Catch::Matchers::WithinAbs(5.0, 1e-6));
    REQUIRE_THAT(v1[2], Catch::Matchers::WithinAbs(6.0, 1e-6));
}

TEST_CASE("Vector - Subtraction Assignment", "[vector][core][arithmetic]") {
    Vector<3> v1 = {5.0f, 7.0f, 9.0f};
    Vector<3> v2 = {1.0f, 2.0f, 3.0f};
    
    v1 -= v2;
    
    REQUIRE_THAT(v1[0], Catch::Matchers::WithinAbs(4.0, 1e-6));
    REQUIRE_THAT(v1[1], Catch::Matchers::WithinAbs(5.0, 1e-6));
    REQUIRE_THAT(v1[2], Catch::Matchers::WithinAbs(6.0, 1e-6));
}

TEST_CASE("Vector - Subtraction Assignment Self Zeros", "[vector][core][arithmetic]") {
    Vector<3> v1 = {1.0f, 2.0f, 3.0f};
    Vector<3> v2 = v1;
    
    v1 -= v2;
    
    REQUIRE(v1 == Vector<3>::zero());
}

TEST_CASE("Vector - Subtraction Assignment Chain", "[vector][core][arithmetic]") {
    Vector<3> v1 = {10.0f, 20.0f, 30.0f};
    Vector<3> v2 = {2.0f, 4.0f, 6.0f};
    
    v1 -= v2;
    v1 -= v2;
    
    REQUIRE_THAT(v1[0], Catch::Matchers::WithinAbs(6.0, 1e-6));
    REQUIRE_THAT(v1[1], Catch::Matchers::WithinAbs(12.0, 1e-6));
    REQUIRE_THAT(v1[2], Catch::Matchers::WithinAbs(18.0, 1e-6));
}

TEST_CASE("Vector - Scalar Multiplication Assignment", "[vector][core][arithmetic]") {
    Vector<3> v = {1.0f, 2.0f, 3.0f};
    
    v *= 2.0f;
    
    REQUIRE_THAT(v[0], Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(v[1], Catch::Matchers::WithinAbs(4.0, 1e-6));
    REQUIRE_THAT(v[2], Catch::Matchers::WithinAbs(6.0, 1e-6));
}

TEST_CASE("Vector - Scalar Multiplication Assignment Zero", "[vector][core][arithmetic]") {
    Vector<3> v = {1.0f, 2.0f, 3.0f};
    
    v *= 0.0f;
    
    REQUIRE(v == Vector<3>::zero());
}

TEST_CASE("Vector - Scalar Multiplication Assignment One", "[vector][core][arithmetic]") {
    Vector<3> v = {1.0f, 2.0f, 3.0f};
    Vector<3> original = v;
    
    v *= 1.0f;
    
    REQUIRE(v == original);
}

TEST_CASE("Vector - Scalar Multiplication Assignment Negative", "[vector][core][arithmetic]") {
    Vector<3> v = {1.0f, -2.0f, 3.0f};
    
    v *= -2.0f;
    
    REQUIRE_THAT(v[0], Catch::Matchers::WithinAbs(-2.0, 1e-6));
    REQUIRE_THAT(v[1], Catch::Matchers::WithinAbs(4.0, 1e-6));
    REQUIRE_THAT(v[2], Catch::Matchers::WithinAbs(-6.0, 1e-6));
}

TEST_CASE("Vector - Scalar Multiplication Assignment Chain", "[vector][core][arithmetic]") {
    Vector<3> v = {1.0f, 2.0f, 3.0f};
    
    v *= 2.0f;
    v *= 3.0f;
    
    REQUIRE_THAT(v[0], Catch::Matchers::WithinAbs(6.0, 1e-6));
    REQUIRE_THAT(v[1], Catch::Matchers::WithinAbs(12.0, 1e-6));
    REQUIRE_THAT(v[2], Catch::Matchers::WithinAbs(18.0, 1e-6));
}

TEST_CASE("Vector - Scalar Division Assignment", "[vector][core][arithmetic]") {
    Vector<3> v = {2.0f, 4.0f, 6.0f};
    
    v /= 2.0f;
    
    REQUIRE_THAT(v[0], Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(v[1], Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(v[2], Catch::Matchers::WithinAbs(3.0, 1e-6));
}

TEST_CASE("Vector - Scalar Division Assignment By One", "[vector][core][arithmetic]") {
    Vector<3> v = {1.0f, 2.0f, 3.0f};
    Vector<3> original = v;
    
    v /= 1.0f;
    
    REQUIRE(v == original);
}

TEST_CASE("Vector - Scalar Division Assignment Fractional", "[vector][core][arithmetic]") {
    Vector<3> v = {1.0f, 2.0f, 3.0f};
    
    v /= 0.5f;
    
    REQUIRE_THAT(v[0], Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(v[1], Catch::Matchers::WithinAbs(4.0, 1e-6));
    REQUIRE_THAT(v[2], Catch::Matchers::WithinAbs(6.0, 1e-6));
}

TEST_CASE("Vector - Scalar Division Assignment Negative", "[vector][core][arithmetic]") {
    Vector<3> v = {2.0f, -4.0f, 6.0f};
    
    v /= -2.0f;
    
    REQUIRE_THAT(v[0], Catch::Matchers::WithinAbs(-1.0, 1e-6));
    REQUIRE_THAT(v[1], Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(v[2], Catch::Matchers::WithinAbs(-3.0, 1e-6));
}

TEST_CASE("Vector - Scalar Division Assignment Chain", "[vector][core][arithmetic]") {
    Vector<3> v = {24.0f, 48.0f, 72.0f};
    
    v /= 2.0f;
    v /= 3.0f;
    
    REQUIRE_THAT(v[0], Catch::Matchers::WithinAbs(4.0, 1e-6));
    REQUIRE_THAT(v[1], Catch::Matchers::WithinAbs(8.0, 1e-6));
    REQUIRE_THAT(v[2], Catch::Matchers::WithinAbs(12.0, 1e-6));
}

TEST_CASE("Vector - Mixed Arithmetic Assignments", "[vector][core][arithmetic]") {
    Vector<3> v = {2.0f, 4.0f, 6.0f};
    
    v *= 2.0f;
    v += Vector<3>{1.0f, 1.0f, 1.0f};
    v /= 5.0f;
    
    REQUIRE_THAT(v[0], Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(v[1], Catch::Matchers::WithinAbs(1.8, 1e-6));
    REQUIRE_THAT(v[2], Catch::Matchers::WithinAbs(2.6, 1e-6));
}

// ----------------------------------------------------------------------------
// Different Type Tests
// ----------------------------------------------------------------------------

TEST_CASE("Vector - Double Type Construction", "[vector][core][type]") {
    Vector<3, double> v = {1.0, 2.0, 3.0};
    
    REQUIRE(v[0] == 1.0);
    REQUIRE(v[1] == 2.0);
    REQUIRE(v[2] == 3.0);
}

TEST_CASE("Vector - Int Type Construction", "[vector][core][type]") {
    Vector<3, int> v = {1, 2, 3};
    
    REQUIRE(v[0] == 1);
    REQUIRE(v[1] == 2);
    REQUIRE(v[2] == 3);
}

TEST_CASE("Vector - Double Type Operations", "[vector][core][type]") {
    Vector<3, double> v1 = {1.0, 2.0, 3.0};
    Vector<3, double> v2 = {4.0, 5.0, 6.0};
    
    v1 += v2;
    
    REQUIRE(v1[0] == 5.0);
    REQUIRE(v1[1] == 7.0);
    REQUIRE(v1[2] == 9.0);
}