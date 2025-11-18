#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "cobalt/math/linear_algebra/matrix/matrix.hpp"
#include "cobalt/math/linear_algebra/matrix/matrix_ops.hpp"

using namespace cobalt::math::linear_algebra;

// ============================================================================
// Matrix Operations Tests (matrix_ops.hpp)
// ============================================================================

TEST_CASE("Matrix - Addition Operator", "[matrix][ops]") {
    Matrix<2, 2> m1 = {
        {1.0f, 2.0f},
        {3.0f, 4.0f}
    };
    Matrix<2, 2> m2 = {
        {5.0f, 6.0f},
        {7.0f, 8.0f}
    };
    
    Matrix<2, 2> result = m1 + m2;
    
    REQUIRE_THAT(result(0, 0), Catch::Matchers::WithinAbs(6.0, 1e-6));
    REQUIRE_THAT(result(0, 1), Catch::Matchers::WithinAbs(8.0, 1e-6));
    REQUIRE_THAT(result(1, 0), Catch::Matchers::WithinAbs(10.0, 1e-6));
    REQUIRE_THAT(result(1, 1), Catch::Matchers::WithinAbs(12.0, 1e-6));
}

TEST_CASE("Matrix - Subtraction Operator", "[matrix][ops]") {
    Matrix<2, 2> m1 = {
        {5.0f, 6.0f},
        {7.0f, 8.0f}
    };
    Matrix<2, 2> m2 = {
        {1.0f, 2.0f},
        {3.0f, 4.0f}
    };
    
    Matrix<2, 2> result = m1 - m2;
    
    REQUIRE_THAT(result(0, 0), Catch::Matchers::WithinAbs(4.0, 1e-6));
    REQUIRE_THAT(result(0, 1), Catch::Matchers::WithinAbs(4.0, 1e-6));
    REQUIRE_THAT(result(1, 0), Catch::Matchers::WithinAbs(4.0, 1e-6));
    REQUIRE_THAT(result(1, 1), Catch::Matchers::WithinAbs(4.0, 1e-6));
}

TEST_CASE("Matrix - Scalar Multiplication", "[matrix][ops]") {
    Matrix<2, 2> m = {
        {1.0f, 2.0f},
        {3.0f, 4.0f}
    };
    
    Matrix<2, 2> result = m * 3.0f;
    
    REQUIRE_THAT(result(0, 0), Catch::Matchers::WithinAbs(3.0, 1e-6));
    REQUIRE_THAT(result(0, 1), Catch::Matchers::WithinAbs(6.0, 1e-6));
    REQUIRE_THAT(result(1, 0), Catch::Matchers::WithinAbs(9.0, 1e-6));
    REQUIRE_THAT(result(1, 1), Catch::Matchers::WithinAbs(12.0, 1e-6));
}

TEST_CASE("Matrix - Scalar Multiplication Commutative", "[matrix][ops]") {
    Matrix<2, 2> m = {
        {1.0f, 2.0f},
        {3.0f, 4.0f}
    };
    
    Matrix<2, 2> result = 3.0f * m;
    
    REQUIRE_THAT(result(0, 0), Catch::Matchers::WithinAbs(3.0, 1e-6));
    REQUIRE_THAT(result(0, 1), Catch::Matchers::WithinAbs(6.0, 1e-6));
    REQUIRE_THAT(result(1, 0), Catch::Matchers::WithinAbs(9.0, 1e-6));
    REQUIRE_THAT(result(1, 1), Catch::Matchers::WithinAbs(12.0, 1e-6));
}

TEST_CASE("Matrix - Matrix Multiplication", "[matrix][ops]") {
    Matrix<2, 3> m1 = {
        {1.0f, 2.0f, 3.0f},
        {4.0f, 5.0f, 6.0f}
    };
    Matrix<3, 2> m2 = {
        {7.0f, 8.0f},
        {9.0f, 10.0f},
        {11.0f, 12.0f}
    };
    
    Matrix<2, 2> result = m1 * m2;
    
    // [1 2 3] * [7  8 ] = [58  64]
    // [4 5 6]   [9  10]   [139 154]
    //           [11 12]
    REQUIRE_THAT(result(0, 0), Catch::Matchers::WithinAbs(58.0, 1e-6));
    REQUIRE_THAT(result(0, 1), Catch::Matchers::WithinAbs(64.0, 1e-6));
    REQUIRE_THAT(result(1, 0), Catch::Matchers::WithinAbs(139.0, 1e-6));
    REQUIRE_THAT(result(1, 1), Catch::Matchers::WithinAbs(154.0, 1e-6));
}

TEST_CASE("Matrix - Matrix-Vector Multiplication", "[matrix][ops]") {
    Matrix<2, 3> m = {
        {1.0f, 2.0f, 3.0f},
        {4.0f, 5.0f, 6.0f}
    };
    Vector<3> v = {1.0f, 2.0f, 3.0f};
    
    Vector<2> result = m * v;
    
    // [1 2 3] * [1] = [14]
    // [4 5 6]   [2]   [32]
    //           [3]
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(14.0, 1e-6));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(32.0, 1e-6));
}

TEST_CASE("Matrix - Unary Negation", "[matrix][ops]") {
    Matrix<2, 2> m = {
        {1.0f, -2.0f},
        {-3.0f, 4.0f}
    };
    
    Matrix<2, 2> result = -m;
    
    REQUIRE_THAT(result(0, 0), Catch::Matchers::WithinAbs(-1.0, 1e-6));
    REQUIRE_THAT(result(0, 1), Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(result(1, 0), Catch::Matchers::WithinAbs(3.0, 1e-6));
    REQUIRE_THAT(result(1, 1), Catch::Matchers::WithinAbs(-4.0, 1e-6));
}

TEST_CASE("Matrix - Scalar Division", "[matrix][ops]") {
    Matrix<2, 2> m = {
        {2.0f, 4.0f},
        {6.0f, 8.0f}
    };
    
    Matrix<2, 2> result = m / 2.0f;
    
    REQUIRE_THAT(result(0, 0), Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(result(0, 1), Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(result(1, 0), Catch::Matchers::WithinAbs(3.0, 1e-6));
    REQUIRE_THAT(result(1, 1), Catch::Matchers::WithinAbs(4.0, 1e-6));
}

TEST_CASE("Matrix - Equality Operator", "[matrix][ops]") {
    Matrix<2, 2> m1 = {
        {1.0f, 2.0f},
        {3.0f, 4.0f}
    };
    Matrix<2, 2> m2 = {
        {1.0f, 2.0f},
        {3.0f, 4.0f}
    };
    Matrix<2, 2> m3 = {
        {1.0f, 2.0f},
        {3.0f, 4.1f}
    };
    
    REQUIRE(m1 == m2);
    REQUIRE_FALSE(m1 == m3);
}

TEST_CASE("Matrix - Inequality Operator", "[matrix][ops]") {
    Matrix<2, 2> m1 = {
        {1.0f, 2.0f},
        {3.0f, 4.0f}
    };
    Matrix<2, 2> m2 = {
        {1.0f, 2.0f},
        {3.0f, 4.0f}
    };
    Matrix<2, 2> m3 = {
        {1.0f, 2.0f},
        {3.0f, 4.1f}
    };
    
    REQUIRE_FALSE(m1 != m2);
    REQUIRE(m1 != m3);
}

TEST_CASE("Matrix - Determinant 2x2", "[matrix][ops]") {
    Matrix<2, 2> m = {
        {1.0f, 2.0f},
        {3.0f, 4.0f}
    };
    
    float result = det(m);
    
    // det = 1*4 - 2*3 = -2
    REQUIRE_THAT(result, Catch::Matchers::WithinAbs(-2.0, 1e-6));
}

TEST_CASE("Matrix - Determinant 3x3", "[matrix][ops]") {
    Matrix<3, 3> m = {
        {1.0f, 2.0f, 3.0f},
        {0.0f, 1.0f, 4.0f},
        {5.0f, 6.0f, 0.0f}
    };
    
    float result = det(m);
    
    // det = 1*(0-24) - 2*(0-20) + 3*(0-5) = -24 + 40 - 15 = 1
    REQUIRE_THAT(result, Catch::Matchers::WithinAbs(1.0, 1e-6));
}

TEST_CASE("Matrix - Determinant Identity", "[matrix][ops]") {
    Matrix<3, 3> m = Matrix<3, 3>::eye();
    
    float result = det(m);
    
    REQUIRE_THAT(result, Catch::Matchers::WithinAbs(1.0, 1e-6));
}