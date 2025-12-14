#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "cobalt/math/linear_algebra/matrix/matrix.hpp"
#include "cobalt/math/linear_algebra/matrix/matrix_ops.hpp"

using namespace cobalt::math::linear_algebra;

// ================================================================================
// Matrix Operations Tests (matrix_ops.hpp)
// ================================================================================

// --------------------------------------------------------------------------------
// Binary Arithmetic Operators
// --------------------------------------------------------------------------------

TEST_CASE("Matrix - Addition Operator", "[matrix][ops][arithmetic]") {
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

TEST_CASE("Matrix - Addition Preserves Operands", "[matrix][ops][arithmetic]") {
    Matrix<2, 2> m1 = {
        {1.0f, 2.0f},
        {3.0f, 4.0f}
    };
    Matrix<2, 2> m2 = {
        {5.0f, 6.0f},
        {7.0f, 8.0f}
    };
    Matrix<2, 2> m1_copy = m1;
    Matrix<2, 2> m2_copy = m2;

    Matrix<2, 2> result = m1 + m2;

    REQUIRE(m1 == m1_copy);
    REQUIRE(m2 == m2_copy);
}

TEST_CASE("Matrix - Addition With Zero", "[matrix][ops][arithmetic]") {
    Matrix<2, 2> m = {
        {1.0f, 2.0f},
        {3.0f, 4.0f}
    };
    Matrix<2, 2> zero = Matrix<2, 2>::zero();

    Matrix<2, 2> result = m + zero;

    REQUIRE(result == m);
}

TEST_CASE("Matrix - Addition Commutative", "[matrix][ops][arithmetic]") {
    Matrix<2, 2> m1 = {
        {1.0f, 2.0f},
        {3.0f, 4.0f}
    };
    Matrix<2, 2> m2 = {
        {5.0f, 6.0f},
        {7.0f, 8.0f}
    };

    Matrix<2, 2> result1 = m1 + m2;
    Matrix<2, 2> result2 = m2 + m1;

    REQUIRE(result1 == result2);
}

TEST_CASE("Matrix - Subtraction Operator", "[matrix][ops][arithmetic]") {
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

TEST_CASE("Matrix - Subtraction Self Zeros", "[matrix][ops][arithmetic]") {
    Matrix<2, 2> m = {
        {1.0f, 2.0f},
        {3.0f, 4.0f}
    };

    Matrix<2, 2> result = m - m;

    REQUIRE(result == Matrix<2, 2>::zero());
}

TEST_CASE("Matrix - Subtraction Not Commutative", "[matrix][ops][arithmetic]") {
    Matrix<2, 2> m1 = {
        {1.0f, 2.0f},
        {3.0f, 4.0f}
    };
    Matrix<2, 2> m2 = {
        {5.0f, 6.0f},
        {7.0f, 8.0f}
    };

    Matrix<2, 2> result1 = m1 - m2;
    Matrix<2, 2> result2 = m2 - m1;

    REQUIRE(result1 != result2);
    REQUIRE(result1 == -result2);
}

TEST_CASE("Matrix - Scalar Multiplication Right", "[matrix][ops][arithmetic]") {
    Matrix<2, 2> m = {
        {1.0f, 2.0f},
        {3.0f, 4.0f}
    };

    Matrix<2, 2> result = m * 2.0f;

    REQUIRE_THAT(result(0, 0), Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(result(0, 1), Catch::Matchers::WithinAbs(4.0, 1e-6));
    REQUIRE_THAT(result(1, 0), Catch::Matchers::WithinAbs(6.0, 1e-6));
    REQUIRE_THAT(result(1, 1), Catch::Matchers::WithinAbs(8.0, 1e-6));
}

TEST_CASE("Matrix - Scalar Multiplication Left", "[matrix][ops][arithmetic]") {
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

TEST_CASE("Matrix - Scalar Multiplication Commutative", "[matrix][ops][arithmetic]") {
    Matrix<2, 2> m = {
        {1.0f, 2.0f},
        {3.0f, 4.0f}
    };

    Matrix<2, 2> result1 = m * 5.0f;
    Matrix<2, 2> result2 = 5.0f * m;

    REQUIRE(result1 == result2);
}

TEST_CASE("Matrix - Scalar Multiplication By Zero", "[matrix][ops][arithmetic]") {
    Matrix<2, 2> m = {
        {1.0f, 2.0f},
        {3.0f, 4.0f}
    };

    Matrix<2, 2> result = m * 0.0f;

    REQUIRE(result == Matrix<2, 2>::zero());
}

TEST_CASE("Matrix - Scalar Multiplication By One", "[matrix][ops][arithmetic]") {
    Matrix<2, 2> m = {
        {1.0f, 2.0f},
        {3.0f, 4.0f}
    };

    Matrix<2, 2> result = m * 1.0f;

    REQUIRE(result == m);
}

TEST_CASE("Matrix - Scalar Multiplication Negative", "[matrix][ops][arithmetic]") {
    Matrix<2, 2> m = {
        {1.0f, 2.0f},
        {3.0f, 4.0f}
    };

    Matrix<2, 2> result = m * -2.0f;

    REQUIRE_THAT(result(0, 0), Catch::Matchers::WithinAbs(-2.0, 1e-6));
    REQUIRE_THAT(result(1, 1), Catch::Matchers::WithinAbs(-8.0, 1e-6));
}

TEST_CASE("Matrix - Matrix Multiplication 2x2", "[matrix][ops][arithmetic]") {
    Matrix<2, 2> m1 = {
        {1.0f, 2.0f},
        {3.0f, 4.0f}
    };
    Matrix<2, 2> m2 = {
        {2.0f, 0.0f},
        {1.0f, 2.0f}
    };

    Matrix<2, 2> result = m1 * m2;

    // [1 2] * [2 0] = [4 4]
    // [3 4]   [1 2]   [10 8]
    REQUIRE_THAT(result(0, 0), Catch::Matchers::WithinAbs(4.0, 1e-6));
    REQUIRE_THAT(result(0, 1), Catch::Matchers::WithinAbs(4.0, 1e-6));
    REQUIRE_THAT(result(1, 0), Catch::Matchers::WithinAbs(10.0, 1e-6));
    REQUIRE_THAT(result(1, 1), Catch::Matchers::WithinAbs(8.0, 1e-6));
}

TEST_CASE("Matrix - Matrix Multiplication 3x3", "[matrix][ops][arithmetic]") {
    Matrix<3, 3> m1 = {
        {1.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f},
        {0.0f, 0.0f, 1.0f}
    };
    Matrix<3, 3> m2 = {
        {2.0f, 3.0f, 4.0f},
        {5.0f, 6.0f, 7.0f},
        {8.0f, 9.0f, 10.0f}
    };

    Matrix<3, 3> result = m1 * m2;

    REQUIRE(result == m2);
}

TEST_CASE("Matrix - Matrix Multiplication Non-Square", "[matrix][ops][arithmetic]") {
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

TEST_CASE("Matrix - Matrix Multiplication Chain Different Sizes", "[matrix][ops][arithmetic]") {
    Matrix<2, 3> A = {
        {1.0f, 2.0f, 3.0f},
        {4.0f, 5.0f, 6.0f}
    };
    Matrix<3, 4> B = {
        {1.0f, 0.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f, 0.0f},
        {0.0f, 0.0f, 1.0f, 0.0f}
    };
    Matrix<4, 2> C = {
        {1.0f, 0.0f},
        {0.0f, 1.0f},
        {0.0f, 0.0f},
        {0.0f, 0.0f}
    };

    Matrix<2, 2> result = A * B * C;

    REQUIRE_THAT(result(0, 0), Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(result(0, 1), Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(result(1, 0), Catch::Matchers::WithinAbs(4.0, 1e-6));
    REQUIRE_THAT(result(1, 1), Catch::Matchers::WithinAbs(5.0, 1e-6));
}

TEST_CASE("Matrix - Matrix Multiplication Not Commutative", "[matrix][ops][arithmetic]") {
    Matrix<2, 2> m1 = {
        {1.0f, 2.0f},
        {3.0f, 4.0f}
    };
    Matrix<2, 2> m2 = {
        {5.0f, 6.0f},
        {7.0f, 8.0f}
    };

    Matrix<2, 2> result1 = m1 * m2;
    Matrix<2, 2> result2 = m2 * m1;

    REQUIRE(result1 != result2);
}

TEST_CASE("Matrix - Matrix Multiplication Identity Left", "[matrix][ops][arithmetic]") {
    Matrix<3, 3> m = {
        {1.0f, 2.0f, 3.0f},
        {4.0f, 5.0f, 6.0f},
        {7.0f, 8.0f, 9.0f}
    };
    Matrix<3, 3> I = Matrix<3, 3>::eye();

    Matrix<3, 3> result = I * m;

    REQUIRE(result == m);
}

TEST_CASE("Matrix - Matrix Multiplication Identity Right", "[matrix][ops][arithmetic]") {
    Matrix<3, 3> m = {
        {1.0f, 2.0f, 3.0f},
        {4.0f, 5.0f, 6.0f},
        {7.0f, 8.0f, 9.0f}
    };
    Matrix<3, 3> I = Matrix<3, 3>::eye();

    Matrix<3, 3> result = m * I;

    REQUIRE(result == m);
}

TEST_CASE("Matrix - Matrix Multiplication By Zero", "[matrix][ops][arithmetic]") {
    Matrix<2, 2> m = {
        {1.0f, 2.0f},
        {3.0f, 4.0f}
    };
    Matrix<2, 2> zero = Matrix<2, 2>::zero();

    Matrix<2, 2> result = m * zero;

    REQUIRE(result == zero);
}

TEST_CASE("Matrix - Vector Multiplication", "[matrix][ops][arithmetic]") {
    Matrix<2, 2> A = {
        {1.0f, 2.0f},
        {3.0f, 4.0f}
    };
    Vector<2> v = {5.0f, 6.0f};

    Vector<2> result = A * v;

    // [1 2] * [5] = [17]
    // [3 4]   [6]   [39]
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(17.0, 1e-6));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(39.0, 1e-6));
}

TEST_CASE("Matrix - Vector Multiplication Identity", "[matrix][ops][arithmetic]") {
    Matrix<3, 3> I = Matrix<3, 3>::eye();
    Vector<3> v = {1.0f, 2.0f, 3.0f};

    Vector<3> result = I * v;

    REQUIRE(result == v);
}

TEST_CASE("Matrix - Vector Multiplication Zero Matrix", "[matrix][ops][arithmetic]") {
    Matrix<2, 2> zero = Matrix<2, 2>::zero();
    Vector<2> v = {5.0f, 6.0f};

    Vector<2> result = zero * v;

    REQUIRE(result == Vector<2>::zero());
}

TEST_CASE("Matrix - Vector Multiplication Zero Vector", "[matrix][ops][arithmetic]") {
    Matrix<2, 2> m = {
        {1.0f, 2.0f},
        {3.0f, 4.0f}
    };
    Vector<2> zero = Vector<2>::zero();

    Vector<2> result = m * zero;

    REQUIRE(result == zero);
}

TEST_CASE("Matrix - Vector Multiplication Non-Square", "[matrix][ops][arithmetic]") {
    Matrix<2, 3> m = {
        {1.0f, 2.0f, 3.0f},
        {4.0f, 5.0f, 6.0f}
    };
    Vector<3> v = {1.0f, 0.0f, 1.0f};

    Vector<2> result = m * v;

    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(4.0, 1e-6));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(10.0, 1e-6));
}

TEST_CASE("Matrix - Unary Negation", "[matrix][ops][arithmetic]") {
    Matrix<2, 2> m = {
        {1.0f, -2.0f},
        {3.0f, -4.0f}
    };

    Matrix<2, 2> result = -m;

    REQUIRE_THAT(result(0, 0), Catch::Matchers::WithinAbs(-1.0, 1e-6));
    REQUIRE_THAT(result(0, 1), Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(result(1, 0), Catch::Matchers::WithinAbs(-3.0, 1e-6));
    REQUIRE_THAT(result(1, 1), Catch::Matchers::WithinAbs(4.0, 1e-6));
}

TEST_CASE("Matrix - Double Negation", "[matrix][ops][arithmetic]") {
    Matrix<2, 2> m = {
        {1.0f, 2.0f},
        {3.0f, 4.0f}
    };

    Matrix<2, 2> result = -(-m);

    REQUIRE(result == m);
}

TEST_CASE("Matrix - Scalar Division", "[matrix][ops][arithmetic]") {
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

TEST_CASE("Matrix - Scalar Division By One", "[matrix][ops][arithmetic]") {
    Matrix<2, 2> m = {
        {1.0f, 2.0f},
        {3.0f, 4.0f}
    };

    Matrix<2, 2> result = m / 1.0f;

    REQUIRE(result == m);
}

// --------------------------------------------------------------------------------
// Comparison Operators
// --------------------------------------------------------------------------------

TEST_CASE("Matrix - Equality Operator Same Values", "[matrix][ops][comparison]") {
    Matrix<2, 2> m1 = {
        {1.0f, 2.0f},
        {3.0f, 4.0f}
    };
    Matrix<2, 2> m2 = {
        {1.0f, 2.0f},
        {3.0f, 4.0f}
    };

    REQUIRE(m1 == m2);
}

TEST_CASE("Matrix - Equality Operator Different Values", "[matrix][ops][comparison]") {
    Matrix<2, 2> m1 = {
        {1.0f, 2.0f},
        {3.0f, 4.0f}
    };
    Matrix<2, 2> m2 = {
        {1.0f, 2.0f},
        {3.0f, 5.0f}
    };

    REQUIRE_FALSE(m1 == m2);
}

TEST_CASE("Matrix - Equality Operator Within Epsilon", "[matrix][ops][comparison]") {
    Matrix<2, 2> m1 = {
        {1.0f, 2.0f},
        {3.0f, 4.0f}
    };
    Matrix<2, 2> m2 = {
        {1.0f + 1e-7f, 2.0f, },
        {3.0f, 4.0f}
    };

    REQUIRE(m1 == m2);
}

TEST_CASE("Matrix - Equality Reflexive", "[matrix][ops][comparison]") {
    Matrix<2, 2> m = {
        {1.0f, 2.0f},
        {3.0f, 4.0f}
    };

    REQUIRE(m == m);
}

TEST_CASE("Matrix - Equality Symmetric", "[matrix][ops][comparison]") {
    Matrix<2, 2> m1 = {
        {1.0f, 2.0f},
        {3.0f, 4.0f}
    };
    Matrix<2, 2> m2 = m1;

    REQUIRE(m1 == m2);
    REQUIRE(m2 == m1);
}

TEST_CASE("Matrix - Inequality Operator", "[matrix][ops][comparison]") {
    Matrix<2, 2> m1 = {
        {1.0f, 2.0f},
        {3.0f, 4.0f}
    };
    Matrix<2, 2> m2 = {
        {1.0f, 2.0f},
        {3.0f, 5.0f}
    };

    REQUIRE(m1 != m2);
}

TEST_CASE("Matrix - Inequality Operator Same Values", "[matrix][ops][comparison]") {
    Matrix<2, 2> m1 = {
        {1.0f, 2.0f},
        {3.0f, 4.0f}
    };
    Matrix<2, 2> m2 = m1;

    REQUIRE_FALSE(m1 != m2);
}

// --------------------------------------------------------------------------------
// Determinant Tests
// --------------------------------------------------------------------------------

TEST_CASE("Matrix - Determinant 1x1", "[matrix][ops][det]") {
    Matrix<1, 1> m = {{5.0f}};
    
    REQUIRE_THAT(det(m), Catch::Matchers::WithinAbs(5.0, 1e-6));
}

TEST_CASE("Matrix - Determinant 1x1 Negative", "[matrix][ops][det]") {
    Matrix<1, 1> m = {{-3.0f}};
    
    REQUIRE_THAT(det(m), Catch::Matchers::WithinAbs(-3.0, 1e-6));
}

TEST_CASE("Matrix - Determinant 2x2", "[matrix][ops][det]") {
    Matrix<2, 2> m = {
        {1.0f, 2.0f},
        {3.0f, 4.0f}
    };
    
    // det = 1*4 - 2*3 = -2
    REQUIRE_THAT(det(m), Catch::Matchers::WithinAbs(-2.0, 1e-6));
}

TEST_CASE("Matrix - Determinant 2x2 Identity", "[matrix][ops][det]") {
    Matrix<2, 2> I = Matrix<2, 2>::eye();
    
    REQUIRE_THAT(det(I), Catch::Matchers::WithinAbs(1.0, 1e-6));
}

TEST_CASE("Matrix - Determinant 2x2 Singular", "[matrix][ops][det]") {
    Matrix<2, 2> m = {
        {1.0f, 2.0f},
        {2.0f, 4.0f}
    };
    
    REQUIRE_THAT(det(m), Catch::Matchers::WithinAbs(0.0, 1e-5));
}

TEST_CASE("Matrix - Determinant 3x3", "[matrix][ops][det]") {
    Matrix<3, 3> m = {
        {6.0f, 1.0f, 1.0f},
        {4.0f, -2.0f, 5.0f},
        {2.0f, 8.0f, 7.0f}
    };
    
    REQUIRE_THAT(det(m), Catch::Matchers::WithinAbs(-306.0, 1e-5));
}

TEST_CASE("Matrix - Determinant 3x3 Identity", "[matrix][ops][det]") {
    Matrix<3, 3> I = Matrix<3, 3>::eye();
    
    REQUIRE_THAT(det(I), Catch::Matchers::WithinAbs(1.0, 1e-6));
}

TEST_CASE("Matrix - Determinant 3x3 Singular", "[matrix][ops][det]") {
    Matrix<3, 3> m = {
        {1.0f, 2.0f, 3.0f},
        {2.0f, 4.0f, 6.0f},
        {3.0f, 6.0f, 9.0f}
    };
    
    REQUIRE_THAT(det(m), Catch::Matchers::WithinAbs(0.0, 1e-5));
}

TEST_CASE("Matrix - Determinant 3x3 Diagonal", "[matrix][ops][det]") {
    Matrix<3, 3> m = {
        {2.0f, 0.0f, 0.0f},
        {0.0f, 3.0f, 0.0f},
        {0.0f, 0.0f, 4.0f}
    };
    
    // det = 2*3*4 = 24
    REQUIRE_THAT(det(m), Catch::Matchers::WithinAbs(24.0, 1e-6));
}

TEST_CASE("Matrix - Determinant 4x4", "[matrix][ops][det]") {
    Matrix<4, 4> m = {
        {1.0f, 2.0f, 3.0f, 4.0f},
        {2.0f, 1.0f, 4.0f, 3.0f},
        {3.0f, 4.0f, 1.0f, 2.0f},
        {4.0f, 3.0f, 2.0f, 1.0f}
    };
    
    REQUIRE_THAT(det(m), Catch::Matchers::WithinAbs(0.0, 1e-4));
}

TEST_CASE("Matrix - Determinant 4x4 Identity", "[matrix][ops][det]") {
    Matrix<4, 4> I = Matrix<4, 4>::eye();
    
    REQUIRE_THAT(det(I), Catch::Matchers::WithinAbs(1.0, 1e-6));
}

TEST_CASE("Matrix - Determinant Product Property", "[matrix][ops][det]") {
    Matrix<2, 2> A = {
        {2.0f, 3.0f},
        {1.0f, 4.0f}
    };
    Matrix<2, 2> B = {
        {1.0f, 2.0f},
        {3.0f, 4.0f}
    };
    
    float detA = det(A);
    float detB = det(B);
    float detAB = det(A * B);
    
    // det(AB) = det(A) * det(B)
    REQUIRE_THAT(detAB, Catch::Matchers::WithinAbs(detA * detB, 1e-5));
}

// --------------------------------------------------------------------------------
// Transpose Tests
// --------------------------------------------------------------------------------

TEST_CASE("Matrix - Transpose 2x2", "[matrix][ops][transpose]") {
    Matrix<2, 2> m = {
        {1.0f, 2.0f},
        {3.0f, 4.0f}
    };

    Matrix<2, 2> result = transpose(m);

    REQUIRE_THAT(result(0, 0), Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(result(0, 1), Catch::Matchers::WithinAbs(3.0, 1e-6));
    REQUIRE_THAT(result(1, 0), Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(result(1, 1), Catch::Matchers::WithinAbs(4.0, 1e-6));
}

TEST_CASE("Matrix - Transpose 3x3", "[matrix][ops][transpose]") {
    Matrix<3, 3> m = {
        {1.0f, 2.0f, 3.0f},
        {4.0f, 5.0f, 6.0f},
        {7.0f, 8.0f, 9.0f}
    };

    Matrix<3, 3> result = transpose(m);

    REQUIRE_THAT(result(0, 0), Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(result(1, 0), Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(result(2, 0), Catch::Matchers::WithinAbs(3.0, 1e-6));
    REQUIRE_THAT(result(0, 2), Catch::Matchers::WithinAbs(7.0, 1e-6));
}

TEST_CASE("Matrix - Transpose Non-Square 2x3", "[matrix][ops][transpose]") {
    Matrix<2, 3> m = {
        {1.0f, 2.0f, 3.0f},
        {4.0f, 5.0f, 6.0f}
    };

    Matrix<3, 2> result = transpose(m);

    REQUIRE_THAT(result(0, 0), Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(result(0, 1), Catch::Matchers::WithinAbs(4.0, 1e-6));
    REQUIRE_THAT(result(1, 0), Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(result(1, 1), Catch::Matchers::WithinAbs(5.0, 1e-6));
    REQUIRE_THAT(result(2, 0), Catch::Matchers::WithinAbs(3.0, 1e-6));
    REQUIRE_THAT(result(2, 1), Catch::Matchers::WithinAbs(6.0, 1e-6));
}

TEST_CASE("Matrix - Transpose 3x2", "[matrix][ops][transpose]") {
    Matrix<3, 2> m = {
        {1.0f, 2.0f},
        {3.0f, 4.0f},
        {5.0f, 6.0f}
    };

    Matrix<2, 3> result = transpose(m);

    REQUIRE(result.rows() == 2);
    REQUIRE(result.cols() == 3);
    REQUIRE_THAT(result(0, 2), Catch::Matchers::WithinAbs(5.0, 1e-6));
}

TEST_CASE("Matrix - Double Transpose Returns Original", "[matrix][ops][transpose]") {
    Matrix<3, 3> m = {
        {1.0f, 2.0f, 3.0f},
        {4.0f, 5.0f, 6.0f},
        {7.0f, 8.0f, 9.0f}
    };

    Matrix<3, 3> result = transpose(transpose(m));

    REQUIRE(result == m);
}

TEST_CASE("Matrix - Transpose Identity", "[matrix][ops][transpose]") {
    Matrix<3, 3> I = Matrix<3, 3>::eye();

    Matrix<3, 3> result = transpose(I);

    REQUIRE(result == I);
}

TEST_CASE("Matrix - Transpose Symmetric Matrix", "[matrix][ops][transpose]") {
    Matrix<3, 3> m = {
        {1.0f, 2.0f, 3.0f},
        {2.0f, 4.0f, 5.0f},
        {3.0f, 5.0f, 6.0f}
    };

    Matrix<3, 3> result = transpose(m);

    REQUIRE(result == m);
}

// --------------------------------------------------------------------------------
// Inverse Tests
// --------------------------------------------------------------------------------

TEST_CASE("Matrix - Inverse 1x1", "[matrix][ops][inverse]") {
    Matrix<1, 1> m = {{4.0f}};
    Matrix<1, 1> inv_m;

    REQUIRE(inv(m, inv_m));
    REQUIRE_THAT(inv_m(0, 0), Catch::Matchers::WithinAbs(0.25, 1e-6));
}

TEST_CASE("Matrix - Inverse 1x1 Singular", "[matrix][ops][inverse]") {
    Matrix<1, 1> m = {{0.0f}};
    Matrix<1, 1> inv_m;

    REQUIRE_FALSE(inv(m, inv_m));
}

TEST_CASE("Matrix - Inverse 2x2", "[matrix][ops][inverse]") {
    Matrix<2, 2> m = {
        {4.0f, 7.0f},
        {2.0f, 6.0f}
    };
    Matrix<2, 2> inv_m;

    REQUIRE(inv(m, inv_m));
    REQUIRE_THAT(inv_m(0, 0), Catch::Matchers::WithinAbs(0.6, 1e-6));
    REQUIRE_THAT(inv_m(0, 1), Catch::Matchers::WithinAbs(-0.7, 1e-6));
    REQUIRE_THAT(inv_m(1, 0), Catch::Matchers::WithinAbs(-0.2, 1e-6));
    REQUIRE_THAT(inv_m(1, 1), Catch::Matchers::WithinAbs(0.4, 1e-6));
}

TEST_CASE("Matrix - Inverse 2x2 Singular", "[matrix][ops][inverse]") {
    Matrix<2, 2> m = {
        {1.0f, 2.0f},
        {2.0f, 4.0f}
    };
    Matrix<2, 2> inv_m;

    REQUIRE_FALSE(inv(m, inv_m));
}

TEST_CASE("Matrix - Inverse 3x3", "[matrix][ops][inverse]") {
    Matrix<3, 3> m = {
        {1.0f, 2.0f, 3.0f},
        {0.0f, 1.0f, 4.0f},
        {5.0f, 6.0f, 0.0f}
    };
    Matrix<3, 3> inv_m;

    REQUIRE(inv(m, inv_m));

    // Verify m * inv_m = I
    Matrix<3, 3> identity_result = m * inv_m;
    Matrix<3, 3> identity_expected = Matrix<3, 3>::eye();

    for(uint8_t i = 0; i < 3; i++) {
        for(uint8_t j = 0; j < 3; j++) {
            REQUIRE_THAT(identity_result(i, j), 
                        Catch::Matchers::WithinAbs(identity_expected(i, j), 1e-5));
        }
    }
}

TEST_CASE("Matrix - Inverse Identity", "[matrix][ops][inverse]") {
    Matrix<3, 3> I = Matrix<3, 3>::eye();
    Matrix<3, 3> inv_I;

    REQUIRE(inv(I, inv_I));
    REQUIRE(inv_I == I);
}

TEST_CASE("Matrix - Inverse Then Multiply Returns Identity", "[matrix][ops][inverse]") {
    Matrix<2, 2> m = {
        {1.0f, 2.0f},
        {3.0f, 5.0f}
    };
    Matrix<2, 2> inv_m;

    REQUIRE(inv(m, inv_m));

    Matrix<2, 2> result = m * inv_m;
    Matrix<2, 2> I = Matrix<2, 2>::eye();

    for(uint8_t i = 0; i < 2; i++) {
        for(uint8_t j = 0; j < 2; j++) {
            REQUIRE_THAT(result(i, j), Catch::Matchers::WithinAbs(I(i, j), 1e-5));
        }
    }
}

TEST_CASE("Matrix - Inverse Double Inverse Returns Original", "[matrix][ops][inverse]") {
    Matrix<2, 2> m = {
        {1.0f, 2.0f},
        {3.0f, 5.0f}
    };
    Matrix<2, 2> inv_m, inv_inv_m;

    REQUIRE(inv(m, inv_m));
    REQUIRE(inv(inv_m, inv_inv_m));

    for(uint8_t i = 0; i < 2; i++) {
        for(uint8_t j = 0; j < 2; j++) {
            REQUIRE_THAT(inv_inv_m(i, j), Catch::Matchers::WithinAbs(m(i, j), 1e-4));
        }
    }
}

// --------------------------------------------------------------------------------
// Rank Tests
// --------------------------------------------------------------------------------

TEST_CASE("Matrix - Rank Full 2x2", "[matrix][ops][rank]") {
    Matrix<2, 2> m = {
        {1.0f, 0.0f},
        {0.0f, 1.0f}
    };

    REQUIRE(rank(m) == 2);
}

TEST_CASE("Matrix - Rank Deficient 2x2", "[matrix][ops][rank]") {
    Matrix<2, 2> m = {
        {1.0f, 2.0f},
        {2.0f, 4.0f}
    };

    REQUIRE(rank(m) == 1);
}

TEST_CASE("Matrix - Rank Full 3x3", "[matrix][ops][rank]") {
    Matrix<3, 3> I = Matrix<3, 3>::eye();

    REQUIRE(rank(I) == 3);
}

TEST_CASE("Matrix - Rank Zero Matrix", "[matrix][ops][rank]") {
    Matrix<3, 3> m{};

    REQUIRE(rank(m) == 0);
}

TEST_CASE("Matrix - Rank Deficient 3x3 Rank 2", "[matrix][ops][rank]") {
    Matrix<3, 3> m = {
        {1.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f},
        {1.0f, 1.0f, 0.0f}  // Row 3 = Row 1 + Row 2
    };

    REQUIRE(rank(m) == 2);
}

// --------------------------------------------------------------------------------
// Trace Tests
// --------------------------------------------------------------------------------

TEST_CASE("Matrix - Trace 2x2", "[matrix][ops][trace]") {
    Matrix<2, 2> m = {
        {1.0f, 2.0f},
        {3.0f, 4.0f}
    };

    REQUIRE_THAT(trace(m), Catch::Matchers::WithinAbs(5.0, 1e-6));
}

TEST_CASE("Matrix - Trace 3x3", "[matrix][ops][trace]") {
    Matrix<3, 3> m = {
        {1.0f, 2.0f, 3.0f},
        {4.0f, 5.0f, 6.0f},
        {7.0f, 8.0f, 9.0f}
    };

    REQUIRE_THAT(trace(m), Catch::Matchers::WithinAbs(15.0, 1e-6));
}

TEST_CASE("Matrix - Trace Identity", "[matrix][ops][trace]") {
    Matrix<4, 4> I = Matrix<4, 4>::eye();

    REQUIRE_THAT(trace(I), Catch::Matchers::WithinAbs(4.0, 1e-6));
}

TEST_CASE("Matrix - Trace Zero Matrix", "[matrix][ops][trace]") {
    Matrix<3, 3> zero = Matrix<3, 3>::zero();

    REQUIRE_THAT(trace(zero), Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Matrix - Trace Non-Square Uses Minimum Dimension", "[matrix][ops][trace]") {
    Matrix<2, 4> m = {
        {1.0f, 2.0f, 3.0f, 4.0f},
        {5.0f, 6.0f, 7.0f, 8.0f}
    };

    // Trace = m(0,0) + m(1,1) = 1 + 6 = 7
    REQUIRE_THAT(trace(m), Catch::Matchers::WithinAbs(7.0, 1e-6));
}

// --------------------------------------------------------------------------------
// Trace Product Tests
// --------------------------------------------------------------------------------

TEST_CASE("Matrix - Trace Product 2x2", "[matrix][ops][trace_product]") {
    Matrix<2, 2> m = {
        {2.0f, 3.0f},
        {4.0f, 5.0f}
    };

    REQUIRE_THAT(traceProduct(m), Catch::Matchers::WithinAbs(10.0, 1e-6));
}

TEST_CASE("Matrix - Trace Product 3x3", "[matrix][ops][trace_product]") {
    Matrix<3, 3> m = {
        {2.0f, 0.0f, 0.0f},
        {0.0f, 3.0f, 0.0f},
        {0.0f, 0.0f, 4.0f}
    };

    REQUIRE_THAT(traceProduct(m), Catch::Matchers::WithinAbs(24.0, 1e-6));
}

TEST_CASE("Matrix - Trace Product Identity", "[matrix][ops][trace_product]") {
    Matrix<3, 3> I = Matrix<3, 3>::eye();

    REQUIRE_THAT(traceProduct(I), Catch::Matchers::WithinAbs(1.0, 1e-6));
}

TEST_CASE("Matrix - Trace Product With Zero Diagonal", "[matrix][ops][trace_product]") {
    Matrix<3, 3> m = {
        {1.0f, 2.0f, 3.0f},
        {4.0f, 0.0f, 6.0f},
        {7.0f, 8.0f, 9.0f}
    };

    REQUIRE_THAT(traceProduct(m), Catch::Matchers::WithinAbs(0.0, 1e-6));
}

// --------------------------------------------------------------------------------
// Hadamard Product Tests
// --------------------------------------------------------------------------------

TEST_CASE("Matrix - Hadamard Product", "[matrix][ops][hadamard]") {
    Matrix<2, 2> m1 = {
        {1.0f, 2.0f},
        {3.0f, 4.0f}
    };
    Matrix<2, 2> m2 = {
        {5.0f, 6.0f},
        {7.0f, 8.0f}
    };

    Matrix<2, 2> result = hadamard(m1, m2);

    REQUIRE_THAT(result(0, 0), Catch::Matchers::WithinAbs(5.0, 1e-6));
    REQUIRE_THAT(result(0, 1), Catch::Matchers::WithinAbs(12.0, 1e-6));
    REQUIRE_THAT(result(1, 0), Catch::Matchers::WithinAbs(21.0, 1e-6));
    REQUIRE_THAT(result(1, 1), Catch::Matchers::WithinAbs(32.0, 1e-6));
}

TEST_CASE("Matrix - Hadamard Product With Identity", "[matrix][ops][hadamard]") {
    Matrix<2, 2> m = {
        {1.0f, 2.0f},
        {3.0f, 4.0f}
    };
    Matrix<2, 2> I = Matrix<2, 2>::eye();

    Matrix<2, 2> result = hadamard(m, I);

    REQUIRE_THAT(result(0, 0), Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(result(0, 1), Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(result(1, 0), Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(result(1, 1), Catch::Matchers::WithinAbs(4.0, 1e-6));
}

TEST_CASE("Matrix - Hadamard Product Commutative", "[matrix][ops][hadamard]") {
    Matrix<2, 2> m1 = {
        {1.0f, 2.0f},
        {3.0f, 4.0f}
    };
    Matrix<2, 2> m2 = {
        {5.0f, 6.0f},
        {7.0f, 8.0f}
    };

    Matrix<2, 2> result1 = hadamard(m1, m2);
    Matrix<2, 2> result2 = hadamard(m2, m1);

    REQUIRE(result1 == result2);
}

// --------------------------------------------------------------------------------
// Matrix Exponential and Logarithm Tests
// --------------------------------------------------------------------------------

TEST_CASE("Matrix - Logarithm of Identity", "[matrix][ops][log]") {
    Matrix<2, 2> I = Matrix<2, 2>::eye();
    Matrix<2, 2> result = log(I);

    // log(I) ≈ 0
    for(uint8_t i = 0; i < 2; i++) {
        for(uint8_t j = 0; j < 2; j++) {
            REQUIRE_THAT(result(i, j), Catch::Matchers::WithinAbs(0.0, 1e-3));
        }
    }
}

TEST_CASE("Matrix - Exponential of Zero Matrix", "[matrix][ops][exp]") {
    Matrix<2, 2> zero = Matrix<2, 2>::zero();
    Matrix<2, 2> result = exp(zero);

    // exp(0) = I
    Matrix<2, 2> I = Matrix<2, 2>::eye();

    for(uint8_t i = 0; i < 2; i++) {
        for(uint8_t j = 0; j < 2; j++) {
            REQUIRE_THAT(result(i, j), Catch::Matchers::WithinAbs(I(i, j), 1e-4));
        }
    }
}

TEST_CASE("Matrix - Exponential Logarithm Inverse", "[matrix][ops][exp]") {
    // For matrices close to identity, exp(log(A)) ≈ A
    Matrix<2, 2> m = {
        {1.1f, 0.1f},
        {0.1f, 1.1f}
    };

    Matrix<2, 2> logM = log(m);
    Matrix<2, 2> result = exp(logM);

    for(uint8_t i = 0; i < 2; i++) {
        for(uint8_t j = 0; j < 2; j++) {
            REQUIRE_THAT(result(i, j), Catch::Matchers::WithinAbs(m(i, j), 1e-2));
        }
    }
}

TEST_CASE("Matrix - Power Scalar Exponent", "[matrix][ops][pow]") {
    Matrix<2, 2> m = {
        {1.1f, 0.0f},
        {0.0f, 1.1f}
    };

    Matrix<2, 2> result = pow(m, 2.0f);

    Matrix<2, 2> expected = m * m;

    for(uint8_t i = 0; i < 2; i++) {
        for(uint8_t j = 0; j < 2; j++) {
            REQUIRE_THAT(result(i, j), Catch::Matchers::WithinAbs(expected(i, j), 1e-2));
        }
    }
}

TEST_CASE("Matrix - Power Integer Exponent Zero", "[matrix][ops][pow]") {
    Matrix<2, 2> m = {
        {2.0f, 3.0f},
        {4.0f, 5.0f}
    };

    Matrix<2, 2> result = powInt(m, 0);
    Matrix<2, 2> I = Matrix<2, 2>::eye();

    REQUIRE(result == I);
}

TEST_CASE("Matrix - Power Integer Exponent One", "[matrix][ops][pow]") {
    Matrix<2, 2> m = {
        {2.0f, 3.0f},
        {4.0f, 5.0f}
    };

    Matrix<2, 2> result = powInt(m, 1);

    REQUIRE(result == m);
}

TEST_CASE("Matrix - Power Integer Exponent Two", "[matrix][ops][pow]") {
    Matrix<2, 2> m = {
        {1.0f, 2.0f},
        {3.0f, 4.0f}
    };

    Matrix<2, 2> result = powInt(m, 2);
    Matrix<2, 2> expected = m * m;

    REQUIRE(result == expected);
}

TEST_CASE("Matrix - Power Integer Exponent Three", "[matrix][ops][pow]") {
    Matrix<2, 2> m = {
        {1.0f, 2.0f},
        {3.0f, 4.0f}
    };

    Matrix<2, 2> result = powInt(m, 3);
    Matrix<2, 2> expected = m * m * m;

    for(uint8_t i = 0; i < 2; i++) {
        for(uint8_t j = 0; j < 2; j++) {
            REQUIRE_THAT(result(i, j), Catch::Matchers::WithinAbs(expected(i, j), 1e-5));
        }
    }
}

// --------------------------------------------------------------------------------
// Matrix Norm Tests
// --------------------------------------------------------------------------------

TEST_CASE("Matrix - Frobenius Norm", "[matrix][ops][norm]") {
    Matrix<2, 2> m = {
        {1.0f, 2.0f},
        {3.0f, 4.0f}
    };

    // ||A||_F = sqrt(1 + 4 + 9 + 16) = sqrt(30)
    float expected = std::sqrt(1.0f + 4.0f + 9.0f + 16.0f);
    REQUIRE_THAT(normFrobenius(m), Catch::Matchers::WithinAbs(expected, 1e-6));
}

TEST_CASE("Matrix - Frobenius Norm Zero Matrix", "[matrix][ops][norm]") {
    Matrix<2, 2> zero = Matrix<2, 2>::zero();

    REQUIRE_THAT(normFrobenius(zero), Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Matrix - Frobenius Norm Identity", "[matrix][ops][norm]") {
    Matrix<3, 3> I = Matrix<3, 3>::eye();

    // ||I||_F = sqrt(3)
    REQUIRE_THAT(normFrobenius(I), Catch::Matchers::WithinAbs(std::sqrt(3.0f), 1e-6));
}

TEST_CASE("Matrix - Infinity Norm", "[matrix][ops][norm]") {
    Matrix<2, 2> m = {
        {1.0f, 2.0f},
        {3.0f, 4.0f}
    };

    // ||A||_∞ = max row sum = max(3, 7) = 7
    REQUIRE_THAT(normInf(m), Catch::Matchers::WithinAbs(7.0, 1e-6));
}

TEST_CASE("Matrix - Infinity Norm Identity", "[matrix][ops][norm]") {
    Matrix<3, 3> I = Matrix<3, 3>::eye();

    REQUIRE_THAT(normInf(I), Catch::Matchers::WithinAbs(1.0, 1e-6));
}

TEST_CASE("Matrix - 1-Norm", "[matrix][ops][norm]") {
    Matrix<2, 2> m = {
        {1.0f, 2.0f},
        {3.0f, 4.0f}
    };

    // ||A||_1 = max col sum = max(4, 6) = 6
    REQUIRE_THAT(norm1(m), Catch::Matchers::WithinAbs(6.0, 1e-6));
}

TEST_CASE("Matrix - 1-Norm Identity", "[matrix][ops][norm]") {
    Matrix<3, 3> I = Matrix<3, 3>::eye();

    REQUIRE_THAT(norm1(I), Catch::Matchers::WithinAbs(1.0, 1e-6));
}

TEST_CASE("Matrix - 2-Norm Simple", "[matrix][ops][norm]") {
    Matrix<2, 2> m = {
        {3.0f, 0.0f},
        {0.0f, 4.0f}
    };

    // For diagonal matrix, 2-norm is largest singular value
    float result = norm2(m);
    REQUIRE_THAT(result, Catch::Matchers::WithinAbs(4.0, 1e-4));
}

TEST_CASE("Matrix - Condition Number Identity", "[matrix][ops][norm]") {
    Matrix<2, 2> I = Matrix<2, 2>::eye();

    REQUIRE_THAT(conditionNum(I), Catch::Matchers::WithinAbs(1.0, 1e-4));
}

TEST_CASE("Matrix - Condition Number Ill-Conditioned", "[matrix][ops][norm]") {
    Matrix<2, 2> m = {
        {1.0f, 1.0f},
        {1.0f, 1.000001f}
    };

    // Should have large condition number
    float cond = conditionNum(m);
    REQUIRE(cond > 1.0f);
}

// --------------------------------------------------------------------------------
// Linear System Solver Tests
// --------------------------------------------------------------------------------

TEST_CASE("Matrix - Solve 2x2 System", "[matrix][ops][solve]") {
    Matrix<2, 2> A = {
        {2.0f, 1.0f},
        {1.0f, 3.0f}
    };
    Vector<2> b = {5.0f, 7.0f};
    Vector<2> x;

    REQUIRE(solve(A, b, x));
    REQUIRE_THAT(x[0], Catch::Matchers::WithinAbs(1.6, 1e-5));
    REQUIRE_THAT(x[1], Catch::Matchers::WithinAbs(1.8, 1e-5));
}

TEST_CASE("Matrix - Solve 2x2 Verify Solution", "[matrix][ops][solve]") {
    Matrix<2, 2> A = {
        {3.0f, 1.0f},
        {1.0f, 2.0f}
    };
    Vector<2> b = {9.0f, 8.0f};
    Vector<2> x;

    REQUIRE(solve(A, b, x));

    // Verify Ax = b
    Vector<2> result = A * x;
    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(b[0], 1e-5));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(b[1], 1e-5));
}

TEST_CASE("Matrix - Solve 3x3 System", "[matrix][ops][solve]") {
    Matrix<3, 3> A = {
        {3.0f, 2.0f, -1.0f},
        {2.0f, -2.0f, 4.0f},
        {-1.0f, 0.5f, -1.0f}
    };
    Vector<3> b = {1.0f, -2.0f, 0.0f};
    Vector<3> x;

    REQUIRE(solve(A, b, x));

    // Verify solution
    Vector<3> result = A * x;
    for(uint8_t i = 0; i < 3; i++) {
        REQUIRE_THAT(result[i], Catch::Matchers::WithinAbs(b[i], 1e-4));
    }
}

TEST_CASE("Matrix - Solve Singular System", "[matrix][ops][solve]") {
    Matrix<2, 2> A = {
        {1.0f, 2.0f},
        {2.0f, 4.0f}
    };
    Vector<2> b = {3.0f, 6.0f};
    Vector<2> x;

    REQUIRE_FALSE(solve(A, b, x));
}

TEST_CASE("Matrix - Solve Identity System", "[matrix][ops][solve]") {
    Matrix<3, 3> I = Matrix<3, 3>::eye();
    Vector<3> b = {1.0f, 2.0f, 3.0f};
    Vector<3> x;

    REQUIRE(solve(I, b, x));
    REQUIRE_THAT(x[0], Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(x[1], Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(x[2], Catch::Matchers::WithinAbs(3.0, 1e-6));
}

TEST_CASE("Matrix - Solve Diagonal System", "[matrix][ops][solve]") {
    Matrix<3, 3> A = {
        {2.0f, 0.0f, 0.0f},
        {0.0f, 3.0f, 0.0f},
        {0.0f, 0.0f, 4.0f}
    };
    Vector<3> b = {6.0f, 9.0f, 12.0f};
    Vector<3> x;

    REQUIRE(solve(A, b, x));
    REQUIRE_THAT(x[0], Catch::Matchers::WithinAbs(3.0, 1e-5));
    REQUIRE_THAT(x[1], Catch::Matchers::WithinAbs(3.0, 1e-5));
    REQUIRE_THAT(x[2], Catch::Matchers::WithinAbs(3.0, 1e-5));
}

TEST_CASE("Matrix - Solve Zero Right Hand Side", "[matrix][ops][solve]") {
    Matrix<2, 2> A = {
        {1.0f, 2.0f},
        {3.0f, 4.0f}
    };
    Vector<2> b = Vector<2>::zero();
    Vector<2> x;

    REQUIRE(solve(A, b, x));
    REQUIRE(x == Vector<2>::zero());
}