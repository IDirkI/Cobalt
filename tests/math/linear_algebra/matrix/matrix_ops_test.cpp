#include "cobalt/math/linear_algebra/matrix/matrix.hpp"
#include "cobalt/math/linear_algebra/matrix/matrix_ops.hpp"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

using namespace cobalt::math::linear_algebra;

// ================================================================================
// Matrix Operations Tests (matrix_ops.hpp)
// ================================================================================

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

TEST_CASE("Matrix - Scalar Multiplication (Right)", "[matrix][ops]") {
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

TEST_CASE("Matrix - Scalar Multiplication (Left)", "[matrix][ops]") {
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
    Matrix<2, 2> m1 = {
        {1.0f, 2.0f},
        {3.0f, 4.0f}
    };
    Matrix<2, 2> m2 = {
        {2.0f, 0.0f},
        {1.0f, 2.0f}
    };

    Matrix<2, 2> result = m1 * m2;

    REQUIRE_THAT(result(0, 0), Catch::Matchers::WithinAbs(4.0, 1e-6));
    REQUIRE_THAT(result(0, 1), Catch::Matchers::WithinAbs(4.0, 1e-6));
    REQUIRE_THAT(result(1, 0), Catch::Matchers::WithinAbs(10.0, 1e-6));
    REQUIRE_THAT(result(1, 1), Catch::Matchers::WithinAbs(8.0, 1e-6));
}

TEST_CASE("Matrix - Non-Square Multiplication", "[matrix][ops]") {
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

    REQUIRE_THAT(result(0, 0), Catch::Matchers::WithinAbs(58.0, 1e-6));
    REQUIRE_THAT(result(0, 1), Catch::Matchers::WithinAbs(64.0, 1e-6));
    REQUIRE_THAT(result(1, 0), Catch::Matchers::WithinAbs(139.0, 1e-6));
    REQUIRE_THAT(result(1, 1), Catch::Matchers::WithinAbs(154.0, 1e-6));
}

TEST_CASE("Matrix - Vector Multiplication", "[matrix][ops]") {
    Matrix<2, 2> A = {
        {1.0f, 2.0f},
        {3.0f, 4.0f}
    };
    Vector<2> v = {5.0f, 6.0f};

    Vector<2> result = A * v;

    REQUIRE_THAT(result[0], Catch::Matchers::WithinAbs(17.0, 1e-6));
    REQUIRE_THAT(result[1], Catch::Matchers::WithinAbs(39.0, 1e-6));
}

TEST_CASE("Matrix - Unary Negation", "[matrix][ops]") {
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
        {3.0f, 5.0f}
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
        {3.0f, 5.0f}
    };

    REQUIRE(m1 != m2);
}

// ================================================================================
// Determinant Tests
// ================================================================================

TEST_CASE("Matrix - Determinant 1x1", "[matrix][ops][det]") {
    Matrix<1, 1> m = {{5.0f}};
    
    REQUIRE_THAT(det(m), Catch::Matchers::WithinAbs(5.0, 1e-6));
}

TEST_CASE("Matrix - Determinant 2x2", "[matrix][ops][det]") {
    Matrix<2, 2> m = {
        {1.0f, 2.0f},
        {3.0f, 4.0f}
    };
    
    REQUIRE_THAT(det(m), Catch::Matchers::WithinAbs(-2.0, 1e-6));
}

TEST_CASE("Matrix - Determinant 2x2 Identity", "[matrix][ops][det]") {
    Matrix<2, 2> I = Matrix<2, 2>::eye();
    
    REQUIRE_THAT(det(I), Catch::Matchers::WithinAbs(1.0, 1e-6));
}

TEST_CASE("Matrix - Determinant 3x3", "[matrix][ops][det]") {
    Matrix<3, 3> m = {
        {6.0f, 1.0f, 1.0f},
        {4.0f, -2.0f, 5.0f},
        {2.0f, 8.0f, 7.0f}
    };
    
    REQUIRE_THAT(det(m), Catch::Matchers::WithinAbs(-306.0, 1e-5));
}

TEST_CASE("Matrix - Determinant 3x3 Singular", "[matrix][ops][det]") {
    Matrix<3, 3> m = {
        {1.0f, 2.0f, 3.0f},
        {2.0f, 4.0f, 6.0f},
        {3.0f, 6.0f, 9.0f}
    };
    
    REQUIRE_THAT(det(m), Catch::Matchers::WithinAbs(0.0, 1e-5));
}

TEST_CASE("Matrix - Determinant 4x4", "[matrix][ops][det]") {
    Matrix<4, 4> m = {
        {1.0f, 2.0f, 3.0f, 4.0f},
        {2.0f, 1.0f, 4.0f, 3.0f},
        {3.0f, 4.0f, 1.0f, 2.0f},
        {4.0f, 3.0f, 2.0f, 1.0f}
    };
    
    REQUIRE_THAT(det(m), Catch::Matchers::WithinAbs(160.0, 1e-4));
}

// ================================================================================
// Transpose Tests
// ================================================================================

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

TEST_CASE("Matrix - Double Transpose Returns Original", "[matrix][ops][transpose]") {
    Matrix<3, 3> m = {
        {1.0f, 2.0f, 3.0f},
        {4.0f, 5.0f, 6.0f},
        {7.0f, 8.0f, 9.0f}
    };

    Matrix<3, 3> result = transpose(transpose(m));

    REQUIRE(result == m);
}

// ================================================================================
// Inverse Tests
// ================================================================================

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

// ================================================================================
// Rank Tests
// ================================================================================

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

// ================================================================================
// Trace Tests
// ================================================================================

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

// ================================================================================
// Trace Product Tests
// ================================================================================

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

// ================================================================================
// Matrix Power Tests
// ================================================================================

TEST_CASE("Matrix - Power to 0", "[matrix][ops][pow]") {
    Matrix<2, 2> m = {
        {2.0f, 3.0f},
        {4.0f, 5.0f}
    };

    Matrix<2, 2> result = pow(m, 0);
    Matrix<2, 2> identity = Matrix<2, 2>::eye();

    REQUIRE(result == identity);
}

TEST_CASE("Matrix - Power to 1", "[matrix][ops][pow]") {
    Matrix<2, 2> m = {
        {2.0f, 3.0f},
        {4.0f, 5.0f}
    };

    Matrix<2, 2> result = pow(m, 1);

    REQUIRE(result == m);
}

TEST_CASE("Matrix - Power to 2", "[matrix][ops][pow]") {
    Matrix<2, 2> m = {
        {1.0f, 2.0f},
        {3.0f, 4.0f}
    };

    Matrix<2, 2> result = pow(m, 2);
    Matrix<2, 2> expected = m * m;

    REQUIRE(result == expected);
}

// ================================================================================
// Matrix Norm Tests
// ================================================================================

TEST_CASE("Matrix - Frobenius Norm", "[matrix][ops][norm]") {
    Matrix<2, 2> m = {
        {1.0f, 2.0f},
        {3.0f, 4.0f}
    };

    float expected = std::sqrt(1.0f + 4.0f + 9.0f + 16.0f);
    REQUIRE_THAT(normFrobenius(m), Catch::Matchers::WithinAbs(expected, 1e-6));
}

TEST_CASE("Matrix - Infinity Norm", "[matrix][ops][norm]") {
    Matrix<2, 2> m = {
        {1.0f, 2.0f},
        {3.0f, 4.0f}
    };

    REQUIRE_THAT(normInf(m), Catch::Matchers::WithinAbs(7.0, 1e-6));
}

TEST_CASE("Matrix - 1-Norm", "[matrix][ops][norm]") {
    Matrix<2, 2> m = {
        {1.0f, 2.0f},
        {3.0f, 4.0f}
    };

    REQUIRE_THAT(norm1(m), Catch::Matchers::WithinAbs(6.0, 1e-6));
}

// ================================================================================
// Linear System Solver Tests
// ================================================================================

TEST_CASE("Matrix - Solve 2x2 System", "[matrix][ops][solve]") {
    Matrix<2, 2> A = {
        {2.0f, 1.0f},
        {1.0f, 3.0f}
    };
    Vector<2> b = {5.0f, 7.0f};
    Vector<2> x;

    REQUIRE(solve(A, b, x));
    REQUIRE_THAT(x[0], Catch::Matchers::WithinAbs(1.0, 1e-5));
    REQUIRE_THAT(x[1], Catch::Matchers::WithinAbs(2.0, 1e-5));
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

    // Verify A*x = b
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