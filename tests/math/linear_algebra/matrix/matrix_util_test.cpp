#define _USE_MATH_DEFINES 
#include <cmath>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "cobalt/math/linear_algebra/matrix/matrix.hpp"
#include "cobalt/math/linear_algebra/matrix/matrix_ops.hpp"
#include "cobalt/math/linear_algebra/matrix/matrix_util.hpp"

using namespace cobalt::math::linear_algebra;

// ============================================================================
// Matrix Utility Tests (matrix_util.hpp)
// ============================================================================

TEST_CASE("Matrix - LU Decomposition", "[matrix][util]") {
    Matrix<3, 3> A = {
        {2.0f, 1.0f, 1.0f},
        {4.0f, 3.0f, 3.0f},
        {8.0f, 7.0f, 9.0f}
    };
    Matrix<3, 3> L, U, P;
    uint8_t swapCount;
    
    bool success = lu(A, L, U, P, swapCount);
    
    REQUIRE(success);
    
    // Reconstruct A with permutation
    Matrix<3, 3> LU = L*U;
    Matrix<3, 3> PA = P*A;
    
    // Check P*A ≈ L*U (check a few elements)
    REQUIRE(PA(0,0) == LU(0,0));
}

TEST_CASE("Matrix - LU Decomposition Identity", "[matrix][util]") {
    Matrix<3, 3> A = Matrix<3, 3>::eye();
    Matrix<3, 3> L, U, P;
    uint8_t swapCount;
    bool success = lu(A, L, U, P, swapCount);
    
    REQUIRE(success);
    REQUIRE(L == Matrix<3, 3>::eye());
    REQUIRE(U == Matrix<3, 3>::eye());
}

TEST_CASE("Matrix - LU Decomposition Singular", "[matrix][util]") {
    Matrix<3, 3> A = {
        {1.0f, 2.0f, 3.0f},
        {2.0f, 4.0f, 6.0f},  // Row 2 = 2 * Row 1
        {4.0f, 5.0f, 6.0f}
    };
    Matrix<3, 3> L, U, P;
    uint8_t swapCount;

    bool success = lu(A, L, U, P, swapCount);
    
    REQUIRE_FALSE(success);
}

TEST_CASE("Matrix - QR Decomposition", "[matrix][util]") {
    Matrix<3, 2> A = {
        {1.0f, 0.0f},
        {1.0f, 1.0f},
        {0.0f, 1.0f}
    };
    Matrix<3, 3> Q;
    Matrix<3, 2> R;
    
    bool success = qr(A, Q, R);
    
    REQUIRE(success);
    
    // Q should be orthonormal (Q^T * Q = I)
    Matrix<3, 3> QtQ = transpose(Q) * Q;
    for(uint8_t i = 0; i < 3; i++) {
        for(uint8_t j = 0; j < 3; j++) {
            if(i == j) {
                REQUIRE_THAT(QtQ(i, j), Catch::Matchers::WithinAbs(1.0, 1e-5));
            } else {
                REQUIRE_THAT(QtQ(i, j), Catch::Matchers::WithinAbs(0.0, 1e-5));
            }
        }
    }
}

TEST_CASE("Matrix - Gram-Schmidt Orthonormalization", "[matrix][util]") {
    Matrix<3, 2> A = {
        {1.0f, 1.0f},
        {0.0f, 1.0f},
        {0.0f, 0.0f}
    };
    Matrix<3, 2> Q;
    
    bool success = gramSchmidtReduced(A, Q);
    
    REQUIRE(success);
    
    // Check orthonormality of columns
    Vector<3> col0 = toVector(Q, 0);
    Vector<3> col1 = toVector(Q, 1);
    
    REQUIRE_THAT(norm(col0), Catch::Matchers::WithinAbs(1.0, 1e-5));
    REQUIRE_THAT(norm(col1), Catch::Matchers::WithinAbs(1.0, 1e-5));
    REQUIRE_THAT(dot(col0, col1), Catch::Matchers::WithinAbs(0.0, 1e-5));
}

TEST_CASE("Matrix - Gram-Schmidt Linearly Dependent", "[matrix][util]") {
    Matrix<3, 2> A = {
        {1.0f, 2.0f},
        {2.0f, 4.0f},  // Column 2 = 2 * Column 1
        {3.0f, 6.0f}
    };
    Matrix<3, 2> Q;
    
    bool success = gramSchmidtReduced(A, Q);
    
    REQUIRE_FALSE(success);
}

TEST_CASE("Matrix - SVD Decomposition", "[matrix][util]") {
    Matrix<3, 2> A = {
        {1.0f, 2.0f},
        {3.0f, 4.0f},
        {5.0f, 6.0f}
    };
    Matrix<3, 3> U;
    Matrix<3, 2> S;
    Matrix<2, 2> V;
    
    size_t iterations = svd(A, U, S, V);
    
    REQUIRE(iterations > 0);
    
    // S should be diagonal with non-negative values
    for(uint8_t i = 0; i < 2; i++) {
        REQUIRE(S(i, i) >= 0.0f);
        for(uint8_t j = 0; j < 2; j++) {
            if(i != j) {
                REQUIRE_THAT(S(i, j), Catch::Matchers::WithinAbs(0.0, 1e-5));
            }
        }
    }
}

TEST_CASE("Matrix - SVD Identity", "[matrix][util]") {
    Matrix<3, 3> A = Matrix<3, 3>::eye();
    Matrix<3, 3> U, S, V;
    
    svd(A, U, S, V);
    
    // For identity matrix, S should also be identity
    for(uint8_t i = 0; i < 3; i++) {
        REQUIRE_THAT(S(i, i), Catch::Matchers::WithinAbs(1.0, 1e-5));
    }
}

TEST_CASE("Matrix - Jacobi Eigenvalue", "[matrix][util]") {
    Matrix<3, 3> A = {
        {4.0f, -1.0f, 1.0f},
        {-1.0f, 3.0f, -2.0f},
        {1.0f, -2.0f, 3.0f}
    };
    Vector<3> eigenvalues;
    Matrix<3, 3> eigenvectors;
    
    size_t iterations = jacobi(A, eigenvalues, eigenvectors);
    
    REQUIRE(iterations > 0);
    
    // Eigenvalues should be real and in descending order
    REQUIRE(eigenvalues[0] >= eigenvalues[1]);
    REQUIRE(eigenvalues[1] >= eigenvalues[2]);
}

TEST_CASE("Matrix - To Vector Conversion", "[matrix][util]") {
    Matrix<3, 2> m = {
        {1.0f, 4.0f},
        {2.0f, 5.0f},
        {3.0f, 6.0f}
    };
    
    Vector<3> col0 = toVector(m, 0);
    Vector<3> col1 = toVector(m, 1);
    
    REQUIRE_THAT(col0[0], Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(col0[1], Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(col0[2], Catch::Matchers::WithinAbs(3.0, 1e-6));
    
    REQUIRE_THAT(col1[0], Catch::Matchers::WithinAbs(4.0, 1e-6));
    REQUIRE_THAT(col1[1], Catch::Matchers::WithinAbs(5.0, 1e-6));
    REQUIRE_THAT(col1[2], Catch::Matchers::WithinAbs(6.0, 1e-6));
}

TEST_CASE("Matrix - Is Singular True", "[matrix][util]") {
    Matrix<2, 2> m = {
        {1.0f, 2.0f},
        {2.0f, 4.0f}
    };
    
    REQUIRE(isSingular(m));
}

TEST_CASE("Matrix - Is Singular False", "[matrix][util]") {
    Matrix<2, 2> m = {
        {1.0f, 2.0f},
        {3.0f, 4.0f}
    };
    
    REQUIRE_FALSE(isSingular(m));
}

TEST_CASE("Matrix - Rank Full Rank", "[matrix][util]") {
    Matrix<3, 3> m = {
        {1.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f},
        {0.0f, 0.0f, 1.0f}
    };
    
    uint8_t r = rank(m);
    
    REQUIRE(r == 3);
}

TEST_CASE("Matrix - Rank Deficient", "[matrix][util]") {
    Matrix<3, 3> m = {
        {1.0f, 2.0f, 3.0f},
        {2.0f, 4.0f, 6.0f},  // Row 2 = 2 * Row 1
        {1.0f, 1.0f, 1.0f}
    };
    
    uint8_t r = rank(m);
    
    REQUIRE(r < 3);
}

// ============================================================================
// Integration Tests
// ============================================================================

TEST_CASE("Matrix - Solve Using Inverse", "[matrix][integration]") {
    Matrix<2, 2> A = {
        {4.0f, 7.0f},
        {2.0f, 6.0f}
    };
    Vector<2> b = {11.0f, 8.0f};
    
    Matrix<2, 2> Ainv;

    bool res = inv(A, Ainv);
    REQUIRE(res);

    Vector<2> x = Ainv * b;
    
    // Verify solution
    Vector<2> check = A * x;
    REQUIRE_THAT(check[0], Catch::Matchers::WithinAbs(b[0], 1e-5));
    REQUIRE_THAT(check[1], Catch::Matchers::WithinAbs(b[1], 1e-5));
}

TEST_CASE("Matrix - Chained Operations", "[matrix][integration]") {
    Matrix<2, 2> A = {
        {1.0f, 2.0f},
        {3.0f, 4.0f}
    };
    Matrix<2, 2> B = {
        {2.0f, 0.0f},
        {1.0f, 2.0f}
    };
    
    Matrix<2, 2> result = (A + B) * 2.0f;
    
    // (A+B)*2 = [[3,2],[4,6]]*2 = [[6,4],[8,12]]
    REQUIRE_THAT(result(0, 0), Catch::Matchers::WithinAbs(6.0, 1e-6));
    REQUIRE_THAT(result(0, 1), Catch::Matchers::WithinAbs(4.0, 1e-6));
    REQUIRE_THAT(result(1, 0), Catch::Matchers::WithinAbs(8.0, 1e-6));
    REQUIRE_THAT(result(1, 1), Catch::Matchers::WithinAbs(12.0, 1e-6));
}

TEST_CASE("Matrix - Rotation Matrix 2D", "[matrix][integration]") {
    float angle = M_PI / 4;  // 45 degrees
    Matrix<2, 2> R = {
        {std::cos(angle), -std::sin(angle)},
        {std::sin(angle), std::cos(angle)}
    };
    
    Vector<2> v = {1.0f, 0.0f};
    Vector<2> rotated = R * v;
    
    // Should rotate to (√2/2, √2/2)
    REQUIRE_THAT(rotated[0], Catch::Matchers::WithinAbs(std::sqrt(2.0f)/2, 1e-5));
    REQUIRE_THAT(rotated[1], Catch::Matchers::WithinAbs(std::sqrt(2.0f)/2, 1e-5));
}

TEST_CASE("Matrix - Projection Matrix", "[matrix][integration]") {
    // Projection onto xy-plane (zero out z)
    Matrix<3, 3> P = {
        {1.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f},
        {0.0f, 0.0f, 0.0f}
    };
    
    Vector<3> v = {1.0f, 2.0f, 3.0f};
    Vector<3> projected = P * v;
    
    REQUIRE_THAT(projected[0], Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(projected[1], Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(projected[2], Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Matrix - Determinant Properties", "[matrix][integration]") {
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

TEST_CASE("Matrix - Transpose Properties", "[matrix][integration]") {
    Matrix<2, 3> A = {
        {1.0f, 2.0f, 3.0f},
        {4.0f, 5.0f, 6.0f}
    };
    
    Matrix<3, 2> At = transpose(A);
    Matrix<2, 3> Att = transpose(At);
    
    // (A^T)^T = A
    REQUIRE(Att == A);
}

TEST_CASE("Matrix - Block Operations", "[matrix][integration]") {
    Matrix<4, 4> m = Matrix<4, 4>::eye();
    
    // Extract 2x2 block from top-left
    Matrix<2, 2> block = m.block<2, 2>(0, 0);
    
    REQUIRE(block == Matrix<2, 2>::eye());
}

TEST_CASE("Matrix - Different Types", "[matrix][integration]") {
    Matrix<2, 2, double> md = {
        {1.0, 2.0},
        {3.0, 4.0}
    };
    Matrix<2, 2, int> mi = {
        {1, 2},
        {3, 4}
    };
    
    REQUIRE(md(0, 0) == 1.0);
    REQUIRE(mi(0, 0) == 1);
}

TEST_CASE("Matrix - Least Squares via Pseudoinverse", "[matrix][integration]") {
    // Overdetermined system: 3 equations, 2 unknowns
    Matrix<3, 2> A = {
        {1.0f, 1.0f},
        {1.0f, 2.0f},
        {1.0f, 3.0f}
    };
    Matrix<2, 3> Apinv;
    
    bool success = pseudoL(A, Apinv);
    
    REQUIRE(success);
    
    // Pseudoinverse should satisfy A * A+ * A ≈ A (within numerical tolerance)
    Matrix<3, 2> reconstructed = A * Apinv * A;
    
    for(uint8_t i = 0; i < 3; i++) {
        for(uint8_t j = 0; j < 2; j++) {
            REQUIRE_THAT(reconstructed(i, j), Catch::Matchers::WithinAbs(A(i, j), 1e-4));
        }
    }
}