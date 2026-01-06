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

// ----------------------------------------------------------------------------
// Clamping Tests
// ----------------------------------------------------------------------------

TEST_CASE("Matrix - Clamp Within Range", "[matrix][util][clamp]") {
    Matrix<2, 2> m = {
        {1.0f, 2.0f},
        {3.0f, 4.0f}
    };

    Matrix<2, 2> result = clamp(m, 0.0f, 5.0f);

    REQUIRE(result == m);
}

TEST_CASE("Matrix - Clamp Above Max", "[matrix][util][clamp]") {
    Matrix<2, 2> m = {
        {1.0f, 7.0f},
        {3.0f, 9.0f}
    };

    Matrix<2, 2> result = clamp(m, 0.0f, 5.0f);

    REQUIRE_THAT(result(0, 0), Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(result(0, 1), Catch::Matchers::WithinAbs(5.0, 1e-6));
    REQUIRE_THAT(result(1, 0), Catch::Matchers::WithinAbs(3.0, 1e-6));
    REQUIRE_THAT(result(1, 1), Catch::Matchers::WithinAbs(5.0, 1e-6));
}

TEST_CASE("Matrix - Clamp Below Min", "[matrix][util][clamp]") {
    Matrix<2, 2> m = {
        {-5.0f, 2.0f},
        {-1.0f, 4.0f}
    };

    Matrix<2, 2> result = clamp(m, 0.0f, 5.0f);

    REQUIRE_THAT(result(0, 0), Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(result(0, 1), Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(result(1, 0), Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(result(1, 1), Catch::Matchers::WithinAbs(4.0, 1e-6));
}

TEST_CASE("Matrix - Clamp Mixed", "[matrix][util][clamp]") {
    Matrix<2, 2> m = {
        {-2.0f, 3.0f},
        {8.0f, 1.0f}
    };

    Matrix<2, 2> result = clamp(m, 0.0f, 5.0f);

    REQUIRE_THAT(result(0, 0), Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(result(0, 1), Catch::Matchers::WithinAbs(3.0, 1e-6));
    REQUIRE_THAT(result(1, 0), Catch::Matchers::WithinAbs(5.0, 1e-6));
    REQUIRE_THAT(result(1, 1), Catch::Matchers::WithinAbs(1.0, 1e-6));
}

TEST_CASE("Matrix - Clamp Symmetric Absolute", "[matrix][util][clamp]") {
    Matrix<2, 2> m = {
        {-7.0f, 2.0f},
        {8.0f, -1.0f}
    };

    Matrix<2, 2> result = clamp(m, 5.0f);

    REQUIRE_THAT(result(0, 0), Catch::Matchers::WithinAbs(-5.0, 1e-6));
    REQUIRE_THAT(result(0, 1), Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(result(1, 0), Catch::Matchers::WithinAbs(5.0, 1e-6));
    REQUIRE_THAT(result(1, 1), Catch::Matchers::WithinAbs(-1.0, 1e-6));
}

TEST_CASE("Matrix - Clamp All Zeros", "[matrix][util][clamp]") {
    Matrix<2, 2> zero = Matrix<2, 2>::zero();

    Matrix<2, 2> result = clamp(zero, -5.0f, 5.0f);

    REQUIRE(result == zero);
}

// ----------------------------------------------------------------------------
// Element-wise Operations Tests
// ----------------------------------------------------------------------------

TEST_CASE("Matrix - Min Element", "[matrix][util][element]") {
    Matrix<2, 2> m = {
        {1.0f, -2.0f},
        {3.0f, -4.0f}
    };

    REQUIRE_THAT(min(m), Catch::Matchers::WithinAbs(-4.0, 1e-6));
}

TEST_CASE("Matrix - Min Element All Positive", "[matrix][util][element]") {
    Matrix<2, 2> m = {
        {5.0f, 2.0f},
        {3.0f, 4.0f}
    };

    REQUIRE_THAT(min(m), Catch::Matchers::WithinAbs(2.0, 1e-6));
}

TEST_CASE("Matrix - Max Element", "[matrix][util][element]") {
    Matrix<2, 2> m = {
        {1.0f, 5.0f},
        {3.0f, 4.0f}
    };

    REQUIRE_THAT(max(m), Catch::Matchers::WithinAbs(5.0, 1e-6));
}

TEST_CASE("Matrix - Max Element All Negative", "[matrix][util][element]") {
    Matrix<2, 2> m = {
        {-5.0f, -2.0f},
        {-3.0f, -4.0f}
    };

    REQUIRE_THAT(max(m), Catch::Matchers::WithinAbs(-2.0, 1e-6));
}

TEST_CASE("Matrix - Abs Element-wise", "[matrix][util][element]") {
    Matrix<2, 2> m = {
        {-1.0f, 2.0f},
        {-3.0f, 4.0f}
    };

    Matrix<2, 2> result = abs(m);

    REQUIRE_THAT(result(0, 0), Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(result(0, 1), Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(result(1, 0), Catch::Matchers::WithinAbs(3.0, 1e-6));
    REQUIRE_THAT(result(1, 1), Catch::Matchers::WithinAbs(4.0, 1e-6));
}

TEST_CASE("Matrix - Abs All Positive", "[matrix][util][element]") {
    Matrix<2, 2> m = {
        {1.0f, 2.0f},
        {3.0f, 4.0f}
    };

    Matrix<2, 2> result = abs(m);

    REQUIRE(result == m);
}

TEST_CASE("Matrix - Sign Element-wise", "[matrix][util][element]") {
    Matrix<2, 2> m = {
        {-5.0f, 3.0f},
        {0.0f, -2.0f}
    };

    Matrix<2, 2> result = sign(m);

    REQUIRE_THAT(result(0, 0), Catch::Matchers::WithinAbs(-1.0, 1e-6));
    REQUIRE_THAT(result(0, 1), Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(result(1, 0), Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(result(1, 1), Catch::Matchers::WithinAbs(-1.0, 1e-6));
}

TEST_CASE("Matrix - Floor Element-wise", "[matrix][util][element]") {
    Matrix<2, 2> m = {
        {1.7f, 2.3f},
        {-1.2f, -2.9f}
    };

    Matrix<2, 2> result = floor(m);

    REQUIRE_THAT(result(0, 0), Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(result(0, 1), Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(result(1, 0), Catch::Matchers::WithinAbs(-2.0, 1e-6));
    REQUIRE_THAT(result(1, 1), Catch::Matchers::WithinAbs(-3.0, 1e-6));
}

TEST_CASE("Matrix - Ceil Element-wise", "[matrix][util][element]") {
    Matrix<2, 2> m = {
        {1.1f, 2.9f},
        {-1.2f, -2.9f}
    };

    Matrix<2, 2> result = ceil(m);

    REQUIRE_THAT(result(0, 0), Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(result(0, 1), Catch::Matchers::WithinAbs(3.0, 1e-6));
    REQUIRE_THAT(result(1, 0), Catch::Matchers::WithinAbs(-1.0, 1e-6));
    REQUIRE_THAT(result(1, 1), Catch::Matchers::WithinAbs(-2.0, 1e-6));
}

TEST_CASE("Matrix - Round Element-wise", "[matrix][util][element]") {
    Matrix<2, 2> m = {
        {1.4f, 2.6f},
        {-1.4f, -2.6f}
    };

    Matrix<2, 2> result = round(m);

    REQUIRE_THAT(result(0, 0), Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(result(0, 1), Catch::Matchers::WithinAbs(3.0, 1e-6));
    REQUIRE_THAT(result(1, 0), Catch::Matchers::WithinAbs(-1.0, 1e-6));
    REQUIRE_THAT(result(1, 1), Catch::Matchers::WithinAbs(-3.0, 1e-6));
}

TEST_CASE("Matrix - Clean Zero", "[matrix][util][element]") {
    Matrix<2, 2> m = {
        {1e-10f, 2.0f},
        {3.0f, -1e-10f}
    };

    Matrix<2, 2> result = cleanZero(m);

    REQUIRE_THAT(result(0, 0), Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(result(0, 1), Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(result(1, 0), Catch::Matchers::WithinAbs(3.0, 1e-6));
    REQUIRE_THAT(result(1, 1), Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Matrix - Clean Zero Preserves Significant Values", "[matrix][util][element]") {
    Matrix<2, 2> m = {
        {0.001f, 2.0f},
        {3.0f, 0.0001f}
    };

    Matrix<2, 2> result = cleanZero(m);

    REQUIRE_THAT(result(0, 0), Catch::Matchers::WithinAbs(0.001, 1e-6));
    REQUIRE_THAT(result(1, 1), Catch::Matchers::WithinAbs(0.0001, 1e-6));
}

// ----------------------------------------------------------------------------
// Pseudo-Inverse Tests
// ----------------------------------------------------------------------------

TEST_CASE("Matrix - Left Pseudo-Inverse Tall Matrix", "[matrix][util][pseudoinv]") {
    Matrix<3, 2> A = {
        {1.0f, 2.0f},
        {3.0f, 4.0f},
        {5.0f, 6.0f}
    };
    Matrix<2, 3> Apinv;

    REQUIRE(pseudoL(A, Apinv));

    // Verify (A^T * A)^-1 * A^T * A = I
    Matrix<2, 2> result = Apinv * A;
    Matrix<2, 2> I = Matrix<2, 2>::eye();

    for(uint8_t i = 0; i < 2; i++) {
        for(uint8_t j = 0; j < 2; j++) {
            REQUIRE_THAT(result(i, j), Catch::Matchers::WithinAbs(I(i, j), 1e-4));
        }
    }
}

TEST_CASE("Matrix - Left Pseudo-Inverse Full Rank", "[matrix][util][pseudoinv]") {
    Matrix<3, 2> A = {
        {1.0f, 0.0f},
        {0.0f, 1.0f},
        {0.0f, 0.0f}
    };
    Matrix<2, 3> Apinv;

    REQUIRE(pseudoL(A, Apinv));
}

TEST_CASE("Matrix - Left Pseudo-Inverse Rank Deficient", "[matrix][util][pseudoinv]") {
    Matrix<3, 2> A = {
        {1.0f, 2.0f},
        {2.0f, 4.0f},
        {3.0f, 6.0f}
    };
    Matrix<2, 3> Apinv;

    REQUIRE_FALSE(pseudoL(A, Apinv));
}

TEST_CASE("Matrix - Right Pseudo-Inverse Wide Matrix", "[matrix][util][pseudoinv]") {
    Matrix<2, 3> A = {
        {1.0f, 2.0f, 3.0f},
        {4.0f, 5.0f, 6.0f}
    };
    Matrix<3, 2> Apinv;

    REQUIRE(pseudoR(A, Apinv));

    // Verify A * (A * A^T)^-1 * A^T = I
    Matrix<2, 2> result = A * Apinv;
    Matrix<2, 2> I = Matrix<2, 2>::eye();

    for(uint8_t i = 0; i < 2; i++) {
        for(uint8_t j = 0; j < 2; j++) {
            REQUIRE_THAT(result(i, j), Catch::Matchers::WithinAbs(I(i, j), 1e-4));
        }
    }
}

TEST_CASE("Matrix - Right Pseudo-Inverse Full Rank", "[matrix][util][pseudoinv]") {
    Matrix<2, 3> A = {
        {1.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f}
    };
    Matrix<3, 2> Apinv;

    REQUIRE(pseudoR(A, Apinv));
}

TEST_CASE("Matrix - Right Pseudo-Inverse Rank Deficient", "[matrix][util][pseudoinv]") {
    Matrix<2, 3> A = {
        {1.0f, 2.0f, 3.0f},
        {2.0f, 4.0f, 6.0f}
    };
    Matrix<3, 2> Apinv;

    REQUIRE_FALSE(pseudoR(A, Apinv));
}

// ----------------------------------------------------------------------------
// Eigenvalue & Eigenvector Tests (Jacobi Method)
// ----------------------------------------------------------------------------

TEST_CASE("Matrix - Jacobi Eigenvalue Identity", "[matrix][util][eigen]") {
    Matrix<3, 3> A = Matrix<3, 3>::eye();
    Vector<3> eigenvalues;
    Matrix<3, 3> eigenvectors;

    size_t iterations = jacobi(A, eigenvalues, eigenvectors);

    REQUIRE(iterations > 0);

    // All eigenvalues should be 1
    for(uint8_t i = 0; i < 3; i++) {
        REQUIRE_THAT(eigenvalues[i], Catch::Matchers::WithinAbs(1.0, 1e-5));
    }
}

TEST_CASE("Matrix - Jacobi Eigenvalue Diagonal", "[matrix][util][eigen]") {
    Matrix<3, 3> A = {
        {4.0f, 0.0f, 0.0f},
        {0.0f, 2.0f, 0.0f},
        {0.0f, 0.0f, 3.0f}
    };
    Vector<3> eigenvalues;
    Matrix<3, 3> eigenvectors;

    jacobi(A, eigenvalues, eigenvectors);

    // Eigenvalues should be diagonal elements in descending order
    REQUIRE_THAT(eigenvalues[0], Catch::Matchers::WithinAbs(4.0, 1e-5));
    REQUIRE_THAT(eigenvalues[1], Catch::Matchers::WithinAbs(3.0, 1e-5));
    REQUIRE_THAT(eigenvalues[2], Catch::Matchers::WithinAbs(2.0, 1e-5));
}

TEST_CASE("Matrix - Jacobi Eigenvalue Symmetric", "[matrix][util][eigen]") {
    Matrix<3, 3> A = {
        {4.0f, -1.0f, 1.0f},
        {-1.0f, 3.0f, -2.0f},
        {1.0f, -2.0f, 3.0f}
    };
    Vector<3> eigenvalues;
    Matrix<3, 3> eigenvectors;

    size_t iterations = jacobi(A, eigenvalues, eigenvectors);

    REQUIRE(iterations > 0);

    // Eigenvalues should be in descending order
    REQUIRE(eigenvalues[0] >= eigenvalues[1]);
    REQUIRE(eigenvalues[1] >= eigenvalues[2]);
}

TEST_CASE("Matrix - Jacobi Eigenvectors Orthogonal", "[matrix][util][eigen]") {
    Matrix<3, 3> A = {
        {2.0f, 1.0f, 0.0f},
        {1.0f, 2.0f, 0.0f},
        {0.0f, 0.0f, 3.0f}
    };
    Vector<3> eigenvalues;
    Matrix<3, 3> eigenvectors;

    jacobi(A, eigenvalues, eigenvectors);

    // Eigenvectors should be orthonormal
    Matrix<3, 3> VtV = transpose(eigenvectors) * eigenvectors;
    Matrix<3, 3> I = Matrix<3, 3>::eye();

    for(uint8_t i = 0; i < 3; i++) {
        for(uint8_t j = 0; j < 3; j++) {
            REQUIRE_THAT(VtV(i, j), Catch::Matchers::WithinAbs(I(i, j), 1e-4));
        }
    }
}

// ----------------------------------------------------------------------------
// LU Decomposition Tests
// ----------------------------------------------------------------------------

TEST_CASE("Matrix - LU Decomposition", "[matrix][util][lu]") {
    Matrix<3, 3> A = {
        {2.0f, 1.0f, 1.0f},
        {4.0f, 3.0f, 3.0f},
        {8.0f, 7.0f, 9.0f}
    };
    Matrix<3, 3> L, U, P;
    size_t swapCount;

    bool success = lu(A, L, U, P, swapCount);

    REQUIRE(success);

    // Verify P*A = L*U
    Matrix<3, 3> LU = L * U;
    Matrix<3, 3> PA = P * A;

    for(uint8_t i = 0; i < 3; i++) {
        for(uint8_t j = 0; j < 3; j++) {
            REQUIRE_THAT(PA(i, j), Catch::Matchers::WithinAbs(LU(i, j), 1e-5));
        }
    }
}

TEST_CASE("Matrix - LU Decomposition Identity", "[matrix][util][lu]") {
    Matrix<3, 3> A = Matrix<3, 3>::eye();
    Matrix<3, 3> L, U, P;
    size_t swapCount;

    bool success = lu(A, L, U, P, swapCount);

    REQUIRE(success);
    REQUIRE(L == Matrix<3, 3>::eye());
    REQUIRE(U == Matrix<3, 3>::eye());
}

TEST_CASE("Matrix - LU Decomposition Singular", "[matrix][util][lu]") {
    Matrix<3, 3> A = {
        {1.0f, 2.0f, 3.0f},
        {2.0f, 4.0f, 6.0f},
        {4.0f, 5.0f, 6.0f}
    };
    Matrix<3, 3> L, U, P;
    size_t swapCount;

    bool success = lu(A, L, U, P, swapCount);

    REQUIRE_FALSE(success);
}

TEST_CASE("Matrix - LU Decomposition Lower Triangular", "[matrix][util][lu]") {
    Matrix<3, 3> A = {
        {2.0f, 1.0f, 1.0f},
        {4.0f, 3.0f, 3.0f},
        {8.0f, 7.0f, 9.0f}
    };
    Matrix<3, 3> L, U, P;
    size_t swapCount;

    bool success = lu(A, L, U, P, swapCount);
    REQUIRE(success);

    // L should have ones on diagonal
    for(uint8_t i = 0; i < 3; i++) {
        REQUIRE_THAT(L(i, i), Catch::Matchers::WithinAbs(1.0, 1e-6));
    }

    // L should be lower triangular (zeros above diagonal)
    for(uint8_t i = 0; i < 3; i++) {
        for(uint8_t j = i + 1; j < 3; j++) {
            REQUIRE_THAT(L(i, j), Catch::Matchers::WithinAbs(0.0, 1e-6));
        }
    }
}

TEST_CASE("Matrix - LU Decomposition Upper Triangular", "[matrix][util][lu]") {
    Matrix<3, 3> A = {
        {2.0f, 1.0f, 1.0f},
        {4.0f, 3.0f, 3.0f},
        {8.0f, 7.0f, 9.0f}
    };
    Matrix<3, 3> L, U, P;
    size_t swapCount;

    bool success = lu(A, L, U, P, swapCount);
    REQUIRE(success);

    // U should be upper triangular (zeros below diagonal)
    for(uint8_t i = 0; i < 3; i++) {
        for(uint8_t j = 0; j < i; j++) {
            REQUIRE_THAT(U(i, j), Catch::Matchers::WithinAbs(0.0, 1e-6));
        }
    }
}

// ----------------------------------------------------------------------------
// QR Decomposition Tests
// ----------------------------------------------------------------------------

TEST_CASE("Matrix - QR Decomposition", "[matrix][util][qr]") {
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

TEST_CASE("Matrix - QR Decomposition Reconstruction", "[matrix][util][qr]") {
    Matrix<3, 2> A = {
        {12.0f, -51.0f},
        {6.0f, 167.0f},
        {-4.0f, 24.0f}
    };
    Matrix<3, 3> Q;
    Matrix<3, 2> R;

    qr(A, Q, R);

    // Verify A = Q * R (first 2 columns)
    Matrix<3, 2> QR;
    for(uint8_t i = 0; i < 3; i++) {
        for(uint8_t j = 0; j < 2; j++) {
            QR(i, j) = 0.0f;
            for(uint8_t k = 0; k < 3; k++) {
                QR(i, j) += Q(i, k) * R(k, j);
            }
        }
    }

    for(uint8_t i = 0; i < 3; i++) {
        for(uint8_t j = 0; j < 2; j++) {
            REQUIRE_THAT(QR(i, j), Catch::Matchers::WithinAbs(A(i, j), 1e-3));
        }
    }
}

TEST_CASE("Matrix - QR Decomposition Linearly Dependent", "[matrix][util][qr]") {
    Matrix<3, 2> A = {
        {1.0f, 2.0f},
        {2.0f, 4.0f},
        {3.0f, 6.0f}
    };
    Matrix<3, 3> Q;
    Matrix<3, 2> R;

    bool success = qr(A, Q, R);

    REQUIRE_FALSE(success);
}

// ----------------------------------------------------------------------------
// Gram-Schmidt Tests
// ----------------------------------------------------------------------------

TEST_CASE("Matrix - Gram-Schmidt Orthonormalization", "[matrix][util][gramschmidt]") {
    Matrix<3, 2> A = {
        {1.0f, 1.0f},
        {0.0f, 1.0f},
        {0.0f, 0.0f}
    };
    Matrix<3, 2> Q;

    bool success = gramSchmidtReduced(A, Q);

    REQUIRE(success);

    // Check orthonormality of columns
    Vector<3> col0 = getColumn(Q, 0);
    Vector<3> col1 = getColumn(Q, 1);

    REQUIRE_THAT(norm(col0), Catch::Matchers::WithinAbs(1.0, 1e-5));
    REQUIRE_THAT(norm(col1), Catch::Matchers::WithinAbs(1.0, 1e-5));
    REQUIRE_THAT(dot(col0, col1), Catch::Matchers::WithinAbs(0.0, 1e-5));
}

TEST_CASE("Matrix - Gram-Schmidt Linearly Dependent", "[matrix][util][gramschmidt]") {
    Matrix<3, 2> A = {
        {1.0f, 2.0f},
        {2.0f, 4.0f},
        {3.0f, 6.0f}
    };
    Matrix<3, 2> Q;

    bool success = gramSchmidtReduced(A, Q);

    REQUIRE_FALSE(success);
}

TEST_CASE("Matrix - Gram-Schmidt Full Basis", "[matrix][util][gramschmidt]") {
    Matrix<3, 2> A = {
        {1.0f, 0.0f},
        {0.0f, 1.0f},
        {0.0f, 0.0f}
    };
    Matrix<3, 3> Q;

    bool success = gramSchmidt(A, Q);

    REQUIRE(success);

    // Q should be orthonormal
    Matrix<3, 3> QtQ = transpose(Q) * Q;
    Matrix<3, 3> I = Matrix<3, 3>::eye();

    for(uint8_t i = 0; i < 3; i++) {
        for(uint8_t j = 0; j < 3; j++) {
            REQUIRE_THAT(QtQ(i, j), Catch::Matchers::WithinAbs(I(i, j), 1e-4));
        }
    }
}

// ----------------------------------------------------------------------------
// SVD Decomposition Tests
// ----------------------------------------------------------------------------

TEST_CASE("Matrix - SVD Decomposition", "[matrix][util][svd]") {
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

TEST_CASE("Matrix - SVD Identity", "[matrix][util][svd]") {
    Matrix<3, 3> A = Matrix<3, 3>::eye();
    Matrix<3, 3> U, S, V;

    svd(A, U, S, V);

    // For identity matrix, S should also be identity
    for(uint8_t i = 0; i < 3; i++) {
        REQUIRE_THAT(S(i, i), Catch::Matchers::WithinAbs(1.0, 1e-5));
    }
}

TEST_CASE("Matrix - SVD Singular Values Ordered", "[matrix][util][svd]") {
    Matrix<3, 2> A = {
        {3.0f, 0.0f},
        {0.0f, 2.0f},
        {0.0f, 0.0f}
    };
    Matrix<3, 3> U;
    Matrix<3, 2> S;
    Matrix<2, 2> V;

    svd(A, U, S, V);

    // Singular values should be in descending order
    REQUIRE(S(0, 0) >= S(1, 1));
}

TEST_CASE("Matrix - SVD Reconstruction", "[matrix][util][svd]") {
    Matrix<2, 2> A = {
        {1.0f, 2.0f},
        {3.0f, 4.0f}
    };
    Matrix<2, 2> U, S, V;

    svd(A, U, S, V);

    // Verify A ≈ U * S * V^T
    Matrix<2, 2> reconstructed = U * S * transpose(V);

    for(uint8_t i = 0; i < 2; i++) {
        for(uint8_t j = 0; j < 2; j++) {
            REQUIRE_THAT(reconstructed(i, j), Catch::Matchers::WithinAbs(A(i, j), 1e-3));
        }
    }
}

// ----------------------------------------------------------------------------
// Conversion Tests
// ----------------------------------------------------------------------------

TEST_CASE("Matrix - Get Column", "[matrix][util][conversion]") {
    Matrix<3, 2> m = {
        {1.0f, 4.0f},
        {2.0f, 5.0f},
        {3.0f, 6.0f}
    };

    Vector<3> col0 = getColumn(m, 0);
    Vector<3> col1 = getColumn(m, 1);

    REQUIRE_THAT(col0[0], Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(col0[1], Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(col0[2], Catch::Matchers::WithinAbs(3.0, 1e-6));

    REQUIRE_THAT(col1[0], Catch::Matchers::WithinAbs(4.0, 1e-6));
    REQUIRE_THAT(col1[1], Catch::Matchers::WithinAbs(5.0, 1e-6));
    REQUIRE_THAT(col1[2], Catch::Matchers::WithinAbs(6.0, 1e-6));
}

TEST_CASE("Matrix - Get Row", "[matrix][util][conversion]") {
    Matrix<2, 3> m = {
        {1.0f, 2.0f, 3.0f},
        {4.0f, 5.0f, 6.0f}
    };

    Vector<3> row0 = getRow(m, 0);
    Vector<3> row1 = getRow(m, 1);

    REQUIRE_THAT(row0[0], Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(row0[1], Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(row0[2], Catch::Matchers::WithinAbs(3.0, 1e-6));

    REQUIRE_THAT(row1[0], Catch::Matchers::WithinAbs(4.0, 1e-6));
    REQUIRE_THAT(row1[1], Catch::Matchers::WithinAbs(5.0, 1e-6));
    REQUIRE_THAT(row1[2], Catch::Matchers::WithinAbs(6.0, 1e-6));
}

TEST_CASE("Matrix - Get Diagonal", "[matrix][util][conversion]") {
    Matrix<3, 3> m = {
        {1.0f, 2.0f, 3.0f},
        {4.0f, 5.0f, 6.0f},
        {7.0f, 8.0f, 9.0f}
    };

    Vector<3> diag = getDiagonal(m);

    REQUIRE_THAT(diag[0], Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(diag[1], Catch::Matchers::WithinAbs(5.0, 1e-6));
    REQUIRE_THAT(diag[2], Catch::Matchers::WithinAbs(9.0, 1e-6));
}

TEST_CASE("Matrix - Get Diagonal Non-Square", "[matrix][util][conversion]") {
    Matrix<2, 3> m = {
        {1.0f, 2.0f, 3.0f},
        {4.0f, 5.0f, 6.0f}
    };

    Vector<2> diag = getDiagonal(m);

    REQUIRE_THAT(diag[0], Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(diag[1], Catch::Matchers::WithinAbs(5.0, 1e-6));
}

// ----------------------------------------------------------------------------
// Check Functions Tests
// ----------------------------------------------------------------------------

TEST_CASE("Matrix - Is Zero True", "[matrix][util][check]") {
    Matrix<2, 2> zero = Matrix<2, 2>::zero();

    REQUIRE(isZero(zero));
}

TEST_CASE("Matrix - Is Zero False", "[matrix][util][check]") {
    Matrix<2, 2> m = {
        {0.0f, 0.0f},
        {0.0f, 0.001f}
    };

    REQUIRE_FALSE(isZero(m));
}

TEST_CASE("Matrix - Is Identity True", "[matrix][util][check]") {
    Matrix<3, 3> I = Matrix<3, 3>::eye();

    REQUIRE(isIdentity(I));
}

TEST_CASE("Matrix - Is Identity False", "[matrix][util][check]") {
    Matrix<3, 3> m = {
        {1.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.001f},
        {0.0f, 0.0f, 1.0f}
    };

    REQUIRE_FALSE(isIdentity(m));
}

TEST_CASE("Matrix - Is Symmetric True", "[matrix][util][check]") {
    Matrix<3, 3> m = {
        {1.0f, 2.0f, 3.0f},
        {2.0f, 4.0f, 5.0f},
        {3.0f, 5.0f, 6.0f}
    };

    REQUIRE(isSymmetric(m));
}

TEST_CASE("Matrix - Is Symmetric False", "[matrix][util][check]") {
    Matrix<3, 3> m = {
        {1.0f, 2.0f, 3.0f},
        {4.0f, 5.0f, 6.0f},
        {7.0f, 8.0f, 9.0f}
    };

    REQUIRE_FALSE(isSymmetric(m));
}

TEST_CASE("Matrix - Is Orthogonal True", "[matrix][util][check]") {
    // Rotation matrix is orthogonal
    float angle = M_PI / 4;
    Matrix<2, 2> R = {
        {std::cos(angle), -std::sin(angle)},
        {std::sin(angle), std::cos(angle)}
    };

    REQUIRE(isOrthogonal(R));
}

TEST_CASE("Matrix - Is Orthogonal False", "[matrix][util][check]") {
    Matrix<2, 2> m = {
        {1.0f, 2.0f},
        {3.0f, 4.0f}
    };

    REQUIRE_FALSE(isOrthogonal(m));
}

TEST_CASE("Matrix - Is Diagonal True", "[matrix][util][check]") {
    Matrix<3, 3> m = {
        {1.0f, 0.0f, 0.0f},
        {0.0f, 2.0f, 0.0f},
        {0.0f, 0.0f, 3.0f}
    };

    REQUIRE(isDiagonal(m));
}

TEST_CASE("Matrix - Is Diagonal False", "[matrix][util][check]") {
    Matrix<3, 3> m = {
        {1.0f, 0.1f, 0.0f},
        {0.0f, 2.0f, 0.0f},
        {0.0f, 0.0f, 3.0f}
    };

    REQUIRE_FALSE(isDiagonal(m));
}

TEST_CASE("Matrix - Is Singular True", "[matrix][util][check]") {
    Matrix<2, 2> m = {
        {1.0f, 2.0f},
        {2.0f, 4.0f}
    };

    REQUIRE(isSingular(m));
}

TEST_CASE("Matrix - Is Singular False", "[matrix][util][check]") {
    Matrix<2, 2> m = {
        {1.0f, 2.0f},
        {3.0f, 4.0f}
    };

    REQUIRE_FALSE(isSingular(m));
}

// ----------------------------------------------------------------------------
// Integration Tests
// ----------------------------------------------------------------------------

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

    // Pseudoinverse should satisfy A * A+ * A ≈ A
    Matrix<3, 2> reconstructed = A * Apinv * A;

    for(uint8_t i = 0; i < 3; i++) {
        for(uint8_t j = 0; j < 2; j++) {
            REQUIRE_THAT(reconstructed(i, j), Catch::Matchers::WithinAbs(A(i, j), 1e-4));
        }
    }
}

TEST_CASE("Matrix - Combined Operations", "[matrix][integration]") {
    Matrix<2, 2> A = {
        {1.0f, 2.0f},
        {3.0f, 4.0f}
    };

    // Clean, transpose, clamp
    Matrix<2, 2> result = cleanZero(transpose(clamp(A, 0.0f, 3.0f)));

    REQUIRE_THAT(result(0, 0), Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(result(1, 0), Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(result(0, 1), Catch::Matchers::WithinAbs(3.0, 1e-6));
    REQUIRE_THAT(result(1, 1), Catch::Matchers::WithinAbs(3.0, 1e-6));
}