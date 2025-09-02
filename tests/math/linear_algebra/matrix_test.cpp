#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <cobalt/math/linear_algebra/matrix/matrix.hpp>
#include <cobalt/math/linear_algebra/matrix/matrix_ops.hpp>
#include <cobalt/math/linear_algebra/matrix/matrix_util.hpp>

#include <cobalt/math/linear_algebra/vector/vector.hpp>

using cobalt::math::linear_algebra::Matrix;
using cobalt::math::linear_algebra::Vector;

TEST_CASE("Matrix, default construction", "[matrix]") {
    Matrix<2, 2> A;

    REQUIRE(A(0, 0) == Catch::Approx(0.0f));
    REQUIRE(A(0, 1) == Catch::Approx(0.0f));
    REQUIRE(A(1, 0) == Catch::Approx(0.0f));
    REQUIRE(A(1, 1) == Catch::Approx(0.0f));
}   

TEST_CASE("Matrix, manual construction", "[matrix]") {
    Matrix<2, 2> A = {{0.0f, 1.0f}, {2.0f, 3.0f}};

    REQUIRE(A(0, 0) == Catch::Approx(0.0f));
    REQUIRE(A(0, 1) == Catch::Approx(1.0f));
    REQUIRE(A(1, 0) == Catch::Approx(2.0f));
    REQUIRE(A(1, 1) == Catch::Approx(3.0f));
}   

TEST_CASE("Matrix, zero & identity construction", "[matrix]") {
    Matrix<3, 3> A = Matrix<3, 3>::zero();

    REQUIRE(A(0, 0) == Catch::Approx(0.0f));
    REQUIRE(A(0, 1) == Catch::Approx(0.0f));
    REQUIRE(A(0, 2) == Catch::Approx(0.0f));
    REQUIRE(A(1, 0) == Catch::Approx(0.0f));
    REQUIRE(A(1, 1) == Catch::Approx(0.0f));
    REQUIRE(A(1, 2) == Catch::Approx(0.0f));
    REQUIRE(A(2, 0) == Catch::Approx(0.0f));
    REQUIRE(A(2, 1) == Catch::Approx(0.0f));
    REQUIRE(A(2, 2) == Catch::Approx(0.0f));

    A = Matrix<3, 3>::eye();

    REQUIRE(A(0, 0) == Catch::Approx(1.0f));
    REQUIRE(A(0, 1) == Catch::Approx(0.0f));
    REQUIRE(A(0, 2) == Catch::Approx(0.0f));
    REQUIRE(A(1, 0) == Catch::Approx(0.0f));
    REQUIRE(A(1, 1) == Catch::Approx(1.0f));
    REQUIRE(A(1, 2) == Catch::Approx(0.0f));
    REQUIRE(A(2, 0) == Catch::Approx(0.0f));
    REQUIRE(A(2, 1) == Catch::Approx(0.0f));
    REQUIRE(A(2, 2) == Catch::Approx(1.0f));

    Matrix<3, 2> B = Matrix<3, 2>::eye();

    REQUIRE(B(0, 0) == Catch::Approx(1.0f));
    REQUIRE(B(0, 1) == Catch::Approx(0.0f));
    REQUIRE(B(1, 0) == Catch::Approx(0.0f));
    REQUIRE(B(1, 1) == Catch::Approx(1.0f));
    REQUIRE(B(2, 0) == Catch::Approx(0.0f));
    REQUIRE(B(2, 1) == Catch::Approx(0.0f));
}  

TEST_CASE("Matrix, arithmetic operations", "[matrix]") {
    Matrix<3, 3> A{{1.0f, 2.0f, 3.0f}, {4.0f, 5.0f, 6.0f}, {7.0f, 8.0f, 9.0f}};
    Matrix<3, 3> B{{1.0f, 1.0f, 1.0f}, {1.0f, 1.0f, 1.0f}, {1.0f, 1.0f, 1.0f}};

    A += B;

    REQUIRE(A(0, 0) == Catch::Approx(2.0f));
    REQUIRE(A(0, 1) == Catch::Approx(3.0f));
    REQUIRE(A(0, 2) == Catch::Approx(4.0f));
    REQUIRE(A(1, 0) == Catch::Approx(5.0f));
    REQUIRE(A(1, 1) == Catch::Approx(6.0f));
    REQUIRE(A(1, 2) == Catch::Approx(7.0f));
    REQUIRE(A(2, 0) == Catch::Approx(8.0f));
    REQUIRE(A(2, 1) == Catch::Approx(9.0f));
    REQUIRE(A(2, 2) == Catch::Approx(10.0f));

    A -= B;

    REQUIRE(A(0, 0) == Catch::Approx(1.0f));
    REQUIRE(A(0, 1) == Catch::Approx(2.0f));
    REQUIRE(A(0, 2) == Catch::Approx(3.0f));
    REQUIRE(A(1, 0) == Catch::Approx(4.0f));
    REQUIRE(A(1, 1) == Catch::Approx(5.0f));
    REQUIRE(A(1, 2) == Catch::Approx(6.0f));
    REQUIRE(A(2, 0) == Catch::Approx(7.0f));
    REQUIRE(A(2, 1) == Catch::Approx(8.0f));
    REQUIRE(A(2, 2) == Catch::Approx(9.0f));

    A *= B;

    REQUIRE(A(0, 0) == Catch::Approx(6.0f));
    REQUIRE(A(0, 1) == Catch::Approx(6.0f));
    REQUIRE(A(0, 2) == Catch::Approx(6.0f));
    REQUIRE(A(1, 0) == Catch::Approx(15.0f));
    REQUIRE(A(1, 1) == Catch::Approx(15.0f));
    REQUIRE(A(1, 2) == Catch::Approx(15.0f));
    REQUIRE(A(2, 0) == Catch::Approx(24.0f));
    REQUIRE(A(2, 1) == Catch::Approx(24.0f));
    REQUIRE(A(2, 2) == Catch::Approx(24.0f));

    A *= 0.5;

    REQUIRE(A(0, 0) == Catch::Approx(3.0f));
    REQUIRE(A(0, 1) == Catch::Approx(3.0f));
    REQUIRE(A(0, 2) == Catch::Approx(3.0f));
    REQUIRE(A(1, 0) == Catch::Approx(7.5f));
    REQUIRE(A(1, 1) == Catch::Approx(7.5f));
    REQUIRE(A(1, 2) == Catch::Approx(7.5f));
    REQUIRE(A(2, 0) == Catch::Approx(12.0f));
    REQUIRE(A(2, 1) == Catch::Approx(12.0f));
    REQUIRE(A(2, 2) == Catch::Approx(12.0f));

    A = -A;

    REQUIRE(A(0, 0) == Catch::Approx(-3.0f));
    REQUIRE(A(0, 1) == Catch::Approx(-3.0f));
    REQUIRE(A(0, 2) == Catch::Approx(-3.0f));
    REQUIRE(A(1, 0) == Catch::Approx(-7.5f));
    REQUIRE(A(1, 1) == Catch::Approx(-7.5f));
    REQUIRE(A(1, 2) == Catch::Approx(-7.5f));
    REQUIRE(A(2, 0) == Catch::Approx(-12.0f));
    REQUIRE(A(2, 1) == Catch::Approx(-12.0f));
    REQUIRE(A(2, 2) == Catch::Approx(-12.0f));
}   

TEST_CASE("Matrix, vector multiplication", "[matrix]") {
    Matrix<3, 3> A{{1.0f, 2.0f, 3.0f}, {4.0f, 5.0f, 6.0f}, {7.0f, 8.0f, 9.0f}};
    Matrix<3, 3> R{{0.0f, -1.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
    Matrix<2, 3> I = Matrix<2, 3>::eye();

    Vector<3> v{1.0f, 1.0f, 1.0f};
    Vector<3> u{3.0f, 4.0f, 0.0f};

    Vector<3> w = A*v;
    Vector<3> r = R*u;
    Vector<2> a = I*u;

    REQUIRE(w[0] == Catch::Approx(6.0f));
    REQUIRE(w[1] == Catch::Approx(15.0f));
    REQUIRE(w[2] == Catch::Approx(24.0f));

    REQUIRE(r[0] == Catch::Approx(-4.0f));
    REQUIRE(r[1] == Catch::Approx(3.0f));
    REQUIRE(r[2] == Catch::Approx(0.0f));

    REQUIRE(a[0] == Catch::Approx(3.0f));
    REQUIRE(a[1] == Catch::Approx(4.0f));
}   

TEST_CASE("Matrix, determinant", "[matrix]") {
    Matrix<1, 1> A{{5}};
    Matrix<2, 2> B{{1.0f, 2.0f}, {3.0f, 4.0f}};
    Matrix<3, 3> C{{1.0f, 2.0f, 3.0f}, {4.0f, 5.0f, 6.0f}, {7.0f, 8.0f, 9.0f}};
    Matrix<4, 4> D{{1.0f, 2.0f, 3.0f, 4.0f}, {5.0f, 6.0f, 3.0f, 8.0f}, {9.0f, 9.0f, 11.0f, 12.0f}, {13.0f, 14.0f, 15.0f, 16.0f}};

    REQUIRE(det(A) == Catch::Approx(5.0f));
    REQUIRE(det(B) == Catch::Approx(-2.0f));
    REQUIRE(det(C) == Catch::Approx(0.0f));
    REQUIRE(det(D) == Catch::Approx(144.0f));
}   

TEST_CASE("Matrix, transposition", "[matrix]") {
    Matrix<3, 3> A{{1.0f, 2.0f, 3.0f}, {4.0f, 5.0f, 6.0f}, {7.0f, 8.0f, 9.0f}};
    Matrix<2, 3> B = {{2.0f, 10.0f, -1.0f}, {3.0f, -3.5f, 0.5f}};

    A = transpose(A);
    Matrix<3, 2>Bt = transpose(B);

    REQUIRE(A(0, 0) == Catch::Approx(1.0f));
    REQUIRE(A(0, 1) == Catch::Approx(4.0f));
    REQUIRE(A(0, 2) == Catch::Approx(7.0f));
    REQUIRE(A(1, 0) == Catch::Approx(2.0f));
    REQUIRE(A(1, 1) == Catch::Approx(5.0f));
    REQUIRE(A(1, 2) == Catch::Approx(8.0f));
    REQUIRE(A(2, 0) == Catch::Approx(3.0f));
    REQUIRE(A(2, 1) == Catch::Approx(6.0f));
    REQUIRE(A(2, 2) == Catch::Approx(9.0f));

    REQUIRE(Bt(0, 0) == Catch::Approx(2.0f));
    REQUIRE(Bt(0, 1) == Catch::Approx(3.0f));
    REQUIRE(Bt(1, 0) == Catch::Approx(10.0f));
    REQUIRE(Bt(1, 1) == Catch::Approx(-3.5f));
    REQUIRE(Bt(2, 0) == Catch::Approx(-1.0f));
    REQUIRE(Bt(2, 1) == Catch::Approx(0.5f));
}   

TEST_CASE("Matrix, inverse", "[matrix]") {
    Matrix<3, 3> A{{1.0f, 2.0f, 6.0f}, {-2.0f, 5.0f, 6.0f}, {7.0f, 1.0f, 9.0f}};
    Matrix<3, 3> Ainv;

    REQUIRE(inv(A, Ainv));

    Matrix<3, 3> I = Matrix<3, 3>::eye();
    Matrix<3, 3> AAi = A * Ainv;
    Matrix<3, 3> AiA = Ainv * A;

    for(uint8_t i = 0; i < 3; i++) {
        for(uint8_t j = 0; j < 3; j++) {
            REQUIRE(AAi(i,j) == Catch::Approx(I(i,j)).margin(1e-6));
        }
    }

    for(uint8_t i = 0; i < 3; i++) {
        for(uint8_t j = 0; j < 3; j++) {
            REQUIRE(AiA(i,j) == Catch::Approx(I(i,j)).margin(1e-6));
        }
    }

}  

TEST_CASE("Matrix, trace & traceProduct", "[matrix]") {
    Matrix<3, 3> A{{1.0f, 2.0f, 3.0f}, {4.0f, 5.0f, 6.0f}, {7.0f, 8.0f, 9.0f}};

    REQUIRE(trace(A) == Catch::Approx(15.0f));
    REQUIRE(traceProduct(A) == Catch::Approx(45.0f));
}   

TEST_CASE("Matrix, solve", "[matrix]") {
    Matrix<3, 3> A{{1.0f, 2.0f, 6.0f}, {-2.0f, 5.0f, 6.0f}, {7.0f, 1.0f, 9.0f}};
    Vector<3> b{9.0f, -2.0f, 1.5f};
    Vector<3> x = Vector<3>::zero();

    REQUIRE(solve(A, b, x));

    REQUIRE(x.x() == Catch::Approx(-5.52381));
    REQUIRE(x.y() == Catch::Approx(-9.19048));
    REQUIRE(x.z() == Catch::Approx(5.48413));
}   

TEST_CASE("Matrix, LU-decomposion", "[matrix]") {
    Matrix<3, 3> A{{4.0f, 3.0f, 2.0f}, {2.0f, 1.0f, 3.0f}, {67.0f, 7.0f, 8.0f}};
    Matrix<3, 3> L, U;
    Vector<3> P;

    REQUIRE(decompLU(A, L, U, P));

    for(uint8_t i = 0; i < 3; i++) {
        REQUIRE(L(i,i) == Catch::Approx(1.0f));
        for(uint8_t j = i+1; j < 3; j++) {
            REQUIRE(L(i,j) == Catch::Approx(0.0f));
        }
    }

    for(uint8_t i = 1; i < 3; i++) {
        for(uint8_t j = 0; j < i; j++) {
            REQUIRE(U(i,j) == Catch::Approx(0.0f));
        }
    }

    Matrix<3,3> PA;

    for(uint8_t i = 0; i < 3; i++) {
        for(uint8_t j = 0; j < 3; j++) {
            PA(i, j) = A(P[i], j);
        }
    }

    Matrix<3,3> LU = L * U;

    for(uint8_t i = 0; i < 3; i++) {
        for(uint8_t j = 0; j < 3; j++) {
            REQUIRE(PA(i,j) == Catch::Approx(LU(i,j)).margin(1e-6));
        }
    }


}   