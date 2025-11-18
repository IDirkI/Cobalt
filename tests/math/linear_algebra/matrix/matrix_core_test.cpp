#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "cobalt/math/linear_algebra/matrix/matrix.hpp"

using namespace cobalt::math::linear_algebra;

// ============================================================================
// Matrix Core Tests (matrix.hpp)
// ============================================================================

TEST_CASE("Matrix - Default Construction", "[matrix][core]") {
    Matrix<3, 3> m;
    
    for(uint8_t i = 0; i < 3; i++) {
        for(uint8_t j = 0; j < 3; j++) {
            REQUIRE_THAT(m(i, j), Catch::Matchers::WithinAbs(0.0, 1e-6));
        }
    }
}

TEST_CASE("Matrix - Initializer List Construction", "[matrix][core]") {
    Matrix<2, 3> m = {
        {1.0f, 2.0f, 3.0f},
        {4.0f, 5.0f, 6.0f}
    };
    
    REQUIRE_THAT(m(0, 0), Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(m(0, 1), Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(m(0, 2), Catch::Matchers::WithinAbs(3.0, 1e-6));
    REQUIRE_THAT(m(1, 0), Catch::Matchers::WithinAbs(4.0, 1e-6));
    REQUIRE_THAT(m(1, 1), Catch::Matchers::WithinAbs(5.0, 1e-6));
    REQUIRE_THAT(m(1, 2), Catch::Matchers::WithinAbs(6.0, 1e-6));
}

TEST_CASE("Matrix - Initializer List Partial", "[matrix][core]") {
    Matrix<3, 3> m = {
        {1.0f, 2.0f},
        {3.0f}
    };
    
    REQUIRE_THAT(m(0, 0), Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(m(0, 1), Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(m(0, 2), Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(m(1, 0), Catch::Matchers::WithinAbs(3.0, 1e-6));
    REQUIRE_THAT(m(1, 1), Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(m(2, 2), Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Matrix - Zero Factory", "[matrix][core]") {
    Matrix<3, 3> m = Matrix<3, 3>::zero();
    
    for(uint8_t i = 0; i < 3; i++) {
        for(uint8_t j = 0; j < 3; j++) {
            REQUIRE_THAT(m(i, j), Catch::Matchers::WithinAbs(0.0, 1e-6));
        }
    }
}

TEST_CASE("Matrix - Identity Factory", "[matrix][core]") {
    Matrix<3, 3> m = Matrix<3, 3>::eye();
    
    for(uint8_t i = 0; i < 3; i++) {
        for(uint8_t j = 0; j < 3; j++) {
            if(i == j) {
                REQUIRE_THAT(m(i, j), Catch::Matchers::WithinAbs(1.0, 1e-6));
            } else {
                REQUIRE_THAT(m(i, j), Catch::Matchers::WithinAbs(0.0, 1e-6));
            }
        }
    }
}

TEST_CASE("Matrix - Identity Factory Non-Square", "[matrix][core]") {
    Matrix<2, 3> m = Matrix<2, 3>::eye();
    
    REQUIRE_THAT(m(0, 0), Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(m(1, 1), Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(m(0, 1), Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(m(1, 0), Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Matrix - Diagonal Factory", "[matrix][core]") {
    Vector<3> v = {1.0f, 2.0f, 3.0f};
    Matrix<3, 3> m = Matrix<3, 3>::diagonal(v);
    
    REQUIRE_THAT(m(0, 0), Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(m(1, 1), Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(m(2, 2), Catch::Matchers::WithinAbs(3.0, 1e-6));
    REQUIRE_THAT(m(0, 1), Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(m(1, 0), Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Matrix - Diagonal Factory Partial", "[matrix][core]") {
    Vector<2> v = {5.0f, 6.0f};
    Matrix<4, 4> m = Matrix<4, 4>::diagonal(v);
    
    REQUIRE_THAT(m(0, 0), Catch::Matchers::WithinAbs(5.0, 1e-6));
    REQUIRE_THAT(m(1, 1), Catch::Matchers::WithinAbs(6.0, 1e-6));
    REQUIRE_THAT(m(2, 2), Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(m(3, 3), Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Matrix - Rows and Cols Getters", "[matrix][core]") {
    Matrix<3, 4> m;
    
    REQUIRE(m.rows() == 3);
    REQUIRE(m.cols() == 4);
}

TEST_CASE("Matrix - Element Access", "[matrix][core]") {
    Matrix<2, 2> m = {
        {1.0f, 2.0f},
        {3.0f, 4.0f}
    };
    
    REQUIRE_THAT(m(0, 0), Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(m(0, 1), Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(m(1, 0), Catch::Matchers::WithinAbs(3.0, 1e-6));
    REQUIRE_THAT(m(1, 1), Catch::Matchers::WithinAbs(4.0, 1e-6));
}

TEST_CASE("Matrix - Element Modification", "[matrix][core]") {
    Matrix<2, 2> m;
    
    m(0, 0) = 10.0f;
    m(0, 1) = 20.0f;
    m(1, 0) = 30.0f;
    m(1, 1) = 40.0f;
    
    REQUIRE_THAT(m(0, 0), Catch::Matchers::WithinAbs(10.0, 1e-6));
    REQUIRE_THAT(m(0, 1), Catch::Matchers::WithinAbs(20.0, 1e-6));
    REQUIRE_THAT(m(1, 0), Catch::Matchers::WithinAbs(30.0, 1e-6));
    REQUIRE_THAT(m(1, 1), Catch::Matchers::WithinAbs(40.0, 1e-6));
}

TEST_CASE("Matrix - Safe Element Access", "[matrix][core]") {
    Matrix<2, 2> m = {
        {1.0f, 2.0f},
        {3.0f, 4.0f}
    };
    
    // at() should clamp to last element
    REQUIRE_THAT(m.at(10, 10), Catch::Matchers::WithinAbs(4.0, 1e-6));
    REQUIRE_THAT(m.at(0, 10), Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(m.at(10, 0), Catch::Matchers::WithinAbs(3.0, 1e-6));
}

TEST_CASE("Matrix - Addition Assignment", "[matrix][core]") {
    Matrix<2, 2> m1 = {
        {1.0f, 2.0f},
        {3.0f, 4.0f}
    };
    Matrix<2, 2> m2 = {
        {5.0f, 6.0f},
        {7.0f, 8.0f}
    };
    
    m1 += m2;
    
    REQUIRE_THAT(m1(0, 0), Catch::Matchers::WithinAbs(6.0, 1e-6));
    REQUIRE_THAT(m1(0, 1), Catch::Matchers::WithinAbs(8.0, 1e-6));
    REQUIRE_THAT(m1(1, 0), Catch::Matchers::WithinAbs(10.0, 1e-6));
    REQUIRE_THAT(m1(1, 1), Catch::Matchers::WithinAbs(12.0, 1e-6));
}

TEST_CASE("Matrix - Subtraction Assignment", "[matrix][core]") {
    Matrix<2, 2> m1 = {
        {5.0f, 6.0f},
        {7.0f, 8.0f}
    };
    Matrix<2, 2> m2 = {
        {1.0f, 2.0f},
        {3.0f, 4.0f}
    };
    
    m1 -= m2;
    
    REQUIRE_THAT(m1(0, 0), Catch::Matchers::WithinAbs(4.0, 1e-6));
    REQUIRE_THAT(m1(0, 1), Catch::Matchers::WithinAbs(4.0, 1e-6));
    REQUIRE_THAT(m1(1, 0), Catch::Matchers::WithinAbs(4.0, 1e-6));
    REQUIRE_THAT(m1(1, 1), Catch::Matchers::WithinAbs(4.0, 1e-6));
}

TEST_CASE("Matrix - Multiplication Assignment", "[matrix][core]") {
    Matrix<2, 2> m1 = {
        {1.0f, 2.0f},
        {3.0f, 4.0f}
    };
    Matrix<2, 2> m2 = {
        {2.0f, 0.0f},
        {1.0f, 2.0f}
    };
    
    m1 *= m2;
    
    // [1 2] * [2 0] = [4 4]
    // [3 4]   [1 2]   [10 8]
    REQUIRE_THAT(m1(0, 0), Catch::Matchers::WithinAbs(4.0, 1e-6));
    REQUIRE_THAT(m1(0, 1), Catch::Matchers::WithinAbs(4.0, 1e-6));
    REQUIRE_THAT(m1(1, 0), Catch::Matchers::WithinAbs(10.0, 1e-6));
    REQUIRE_THAT(m1(1, 1), Catch::Matchers::WithinAbs(8.0, 1e-6));
}

TEST_CASE("Matrix - Scalar Multiplication Assignment", "[matrix][core]") {
    Matrix<2, 2> m = {
        {1.0f, 2.0f},
        {3.0f, 4.0f}
    };
    
    m *= 2.0f;
    
    REQUIRE_THAT(m(0, 0), Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(m(0, 1), Catch::Matchers::WithinAbs(4.0, 1e-6));
    REQUIRE_THAT(m(1, 0), Catch::Matchers::WithinAbs(6.0, 1e-6));
    REQUIRE_THAT(m(1, 1), Catch::Matchers::WithinAbs(8.0, 1e-6));
}

TEST_CASE("Matrix - Scalar Division Assignment", "[matrix][core]") {
    Matrix<2, 2> m = {
        {2.0f, 4.0f},
        {6.0f, 8.0f}
    };
    
    m /= 2.0f;
    
    REQUIRE_THAT(m(0, 0), Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(m(0, 1), Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(m(1, 0), Catch::Matchers::WithinAbs(3.0, 1e-6));
    REQUIRE_THAT(m(1, 1), Catch::Matchers::WithinAbs(4.0, 1e-6));
}

TEST_CASE("Matrix - Block Extraction", "[matrix][core]") {
    Matrix<4, 4> m = {
        {1.0f, 2.0f, 3.0f, 4.0f},
        {5.0f, 6.0f, 7.0f, 8.0f},
        {9.0f, 10.0f, 11.0f, 12.0f},
        {13.0f, 14.0f, 15.0f, 16.0f}
    };
    
    Matrix<2, 2> block = m.block<2, 2>(1, 1);
    
    REQUIRE_THAT(block(0, 0), Catch::Matchers::WithinAbs(6.0, 1e-6));
    REQUIRE_THAT(block(0, 1), Catch::Matchers::WithinAbs(7.0, 1e-6));
    REQUIRE_THAT(block(1, 0), Catch::Matchers::WithinAbs(10.0, 1e-6));
    REQUIRE_THAT(block(1, 1), Catch::Matchers::WithinAbs(11.0, 1e-6));
}