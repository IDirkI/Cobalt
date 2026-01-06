#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "cobalt/math/linear_algebra/matrix/matrix.hpp"
#include "cobalt/math/linear_algebra/matrix/matrix_ops.hpp"

using namespace cobalt::math::linear_algebra;

// ============================================================================
// Matrix Core Tests (matrix.hpp)
// ============================================================================

// ----------------------------------------------------------------------------
// Construction Tests
// ----------------------------------------------------------------------------

TEST_CASE("Matrix - Default Construction Zeros All Elements", "[matrix][core][construction]") {
    Matrix<3, 3> m;
    
    for(uint8_t i = 0; i < 3; i++) {
        for(uint8_t j = 0; j < 3; j++) {
            REQUIRE_THAT(m(i, j), Catch::Matchers::WithinAbs(0.0, 1e-6));
        }
    }
}

TEST_CASE("Matrix - Default Construction Various Sizes", "[matrix][core][construction]") {
    Matrix<1, 1> m1;
    Matrix<2, 3> m2;
    Matrix<5, 4> m3;
    
    REQUIRE_THAT(m1(0, 0), Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(m2(1, 2), Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(m3(4, 3), Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Matrix - Initializer List Full Construction", "[matrix][core][construction]") {
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

TEST_CASE("Matrix - Initializer List Partial Row", "[matrix][core][construction]") {
    Matrix<3, 3> m = {
        {1.0f, 2.0f},
        {3.0f}
    };
    
    REQUIRE_THAT(m(0, 0), Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(m(0, 1), Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(m(0, 2), Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(m(1, 0), Catch::Matchers::WithinAbs(3.0, 1e-6));
    REQUIRE_THAT(m(1, 1), Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(m(1, 2), Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(m(2, 0), Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(m(2, 1), Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(m(2, 2), Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Matrix - Initializer List Partial Rows", "[matrix][core][construction]") {
    Matrix<4, 4> m = {
        {1.0f, 2.0f, 3.0f, 4.0f},
        {5.0f, 6.0f}
    };
    
    REQUIRE_THAT(m(0, 3), Catch::Matchers::WithinAbs(4.0, 1e-6));
    REQUIRE_THAT(m(1, 0), Catch::Matchers::WithinAbs(5.0, 1e-6));
    REQUIRE_THAT(m(1, 1), Catch::Matchers::WithinAbs(6.0, 1e-6));
    REQUIRE_THAT(m(1, 2), Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(m(2, 0), Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(m(3, 3), Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Matrix - Initializer List Excess Elements Ignored", "[matrix][core][construction]") {
    Matrix<2, 2> m = {
        {1.0f, 2.0f, 99.0f},  // 99.0f should be ignored
        {3.0f, 4.0f, 88.0f},  // 88.0f should be ignored
        {77.0f, 66.0f}        // This entire row should be ignored
    };
    
    REQUIRE_THAT(m(0, 0), Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(m(0, 1), Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(m(1, 0), Catch::Matchers::WithinAbs(3.0, 1e-6));
    REQUIRE_THAT(m(1, 1), Catch::Matchers::WithinAbs(4.0, 1e-6));
}

TEST_CASE("Matrix - Initializer List Single Element", "[matrix][core][construction]") {
    Matrix<3, 3> m = {{5.0f}};
    
    REQUIRE_THAT(m(0, 0), Catch::Matchers::WithinAbs(5.0, 1e-6));
    REQUIRE_THAT(m(0, 1), Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(m(1, 0), Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Matrix - Initializer List Empty Rows", "[matrix][core][construction]") {
    Matrix<3, 3> m = {
        {},
        {1.0f, 2.0f},
        {}
    };
    
    REQUIRE_THAT(m(0, 0), Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(m(1, 0), Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(m(1, 1), Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(m(2, 0), Catch::Matchers::WithinAbs(0.0, 1e-6));
}

// ----------------------------------------------------------------------------
// Factory Method Tests
// ----------------------------------------------------------------------------

TEST_CASE("Matrix - Zero Factory Creates All Zeros", "[matrix][core][factory]") {
    Matrix<3, 3> m = Matrix<3, 3>::zero();
    
    for(uint8_t i = 0; i < 3; i++) {
        for(uint8_t j = 0; j < 3; j++) {
            REQUIRE_THAT(m(i, j), Catch::Matchers::WithinAbs(0.0, 1e-6));
        }
    }
}

TEST_CASE("Matrix - Zero Factory Non-Square", "[matrix][core][factory]") {
    Matrix<2, 4> m = Matrix<2, 4>::zero();
    
    for(uint8_t i = 0; i < 2; i++) {
        for(uint8_t j = 0; j < 4; j++) {
            REQUIRE_THAT(m(i, j), Catch::Matchers::WithinAbs(0.0, 1e-6));
        }
    }
}

TEST_CASE("Matrix - Identity Factory Square", "[matrix][core][factory]") {
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

TEST_CASE("Matrix - Identity Factory 1x1", "[matrix][core][factory]") {
    Matrix<1, 1> m = Matrix<1, 1>::eye();
    
    REQUIRE_THAT(m(0, 0), Catch::Matchers::WithinAbs(1.0, 1e-6));
}

TEST_CASE("Matrix - Identity Factory Tall Matrix", "[matrix][core][factory]") {
    Matrix<4, 2> m = Matrix<4, 2>::eye();
    
    REQUIRE_THAT(m(0, 0), Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(m(1, 1), Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(m(0, 1), Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(m(1, 0), Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(m(2, 0), Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(m(2, 1), Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(m(3, 0), Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Matrix - Identity Factory Wide Matrix", "[matrix][core][factory]") {
    Matrix<2, 5> m = Matrix<2, 5>::eye();
    
    REQUIRE_THAT(m(0, 0), Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(m(1, 1), Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(m(0, 2), Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(m(1, 2), Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Matrix - Diagonal Factory Full Vector", "[matrix][core][factory]") {
    Vector<3> v = {1.0f, 2.0f, 3.0f};
    Matrix<3, 3> m = Matrix<3, 3>::diagonal(v);
    
    REQUIRE_THAT(m(0, 0), Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(m(1, 1), Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(m(2, 2), Catch::Matchers::WithinAbs(3.0, 1e-6));
    REQUIRE_THAT(m(0, 1), Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(m(1, 0), Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(m(0, 2), Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Matrix - Diagonal Factory Partial Vector", "[matrix][core][factory]") {
    Vector<2> v = {5.0f, 6.0f};
    Matrix<4, 4> m = Matrix<4, 4>::diagonal(v);
    
    REQUIRE_THAT(m(0, 0), Catch::Matchers::WithinAbs(5.0, 1e-6));
    REQUIRE_THAT(m(1, 1), Catch::Matchers::WithinAbs(6.0, 1e-6));
    REQUIRE_THAT(m(2, 2), Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(m(3, 3), Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(m(0, 1), Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Matrix - Diagonal Factory Single Element", "[matrix][core][factory]") {
    Vector<1> v = {7.0f};
    Matrix<3, 3> m = Matrix<3, 3>::diagonal(v);
    
    REQUIRE_THAT(m(0, 0), Catch::Matchers::WithinAbs(7.0, 1e-6));
    REQUIRE_THAT(m(1, 1), Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(m(2, 2), Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Matrix - Diagonal Factory Non-Square Tall", "[matrix][core][factory]") {
    Vector<2> v = {1.0f, 2.0f};
    Matrix<4, 3> m = Matrix<4, 3>::diagonal(v);
    
    REQUIRE_THAT(m(0, 0), Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(m(1, 1), Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(m(2, 2), Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Matrix - Diagonal Factory Non-Square Wide", "[matrix][core][factory]") {
    Vector<2> v = {3.0f, 4.0f};
    Matrix<3, 5> m = Matrix<3, 5>::diagonal(v);
    
    REQUIRE_THAT(m(0, 0), Catch::Matchers::WithinAbs(3.0, 1e-6));
    REQUIRE_THAT(m(1, 1), Catch::Matchers::WithinAbs(4.0, 1e-6));
    REQUIRE_THAT(m(2, 2), Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(m(0, 3), Catch::Matchers::WithinAbs(0.0, 1e-6));
}

// ----------------------------------------------------------------------------
// Getter Tests
// ----------------------------------------------------------------------------

TEST_CASE("Matrix - Rows Getter", "[matrix][core][getter]") {
    Matrix<3, 4> m;
    REQUIRE(m.rows() == 3);
}

TEST_CASE("Matrix - Cols Getter", "[matrix][core][getter]") {
    Matrix<3, 4> m;
    REQUIRE(m.cols() == 4);
}

TEST_CASE("Matrix - Rows and Cols Square Matrix", "[matrix][core][getter]") {
    Matrix<5, 5> m;
    REQUIRE(m.rows() == 5);
    REQUIRE(m.cols() == 5);
}

TEST_CASE("Matrix - Rows and Cols Minimal Matrix", "[matrix][core][getter]") {
    Matrix<1, 1> m;
    REQUIRE(m.rows() == 1);
    REQUIRE(m.cols() == 1);
}

// ----------------------------------------------------------------------------
// Element Access Tests
// ----------------------------------------------------------------------------

TEST_CASE("Matrix - Element Access Operator", "[matrix][core][access]") {
    Matrix<2, 2> m = {
        {1.0f, 2.0f},
        {3.0f, 4.0f}
    };
    
    REQUIRE_THAT(m(0, 0), Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(m(0, 1), Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(m(1, 0), Catch::Matchers::WithinAbs(3.0, 1e-6));
    REQUIRE_THAT(m(1, 1), Catch::Matchers::WithinAbs(4.0, 1e-6));
}

TEST_CASE("Matrix - Element Access First and Last", "[matrix][core][access]") {
    Matrix<3, 4> m = {
        {1.0f, 2.0f, 3.0f, 4.0f},
        {5.0f, 6.0f, 7.0f, 8.0f},
        {9.0f, 10.0f, 11.0f, 12.0f}
    };
    
    REQUIRE_THAT(m(0, 0), Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(m(2, 3), Catch::Matchers::WithinAbs(12.0, 1e-6));
}

TEST_CASE("Matrix - Element Modification", "[matrix][core][access]") {
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

TEST_CASE("Matrix - Element Modification Chain", "[matrix][core][access]") {
    Matrix<2, 2> m;
    
    m(0, 0) = 1.0f;
    m(0, 0) += 2.0f;
    m(0, 0) *= 3.0f;
    
    REQUIRE_THAT(m(0, 0), Catch::Matchers::WithinAbs(9.0, 1e-6));
}

TEST_CASE("Matrix - Const Element Access", "[matrix][core][access]") {
    const Matrix<2, 2> m = {
        {1.0f, 2.0f},
        {3.0f, 4.0f}
    };
    
    REQUIRE_THAT(m(0, 1), Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(m(1, 0), Catch::Matchers::WithinAbs(3.0, 1e-6));
}

TEST_CASE("Matrix - Safe At Method In Bounds", "[matrix][core][access]") {
    Matrix<2, 2> m = {
        {1.0f, 2.0f},
        {3.0f, 4.0f}
    };
    
    REQUIRE_THAT(m.at(0, 0), Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(m.at(1, 1), Catch::Matchers::WithinAbs(4.0, 1e-6));
}

TEST_CASE("Matrix - Safe At Method Clamps Row Out of Bounds", "[matrix][core][access]") {
    Matrix<2, 2> m = {
        {1.0f, 2.0f},
        {3.0f, 4.0f}
    };
    
    // Should clamp to last row
    REQUIRE_THAT(m.at(10, 0), Catch::Matchers::WithinAbs(3.0, 1e-6));
    REQUIRE_THAT(m.at(10, 1), Catch::Matchers::WithinAbs(4.0, 1e-6));
}

TEST_CASE("Matrix - Safe At Method Clamps Col Out of Bounds", "[matrix][core][access]") {
    Matrix<2, 2> m = {
        {1.0f, 2.0f},
        {3.0f, 4.0f}
    };
    
    // Should clamp to last column
    REQUIRE_THAT(m.at(0, 10), Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(m.at(1, 10), Catch::Matchers::WithinAbs(4.0, 1e-6));
}

TEST_CASE("Matrix - Safe At Method Clamps Both Out of Bounds", "[matrix][core][access]") {
    Matrix<2, 2> m = {
        {1.0f, 2.0f},
        {3.0f, 4.0f}
    };
    
    // Should clamp to last element
    REQUIRE_THAT(m.at(100, 100), Catch::Matchers::WithinAbs(4.0, 1e-6));
}

TEST_CASE("Matrix - Safe At Method Const Version", "[matrix][core][access]") {
    const Matrix<2, 2> m = {
        {1.0f, 2.0f},
        {3.0f, 4.0f}
    };
    
    REQUIRE_THAT(m.at(10, 10), Catch::Matchers::WithinAbs(4.0, 1e-6));
}

TEST_CASE("Matrix - Data Pointer Access", "[matrix][core][access]") {
    Matrix<2, 2> m = {
        {1.0f, 2.0f},
        {3.0f, 4.0f}
    };
    
    float* ptr = m.data();
    REQUIRE(ptr != nullptr);
    REQUIRE_THAT(ptr[0], Catch::Matchers::WithinAbs(1.0, 1e-6));
}

TEST_CASE("Matrix - Data Pointer Modification", "[matrix][core][access]") {
    Matrix<2, 2> m;
    float* ptr = m.data();
    
    ptr[0] = 5.0f;
    ptr[1] = 6.0f;
    
    REQUIRE_THAT(m(0, 0), Catch::Matchers::WithinAbs(5.0, 1e-6));
    REQUIRE_THAT(m(0, 1), Catch::Matchers::WithinAbs(6.0, 1e-6));
}

TEST_CASE("Matrix - Const Data Pointer Access", "[matrix][core][access]") {
    const Matrix<2, 2> m = {
        {1.0f, 2.0f},
        {3.0f, 4.0f}
    };
    
    const float* ptr = m.data();
    REQUIRE(ptr != nullptr);
    REQUIRE_THAT(ptr[3], Catch::Matchers::WithinAbs(4.0, 1e-6));
}

TEST_CASE("Matrix - Iterator Begin and End", "[matrix][core][access]") {
    Matrix<2, 2> m = {
        {1.0f, 2.0f},
        {3.0f, 4.0f}
    };
    
    auto it = m.begin();
    REQUIRE(it != m.end());
    REQUIRE_THAT(*it, Catch::Matchers::WithinAbs(1.0, 1e-6));
}

TEST_CASE("Matrix - Iterator Range Loop", "[matrix][core][access]") {
    Matrix<2, 2> m = {
        {1.0f, 2.0f},
        {3.0f, 4.0f}
    };
    
    float sum = 0.0f;
    for(auto val : m) {
        sum += val;
    }
    
    REQUIRE_THAT(sum, Catch::Matchers::WithinAbs(10.0, 1e-6));
}

// ----------------------------------------------------------------------------
// Arithmetic Assignment Operator Tests
// ----------------------------------------------------------------------------

TEST_CASE("Matrix - Addition Assignment", "[matrix][core][arithmetic]") {
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

TEST_CASE("Matrix - Addition Assignment With Zeros", "[matrix][core][arithmetic]") {
    Matrix<2, 2> m1 = {
        {1.0f, 2.0f},
        {3.0f, 4.0f}
    };
    Matrix<2, 2> m2 = Matrix<2, 2>::zero();
    
    Matrix<2, 2> original = m1;
    m1 += m2;
    
    REQUIRE(m1 == original);
}

TEST_CASE("Matrix - Addition Assignment Chain", "[matrix][core][arithmetic]") {
    Matrix<2, 2> m1 = {
        {1.0f, 2.0f},
        {3.0f, 4.0f}
    };
    Matrix<2, 2> m2 = {
        {1.0f, 1.0f},
        {1.0f, 1.0f}
    };
    
    m1 += m2;
    m1 += m2;
    m1 += m2;
    
    REQUIRE_THAT(m1(0, 0), Catch::Matchers::WithinAbs(4.0, 1e-6));
    REQUIRE_THAT(m1(1, 1), Catch::Matchers::WithinAbs(7.0, 1e-6));
}

TEST_CASE("Matrix - Subtraction Assignment", "[matrix][core][arithmetic]") {
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

TEST_CASE("Matrix - Subtraction Assignment Self Zeros", "[matrix][core][arithmetic]") {
    Matrix<2, 2> m1 = {
        {5.0f, 6.0f},
        {7.0f, 8.0f}
    };
    Matrix<2, 2> m2 = m1;
    
    m1 -= m2;
    
    REQUIRE(m1 == Matrix<2, 2>::zero());
}

TEST_CASE("Matrix - Multiplication Assignment Square", "[matrix][core][arithmetic]") {
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

TEST_CASE("Matrix - Multiplication Assignment Identity", "[matrix][core][arithmetic]") {
    Matrix<3, 3> m = {
        {1.0f, 2.0f, 3.0f},
        {4.0f, 5.0f, 6.0f},
        {7.0f, 8.0f, 9.0f}
    };
    Matrix<3, 3> original = m;
    Matrix<3, 3> I = Matrix<3, 3>::eye();
    
    m *= I;
    
    REQUIRE(m == original);
}

TEST_CASE("Matrix - Multiplication Assignment Zero", "[matrix][core][arithmetic]") {
    Matrix<2, 2> m = {
        {1.0f, 2.0f},
        {3.0f, 4.0f}
    };
    Matrix<2, 2> zero = Matrix<2, 2>::zero();
    
    m *= zero;
    
    REQUIRE(m == zero);
}

TEST_CASE("Matrix - Scalar Multiplication Assignment", "[matrix][core][arithmetic]") {
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

TEST_CASE("Matrix - Scalar Multiplication Assignment Zero", "[matrix][core][arithmetic]") {
    Matrix<2, 2> m = {
        {1.0f, 2.0f},
        {3.0f, 4.0f}
    };
    
    m *= 0.0f;
    
    REQUIRE(m == Matrix<2, 2>::zero());
}

TEST_CASE("Matrix - Scalar Multiplication Assignment One", "[matrix][core][arithmetic]") {
    Matrix<2, 2> m = {
        {1.0f, 2.0f},
        {3.0f, 4.0f}
    };
    Matrix<2, 2> original = m;
    
    m *= 1.0f;
    
    REQUIRE(m == original);
}

TEST_CASE("Matrix - Scalar Multiplication Assignment Negative", "[matrix][core][arithmetic]") {
    Matrix<2, 2> m = {
        {1.0f, -2.0f},
        {-3.0f, 4.0f}
    };
    
    m *= -1.0f;
    
    REQUIRE_THAT(m(0, 0), Catch::Matchers::WithinAbs(-1.0, 1e-6));
    REQUIRE_THAT(m(0, 1), Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(m(1, 0), Catch::Matchers::WithinAbs(3.0, 1e-6));
    REQUIRE_THAT(m(1, 1), Catch::Matchers::WithinAbs(-4.0, 1e-6));
}

TEST_CASE("Matrix - Scalar Division Assignment", "[matrix][core][arithmetic]") {
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

TEST_CASE("Matrix - Scalar Division Assignment By One", "[matrix][core][arithmetic]") {
    Matrix<2, 2> m = {
        {1.0f, 2.0f},
        {3.0f, 4.0f}
    };
    Matrix<2, 2> original = m;
    
    m /= 1.0f;
    
    REQUIRE(m == original);
}

TEST_CASE("Matrix - Scalar Division Assignment Fractional", "[matrix][core][arithmetic]") {
    Matrix<2, 2> m = {
        {1.0f, 2.0f},
        {3.0f, 4.0f}
    };
    
    m /= 0.5f;
    
    REQUIRE_THAT(m(0, 0), Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(m(0, 1), Catch::Matchers::WithinAbs(4.0, 1e-6));
    REQUIRE_THAT(m(1, 0), Catch::Matchers::WithinAbs(6.0, 1e-6));
    REQUIRE_THAT(m(1, 1), Catch::Matchers::WithinAbs(8.0, 1e-6));
}

// ----------------------------------------------------------------------------
// Block Extraction Tests
// ----------------------------------------------------------------------------

TEST_CASE("Matrix - Block Extraction Top Left", "[matrix][core][block]") {
    Matrix<4, 4> m = {
        {1.0f, 2.0f, 3.0f, 4.0f},
        {5.0f, 6.0f, 7.0f, 8.0f},
        {9.0f, 10.0f, 11.0f, 12.0f},
        {13.0f, 14.0f, 15.0f, 16.0f}
    };
    
    Matrix<2, 2> block = m.block<2, 2>(0, 0);
    
    REQUIRE_THAT(block(0, 0), Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(block(0, 1), Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(block(1, 0), Catch::Matchers::WithinAbs(5.0, 1e-6));
    REQUIRE_THAT(block(1, 1), Catch::Matchers::WithinAbs(6.0, 1e-6));
}

TEST_CASE("Matrix - Block Extraction Middle", "[matrix][core][block]") {
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

TEST_CASE("Matrix - Block Extraction Bottom Right", "[matrix][core][block]") {
    Matrix<4, 4> m = {
        {1.0f, 2.0f, 3.0f, 4.0f},
        {5.0f, 6.0f, 7.0f, 8.0f},
        {9.0f, 10.0f, 11.0f, 12.0f},
        {13.0f, 14.0f, 15.0f, 16.0f}
    };
    
    Matrix<2, 2> block = m.block<2, 2>(2, 2);
    
    REQUIRE_THAT(block(0, 0), Catch::Matchers::WithinAbs(11.0, 1e-6));
    REQUIRE_THAT(block(0, 1), Catch::Matchers::WithinAbs(12.0, 1e-6));
    REQUIRE_THAT(block(1, 0), Catch::Matchers::WithinAbs(15.0, 1e-6));
    REQUIRE_THAT(block(1, 1), Catch::Matchers::WithinAbs(16.0, 1e-6));
}

TEST_CASE("Matrix - Block Extraction Single Element", "[matrix][core][block]") {
    Matrix<3, 3> m = {
        {1.0f, 2.0f, 3.0f},
        {4.0f, 5.0f, 6.0f},
        {7.0f, 8.0f, 9.0f}
    };
    
    Matrix<1, 1> block = m.block<1, 1>(1, 1);
    
    REQUIRE_THAT(block(0, 0), Catch::Matchers::WithinAbs(5.0, 1e-6));
}

TEST_CASE("Matrix - Block Extraction Full Row", "[matrix][core][block]") {
    Matrix<3, 4> m = {
        {1.0f, 2.0f, 3.0f, 4.0f},
        {5.0f, 6.0f, 7.0f, 8.0f},
        {9.0f, 10.0f, 11.0f, 12.0f}
    };
    
    Matrix<1, 4> block = m.block<1, 4>(1, 0);
    
    REQUIRE_THAT(block(0, 0), Catch::Matchers::WithinAbs(5.0, 1e-6));
    REQUIRE_THAT(block(0, 1), Catch::Matchers::WithinAbs(6.0, 1e-6));
    REQUIRE_THAT(block(0, 2), Catch::Matchers::WithinAbs(7.0, 1e-6));
    REQUIRE_THAT(block(0, 3), Catch::Matchers::WithinAbs(8.0, 1e-6));
}

TEST_CASE("Matrix - Block Extraction Full Column", "[matrix][core][block]") {
    Matrix<3, 4> m = {
        {1.0f, 2.0f, 3.0f, 4.0f},
        {5.0f, 6.0f, 7.0f, 8.0f},
        {9.0f, 10.0f, 11.0f, 12.0f}
    };
    
    Matrix<3, 1> block = m.block<3, 1>(0, 2);
    
    REQUIRE_THAT(block(0, 0), Catch::Matchers::WithinAbs(3.0, 1e-6));
    REQUIRE_THAT(block(1, 0), Catch::Matchers::WithinAbs(7.0, 1e-6));
    REQUIRE_THAT(block(2, 0), Catch::Matchers::WithinAbs(11.0, 1e-6));
}

TEST_CASE("Matrix - Block Extraction Non-Square", "[matrix][core][block]") {
    Matrix<4, 5> m = {
        {1.0f, 2.0f, 3.0f, 4.0f, 5.0f},
        {6.0f, 7.0f, 8.0f, 9.0f, 10.0f},
        {11.0f, 12.0f, 13.0f, 14.0f, 15.0f},
        {16.0f, 17.0f, 18.0f, 19.0f, 20.0f}
    };
    
    Matrix<2, 3> block = m.block<2, 3>(1, 2);
    
    REQUIRE_THAT(block(0, 0), Catch::Matchers::WithinAbs(8.0, 1e-6));
    REQUIRE_THAT(block(0, 1), Catch::Matchers::WithinAbs(9.0, 1e-6));
    REQUIRE_THAT(block(0, 2), Catch::Matchers::WithinAbs(10.0, 1e-6));
    REQUIRE_THAT(block(1, 0), Catch::Matchers::WithinAbs(13.0, 1e-6));
    REQUIRE_THAT(block(1, 1), Catch::Matchers::WithinAbs(14.0, 1e-6));
    REQUIRE_THAT(block(1, 2), Catch::Matchers::WithinAbs(15.0, 1e-6));
}

TEST_CASE("Matrix - Block Extraction Exceeds Bounds Fills Zeros", "[matrix][core][block]") {
    Matrix<2, 2> m = {
        {1.0f, 2.0f},
        {3.0f, 4.0f}
    };
    
    Matrix<3, 3> block = m.block<3, 3>(0, 0);
    
    REQUIRE_THAT(block(0, 0), Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(block(0, 1), Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(block(1, 0), Catch::Matchers::WithinAbs(3.0, 1e-6));
    REQUIRE_THAT(block(1, 1), Catch::Matchers::WithinAbs(4.0, 1e-6));
    REQUIRE_THAT(block(2, 0), Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(block(0, 2), Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(block(2, 2), Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Matrix - Block Extraction Partial Overlap", "[matrix][core][block]") {
    Matrix<3, 3> m = {
        {1.0f, 2.0f, 3.0f},
        {4.0f, 5.0f, 6.0f},
        {7.0f, 8.0f, 9.0f}
    };
    
    Matrix<2, 2> block = m.block<2, 2>(2, 2);
    
    REQUIRE_THAT(block(0, 0), Catch::Matchers::WithinAbs(9.0, 1e-6));
    REQUIRE_THAT(block(0, 1), Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(block(1, 0), Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(block(1, 1), Catch::Matchers::WithinAbs(0.0, 1e-6));
}