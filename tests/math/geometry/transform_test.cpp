#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <cobalt/math/geometry/transform/transform.hpp>
#include <cobalt/math/geometry/transform/transform_ops.hpp>

#include <cobalt/math/linear_algebra/vector/vector.hpp>
#include <cobalt/math/linear_algebra/matrix/matrix.hpp>

#include "stdio.h"


using cobalt::math::geometry::Transform;
using cobalt::math::linear_algebra::Vector;
using cobalt::math::linear_algebra::Matrix;

TEST_CASE("Transform, default construction", "[transform]") {
    Transform H;

    for(int i = 0; i < 4; i++) {
        for(int j = 0; j < 4; j++) {
            if(i == j) { REQUIRE(H(i, j) == 1.0f); }
            else { REQUIRE(H(i, j) == 0.0f); }
        }
    }
}   

TEST_CASE("Transform, manual construction", "[transform]") {
    Matrix<3,3> R = Matrix<3,3>::eye();
    Vector<3> t = Vector<3>::zero();

    Transform H(R, t);

    for(int i = 0; i < 4; i++) {
        for(int j = 0; j < 4; j++) {
            if(i == j) { REQUIRE(H(i, j) == 1.0f); }
            else { REQUIRE(H(i, j) == 0.0f); }
        }
    }
}   

TEST_CASE("Transform, transform apply", "[transform]") {
    Matrix<3,3> R1= Matrix<3,3>::eye();
    Vector<3> t1{1.0f, 1.0f, 1.0f};

    Transform H1(R1, t1);
    Vector<3> v{3.0f, 4.0f, 5.0f};

    Vector<3> u1 = H1.apply(v);

    for(int i = 0; i < 3; i++) {
        REQUIRE(u1[i] == (v[i] + t1[i]));
    }
}   

TEST_CASE("Transform, transform inverse", "[transform]") {
    Matrix<3,3> R1 = Matrix<3,3>::eye();
    Vector<3> t1{1.0f, 1.0f, 1.0f};

    Matrix<3,3> R2{{0.0f, -1.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
    Vector<3> t2 = Vector<3>::zero();

    Matrix<3,3> R3{{0.0f, 0.0f, 1.0f}, {0.0f, 1.0f, 0.0f}, {-1.0f, 0.0f, 0.0f}};
    Vector<3> t3{3.0f, -2.0f, 9.74f};

    Transform H1(R1, t1);
    Transform H1inv = inv(H1);

    Transform H2(R2, t2);
    Transform H2inv = inv(H2);

    Transform H3(R3, t3);
    Transform H3inv = inv(H3);

    Transform I1 = H1inv*H1;
    Transform I2 = H2inv*H2;
    Transform I3 = H3inv*H3;
 
    for(int i = 0; i < 4; i++) {
        for(int j = 0; j < 4; j++) {
            if(i == j) { 
                REQUIRE(I1(i, j) == 1.0f);
                REQUIRE(I2(i, j) == 1.0f);
                REQUIRE(I3(i, j) == 1.0f);
            }
            else { 
                REQUIRE(I1(i, j) == 0.0f); 
                REQUIRE(I2(i, j) == 0.0f);
                REQUIRE(I3(i, j) == 0.0f);
            }
        }
    }
}   