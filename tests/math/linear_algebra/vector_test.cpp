#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <cmath>

#include <cobalt/math/linear_algebra/vector/vector.hpp>
#include <cobalt/math/linear_algebra/vector/vector_ops.hpp>
#include <cobalt/math/linear_algebra/vector/vector_util.hpp>

using cobalt::math::linear_algebra::Vector;
using cobalt::math::linear_algebra::Matrix;

TEST_CASE("Vector, default construction", "[vector]") {
    Vector<3> v;

    REQUIRE(v[0] == Catch::Approx(0.0f));
    REQUIRE(v[1] == Catch::Approx(0.0f));
    REQUIRE(v[2] == Catch::Approx(0.0f));
}   

TEST_CASE("Vector, manual construction", "[vector]") {
    Vector<3> v(1.0f, 2.0f, 3.0f);

    REQUIRE(v[0] == Catch::Approx(1.0f));
    REQUIRE(v[1] == Catch::Approx(2.0f));
    REQUIRE(v[2] == Catch::Approx(3.0f));
}   

TEST_CASE("Vector, brace construction", "[vector]") {
    Vector<3> v{4.0f, 5.0f, 6.0f};

    REQUIRE(v[0] == Catch::Approx(4.0f));
    REQUIRE(v[1] == Catch::Approx(5.0f));
    REQUIRE(v[2] == Catch::Approx(6.0f));
}   

TEST_CASE("Vector, from/to array", "[vector]") {
    std::array<float, 4> arr = {1.0f, 2.0f, 10.0f, -2.0f};

    Vector<4> v = Vector<4>::fromArray(arr);

    REQUIRE(v[0] == Catch::Approx(1.0f));
    REQUIRE(v[1] == Catch::Approx(2.0f));
    REQUIRE(v[2] == Catch::Approx(10.0f));
    REQUIRE(v[3] == Catch::Approx(-2.0f));

    std::array<float, 4> arr2 = toArray(v);
    
    REQUIRE(arr2[0] == Catch::Approx(1.0f));
    REQUIRE(arr2[1] == Catch::Approx(2.0f));
    REQUIRE(arr2[2] == Catch::Approx(10.0f));
    REQUIRE(arr2[3] == Catch::Approx(-2.0f));
}   

TEST_CASE("Vector, skew-symmetric", "[vector]") {
    Vector<3> v = {3.0f, -1.0f, 0.3f};

    Matrix<3, 3> S = skew(v);

    REQUIRE(S(0, 1) == Catch::Approx(-0.3f));
    REQUIRE(S(0, 2) == Catch::Approx(-1.0f));
    REQUIRE(S(1, 2) == Catch::Approx(-3.0f));

    REQUIRE(S(1, 0) == Catch::Approx(0.3f));
    REQUIRE(S(2, 0) == Catch::Approx(1.0f));
    REQUIRE(S(2, 1) == Catch::Approx(3.0f));

    REQUIRE(S(0, 0) == Catch::Approx(0.0f));
    REQUIRE(S(1, 1) == Catch::Approx(0.0f));
    REQUIRE(S(2, 2) == Catch::Approx(0.0f));
}   

TEST_CASE("Vector, x,y,z accessors", "[vector]") {
    Vector<2> v{1.0f, 2.0f};
    Vector<3> u{3.0f, 4.0f, 5.0f};

    REQUIRE(v.x() == Catch::Approx(1.0f));
    REQUIRE(v.y() == Catch::Approx(2.0f));

    REQUIRE(u.x() == Catch::Approx(3.0f));
    REQUIRE(u.y() == Catch::Approx(4.0f));
    REQUIRE(u.z() == Catch::Approx(5.0f));
}   

TEST_CASE("Vector, size out-of-bounds", "[vector]") {
    Vector<3> v;

    REQUIRE(v[8] == Catch::Approx(0.0f));
    REQUIRE(v[5] == Catch::Approx(0.0f));
    REQUIRE(v[-2] == Catch::Approx(0.0f));
}   

TEST_CASE("Vector, constant directions", "[vector]") {
    Vector<3> zero = Vector<3>::zero();
    Vector<3> uX = Vector<3>::unitX();
    Vector<3> uY = Vector<3>::unitY();
    Vector<3> uZ = Vector<3>::unitZ();

    REQUIRE(zero.x() == Catch::Approx(0.0f));
    REQUIRE(zero.y() == Catch::Approx(0.0f));
    REQUIRE(zero.z() == Catch::Approx(0.0f));

    REQUIRE(uX.x() == Catch::Approx(1.0f));
    REQUIRE(uX.y() == Catch::Approx(0.0f));
    REQUIRE(uX.z() == Catch::Approx(0.0f));

    REQUIRE(uY.x() == Catch::Approx(0.0f));
    REQUIRE(uY.y() == Catch::Approx(1.0f));
    REQUIRE(uY.z() == Catch::Approx(0.0f));

    REQUIRE(uZ.x() == Catch::Approx(0.0f));
    REQUIRE(uZ.y() == Catch::Approx(0.0f));
    REQUIRE(uZ.z() == Catch::Approx(1.0f));
}

TEST_CASE("Vector, operations", "[vector]") {
    Vector<3> v{1.0f, 2.0f, 3.0f};
    Vector<3> u{4.0f, 5.0f, 6.0f};

    Vector<3> w = v + u;
    REQUIRE(w[0] == Catch::Approx(5.0f));
    REQUIRE(w[1] == Catch::Approx(7.0f));
    REQUIRE(w[2] == Catch::Approx(9.0f));

    v += u;
    REQUIRE(v[0] == Catch::Approx(5.0f));
    REQUIRE(v[1] == Catch::Approx(7.0f));
    REQUIRE(v[2] == Catch::Approx(9.0f));

    Vector<3> n = v * 2.0f;
    REQUIRE(n[0] == Catch::Approx(10.0f));
    REQUIRE(n[1] == Catch::Approx(14.0f));
    REQUIRE(n[2] == Catch::Approx(18.0f));

    v *= 0.5f;
    REQUIRE(v[0] == Catch::Approx(2.5f));
    REQUIRE(v[1] == Catch::Approx(3.5f));
    REQUIRE(v[2] == Catch::Approx(4.5f));

    v = -v;
    REQUIRE(v[0] == Catch::Approx(-2.5f));
    REQUIRE(v[1] == Catch::Approx(-3.5f));
    REQUIRE(v[2] == Catch::Approx(-4.5f));
}  

TEST_CASE("Vector, equality", "[vector]") {
    Vector<3> v{1.0f, 2.0f, 3.0f};
    Vector<3> u = v;
    Vector<3> w;

    REQUIRE(v == u);
    REQUIRE(v != w);
    REQUIRE(u != w);
}  

TEST_CASE("Vector, dot product & norm", "[vector]") {
    Vector<3> v{1.0f, 2.0f, 3.0f};
    Vector<3> u{-4.0f, -5.0f, 6.0f};

    REQUIRE(dot(v, u) == Catch::Approx(4.0f));
    REQUIRE(norm(Vector<3>(3.0f, 4.0f, 0.0f)) == Catch::Approx(5.0f));
}  

TEST_CASE("Vector, cross product", "[vector]") {
    Vector<3> v{1.0f, 0.0f, 0.0f};
    Vector<3> u{0.0f, 1.0f, 0.0f};

    Vector<3> c = cross(v, u);
    REQUIRE(c[0] == Catch::Approx(0.0f));
    REQUIRE(c[1] == Catch::Approx(0.0f));
    REQUIRE(c[2] == Catch::Approx(1.0f));
}  

TEST_CASE("Vector, hadamard product", "[vector]") {
    Vector<3> v{1.0f, 2.0f, 3.0f};
    Vector<3> u{4.0f, 5.0f, 6.0f};

    Vector<3> c = hadamard(v, u);
    REQUIRE(c[0] == Catch::Approx(4.0f));
    REQUIRE(c[1] == Catch::Approx(10.0f));
    REQUIRE(c[2] == Catch::Approx(18.0f));
}  

TEST_CASE("Vector, normalization", "[vector]") {
    Vector<3> v1{3.0f, 0.0f, 4.0f};
    Vector<3> v2{0.0000000003f, 0.0f, 0.0000000004f};

    Vector<3> n = normalize(v1);
    REQUIRE(isNormalized(n));

    REQUIRE(n[0] == Catch::Approx(0.6f));
    REQUIRE(n[1] == Catch::Approx(0.0f));
    REQUIRE(n[2] == Catch::Approx(0.8f));

    n = normalize(v2);
    REQUIRE(isNormalized(n));

    REQUIRE(n[0] == Catch::Approx(0.6f));
    REQUIRE(n[1] == Catch::Approx(0.0f));
    REQUIRE(n[2] == Catch::Approx(0.8f));
}  

TEST_CASE("Vector, distance & angle", "[vector]") {
    Vector<3> v{3.0f, 4.0f, 0.0f};
    Vector<3> u{4.0f, -3.0f, 0.0f};
    Vector<3> w{1.0f, 0.0f, 0.0f};

    REQUIRE(getDistance(v, u) == Catch::Approx(7.071067));
    REQUIRE(getDistanceSqr(v, u) == Catch::Approx(50.0f));
    REQUIRE(getAngle(v, u) == Catch::Approx(M_PI/2));

    REQUIRE(getAngle(w, Vector<3>::unitX()) == Catch::Approx(0.0f));
    REQUIRE(getAngle(w, Vector<3>::unitY()) == Catch::Approx(M_PI/2));
    REQUIRE(getAngle(w, Vector<3>::unitZ()) == Catch::Approx(M_PI/2));
}  

TEST_CASE("Vector, clamp", "[vector]") {
    Vector<3> v{3.0f, 4.0f, 2.0f};

    v = clamp(v, 2.5f, 3.5f);

    REQUIRE(v.x() == Catch::Approx(3.0f));
    REQUIRE(v.y() == Catch::Approx(3.5f));
    REQUIRE(v.z() == Catch::Approx(2.5f));
}  

TEST_CASE("Vector, sign abs & min/max", "[vector]") {
    Vector<6> v{3.0f, -4.0f, -2.0f, 6.0f, 0.0f, -1.5f};
    Vector<6> s = sign(v);

    REQUIRE(s[0] == Catch::Approx(1.0f));
    REQUIRE(s[1] == Catch::Approx(-1.0f));
    REQUIRE(s[2] == Catch::Approx(-1.0f));
    REQUIRE(s[3] == Catch::Approx(1.0f));
    REQUIRE(s[4] == Catch::Approx(0.0f));
    REQUIRE(s[5] == Catch::Approx(-1.0f));

    REQUIRE(min(v) == Catch::Approx(-4.0f));
    REQUIRE(max(v) == Catch::Approx(6.0f));

    v = abs(v);
    REQUIRE(v[0] == Catch::Approx(3.0f));
    REQUIRE(v[1] == Catch::Approx(4.0f));
    REQUIRE(v[2] == Catch::Approx(2.0f));
    REQUIRE(v[3] == Catch::Approx(6.0f));
    REQUIRE(v[4] == Catch::Approx(0.0f));
    REQUIRE(v[5] == Catch::Approx(1.5f));
} 

TEST_CASE("Vector, projection", "[vector]") {
    Vector<3> v{3.0f, 4.0f, 0.0f};
    Vector<3> vX = projectOnto(v, Vector<3>::unitX());
    Vector<3> vY = projectOnto(v, Vector<3>::unitY());

    REQUIRE(vX.x() == Catch::Approx(3.0f));
    REQUIRE(vX.y() == Catch::Approx(0.0f));
    REQUIRE(vX.z() == Catch::Approx(0.0f));

    REQUIRE(vY.x() == Catch::Approx(0.0f));
    REQUIRE(vY.y() == Catch::Approx(4.0f));
    REQUIRE(vY.z() == Catch::Approx(0.0f));
}  

TEST_CASE("Vector, sum/multiply across", "[vector]") {
    Vector<7> v{3.0f, 4.0f, 0.2f, 8.1f, -1.0f, 2.3f, -0.4f};

    REQUIRE(sumElements(v) == Catch::Approx(16.2f));
    REQUIRE(productElements(v) == Catch::Approx(17.8848f));
}  

TEST_CASE("Vector, lerp", "[vector]") {
    Vector<3> v{3.0f, 4.0f, 0.0f};
    Vector<3> u{1.0f, 1.0f, 1.0f};

    REQUIRE(lerp(v, u, 0.0f) == v);
    REQUIRE(lerp(v, u, 0.5f) == 0.5f*v + 0.5f*u);
    REQUIRE(lerp(v, u, 1.0f) == u);
}  

TEST_CASE("Vector, slerp", "[vector]") {
    Vector<3> v = Vector<3>::unitX();
    Vector<3> u = Vector<3>::unitY();

    Vector<3> result = slerp(v, u, 0.0f);
    REQUIRE(result.x() == Catch::Approx(1.0f).margin(0.001f));
    REQUIRE(result.y() == Catch::Approx(0.0f).margin(0.001f));
    REQUIRE(result.z() == Catch::Approx(0.0f).margin(0.001f));

    result = slerp(v, u, 0.5f);
    REQUIRE(norm(result) == Catch::Approx(1.0f));

    REQUIRE(result.x() == Catch::Approx(0.7071f).margin(0.001f));
    REQUIRE(result.y() == Catch::Approx(0.7071f).margin(0.001f));
    REQUIRE(result.z() == Catch::Approx(0.0f).margin(0.001f));

    result = slerp(v, u, 1.0f);
    REQUIRE(result.x() == Catch::Approx(0.0f).margin(0.001f));
    REQUIRE(result.y() == Catch::Approx(1.0f).margin(0.001f));
    REQUIRE(result.z() == Catch::Approx(0.0f).margin(0.001f));
}  