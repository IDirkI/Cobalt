#define _USE_MATH_DEFINES
#include <cmath>


#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "cobalt/math/algebra/complex/complex.hpp"
#include "cobalt/math/algebra/complex/complex_ops.hpp"
#include "cobalt/math/algebra/complex/complex_util.hpp"

using namespace cobalt::math::algebra;

// ================================================================================
// Complex Util Tests (complex_util_ops.hpp)
// ================================================================================

TEST_CASE("Complex - isZero True", "[complex][util][check]") {
    Complex z = Complex::zero();
    
    REQUIRE(isZero(z));
}

TEST_CASE("Complex - isZero False", "[complex][util][check]") {
    Complex z(1.0f, 0.0f);
    
    REQUIRE_FALSE(isZero(z));
}

TEST_CASE("Complex - isZero Near Zero", "[complex][util][check]") {
    Complex z(1e-6f, 1e-6f);
    
    REQUIRE(isZero(z));
}

TEST_CASE("Complex - isReal True", "[complex][util][check]") {
    Complex z(5.0f, 0.0f);
    
    REQUIRE(isReal(z));
}

TEST_CASE("Complex - isReal False", "[complex][util][check]") {
    Complex z(5.0f, 0.1f);
    
    REQUIRE_FALSE(isReal(z));
}

TEST_CASE("Complex - isReal Zero", "[complex][util][check]") {
    Complex z = Complex::zero();
    
    REQUIRE(isReal(z));
}

TEST_CASE("Complex - isImag True", "[complex][util][check]") {
    Complex z(0.0f, 5.0f);
    
    REQUIRE(isImag(z));
}

TEST_CASE("Complex - isImag False Real Part", "[complex][util][check]") {
    Complex z(0.1f, 5.0f);
    
    REQUIRE_FALSE(isImag(z));
}

TEST_CASE("Complex - isImag False Zero Imaginary", "[complex][util][check]") {
    Complex z(0.0f, 0.0f);
    
    REQUIRE_FALSE(isImag(z));
}

TEST_CASE("Complex - isUnit True", "[complex][util][check]") {
    Complex z = Complex::polar(1.0f, M_PI / 4.0f);
    
    REQUIRE(isUnit(z));
}

TEST_CASE("Complex - isUnit False", "[complex][util][check]") {
    Complex z(2.0f, 0.0f);
    
    REQUIRE_FALSE(isUnit(z));
}

TEST_CASE("Complex - isUnit 3-4-5 False", "[complex][util][check]") {
    Complex z(3.0f, 4.0f);
    
    REQUIRE_FALSE(isUnit(z));
}

TEST_CASE("Complex - isConj True", "[complex][util][check]") {
    Complex z(2.0f, 3.0f);
    Complex w(2.0f, -3.0f);
    
    REQUIRE(isConjugate(z, w));
}

TEST_CASE("Complex - isConj False", "[complex][util][check]") {
    Complex z(2.0f, 3.0f);
    Complex w(2.0f, 3.0f);
    
    REQUIRE_FALSE(isConjugate(z, w));
}

TEST_CASE("Complex - isConj Real Number", "[complex][util][check]") {
    Complex z(5.0f, 0.0f);
    Complex w(5.0f, 0.0f);
    
    REQUIRE(isConjugate(z, w));
}

TEST_CASE("Complex - toPolar Positive Real", "[complex][util][convert]") {
    Complex z(5.0f, 0.0f);
    float r, theta;
    
    toPolar(z, r, theta);
    
    REQUIRE_THAT(r, Catch::Matchers::WithinAbs(5.0f, COMPLEX_EQUAL_THRESHOLD));
    REQUIRE_THAT(theta, Catch::Matchers::WithinAbs(0.0f, COMPLEX_EQUAL_THRESHOLD));
}

TEST_CASE("Complex - toPolar 3-4-5 Triangle", "[complex][util][convert]") {
    Complex z(3.0f, 4.0f);
    float r, theta;
    
    toPolar(z, r, theta);
    
    REQUIRE_THAT(r, Catch::Matchers::WithinAbs(5.0f, COMPLEX_EQUAL_THRESHOLD));
    REQUIRE_THAT(theta, Catch::Matchers::WithinAbs(std::atan2(4.0f, 3.0f), COMPLEX_EQUAL_THRESHOLD));
}

TEST_CASE("Complex - toPolar 45 Degrees", "[complex][util][convert]") {
    Complex z(1.0f, 1.0f);
    float r, theta;
    
    toPolar(z, r, theta);
    
    REQUIRE_THAT(r, Catch::Matchers::WithinAbs(std::sqrt(2.0f), COMPLEX_EQUAL_THRESHOLD));
    REQUIRE_THAT(theta, Catch::Matchers::WithinAbs(M_PI / 4.0f, COMPLEX_EQUAL_THRESHOLD));
}

TEST_CASE("Complex - clampMagnitude No Clamp Needed", "[complex][util][clamp]") {
    Complex z(3.0f, 4.0f);
    
    Complex result = clampMagnitude(z, 10.0f);
    
    REQUIRE_THAT(result.real(), Catch::Matchers::WithinAbs(3.0f, COMPLEX_EQUAL_THRESHOLD));
    REQUIRE_THAT(result.imag(), Catch::Matchers::WithinAbs(4.0f, COMPLEX_EQUAL_THRESHOLD));
}

TEST_CASE("Complex - clampMagnitude Clamp Applied", "[complex][util][clamp]") {
    Complex z(3.0f, 4.0f);  // Magnitude = 5
    
    Complex result = clampMagnitude(z, 2.5f);
    
    // Should scale to magnitude 2.5
    float mag = std::sqrt(result.real()*result.real() + result.imag()*result.imag());
    REQUIRE_THAT(mag, Catch::Matchers::WithinAbs(2.5f, COMPLEX_EQUAL_THRESHOLD));
    
    // Should maintain angle
    float originalAngle = std::atan2(4.0f, 3.0f);
    float resultAngle = std::atan2(result.imag(), result.real());
    REQUIRE_THAT(resultAngle, Catch::Matchers::WithinAbs(originalAngle, COMPLEX_EQUAL_THRESHOLD));
}

TEST_CASE("Complex - round Basic", "[complex][util][round]") {
    Complex z(2.7f, 3.3f);
    
    Complex result = round(z);
    
    REQUIRE_THAT(result.real(), Catch::Matchers::WithinAbs(3.0f, COMPLEX_EQUAL_THRESHOLD));
    REQUIRE_THAT(result.imag(), Catch::Matchers::WithinAbs(3.0f, COMPLEX_EQUAL_THRESHOLD));
}

TEST_CASE("Complex - floor Basic", "[complex][util][round]") {
    Complex z(2.7f, 3.3f);
    
    Complex result = floor(z);
    
    REQUIRE_THAT(result.real(), Catch::Matchers::WithinAbs(2.0f, COMPLEX_EQUAL_THRESHOLD));
    REQUIRE_THAT(result.imag(), Catch::Matchers::WithinAbs(3.0f, COMPLEX_EQUAL_THRESHOLD));
}

TEST_CASE("Complex - ceil Basic", "[complex][util][round]") {
    Complex z(2.3f, 3.7f);
    
    Complex result = ceil(z);
    
    REQUIRE_THAT(result.real(), Catch::Matchers::WithinAbs(3.0f, COMPLEX_EQUAL_THRESHOLD));
    REQUIRE_THAT(result.imag(), Catch::Matchers::WithinAbs(4.0f, COMPLEX_EQUAL_THRESHOLD));
}

TEST_CASE("Complex - cleanZero Both Below Threshold", "[complex][util][round]") {
    Complex z(1e-6f, 1e-7f);
    
    Complex result = cleanZero(z);
    
    REQUIRE_THAT(result.real(), Catch::Matchers::WithinAbs(0.0f, COMPLEX_EQUAL_THRESHOLD));
    REQUIRE_THAT(result.imag(), Catch::Matchers::WithinAbs(0.0f, COMPLEX_EQUAL_THRESHOLD));
}

TEST_CASE("Complex - cleanZero One Above Threshold", "[complex][util][round]") {
    Complex z(1e-3f, 1e-7f);
    
    Complex result = cleanZero(z);
    
    REQUIRE_THAT(result.real(), Catch::Matchers::WithinAbs(1e-3f, COMPLEX_EQUAL_THRESHOLD));
    REQUIRE_THAT(result.imag(), Catch::Matchers::WithinAbs(0.0f, COMPLEX_EQUAL_THRESHOLD));
}

TEST_CASE("Complex - distance Same Point", "[complex][util][distance]") {
    Complex z(2.0f, 3.0f);
    Complex w(2.0f, 3.0f);
    
    float result = distance(z, w);
    
    REQUIRE_THAT(result, Catch::Matchers::WithinAbs(0.0f, COMPLEX_EQUAL_THRESHOLD));
}

TEST_CASE("Complex - distance 3-4-5 Triangle", "[complex][util][distance]") {
    Complex z(0.0f, 0.0f);
    Complex w(3.0f, 4.0f);
    
    float result = distance(z, w);
    
    REQUIRE_THAT(result, Catch::Matchers::WithinAbs(5.0f, COMPLEX_EQUAL_THRESHOLD));
}

TEST_CASE("Complex - lerp Start", "[complex][util][interp]") {
    Complex z(1.0f, 2.0f);
    Complex w(5.0f, 6.0f);
    
    Complex result = lerp(z, w, 0.0f);
    
    REQUIRE_THAT(result.real(), Catch::Matchers::WithinAbs(z.real(), COMPLEX_EQUAL_THRESHOLD));
    REQUIRE_THAT(result.imag(), Catch::Matchers::WithinAbs(z.imag(), COMPLEX_EQUAL_THRESHOLD));
}

TEST_CASE("Complex - lerp End", "[complex][util][interp]") {
    Complex z(1.0f, 2.0f);
    Complex w(5.0f, 6.0f);
    
    Complex result = lerp(z, w, 1.0f);
    
    REQUIRE_THAT(result.real(), Catch::Matchers::WithinAbs(w.real(), COMPLEX_EQUAL_THRESHOLD));
    REQUIRE_THAT(result.imag(), Catch::Matchers::WithinAbs(w.imag(), COMPLEX_EQUAL_THRESHOLD));
}

TEST_CASE("Complex - lerp Midpoint", "[complex][util][interp]") {
    Complex z(0.0f, 0.0f);
    Complex w(10.0f, 20.0f);
    
    Complex result = lerp(z, w, 0.5f);
    
    REQUIRE_THAT(result.real(), Catch::Matchers::WithinAbs(5.0f, COMPLEX_EQUAL_THRESHOLD));
    REQUIRE_THAT(result.imag(), Catch::Matchers::WithinAbs(10.0f, COMPLEX_EQUAL_THRESHOLD));
}

TEST_CASE("Complex - slerp Unit Circle", "[complex][util][interp]") {
    Complex z = Complex::polar(1.0f, 0.0f);
    Complex w = Complex::polar(1.0f, M_PI / 2.0f);
    
    Complex result = slerp(z, w, 0.5f);
    
    // Should be at 45 degrees on unit circle
    REQUIRE_THAT(norm(result), Catch::Matchers::WithinAbs(1.0f, COMPLEX_EQUAL_THRESHOLD));
    REQUIRE_THAT(arg(result), Catch::Matchers::WithinAbs(M_PI / 4.0f, COMPLEX_EQUAL_THRESHOLD));
}

TEST_CASE("Complex - nthRoots Square Roots of 1", "[complex][util][roots]") {
    Complex z = Complex::one();
    std::array<Complex, 2> roots;
    
    nthRoots(z, roots);
    
    // Should get 1 and -1
    REQUIRE_THAT(norm(roots[0]), Catch::Matchers::WithinAbs(1.0f, COMPLEX_EQUAL_THRESHOLD));
    REQUIRE_THAT(norm(roots[1]), Catch::Matchers::WithinAbs(1.0f, COMPLEX_EQUAL_THRESHOLD));
}

TEST_CASE("Complex - nthRoots Cube Roots of 1", "[complex][util][roots]") {
    Complex z = Complex::one();
    std::array<Complex, 3> roots;
    
    nthRoots(z, roots);
    
    // All should have magnitude 1
    for(int i = 0; i < 3; i++) {
        REQUIRE_THAT(norm(roots[i]), Catch::Matchers::WithinAbs(1.0f, COMPLEX_EQUAL_THRESHOLD));
    }
    
    // Should be equally spaced by 2π/3
    float angle0 = arg(roots[0]);
    float angle1 = arg(roots[1]);
    float angleDiff = angle1 - angle0;
    if(angleDiff < 0) angleDiff += 2.0f * M_PI;
    
    REQUIRE_THAT(angleDiff, Catch::Matchers::WithinAbs(2.0f * M_PI / 3.0f, 1e-4));
}

TEST_CASE("Complex - projectReal", "[complex][util][project]") {
    Complex z(3.0f, 4.0f);
    
    Complex result = projectReal(z);
    
    REQUIRE_THAT(result.real(), Catch::Matchers::WithinAbs(3.0f, COMPLEX_EQUAL_THRESHOLD));
    REQUIRE_THAT(result.imag(), Catch::Matchers::WithinAbs(0.0f, COMPLEX_EQUAL_THRESHOLD));
}

TEST_CASE("Complex - projectImag", "[complex][util][project]") {
    Complex z(3.0f, 4.0f);
    
    Complex result = projectImag(z);
    
    REQUIRE_THAT(result.real(), Catch::Matchers::WithinAbs(0.0f, COMPLEX_EQUAL_THRESHOLD));
    REQUIRE_THAT(result.imag(), Catch::Matchers::WithinAbs(4.0f, COMPLEX_EQUAL_THRESHOLD));
}

TEST_CASE("Complex - projectUnit", "[complex][util][project]") {
    Complex z(3.0f, 4.0f);
    
    Complex result = projectUnit(z);
    
    REQUIRE_THAT(norm(result), Catch::Matchers::WithinAbs(1.0f, COMPLEX_EQUAL_THRESHOLD));
    
    // Should preserve angle
    REQUIRE_THAT(arg(result), Catch::Matchers::WithinAbs(arg(z), COMPLEX_EQUAL_THRESHOLD));
}

TEST_CASE("Complex - projectUnit Zero", "[complex][util][project]") {
    Complex z = Complex::zero();
    
    Complex result = projectUnit(z);
    
    // Should default to 1
    REQUIRE_THAT(result.real(), Catch::Matchers::WithinAbs(1.0f, COMPLEX_EQUAL_THRESHOLD));
    REQUIRE_THAT(result.imag(), Catch::Matchers::WithinAbs(0.0f, COMPLEX_EQUAL_THRESHOLD));
}

TEST_CASE("Complex - rotate 90 Degrees", "[complex][util][rotate]") {
    Complex z(1.0f, 0.0f);
    
    Complex result = rotate(z, M_PI / 2.0f);
    
    REQUIRE_THAT(result.real(), Catch::Matchers::WithinAbs(0.0f, COMPLEX_EQUAL_THRESHOLD));
    REQUIRE_THAT(result.imag(), Catch::Matchers::WithinAbs(1.0f, COMPLEX_EQUAL_THRESHOLD));
}

TEST_CASE("Complex - rotate 180 Degrees", "[complex][util][rotate]") {
    Complex z(1.0f, 0.0f);
    
    Complex result = rotate(z, M_PI);
    
    REQUIRE_THAT(result.real(), Catch::Matchers::WithinAbs(-1.0f, COMPLEX_EQUAL_THRESHOLD));
    REQUIRE_THAT(result.imag(), Catch::Matchers::WithinAbs(0.0f, COMPLEX_EQUAL_THRESHOLD));
}

TEST_CASE("Complex - rotate90 Function", "[complex][util][rotate]") {
    Complex z(1.0f, 2.0f);
    
    Complex result = rotate90(z);
    
    // (1 + 2i) * i = i + 2i² = -2 + i
    REQUIRE_THAT(result.real(), Catch::Matchers::WithinAbs(-2.0f, COMPLEX_EQUAL_THRESHOLD));
    REQUIRE_THAT(result.imag(), Catch::Matchers::WithinAbs(1.0f, COMPLEX_EQUAL_THRESHOLD));
}

TEST_CASE("Complex - rotate180 Function", "[complex][util][rotate]") {
    Complex z(3.0f, 4.0f);
    
    Complex result = rotate180(z);
    
    REQUIRE_THAT(result.real(), Catch::Matchers::WithinAbs(-3.0f, COMPLEX_EQUAL_THRESHOLD));
    REQUIRE_THAT(result.imag(), Catch::Matchers::WithinAbs(-4.0f, COMPLEX_EQUAL_THRESHOLD));
}

TEST_CASE("Complex - rotate270 Function", "[complex][util][rotate]") {
    Complex z(1.0f, 2.0f);
    
    Complex result = rotate270(z);
    
    // (1 + 2i) * (-i) = -i - 2i² = 2 - i
    REQUIRE_THAT(result.real(), Catch::Matchers::WithinAbs(2.0f, COMPLEX_EQUAL_THRESHOLD));
    REQUIRE_THAT(result.imag(), Catch::Matchers::WithinAbs(-1.0f, COMPLEX_EQUAL_THRESHOLD));
}

TEST_CASE("Complex - Four 90 Degree Rotations", "[complex][util][rotate]") {
    Complex z(3.0f, 4.0f);
    
    Complex result = rotate90(rotate90(rotate90(rotate90(z))));
    
    // Should return to original
    REQUIRE_THAT(result.real(), Catch::Matchers::WithinAbs(z.real(), 1e-4));
    REQUIRE_THAT(result.imag(), Catch::Matchers::WithinAbs(z.imag(), 1e-4));
}

TEST_CASE("Complex - Normalize and Project Unit Equivalence", "[complex][integration]") {
    Complex z(3.0f, 4.0f);
    
    Complex projected = projectUnit(z);
    Complex normalized = z;
    float mag = norm(z);
    normalized = Complex(normalized.real() / mag, normalized.imag() / mag);
    
    REQUIRE_THAT(projected.real(), Catch::Matchers::WithinAbs(normalized.real(), COMPLEX_EQUAL_THRESHOLD));
    REQUIRE_THAT(projected.imag(), Catch::Matchers::WithinAbs(normalized.imag(), COMPLEX_EQUAL_THRESHOLD));
}

TEST_CASE("Complex - Conjugate Check Consistency", "[complex][integration]") {
    Complex z(2.0f, 3.0f);
    Complex w = conj(z);
    
    REQUIRE(isConjugate(z, w));
    REQUIRE(isConjugate(w, z));
}

TEST_CASE("Complex - Distance Symmetry", "[complex][integration]") {
    Complex z(1.0f, 2.0f);
    Complex w(4.0f, 6.0f);
    
    float d1 = distance(z, w);
    float d2 = distance(w, z);
    
    REQUIRE_THAT(d1, Catch::Matchers::WithinAbs(d2, COMPLEX_EQUAL_THRESHOLD));
}