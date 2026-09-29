#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <limits>

#include <GeoCore/linear/Matrix.hpp>

using Catch::Approx;

using GeoCore::core::Tolerance;
using GeoCore::linear::determinant;
using GeoCore::linear::identity;
using GeoCore::linear::inverse;
using GeoCore::linear::Matrix2;
using GeoCore::linear::Matrix3;
using GeoCore::linear::Matrix4;

TEST_CASE("determinant of a 2x2 matrix", "[linear][matrix][determinant]") {
    CHECK(determinant(Matrix2{{{1.0, 2.0}, {3.0, 4.0}}}) == Approx(-2.0));
    CHECK(determinant(identity<double, 2>()) == Approx(1.0));
    CHECK(determinant(Matrix2{{{1.0, 2.0}, {2.0, 4.0}}}) == Approx(0.0));
}

TEST_CASE("determinant of a 3x3 matrix", "[linear][matrix][determinant]") {
    const Matrix3 m{{{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}, {7.0, 8.0, 10.0}}};
    CHECK(determinant(m) == Approx(-3.0));

    CHECK(determinant(identity<double, 3>()) == Approx(1.0));
}

TEST_CASE("determinant of a singular 3x3 matrix is zero",
          "[linear][matrix][determinant][degenerate]") {
    // 第三行是前两行之和
    const Matrix3 m{{{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}, {5.0, 7.0, 9.0}}};
    CHECK(determinant(m) == Approx(0.0));
}

TEST_CASE("determinant of a 4x4 matrix", "[linear][matrix][determinant]") {
    CHECK(determinant(identity<double, 4>()) == Approx(1.0));

    // 下三角矩阵的行列式是对角线之积
    const Matrix4 lower{{
        {2.0, 0.0, 0.0, 0.0},
        {3.0, 4.0, 0.0, 0.0},
        {5.0, 6.0, 7.0, 0.0},
        {8.0, 9.0, 10.0, 11.0},
    }};
    CHECK(determinant(lower) == Approx(2.0 * 4.0 * 7.0 * 11.0));
}

TEST_CASE("inverse times original is the identity", "[linear][matrix][inverse]") {
    const Matrix2 m{{{4.0, 7.0}, {2.0, 6.0}}};
    const auto inv = inverse(m);

    REQUIRE(inv.has_value());
    const Matrix2 product = m * *inv;

    CHECK(product(0, 0) == Approx(1.0));
    CHECK(product(0, 1) == Approx(0.0));
    CHECK(product(1, 0) == Approx(0.0));
    CHECK(product(1, 1) == Approx(1.0));
}

TEST_CASE("3x3 inverse round-trips", "[linear][matrix][inverse]") {
    const Matrix3 m{{{1.0, 2.0, 3.0}, {0.0, 1.0, 4.0}, {5.0, 6.0, 0.0}}};
    const auto inv = inverse(m);

    REQUIRE(inv.has_value());
    const Matrix3 product = m * *inv;
    const Matrix3 unit = identity<double, 3>();

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            CHECK(product(i, j) == Approx(unit(i, j)).margin(1e-12));
        }
    }
}

TEST_CASE("inverse of a singular matrix is nullopt",
          "[linear][matrix][inverse][degenerate]") {
    // 第二行是第一行的两倍
    const Matrix2 singular{{{1.0, 2.0}, {2.0, 4.0}}};

    const auto inv = inverse(singular);

    // 必须是 nullopt，而不是含 inf / NaN 的矩阵
    CHECK_FALSE(inv.has_value());

    const Matrix3 singular3{{
        {1.0, 2.0, 3.0},
        {2.0, 4.0, 6.0},
        {1.0, 1.0, 1.0},
    }};
    CHECK_FALSE(inverse(singular3).has_value());
}

TEST_CASE("inverse of a matrix containing NaN does not crash",
          "[linear][matrix][inverse][degenerate]") {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const Matrix2 bad{{{nan, 1.0}, {2.0, 3.0}}};

    // 结果不作保证，但不得崩溃
    const auto inv = inverse(bad);
    (void)inv;
    SUCCEED("inverse of a NaN matrix returned without crashing");
}

TEST_CASE("an exact tolerance rejects only a genuinely singular matrix",
          "[linear][matrix][inverse][degenerate]") {
    const Tolerance exact{0.0, 0.0};

    CHECK_FALSE(inverse(Matrix2{{{1.0, 2.0}, {2.0, 4.0}}}, exact).has_value());
    CHECK(inverse(Matrix2{{{1.0, 2.0}, {2.0, 4.0000000001}}}, exact).has_value());
}
