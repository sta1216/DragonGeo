#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <limits>

#include <GeoCore/core/Constants.hpp>
#include <GeoCore/linear/Quaternion.hpp>

using Catch::Approx;

using GeoCore::core::half_pi;
using GeoCore::linear::conjugate;
using GeoCore::linear::dot;
using GeoCore::linear::from_axis_angle;
using GeoCore::linear::normalize;
using GeoCore::linear::Quaternion;
using GeoCore::linear::rotate;
using GeoCore::linear::to_matrix;
using GeoCore::linear::UnitVector3;
using GeoCore::linear::Vector3;

namespace {
const UnitVector3 z_axis = UnitVector3::from_normalized_unchecked(Vector3{0.0, 0.0, 1.0});
}

TEST_CASE("default-constructed quaternion is the identity rotation",
          "[linear][quaternion]") {
    const Quaternion q{};

    CHECK(q.w == 1.0);
    CHECK(q.x == 0.0);
    CHECK(q.y == 0.0);
    CHECK(q.z == 0.0);

    // 单位旋转不改变任何向量
    CHECK(rotate(q, Vector3{1.0, 2.0, 3.0}) == Vector3{1.0, 2.0, 3.0});
}

TEST_CASE("axis-angle construction produces a unit quaternion",
          "[linear][quaternion]") {
    const Quaternion q = from_axis_angle(z_axis, half_pi);

    CHECK(q.w == Approx(std::cos(half_pi / 2.0)));
    CHECK(q.z == Approx(std::sin(half_pi / 2.0)));
    CHECK(norm(q) == Approx(1.0));
}

TEST_CASE("rotation about z by 90 degrees maps x to y",
          "[linear][quaternion]") {
    const Quaternion q = from_axis_angle(z_axis, half_pi);

    const Vector3 rotated = rotate(q, Vector3{1.0, 0.0, 0.0});
    CHECK(rotated.x == Approx(0.0).margin(1e-15));
    CHECK(rotated.y == Approx(1.0));
    CHECK(rotated.z == Approx(0.0).margin(1e-15));
}

TEST_CASE("rotation preserves length", "[linear][quaternion]") {
    const Quaternion q = from_axis_angle(z_axis, 0.7);
    const Vector3 v{3.0, 4.0, 0.0};

    CHECK(rotate(q, v).length() == Approx(v.length()));
}

TEST_CASE("conjugate undoes the rotation", "[linear][quaternion]") {
    const Quaternion q = from_axis_angle(z_axis, 0.7);
    const Vector3 v{1.0, 2.0, 3.0};

    const Vector3 there_and_back = rotate(conjugate(q), rotate(q, v));
    CHECK(there_and_back.x == Approx(v.x));
    CHECK(there_and_back.y == Approx(v.y));
    CHECK(there_and_back.z == Approx(v.z));
}

TEST_CASE("quaternion multiplication composes rotations",
          "[linear][quaternion]") {
    const Quaternion half = from_axis_angle(z_axis, half_pi / 2.0);
    const Quaternion full = half * half;

    const Vector3 rotated = rotate(full, Vector3{1.0, 0.0, 0.0});
    CHECK(rotated.x == Approx(0.0).margin(1e-15));
    CHECK(rotated.y == Approx(1.0));
}

TEST_CASE("dot of a unit quaternion with itself is 1", "[linear][quaternion]") {
    const Quaternion q = from_axis_angle(z_axis, 0.7);
    CHECK(dot(q, q) == Approx(1.0));
}

TEST_CASE("normalize rejects the zero quaternion",
          "[linear][quaternion][degenerate]") {
    const Quaternion zero{0.0, 0.0, 0.0, 0.0};

    CHECK_FALSE(normalize(zero).has_value());
    CHECK(normalize(Quaternion{}).has_value());
}

TEST_CASE("to_matrix agrees with rotate", "[linear][quaternion]") {
    const Quaternion q = from_axis_angle(z_axis, half_pi);
    const GeoCore::linear::Matrix3 m = to_matrix(q);

    const Vector3 v{1.0, 2.0, 3.0};
    const Vector3 by_matrix = m * v;
    const Vector3 by_quaternion = rotate(q, v);

    CHECK(by_matrix.x == Approx(by_quaternion.x));
    CHECK(by_matrix.y == Approx(by_quaternion.y));
    CHECK(by_matrix.z == Approx(by_quaternion.z));
}

TEST_CASE("norm and normalize handle non-finite input consistently",
          "[linear][quaternion][degenerate]") {
    const double infinity = std::numeric_limits<double>::infinity();
    const double not_a_number = std::numeric_limits<double>::quiet_NaN();

    // norm 与 Vector3T::length 语义一致：无穷输入的模长就是无穷，不是 NaN
    CHECK(norm(Quaternion{infinity, 0.0, 0.0, 0.0}) == infinity);
    CHECK(norm(Quaternion{0.0, 0.0, infinity, 0.0}) == infinity);

    // normalize 与 UnitVector::normalize / Matrix::inverse 同一条原则：
    // 绝不交出一个 has_value() 为真、内容却是 NaN 的结果。
    CHECK_FALSE(normalize(Quaternion{infinity, 0.0, 0.0, 0.0}).has_value());
    CHECK_FALSE(normalize(Quaternion{0.0, infinity, 0.0, 0.0}).has_value());
    CHECK_FALSE(normalize(Quaternion{not_a_number, 0.0, 0.0, 0.0}).has_value());
    CHECK_FALSE(normalize(Quaternion{not_a_number, not_a_number, not_a_number, not_a_number}).has_value());
}
