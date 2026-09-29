#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>

#include <GeoCore/core/Constants.hpp>
#include <GeoCore/linear/Transform3.hpp>

using Catch::Approx;

using GeoCore::core::half_pi;
using GeoCore::linear::apply;
using GeoCore::linear::inverse;
using GeoCore::linear::identity_transform;
using GeoCore::linear::rotation_3d;
using GeoCore::linear::scaling_3d;
using GeoCore::linear::Transform3;
using GeoCore::linear::translation_3d;
using GeoCore::linear::UnitVector3;
using GeoCore::linear::Vector3;

namespace {
const UnitVector3 z_axis = UnitVector3::from_normalized_unchecked(Vector3{0.0, 0.0, 1.0});
}

TEST_CASE("identity transform leaves a position unchanged", "[linear][transform3]") {
    const Transform3 t = identity_transform<double>();

    CHECK(apply(t, Vector3{1.0, 2.0, 3.0}) == Vector3{1.0, 2.0, 3.0});
}

TEST_CASE("translation moves a position but not a direction",
          "[linear][transform3]") {
    const Transform3 t = translation_3d(Vector3{10.0, 20.0, 30.0});

    // 位置被平移
    CHECK(apply(t, Vector3{1.0, 2.0, 3.0}) == Vector3{11.0, 22.0, 33.0});

    // 方向不受平移影响 —— 这正是 operator* 与 apply 的分工
    CHECK(t * Vector3{1.0, 2.0, 3.0} == Vector3{1.0, 2.0, 3.0});
}

TEST_CASE("uniform scaling scales both positions and directions",
          "[linear][transform3]") {
    const Transform3 t = scaling_3d(2.0);

    CHECK(apply(t, Vector3{1.0, 2.0, 3.0}) == Vector3{2.0, 4.0, 6.0});
    CHECK(t * Vector3{1.0, 2.0, 3.0} == Vector3{2.0, 4.0, 6.0});
}

TEST_CASE("non-uniform scaling scales each axis independently",
          "[linear][transform3]") {
    const Transform3 t = scaling_3d(Vector3{2.0, 3.0, 4.0});

    CHECK(apply(t, Vector3{1.0, 1.0, 1.0}) == Vector3{2.0, 3.0, 4.0});
}

TEST_CASE("rotation about z by 90 degrees", "[linear][transform3]") {
    const Transform3 t = rotation_3d(z_axis, half_pi);

    const Vector3 rotated = apply(t, Vector3{1.0, 0.0, 0.0});
    CHECK(rotated.x == Approx(0.0).margin(1e-15));
    CHECK(rotated.y == Approx(1.0));
    CHECK(rotated.z == Approx(0.0).margin(1e-15));
}

TEST_CASE("composition applies the right operand first", "[linear][transform3]") {
    const Transform3 move = translation_3d(Vector3{1.0, 0.0, 0.0});
    const Transform3 spin = rotation_3d(z_axis, half_pi);

    // 绕 z 的 90° 旋转在浮点下含约 1e-16 的误差，故用 margin 而非精确相等
    // 先平移再旋转： (1,0,0) -> (2,0,0) -> (0,2,0)
    const Vector3 rotate_after = apply(spin * move, Vector3{1.0, 0.0, 0.0});
    CHECK(rotate_after.x == Approx(0.0).margin(1e-15));
    CHECK(rotate_after.y == Approx(2.0));
    CHECK(rotate_after.z == Approx(0.0).margin(1e-15));

    // 先旋转再平移： (1,0,0) -> (0,1,0) -> (1,1,0)
    const Vector3 translate_after = apply(move * spin, Vector3{1.0, 0.0, 0.0});
    CHECK(translate_after.x == Approx(1.0));
    CHECK(translate_after.y == Approx(1.0));
    CHECK(translate_after.z == Approx(0.0).margin(1e-15));
}

TEST_CASE("inverse undoes the transform", "[linear][transform3]") {
    const Transform3 t = translation_3d(Vector3{5.0, -3.0, 2.0})
                       * rotation_3d(z_axis, 0.7)
                       * scaling_3d(2.0);

    const auto inv = inverse(t);

    REQUIRE(inv.has_value());
    const Vector3 original{1.0, 2.0, 3.0};
    const Vector3 round_trip = apply(*inv, apply(t, original));

    CHECK(round_trip.x == Approx(original.x).margin(1e-12));
    CHECK(round_trip.y == Approx(original.y).margin(1e-12));
    CHECK(round_trip.z == Approx(original.z).margin(1e-12));
}

TEST_CASE("inverse of a degenerate transform is nullopt",
          "[linear][transform3][degenerate]") {
    const Transform3 flatten = scaling_3d(Vector3{1.0, 0.0, 1.0});

    CHECK_FALSE(inverse(flatten).has_value());
}
