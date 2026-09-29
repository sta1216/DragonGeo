#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <GeoCore/core/Constants.hpp>
#include <GeoCore/linear/Transform2.hpp>

using Catch::Approx;

using GeoCore::core::half_pi;
using GeoCore::linear::apply;
using GeoCore::linear::rotation_2d;
using GeoCore::linear::Transform2;
using GeoCore::linear::translation_2d;
using GeoCore::linear::Vector2;

TEST_CASE("2D translation moves positions only", "[linear][transform2]") {
    const Transform2 t = translation_2d(Vector2{5.0, 7.0});

    CHECK(apply(t, Vector2{1.0, 2.0}) == Vector2{6.0, 9.0});
    CHECK(t * Vector2{1.0, 2.0} == Vector2{1.0, 2.0});
}

TEST_CASE("2D rotation by 90 degrees maps x to y", "[linear][transform2]") {
    const Transform2 t = rotation_2d(half_pi);

    const Vector2 rotated = apply(t, Vector2{1.0, 0.0});
    CHECK(rotated.x == Approx(0.0).margin(1e-15));
    CHECK(rotated.y == Approx(1.0));
}
