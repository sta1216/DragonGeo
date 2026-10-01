#include <catch2/catch_test_macros.hpp>

#include <limits>
#include <type_traits>

#include <DragonGeo/Prim/Prim.hpp>
#include <DragonGeo/Prim/Ray2.hpp>

using DragonGeo::Linear::Point2;
using DragonGeo::Linear::UnitVector2;
using DragonGeo::Linear::Vector2;
using DragonGeo::Prim::Ray2;
using DragonGeo::Prim::Ray2T;
using DragonGeo::Prim::Ray2f;

TEST_CASE("Prim.hpp includes every step-1 shell", "[prim]") {
    STATIC_REQUIRE(std::is_aggregate_v<DragonGeo::Prim::Segment2>);
    STATIC_REQUIRE(std::is_aggregate_v<DragonGeo::Prim::Segment3>);
    STATIC_REQUIRE(std::is_aggregate_v<DragonGeo::Prim::Line2>);
    STATIC_REQUIRE(std::is_aggregate_v<DragonGeo::Prim::Line3>);
    STATIC_REQUIRE(std::is_aggregate_v<DragonGeo::Prim::Ray3>);
    STATIC_REQUIRE(std::is_aggregate_v<DragonGeo::Prim::Plane>);
    STATIC_REQUIRE(std::is_aggregate_v<DragonGeo::Prim::Triangle2>);
    STATIC_REQUIRE(std::is_aggregate_v<DragonGeo::Prim::Triangle3>);
    STATIC_REQUIRE(!std::is_aggregate_v<DragonGeo::Prim::Polyline3>);
    STATIC_REQUIRE(std::is_same_v<DragonGeo::Prim::Planef, DragonGeo::Prim::PlaneT<float>>);
}

TEST_CASE("Ray2 is an aggregate of an origin and a direction", "[prim][ray2]") {
    const auto direction = UnitVector2::FromNormalizedUnchecked(Vector2{1.0, 0.0});
    const Ray2 ray{Point2{0.0, 1.0}, direction};
    CHECK(ray.Origin == Point2{0.0, 1.0});
    CHECK(ray.Direction == direction);
    CHECK(ray.IsValid());
    CHECK(ray == Ray2{Point2{0.0, 1.0}, direction});
    CHECK(ray != Ray2{Point2{0.0, 2.0}, direction});
    const auto otherDirection = UnitVector2::FromNormalizedUnchecked(Vector2{0.0, 1.0});
    CHECK(ray != Ray2{Point2{0.0, 1.0}, otherDirection});
    STATIC_REQUIRE(std::is_aggregate_v<Ray2>);
    STATIC_REQUIRE(std::is_same_v<Ray2f, Ray2T<float>>);
}

TEST_CASE("a Ray2 with a finite non-unit direction is valid", "[prim][ray2]") {
    const auto direction = UnitVector2::FromNormalizedUnchecked(Vector2{2.0, 0.0});
    const Ray2 ray{Point2{0.0, 0.0}, direction};
    CHECK(ray.IsValid());
}

TEST_CASE("Ray2 with a non-finite coordinate is invalid", "[prim][ray2]") {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const auto direction = UnitVector2::FromNormalizedUnchecked(Vector2{1.0, 0.0});
    const Ray2 nanOrigin{Point2{nan, 0.0}, direction};
    CHECK_FALSE(nanOrigin.IsValid());

    const auto nanDirection = UnitVector2::FromNormalizedUnchecked(Vector2{nan, 0.0});
    const Ray2 nanDirectionRay{Point2{0.0, 0.0}, nanDirection};
    CHECK_FALSE(nanDirectionRay.IsValid());
}
