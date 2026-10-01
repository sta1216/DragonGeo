#include <catch2/catch_test_macros.hpp>

#include <limits>
#include <type_traits>

#include <DragonGeo/Prim/Ray3.hpp>

using DragonGeo::Linear::Point3;
using DragonGeo::Linear::UnitVector3;
using DragonGeo::Linear::Vector3;
using DragonGeo::Prim::Ray3;
using DragonGeo::Prim::Ray3T;
using DragonGeo::Prim::Ray3f;

TEST_CASE("Ray3 is an aggregate of an origin and a direction", "[prim][ray3]") {
    const auto direction = UnitVector3::FromNormalizedUnchecked(Vector3{0.0, 0.0, 1.0});
    const Ray3 ray{Point3{0.0, 1.0, 2.0}, direction};
    CHECK(ray.Origin == Point3{0.0, 1.0, 2.0});
    CHECK(ray.Direction == direction);
    CHECK(ray.IsValid());
    CHECK(ray == Ray3{Point3{0.0, 1.0, 2.0}, direction});
    CHECK(ray != Ray3{Point3{0.0, 1.0, 9.0}, direction});
    const auto otherDirection = UnitVector3::FromNormalizedUnchecked(Vector3{1.0, 0.0, 0.0});
    CHECK(ray != Ray3{Point3{0.0, 1.0, 2.0}, otherDirection});
    STATIC_REQUIRE(std::is_aggregate_v<Ray3>);
    STATIC_REQUIRE(std::is_same_v<Ray3f, Ray3T<float>>);
}

TEST_CASE("a Ray3 with a finite non-unit direction is valid", "[prim][ray3]") {
    const auto direction = UnitVector3::FromNormalizedUnchecked(Vector3{2.0, 0.0, 0.0});
    const Ray3 ray{Point3{0.0, 0.0, 0.0}, direction};
    CHECK(ray.IsValid());
}

TEST_CASE("Ray3 with a non-finite coordinate is invalid", "[prim][ray3]") {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const auto direction = UnitVector3::FromNormalizedUnchecked(Vector3{0.0, 0.0, 1.0});
    const Ray3 nanOrigin{Point3{0.0, nan, 0.0}, direction};
    CHECK_FALSE(nanOrigin.IsValid());

    const auto nanDirection = UnitVector3::FromNormalizedUnchecked(Vector3{0.0, 0.0, nan});
    const Ray3 nanDirectionRay{Point3{0.0, 0.0, 0.0}, nanDirection};
    CHECK_FALSE(nanDirectionRay.IsValid());
}
