#include <catch2/catch_test_macros.hpp>

#include <limits>
#include <type_traits>

#include <DragonGeo/Prim/Line3.hpp>

using DragonGeo::Linear::Point3;
using DragonGeo::Linear::UnitVector3;
using DragonGeo::Linear::Vector3;
using DragonGeo::Prim::Line3;
using DragonGeo::Prim::Line3T;
using DragonGeo::Prim::Line3f;

TEST_CASE("Line3 is an aggregate of an origin and a direction", "[prim][line3]") {
    const auto direction = UnitVector3::FromNormalizedUnchecked(Vector3{0.0, 0.0, 1.0});
    const Line3 line{Point3{0.0, 1.0, 2.0}, direction};
    CHECK(line.Origin == Point3{0.0, 1.0, 2.0});
    CHECK(line.Direction == direction);
    CHECK(line.IsValid());
    CHECK(line == Line3{Point3{0.0, 1.0, 2.0}, direction});
    CHECK(line != Line3{Point3{0.0, 1.0, 9.0}, direction});
    const auto otherDirection = UnitVector3::FromNormalizedUnchecked(Vector3{1.0, 0.0, 0.0});
    CHECK(line != Line3{Point3{0.0, 1.0, 2.0}, otherDirection});
    STATIC_REQUIRE(std::is_aggregate_v<Line3>);
    STATIC_REQUIRE(std::is_same_v<Line3f, Line3T<float>>);
}

TEST_CASE("a Line3 with a finite non-unit direction is valid", "[prim][line3]") {
    const auto direction = UnitVector3::FromNormalizedUnchecked(Vector3{2.0, 0.0, 0.0});
    const Line3 line{Point3{0.0, 0.0, 0.0}, direction};
    CHECK(line.IsValid());
}

TEST_CASE("Line3 with a non-finite coordinate is invalid", "[prim][line3]") {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const auto direction = UnitVector3::FromNormalizedUnchecked(Vector3{0.0, 0.0, 1.0});
    const Line3 nanOrigin{Point3{0.0, nan, 0.0}, direction};
    CHECK_FALSE(nanOrigin.IsValid());

    const auto nanDirection = UnitVector3::FromNormalizedUnchecked(Vector3{0.0, 0.0, nan});
    const Line3 nanDirectionLine{Point3{0.0, 0.0, 0.0}, nanDirection};
    CHECK_FALSE(nanDirectionLine.IsValid());
}
