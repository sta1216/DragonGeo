#include <catch2/catch_test_macros.hpp>

#include <limits>
#include <type_traits>

#include <DragonGeo/Prim/Line2.hpp>

using DragonGeo::Linear::Point2;
using DragonGeo::Linear::UnitVector2;
using DragonGeo::Linear::Vector2;
using DragonGeo::Prim::Line2;
using DragonGeo::Prim::Line2T;
using DragonGeo::Prim::Line2f;

TEST_CASE("Line2 is an aggregate of an origin and a direction", "[prim][line2]") {
    const auto direction = UnitVector2::FromNormalizedUnchecked(Vector2{1.0, 0.0});
    const Line2 line{Point2{0.0, 1.0}, direction};
    CHECK(line.Origin == Point2{0.0, 1.0});
    CHECK(line.Direction == direction);
    CHECK(line.IsValid());
    CHECK(line == Line2{Point2{0.0, 1.0}, direction});
    CHECK(line != Line2{Point2{0.0, 2.0}, direction});
    const auto otherDirection = UnitVector2::FromNormalizedUnchecked(Vector2{0.0, 1.0});
    CHECK(line != Line2{Point2{0.0, 1.0}, otherDirection});
    STATIC_REQUIRE(std::is_aggregate_v<Line2>);
    STATIC_REQUIRE(std::is_same_v<Line2f, Line2T<float>>);
}

TEST_CASE("a Line2 with a finite non-unit direction is valid", "[prim][line2]") {
    const auto direction = UnitVector2::FromNormalizedUnchecked(Vector2{2.0, 0.0});
    const Line2 line{Point2{0.0, 0.0}, direction};
    CHECK(line.IsValid());
}

TEST_CASE("Line2 with a non-finite coordinate is invalid", "[prim][line2]") {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const auto direction = UnitVector2::FromNormalizedUnchecked(Vector2{1.0, 0.0});
    const Line2 nanOrigin{Point2{nan, 0.0}, direction};
    CHECK_FALSE(nanOrigin.IsValid());

    const auto nanDirection = UnitVector2::FromNormalizedUnchecked(Vector2{nan, 0.0});
    const Line2 nanDirectionLine{Point2{0.0, 0.0}, nanDirection};
    CHECK_FALSE(nanDirectionLine.IsValid());
}
