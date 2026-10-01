#include <catch2/catch_test_macros.hpp>

#include <numbers>

#include <DragonGeo/DragonGeo.hpp>

using DragonGeo::Core::HALF_PI;
using DragonGeo::Core::PI;
using DragonGeo::Core::QUARTER_PI;
using DragonGeo::Core::SQRT_TWO;
using DragonGeo::Core::TWO_PI;

TEST_CASE("DragonGeo headers can be included and version macros are defined", "[smoke]") {
    STATIC_REQUIRE(DRAGONGEO_VERSION_MAJOR >= 0);
    STATIC_REQUIRE(DRAGONGEO_VERSION_MINOR >= 1);
    SUCCEED("DragonGeo.hpp included successfully");

    STATIC_REQUIRE(PI == std::numbers::pi);
    STATIC_REQUIRE(HALF_PI == PI / 2.0);
    STATIC_REQUIRE(TWO_PI == PI * 2.0);
    STATIC_REQUIRE(QUARTER_PI == PI / 4.0);
    STATIC_REQUIRE(SQRT_TWO == std::numbers::sqrt2);

    using DragonGeo::Linear::Point2;
    using DragonGeo::Linear::UnitVector2;
    using DragonGeo::Prim::Line2;
    using DragonGeo::Prim::Segment2;
    using DragonGeo::Query::CurveMeet;
    using DragonGeo::Query::Intersection;

    const Segment2 segment{Point2{0.0, 0.0}, Point2{1.0, 0.0}};
    CHECK(segment.Length() == 1.0);

    const Line2 horizontal{Point2{0.0, 0.0}, UnitVector2::XAxis};
    const Line2 vertical{Point2{0.0, 0.0}, UnitVector2::YAxis};
    const auto cross = Intersection(horizontal, vertical);
    CHECK(cross.Kind == CurveMeet::Point);
    CHECK(cross.Point == Point2{0.0, 0.0});
}
