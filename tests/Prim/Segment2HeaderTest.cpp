#include <catch2/catch_test_macros.hpp>

#include <DragonGeo/Linear/Point2.hpp>
#include <DragonGeo/Linear/UnitVector2.hpp>
#include <DragonGeo/Prim/Segment2.hpp>

using DragonGeo::Linear::Point2;
using DragonGeo::Linear::UnitVector2;
using DragonGeo::Prim::Segment2;

TEST_CASE("Segment2 AsRay and AsLine instantiate from the segment header", "[prim][segment2]") {
    const Segment2 segment{Point2{0.0, 0.0}, Point2{3.0, 0.0}};
    const auto ray = segment.AsRay();
    REQUIRE(ray.has_value());
    CHECK(ray->Origin == Point2{0.0, 0.0});
    CHECK(ray->Direction == UnitVector2::XAxis);

    const auto line = segment.AsLine();
    REQUIRE(line.has_value());
    CHECK(line->Origin == Point2{0.0, 0.0});
    CHECK(line->Direction == UnitVector2::XAxis);

    const Segment2 degenerate{Point2{1.0, 2.0}, Point2{1.0, 2.0}};
    CHECK_FALSE(degenerate.AsRay().has_value());
    CHECK_FALSE(degenerate.AsLine().has_value());
}
