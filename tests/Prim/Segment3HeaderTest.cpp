#include <catch2/catch_test_macros.hpp>

#include <DragonGeo/Linear/Point3.hpp>
#include <DragonGeo/Linear/UnitVector3.hpp>
#include <DragonGeo/Prim/Segment3.hpp>

using DragonGeo::Linear::Point3;
using DragonGeo::Linear::UnitVector3;
using DragonGeo::Prim::Segment3;

TEST_CASE("Segment3 AsRay and AsLine instantiate from the segment header", "[prim][segment3]") {
    const Segment3 segment{Point3{0.0, 0.0, 0.0}, Point3{0.0, 0.0, 4.0}};
    const auto ray = segment.AsRay();
    REQUIRE(ray.has_value());
    CHECK(ray->Origin == Point3{0.0, 0.0, 0.0});
    CHECK(ray->Direction == UnitVector3::ZAxis);

    const auto line = segment.AsLine();
    REQUIRE(line.has_value());
    CHECK(line->Origin == Point3{0.0, 0.0, 0.0});
    CHECK(line->Direction == UnitVector3::ZAxis);

    const Segment3 degenerate{Point3{1.0, 2.0, 3.0}, Point3{1.0, 2.0, 3.0}};
    CHECK_FALSE(degenerate.AsRay().has_value());
    CHECK_FALSE(degenerate.AsLine().has_value());
}
