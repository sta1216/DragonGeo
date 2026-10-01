#include <catch2/catch_test_macros.hpp>

#include <limits>
#include <type_traits>

#include <DragonGeo/Prim/Prim.hpp>
#include <DragonGeo/Prim/Segment3.hpp>

using DragonGeo::Linear::Point3;
using DragonGeo::Prim::Segment3;
using DragonGeo::Prim::Segment3T;
using DragonGeo::Prim::Segment3f;

TEST_CASE("Prim.hpp includes Ray2", "[prim]") {
    STATIC_REQUIRE(std::is_aggregate_v<DragonGeo::Prim::Ray2>);
    STATIC_REQUIRE(std::is_same_v<DragonGeo::Prim::Ray2f, DragonGeo::Prim::Ray2T<float>>);
}

TEST_CASE("Segment3 is an aggregate of two points", "[prim][segment3]") {
    const Segment3 segment{Point3{0.0, 1.0, 2.0}, Point3{3.0, 4.0, 5.0}};
    CHECK(segment.A == Point3{0.0, 1.0, 2.0});
    CHECK(segment.B == Point3{3.0, 4.0, 5.0});
    CHECK(segment.IsValid());
    CHECK(segment == Segment3{Point3{0.0, 1.0, 2.0}, Point3{3.0, 4.0, 5.0}});
    CHECK(segment != Segment3{Point3{9.0, 1.0, 2.0}, Point3{3.0, 4.0, 5.0}});
    CHECK(segment != Segment3{Point3{0.0, 1.0, 2.0}, Point3{3.0, 4.0, 9.0}});
    STATIC_REQUIRE(std::is_aggregate_v<Segment3>);
    STATIC_REQUIRE(std::is_same_v<Segment3f, Segment3T<float>>);
}

TEST_CASE("a zero-length Segment3 is valid", "[prim][segment3]") {
    const Segment3 segment{Point3{1.0, 1.0, 1.0}, Point3{1.0, 1.0, 1.0}};
    CHECK(segment.IsValid());
}

TEST_CASE("Segment3 with a non-finite coordinate is invalid", "[prim][segment3]") {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const Segment3 segment{Point3{nan, 0.0, 0.0}, Point3{1.0, 0.0, 0.0}};
    CHECK_FALSE(segment.IsValid());
}
