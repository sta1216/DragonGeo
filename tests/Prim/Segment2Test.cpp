#include <catch2/catch_test_macros.hpp>

#include <limits>
#include <type_traits>

#include <DragonGeo/Prim/Segment2.hpp>

using DragonGeo::Prim::Segment2;
using DragonGeo::Prim::Segment2f;
using DragonGeo::Prim::Segment2T;
using DragonGeo::Linear::Point2;

TEST_CASE("Segment2 is an aggregate of two points", "[prim][segment2]") {
    Segment2 segment{Point2{0.0, 1.0}, Point2{2.0, 3.0}};
    CHECK(segment.A == Point2{0.0, 1.0});
    CHECK(segment.B == Point2{2.0, 3.0});
    CHECK(segment.IsValid());
    CHECK(segment == Segment2{Point2{0.0, 1.0}, Point2{2.0, 3.0}});
    CHECK(segment != Segment2{Point2{0.0, 1.0}, Point2{2.0, 4.0}});
    STATIC_REQUIRE(std::is_aggregate_v<Segment2>);
    STATIC_REQUIRE(std::is_same_v<Segment2f, Segment2T<float>>);
}

TEST_CASE("a zero-length Segment2 is valid", "[prim][segment2]") {
    const Segment2 segment{Point2{1.0, 1.0}, Point2{1.0, 1.0}};
    CHECK(segment.IsValid());
}

TEST_CASE("Segment2 with a non-finite coordinate is invalid", "[prim][segment2]") {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const Segment2 segment{Point2{nan, 0.0}, Point2{1.0, 0.0}};
    CHECK_FALSE(segment.IsValid());
}
