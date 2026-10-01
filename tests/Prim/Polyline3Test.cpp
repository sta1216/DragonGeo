#include <catch2/catch_test_macros.hpp>

#include <limits>
#include <span>
#include <type_traits>

#include <DragonGeo/Prim/Polyline3.hpp>

using DragonGeo::Linear::Point3;
using DragonGeo::Prim::Polyline3;
using DragonGeo::Prim::Polyline3T;
using DragonGeo::Prim::Polyline3f;
using DragonGeo::Prim::Segment3;

TEST_CASE("Polyline3f is the float alias and Polyline3 is not an aggregate", "[prim][polyline3]") {
    STATIC_REQUIRE(std::is_same_v<Polyline3f, Polyline3T<float>>);
    STATIC_REQUIRE(!std::is_aggregate_v<Polyline3>);
    STATIC_REQUIRE(!std::is_aggregate_v<Polyline3f>);
}

TEST_CASE("joined segments form a Polyline3", "[prim][polyline3]") {
    const Segment3 first{Point3{0.0, 0.0, 0.0}, Point3{1.0, 0.0, 0.0}};
    const Segment3 second{Point3{1.0, 0.0, 0.0}, Point3{1.0, 2.0, 0.0}};
    const Segment3 segments[]{first, second};

    const auto polyline = Polyline3::FromSegments(segments);
    REQUIRE(polyline.has_value());
    CHECK(polyline->SegmentCount() == 2);
    CHECK(polyline->PointCount() == 3);
    CHECK(polyline->Segment(0) == first);
    CHECK(polyline->Segment(1) == second);
    CHECK(polyline->Point(0) == first.A);
    CHECK(polyline->Point(1) == first.B);
    CHECK(polyline->Point(2) == second.B);
    CHECK(polyline->IsValid());

    const auto same = Polyline3::FromSegments(segments);
    REQUIRE(same.has_value());
    CHECK(*polyline == *same);

    const Segment3 longer{Point3{1.0, 0.0, 0.0}, Point3{1.0, 3.0, 0.0}};
    const Segment3 other[]{first, longer};
    const auto different = Polyline3::FromSegments(other);
    REQUIRE(different.has_value());
    CHECK(*polyline != *different);

    const Segment3 onlyFirst[]{first};
    const auto single = Polyline3::FromSegments(onlyFirst);
    REQUIRE(single.has_value());
    CHECK(single->SegmentCount() == 1);
    CHECK(single->PointCount() == 2);
    CHECK(single->Point(0) == first.A);
    CHECK(single->Point(1) == first.B);
    CHECK(*polyline != *single);
}

TEST_CASE("a Polyline3 with a non-finite coordinate is invalid", "[prim][polyline3]") {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const Segment3 segment{Point3{nan, 0.0, 0.0}, Point3{1.0, 0.0, 0.0}};
    const Segment3 segments[]{segment};
    const auto polyline = Polyline3::FromSegments(segments);
    REQUIRE(polyline.has_value());
    CHECK_FALSE(polyline->IsValid());
}

TEST_CASE("an empty span does not form a Polyline3", "[prim][polyline3]") {
    const std::span<const Segment3> empty;
    CHECK_FALSE(Polyline3::FromSegments(empty).has_value());
}

TEST_CASE("segments whose endpoints are not exactly equal do not form a Polyline3", "[prim][polyline3]") {
    const Segment3 first{Point3{0.0, 0.0, 0.0}, Point3{1.0, 0.0, 0.0}};
    const Segment3 second{Point3{1.0 + 1e-12, 0.0, 0.0}, Point3{2.0, 0.0, 0.0}};
    const Segment3 segments[]{first, second};
    CHECK_FALSE(Polyline3::FromSegments(segments).has_value());
}
