#include <catch2/catch_test_macros.hpp>

#include <limits>
#include <optional>
#include <type_traits>

#include <DragonGeo/Linear/Interval.hpp>
#include <DragonGeo/Linear/Point2.hpp>
#include <DragonGeo/Prim/Segment2.hpp>
#include <DragonGeo/Prim/Triangle2.hpp>

using DragonGeo::Linear::Interval;
using DragonGeo::Linear::Point2;
using DragonGeo::Prim::Segment2;
using DragonGeo::Prim::Triangle2;
using DragonGeo::Prim::Triangle2T;
using DragonGeo::Prim::Triangle2f;

TEST_CASE("Triangle2 is an aggregate of three points", "[prim][triangle2]") {
    const Triangle2 triangle{Point2{0.0, 0.0}, Point2{1.0, 0.0}, Point2{0.0, 1.0}};
    CHECK(triangle.A == Point2{0.0, 0.0});
    CHECK(triangle.B == Point2{1.0, 0.0});
    CHECK(triangle.C == Point2{0.0, 1.0});
    CHECK(triangle.IsValid());
    CHECK(triangle == Triangle2{Point2{0.0, 0.0}, Point2{1.0, 0.0}, Point2{0.0, 1.0}});
    CHECK(triangle != Triangle2{Point2{9.0, 0.0}, Point2{1.0, 0.0}, Point2{0.0, 1.0}});
    CHECK(triangle != Triangle2{Point2{0.0, 0.0}, Point2{9.0, 0.0}, Point2{0.0, 1.0}});
    CHECK(triangle != Triangle2{Point2{0.0, 0.0}, Point2{1.0, 0.0}, Point2{9.0, 0.0}});
    STATIC_REQUIRE(std::is_aggregate_v<Triangle2>);
    STATIC_REQUIRE(std::is_same_v<Triangle2f, Triangle2T<float>>);
}

TEST_CASE("a zero-area Triangle2 is valid", "[prim][triangle2]") {
    const Triangle2 triangle{Point2{0.0, 0.0}, Point2{1.0, 0.0}, Point2{2.0, 0.0}};
    CHECK(triangle.IsValid());
}

TEST_CASE("Triangle2 with a non-finite coordinate is invalid", "[prim][triangle2]") {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const Triangle2 triangle{Point2{nan, 0.0}, Point2{1.0, 0.0}, Point2{0.0, 1.0}};
    CHECK_FALSE(triangle.IsValid());
}

TEST_CASE("Triangle2 signed area is positive counter-clockwise", "[prim][triangle2]") {
    constexpr Triangle2 counterClockwise{Point2{0.0, 0.0}, Point2{1.0, 0.0}, Point2{0.0, 1.0}};
    constexpr Triangle2 clockwise{Point2{0.0, 0.0}, Point2{0.0, 1.0}, Point2{1.0, 0.0}};
    constexpr Triangle2 flat{Point2{0.0, 0.0}, Point2{1.0, 0.0}, Point2{2.0, 0.0}};
    CHECK(counterClockwise.SignedArea() == 0.5);
    CHECK(clockwise.SignedArea() == -0.5);
    CHECK(flat.SignedArea() == 0.0);
    STATIC_REQUIRE(counterClockwise.SignedArea() == 0.5);
    STATIC_REQUIRE(clockwise.SignedArea() == -0.5);
    STATIC_REQUIRE(flat.SignedArea() == 0.0);
    STATIC_REQUIRE(noexcept(counterClockwise.SignedArea()));
}

TEST_CASE("Triangle2 contains its boundary and only double is constrained", "[prim][triangle2]") {
    const Triangle2 triangle{Point2{0.0, 0.0}, Point2{1.0, 0.0}, Point2{0.0, 1.0}};
    CHECK(triangle.Contains(Point2{0.0, 0.0}));
    CHECK(triangle.Contains(Point2{0.5, 0.0}));
    CHECK(triangle.Contains(Point2{0.2, 0.2}));
    CHECK_FALSE(triangle.Contains(Point2{1.0, 1.0}));
    CHECK_FALSE(triangle.Contains(Point2{2.0, 0.0}));

    const Triangle2 clockwise{Point2{0.0, 0.0}, Point2{0.0, 1.0}, Point2{1.0, 0.0}};
    CHECK(clockwise.Contains(Point2{0.2, 0.2}));
    CHECK_FALSE(clockwise.Contains(Point2{1.0, 1.0}));

    STATIC_REQUIRE(requires(const Triangle2& shape, Point2 point) { shape.Contains(point); });
    STATIC_REQUIRE(noexcept(triangle.Contains(Point2{})));
}

TEST_CASE("a zero-area Triangle2 contains only points on a degenerate edge", "[prim][triangle2]") {
    const Triangle2 triangle{Point2{0.0, 0.0}, Point2{1.0, 0.0}, Point2{2.0, 0.0}};
    CHECK(triangle.Contains(Point2{0.5, 0.0}));
    CHECK(triangle.Contains(Point2{0.0, 0.0}));
    CHECK_FALSE(triangle.Contains(Point2{3.0, 0.0}));
    CHECK_FALSE(triangle.Contains(Point2{0.5, 1.0}));

    const Triangle2 point{Point2{5.0, 5.0}, Point2{5.0, 5.0}, Point2{5.0, 5.0}};
    CHECK(point.Contains(Point2{5.0, 5.0}));
    CHECK_FALSE(point.Contains(Point2{5.0, 6.0}));
}

TEST_CASE("Triangle2 parameter midpoint is on the boundary", "[prim][triangle2]") {
    constexpr Triangle2 triangle{Point2{0.0, 0.0}, Point2{1.0, 0.0}, Point2{0.0, 1.0}};
    CHECK(triangle.Domain() == Interval{0.0, 3.0});
    CHECK(triangle.PointAt(0.0) == Point2{0.0, 0.0});
    CHECK(triangle.PointAt(1.0) == Point2{1.0, 0.0});
    CHECK(triangle.PointAt(1.5) == Point2{0.5, 0.5});
    CHECK(triangle.PointAt(2.0) == Point2{0.0, 1.0});
    CHECK(triangle.PointAt(3.0) == Point2{0.0, 0.0});
    CHECK(triangle.MidPoint() == Point2{0.5, 0.5});
    CHECK(triangle.MidPoint() != Point2{1.0 / 3.0, 1.0 / 3.0});
    const double nan = std::numeric_limits<double>::quiet_NaN();
    CHECK_FALSE(triangle.PointAt(-0.1).has_value());
    CHECK_FALSE(triangle.PointAt(3.1).has_value());
    CHECK_FALSE(triangle.PointAt(nan).has_value());
    CHECK_FALSE(triangle.PointAt(std::numeric_limits<double>::infinity()).has_value());

    STATIC_REQUIRE(triangle.Domain() == Interval{0.0, 3.0});
    STATIC_REQUIRE(triangle.PointAt(1.5) == Point2{0.5, 0.5});
    STATIC_REQUIRE(triangle.MidPoint() == Point2{0.5, 0.5});
    STATIC_REQUIRE(noexcept(triangle.Domain()));
    STATIC_REQUIRE(noexcept(triangle.PointAt(1.5)));
    STATIC_REQUIRE(noexcept(triangle.MidPoint()));
}

TEST_CASE("Triangle2 subcurve is a segment only on one edge", "[prim][triangle2]") {
    const Triangle2 triangle{Point2{0.0, 0.0}, Point2{1.0, 0.0}, Point2{0.0, 1.0}};
    STATIC_REQUIRE(std::is_same_v<decltype(triangle.Subcurve(Interval{})), std::optional<Segment2>>);

    const auto firstEdge = triangle.Subcurve(Interval{0.0, 1.0});
    REQUIRE(firstEdge.has_value());
    CHECK(*firstEdge == Segment2{Point2{0.0, 0.0}, Point2{1.0, 0.0}});

    const auto partial = triangle.Subcurve(Interval{0.25, 0.75});
    REQUIRE(partial.has_value());
    CHECK(*partial == Segment2{Point2{0.25, 0.0}, Point2{0.75, 0.0}});

    const auto secondEdge = triangle.Subcurve(Interval{1.0, 1.5});
    REQUIRE(secondEdge.has_value());
    CHECK(*secondEdge == Segment2{Point2{1.0, 0.0}, Point2{0.5, 0.5}});

    CHECK_FALSE(triangle.Subcurve(Interval{0.5, 1.5}).has_value());
    CHECK_FALSE(triangle.Subcurve(Interval{0.25, 0.25}).has_value());
    CHECK_FALSE(triangle.Subcurve(Interval{1.0, 0.0}).has_value());
    CHECK_FALSE(triangle.Subcurve(Interval{-0.1, 0.5}).has_value());
    CHECK_FALSE(triangle.Subcurve(Interval{2.5, 3.5}).has_value());
    const double nan = std::numeric_limits<double>::quiet_NaN();
    CHECK_FALSE(triangle.Subcurve(Interval{nan, 1.0}).has_value());
    STATIC_REQUIRE(noexcept(triangle.Subcurve(Interval{})));
}
