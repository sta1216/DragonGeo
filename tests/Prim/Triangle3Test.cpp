#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <limits>
#include <optional>
#include <type_traits>
#include <variant>

#include <DragonGeo/Linear/Interval.hpp>
#include <DragonGeo/Linear/Point3.hpp>
#include <DragonGeo/Prim/Polyline3.hpp>
#include <DragonGeo/Prim/Segment3.hpp>
#include <DragonGeo/Prim/Triangle3.hpp>

using Catch::Approx;

using DragonGeo::Linear::Interval;
using DragonGeo::Linear::Point3;
using DragonGeo::Prim::Polyline3;
using DragonGeo::Prim::Segment3;
using DragonGeo::Prim::Triangle3;
using DragonGeo::Prim::Triangle3T;
using DragonGeo::Prim::Triangle3f;

TEST_CASE("Triangle3 is an aggregate of three points", "[prim][triangle3]") {
    const Triangle3 triangle{
        Point3{0.0, 0.0, 0.0}, Point3{1.0, 0.0, 0.0}, Point3{0.0, 1.0, 0.0}};
    CHECK(triangle.A == Point3{0.0, 0.0, 0.0});
    CHECK(triangle.B == Point3{1.0, 0.0, 0.0});
    CHECK(triangle.C == Point3{0.0, 1.0, 0.0});
    CHECK(triangle.IsValid());
    CHECK(triangle == Triangle3{Point3{0.0, 0.0, 0.0}, Point3{1.0, 0.0, 0.0}, Point3{0.0, 1.0, 0.0}});
    CHECK(triangle != Triangle3{Point3{9.0, 0.0, 0.0}, Point3{1.0, 0.0, 0.0}, Point3{0.0, 1.0, 0.0}});
    CHECK(triangle != Triangle3{Point3{0.0, 0.0, 0.0}, Point3{9.0, 0.0, 0.0}, Point3{0.0, 1.0, 0.0}});
    CHECK(triangle != Triangle3{Point3{0.0, 0.0, 0.0}, Point3{1.0, 0.0, 0.0}, Point3{0.0, 9.0, 0.0}});
    STATIC_REQUIRE(std::is_aggregate_v<Triangle3>);
    STATIC_REQUIRE(std::is_same_v<Triangle3f, Triangle3T<float>>);
}

TEST_CASE("a zero-area Triangle3 is valid", "[prim][triangle3]") {
    const Triangle3 triangle{
        Point3{0.0, 0.0, 0.0}, Point3{1.0, 0.0, 0.0}, Point3{2.0, 0.0, 0.0}};
    CHECK(triangle.IsValid());
}

TEST_CASE("Triangle3 with a non-finite coordinate is invalid", "[prim][triangle3]") {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const Triangle3 triangle{
        Point3{0.0, 0.0, nan}, Point3{1.0, 0.0, 0.0}, Point3{0.0, 1.0, 0.0}};
    CHECK_FALSE(triangle.IsValid());
}

TEST_CASE("Triangle3 signed area is half the cross length with vertex order", "[prim][triangle3]") {
    const Triangle3 counterClockwise{
        Point3{0.0, 0.0, 0.0}, Point3{1.0, 0.0, 0.0}, Point3{0.0, 1.0, 0.0}};
    const Triangle3 clockwise{
        Point3{0.0, 0.0, 0.0}, Point3{0.0, 1.0, 0.0}, Point3{1.0, 0.0, 0.0}};
    const Triangle3 flat{
        Point3{0.0, 0.0, 0.0}, Point3{1.0, 0.0, 0.0}, Point3{2.0, 0.0, 0.0}};
    CHECK(counterClockwise.SignedArea() == 0.5);
    CHECK(clockwise.SignedArea() == -0.5);
    CHECK(flat.SignedArea() == 0.0);

    const Triangle3 tied{
        Point3{0.0, 0.0, 0.0}, Point3{1.0, 0.0, 0.0}, Point3{0.0, 1.0, 1.0}};
    CHECK(tied.SignedArea() == Approx(-std::sqrt(2.0) / 2.0));
    STATIC_REQUIRE(noexcept(counterClockwise.SignedArea()));
}

TEST_CASE("Triangle3 contains a coplanar interior point only", "[prim][triangle3]") {
    const Triangle3 triangle{
        Point3{0.0, 0.0, 0.0}, Point3{1.0, 0.0, 0.0}, Point3{0.0, 1.0, 0.0}};
    CHECK(triangle.Contains(Point3{0.2, 0.2, 0.0}));
    CHECK(triangle.Contains(Point3{0.0, 0.0, 0.0}));
    CHECK_FALSE(triangle.Contains(Point3{0.2, 0.2, 1.0}));
    CHECK_FALSE(triangle.Contains(Point3{1.0, 1.0, 0.0}));

    const Triangle3 onYZ{
        Point3{0.0, 0.0, 0.0}, Point3{0.0, 1.0, 0.0}, Point3{0.0, 0.0, 1.0}};
    CHECK(onYZ.Contains(Point3{0.0, 0.2, 0.2}));
    CHECK_FALSE(onYZ.Contains(Point3{1.0, 0.2, 0.2}));
    CHECK_FALSE(onYZ.Contains(Point3{0.0, 1.0, 1.0}));

    const Triangle3 alongX{
        Point3{0.0, 0.0, 0.0}, Point3{1.0, 0.0, 0.0}, Point3{2.0, 0.0, 0.0}};
    CHECK(alongX.Contains(Point3{0.5, 0.0, 0.0}));
    CHECK_FALSE(alongX.Contains(Point3{3.0, 0.0, 0.0}));
    CHECK_FALSE(alongX.Contains(Point3{0.5, 1.0, 0.0}));
    CHECK_FALSE(alongX.Contains(Point3{0.5, 0.0, 1.0}));

    const Triangle3 point{Point3{5.0, 5.0, 5.0}, Point3{5.0, 5.0, 5.0}, Point3{5.0, 5.0, 5.0}};
    CHECK(point.Contains(Point3{5.0, 5.0, 5.0}));
    CHECK_FALSE(point.Contains(Point3{5.0, 5.0, 6.0}));

    STATIC_REQUIRE(requires(const Triangle3& shape, Point3 point) { shape.Contains(point); });
    STATIC_REQUIRE(noexcept(triangle.Contains(Point3{})));
}

TEST_CASE("Triangle3 parameter midpoint is on the boundary", "[prim][triangle3]") {
    constexpr Triangle3 triangle{
        Point3{0.0, 0.0, 0.0}, Point3{1.0, 0.0, 0.0}, Point3{0.0, 1.0, 0.0}};
    CHECK(triangle.Domain() == Interval{0.0, 3.0});
    CHECK(triangle.PointAt(1.5) == Point3{0.5, 0.5, 0.0});
    CHECK(triangle.MidPoint() == Point3{0.5, 0.5, 0.0});
    CHECK(triangle.PointAt(0.0) == triangle.PointAt(3.0));
    CHECK_FALSE(triangle.PointAt(4.0).has_value());
    CHECK_FALSE(triangle.PointAt(std::numeric_limits<double>::quiet_NaN()).has_value());
    STATIC_REQUIRE(triangle.Domain() == Interval{0.0, 3.0});
    STATIC_REQUIRE(triangle.MidPoint() == Point3{0.5, 0.5, 0.0});
    STATIC_REQUIRE(noexcept(triangle.Domain()));
    STATIC_REQUIRE(noexcept(triangle.PointAt(0.0)));
    STATIC_REQUIRE(noexcept(triangle.MidPoint()));
}

TEST_CASE("Triangle3 subcurve is a segment or a polyline", "[prim][triangle3]") {
    const Triangle3 triangle{
        Point3{0.0, 0.0, 0.0}, Point3{1.0, 0.0, 0.0}, Point3{0.0, 1.0, 0.0}};
    STATIC_REQUIRE(std::is_same_v<
                   decltype(triangle.Subcurve(Interval{})),
                   std::optional<std::variant<Segment3, Polyline3>>>);

    const auto firstEdge = triangle.Subcurve(Interval{0.0, 1.0});
    REQUIRE(firstEdge.has_value());
    REQUIRE(std::holds_alternative<Segment3>(*firstEdge));
    CHECK(std::get<Segment3>(*firstEdge)
          == Segment3{Point3{0.0, 0.0, 0.0}, Point3{1.0, 0.0, 0.0}});

    const auto across = triangle.Subcurve(Interval{0.5, 1.5});
    REQUIRE(across.has_value());
    REQUIRE(std::holds_alternative<Polyline3>(*across));
    const Polyline3& polyline = std::get<Polyline3>(*across);
    CHECK(polyline.SegmentCount() == 2);
    CHECK(polyline.Point(0) == Point3{0.5, 0.0, 0.0});
    CHECK(polyline.Point(1) == Point3{1.0, 0.0, 0.0});
    CHECK(polyline.Point(2) == Point3{0.5, 0.5, 0.0});
    CHECK(polyline.PointAt(0.0) == Point3{0.5, 0.0, 0.0});
    CHECK(polyline.PointAt(1.0) == Point3{1.0, 0.0, 0.0});
    CHECK(polyline.PointAt(2.0) == Point3{0.5, 0.5, 0.0});

    CHECK_FALSE(triangle.Subcurve(Interval{0.0, 4.0}).has_value());
    CHECK_FALSE(triangle.Subcurve(Interval{1.0, 1.0}).has_value());
    CHECK_FALSE(triangle.Subcurve(Interval{1.0, 0.0}).has_value());
    const double nan = std::numeric_limits<double>::quiet_NaN();
    CHECK_FALSE(triangle.Subcurve(Interval{nan, 1.0}).has_value());
}
