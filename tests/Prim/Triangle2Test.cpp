#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <limits>
#include <optional>
#include <type_traits>

#include <DragonGeo/Core/Constants.hpp>
#include <DragonGeo/Linear/Box2.hpp>
#include <DragonGeo/Linear/Interval.hpp>
#include <DragonGeo/Linear/Point2.hpp>
#include <DragonGeo/Linear/Transform2.hpp>
#include <DragonGeo/Linear/UnitVector2.hpp>
#include <DragonGeo/Linear/Vector2.hpp>
#include <DragonGeo/Prim/Segment2.hpp>
#include <DragonGeo/Prim/Triangle2.hpp>

using Catch::Approx;

using DragonGeo::Core::HALF_PI;
using DragonGeo::Linear::Box2;
using DragonGeo::Linear::Interval;
using DragonGeo::Linear::Point2;
using DragonGeo::Linear::Point2f;
using DragonGeo::Linear::Transform2;
using DragonGeo::Linear::UnitVector2;
using DragonGeo::Linear::Vector2;
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

TEST_CASE("Triangle2 is a closed curve with perimeter and tangents", "[prim][triangle2]") {
    constexpr Triangle2 triangle{Point2{0.0, 0.0}, Point2{1.0, 0.0}, Point2{0.0, 1.0}};
    CHECK(triangle.IsClosed());
    CHECK(triangle.StartPoint() == Point2{0.0, 0.0});
    CHECK(triangle.EndPoint() == Point2{0.0, 0.0});
    CHECK(triangle.Length() == Approx(2.0 + std::sqrt(2.0)));
    CHECK(triangle.Area() == 0.5);
    CHECK(triangle.Orientation() == 1);
    CHECK(triangle.Centroid() == Point2{1.0 / 3.0, 1.0 / 3.0});
    CHECK(triangle.Bounds() == Box2::FromCorners(Point2{0.0, 0.0}, Point2{1.0, 1.0}));
    CHECK(triangle.StartTangent() == UnitVector2::XAxis);
    CHECK(triangle.EndTangent() == -UnitVector2::YAxis);
    CHECK(triangle.TangentAt(0.0) == UnitVector2::XAxis);
    CHECK(triangle.TangentAt(1.0) == triangle.MidTangent());
    CHECK(triangle.TangentAt(3.0) == -UnitVector2::YAxis);
    CHECK(triangle.TangentAt(2.0) == triangle.EndTangent());
    const auto mid = triangle.MidTangent();
    REQUIRE(mid.has_value());
    CHECK(mid->X() == Approx(-std::sqrt(0.5)));
    CHECK(mid->Y() == Approx(std::sqrt(0.5)));

    const double nan = std::numeric_limits<double>::quiet_NaN();
    CHECK_FALSE(triangle.TangentAt(-0.1).has_value());
    CHECK_FALSE(triangle.TangentAt(3.1).has_value());
    CHECK_FALSE(triangle.TangentAt(nan).has_value());

    const Triangle2 missingEdge{Point2{0.0, 0.0}, Point2{0.0, 0.0}, Point2{1.0, 0.0}};
    CHECK_FALSE(missingEdge.StartTangent().has_value());
    CHECK_FALSE(missingEdge.TangentAt(0.0).has_value());
    CHECK(missingEdge.MidTangent() == UnitVector2::XAxis);
    CHECK(missingEdge.EndTangent() == -UnitVector2::XAxis);

    STATIC_REQUIRE(triangle.IsClosed());
    STATIC_REQUIRE(triangle.StartPoint() == Point2{0.0, 0.0});
    STATIC_REQUIRE(triangle.EndPoint() == Point2{0.0, 0.0});
    STATIC_REQUIRE(triangle.Area() == 0.5);
    STATIC_REQUIRE(triangle.Orientation() == 1);
    STATIC_REQUIRE(triangle.Centroid() == Point2{1.0 / 3.0, 1.0 / 3.0});
    STATIC_REQUIRE(noexcept(triangle.Length()));
    STATIC_REQUIRE(noexcept(triangle.TangentAt(0.0)));
    STATIC_REQUIRE(noexcept(triangle.ContainsPoint(Point2{})));
    STATIC_REQUIRE(noexcept(triangle.ParameterOf(Point2{})));
}

TEST_CASE("a zero-area Triangle2 has no centroid", "[prim][triangle2]") {
    const Triangle2 flat{Point2{0.0, 0.0}, Point2{1.0, 0.0}, Point2{2.0, 0.0}};
    CHECK(flat.Area() == 0.0);
    CHECK(flat.Orientation() == 0);
    CHECK_FALSE(flat.Centroid().has_value());
    CHECK(flat.Length() == 4.0);

    const Triangle2 collapsed{Point2{0.0, 0.0}, Point2{0.0, 0.0}, Point2{0.0, 0.0}};
    CHECK(collapsed.ContainsPoint(Point2{1e-10, 0.0}));
    CHECK_FALSE(collapsed.ContainsPoint(Point2{1e-6, 0.0}));
    CHECK_FALSE(collapsed.StartTangent().has_value());
    CHECK_FALSE(collapsed.EndTangent().has_value());
    CHECK_FALSE(collapsed.MidTangent().has_value());
}

TEST_CASE("Triangle2 ContainsPoint follows the boundary, not the interior", "[prim][triangle2]") {
    const Triangle2 triangle{Point2{0.0, 0.0}, Point2{1.0, 0.0}, Point2{0.0, 1.0}};
    CHECK(triangle.ContainsPoint(Point2{0.5, 0.0}));
    CHECK_FALSE(triangle.ContainsPoint(Point2{0.2, 0.2}));
    CHECK(triangle.ParameterOf(Point2{0.5, 0.0}) == 0.5);
    CHECK(triangle.ParameterOf(Point2{0.0, 0.0}) == 0.0);
    CHECK_FALSE(triangle.ParameterOf(Point2{0.2, 0.2}).has_value());
    CHECK(triangle.ParameterOf(Point2{0.0, 0.25}) == Approx(2.75));

    CHECK(triangle.ClosestPoint(Point2{0.2, 0.2}) == Point2{0.2, 0.2});
    CHECK(triangle.Distance(Point2{0.2, 0.2}) == 0.0);
    CHECK(triangle.DistanceSquared(Point2{0.2, 0.2}) == 0.0);
    CHECK(triangle.ClosestPoint(Point2{2.0, 0.0}) == Point2{1.0, 0.0});
    CHECK(triangle.Distance(Point2{2.0, 0.0}) == 1.0);
    CHECK(triangle.DistanceSquared(Point2{2.0, 0.0}) == 1.0);

    const Triangle2f small{Point2f{0.0f, 0.0f}, Point2f{1.0f, 0.0f}, Point2f{0.0f, 1.0f}};
    CHECK(small.ClosestPoint(Point2f{0.2f, 0.2f}) == Point2f{0.2f, 0.2f});
    CHECK(small.Distance(Point2f{0.2f, 0.2f}) == 0.0f);
    CHECK(small.DistanceSquared(Point2f{0.2f, 0.2f}) == 0.0f);
    STATIC_REQUIRE(noexcept(small.ClosestPoint(Point2f{})));
    STATIC_REQUIRE(noexcept(small.Distance(Point2f{})));
    STATIC_REQUIRE(noexcept(small.DistanceSquared(Point2f{})));
}

TEST_CASE("Triangle2 reverses, mirrors, rotates and transforms", "[prim][triangle2]") {
    constexpr Triangle2 triangle{Point2{0.0, 0.0}, Point2{1.0, 0.0}, Point2{0.0, 1.0}};
    const Triangle2 reversed = triangle.Reversed();
    CHECK(reversed.Orientation() == -1);
    CHECK(reversed.StartPoint() == triangle.A);
    CHECK(reversed.EndPoint() == triangle.A);
    CHECK(reversed.B == triangle.C);
    CHECK(reversed.C == triangle.B);
    CHECK(reversed.Area() == 0.5);
    STATIC_REQUIRE(triangle.Reversed().Orientation() == -1);
    STATIC_REQUIRE(triangle.Reversed().StartPoint() == Point2{0.0, 0.0});

    const Triangle2 translated = triangle.Translated(Vector2{1.0, 0.0});
    CHECK(translated.A == Point2{1.0, 0.0});
    CHECK(translated.B == Point2{2.0, 0.0});
    CHECK(translated.C == Point2{1.0, 1.0});

    const Triangle2 mirrored = triangle.Mirrored(Point2{0.0, 0.0}, UnitVector2::YAxis);
    CHECK(mirrored.A == Point2{0.0, 0.0});
    CHECK(mirrored.B == Point2{1.0, 0.0});
    CHECK(mirrored.C == Point2{0.0, -1.0});

    const Triangle2 rotated = triangle.Rotated(Point2{0.0, 0.0}, HALF_PI);
    CHECK(rotated.A.X == Approx(0.0).margin(1e-12));
    CHECK(rotated.A.Y == Approx(0.0).margin(1e-12));
    CHECK(rotated.B.X == Approx(0.0).margin(1e-12));
    CHECK(rotated.B.Y == Approx(1.0).margin(1e-12));
    CHECK(rotated.C.X == Approx(-1.0).margin(1e-12));
    CHECK(rotated.C.Y == Approx(0.0).margin(1e-12));

    CHECK(triangle.Clone() == triangle);
    CHECK(triangle.Transformed(Transform2::Identity()) == triangle);
    const auto collapsed = triangle.Transformed(Transform2::Scaling(0.0));
    REQUIRE(collapsed.has_value());
    CHECK(collapsed->Area() == 0.0);
    const double infinity = std::numeric_limits<double>::infinity();
    CHECK_FALSE(triangle.Transformed(Transform2::Translation(Vector2{infinity, 0.0})).has_value());
    STATIC_REQUIRE(noexcept(triangle.Rotated(Point2{}, 0.0)));
    STATIC_REQUIRE(noexcept(triangle.Mirrored(Point2{}, UnitVector2::YAxis)));
    STATIC_REQUIRE(noexcept(triangle.Transformed(Transform2::Identity())));
    STATIC_REQUIRE(noexcept(triangle.Reversed()));
    STATIC_REQUIRE(noexcept(triangle.Clone()));
}
