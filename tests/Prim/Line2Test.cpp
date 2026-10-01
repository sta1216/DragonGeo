#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <limits>
#include <type_traits>

#include <DragonGeo/Core/Constants.hpp>
#include <DragonGeo/Linear/Box2.hpp>
#include <DragonGeo/Linear/Interval.hpp>
#include <DragonGeo/Linear/Transform2.hpp>
#include <DragonGeo/Prim/Line2.hpp>
#include <DragonGeo/Prim/Segment2.hpp>

using Catch::Approx;

using DragonGeo::Core::HALF_PI;
using DragonGeo::Linear::Box2;
using DragonGeo::Linear::Interval;
using DragonGeo::Linear::Point2;
using DragonGeo::Linear::Transform2;
using DragonGeo::Linear::UnitVector2;
using DragonGeo::Linear::Vector2;
using DragonGeo::Prim::Line2;
using DragonGeo::Prim::Line2T;
using DragonGeo::Prim::Line2f;
using DragonGeo::Prim::Segment2;

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

TEST_CASE("Line2 has no endpoints and does not clamp", "[prim][line2]") {
    const Line2 line{Point2{0.0, 0.0}, UnitVector2::XAxis};
    CHECK_FALSE(line.StartPoint().has_value());
    CHECK_FALSE(line.EndPoint().has_value());
    CHECK_FALSE(line.MidPoint().has_value());
    CHECK_FALSE(line.StartTangent().has_value());
    CHECK_FALSE(line.EndTangent().has_value());
    CHECK_FALSE(line.MidTangent().has_value());
    CHECK(line.ClosestPoint(Point2{1.0, 2.0}) == Point2{1.0, 0.0});
    CHECK(line.ClosestPoint(Point2{-2.0, 3.0}) == Point2{-2.0, 0.0});
    CHECK(line.DistanceSquared(Point2{1.0, 2.0}) == 4.0);
    CHECK(line.Distance(Point2{1.0, 2.0}) == 2.0);
    CHECK(line.Bounds() == Box2::Empty());
    const double infinity = std::numeric_limits<double>::infinity();
    CHECK(line.Length() == infinity);
    CHECK(line.Domain() == Interval::Unbounded());
    CHECK(line.PointAt(-2.0) == Point2{-2.0, 0.0});
    CHECK(line.PointAt(1.0) == Point2{1.0, 0.0});
    CHECK_FALSE(line.PointAt(infinity).has_value());
    CHECK(line.TangentAt(0.0) == UnitVector2::XAxis);
    CHECK(line.ParameterOf(Point2{-2.0, 0.0}) == -2.0);
    CHECK_FALSE(line.ParameterOf(Point2{1.0, 2.0}).has_value());
}

TEST_CASE("Line2 protocol copies, reflects and cuts a finite span", "[prim][line2]") {
    const Line2 line{Point2{0.0, 0.0}, UnitVector2::XAxis};
    const Line2 translated = line.Translated(Vector2{1.0, 0.0});
    CHECK(translated.Origin == Point2{1.0, 0.0});
    CHECK(translated.Direction == UnitVector2::XAxis);

    const Line2 mirrored = line.Mirrored(Point2{0.0, 0.0}, UnitVector2::YAxis);
    CHECK(mirrored.Origin == Point2{0.0, 0.0});
    CHECK(mirrored.Direction == UnitVector2::XAxis);

    const Line2 rotated = line.Rotated(Point2{0.0, 0.0}, HALF_PI);
    CHECK(rotated.Origin.X == Approx(0.0).margin(1e-12));
    CHECK(rotated.Origin.Y == Approx(0.0).margin(1e-12));
    CHECK(rotated.Direction.X() == Approx(0.0).margin(1e-12));
    CHECK(rotated.Direction.Y() == Approx(1.0).margin(1e-12));

    CHECK(line.Transformed(Transform2::Identity()) == line);
    CHECK_FALSE(line.Transformed(Transform2::Scaling(0.0)).has_value());
    const Line2 reversed = line.Reversed();
    CHECK(reversed.Origin == line.Origin);
    CHECK(reversed.Direction == -line.Direction);
    CHECK(line.Clone() == line);

    const auto span = line.Subcurve(Interval{-1.0, 1.0});
    REQUIRE(span.has_value());
    CHECK(*span == Segment2{Point2{-1.0, 0.0}, Point2{1.0, 0.0}});
    CHECK_FALSE(line.Subcurve(Interval::Unbounded()).has_value());
    CHECK_FALSE(line.Subcurve(Interval{1.0, 1.0}).has_value());

    CHECK_FALSE(line.IsClosed());
    CHECK_FALSE(line.Area().has_value());
    CHECK_FALSE(line.Orientation().has_value());
    CHECK_FALSE(line.Centroid().has_value());
    CHECK_FALSE(line.Contains(Point2{0.0, 0.0}));
    CHECK(line.ContainsPoint(Point2{4.0, 0.0}));
    CHECK_FALSE(line.ContainsPoint(Point2{4.0, 1.0}));

    STATIC_REQUIRE(noexcept(line.Domain()));
    STATIC_REQUIRE(noexcept(line.PointAt(0.0)));
    STATIC_REQUIRE(noexcept(line.ClosestPoint(Point2{})));
    STATIC_REQUIRE(noexcept(line.DistanceSquared(Point2{})));
    STATIC_REQUIRE(noexcept(line.Distance(Point2{})));
    STATIC_REQUIRE(noexcept(line.Length()));
    STATIC_REQUIRE(noexcept(line.Bounds()));
    STATIC_REQUIRE(noexcept(line.StartPoint()));
    STATIC_REQUIRE(noexcept(line.TangentAt(0.0)));
    STATIC_REQUIRE(noexcept(line.ContainsPoint(Point2{})));
    STATIC_REQUIRE(noexcept(line.ParameterOf(Point2{})));
    STATIC_REQUIRE(noexcept(line.Translated(Vector2{})));
    STATIC_REQUIRE(noexcept(line.Rotated(Point2{}, 0.0)));
    STATIC_REQUIRE(noexcept(line.Mirrored(Point2{}, UnitVector2::YAxis)));
    STATIC_REQUIRE(noexcept(line.Reversed()));
    STATIC_REQUIRE(noexcept(line.Clone()));
    STATIC_REQUIRE(noexcept(line.Transformed(Transform2::Identity())));
    STATIC_REQUIRE(noexcept(line.Subcurve(Interval{})));
}
