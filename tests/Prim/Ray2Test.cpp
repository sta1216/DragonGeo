#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <limits>
#include <type_traits>

#include <DragonGeo/Core/Constants.hpp>
#include <DragonGeo/Linear/Box2.hpp>
#include <DragonGeo/Linear/Interval.hpp>
#include <DragonGeo/Linear/Transform2.hpp>
#include <DragonGeo/Prim/Prim.hpp>
#include <DragonGeo/Prim/Ray2.hpp>
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
using DragonGeo::Prim::Ray2;
using DragonGeo::Prim::Ray2T;
using DragonGeo::Prim::Ray2f;
using DragonGeo::Prim::Segment2;

TEST_CASE("Prim.hpp includes every step-1 shell", "[prim]") {
    STATIC_REQUIRE(std::is_aggregate_v<DragonGeo::Prim::Segment2>);
    STATIC_REQUIRE(std::is_aggregate_v<DragonGeo::Prim::Segment3>);
    STATIC_REQUIRE(std::is_aggregate_v<DragonGeo::Prim::Line2>);
    STATIC_REQUIRE(std::is_aggregate_v<DragonGeo::Prim::Line3>);
    STATIC_REQUIRE(std::is_aggregate_v<DragonGeo::Prim::Ray3>);
    STATIC_REQUIRE(std::is_aggregate_v<DragonGeo::Prim::Plane>);
    STATIC_REQUIRE(std::is_aggregate_v<DragonGeo::Prim::Triangle2>);
    STATIC_REQUIRE(std::is_aggregate_v<DragonGeo::Prim::Triangle3>);
    STATIC_REQUIRE(!std::is_aggregate_v<DragonGeo::Prim::Polyline3>);
    STATIC_REQUIRE(std::is_same_v<DragonGeo::Prim::Planef, DragonGeo::Prim::PlaneT<float>>);
}

TEST_CASE("Ray2 is an aggregate of an origin and a direction", "[prim][ray2]") {
    const auto direction = UnitVector2::FromNormalizedUnchecked(Vector2{1.0, 0.0});
    const Ray2 ray{Point2{0.0, 1.0}, direction};
    CHECK(ray.Origin == Point2{0.0, 1.0});
    CHECK(ray.Direction == direction);
    CHECK(ray.IsValid());
    CHECK(ray == Ray2{Point2{0.0, 1.0}, direction});
    CHECK(ray != Ray2{Point2{0.0, 2.0}, direction});
    const auto otherDirection = UnitVector2::FromNormalizedUnchecked(Vector2{0.0, 1.0});
    CHECK(ray != Ray2{Point2{0.0, 1.0}, otherDirection});
    STATIC_REQUIRE(std::is_aggregate_v<Ray2>);
    STATIC_REQUIRE(std::is_same_v<Ray2f, Ray2T<float>>);
}

TEST_CASE("a Ray2 with a finite non-unit direction is valid", "[prim][ray2]") {
    const auto direction = UnitVector2::FromNormalizedUnchecked(Vector2{2.0, 0.0});
    const Ray2 ray{Point2{0.0, 0.0}, direction};
    CHECK(ray.IsValid());
}

TEST_CASE("Ray2 with a non-finite coordinate is invalid", "[prim][ray2]") {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const auto direction = UnitVector2::FromNormalizedUnchecked(Vector2{1.0, 0.0});
    const Ray2 nanOrigin{Point2{nan, 0.0}, direction};
    CHECK_FALSE(nanOrigin.IsValid());

    const auto nanDirection = UnitVector2::FromNormalizedUnchecked(Vector2{nan, 0.0});
    const Ray2 nanDirectionRay{Point2{0.0, 0.0}, nanDirection};
    CHECK_FALSE(nanDirectionRay.IsValid());
}

TEST_CASE("Ray2 rejects a negative parameter and clamps behind the origin", "[prim][ray2]") {
    const auto negativeX = -UnitVector2::XAxis;
    const Ray2 ray{Point2{0.0, 0.0}, negativeX};
    CHECK_FALSE(ray.PointAt(-1.0).has_value());
    CHECK(ray.PointAt(0.0) == Point2{0.0, 0.0});
    CHECK(ray.PointAt(2.0) == Point2{-2.0, 0.0});
    CHECK(ray.ClosestPoint(Point2{4.0, 0.0}) == Point2{0.0, 0.0});
    CHECK(ray.DistanceSquared(Point2{4.0, 0.0}) == 16.0);
    CHECK(ray.Distance(Point2{4.0, 0.0}) == 4.0);
    CHECK(ray.Domain().Min == 0.0);
    CHECK(ray.Domain().Max == std::numeric_limits<double>::infinity());
    CHECK(ray.Length() == std::numeric_limits<double>::infinity());
    CHECK(ray.Bounds() == Box2::Empty());
    CHECK(ray.StartPoint() == Point2{0.0, 0.0});
    CHECK_FALSE(ray.EndPoint().has_value());
    CHECK_FALSE(ray.MidPoint().has_value());
    CHECK(ray.StartTangent() == negativeX);
    CHECK_FALSE(ray.EndTangent().has_value());
    CHECK_FALSE(ray.MidTangent().has_value());
    CHECK(ray.TangentAt(1.0) == negativeX);
    CHECK_FALSE(ray.TangentAt(-1.0).has_value());
    CHECK(ray.ParameterOf(Point2{0.0, 0.0}) == 0.0);
    CHECK_FALSE(ray.ParameterOf(Point2{4.0, 0.0}).has_value());
    CHECK(ray.ContainsPoint(Point2{-2.0, 0.0}));
    CHECK_FALSE(ray.ContainsPoint(Point2{4.0, 0.0}));
}

TEST_CASE("Ray2 protocol keeps the origin and cuts a finite segment", "[prim][ray2]") {
    const Ray2 ray{Point2{0.0, 0.0}, UnitVector2::XAxis};
    const Ray2 translated = ray.Translated(Vector2{1.0, 0.0});
    CHECK(translated.Origin == Point2{1.0, 0.0});
    CHECK(translated.Direction == UnitVector2::XAxis);

    const Ray2 upright{Point2{0.0, 1.0}, UnitVector2::XAxis};
    const Ray2 mirrored = upright.Mirrored(Point2{0.0, 0.0}, UnitVector2::YAxis);
    CHECK(mirrored.Origin == Point2{0.0, -1.0});
    CHECK(mirrored.Direction == UnitVector2::XAxis);

    const Ray2 rotated = ray.Rotated(Point2{0.0, 0.0}, HALF_PI);
    CHECK(rotated.Origin.X == Approx(0.0).margin(1e-12));
    CHECK(rotated.Origin.Y == Approx(0.0).margin(1e-12));
    CHECK(rotated.Direction.X() == Approx(0.0).margin(1e-12));
    CHECK(rotated.Direction.Y() == Approx(1.0).margin(1e-12));

    CHECK(ray.Transformed(Transform2::Identity()) == ray);
    CHECK_FALSE(ray.Transformed(Transform2::Scaling(0.0)).has_value());
    const Ray2 reversed = ray.Reversed();
    CHECK(reversed.Origin == ray.Origin);
    CHECK(reversed.Direction == -ray.Direction);
    CHECK(ray.Clone() == ray);

    const auto span = ray.Subcurve(Interval{0.0, 2.0});
    REQUIRE(span.has_value());
    CHECK(*span == Segment2{Point2{0.0, 0.0}, Point2{2.0, 0.0}});
    CHECK_FALSE(ray.Subcurve(Interval{-1.0, 1.0}).has_value());
    CHECK_FALSE(ray.Subcurve(Interval{1.0, 1.0}).has_value());

    CHECK_FALSE(ray.IsClosed());
    CHECK_FALSE(ray.Area().has_value());
    CHECK_FALSE(ray.Orientation().has_value());
    CHECK_FALSE(ray.Centroid().has_value());
    CHECK_FALSE(ray.Contains(Point2{1.0, 0.0}));

    STATIC_REQUIRE(noexcept(ray.Domain()));
    STATIC_REQUIRE(noexcept(ray.PointAt(0.0)));
    STATIC_REQUIRE(noexcept(ray.ClosestPoint(Point2{})));
    STATIC_REQUIRE(noexcept(ray.DistanceSquared(Point2{})));
    STATIC_REQUIRE(noexcept(ray.Distance(Point2{})));
    STATIC_REQUIRE(noexcept(ray.Length()));
    STATIC_REQUIRE(noexcept(ray.Bounds()));
    STATIC_REQUIRE(noexcept(ray.StartPoint()));
    STATIC_REQUIRE(noexcept(ray.EndPoint()));
    STATIC_REQUIRE(noexcept(ray.MidPoint()));
    STATIC_REQUIRE(noexcept(ray.StartTangent()));
    STATIC_REQUIRE(noexcept(ray.EndTangent()));
    STATIC_REQUIRE(noexcept(ray.MidTangent()));
    STATIC_REQUIRE(noexcept(ray.TangentAt(0.0)));
    STATIC_REQUIRE(noexcept(ray.IsClosed()));
    STATIC_REQUIRE(noexcept(ray.Area()));
    STATIC_REQUIRE(noexcept(ray.Orientation()));
    STATIC_REQUIRE(noexcept(ray.Centroid()));
    STATIC_REQUIRE(noexcept(ray.Contains(Point2{})));
    STATIC_REQUIRE(noexcept(ray.ContainsPoint(Point2{})));
    STATIC_REQUIRE(noexcept(ray.ParameterOf(Point2{})));
    STATIC_REQUIRE(noexcept(ray.Translated(Vector2{})));
    STATIC_REQUIRE(noexcept(ray.Rotated(Point2{}, 0.0)));
    STATIC_REQUIRE(noexcept(ray.Mirrored(Point2{}, UnitVector2::YAxis)));
    STATIC_REQUIRE(noexcept(ray.Reversed()));
    STATIC_REQUIRE(noexcept(ray.Clone()));
    STATIC_REQUIRE(noexcept(ray.Transformed(Transform2::Identity())));
    STATIC_REQUIRE(noexcept(ray.Subcurve(Interval{})));
    STATIC_REQUIRE(noexcept(ray.AsLine()));
    STATIC_REQUIRE(noexcept(ray.AsSegment(1.0)));
}

TEST_CASE("Ray2 AsSegment builds a finite segment along the ray", "[prim][ray2]") {
    const Ray2 ray{Point2{0.0, 0.0}, UnitVector2::XAxis};
    const auto segment = ray.AsSegment(2.0);
    REQUIRE(segment.has_value());
    CHECK(segment->A == Point2{0.0, 0.0});
    CHECK(segment->B == Point2{2.0, 0.0});
    CHECK_FALSE(ray.AsSegment(0.0).has_value());
}

TEST_CASE("Ray2 AsLine keeps the same origin and direction", "[prim][ray2]") {
    const Ray2 ray{Point2{1.0, 2.0}, UnitVector2::YAxis};
    const Line2 line = ray.AsLine();
    CHECK(line.Origin == ray.Origin);
    CHECK(line.Direction == ray.Direction);
}
