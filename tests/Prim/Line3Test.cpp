#include <catch2/catch_test_macros.hpp>

#include <limits>
#include <type_traits>

#include <DragonGeo/Linear/Box3.hpp>
#include <DragonGeo/Linear/Interval.hpp>
#include <DragonGeo/Linear/Transform3.hpp>
#include <DragonGeo/Prim/Line3.hpp>
#include <DragonGeo/Prim/Segment3.hpp>

using DragonGeo::Linear::Box3;
using DragonGeo::Linear::Interval;
using DragonGeo::Linear::Point3;
using DragonGeo::Linear::Transform3;
using DragonGeo::Linear::UnitVector3;
using DragonGeo::Linear::Vector3;
using DragonGeo::Prim::Line3;
using DragonGeo::Prim::Line3T;
using DragonGeo::Prim::Line3f;
using DragonGeo::Prim::Segment3;

TEST_CASE("Line3 is an aggregate of an origin and a direction", "[prim][line3]") {
    const auto direction = UnitVector3::FromNormalizedUnchecked(Vector3{0.0, 0.0, 1.0});
    const Line3 line{Point3{0.0, 1.0, 2.0}, direction};
    CHECK(line.Origin == Point3{0.0, 1.0, 2.0});
    CHECK(line.Direction == direction);
    CHECK(line.IsValid());
    CHECK(line == Line3{Point3{0.0, 1.0, 2.0}, direction});
    CHECK(line != Line3{Point3{0.0, 1.0, 9.0}, direction});
    const auto otherDirection = UnitVector3::FromNormalizedUnchecked(Vector3{1.0, 0.0, 0.0});
    CHECK(line != Line3{Point3{0.0, 1.0, 2.0}, otherDirection});
    STATIC_REQUIRE(std::is_aggregate_v<Line3>);
    STATIC_REQUIRE(std::is_same_v<Line3f, Line3T<float>>);
}

TEST_CASE("a Line3 with a finite non-unit direction is valid", "[prim][line3]") {
    const auto direction = UnitVector3::FromNormalizedUnchecked(Vector3{2.0, 0.0, 0.0});
    const Line3 line{Point3{0.0, 0.0, 0.0}, direction};
    CHECK(line.IsValid());
}

TEST_CASE("Line3 with a non-finite coordinate is invalid", "[prim][line3]") {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const auto direction = UnitVector3::FromNormalizedUnchecked(Vector3{0.0, 0.0, 1.0});
    const Line3 nanOrigin{Point3{0.0, nan, 0.0}, direction};
    CHECK_FALSE(nanOrigin.IsValid());

    const auto nanDirection = UnitVector3::FromNormalizedUnchecked(Vector3{0.0, 0.0, nan});
    const Line3 nanDirectionLine{Point3{0.0, 0.0, 0.0}, nanDirection};
    CHECK_FALSE(nanDirectionLine.IsValid());
}

TEST_CASE("Line3 has no start point", "[prim][line3]") {
    const Line3 line{Point3{0.0, 0.0, 0.0}, UnitVector3::ZAxis};
    CHECK_FALSE(line.StartPoint().has_value());
    CHECK_FALSE(line.EndPoint().has_value());
    CHECK_FALSE(line.MidPoint().has_value());
    CHECK_FALSE(line.StartTangent().has_value());
    CHECK_FALSE(line.EndTangent().has_value());
    CHECK_FALSE(line.MidTangent().has_value());
    CHECK(line.PointAt(-2.0) == Point3{0.0, 0.0, -2.0});
    CHECK_FALSE(line.PointAt(std::numeric_limits<double>::infinity()).has_value());
    CHECK(line.ClosestPoint(Point3{4.0, 0.0, 1.0}) == Point3{0.0, 0.0, 1.0});
    CHECK(line.DistanceSquared(Point3{4.0, 0.0, 1.0}) == 16.0);
    CHECK(line.Distance(Point3{4.0, 0.0, 1.0}) == 4.0);
    CHECK(line.Bounds() == Box3::Empty());
    CHECK(line.Length() == std::numeric_limits<double>::infinity());
    CHECK(line.Domain() == Interval::Unbounded());
    CHECK(line.TangentAt(1.0) == UnitVector3::ZAxis);
    CHECK(line.ParameterOf(Point3{0.0, 0.0, -2.0}) == -2.0);
    CHECK_FALSE(line.ParameterOf(Point3{4.0, 0.0, 1.0}).has_value());
    CHECK(line.ContainsPoint(Point3{0.0, 0.0, 3.0}));
    CHECK_FALSE(line.ContainsPoint(Point3{1.0, 0.0, 3.0}));
    CHECK_FALSE(line.Contains(Point3{}));
    CHECK_FALSE(line.IsClosed());
    CHECK_FALSE(line.Area().has_value());
    CHECK_FALSE(line.Orientation().has_value());
    CHECK_FALSE(line.Centroid().has_value());

    const Line3 translated = line.Translated(Vector3{1.0, 0.0, 0.0});
    CHECK(translated.Origin == Point3{1.0, 0.0, 0.0});
    const Line3 mirrored = line.Mirrored(Point3{0.0, 0.0, 0.0}, UnitVector3::ZAxis);
    CHECK(mirrored.Direction == -UnitVector3::ZAxis);
    const Line3 rotated = line.Rotated(Point3{0.0, 0.0, 0.0}, UnitVector3::YAxis, 0.0);
    CHECK(rotated == line);
    CHECK(line.Transformed(Transform3::Identity()) == line);
    CHECK_FALSE(line.Transformed(Transform3::Scaling(0.0)).has_value());
    CHECK(line.Reversed().Direction == -line.Direction);
    CHECK(line.Clone() == line);
    const auto span = line.Subcurve(Interval{-1.0, 1.0});
    REQUIRE(span.has_value());
    CHECK(*span == Segment3{Point3{0.0, 0.0, -1.0}, Point3{0.0, 0.0, 1.0}});
    CHECK_FALSE(line.Subcurve(Interval::Unbounded()).has_value());

    STATIC_REQUIRE(noexcept(line.StartPoint()));
    STATIC_REQUIRE(noexcept(line.ClosestPoint(Point3{})));
    STATIC_REQUIRE(noexcept(line.Rotated(Point3{}, UnitVector3::ZAxis, 0.0)));
    STATIC_REQUIRE(noexcept(line.Transformed(Transform3::Identity())));
    STATIC_REQUIRE(noexcept(line.Subcurve(Interval{})));
}
