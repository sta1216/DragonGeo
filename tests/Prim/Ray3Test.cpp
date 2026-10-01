#include <catch2/catch_test_macros.hpp>

#include <limits>
#include <type_traits>

#include <DragonGeo/Linear/Box3.hpp>
#include <DragonGeo/Linear/Interval.hpp>
#include <DragonGeo/Linear/Transform3.hpp>
#include <DragonGeo/Prim/Ray3.hpp>
#include <DragonGeo/Prim/Segment3.hpp>

using DragonGeo::Linear::Box3;
using DragonGeo::Linear::Interval;
using DragonGeo::Linear::Point3;
using DragonGeo::Linear::Transform3;
using DragonGeo::Linear::UnitVector3;
using DragonGeo::Linear::Vector3;
using DragonGeo::Prim::Ray3;
using DragonGeo::Prim::Ray3T;
using DragonGeo::Prim::Ray3f;
using DragonGeo::Prim::Segment3;

TEST_CASE("Ray3 is an aggregate of an origin and a direction", "[prim][ray3]") {
    const auto direction = UnitVector3::FromNormalizedUnchecked(Vector3{0.0, 0.0, 1.0});
    const Ray3 ray{Point3{0.0, 1.0, 2.0}, direction};
    CHECK(ray.Origin == Point3{0.0, 1.0, 2.0});
    CHECK(ray.Direction == direction);
    CHECK(ray.IsValid());
    CHECK(ray == Ray3{Point3{0.0, 1.0, 2.0}, direction});
    CHECK(ray != Ray3{Point3{0.0, 1.0, 9.0}, direction});
    const auto otherDirection = UnitVector3::FromNormalizedUnchecked(Vector3{1.0, 0.0, 0.0});
    CHECK(ray != Ray3{Point3{0.0, 1.0, 2.0}, otherDirection});
    STATIC_REQUIRE(std::is_aggregate_v<Ray3>);
    STATIC_REQUIRE(std::is_same_v<Ray3f, Ray3T<float>>);
}

TEST_CASE("a Ray3 with a finite non-unit direction is valid", "[prim][ray3]") {
    const auto direction = UnitVector3::FromNormalizedUnchecked(Vector3{2.0, 0.0, 0.0});
    const Ray3 ray{Point3{0.0, 0.0, 0.0}, direction};
    CHECK(ray.IsValid());
}

TEST_CASE("Ray3 with a non-finite coordinate is invalid", "[prim][ray3]") {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const auto direction = UnitVector3::FromNormalizedUnchecked(Vector3{0.0, 0.0, 1.0});
    const Ray3 nanOrigin{Point3{0.0, nan, 0.0}, direction};
    CHECK_FALSE(nanOrigin.IsValid());

    const auto nanDirection = UnitVector3::FromNormalizedUnchecked(Vector3{0.0, 0.0, nan});
    const Ray3 nanDirectionRay{Point3{0.0, 0.0, 0.0}, nanDirection};
    CHECK_FALSE(nanDirectionRay.IsValid());
}

TEST_CASE("Ray3 rejects a negative parameter", "[prim][ray3]") {
    const Ray3 ray{Point3{0.0, 0.0, 0.0}, UnitVector3::ZAxis};
    CHECK_FALSE(ray.PointAt(-1.0).has_value());
    CHECK(ray.PointAt(0.0) == Point3{0.0, 0.0, 0.0});
    CHECK(ray.PointAt(2.0) == Point3{0.0, 0.0, 2.0});
    CHECK(ray.ClosestPoint(Point3{0.0, 0.0, -4.0}) == Point3{0.0, 0.0, 0.0});
    CHECK(ray.DistanceSquared(Point3{0.0, 0.0, -4.0}) == 16.0);
    CHECK(ray.Distance(Point3{0.0, 0.0, -4.0}) == 4.0);
    CHECK(ray.Domain().Min == 0.0);
    CHECK(ray.Length() == std::numeric_limits<double>::infinity());
    CHECK(ray.Box() == Box3::Empty());
    CHECK(ray.StartPoint() == Point3{0.0, 0.0, 0.0});
    CHECK_FALSE(ray.EndPoint().has_value());
    CHECK_FALSE(ray.MidPoint().has_value());
    CHECK(ray.StartTangent() == UnitVector3::ZAxis);
    CHECK_FALSE(ray.EndTangent().has_value());
    CHECK_FALSE(ray.MidTangent().has_value());
    CHECK(ray.TangentAt(1.0) == UnitVector3::ZAxis);
    CHECK_FALSE(ray.TangentAt(-0.5).has_value());
    CHECK(ray.ParameterOf(Point3{0.0, 0.0, 2.0}) == 2.0);
    CHECK_FALSE(ray.ParameterOf(Point3{0.0, 0.0, -4.0}).has_value());
    CHECK(ray.ContainsPoint(Point3{0.0, 0.0, 2.0}));
    CHECK_FALSE(ray.ContainsPoint(Point3{0.0, 0.0, -4.0}));
    CHECK_FALSE(ray.Contains(Point3{}));
    CHECK_FALSE(ray.IsClosed());
    CHECK_FALSE(ray.Area().has_value());
    CHECK_FALSE(ray.Orientation().has_value());
    CHECK_FALSE(ray.Centroid().has_value());

    Ray3 translated = ray;
    translated.Translate(Vector3{1.0, 0.0, 0.0});
    CHECK(translated.Origin == Point3{1.0, 0.0, 0.0});
    CHECK(translated.Direction == UnitVector3::ZAxis);
    CHECK(ray.Origin == Point3{0.0, 0.0, 0.0});
    Ray3 mirrored = ray;
    mirrored.Mirror(Point3{0.0, 0.0, 0.0}, UnitVector3::ZAxis);
    CHECK(mirrored.Direction == -UnitVector3::ZAxis);
    CHECK(ray.Direction == UnitVector3::ZAxis);
    Ray3 rotated = ray;
    rotated.Rotate(Point3{0.0, 0.0, 0.0}, UnitVector3::ZAxis, 0.0);
    CHECK(rotated.Origin == ray.Origin);
    CHECK(rotated.Direction == ray.Direction);
    Ray3 identity = ray;
    CHECK(identity.Transform(Transform3::Identity()));
    CHECK(identity == ray);
    Ray3 collapsed = ray;
    CHECK_FALSE(collapsed.Transform(Transform3::Scaling(0.0)));
    CHECK(collapsed == ray);
    Ray3 reversed = ray;
    reversed.Reverse();
    CHECK(reversed.Direction == -ray.Direction);
    CHECK(reversed.Origin == ray.Origin);
    CHECK(ray.Direction == UnitVector3::ZAxis);
    CHECK(ray.Clone() == ray);
    const auto span = ray.Subcurve(Interval{0.0, 2.0});
    REQUIRE(span.has_value());
    CHECK(*span == Segment3{Point3{0.0, 0.0, 0.0}, Point3{0.0, 0.0, 2.0}});
    CHECK_FALSE(ray.Subcurve(Interval{-1.0, 1.0}).has_value());

    STATIC_REQUIRE(noexcept(ray.PointAt(-1.0)));
    STATIC_REQUIRE(noexcept(ray.ClosestPoint(Point3{})));
    Ray3 mutableRay = ray;
    STATIC_REQUIRE(noexcept(mutableRay.Rotate(Point3{}, UnitVector3::ZAxis, 0.0)));
    STATIC_REQUIRE(noexcept(mutableRay.Mirror(Point3{}, UnitVector3::XAxis)));
    STATIC_REQUIRE(noexcept(mutableRay.Transform(Transform3::Identity())));
    STATIC_REQUIRE(noexcept(ray.Subcurve(Interval{})));
}
