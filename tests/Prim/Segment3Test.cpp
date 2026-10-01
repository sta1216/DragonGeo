#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <limits>
#include <type_traits>

#include <DragonGeo/Core/Constants.hpp>
#include <DragonGeo/Linear/Box3.hpp>
#include <DragonGeo/Linear/Interval.hpp>
#include <DragonGeo/Linear/Transform3.hpp>
#include <DragonGeo/Linear/UnitVector3.hpp>
#include <DragonGeo/Linear/Vector3.hpp>
#include <DragonGeo/Prim/Prim.hpp>
#include <DragonGeo/Prim/Segment3.hpp>

using Catch::Approx;

using DragonGeo::Core::HALF_PI;
using DragonGeo::Linear::Box3;
using DragonGeo::Linear::Interval;
using DragonGeo::Linear::Point3;
using DragonGeo::Linear::Transform3;
using DragonGeo::Linear::UnitVector3;
using DragonGeo::Linear::Vector3;
using DragonGeo::Prim::Ray3;
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

TEST_CASE("Segment3 midpoint is PointAt one half", "[prim][segment3]") {
    const Segment3 segment{Point3{0.0, 0.0, 0.0}, Point3{0.0, 0.0, 4.0}};
    CHECK(segment.PointAt(0.0) == Point3{0.0, 0.0, 0.0});
    CHECK(segment.PointAt(1.0) == Point3{0.0, 0.0, 4.0});
    CHECK(segment.PointAt(0.5) == Point3{0.0, 0.0, 2.0});
    CHECK(segment.MidPoint() == Point3{0.0, 0.0, 2.0});
    CHECK_FALSE(segment.PointAt(2.0).has_value());
    CHECK(segment.Domain() == Interval{0.0, 1.0});
    CHECK(segment.ClosestPoint(Point3{3.0, 0.0, 1.0}) == Point3{0.0, 0.0, 1.0});
    CHECK(segment.DistanceSquared(Point3{3.0, 0.0, 1.0}) == 9.0);
    CHECK(segment.Distance(Point3{3.0, 0.0, 1.0}) == 3.0);
    CHECK(segment.Length() == 4.0);
    CHECK(segment.LengthSquared() == 16.0);
    CHECK(segment.Direction() == UnitVector3::ZAxis);
    CHECK(segment.Box() == Box3::FromCorners(Point3{0.0, 0.0, 0.0}, Point3{0.0, 0.0, 4.0}));
    CHECK(segment.StartPoint() == Point3{0.0, 0.0, 0.0});
    CHECK(segment.EndPoint() == Point3{0.0, 0.0, 4.0});
    CHECK(segment.StartTangent() == UnitVector3::ZAxis);
    CHECK(segment.EndTangent() == UnitVector3::ZAxis);
    CHECK(segment.MidTangent() == UnitVector3::ZAxis);
    CHECK(segment.TangentAt(0.25) == UnitVector3::ZAxis);

    const Segment3 degenerate{Point3{1.0, 1.0, 1.0}, Point3{1.0, 1.0, 1.0}};
    CHECK_FALSE(degenerate.Direction().has_value());
    CHECK(degenerate.Length() == 0.0);
    CHECK(degenerate.ParameterOf(Point3{1.0, 1.0, 1.0}) == 0.0);
    CHECK_FALSE(degenerate.TangentAt(0.0).has_value());

    Segment3 reversed = segment;
    reversed.Reverse();
    CHECK(reversed.A == Point3{0.0, 0.0, 4.0});
    CHECK(reversed.B == Point3{0.0, 0.0, 0.0});
    CHECK(segment == Segment3{Point3{0.0, 0.0, 0.0}, Point3{0.0, 0.0, 4.0}});
    Segment3 translated = segment;
    translated.Translate(Vector3{1.0, 0.0, 0.0});
    CHECK(translated.A == Point3{1.0, 0.0, 0.0});
    CHECK(translated.B == Point3{1.0, 0.0, 4.0});

    const Segment3 above{Point3{0.0, 0.0, 1.0}, Point3{2.0, 0.0, 1.0}};
    Segment3 mirrored = above;
    mirrored.Mirror(Point3{0.0, 0.0, 0.0}, UnitVector3::ZAxis);
    CHECK(mirrored.A == Point3{0.0, 0.0, -1.0});
    CHECK(mirrored.B == Point3{2.0, 0.0, -1.0});
    CHECK(above == Segment3{Point3{0.0, 0.0, 1.0}, Point3{2.0, 0.0, 1.0}});

    const Segment3 onAxis{Point3{1.0, 0.0, 0.0}, Point3{2.0, 0.0, 0.0}};
    Segment3 rotated = onAxis;
    rotated.Rotate(Point3{0.0, 0.0, 0.0}, UnitVector3::ZAxis, HALF_PI);
    CHECK(rotated.A.X == Approx(0.0).margin(1e-12));
    CHECK(rotated.A.Y == Approx(1.0).margin(1e-12));
    CHECK(rotated.A.Z == Approx(0.0).margin(1e-12));
    CHECK(onAxis == Segment3{Point3{1.0, 0.0, 0.0}, Point3{2.0, 0.0, 0.0}});

    Segment3 identity = segment;
    CHECK(identity.Transform(Transform3::Identity()));
    CHECK(identity == segment);
    Segment3 collapsed = segment;
    CHECK_FALSE(collapsed.Transform(Transform3::Scaling(0.0)));
    CHECK(collapsed == segment);
    CHECK(segment.Clone() == segment);
    CHECK(segment.ContainsPoint(Point3{0.0, 0.0, 2.0}));
    CHECK_FALSE(segment.ContainsPoint(Point3{0.0, 1.0, 2.0}));
    CHECK(segment.ParameterOf(Point3{0.0, 0.0, 0.0}) == 0.0);
    const auto half = segment.Subcurve(Interval{0.0, 0.5});
    REQUIRE(half.has_value());
    CHECK(*half == Segment3{Point3{0.0, 0.0, 0.0}, Point3{0.0, 0.0, 2.0}});
    CHECK_FALSE(segment.Subcurve(Interval{0.0, 2.0}).has_value());
    CHECK_FALSE(segment.IsClosed());
    CHECK_FALSE(segment.Area().has_value());
    CHECK_FALSE(segment.Orientation().has_value());
    CHECK_FALSE(segment.Centroid().has_value());
    CHECK_FALSE(segment.Contains(Point3{0.0, 0.0, 2.0}));

    STATIC_REQUIRE(noexcept(segment.Domain()));
    STATIC_REQUIRE(noexcept(segment.PointAt(0.5)));
    Segment3 mutableSegment = segment;
    STATIC_REQUIRE(noexcept(mutableSegment.Rotate(Point3{}, UnitVector3::ZAxis, 0.0)));
    STATIC_REQUIRE(noexcept(mutableSegment.Mirror(Point3{}, UnitVector3::ZAxis)));
    STATIC_REQUIRE(noexcept(mutableSegment.Transform(Transform3::Identity())));
    STATIC_REQUIRE(noexcept(segment.Subcurve(Interval{})));
}

TEST_CASE("Segment3 containment uses the supporting line and a parameter window", "[prim][segment3]") {
    const Segment3 segment{Point3{0.0, 0.0, 0.0}, Point3{1e-8, 0.0, 0.0}};
    CHECK_FALSE(segment.ContainsPoint(Point3{1e-8 + 1e-12, 0.0, 0.0}));
    CHECK(segment.ContainsPoint(Point3{0.5e-8, 0.0, 0.0}));
    CHECK_FALSE(segment.ContainsPoint(Point3{0.5e-8, 1.0, 0.0}));

    const Segment3 unit{Point3{0.0, 0.0, 0.0}, Point3{1.0, 0.0, 0.0}};
    const auto beforeStart = unit.ParameterOf(Point3{-1e-10, 0.0, 0.0});
    REQUIRE(beforeStart.has_value());
    CHECK(*beforeStart == Approx(-1e-10));
    CHECK(*beforeStart < 0.0);

    const Segment3 point{Point3{1.0, 1.0, 1.0}, Point3{1.0, 1.0, 1.0}};
    CHECK(point.ContainsPoint(Point3{1.0, 1.0, 1.0}));
    CHECK(point.ParameterOf(Point3{1.0, 1.0, 1.0}) == 0.0);
    CHECK_FALSE(point.ContainsPoint(Point3{1.0 + 1e-10, 1.0, 1.0}));
}

TEST_CASE("Segment3 along +Z converts to a Ray3 from the start", "[prim][segment3]") {
    const Segment3 segment{Point3{0.0, 0.0, 0.0}, Point3{0.0, 0.0, 4.0}};
    const auto ray = segment.AsRay();
    REQUIRE(ray.has_value());
    CHECK(ray->Origin == Point3{0.0, 0.0, 0.0});
    CHECK(ray->Direction == UnitVector3::ZAxis);
}
