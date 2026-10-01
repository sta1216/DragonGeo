#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <limits>
#include <type_traits>

#include <DragonGeo/Core/Constants.hpp>
#include <DragonGeo/Detail/CurveParameter.hpp>
#include <DragonGeo/Linear/Box2.hpp>
#include <DragonGeo/Linear/Interval.hpp>
#include <DragonGeo/Linear/Transform2.hpp>
#include <DragonGeo/Linear/UnitVector2.hpp>
#include <DragonGeo/Linear/Vector2.hpp>
#include <DragonGeo/Prim/Line2.hpp>
#include <DragonGeo/Prim/Segment2.hpp>

using Catch::Approx;

using DragonGeo::Core::HALF_PI;
using DragonGeo::Detail::ClampParameter;
using DragonGeo::Detail::CoordinatesAreFinite;
using DragonGeo::Detail::IsAcceptedParameter;
using DragonGeo::Detail::IsFiniteSubinterval;
using DragonGeo::Detail::ProjectParameter;
using DragonGeo::Linear::Box2;
using DragonGeo::Linear::Interval;
using DragonGeo::Linear::Point2;
using DragonGeo::Linear::Transform2;
using DragonGeo::Linear::UnitVector2;
using DragonGeo::Linear::Vector2;
using DragonGeo::Prim::Line2;
using DragonGeo::Prim::Ray2;
using DragonGeo::Prim::Segment2;
using DragonGeo::Prim::Segment2f;
using DragonGeo::Prim::Segment2T;

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

TEST_CASE("Segment2 evaluates parameters and distances", "[prim][segment2]") {
    const Segment2 segment{Point2{0.0, 0.0}, Point2{3.0, 0.0}};
    CHECK(segment.Domain() == Interval{0.0, 1.0});
    CHECK(segment.PointAt(0.0) == Point2{0.0, 0.0});
    CHECK(segment.PointAt(1.0) == Point2{3.0, 0.0});
    CHECK(segment.PointAt(0.5) == Point2{1.5, 0.0});
    CHECK_FALSE(segment.PointAt(2.0).has_value());
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double infinity = std::numeric_limits<double>::infinity();
    CHECK_FALSE(segment.PointAt(nan).has_value());
    CHECK_FALSE(segment.PointAt(infinity).has_value());

    CHECK(segment.ClosestPoint(Point2{1.0, 4.0}) == Point2{1.0, 0.0});
    CHECK(segment.DistanceSquared(Point2{1.0, 4.0}) == 16.0);
    CHECK(segment.Distance(Point2{1.0, 4.0}) == 4.0);
    CHECK(segment.ClosestPoint(Point2{-1.0, 1.0}) == Point2{0.0, 0.0});
    CHECK(segment.ClosestPoint(Point2{4.0, 1.0}) == Point2{3.0, 0.0});

    CHECK(segment.Length() == 3.0);
    CHECK(segment.LengthSquared() == 9.0);
    CHECK(segment.Direction() == UnitVector2::XAxis);
    CHECK(segment.Bounds() == Box2::FromCorners(Point2{0.0, 0.0}, Point2{3.0, 0.0}));
    CHECK(segment.StartPoint() == Point2{0.0, 0.0});
    CHECK(segment.EndPoint() == Point2{3.0, 0.0});
    CHECK(segment.MidPoint() == Point2{1.5, 0.0});
    CHECK(segment.StartTangent() == UnitVector2::XAxis);
    CHECK(segment.EndTangent() == UnitVector2::XAxis);
    CHECK(segment.MidTangent() == UnitVector2::XAxis);
    CHECK(segment.TangentAt(0.25) == UnitVector2::XAxis);
    CHECK_FALSE(segment.TangentAt(2.0).has_value());
    CHECK_FALSE(segment.TangentAt(nan).has_value());
}

TEST_CASE("a zero-length Segment2 has no direction", "[prim][segment2]") {
    const Segment2 segment{Point2{1.0, 1.0}, Point2{1.0, 1.0}};
    CHECK(segment.IsValid());
    CHECK_FALSE(segment.Direction().has_value());
    CHECK(segment.Length() == 0.0);
    CHECK(segment.LengthSquared() == 0.0);
    CHECK(segment.ClosestPoint(Point2{4.0, 5.0}) == Point2{1.0, 1.0});
    CHECK_FALSE(segment.StartTangent().has_value());
    CHECK_FALSE(segment.EndTangent().has_value());
    CHECK_FALSE(segment.MidTangent().has_value());
    CHECK_FALSE(segment.TangentAt(0.0).has_value());
    CHECK(segment.ParameterOf(Point2{1.0, 1.0}) == 0.0);
    CHECK_FALSE(segment.ParameterOf(Point2{2.0, 1.0}).has_value());
    CHECK(segment.Transformed(Transform2::Identity()) == segment);
}

TEST_CASE("Segment2 reverses and transforms its endpoints", "[prim][segment2]") {
    const Segment2 segment{Point2{0.0, 0.0}, Point2{3.0, 0.0}};
    const Segment2 reversed = segment.Reversed();
    CHECK(reversed.A == Point2{3.0, 0.0});
    CHECK(reversed.B == Point2{0.0, 0.0});

    const Segment2 translated = segment.Translated(Vector2{1.0, 0.0});
    CHECK(translated.A == Point2{1.0, 0.0});
    CHECK(translated.B == Point2{4.0, 0.0});

    const Segment2 upright{Point2{0.0, 1.0}, Point2{2.0, 1.0}};
    const Segment2 mirrored = upright.Mirrored(Point2{0.0, 0.0}, UnitVector2::YAxis);
    CHECK(mirrored.A == Point2{0.0, -1.0});
    CHECK(mirrored.B == Point2{2.0, -1.0});

    const Segment2 onAxis{Point2{1.0, 0.0}, Point2{2.0, 0.0}};
    const Segment2 rotated = onAxis.Rotated(Point2{0.0, 0.0}, HALF_PI);
    CHECK(rotated.A.X == Approx(0.0).margin(1e-12));
    CHECK(rotated.A.Y == Approx(1.0).margin(1e-12));
    CHECK(rotated.B.X == Approx(0.0).margin(1e-12));
    CHECK(rotated.B.Y == Approx(2.0).margin(1e-12));

    CHECK(segment.Transformed(Transform2::Identity()) == segment);
    CHECK_FALSE(segment.Transformed(Transform2::Scaling(0.0)).has_value());
    const double infinity = std::numeric_limits<double>::infinity();
    CHECK_FALSE(segment.Transformed(Transform2::Translation(Vector2{infinity, 0.0})).has_value());
    CHECK(segment.Clone() == segment);
}

TEST_CASE("Segment2 containment, parameters and subcurves", "[prim][segment2]") {
    const Segment2 segment{Point2{0.0, 0.0}, Point2{3.0, 0.0}};
    CHECK(segment.ContainsPoint(Point2{1.5, 0.0}));
    CHECK_FALSE(segment.ContainsPoint(Point2{1.5, 1.0}));
    CHECK(segment.ParameterOf(Point2{0.0, 0.0}) == 0.0);
    CHECK(segment.ParameterOf(Point2{1.5, 0.0}) == 0.5);
    CHECK_FALSE(segment.ParameterOf(Point2{1.0, 4.0}).has_value());

    const auto half = segment.Subcurve(Interval{0.0, 0.5});
    REQUIRE(half.has_value());
    CHECK(*half == Segment2{Point2{0.0, 0.0}, Point2{1.5, 0.0}});
    CHECK_FALSE(segment.Subcurve(Interval{0.0, 2.0}).has_value());
    CHECK_FALSE(segment.Subcurve(Interval{0.25, 0.25}).has_value());

    CHECK_FALSE(segment.IsClosed());
    CHECK_FALSE(segment.Area().has_value());
    CHECK_FALSE(segment.Orientation().has_value());
    CHECK_FALSE(segment.Centroid().has_value());
    CHECK_FALSE(segment.Contains(Point2{1.5, 0.0}));
}

TEST_CASE("CurveParameter projects and clamps", "[prim][segment2]") {
    CHECK(ProjectParameter(Point2{0.0, 0.0}, UnitVector2::XAxis, Point2{2.0, 5.0}) == 2.0);
    const Interval segmentDomain{0.0, 1.0};
    CHECK(ClampParameter(-0.5, segmentDomain) == 0.0);
    CHECK(ClampParameter(0.25, segmentDomain) == 0.25);
    CHECK(ClampParameter(2.0, segmentDomain) == 1.0);
    CHECK(ClampParameter(0.5, Interval::Empty()) == 0.5);

    const double infinity = std::numeric_limits<double>::infinity();
    const Interval rayDomain{0.0, infinity};
    CHECK(ClampParameter(-1.0, rayDomain) == 0.0);
    CHECK(ClampParameter(4.0, rayDomain) == 4.0);

    CHECK(IsAcceptedParameter(0.5, segmentDomain));
    CHECK_FALSE(IsAcceptedParameter(2.0, segmentDomain));
    CHECK_FALSE(IsAcceptedParameter(infinity, Interval::Unbounded()));
    CHECK(IsFiniteSubinterval(Interval{0.0, 0.5}, segmentDomain));
    CHECK_FALSE(IsFiniteSubinterval(Interval{0.0, 0.0}, segmentDomain));
    CHECK_FALSE(IsFiniteSubinterval(Interval{0.0, 2.0}, segmentDomain));
    CHECK(CoordinatesAreFinite(Point2{1.0, 2.0}));
    const double nan = std::numeric_limits<double>::quiet_NaN();
    CHECK_FALSE(CoordinatesAreFinite(Point2{nan, 0.0}));
    CHECK_FALSE(CoordinatesAreFinite(Vector2{0.0, infinity}));
}

TEST_CASE("Segment2 curve methods are noexcept", "[prim][segment2]") {
    const Segment2 segment{Point2{0.0, 0.0}, Point2{3.0, 0.0}};
    STATIC_REQUIRE(noexcept(segment.Domain()));
    STATIC_REQUIRE(noexcept(segment.PointAt(0.0)));
    STATIC_REQUIRE(noexcept(segment.ClosestPoint(Point2{})));
    STATIC_REQUIRE(noexcept(segment.DistanceSquared(Point2{})));
    STATIC_REQUIRE(noexcept(segment.Distance(Point2{})));
    STATIC_REQUIRE(noexcept(segment.Length()));
    STATIC_REQUIRE(noexcept(segment.LengthSquared()));
    STATIC_REQUIRE(noexcept(segment.Direction()));
    STATIC_REQUIRE(noexcept(segment.Bounds()));
    STATIC_REQUIRE(noexcept(segment.StartPoint()));
    STATIC_REQUIRE(noexcept(segment.EndPoint()));
    STATIC_REQUIRE(noexcept(segment.MidPoint()));
    STATIC_REQUIRE(noexcept(segment.StartTangent()));
    STATIC_REQUIRE(noexcept(segment.EndTangent()));
    STATIC_REQUIRE(noexcept(segment.MidTangent()));
    STATIC_REQUIRE(noexcept(segment.TangentAt(0.0)));
    STATIC_REQUIRE(noexcept(segment.IsClosed()));
    STATIC_REQUIRE(noexcept(segment.Area()));
    STATIC_REQUIRE(noexcept(segment.Orientation()));
    STATIC_REQUIRE(noexcept(segment.Centroid()));
    STATIC_REQUIRE(noexcept(segment.Contains(Point2{})));
    STATIC_REQUIRE(noexcept(segment.ContainsPoint(Point2{})));
    STATIC_REQUIRE(noexcept(segment.ParameterOf(Point2{})));
    STATIC_REQUIRE(noexcept(segment.Translated(Vector2{})));
    STATIC_REQUIRE(noexcept(segment.Rotated(Point2{}, 0.0)));
    STATIC_REQUIRE(noexcept(segment.Mirrored(Point2{}, UnitVector2::YAxis)));
    STATIC_REQUIRE(noexcept(segment.Reversed()));
    STATIC_REQUIRE(noexcept(segment.Clone()));
    STATIC_REQUIRE(noexcept(segment.Transformed(Transform2::Identity())));
    STATIC_REQUIRE(noexcept(segment.AsRay()));
    STATIC_REQUIRE(noexcept(segment.AsLine()));
    STATIC_REQUIRE(noexcept(segment.Subcurve(Interval{})));

    constexpr Segment2 axis{Point2{0.0, 0.0}, Point2{3.0, 0.0}};
    STATIC_REQUIRE(axis.Domain() == Interval{0.0, 1.0});
    STATIC_REQUIRE(axis.PointAt(0.0) == Point2{0.0, 0.0});
    STATIC_REQUIRE(axis.PointAt(1.0) == Point2{3.0, 0.0});
    STATIC_REQUIRE_FALSE(axis.PointAt(2.0).has_value());
    STATIC_REQUIRE(axis.LengthSquared() == 9.0);
    STATIC_REQUIRE_FALSE(axis.IsClosed());
    STATIC_REQUIRE_FALSE(axis.Contains(Point2{}));
    STATIC_REQUIRE(axis.Reversed().A == Point2{3.0, 0.0});
    STATIC_REQUIRE(axis.Reversed().B == Point2{0.0, 0.0});
}

TEST_CASE("Segment2 containment uses the supporting line and a parameter window", "[prim][segment2]") {
    const Segment2 segment{Point2{0.0, 0.0}, Point2{1e-8, 0.0}};
    CHECK_FALSE(segment.ContainsPoint(Point2{1e-8 + 1e-12, 0.0}));
    CHECK(segment.ContainsPoint(Point2{0.5e-8, 0.0}));
    CHECK_FALSE(segment.ContainsPoint(Point2{0.5e-8, 1.0}));

    const Segment2 unit{Point2{0.0, 0.0}, Point2{1.0, 0.0}};
    const auto beforeStart = unit.ParameterOf(Point2{-1e-10, 0.0});
    REQUIRE(beforeStart.has_value());
    CHECK(*beforeStart == Approx(-1e-10));
    CHECK(*beforeStart < 0.0);

    const Segment2 point{Point2{1.0, 1.0}, Point2{1.0, 1.0}};
    CHECK(point.ContainsPoint(Point2{1.0, 1.0}));
    CHECK(point.ParameterOf(Point2{1.0, 1.0}) == 0.0);
    CHECK_FALSE(point.ContainsPoint(Point2{1.0 + 1e-10, 1.0}));
}

TEST_CASE("zero-length Segment2 AsRay and AsLine are empty", "[prim][segment2]") {
    const Segment2 segment{Point2{1.0, 2.0}, Point2{1.0, 2.0}};
    CHECK_FALSE(segment.AsRay().has_value());
    CHECK_FALSE(segment.AsLine().has_value());
}

TEST_CASE("Segment2 along +X converts to a Ray2 from the start", "[prim][segment2]") {
    const Segment2 segment{Point2{0.0, 0.0}, Point2{3.0, 0.0}};
    const auto ray = segment.AsRay();
    REQUIRE(ray.has_value());
    CHECK(ray->Origin == Point2{0.0, 0.0});
    CHECK(ray->Direction == UnitVector2::XAxis);
}
