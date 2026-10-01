#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <limits>
#include <optional>
#include <span>
#include <type_traits>
#include <variant>

#include <DragonGeo/Core/Constants.hpp>
#include <DragonGeo/Linear/Box2.hpp>
#include <DragonGeo/Linear/Interval.hpp>
#include <DragonGeo/Linear/Point2.hpp>
#include <DragonGeo/Linear/Transform2.hpp>
#include <DragonGeo/Linear/UnitVector2.hpp>
#include <DragonGeo/Linear/Vector2.hpp>
#include <DragonGeo/Prim/Polyline.hpp>
#include <DragonGeo/Prim/Segment2.hpp>

using Catch::Approx;

using DragonGeo::Core::HALF_PI;
using DragonGeo::Linear::Box2;
using DragonGeo::Linear::Interval;
using DragonGeo::Linear::Point2;
using DragonGeo::Linear::Point2f;
using DragonGeo::Linear::Transform2;
using DragonGeo::Linear::UnitVector2;
using DragonGeo::Linear::Vector2;
using DragonGeo::Prim::Polyline;
using DragonGeo::Prim::PolylineT;
using DragonGeo::Prim::Polylinef;
using DragonGeo::Prim::Segment2;
using DragonGeo::Prim::Winding;

TEST_CASE("Polylinef is the float alias and Polyline is not an aggregate", "[prim][polyline]") {
    STATIC_REQUIRE(std::is_same_v<Polyline, PolylineT<double>>);
    STATIC_REQUIRE(std::is_same_v<Polylinef, PolylineT<float>>);
    STATIC_REQUIRE(!std::is_aggregate_v<Polyline>);
    STATIC_REQUIRE(!std::is_aggregate_v<Polylinef>);

    const Point2f points[]{Point2f{0.0f, 0.0f}, Point2f{1.0f, 0.0f}};
    const auto polyline = Polylinef::FromPoints(points);
    REQUIRE(polyline.has_value());
    CHECK(polyline->PointCount() == 2);
    CHECK(polyline->SegmentCount() == 1);
    CHECK(polyline->IsValid());
    CHECK_FALSE(polyline->IsClosed());
}

TEST_CASE("FromPoints rejects an empty span, one point, and a non-finite point", "[prim][polyline]") {
    const std::span<const Point2> empty;
    CHECK_FALSE(Polyline::FromPoints(empty).has_value());

    const Point2 only[]{Point2{0.0, 0.0}};
    CHECK_FALSE(Polyline::FromPoints(only).has_value());

    const double nan = std::numeric_limits<double>::quiet_NaN();
    const Point2 withNan[]{Point2{0.0, 0.0}, Point2{nan, 0.0}};
    CHECK_FALSE(Polyline::FromPoints(withNan).has_value());

    const double infinity = std::numeric_limits<double>::infinity();
    const Point2 withInfinity[]{Point2{0.0, 0.0}, Point2{0.0, infinity}};
    CHECK_FALSE(Polyline::FromPoints(withInfinity).has_value());
}

TEST_CASE("an open two-point polyline is one segment without area", "[prim][polyline]") {
    const Point2 points[]{Point2{0.0, 0.0}, Point2{1.0, 0.0}};
    const auto polyline = Polyline::FromPoints(points);
    REQUIRE(polyline.has_value());
    CHECK(polyline->IsValid());
    CHECK_FALSE(polyline->IsClosed());
    CHECK(polyline->PointCount() == 2);
    CHECK(polyline->SegmentCount() == 1);
    CHECK(polyline->Point(0) == Point2{0.0, 0.0});
    CHECK(polyline->Point(1) == Point2{1.0, 0.0});
    CHECK(polyline->Segment(0) == Segment2{Point2{0.0, 0.0}, Point2{1.0, 0.0}});
    CHECK_FALSE(polyline->Area().has_value());
    CHECK_FALSE(polyline->Orientation().has_value());
    CHECK_FALSE(polyline->Centroid().has_value());
    CHECK_FALSE(polyline->Contains(Point2{0.5, 0.0}));

    CHECK(polyline->Domain() == Interval{0.0, 1.0});
    CHECK(polyline->PointAt(0.0) == Point2{0.0, 0.0});
    CHECK(polyline->PointAt(0.5) == Point2{0.5, 0.0});
    CHECK(polyline->PointAt(1.0) == Point2{1.0, 0.0});
    const double nan = std::numeric_limits<double>::quiet_NaN();
    CHECK_FALSE(polyline->PointAt(-0.1).has_value());
    CHECK_FALSE(polyline->PointAt(1.1).has_value());
    CHECK_FALSE(polyline->PointAt(nan).has_value());
    CHECK_FALSE(polyline->PointAt(std::numeric_limits<double>::infinity()).has_value());
    CHECK(polyline->StartPoint() == Point2{0.0, 0.0});
    CHECK(polyline->EndPoint() == Point2{1.0, 0.0});
    CHECK(polyline->MidPoint() == Point2{0.5, 0.0});
    CHECK(polyline->Length() == 1.0);
    CHECK(polyline->Box() == Box2::FromCorners(Point2{0.0, 0.0}, Point2{1.0, 0.0}));
    CHECK(polyline->StartTangent() == UnitVector2::XAxis);
    CHECK(polyline->EndTangent() == UnitVector2::XAxis);
    CHECK(polyline->MidTangent() == UnitVector2::XAxis);
    CHECK(polyline->TangentAt(0.0) == UnitVector2::XAxis);
    CHECK(polyline->TangentAt(1.0) == UnitVector2::XAxis);
    CHECK_FALSE(polyline->TangentAt(-0.1).has_value());
    CHECK_FALSE(polyline->TangentAt(nan).has_value());
    CHECK(polyline->ContainsPoint(Point2{0.5, 0.0}));
    CHECK(polyline->ParameterOf(Point2{0.5, 0.0}) == 0.5);
    CHECK(polyline->ParameterOf(Point2{0.0, 0.0}) == 0.0);
    CHECK(polyline->ParameterOf(Point2{1.0, 0.0}) == 1.0);
    CHECK_FALSE(polyline->ContainsPoint(Point2{0.5, 1.0}));
    CHECK_FALSE(polyline->ParameterOf(Point2{0.5, 1.0}).has_value());

    const auto same = Polyline::FromPoints(points);
    REQUIRE(same.has_value());
    CHECK(*polyline == *same);
    const Point2 otherPoints[]{Point2{0.0, 0.0}, Point2{2.0, 0.0}};
    const auto different = Polyline::FromPoints(otherPoints);
    REQUIRE(different.has_value());
    CHECK(*polyline != *different);
    const Point2 longer[]{Point2{0.0, 0.0}, Point2{1.0, 0.0}, Point2{1.0, 1.0}};
    const auto extra = Polyline::FromPoints(longer);
    REQUIRE(extra.has_value());
    CHECK(*polyline != *extra);

    STATIC_REQUIRE(noexcept(polyline->Domain()));
    STATIC_REQUIRE(noexcept(polyline->PointAt(0.0)));
    STATIC_REQUIRE(noexcept(polyline->IsValid()));
    STATIC_REQUIRE(noexcept(polyline->IsClosed()));
    STATIC_REQUIRE(noexcept(polyline->Length()));
    STATIC_REQUIRE(noexcept(polyline->Area()));
    STATIC_REQUIRE(noexcept(polyline->Orientation()));
    STATIC_REQUIRE(noexcept(polyline->Box()));
    STATIC_REQUIRE(noexcept(polyline->StartPoint()));
    STATIC_REQUIRE(noexcept(polyline->EndPoint()));
    STATIC_REQUIRE(noexcept(polyline->MidPoint()));
    STATIC_REQUIRE(noexcept(polyline->TangentAt(0.0)));
    STATIC_REQUIRE(noexcept(polyline->ContainsPoint(Point2{})));
    STATIC_REQUIRE(noexcept(polyline->ParameterOf(Point2{})));
    STATIC_REQUIRE(noexcept(*polyline == *same));
}

TEST_CASE("a closed triangle polyline reports area, winding, and containment", "[prim][polyline]") {
    const Point2 points[]{Point2{0.0, 0.0}, Point2{1.0, 0.0}, Point2{0.0, 1.0}, Point2{0.0, 0.0}};
    const auto polyline = Polyline::FromPoints(points);
    REQUIRE(polyline.has_value());
    CHECK(polyline->IsClosed());
    CHECK(polyline->SegmentCount() == 3);
    CHECK(polyline->PointCount() == 4);
    CHECK(polyline->Orientation() == Winding::CounterClockwise);
    CHECK(polyline->Area() == 0.5);
    CHECK(polyline->Length() == Approx(2.0 + std::sqrt(2.0)));
    const auto centroid = polyline->Centroid();
    REQUIRE(centroid.has_value());
    CHECK(centroid->X == Approx(1.0 / 3.0));
    CHECK(centroid->Y == Approx(1.0 / 3.0));
    CHECK(polyline->Box() == Box2::FromCorners(Point2{0.0, 0.0}, Point2{1.0, 1.0}));
    CHECK(polyline->Domain() == Interval{0.0, 3.0});
    CHECK(polyline->PointAt(0.0) == Point2{0.0, 0.0});
    CHECK(polyline->PointAt(3.0) == Point2{0.0, 0.0});
    CHECK(polyline->MidPoint() == Point2{0.5, 0.5});
    CHECK(polyline->StartPoint() == Point2{0.0, 0.0});
    CHECK(polyline->EndPoint() == Point2{0.0, 0.0});
    CHECK(polyline->StartTangent() == UnitVector2::XAxis);
    CHECK(polyline->EndTangent() == -UnitVector2::YAxis);
    CHECK(polyline->TangentAt(0.0) == UnitVector2::XAxis);
    CHECK(polyline->TangentAt(3.0) == -UnitVector2::YAxis);
    CHECK(polyline->TangentAt(1.0) == polyline->MidTangent());
    const auto mid = polyline->MidTangent();
    REQUIRE(mid.has_value());
    CHECK(mid->X() == Approx(-std::sqrt(0.5)));
    CHECK(mid->Y() == Approx(std::sqrt(0.5)));

    CHECK(polyline->Contains(Point2{0.2, 0.2}));
    CHECK(polyline->Contains(Point2{0.5, 0.0}));
    CHECK(polyline->Contains(Point2{0.0, 0.0}));
    CHECK_FALSE(polyline->Contains(Point2{1.0, 1.0}));
    CHECK(polyline->ContainsPoint(Point2{0.5, 0.0}));
    CHECK_FALSE(polyline->ContainsPoint(Point2{0.2, 0.2}));
    CHECK(polyline->ParameterOf(Point2{0.0, 0.0}) == 0.0);
    CHECK(polyline->ParameterOf(Point2{0.0, 0.5}) == Approx(2.5));
    CHECK_FALSE(polyline->ParameterOf(Point2{0.2, 0.2}).has_value());
    STATIC_REQUIRE(requires(const Polyline& shape, Point2 point) { shape.Contains(point); });
    STATIC_REQUIRE(noexcept(polyline->Contains(Point2{})));
    STATIC_REQUIRE(noexcept(polyline->Centroid()));
}

TEST_CASE("a clockwise closed polyline keeps the opposite winding", "[prim][polyline]") {
    const Point2 points[]{Point2{0.0, 0.0}, Point2{0.0, 1.0}, Point2{1.0, 0.0}, Point2{0.0, 0.0}};
    const auto polyline = Polyline::FromPoints(points);
    REQUIRE(polyline.has_value());
    CHECK(polyline->Orientation() == Winding::Clockwise);
    CHECK(polyline->Area() == 0.5);
    CHECK(polyline->Contains(Point2{0.2, 0.2}));
    CHECK_FALSE(polyline->Contains(Point2{1.0, 1.0}));
}

TEST_CASE("a closed polyline keeps a zero area and only its boundary", "[prim][polyline]") {
    const Point2 points[]{Point2{0.0, 0.0}, Point2{2.0, 0.0}, Point2{0.0, 0.0}};
    const auto polyline = Polyline::FromPoints(points);
    REQUIRE(polyline.has_value());
    CHECK(polyline->IsClosed());
    CHECK(polyline->IsValid());
    CHECK(polyline->Area() == 0.0);
    CHECK(polyline->Orientation() == Winding::Degenerate);
    CHECK_FALSE(polyline->Centroid().has_value());
    CHECK(polyline->Contains(Point2{1.0, 0.0}));
    CHECK(polyline->Contains(Point2{0.0, 0.0}));
    CHECK_FALSE(polyline->Contains(Point2{0.0, 1.0}));
}

TEST_CASE("centroid averages unique vertices and ignores the repeated end", "[prim][polyline]") {
    const Point2 points[]{Point2{0.0, 0.0}, Point2{1.0, 0.0}, Point2{1.0, 1.0}, Point2{0.0, 2.0}, Point2{0.0, 0.0}};
    const auto polyline = Polyline::FromPoints(points);
    REQUIRE(polyline.has_value());
    const auto centroid = polyline->Centroid();
    REQUIRE(centroid.has_value());
    CHECK(centroid->X == Approx(0.5));
    CHECK(centroid->Y == Approx(0.75));
    CHECK(polyline->Area() == Approx(1.5));
}

TEST_CASE("a zero signed area can still contain a nonzero winding", "[prim][polyline]") {
    const Point2 points[]{Point2{0.0, 0.0}, Point2{1.0, 1.0}, Point2{1.0, 0.0}, Point2{0.0, 1.0}, Point2{0.0, 0.0}};
    const auto polyline = Polyline::FromPoints(points);
    REQUIRE(polyline.has_value());
    CHECK(polyline->Area() == 0.0);
    CHECK(polyline->Orientation() == Winding::Degenerate);
    CHECK_FALSE(polyline->Centroid().has_value());
    CHECK(polyline->Contains(Point2{0.9, 0.6}));
    CHECK(polyline->Contains(Point2{0.0, 0.0}));
    CHECK_FALSE(polyline->Contains(Point2{2.0, 2.0}));
}

TEST_CASE("a zero-length edge has no tangent and uses scale 1", "[prim][polyline]") {
    const Point2 points[]{Point2{5.0, 5.0}, Point2{5.0, 5.0}};
    const auto polyline = Polyline::FromPoints(points);
    REQUIRE(polyline.has_value());
    CHECK(polyline->IsClosed());
    CHECK(polyline->IsValid());
    CHECK(polyline->Length() == 0.0);
    CHECK_FALSE(polyline->StartTangent().has_value());
    CHECK_FALSE(polyline->EndTangent().has_value());
    CHECK_FALSE(polyline->MidTangent().has_value());
    CHECK_FALSE(polyline->TangentAt(0.0).has_value());
    CHECK_FALSE(polyline->TangentAt(1.0).has_value());
    CHECK(polyline->Contains(Point2{5.0, 5.0}));
    CHECK_FALSE(polyline->Contains(Point2{5.0, 6.0}));
    CHECK(polyline->ContainsPoint(Point2{5.0, 5.0}));
    CHECK(polyline->ContainsPoint(Point2{5.0 + 1e-10, 5.0}));
    CHECK_FALSE(polyline->ContainsPoint(Point2{5.0 + 1e-6, 5.0}));
}

TEST_CASE("polyline transforms mutate points and reverse keeps a closed seam", "[prim][polyline]") {
    const Point2 points[]{Point2{0.0, 0.0}, Point2{1.0, 0.0}, Point2{0.0, 1.0}, Point2{0.0, 0.0}};
    const auto original = Polyline::FromPoints(points);
    REQUIRE(original.has_value());

    Polyline translated = *original;
    translated.Translate(Vector2{0.0, 1.0});
    CHECK(translated.Point(0) == Point2{0.0, 1.0});
    CHECK(translated.Point(1) == Point2{1.0, 1.0});
    CHECK(translated.Point(2) == Point2{0.0, 2.0});
    CHECK(translated.Point(3) == Point2{0.0, 1.0});
    CHECK(original->Point(0) == Point2{0.0, 0.0});
    CHECK(original->Point(2) == Point2{0.0, 1.0});

    Polyline reversed = *original;
    reversed.Reverse();
    CHECK(reversed.Point(0) == Point2{0.0, 0.0});
    CHECK(reversed.Point(1) == Point2{0.0, 1.0});
    CHECK(reversed.Point(2) == Point2{1.0, 0.0});
    CHECK(reversed.Point(3) == Point2{0.0, 0.0});
    CHECK(reversed.IsClosed());
    CHECK(reversed.Orientation() == Winding::Clockwise);
    CHECK(original->Orientation() == Winding::CounterClockwise);

    const Point2 openPoints[]{Point2{0.0, 0.0}, Point2{1.0, 0.0}, Point2{1.0, 2.0}};
    const auto open = Polyline::FromPoints(openPoints);
    REQUIRE(open.has_value());
    Polyline openReversed = *open;
    openReversed.Reverse();
    CHECK(openReversed.Point(0) == Point2{1.0, 2.0});
    CHECK(openReversed.Point(1) == Point2{1.0, 0.0});
    CHECK(openReversed.Point(2) == Point2{0.0, 0.0});
    CHECK_FALSE(openReversed.IsClosed());

    Polyline mirrored = *original;
    mirrored.Mirror(Point2{0.0, 0.0}, UnitVector2::YAxis);
    CHECK(mirrored.Point(2) == Point2{0.0, -1.0});
    CHECK(original->Point(2) == Point2{0.0, 1.0});

    Polyline rotated = *original;
    rotated.Rotate(Point2{0.0, 0.0}, HALF_PI);
    CHECK(rotated.Point(1).X == Approx(0.0).margin(1e-12));
    CHECK(rotated.Point(1).Y == Approx(1.0).margin(1e-12));
    CHECK(rotated.Point(2).X == Approx(-1.0).margin(1e-12));
    CHECK(rotated.Point(2).Y == Approx(0.0).margin(1e-12));
    CHECK(original->Point(1) == Point2{1.0, 0.0});

    CHECK(original->Clone() == *original);
    Polyline clone = original->Clone();
    clone.Translate(Vector2{1.0, 0.0});
    CHECK(original->Point(0) == Point2{0.0, 0.0});
    CHECK(clone.Point(0) == Point2{1.0, 0.0});

    Polyline identity = *original;
    CHECK(identity.Transform(Transform2::Identity()));
    CHECK(identity == *original);

    const double infinity = std::numeric_limits<double>::infinity();
    Polyline shifted = *original;
    CHECK_FALSE(shifted.Transform(Transform2::Translation(Vector2{infinity, 0.0})));
    CHECK(shifted == *original);
    CHECK(shifted.Point(0) == Point2{0.0, 0.0});
    CHECK(shifted.Point(1) == Point2{1.0, 0.0});
    CHECK(shifted.Point(2) == Point2{0.0, 1.0});
    CHECK(shifted.Point(3) == Point2{0.0, 0.0});

    Polyline mutablePolyline = *original;
    STATIC_REQUIRE(noexcept(mutablePolyline.Translate(Vector2{})));
    STATIC_REQUIRE(noexcept(mutablePolyline.Rotate(Point2{}, 0.0)));
    STATIC_REQUIRE(noexcept(mutablePolyline.Mirror(Point2{}, UnitVector2::YAxis)));
    STATIC_REQUIRE(noexcept(mutablePolyline.Reverse()));
    STATIC_REQUIRE(noexcept(mutablePolyline.Transform(Transform2::Identity())));
}

TEST_CASE("Polyline subcurve is a segment or a polyline", "[prim][polyline]") {
    const Point2 points[]{Point2{0.0, 0.0}, Point2{1.0, 0.0}, Point2{1.0, 2.0}};
    const auto polyline = Polyline::FromPoints(points);
    REQUIRE(polyline.has_value());
    STATIC_REQUIRE(std::is_same_v< decltype(polyline->Subcurve(Interval{})), std::optional<std::variant<Segment2, Polyline>>>);

    const auto onEdge = polyline->Subcurve(Interval{0.0, 0.5});
    REQUIRE(onEdge.has_value());
    REQUIRE(std::holds_alternative<Segment2>(*onEdge));
    CHECK(std::get<Segment2>(*onEdge) == Segment2{Point2{0.0, 0.0}, Point2{0.5, 0.0}});

    const auto across = polyline->Subcurve(Interval{0.5, 1.5});
    REQUIRE(across.has_value());
    REQUIRE(std::holds_alternative<Polyline>(*across));
    const Polyline& piece = std::get<Polyline>(*across);
    CHECK(piece.PointCount() == 3);
    CHECK(piece.Point(0) == Point2{0.5, 0.0});
    CHECK(piece.Point(1) == Point2{1.0, 0.0});
    CHECK(piece.Point(2) == Point2{1.0, 1.0});
    CHECK(piece.PointAt(0.0) == Point2{0.5, 0.0});
    CHECK(piece.PointAt(2.0) == Point2{1.0, 1.0});

    CHECK_FALSE(polyline->Subcurve(Interval{0.0, 3.0}).has_value());
    CHECK_FALSE(polyline->Subcurve(Interval{0.5, 0.5}).has_value());
    CHECK_FALSE(polyline->Subcurve(Interval{1.0, 0.0}).has_value());
    const double nan = std::numeric_limits<double>::quiet_NaN();
    CHECK_FALSE(polyline->Subcurve(Interval{nan, 1.0}).has_value());
}
