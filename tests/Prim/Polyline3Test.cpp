#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <limits>
#include <optional>
#include <span>
#include <type_traits>
#include <variant>

#include <DragonGeo/Core/Constants.hpp>
#include <DragonGeo/Linear/Box3.hpp>
#include <DragonGeo/Linear/Interval.hpp>
#include <DragonGeo/Linear/Point3.hpp>
#include <DragonGeo/Linear/Transform3.hpp>
#include <DragonGeo/Linear/UnitVector3.hpp>
#include <DragonGeo/Linear/Vector3.hpp>
#include <DragonGeo/Prim/Polyline3.hpp>
#include <DragonGeo/Prim/Segment3.hpp>

using Catch::Approx;

using DragonGeo::Core::HALF_PI;
using DragonGeo::Linear::Box3;
using DragonGeo::Linear::Interval;
using DragonGeo::Linear::Point3;
using DragonGeo::Linear::Point3f;
using DragonGeo::Linear::Transform3;
using DragonGeo::Linear::UnitVector3;
using DragonGeo::Linear::Vector3;
using DragonGeo::Prim::Polyline3;
using DragonGeo::Prim::Polyline3T;
using DragonGeo::Prim::Polyline3f;
using DragonGeo::Prim::Segment3;
using DragonGeo::Prim::Winding;

TEST_CASE("Polyline3f is the float alias and Polyline3 is not an aggregate", "[prim][polyline3]") {
    STATIC_REQUIRE(std::is_same_v<Polyline3f, Polyline3T<float>>);
    STATIC_REQUIRE(!std::is_aggregate_v<Polyline3>);
    STATIC_REQUIRE(!std::is_aggregate_v<Polyline3f>);

    const Point3f points[]{Point3f{0.0f, 0.0f, 0.0f}, Point3f{1.0f, 0.0f, 0.0f}};
    const auto polyline = Polyline3f::FromPoints(points);
    REQUIRE(polyline.has_value());
    CHECK(polyline->PointCount() == 2);
    CHECK(polyline->SegmentCount() == 1);
    CHECK(polyline->IsValid());
    CHECK_FALSE(polyline->IsClosed());
}

TEST_CASE("FromPoints rejects an empty span, one point, and a non-finite point", "[prim][polyline3]") {
    const std::span<const Point3> empty;
    CHECK_FALSE(Polyline3::FromPoints(empty).has_value());

    const Point3 only[]{Point3{0.0, 0.0, 0.0}};
    CHECK_FALSE(Polyline3::FromPoints(only).has_value());

    const double nan = std::numeric_limits<double>::quiet_NaN();
    const Point3 withNan[]{Point3{0.0, 0.0, 0.0}, Point3{nan, 0.0, 0.0}};
    CHECK_FALSE(Polyline3::FromPoints(withNan).has_value());

    const double infinity = std::numeric_limits<double>::infinity();
    const Point3 withInfinity[]{Point3{0.0, 0.0, 0.0}, Point3{0.0, infinity, 0.0}};
    CHECK_FALSE(Polyline3::FromPoints(withInfinity).has_value());
}

TEST_CASE("an open two-point polyline is one segment without area", "[prim][polyline3]") {
    const Point3 points[]{Point3{0.0, 0.0, 0.0}, Point3{1.0, 0.0, 0.0}};
    const auto polyline = Polyline3::FromPoints(points);
    REQUIRE(polyline.has_value());
    CHECK(polyline->IsValid());
    CHECK_FALSE(polyline->IsClosed());
    CHECK(polyline->PointCount() == 2);
    CHECK(polyline->SegmentCount() == 1);
    CHECK(polyline->Point(0) == Point3{0.0, 0.0, 0.0});
    CHECK(polyline->Point(1) == Point3{1.0, 0.0, 0.0});
    CHECK(polyline->Segment(0) == Segment3{Point3{0.0, 0.0, 0.0}, Point3{1.0, 0.0, 0.0}});
    CHECK_FALSE(polyline->Area().has_value());
    CHECK_FALSE(polyline->Orientation().has_value());
    CHECK_FALSE(polyline->Centroid().has_value());
    CHECK_FALSE(polyline->Contains(Point3{0.5, 0.0, 0.0}));

    CHECK(polyline->Domain() == Interval{0.0, 1.0});
    CHECK(polyline->PointAt(0.0) == Point3{0.0, 0.0, 0.0});
    CHECK(polyline->PointAt(0.5) == Point3{0.5, 0.0, 0.0});
    CHECK(polyline->PointAt(1.0) == Point3{1.0, 0.0, 0.0});
    const double nan = std::numeric_limits<double>::quiet_NaN();
    CHECK_FALSE(polyline->PointAt(-0.1).has_value());
    CHECK_FALSE(polyline->PointAt(1.1).has_value());
    CHECK_FALSE(polyline->PointAt(nan).has_value());
    CHECK_FALSE(polyline->PointAt(std::numeric_limits<double>::infinity()).has_value());
    CHECK(polyline->StartPoint() == Point3{0.0, 0.0, 0.0});
    CHECK(polyline->EndPoint() == Point3{1.0, 0.0, 0.0});
    CHECK(polyline->MidPoint() == Point3{0.5, 0.0, 0.0});
    CHECK(polyline->Length() == 1.0);
    CHECK(polyline->Box() == Box3::FromCorners(Point3{0.0, 0.0, 0.0}, Point3{1.0, 0.0, 0.0}));
    CHECK(polyline->StartTangent() == UnitVector3::XAxis);
    CHECK(polyline->EndTangent() == UnitVector3::XAxis);
    CHECK(polyline->MidTangent() == UnitVector3::XAxis);
    CHECK(polyline->TangentAt(0.0) == UnitVector3::XAxis);
    CHECK(polyline->TangentAt(1.0) == UnitVector3::XAxis);
    CHECK_FALSE(polyline->TangentAt(-0.1).has_value());
    CHECK_FALSE(polyline->TangentAt(nan).has_value());
    CHECK(polyline->ContainsPoint(Point3{0.5, 0.0, 0.0}));
    CHECK(polyline->ParameterOf(Point3{0.5, 0.0, 0.0}) == 0.5);
    CHECK(polyline->ParameterOf(Point3{0.0, 0.0, 0.0}) == 0.0);
    CHECK(polyline->ParameterOf(Point3{1.0, 0.0, 0.0}) == 1.0);
    CHECK_FALSE(polyline->ContainsPoint(Point3{0.5, 1.0, 0.0}));
    CHECK_FALSE(polyline->ParameterOf(Point3{0.5, 1.0, 0.0}).has_value());

    const auto same = Polyline3::FromPoints(points);
    REQUIRE(same.has_value());
    CHECK(*polyline == *same);
    const Point3 otherPoints[]{Point3{0.0, 0.0, 0.0}, Point3{2.0, 0.0, 0.0}};
    const auto different = Polyline3::FromPoints(otherPoints);
    REQUIRE(different.has_value());
    CHECK(*polyline != *different);
    const Point3 longer[]{Point3{0.0, 0.0, 0.0}, Point3{1.0, 0.0, 0.0}, Point3{1.0, 1.0, 0.0}};
    const auto extra = Polyline3::FromPoints(longer);
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
    STATIC_REQUIRE(noexcept(polyline->ContainsPoint(Point3{})));
    STATIC_REQUIRE(noexcept(polyline->ParameterOf(Point3{})));
    STATIC_REQUIRE(noexcept(*polyline == *same));
}

TEST_CASE("a closed triangle polyline reports area, winding, and containment", "[prim][polyline3]") {
    const Point3 points[]{Point3{0.0, 0.0, 0.0}, Point3{1.0, 0.0, 0.0}, Point3{0.0, 1.0, 0.0}, Point3{0.0, 0.0, 0.0}};
    const auto polyline = Polyline3::FromPoints(points);
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
    CHECK(centroid->Z == Approx(0.0));
    CHECK(polyline->Box() == Box3::FromCorners(Point3{0.0, 0.0, 0.0}, Point3{1.0, 1.0, 0.0}));
    CHECK(polyline->Domain() == Interval{0.0, 3.0});
    CHECK(polyline->PointAt(0.0) == Point3{0.0, 0.0, 0.0});
    CHECK(polyline->PointAt(3.0) == Point3{0.0, 0.0, 0.0});
    CHECK(polyline->MidPoint() == Point3{0.5, 0.5, 0.0});
    CHECK(polyline->StartPoint() == Point3{0.0, 0.0, 0.0});
    CHECK(polyline->EndPoint() == Point3{0.0, 0.0, 0.0});
    CHECK(polyline->StartTangent() == UnitVector3::XAxis);
    CHECK(polyline->EndTangent() == -UnitVector3::YAxis);
    CHECK(polyline->TangentAt(0.0) == UnitVector3::XAxis);
    CHECK(polyline->TangentAt(3.0) == -UnitVector3::YAxis);
    CHECK(polyline->TangentAt(1.0) == polyline->MidTangent());
    const auto mid = polyline->MidTangent();
    REQUIRE(mid.has_value());
    CHECK(mid->X() == Approx(-std::sqrt(0.5)));
    CHECK(mid->Y() == Approx(std::sqrt(0.5)));
    CHECK(mid->Z() == Approx(0.0));

    CHECK(polyline->Contains(Point3{0.2, 0.2, 0.0}));
    CHECK(polyline->Contains(Point3{0.5, 0.0, 0.0}));
    CHECK(polyline->Contains(Point3{0.0, 0.0, 0.0}));
    CHECK_FALSE(polyline->Contains(Point3{1.0, 1.0, 0.0}));
    CHECK_FALSE(polyline->Contains(Point3{0.2, 0.2, 1.0}));
    CHECK(polyline->ContainsPoint(Point3{0.5, 0.0, 0.0}));
    CHECK_FALSE(polyline->ContainsPoint(Point3{0.2, 0.2, 0.0}));
    CHECK(polyline->ParameterOf(Point3{0.0, 0.0, 0.0}) == 0.0);
    CHECK(polyline->ParameterOf(Point3{0.0, 0.5, 0.0}) == Approx(2.5));
    CHECK_FALSE(polyline->ParameterOf(Point3{0.2, 0.2, 0.0}).has_value());
    STATIC_REQUIRE(requires(const Polyline3& shape, Point3 point) { shape.Contains(point); });
    STATIC_REQUIRE(noexcept(polyline->Contains(Point3{})));
    STATIC_REQUIRE(noexcept(polyline->Centroid()));
}

TEST_CASE("a closed polyline keeps a zero area and only its boundary", "[prim][polyline3]") {
    const Point3 points[]{Point3{0.0, 0.0, 0.0}, Point3{2.0, 0.0, 0.0}, Point3{0.0, 0.0, 0.0}};
    const auto polyline = Polyline3::FromPoints(points);
    REQUIRE(polyline.has_value());
    CHECK(polyline->IsClosed());
    CHECK(polyline->IsValid());
    CHECK(polyline->Area() == 0.0);
    CHECK(polyline->Orientation() == Winding::Degenerate);
    CHECK_FALSE(polyline->Centroid().has_value());
    CHECK(polyline->Contains(Point3{1.0, 0.0, 0.0}));
    CHECK(polyline->Contains(Point3{0.0, 0.0, 0.0}));
    CHECK_FALSE(polyline->Contains(Point3{0.0, 1.0, 0.0}));
}

TEST_CASE("equal absolute area components keep the earlier axis", "[prim][polyline3]") {
    const Point3 points[]{Point3{0.0, 0.0, 0.0}, Point3{1.0, 1.0, 0.0}, Point3{0.0, 0.0, 1.0}, Point3{0.0, 0.0, 0.0}};
    const auto polyline = Polyline3::FromPoints(points);
    REQUIRE(polyline.has_value());
    CHECK(polyline->Orientation() == Winding::CounterClockwise);
    CHECK(polyline->Area() == Approx(0.5 * std::sqrt(2.0)));
    CHECK(polyline->Contains(Point3{1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0}));
    CHECK_FALSE(polyline->Contains(Point3{1.0 / 3.0, 1.0, 1.0 / 3.0}));
}

TEST_CASE("a non-planar closed polyline uses the vector area and contains nothing", "[prim][polyline3]") {
    const Point3 points[]{Point3{0.0, 0.0, 0.0}, Point3{1.0, 0.0, 0.0}, Point3{1.0, 1.0, 0.0}, Point3{0.0, 1.0, 1.0}, Point3{0.0, 0.0, 0.0}};
    const auto polyline = Polyline3::FromPoints(points);
    REQUIRE(polyline.has_value());
    CHECK(polyline->IsClosed());
    CHECK(polyline->Area() == Approx(std::sqrt(1.5)));
    CHECK(polyline->Orientation() == Winding::CounterClockwise);
    CHECK_FALSE(polyline->Contains(Point3{0.0, 0.0, 0.0}));
    CHECK_FALSE(polyline->Contains(Point3{0.5, 0.0, 0.0}));
    CHECK_FALSE(polyline->Contains(Point3{0.5, 0.5, 0.0}));
}

TEST_CASE("centroid averages unique vertices and ignores the repeated end", "[prim][polyline3]") {
    const Point3 points[]{Point3{0.0, 0.0, 0.0}, Point3{1.0, 0.0, 0.0}, Point3{1.0, 1.0, 0.0}, Point3{0.0, 2.0, 0.0}, Point3{0.0, 0.0, 0.0}};
    const auto polyline = Polyline3::FromPoints(points);
    REQUIRE(polyline.has_value());
    const auto centroid = polyline->Centroid();
    REQUIRE(centroid.has_value());
    CHECK(centroid->X == Approx(0.5));
    CHECK(centroid->Y == Approx(0.75));
    CHECK(centroid->Z == Approx(0.0));
    CHECK(polyline->Area() == Approx(1.5));
}

TEST_CASE("a zero vector area can still contain a nonzero winding", "[prim][polyline3]") {
    const Point3 points[]{Point3{0.0, 0.0, 0.0}, Point3{1.0, 1.0, 0.0}, Point3{1.0, 0.0, 0.0}, Point3{0.0, 1.0, 0.0}, Point3{0.0, 0.0, 0.0}};
    const auto polyline = Polyline3::FromPoints(points);
    REQUIRE(polyline.has_value());
    CHECK(polyline->Area() == 0.0);
    CHECK(polyline->Orientation() == Winding::Degenerate);
    CHECK_FALSE(polyline->Centroid().has_value());
    CHECK(polyline->Contains(Point3{0.9, 0.6, 0.0}));
    CHECK(polyline->Contains(Point3{0.0, 0.0, 0.0}));
    CHECK_FALSE(polyline->Contains(Point3{2.0, 2.0, 0.0}));
}

TEST_CASE("a zero-length edge has no tangent and uses scale 1", "[prim][polyline3]") {
    const Point3 points[]{Point3{5.0, 5.0, 5.0}, Point3{5.0, 5.0, 5.0}};
    const auto polyline = Polyline3::FromPoints(points);
    REQUIRE(polyline.has_value());
    CHECK(polyline->IsClosed());
    CHECK(polyline->IsValid());
    CHECK(polyline->Length() == 0.0);
    CHECK_FALSE(polyline->StartTangent().has_value());
    CHECK_FALSE(polyline->EndTangent().has_value());
    CHECK_FALSE(polyline->MidTangent().has_value());
    CHECK_FALSE(polyline->TangentAt(0.0).has_value());
    CHECK_FALSE(polyline->TangentAt(1.0).has_value());
    CHECK(polyline->Contains(Point3{5.0, 5.0, 5.0}));
    CHECK_FALSE(polyline->Contains(Point3{5.0, 5.0, 6.0}));
    CHECK(polyline->ContainsPoint(Point3{5.0, 5.0, 5.0}));
    CHECK(polyline->ContainsPoint(Point3{5.0 + 1e-10, 5.0, 5.0}));
    CHECK_FALSE(polyline->ContainsPoint(Point3{5.0 + 1e-6, 5.0, 5.0}));
}

TEST_CASE("polyline transforms mutate points and reverse keeps a closed seam", "[prim][polyline3]") {
    const Point3 points[]{Point3{0.0, 0.0, 0.0}, Point3{1.0, 0.0, 0.0}, Point3{0.0, 1.0, 0.0}, Point3{0.0, 0.0, 0.0}};
    const auto original = Polyline3::FromPoints(points);
    REQUIRE(original.has_value());

    Polyline3 translated = *original;
    translated.Translate(Vector3{0.0, 0.0, 1.0});
    CHECK(translated.Point(0) == Point3{0.0, 0.0, 1.0});
    CHECK(translated.Point(1) == Point3{1.0, 0.0, 1.0});
    CHECK(translated.Point(2) == Point3{0.0, 1.0, 1.0});
    CHECK(translated.Point(3) == Point3{0.0, 0.0, 1.0});
    CHECK(original->Point(0) == Point3{0.0, 0.0, 0.0});
    CHECK(original->Point(2) == Point3{0.0, 1.0, 0.0});

    Polyline3 reversed = *original;
    reversed.Reverse();
    CHECK(reversed.Point(0) == Point3{0.0, 0.0, 0.0});
    CHECK(reversed.Point(1) == Point3{0.0, 1.0, 0.0});
    CHECK(reversed.Point(2) == Point3{1.0, 0.0, 0.0});
    CHECK(reversed.Point(3) == Point3{0.0, 0.0, 0.0});
    CHECK(reversed.IsClosed());
    CHECK(reversed.Orientation() == Winding::Clockwise);
    CHECK(original->Orientation() == Winding::CounterClockwise);

    const Point3 openPoints[]{Point3{0.0, 0.0, 0.0}, Point3{1.0, 0.0, 0.0}, Point3{1.0, 2.0, 0.0}};
    const auto open = Polyline3::FromPoints(openPoints);
    REQUIRE(open.has_value());
    Polyline3 openReversed = *open;
    openReversed.Reverse();
    CHECK(openReversed.Point(0) == Point3{1.0, 2.0, 0.0});
    CHECK(openReversed.Point(1) == Point3{1.0, 0.0, 0.0});
    CHECK(openReversed.Point(2) == Point3{0.0, 0.0, 0.0});
    CHECK_FALSE(openReversed.IsClosed());

    Polyline3 mirrored = *original;
    mirrored.Mirror(Point3{0.0, 0.0, 0.0}, UnitVector3::YAxis);
    CHECK(mirrored.Point(2) == Point3{0.0, -1.0, 0.0});
    CHECK(original->Point(2) == Point3{0.0, 1.0, 0.0});

    Polyline3 rotated = *original;
    rotated.Rotate(Point3{0.0, 0.0, 0.0}, UnitVector3::ZAxis, HALF_PI);
    CHECK(rotated.Point(1).X == Approx(0.0).margin(1e-12));
    CHECK(rotated.Point(1).Y == Approx(1.0).margin(1e-12));
    CHECK(rotated.Point(1).Z == Approx(0.0).margin(1e-12));
    CHECK(rotated.Point(2).X == Approx(-1.0).margin(1e-12));
    CHECK(rotated.Point(2).Y == Approx(0.0).margin(1e-12));
    CHECK(original->Point(1) == Point3{1.0, 0.0, 0.0});

    CHECK(original->Clone() == *original);
    Polyline3 clone = original->Clone();
    clone.Translate(Vector3{1.0, 0.0, 0.0});
    CHECK(original->Point(0) == Point3{0.0, 0.0, 0.0});
    CHECK(clone.Point(0) == Point3{1.0, 0.0, 0.0});

    Polyline3 identity = *original;
    CHECK(identity.Transform(Transform3::Identity()));
    CHECK(identity == *original);

    const double infinity = std::numeric_limits<double>::infinity();
    Polyline3 shifted = *original;
    CHECK_FALSE(shifted.Transform(Transform3::Translation(Vector3{infinity, 0.0, 0.0})));
    CHECK(shifted == *original);
    CHECK(shifted.Point(0) == Point3{0.0, 0.0, 0.0});
    CHECK(shifted.Point(1) == Point3{1.0, 0.0, 0.0});
    CHECK(shifted.Point(2) == Point3{0.0, 1.0, 0.0});
    CHECK(shifted.Point(3) == Point3{0.0, 0.0, 0.0});

    Polyline3 mutablePolyline = *original;
    STATIC_REQUIRE(noexcept(mutablePolyline.Translate(Vector3{})));
    STATIC_REQUIRE(noexcept(mutablePolyline.Rotate(Point3{}, UnitVector3::ZAxis, 0.0)));
    STATIC_REQUIRE(noexcept(mutablePolyline.Mirror(Point3{}, UnitVector3::YAxis)));
    STATIC_REQUIRE(noexcept(mutablePolyline.Reverse()));
    STATIC_REQUIRE(noexcept(mutablePolyline.Transform(Transform3::Identity())));
}

TEST_CASE("Polyline3 subcurve is a segment or a polyline", "[prim][polyline3]") {
    const Point3 points[]{Point3{0.0, 0.0, 0.0}, Point3{1.0, 0.0, 0.0}, Point3{1.0, 2.0, 0.0}};
    const auto polyline = Polyline3::FromPoints(points);
    REQUIRE(polyline.has_value());
    STATIC_REQUIRE(std::is_same_v< decltype(polyline->Subcurve(Interval{})), std::optional<std::variant<Segment3, Polyline3>>>);

    const auto onEdge = polyline->Subcurve(Interval{0.0, 0.5});
    REQUIRE(onEdge.has_value());
    REQUIRE(std::holds_alternative<Segment3>(*onEdge));
    CHECK(std::get<Segment3>(*onEdge) == Segment3{Point3{0.0, 0.0, 0.0}, Point3{0.5, 0.0, 0.0}});

    const auto across = polyline->Subcurve(Interval{0.5, 1.5});
    REQUIRE(across.has_value());
    REQUIRE(std::holds_alternative<Polyline3>(*across));
    const Polyline3& piece = std::get<Polyline3>(*across);
    CHECK(piece.PointCount() == 3);
    CHECK(piece.Point(0) == Point3{0.5, 0.0, 0.0});
    CHECK(piece.Point(1) == Point3{1.0, 0.0, 0.0});
    CHECK(piece.Point(2) == Point3{1.0, 1.0, 0.0});
    CHECK(piece.PointAt(0.0) == Point3{0.5, 0.0, 0.0});
    CHECK(piece.PointAt(2.0) == Point3{1.0, 1.0, 0.0});

    CHECK_FALSE(polyline->Subcurve(Interval{0.0, 3.0}).has_value());
    CHECK_FALSE(polyline->Subcurve(Interval{0.5, 0.5}).has_value());
    CHECK_FALSE(polyline->Subcurve(Interval{1.0, 0.0}).has_value());
    const double nan = std::numeric_limits<double>::quiet_NaN();
    CHECK_FALSE(polyline->Subcurve(Interval{nan, 1.0}).has_value());
}
