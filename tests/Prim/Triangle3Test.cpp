#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <limits>
#include <optional>
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
#include <DragonGeo/Prim/Triangle3.hpp>

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
using DragonGeo::Prim::Segment3;
using DragonGeo::Prim::Triangle3;
using DragonGeo::Prim::Winding;
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

TEST_CASE("Triangle3 is a closed curve with perimeter and tangents", "[prim][triangle3]") {
    const Triangle3 triangle{
        Point3{0.0, 0.0, 0.0}, Point3{1.0, 0.0, 0.0}, Point3{0.0, 1.0, 0.0}};
    CHECK(triangle.IsClosed());
    CHECK(triangle.StartPoint() == Point3{0.0, 0.0, 0.0});
    CHECK(triangle.EndPoint() == Point3{0.0, 0.0, 0.0});
    CHECK(triangle.Length() == Approx(2.0 + std::sqrt(2.0)));
    CHECK(triangle.Area() == 0.5);
    CHECK(triangle.Orientation() == Winding::CounterClockwise);
    const auto centroid = triangle.Centroid();
    REQUIRE(centroid.has_value());
    CHECK(centroid->X == Approx(1.0 / 3.0));
    CHECK(centroid->Y == Approx(1.0 / 3.0));
    CHECK(centroid->Z == Approx(0.0));
    CHECK(triangle.Box()
          == Box3::FromCorners(Point3{0.0, 0.0, 0.0}, Point3{1.0, 1.0, 0.0}));
    CHECK(triangle.StartTangent() == UnitVector3::XAxis);
    CHECK(triangle.EndTangent() == -UnitVector3::YAxis);
    CHECK(triangle.TangentAt(0.0) == UnitVector3::XAxis);
    CHECK(triangle.TangentAt(1.0) == triangle.MidTangent());
    CHECK(triangle.TangentAt(3.0) == -UnitVector3::YAxis);
    CHECK(triangle.TangentAt(2.0) == triangle.EndTangent());
    const auto mid = triangle.MidTangent();
    REQUIRE(mid.has_value());
    CHECK(mid->X() == Approx(-std::sqrt(0.5)));
    CHECK(mid->Y() == Approx(std::sqrt(0.5)));
    CHECK(mid->Z() == Approx(0.0));

    const double nan = std::numeric_limits<double>::quiet_NaN();
    CHECK_FALSE(triangle.TangentAt(-0.1).has_value());
    CHECK_FALSE(triangle.TangentAt(3.1).has_value());
    CHECK_FALSE(triangle.TangentAt(nan).has_value());

    const Triangle3 missingEdge{
        Point3{0.0, 0.0, 0.0}, Point3{0.0, 0.0, 0.0}, Point3{1.0, 0.0, 0.0}};
    CHECK_FALSE(missingEdge.StartTangent().has_value());
    CHECK_FALSE(missingEdge.TangentAt(0.0).has_value());
    CHECK(missingEdge.MidTangent() == UnitVector3::XAxis);
    CHECK(missingEdge.EndTangent() == -UnitVector3::XAxis);

    STATIC_REQUIRE(noexcept(triangle.Length()));
    STATIC_REQUIRE(noexcept(triangle.IsClosed()));
    STATIC_REQUIRE(noexcept(triangle.TangentAt(0.0)));
    STATIC_REQUIRE(noexcept(triangle.ContainsPoint(Point3{})));
    STATIC_REQUIRE(noexcept(triangle.ParameterOf(Point3{})));
    STATIC_REQUIRE(noexcept(triangle.Clone()));
}

TEST_CASE("a zero-area Triangle3 has no centroid", "[prim][triangle3]") {
    const Triangle3 flat{
        Point3{0.0, 0.0, 0.0}, Point3{1.0, 0.0, 0.0}, Point3{2.0, 0.0, 0.0}};
    CHECK(flat.Area() == 0.0);
    CHECK(flat.Orientation() == Winding::Degenerate);
    CHECK_FALSE(flat.Centroid().has_value());

    const Triangle3 collapsed{
        Point3{0.0, 0.0, 0.0}, Point3{0.0, 0.0, 0.0}, Point3{0.0, 0.0, 0.0}};
    CHECK(collapsed.ContainsPoint(Point3{1e-10, 0.0, 0.0}));
    CHECK_FALSE(collapsed.ContainsPoint(Point3{1e-6, 0.0, 0.0}));
}

TEST_CASE("Triangle3 ContainsPoint follows the boundary, not the face", "[prim][triangle3]") {
    const Triangle3 triangle{
        Point3{0.0, 0.0, 0.0}, Point3{1.0, 0.0, 0.0}, Point3{0.0, 1.0, 0.0}};
    CHECK(triangle.ContainsPoint(Point3{0.5, 0.0, 0.0}));
    CHECK_FALSE(triangle.ContainsPoint(Point3{0.2, 0.2, 0.0}));
    CHECK(triangle.ParameterOf(Point3{0.0, 0.0, 0.0}) == 0.0);
    CHECK_FALSE(triangle.ParameterOf(Point3{0.2, 0.2, 0.0}).has_value());

    CHECK(triangle.ClosestPoint(Point3{0.2, 0.2, 0.0}) == Point3{0.2, 0.2, 0.0});
    CHECK(triangle.Distance(Point3{0.2, 0.2, 0.0}) == 0.0);
    CHECK(triangle.DistanceSquared(Point3{0.2, 0.2, 0.0}) == 0.0);
    CHECK(triangle.ClosestPoint(Point3{0.2, 0.2, 5.0}) == Point3{0.2, 0.2, 0.0});
    CHECK(triangle.Distance(Point3{0.2, 0.2, 5.0}) == 5.0);
    CHECK(triangle.DistanceSquared(Point3{0.2, 0.2, 5.0}) == 25.0);

    const Triangle3 vertical{
        Point3{0.0, 0.0, 0.0}, Point3{1.0, 0.0, 0.0}, Point3{0.0, 0.0, 1.0}};
    CHECK(vertical.ClosestPoint(Point3{0.2, 4.0, 0.2}) == Point3{0.2, 0.0, 0.2});
    CHECK(vertical.Distance(Point3{0.2, 4.0, 0.2}) == 4.0);

    CHECK(triangle.ClosestPoint(Point3{2.0, 0.0, 0.0}) == Point3{1.0, 0.0, 0.0});
    CHECK(triangle.DistanceSquared(Point3{2.0, 0.0, 0.0}) == 1.0);

    const Triangle3f small{
        Point3f{0.0f, 0.0f, 0.0f}, Point3f{1.0f, 0.0f, 0.0f}, Point3f{0.0f, 1.0f, 0.0f}};
    CHECK(small.ClosestPoint(Point3f{0.2f, 0.2f, 0.0f}) == Point3f{0.2f, 0.2f, 0.0f});
    CHECK(small.Distance(Point3f{0.2f, 0.2f, 0.0f}) == 0.0f);
    CHECK(small.DistanceSquared(Point3f{0.2f, 0.2f, 0.0f}) == 0.0f);
}

TEST_CASE("Triangle3 reverses, mirrors, rotates and transforms", "[prim][triangle3]") {
    const Triangle3 triangle{
        Point3{0.0, 0.0, 0.0}, Point3{1.0, 0.0, 0.0}, Point3{0.0, 1.0, 0.0}};
    Triangle3 reversed = triangle;
    reversed.Reverse();
    CHECK(reversed.Orientation() == Winding::Clockwise);
    CHECK(reversed.StartPoint() == triangle.A);
    CHECK(reversed.EndPoint() == triangle.A);
    CHECK(reversed.B == triangle.C);
    CHECK(reversed.C == triangle.B);
    CHECK(triangle.B == Point3{1.0, 0.0, 0.0});
    CHECK(triangle.C == Point3{0.0, 1.0, 0.0});

    Triangle3 translated = triangle;
    translated.Translate(Vector3{0.0, 0.0, 1.0});
    CHECK(translated.A == Point3{0.0, 0.0, 1.0});
    CHECK(translated.C == Point3{0.0, 1.0, 1.0});
    CHECK(triangle.A == Point3{0.0, 0.0, 0.0});

    Triangle3 mirrored = triangle;
    mirrored.Mirror(Point3{0.0, 0.0, 0.0}, UnitVector3::YAxis);
    CHECK(mirrored.C == Point3{0.0, -1.0, 0.0});
    CHECK(triangle.C == Point3{0.0, 1.0, 0.0});

    Triangle3 rotated = triangle;
    rotated.Rotate(Point3{0.0, 0.0, 0.0}, UnitVector3::ZAxis, HALF_PI);
    CHECK(rotated.B.X == Approx(0.0).margin(1e-12));
    CHECK(rotated.B.Y == Approx(1.0).margin(1e-12));
    CHECK(rotated.B.Z == Approx(0.0).margin(1e-12));
    CHECK(rotated.C.X == Approx(-1.0).margin(1e-12));
    CHECK(rotated.C.Y == Approx(0.0).margin(1e-12));
    CHECK(triangle.B == Point3{1.0, 0.0, 0.0});

    CHECK(triangle.Clone() == triangle);
    Triangle3 identity = triangle;
    CHECK(identity.Transform(Transform3::Identity()));
    CHECK(identity == triangle);
    Triangle3 collapsed = triangle;
    CHECK(collapsed.Transform(Transform3::Scaling(0.0)));
    CHECK(collapsed.Area() == 0.0);
    CHECK(triangle.Area() == 0.5);
    const double infinity = std::numeric_limits<double>::infinity();
    Triangle3 shifted = triangle;
    CHECK_FALSE(shifted.Transform(Transform3::Translation(Vector3{infinity, 0.0, 0.0})));
    CHECK(shifted == triangle);
    Triangle3 mutableTriangle = triangle;
    STATIC_REQUIRE(noexcept(mutableTriangle.Rotate(Point3{}, UnitVector3::ZAxis, 0.0)));
    STATIC_REQUIRE(noexcept(mutableTriangle.Mirror(Point3{}, UnitVector3::YAxis)));
    STATIC_REQUIRE(noexcept(mutableTriangle.Transform(Transform3::Identity())));
}
