#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <type_traits>

#include <DragonGeo/Linear/Box2.hpp>
#include <DragonGeo/Linear/Box3.hpp>
#include <DragonGeo/Linear/Coordinate2.hpp>
#include <DragonGeo/Linear/Coordinate3.hpp>
#include <DragonGeo/Linear/OrientedBox2.hpp>
#include <DragonGeo/Linear/OrientedBox3.hpp>
#include <DragonGeo/Linear/UnitVector2.hpp>
#include <DragonGeo/Linear/UnitVector3.hpp>
#include <DragonGeo/Linear/Vector2.hpp>
#include <DragonGeo/Linear/Vector3.hpp>
#include <DragonGeo/Prim/Line2.hpp>
#include <DragonGeo/Prim/Line3.hpp>
#include <DragonGeo/Prim/Plane.hpp>
#include <DragonGeo/Prim/Ray2.hpp>
#include <DragonGeo/Prim/Ray3.hpp>
#include <DragonGeo/Prim/Segment2.hpp>
#include <DragonGeo/Prim/Segment3.hpp>
#include <DragonGeo/Prim/Triangle3.hpp>
#include <DragonGeo/Query/Query.hpp>

using Catch::Approx;

using DragonGeo::Linear::Box2;
using DragonGeo::Linear::Box3;
using DragonGeo::Linear::Coordinate2;
using DragonGeo::Linear::Coordinate3;
using DragonGeo::Linear::OrientedBox2;
using DragonGeo::Linear::OrientedBox3;
using DragonGeo::Linear::Point2;
using DragonGeo::Linear::Point3;
using DragonGeo::Linear::UnitVector2;
using DragonGeo::Linear::UnitVector3;
using DragonGeo::Linear::Vector2;
using DragonGeo::Linear::Vector3;
using DragonGeo::Prim::Line2;
using DragonGeo::Prim::Line3;
using DragonGeo::Prim::Plane;
using DragonGeo::Prim::Ray2;
using DragonGeo::Prim::Ray3;
using DragonGeo::Prim::Segment2;
using DragonGeo::Prim::Segment3;
using DragonGeo::Prim::Triangle3;
using DragonGeo::Query::CurveMeet;
using DragonGeo::Query::Distance;
using DragonGeo::Query::DistanceSquared;
using DragonGeo::Query::Intersection;
using DragonGeo::Query::Intersects;

namespace {

bool OnBoxBoundary(Point2 point, Box2 box) {
    const bool onEdge = point.X == box.Min.X || point.X == box.Max.X || point.Y == box.Min.Y
        || point.Y == box.Max.Y;
    return box.Contains(point) && onEdge;
}

} // namespace

TEST_CASE("two Line2 cross, separate, or coincide", "[query]") {
    const Line2 horizontal{Point2{0.0, 0.0}, UnitVector2::XAxis};
    const Line2 vertical{Point2{0.0, 0.0}, UnitVector2::YAxis};
    const auto cross = Intersection(horizontal, vertical);
    CHECK(cross.Kind == CurveMeet::Point);
    CHECK(cross.Point == Point2{0.0, 0.0});
    CHECK(cross.ParameterOnFirst == 0.0);
    CHECK(cross.ParameterOnSecond == 0.0);
    CHECK(Intersects(horizontal, vertical));

    const Line2 raised{Point2{0.0, 1.0}, UnitVector2::XAxis};
    CHECK(Intersection(horizontal, raised).Kind == CurveMeet::None);
    CHECK_FALSE(Intersects(horizontal, raised));

    const Line2 same{Point2{4.0, 0.0}, UnitVector2::XAxis};
    CHECK(Intersection(horizontal, same).Kind == CurveMeet::Coincident);
    CHECK(Intersects(horizontal, same));

    const Line2 reversed{Point2{2.0, 0.0}, -UnitVector2::XAxis};
    CHECK(Intersection(horizontal, reversed).Kind == CurveMeet::Coincident);
}

TEST_CASE("Segment2 meets at an endpoint or overlaps", "[query]") {
    const Segment2 east{Point2{0.0, 0.0}, Point2{1.0, 0.0}};
    const Segment2 north{Point2{1.0, 0.0}, Point2{1.0, 1.0}};
    const auto touch = Intersection(east, north);
    CHECK(touch.Kind == CurveMeet::Point);
    CHECK(touch.Point == Point2{1.0, 0.0});
    CHECK(touch.ParameterOnFirst == 1.0);
    CHECK(touch.ParameterOnSecond == 0.0);
    CHECK(Intersects(east, north));

    const Segment2 first{Point2{0.0, 0.0}, Point2{2.0, 0.0}};
    const Segment2 second{Point2{1.0, 0.0}, Point2{3.0, 0.0}};
    const auto overlap = Intersection(first, second);
    CHECK(overlap.Kind == CurveMeet::Overlap);
    CHECK(overlap.Overlap == Segment2{Point2{1.0, 0.0}, Point2{2.0, 0.0}});
    CHECK(Intersects(first, second));

    const Segment2 reversedSecond{Point2{3.0, 0.0}, Point2{1.0, 0.0}};
    CHECK(Intersection(first, reversedSecond).Overlap
          == Segment2{Point2{1.0, 0.0}, Point2{2.0, 0.0}});

    const Segment2 gap{Point2{3.0, 0.0}, Point2{4.0, 0.0}};
    CHECK(Intersection(first, gap).Kind == CurveMeet::None);
    CHECK_FALSE(Intersects(first, gap));

    const Segment2 point{Point2{2.0, 0.0}, Point2{2.0, 0.0}};
    const auto zero = Intersection(first, point);
    CHECK(zero.Kind == CurveMeet::Point);
    CHECK(zero.Point == Point2{2.0, 0.0});
}

TEST_CASE("Line2 meets Segment2 at a point or along the segment", "[query]") {
    const Line2 line{Point2{0.0, 0.0}, UnitVector2::XAxis};
    const Segment2 crossing{Point2{1.0, -1.0}, Point2{1.0, 1.0}};
    const auto hit = Intersection(line, crossing);
    CHECK(hit.Kind == CurveMeet::Point);
    CHECK(hit.Point == Point2{1.0, 0.0});
    CHECK(hit.ParameterOnFirst == 1.0);
    CHECK(hit.ParameterOnSecond == 0.5);
    CHECK(Intersects(line, crossing));

    const Segment2 along{Point2{3.0, 0.0}, Point2{1.0, 0.0}};
    const auto overlap = Intersection(line, along);
    CHECK(overlap.Kind == CurveMeet::Overlap);
    CHECK(overlap.Overlap == Segment2{Point2{1.0, 0.0}, Point2{3.0, 0.0}});

    const Segment2 miss{Point2{0.0, 1.0}, Point2{1.0, 2.0}};
    CHECK(Intersection(line, miss).Kind == CurveMeet::None);
    CHECK_FALSE(Intersects(line, miss));

    const Segment2 point{Point2{2.0, 0.0}, Point2{2.0, 0.0}};
    const auto onLine = Intersection(line, point);
    CHECK(onLine.Kind == CurveMeet::Point);
    CHECK(onLine.Point == Point2{2.0, 0.0});
    CHECK(onLine.ParameterOnFirst == 2.0);
}

TEST_CASE("Ray3 and Segment3 meet a Plane", "[query]") {
    const Plane plane{Point3{0.0, 0.0, 0.0}, UnitVector3::ZAxis};
    const Ray3 ray{Point3{0.0, 0.0, -1.0}, UnitVector3::ZAxis};
    const auto hit = Intersection(ray, plane);
    REQUIRE(hit.has_value());
    CHECK(hit->Parameter == 1.0);
    CHECK(hit->Point == Point3{0.0, 0.0, 0.0});
    CHECK(Intersects(ray, plane));

    const Ray3 away{Point3{0.0, 0.0, 1.0}, UnitVector3::ZAxis};
    CHECK_FALSE(Intersects(away, plane));
    CHECK_FALSE(Intersection(away, plane).has_value());

    const Segment3 lying{Point3{0.0, 0.0, 0.0}, Point3{1.0, 0.0, 0.0}};
    CHECK(Intersects(lying, plane));
    CHECK_FALSE(Intersection(lying, plane).has_value());

    const Ray3 along{Point3{1.0, 0.0, 0.0}, UnitVector3::XAxis};
    CHECK(Intersects(along, plane));
    CHECK_FALSE(Intersection(along, plane).has_value());

    const Segment3 point{Point3{2.0, 0.0, 0.0}, Point3{2.0, 0.0, 0.0}};
    const auto pointHit = Intersection(point, plane);
    REQUIRE(pointHit.has_value());
    CHECK(pointHit->Parameter == 0.0);
    CHECK(pointHit->Point == Point3{2.0, 0.0, 0.0});

    const Segment3 crossing{Point3{0.0, 0.0, -1.0}, Point3{0.0, 0.0, 1.0}};
    const auto cut = Intersection(crossing, plane);
    REQUIRE(cut.has_value());
    CHECK(cut->Parameter == 0.5);
    CHECK(cut->Point == Point3{0.0, 0.0, 0.0});
    CHECK(Intersects(crossing, plane));
}

TEST_CASE("Line3 meets a Plane at a point or lies in it", "[query]") {
    const Plane plane{Point3{0.0, 0.0, 0.0}, UnitVector3::ZAxis};
    const Line3 piercing{Point3{0.0, 0.0, -2.0}, UnitVector3::ZAxis};
    const auto hit = Intersection(piercing, plane);
    CHECK(hit.Kind == CurveMeet::Point);
    CHECK(hit.Point == Point3{0.0, 0.0, 0.0});
    CHECK(hit.ParameterOnFirst == 2.0);
    CHECK(Intersects(piercing, plane));

    const Line3 lying{Point3{1.0, 2.0, 0.0}, UnitVector3::XAxis};
    CHECK(Intersection(lying, plane).Kind == CurveMeet::Coincident);
    CHECK(Intersects(lying, plane));

    const Line3 parallel{Point3{0.0, 0.0, 1.0}, UnitVector3::XAxis};
    CHECK(Intersection(parallel, plane).Kind == CurveMeet::None);
    CHECK_FALSE(Intersects(parallel, plane));
}

TEST_CASE("a line, ray, or segment meets a Triangle3", "[query]") {
    const Triangle3 triangle{Point3{0.0, 0.0, 0.0}, Point3{2.0, 0.0, 0.0}, Point3{0.0, 2.0, 0.0}};
    const Line3 overlapping{Point3{-1.0, 0.25, 0.0}, UnitVector3::XAxis};
    CHECK(Intersects(overlapping, triangle));
    CHECK_FALSE(Intersection(overlapping, triangle).has_value());

    const Ray3 piercing{Point3{0.5, 0.25, -1.0}, UnitVector3::ZAxis};
    const auto hit = Intersection(piercing, triangle);
    REQUIRE(hit.has_value());
    CHECK(hit->Parameter == 1.0);
    CHECK(hit->Point == Point3{0.5, 0.25, 0.0});
    CHECK(Intersects(piercing, triangle));

    const Ray3 miss{Point3{3.0, 3.0, -1.0}, UnitVector3::ZAxis};
    CHECK_FALSE(Intersects(miss, triangle));
    CHECK_FALSE(Intersection(miss, triangle).has_value());

    const Segment3 stab{Point3{0.5, 0.25, -1.0}, Point3{0.5, 0.25, 1.0}};
    const auto cut = Intersection(stab, triangle);
    REQUIRE(cut.has_value());
    CHECK(cut->Parameter == 0.5);
    CHECK(Intersects(stab, triangle));

    const Segment3 coplanar{Point3{0.2, 0.2, 0.0}, Point3{0.4, 0.2, 0.0}};
    CHECK(Intersects(coplanar, triangle));
    CHECK_FALSE(Intersection(coplanar, triangle).has_value());
}

TEST_CASE("a segment or line crosses a Box2, and an empty box misses", "[query]") {
    const Box2 box{Point2{0.0, 0.0}, Point2{1.0, 1.0}};
    const Segment2 through{Point2{-1.0, 0.5}, Point2{1.0, 0.5}};
    const auto segmentHit = Intersection(through, box);
    REQUIRE(segmentHit.has_value());
    CHECK(segmentHit->Enter < segmentHit->Exit);
    CHECK(segmentHit->Enter == 0.5);
    CHECK(segmentHit->Exit == 1.0);
    CHECK(OnBoxBoundary(segmentHit->EnterPoint, box));
    CHECK(OnBoxBoundary(segmentHit->ExitPoint, box));
    CHECK(Intersects(through, box));

    CHECK_FALSE(Intersects(through, Box2::Empty()));
    CHECK_FALSE(Intersection(through, Box2::Empty()).has_value());

    const Line2 line{Point2{-1.0, 0.5}, UnitVector2::XAxis};
    const auto lineHit = Intersection(line, box);
    REQUIRE(lineHit.has_value());
    CHECK(lineHit->Enter < lineHit->Exit);
    CHECK(lineHit->Enter == 1.0);
    CHECK(lineHit->Exit == 2.0);
    CHECK(Intersects(line, box));
    CHECK_FALSE(Intersects(line, Box2::Empty()));

    const Ray2 inside{Point2{0.25, 0.5}, UnitVector2::XAxis};
    const auto fromInside = Intersection(inside, box);
    REQUIRE(fromInside.has_value());
    CHECK(fromInside->Enter == 0.0);
    CHECK(fromInside->EnterPoint == Point2{0.25, 0.5});
    CHECK(fromInside->Exit == Approx(0.75));

    const Vector2 diagonal{1.0, -1.0};
    const Ray2 tangent{Point2{-1.0, 1.0}, *diagonal.Normalized()};
    const auto graze = Intersection(tangent, box);
    REQUIRE(graze.has_value());
    CHECK(graze->Enter == graze->Exit);

    const Ray2 behind{Point2{2.0, 0.5}, UnitVector2::XAxis};
    CHECK_FALSE(Intersection(behind, box).has_value());
    CHECK_FALSE(Intersects(behind, box));
}

TEST_CASE("Ray3, Segment3, and Line3 cross a Box3", "[query]") {
    const Box3 box{Point3{0.0, 0.0, 0.0}, Point3{1.0, 1.0, 1.0}};
    const Ray3 ray{Point3{-1.0, 0.5, 0.5}, UnitVector3::XAxis};
    const auto rayHit = Intersection(ray, box);
    REQUIRE(rayHit.has_value());
    CHECK(rayHit->Enter == 1.0);
    CHECK(rayHit->Exit == 2.0);
    CHECK(Intersects(ray, box));

    const Segment3 segment{Point3{-1.0, 0.5, 0.5}, Point3{2.0, 0.5, 0.5}};
    const auto segmentHit = Intersection(segment, box);
    REQUIRE(segmentHit.has_value());
    CHECK(segmentHit->Enter < segmentHit->Exit);
    CHECK(segmentHit->Enter >= 0.0);
    CHECK(segmentHit->Exit <= 1.0);

    const Line3 line{Point3{-1.0, 0.5, 0.5}, UnitVector3::XAxis};
    const auto lineHit = Intersection(line, box);
    REQUIRE(lineHit.has_value());
    CHECK(lineHit->Enter < lineHit->Exit);

    CHECK_FALSE(Intersects(ray, Box3::Empty()));
    CHECK_FALSE(Intersection(segment, Box3::Empty()).has_value());
    CHECK_FALSE(Intersection(line, Box3::Empty()).has_value());
}

TEST_CASE("a ray or segment meets an oriented box", "[query]") {
    const auto frame = Coordinate3::FromAxes(
        Point3{5.0, 0.0, 0.0}, UnitVector3::XAxis, UnitVector3::YAxis, UnitVector3::ZAxis);
    REQUIRE(frame.has_value());
    const OrientedBox3 box{*frame, Vector3{1.0, 1.0, 1.0}};
    const Ray3 ray{Point3{0.0, 0.0, 0.0}, UnitVector3::XAxis};
    const auto hit = Intersection(ray, box);
    REQUIRE(hit.has_value());
    CHECK(hit->Enter == 4.0);
    CHECK(hit->Exit == 6.0);
    CHECK(hit->EnterPoint == Point3{4.0, 0.0, 0.0});
    CHECK(Intersects(ray, box));

    const Segment3 segment{Point3{0.0, 0.0, 0.0}, Point3{10.0, 0.0, 0.0}};
    const auto segmentHit = Intersection(segment, box);
    REQUIRE(segmentHit.has_value());
    CHECK(segmentHit->Enter == Approx(0.4));
    CHECK(segmentHit->Exit == Approx(0.6));

    const auto flat = Coordinate2::FromAxes(
        Point2{0.0, 3.0}, UnitVector2::XAxis, UnitVector2::YAxis);
    REQUIRE(flat.has_value());
    const OrientedBox2 box2{*flat, Vector2{1.0, 1.0}};
    const Ray2 upward{Point2{0.0, 0.0}, UnitVector2::YAxis};
    const auto hit2 = Intersection(upward, box2);
    REQUIRE(hit2.has_value());
    CHECK(hit2->Enter == 2.0);
    CHECK(Intersects(upward, box2));

    const Segment2 climb{Point2{0.0, 0.0}, Point2{0.0, 5.0}};
    CHECK(Intersection(climb, box2).has_value());
    CHECK(Intersects(climb, box2));
}

TEST_CASE("segment and triangle distances", "[query]") {
    const Segment3 first{Point3{0.0, 0.0, 0.0}, Point3{1.0, 0.0, 0.0}};
    const Segment3 second{Point3{0.0, 1.0, 1.0}, Point3{0.0, 1.0, 2.0}};
    CHECK(DistanceSquared(first, second) == 2.0);
    CHECK(Distance(first, second) == Approx(std::sqrt(2.0)));
    CHECK(Distance(first, second) >= 0.0);

    const Segment3 crossing{Point3{0.5, -1.0, 0.0}, Point3{0.5, 1.0, 0.0}};
    CHECK(DistanceSquared(first, crossing) == 0.0);
    CHECK(Distance(first, crossing) == 0.0);

    const Segment2 flatA{Point2{0.0, 0.0}, Point2{1.0, 0.0}};
    const Segment2 flatB{Point2{0.0, 2.0}, Point2{1.0, 2.0}};
    CHECK(DistanceSquared(flatA, flatB) == 4.0);
    CHECK(Distance(flatA, flatB) == 2.0);

    const Triangle3 lower{Point3{0.0, 0.0, 0.0}, Point3{1.0, 0.0, 0.0}, Point3{0.0, 1.0, 0.0}};
    const Triangle3 upper{Point3{0.0, 0.0, 2.0}, Point3{1.0, 0.0, 2.0}, Point3{0.0, 1.0, 2.0}};
    CHECK(DistanceSquared(lower, upper) == 4.0);
    CHECK(Distance(lower, upper) == 2.0);

    const Triangle3 piercing{Point3{0.2, 0.2, -1.0}, Point3{0.2, 0.2, 1.0}, Point3{2.0, 0.2, 0.0}};
    CHECK(DistanceSquared(lower, piercing) == 0.0);
    CHECK(Distance(lower, piercing) == 0.0);
}

TEST_CASE("linear queries are noexcept", "[query]") {
    const Line2 line2{Point2{0.0, 0.0}, UnitVector2::XAxis};
    const Segment2 segment2{Point2{0.0, 0.0}, Point2{1.0, 0.0}};
    const Ray2 ray2{Point2{0.0, 0.0}, UnitVector2::XAxis};
    const Box2 box2{Point2{0.0, 0.0}, Point2{1.0, 1.0}};
    const OrientedBox2 obb2{Coordinate2::Identity(), Vector2{1.0, 1.0}};

    const Line3 line3{Point3{0.0, 0.0, 0.0}, UnitVector3::ZAxis};
    const Segment3 segment3{Point3{0.0, 0.0, 0.0}, Point3{0.0, 0.0, 1.0}};
    const Ray3 ray3{Point3{0.0, 0.0, 0.0}, UnitVector3::ZAxis};
    const Plane plane{Point3{0.0, 0.0, 0.0}, UnitVector3::ZAxis};
    const Triangle3 triangle{Point3{0.0, 0.0, 0.0}, Point3{1.0, 0.0, 0.0}, Point3{0.0, 1.0, 0.0}};
    const Box3 box3{Point3{0.0, 0.0, 0.0}, Point3{1.0, 1.0, 1.0}};
    const OrientedBox3 obb3{Coordinate3::Identity(), Vector3{1.0, 1.0, 1.0}};

    STATIC_REQUIRE(noexcept(Intersects(line2, line2)));
    STATIC_REQUIRE(noexcept(Intersection(line2, line2)));
    STATIC_REQUIRE(noexcept(Intersects(segment2, segment2)));
    STATIC_REQUIRE(noexcept(Intersection(segment2, segment2)));
    STATIC_REQUIRE(noexcept(Intersects(line2, segment2)));
    STATIC_REQUIRE(noexcept(Intersection(line2, segment2)));
    STATIC_REQUIRE(noexcept(Intersects(ray3, plane)));
    STATIC_REQUIRE(noexcept(Intersection(ray3, plane)));
    STATIC_REQUIRE(noexcept(Intersects(segment3, plane)));
    STATIC_REQUIRE(noexcept(Intersection(segment3, plane)));
    STATIC_REQUIRE(noexcept(Intersects(line3, plane)));
    STATIC_REQUIRE(noexcept(Intersection(line3, plane)));
    STATIC_REQUIRE(noexcept(Intersects(ray3, triangle)));
    STATIC_REQUIRE(noexcept(Intersection(ray3, triangle)));
    STATIC_REQUIRE(noexcept(Intersects(segment3, triangle)));
    STATIC_REQUIRE(noexcept(Intersection(segment3, triangle)));
    STATIC_REQUIRE(noexcept(Intersects(line3, triangle)));
    STATIC_REQUIRE(noexcept(Intersection(line3, triangle)));
    STATIC_REQUIRE(noexcept(Intersects(ray2, box2)));
    STATIC_REQUIRE(noexcept(Intersection(ray2, box2)));
    STATIC_REQUIRE(noexcept(Intersects(segment2, box2)));
    STATIC_REQUIRE(noexcept(Intersection(segment2, box2)));
    STATIC_REQUIRE(noexcept(Intersects(line2, box2)));
    STATIC_REQUIRE(noexcept(Intersection(line2, box2)));
    STATIC_REQUIRE(noexcept(Intersects(ray3, box3)));
    STATIC_REQUIRE(noexcept(Intersection(ray3, box3)));
    STATIC_REQUIRE(noexcept(Intersects(segment3, box3)));
    STATIC_REQUIRE(noexcept(Intersection(segment3, box3)));
    STATIC_REQUIRE(noexcept(Intersects(line3, box3)));
    STATIC_REQUIRE(noexcept(Intersection(line3, box3)));
    STATIC_REQUIRE(noexcept(Intersects(ray2, obb2)));
    STATIC_REQUIRE(noexcept(Intersection(ray2, obb2)));
    STATIC_REQUIRE(noexcept(Intersects(segment2, obb2)));
    STATIC_REQUIRE(noexcept(Intersection(segment2, obb2)));
    STATIC_REQUIRE(noexcept(Intersects(ray3, obb3)));
    STATIC_REQUIRE(noexcept(Intersection(ray3, obb3)));
    STATIC_REQUIRE(noexcept(Intersects(segment3, obb3)));
    STATIC_REQUIRE(noexcept(Intersection(segment3, obb3)));
    STATIC_REQUIRE(noexcept(Distance(segment2, segment2)));
    STATIC_REQUIRE(noexcept(DistanceSquared(segment2, segment2)));
    STATIC_REQUIRE(noexcept(Distance(segment3, segment3)));
    STATIC_REQUIRE(noexcept(DistanceSquared(segment3, segment3)));
    STATIC_REQUIRE(noexcept(Distance(triangle, triangle)));
    STATIC_REQUIRE(noexcept(DistanceSquared(triangle, triangle)));
    CHECK(std::is_same_v<decltype(Intersection(line2, line2)), DragonGeo::Query::CurveMeet2>);
    CHECK(std::is_same_v<decltype(Intersection(ray3, plane)),
                         std::optional<DragonGeo::Query::ParameterPoint3>>);
    CHECK(std::is_same_v<decltype(Intersection(ray2, box2)),
                         std::optional<DragonGeo::Query::ParameterInterval2>>);
}
