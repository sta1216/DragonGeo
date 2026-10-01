#include <benchmark/benchmark.h>

#include <cmath>
#include <numbers>
#include <optional>

#include <DragonGeo/DragonGeo.hpp>

using DragonGeo::Linear::Box2;
using DragonGeo::Linear::Box3;
using DragonGeo::Linear::Coordinate2;
using DragonGeo::Linear::Coordinate3;
using DragonGeo::Linear::OrientedBox2;
using DragonGeo::Linear::OrientedBox3;
using DragonGeo::Linear::Point2;
using DragonGeo::Linear::Point3;
using DragonGeo::Linear::Transform3;
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
using DragonGeo::Query::Intersection;
using DragonGeo::Query::Intersects;
using DragonGeo::Query::Distance;
using DragonGeo::Query::DistanceSquared;

namespace {

template <typename Function, typename... Arguments> void Measure(benchmark::State& state, Function&& function, Arguments&... arguments) {
    for (auto _ : state) {
        (benchmark::DoNotOptimize(arguments), ...);
        benchmark::DoNotOptimize(function(arguments...));
    }
}

bool Expect(benchmark::State& state, bool ok, const char* message) {
    if (!ok) {
        state.SkipWithError(message);
    }
    return ok;
}

UnitVector2 AxisX2() { return UnitVector2::FromNormalizedUnchecked(Vector2{1.0, 0.0}); }

UnitVector2 AxisY2() { return UnitVector2::FromNormalizedUnchecked(Vector2{0.0, 1.0}); }

UnitVector2 Falling2() {
    const Vector2 raw{1.0, -0.1};
    const double length = std::sqrt(raw.LengthSquared());
    return UnitVector2::FromNormalizedUnchecked(Vector2{raw.X / length, raw.Y / length});
}

UnitVector3 AxisX3() { return UnitVector3::FromNormalizedUnchecked(Vector3{1.0, 0.0, 0.0}); }

UnitVector3 AxisZ3() { return UnitVector3::FromNormalizedUnchecked(Vector3{0.0, 0.0, 1.0}); }

Line2 LineAlongX() { return Line2{Point2{0.0, 0.0}, AxisX2()}; }

Line2 LineAlongY() { return Line2{Point2{0.0, 0.0}, AxisY2()}; }

Segment2 SegmentAlongX() { return Segment2{Point2{0.0, 0.0}, Point2{1.0, 0.0}}; }

Segment2 SegmentCrossingX() { return Segment2{Point2{0.5, -1.0}, Point2{0.5, 1.0}}; }

Box2 UnitBox2() { return Box2::FromCorners(Point2{0.0, 0.0}, Point2{2.0, 2.0}); }

Box3 UnitBox3() { return Box3::FromCorners(Point3{0.0, 0.0, 0.0}, Point3{2.0, 2.0, 2.0}); }

Plane PlaneZ() { return Plane{Point3{0.0, 0.0, 0.0}, AxisZ3()}; }

Triangle3 TriangleXY() { return Triangle3{Point3{0.0, 0.0, 0.0}, Point3{1.0, 0.0, 0.0}, Point3{0.0, 1.0, 0.0}}; }

std::optional<OrientedBox2> RotatedBox2() {
    const double component = std::sqrt(0.5);
    const auto axis = UnitVector2::FromNormalizedUnchecked(Vector2{component, component});
    const auto frame = Coordinate2::FromXAxis(Point2{0.0, 0.0}, axis);
    if (!frame.has_value()) {
        return std::nullopt;
    }
    return OrientedBox2{*frame, Vector2{1.0, 1.0}};
}

std::optional<OrientedBox3> RotatedBox3() {
    const auto frame = Coordinate3::FromTransform(Transform3::Rotation(AxisZ3(), std::numbers::pi / 4.0));
    if (!frame.has_value()) {
        return std::nullopt;
    }
    return OrientedBox3{*frame, Vector3{1.0, 1.0, 1.0}};
}

void QueryLine2Line2(benchmark::State& state) {
    Line2 first = LineAlongX();
    Line2 second = LineAlongY();
    if (!Expect(state, Intersection(first, second).Kind == CurveMeet::Point, "expected a point")) {
        return;
    }
    Measure(state, [](Line2& a, Line2& b) { return Intersection(a, b); }, first, second);
}
BENCHMARK(QueryLine2Line2);

void QueryLine2Line2Miss(benchmark::State& state) {
    Line2 first = LineAlongX();
    Line2 second{Point2{0.0, 1.0}, AxisX2()};
    if (!Expect(state, Intersection(first, second).Kind == CurveMeet::None, "expected a miss")) {
        return;
    }
    Measure(state, [](Line2& a, Line2& b) { return Intersection(a, b); }, first, second);
}
BENCHMARK(QueryLine2Line2Miss);

void QueryLine2Line2Coincident(benchmark::State& state) {
    Line2 first = LineAlongX();
    Line2 second{Point2{2.0, 0.0}, AxisX2()};
    if (!Expect(state, Intersection(first, second).Kind == CurveMeet::Coincident, "expected coincident lines")) {
        return;
    }
    Measure(state, [](Line2& a, Line2& b) { return Intersection(a, b); }, first, second);
}
BENCHMARK(QueryLine2Line2Coincident);

void QuerySegment2Segment2(benchmark::State& state) {
    Segment2 first = SegmentAlongX();
    Segment2 second = SegmentCrossingX();
    if (!Expect(state, Intersection(first, second).Kind == CurveMeet::Point, "expected a point")) {
        return;
    }
    Measure(state, [](Segment2& a, Segment2& b) { return Intersection(a, b); }, first, second);
}
BENCHMARK(QuerySegment2Segment2);

void QuerySegment2Segment2Miss(benchmark::State& state) {
    Segment2 first = SegmentAlongX();
    Segment2 second{Point2{0.0, 1.0}, Point2{1.0, 2.0}};
    if (!Expect(state, Intersection(first, second).Kind == CurveMeet::None, "expected a miss")) {
        return;
    }
    Measure(state, [](Segment2& a, Segment2& b) { return Intersection(a, b); }, first, second);
}
BENCHMARK(QuerySegment2Segment2Miss);

void QuerySegment2Segment2Overlap(benchmark::State& state) {
    Segment2 first = SegmentAlongX();
    Segment2 second{Point2{0.5, 0.0}, Point2{1.5, 0.0}};
    if (!Expect(state, Intersection(first, second).Kind == CurveMeet::Overlap, "expected an overlap")) {
        return;
    }
    Measure(state, [](Segment2& a, Segment2& b) { return Intersection(a, b); }, first, second);
}
BENCHMARK(QuerySegment2Segment2Overlap);

void QueryLine2Segment2(benchmark::State& state) {
    Line2 line = LineAlongX();
    Segment2 segment = SegmentCrossingX();
    if (!Expect(state, Intersection(line, segment).Kind == CurveMeet::Point, "expected a point")) {
        return;
    }
    Measure(state, [](Line2& a, Segment2& b) { return Intersection(a, b); }, line, segment);
}
BENCHMARK(QueryLine2Segment2);

void QueryLine2Segment2Miss(benchmark::State& state) {
    Line2 line = LineAlongX();
    Segment2 segment{Point2{0.0, 1.0}, Point2{1.0, 1.0}};
    if (!Expect(state, Intersection(line, segment).Kind == CurveMeet::None, "expected a miss")) {
        return;
    }
    Measure(state, [](Line2& a, Segment2& b) { return Intersection(a, b); }, line, segment);
}
BENCHMARK(QueryLine2Segment2Miss);

void QueryLine2Segment2Overlap(benchmark::State& state) {
    Line2 line = LineAlongX();
    Segment2 segment{Point2{1.0, 0.0}, Point2{2.0, 0.0}};
    if (!Expect(state, Intersection(line, segment).Kind == CurveMeet::Overlap, "expected an overlap")) {
        return;
    }
    Measure(state, [](Line2& a, Segment2& b) { return Intersection(a, b); }, line, segment);
}
BENCHMARK(QueryLine2Segment2Overlap);

void QueryRay2Box2(benchmark::State& state) {
    Ray2 ray{Point2{-1.0, 1.0}, AxisX2()};
    Box2 box = UnitBox2();
    if (!Expect(state, Intersection(ray, box).has_value(), "expected a hit")) {
        return;
    }
    Measure(state, [](Ray2& curve, Box2& solid) { return Intersection(curve, solid); }, ray, box);
}
BENCHMARK(QueryRay2Box2);

void QueryRay2Box2Miss(benchmark::State& state) {
    Ray2 ray{Point2{-1.0, 3.0}, Falling2()};
    Box2 box = UnitBox2();
    if (!Expect(state, !Intersection(ray, box).has_value(), "expected a miss")) {
        return;
    }
    Measure(state, [](Ray2& curve, Box2& solid) { return Intersection(curve, solid); }, ray, box);
}
BENCHMARK(QueryRay2Box2Miss);

void QueryRay2Box2AxisAligned(benchmark::State& state) {
    Ray2 ray{Point2{-1.0, 1.0}, AxisX2()};
    Box2 box = UnitBox2();
    if (!Expect(state, ray.Direction.Y() == 0.0 && Intersection(ray, box).has_value(), "expected an axis-aligned hit")) {
        return;
    }
    Measure(state, [](Ray2& curve, Box2& solid) { return Intersection(curve, solid); }, ray, box);
}
BENCHMARK(QueryRay2Box2AxisAligned);

void QuerySegment2Box2(benchmark::State& state) {
    Segment2 segment{Point2{-1.0, 1.0}, Point2{3.0, 1.0}};
    Box2 box = UnitBox2();
    if (!Expect(state, Intersection(segment, box).has_value(), "expected a hit")) {
        return;
    }
    Measure(state, [](Segment2& curve, Box2& solid) { return Intersection(curve, solid); }, segment, box);
}
BENCHMARK(QuerySegment2Box2);

void QueryLine2Box2(benchmark::State& state) {
    Line2 line{Point2{-1.0, 1.0}, AxisX2()};
    Box2 box = UnitBox2();
    if (!Expect(state, Intersection(line, box).has_value(), "expected a hit")) {
        return;
    }
    Measure(state, [](Line2& curve, Box2& solid) { return Intersection(curve, solid); }, line, box);
}
BENCHMARK(QueryLine2Box2);

void QueryRay2OrientedBox2(benchmark::State& state) {
    const auto box = RotatedBox2();
    if (!box.has_value()) {
        state.SkipWithError("frame construction failed");
        return;
    }
    Ray2 ray{Point2{-2.0, 0.0}, AxisX2()};
    OrientedBox2 solid = *box;
    if (!Expect(state, Intersection(ray, solid).has_value(), "expected a hit")) {
        return;
    }
    Measure(state, [](Ray2& curve, OrientedBox2& value) { return Intersection(curve, value); }, ray, solid);
}
BENCHMARK(QueryRay2OrientedBox2);

void QuerySegment2OrientedBox2(benchmark::State& state) {
    const auto box = RotatedBox2();
    if (!box.has_value()) {
        state.SkipWithError("frame construction failed");
        return;
    }
    Segment2 segment{Point2{-2.0, 0.0}, Point2{2.0, 0.0}};
    OrientedBox2 solid = *box;
    if (!Expect(state, Intersection(segment, solid).has_value(), "expected a hit")) {
        return;
    }
    Measure(state, [](Segment2& curve, OrientedBox2& value) { return Intersection(curve, value); }, segment, solid);
}
BENCHMARK(QuerySegment2OrientedBox2);

void QueryRay3Plane(benchmark::State& state) {
    Ray3 ray{Point3{0.0, 0.0, -1.0}, AxisZ3()};
    Plane plane = PlaneZ();
    if (!Expect(state, Intersection(ray, plane).has_value(), "expected a hit")) {
        return;
    }
    Measure(state, [](Ray3& curve, Plane& surface) { return Intersection(curve, surface); }, ray, plane);
}
BENCHMARK(QueryRay3Plane);

void QueryRay3PlaneMiss(benchmark::State& state) {
    Ray3 ray{Point3{0.0, 0.0, 1.0}, AxisX3()};
    Plane plane = PlaneZ();
    if (!Expect(state, !Intersects(ray, plane), "expected a miss")) {
        return;
    }
    Measure(state, [](Ray3& curve, Plane& surface) { return Intersection(curve, surface); }, ray, plane);
}
BENCHMARK(QueryRay3PlaneMiss);

void QueryRay3PlaneOnPlane(benchmark::State& state) {
    Ray3 ray{Point3{0.0, 0.0, 0.0}, AxisX3()};
    Plane plane = PlaneZ();
    if (!Expect(state, Intersects(ray, plane) && !Intersection(ray, plane).has_value(), "expected a ray lying on the plane")) {
        return;
    }
    Measure(state, [](Ray3& curve, Plane& surface) { return Intersection(curve, surface); }, ray, plane);
}
BENCHMARK(QueryRay3PlaneOnPlane);

void QuerySegment3Plane(benchmark::State& state) {
    Segment3 segment{Point3{0.0, 0.0, -1.0}, Point3{0.0, 0.0, 1.0}};
    Plane plane = PlaneZ();
    if (!Expect(state, Intersection(segment, plane).has_value(), "expected a hit")) {
        return;
    }
    Measure(state, [](Segment3& curve, Plane& surface) { return Intersection(curve, surface); }, segment, plane);
}
BENCHMARK(QuerySegment3Plane);

void QuerySegment3PlaneOutside(benchmark::State& state) {
    Segment3 segment{Point3{0.0, 0.0, 1.0}, Point3{0.0, 0.0, 2.0}};
    Plane plane = PlaneZ();
    if (!Expect(state, !Intersects(segment, plane), "expected the hit to fall outside the segment")) {
        return;
    }
    Measure(state, [](Segment3& curve, Plane& surface) { return Intersection(curve, surface); }, segment, plane);
}
BENCHMARK(QuerySegment3PlaneOutside);

void QueryLine3Plane(benchmark::State& state) {
    Line3 line{Point3{0.0, 0.0, -1.0}, AxisZ3()};
    Plane plane = PlaneZ();
    if (!Expect(state, Intersection(line, plane).Kind == CurveMeet::Point, "expected a point")) {
        return;
    }
    Measure(state, [](Line3& curve, Plane& surface) { return Intersection(curve, surface); }, line, plane);
}
BENCHMARK(QueryLine3Plane);

void QueryRay3Triangle3(benchmark::State& state) {
    Ray3 ray{Point3{0.2, 0.2, -1.0}, AxisZ3()};
    Triangle3 triangle = TriangleXY();
    if (!Expect(state, Intersection(ray, triangle).has_value(), "expected a hit")) {
        return;
    }
    Measure(state, [](Ray3& curve, Triangle3& face) { return Intersection(curve, face); }, ray, triangle);
}
BENCHMARK(QueryRay3Triangle3);

void QueryRay3Triangle3Miss(benchmark::State& state) {
    Ray3 ray{Point3{2.0, 2.0, -1.0}, AxisZ3()};
    Triangle3 triangle = TriangleXY();
    if (!Expect(state, !Intersects(ray, triangle), "expected a miss")) {
        return;
    }
    Measure(state, [](Ray3& curve, Triangle3& face) { return Intersection(curve, face); }, ray, triangle);
}
BENCHMARK(QueryRay3Triangle3Miss);

void QueryRay3Triangle3Coplanar(benchmark::State& state) {
    Ray3 ray{Point3{-1.0, 0.2, 0.0}, AxisX3()};
    Triangle3 triangle = TriangleXY();
    if (!Expect(state, Intersects(ray, triangle) && !Intersection(ray, triangle).has_value(), "expected a coplanar overlap")) {
        return;
    }
    Measure(state, [](Ray3& curve, Triangle3& face) { return Intersection(curve, face); }, ray, triangle);
}
BENCHMARK(QueryRay3Triangle3Coplanar);

void QuerySegment3Triangle3(benchmark::State& state) {
    Segment3 segment{Point3{0.2, 0.2, -1.0}, Point3{0.2, 0.2, 1.0}};
    Triangle3 triangle = TriangleXY();
    if (!Expect(state, Intersection(segment, triangle).has_value(), "expected a hit")) {
        return;
    }
    Measure(state, [](Segment3& curve, Triangle3& face) { return Intersection(curve, face); }, segment, triangle);
}
BENCHMARK(QuerySegment3Triangle3);

void QuerySegment3Triangle3Outside(benchmark::State& state) {
    Segment3 segment{Point3{0.2, 0.2, 1.0}, Point3{0.2, 0.2, 2.0}};
    Triangle3 triangle = TriangleXY();
    if (!Expect(state, !Intersects(segment, triangle), "expected the hit to fall outside the segment")) {
        return;
    }
    Measure(state, [](Segment3& curve, Triangle3& face) { return Intersection(curve, face); }, segment, triangle);
}
BENCHMARK(QuerySegment3Triangle3Outside);

void QueryLine3Triangle3(benchmark::State& state) {
    Line3 line{Point3{0.2, 0.2, -1.0}, AxisZ3()};
    Triangle3 triangle = TriangleXY();
    if (!Expect(state, Intersection(line, triangle).has_value(), "expected a hit")) {
        return;
    }
    Measure(state, [](Line3& curve, Triangle3& face) { return Intersection(curve, face); }, line, triangle);
}
BENCHMARK(QueryLine3Triangle3);

void QueryRay3Box3(benchmark::State& state) {
    Ray3 ray{Point3{-1.0, 1.0, 1.0}, AxisX3()};
    Box3 box = UnitBox3();
    if (!Expect(state, Intersection(ray, box).has_value(), "expected a hit")) {
        return;
    }
    Measure(state, [](Ray3& curve, Box3& solid) { return Intersection(curve, solid); }, ray, box);
}
BENCHMARK(QueryRay3Box3);

void QuerySegment3Box3(benchmark::State& state) {
    Segment3 segment{Point3{-1.0, 1.0, 1.0}, Point3{3.0, 1.0, 1.0}};
    Box3 box = UnitBox3();
    if (!Expect(state, Intersection(segment, box).has_value(), "expected a hit")) {
        return;
    }
    Measure(state, [](Segment3& curve, Box3& solid) { return Intersection(curve, solid); }, segment, box);
}
BENCHMARK(QuerySegment3Box3);

void QueryLine3Box3(benchmark::State& state) {
    Line3 line{Point3{-1.0, 1.0, 1.0}, AxisX3()};
    Box3 box = UnitBox3();
    if (!Expect(state, Intersection(line, box).has_value(), "expected a hit")) {
        return;
    }
    Measure(state, [](Line3& curve, Box3& solid) { return Intersection(curve, solid); }, line, box);
}
BENCHMARK(QueryLine3Box3);

void QueryRay3OrientedBox3(benchmark::State& state) {
    const auto box = RotatedBox3();
    if (!box.has_value()) {
        state.SkipWithError("frame construction failed");
        return;
    }
    Ray3 ray{Point3{-2.0, 0.0, 0.0}, AxisX3()};
    OrientedBox3 solid = *box;
    if (!Expect(state, Intersection(ray, solid).has_value(), "expected a hit")) {
        return;
    }
    Measure(state, [](Ray3& curve, OrientedBox3& value) { return Intersection(curve, value); }, ray, solid);
}
BENCHMARK(QueryRay3OrientedBox3);

void QuerySegment3OrientedBox3(benchmark::State& state) {
    const auto box = RotatedBox3();
    if (!box.has_value()) {
        state.SkipWithError("frame construction failed");
        return;
    }
    Segment3 segment{Point3{-2.0, 0.0, 0.0}, Point3{2.0, 0.0, 0.0}};
    OrientedBox3 solid = *box;
    if (!Expect(state, Intersection(segment, solid).has_value(), "expected a hit")) {
        return;
    }
    Measure(state, [](Segment3& curve, OrientedBox3& value) { return Intersection(curve, value); }, segment, solid);
}
BENCHMARK(QuerySegment3OrientedBox3);

void QueryDistanceSquaredSegment2Interior(benchmark::State& state) {
    Segment2 first = SegmentAlongX();
    Segment2 second = SegmentCrossingX();
    if (!Expect(state, DistanceSquared(first, second) == 0.0, "expected interior distance 0")) {
        return;
    }
    Measure(state, [](Segment2& a, Segment2& b) { return DistanceSquared(a, b); }, first, second);
}
BENCHMARK(QueryDistanceSquaredSegment2Interior);

void QueryDistanceSquaredSegment2Clamp(benchmark::State& state) {
    Segment2 first = SegmentAlongX();
    Segment2 second{Point2{2.0, 0.5}, Point2{2.0, 1.5}};
    if (!Expect(state, DistanceSquared(first, second) > 0.0, "expected a positive clamped distance")) {
        return;
    }
    Measure(state, [](Segment2& a, Segment2& b) { return DistanceSquared(a, b); }, first, second);
}
BENCHMARK(QueryDistanceSquaredSegment2Clamp);

void QueryDistanceSquaredSegment2Parallel(benchmark::State& state) {
    Segment2 first = SegmentAlongX();
    Segment2 second{Point2{0.0, 1.0}, Point2{1.0, 1.0}};
    if (!Expect(state, DistanceSquared(first, second) > 0.0, "expected a positive parallel distance")) {
        return;
    }
    Measure(state, [](Segment2& a, Segment2& b) { return DistanceSquared(a, b); }, first, second);
}
BENCHMARK(QueryDistanceSquaredSegment2Parallel);

void QueryDistanceSquaredSegment2Zero(benchmark::State& state) {
    Segment2 first{Point2{0.0, 0.0}, Point2{0.0, 0.0}};
    Segment2 second = SegmentAlongX();
    if (!Expect(state, DistanceSquared(first, second) == 0.0, "expected the zero-length point to lie on the other segment")) {
        return;
    }
    Measure(state, [](Segment2& a, Segment2& b) { return DistanceSquared(a, b); }, first, second);
}
BENCHMARK(QueryDistanceSquaredSegment2Zero);

void QueryDistanceSquaredSegment3Skew(benchmark::State& state) {
    Segment3 first{Point3{-1.0, 0.0, 0.0}, Point3{1.0, 0.0, 0.0}};
    Segment3 second{Point3{-1.0, 1.0, -1.0}, Point3{1.0, 1.0, 1.0}};
    if (!Expect(state, DistanceSquared(first, second) > 0.0, "expected a positive skew distance")) {
        return;
    }
    Measure(state, [](Segment3& a, Segment3& b) { return DistanceSquared(a, b); }, first, second);
}
BENCHMARK(QueryDistanceSquaredSegment3Skew);

void QueryDistanceSquaredTriangle3Intersect(benchmark::State& state) {
    Triangle3 first = TriangleXY();
    Triangle3 second{Point3{0.2, 0.1, -1.0}, Point3{0.2, 0.1, 1.0}, Point3{0.3, 0.2, 0.0}};
    if (!Expect(state, DistanceSquared(first, second) == 0.0, "expected intersecting triangles")) {
        return;
    }
    Measure(state, [](Triangle3& a, Triangle3& b) { return DistanceSquared(a, b); }, first, second);
}
BENCHMARK(QueryDistanceSquaredTriangle3Intersect);

void QueryDistanceSquaredTriangle3Separated(benchmark::State& state) {
    Triangle3 first = TriangleXY();
    Triangle3 second{Point3{0.0, 0.0, 1.0}, Point3{1.0, 0.0, 1.0}, Point3{0.0, 1.0, 1.0}};
    if (!Expect(state, DistanceSquared(first, second) > 0.0, "expected separated triangles")) {
        return;
    }
    Measure(state, [](Triangle3& a, Triangle3& b) { return DistanceSquared(a, b); }, first, second);
}
BENCHMARK(QueryDistanceSquaredTriangle3Separated);

void QueryDistanceSquaredTriangle3CoplanarOverlap(benchmark::State& state) {
    Triangle3 first = TriangleXY();
    Triangle3 second{Point3{0.1, 0.1, 0.0}, Point3{1.0, 0.0, 0.0}, Point3{0.0, 1.0, 0.0}};
    if (!Expect(state, DistanceSquared(first, second) == 0.0, "expected coplanar overlap")) {
        return;
    }
    Measure(state, [](Triangle3& a, Triangle3& b) { return DistanceSquared(a, b); }, first, second);
}
BENCHMARK(QueryDistanceSquaredTriangle3CoplanarOverlap);

void QueryDistanceSquaredTriangle3CoplanarSeparated(benchmark::State& state) {
    Triangle3 first = TriangleXY();
    Triangle3 second{Point3{3.0, 0.0, 0.0}, Point3{4.0, 0.0, 0.0}, Point3{3.0, 1.0, 0.0}};
    if (!Expect(state, DistanceSquared(first, second) > 0.0, "expected coplanar separation")) {
        return;
    }
    Measure(state, [](Triangle3& a, Triangle3& b) { return DistanceSquared(a, b); }, first, second);
}
BENCHMARK(QueryDistanceSquaredTriangle3CoplanarSeparated);

void QueryIntersectsLine2Line2(benchmark::State& state) {
    Line2 first = LineAlongX();
    Line2 second = LineAlongY();
    if (!Expect(state, Intersects(first, second), "expected a hit")) {
        return;
    }
    Measure(state, [](Line2& a, Line2& b) { return Intersects(a, b); }, first, second);
}
BENCHMARK(QueryIntersectsLine2Line2);

void QueryIntersectsSegment2Segment2(benchmark::State& state) {
    Segment2 first = SegmentAlongX();
    Segment2 second = SegmentCrossingX();
    if (!Expect(state, Intersects(first, second), "expected a hit")) {
        return;
    }
    Measure(state, [](Segment2& a, Segment2& b) { return Intersects(a, b); }, first, second);
}
BENCHMARK(QueryIntersectsSegment2Segment2);

void QueryIntersectsLine2Segment2(benchmark::State& state) {
    Line2 line = LineAlongX();
    Segment2 segment = SegmentCrossingX();
    if (!Expect(state, Intersects(line, segment), "expected a hit")) {
        return;
    }
    Measure(state, [](Line2& a, Segment2& b) { return Intersects(a, b); }, line, segment);
}
BENCHMARK(QueryIntersectsLine2Segment2);

void QueryIntersectsRay2Box2(benchmark::State& state) {
    Ray2 ray{Point2{-1.0, 1.0}, AxisX2()};
    Box2 box = UnitBox2();
    if (!Expect(state, Intersects(ray, box), "expected a hit")) {
        return;
    }
    Measure(state, [](Ray2& curve, Box2& solid) { return Intersects(curve, solid); }, ray, box);
}
BENCHMARK(QueryIntersectsRay2Box2);

void QueryIntersectsSegment2Box2(benchmark::State& state) {
    Segment2 segment{Point2{-1.0, 1.0}, Point2{3.0, 1.0}};
    Box2 box = UnitBox2();
    if (!Expect(state, Intersects(segment, box), "expected a hit")) {
        return;
    }
    Measure(state, [](Segment2& curve, Box2& solid) { return Intersects(curve, solid); }, segment, box);
}
BENCHMARK(QueryIntersectsSegment2Box2);

void QueryIntersectsLine2Box2(benchmark::State& state) {
    Line2 line{Point2{-1.0, 1.0}, AxisX2()};
    Box2 box = UnitBox2();
    if (!Expect(state, Intersects(line, box), "expected a hit")) {
        return;
    }
    Measure(state, [](Line2& curve, Box2& solid) { return Intersects(curve, solid); }, line, box);
}
BENCHMARK(QueryIntersectsLine2Box2);

void QueryIntersectsRay2OrientedBox2(benchmark::State& state) {
    const auto box = RotatedBox2();
    if (!box.has_value()) {
        state.SkipWithError("frame construction failed");
        return;
    }
    Ray2 ray{Point2{-2.0, 0.0}, AxisX2()};
    OrientedBox2 solid = *box;
    if (!Expect(state, Intersects(ray, solid), "expected a hit")) {
        return;
    }
    Measure(state, [](Ray2& curve, OrientedBox2& value) { return Intersects(curve, value); }, ray, solid);
}
BENCHMARK(QueryIntersectsRay2OrientedBox2);

void QueryIntersectsSegment2OrientedBox2(benchmark::State& state) {
    const auto box = RotatedBox2();
    if (!box.has_value()) {
        state.SkipWithError("frame construction failed");
        return;
    }
    Segment2 segment{Point2{-2.0, 0.0}, Point2{2.0, 0.0}};
    OrientedBox2 solid = *box;
    if (!Expect(state, Intersects(segment, solid), "expected a hit")) {
        return;
    }
    Measure(state, [](Segment2& curve, OrientedBox2& value) { return Intersects(curve, value); }, segment, solid);
}
BENCHMARK(QueryIntersectsSegment2OrientedBox2);

void QueryIntersectsRay3Plane(benchmark::State& state) {
    Ray3 ray{Point3{0.0, 0.0, -1.0}, AxisZ3()};
    Plane plane = PlaneZ();
    if (!Expect(state, Intersects(ray, plane), "expected a hit")) {
        return;
    }
    Measure(state, [](Ray3& curve, Plane& surface) { return Intersects(curve, surface); }, ray, plane);
}
BENCHMARK(QueryIntersectsRay3Plane);

void QueryIntersectsSegment3Plane(benchmark::State& state) {
    Segment3 segment{Point3{0.0, 0.0, -1.0}, Point3{0.0, 0.0, 1.0}};
    Plane plane = PlaneZ();
    if (!Expect(state, Intersects(segment, plane), "expected a hit")) {
        return;
    }
    Measure(state, [](Segment3& curve, Plane& surface) { return Intersects(curve, surface); }, segment, plane);
}
BENCHMARK(QueryIntersectsSegment3Plane);

void QueryIntersectsLine3Plane(benchmark::State& state) {
    Line3 line{Point3{0.0, 0.0, -1.0}, AxisZ3()};
    Plane plane = PlaneZ();
    if (!Expect(state, Intersects(line, plane), "expected a hit")) {
        return;
    }
    Measure(state, [](Line3& curve, Plane& surface) { return Intersects(curve, surface); }, line, plane);
}
BENCHMARK(QueryIntersectsLine3Plane);

void QueryIntersectsRay3Triangle3(benchmark::State& state) {
    Ray3 ray{Point3{0.2, 0.2, -1.0}, AxisZ3()};
    Triangle3 triangle = TriangleXY();
    if (!Expect(state, Intersects(ray, triangle), "expected a hit")) {
        return;
    }
    Measure(state, [](Ray3& curve, Triangle3& face) { return Intersects(curve, face); }, ray, triangle);
}
BENCHMARK(QueryIntersectsRay3Triangle3);

void QueryIntersectsSegment3Triangle3(benchmark::State& state) {
    Segment3 segment{Point3{0.2, 0.2, -1.0}, Point3{0.2, 0.2, 1.0}};
    Triangle3 triangle = TriangleXY();
    if (!Expect(state, Intersects(segment, triangle), "expected a hit")) {
        return;
    }
    Measure(state, [](Segment3& curve, Triangle3& face) { return Intersects(curve, face); }, segment, triangle);
}
BENCHMARK(QueryIntersectsSegment3Triangle3);

void QueryIntersectsLine3Triangle3(benchmark::State& state) {
    Line3 line{Point3{0.2, 0.2, -1.0}, AxisZ3()};
    Triangle3 triangle = TriangleXY();
    if (!Expect(state, Intersects(line, triangle), "expected a hit")) {
        return;
    }
    Measure(state, [](Line3& curve, Triangle3& face) { return Intersects(curve, face); }, line, triangle);
}
BENCHMARK(QueryIntersectsLine3Triangle3);

void QueryIntersectsRay3Box3(benchmark::State& state) {
    Ray3 ray{Point3{-1.0, 1.0, 1.0}, AxisX3()};
    Box3 box = UnitBox3();
    if (!Expect(state, Intersects(ray, box), "expected a hit")) {
        return;
    }
    Measure(state, [](Ray3& curve, Box3& solid) { return Intersects(curve, solid); }, ray, box);
}
BENCHMARK(QueryIntersectsRay3Box3);

void QueryIntersectsSegment3Box3(benchmark::State& state) {
    Segment3 segment{Point3{-1.0, 1.0, 1.0}, Point3{3.0, 1.0, 1.0}};
    Box3 box = UnitBox3();
    if (!Expect(state, Intersects(segment, box), "expected a hit")) {
        return;
    }
    Measure(state, [](Segment3& curve, Box3& solid) { return Intersects(curve, solid); }, segment, box);
}
BENCHMARK(QueryIntersectsSegment3Box3);

void QueryIntersectsLine3Box3(benchmark::State& state) {
    Line3 line{Point3{-1.0, 1.0, 1.0}, AxisX3()};
    Box3 box = UnitBox3();
    if (!Expect(state, Intersects(line, box), "expected a hit")) {
        return;
    }
    Measure(state, [](Line3& curve, Box3& solid) { return Intersects(curve, solid); }, line, box);
}
BENCHMARK(QueryIntersectsLine3Box3);

void QueryIntersectsRay3OrientedBox3(benchmark::State& state) {
    const auto box = RotatedBox3();
    if (!box.has_value()) {
        state.SkipWithError("frame construction failed");
        return;
    }
    Ray3 ray{Point3{-2.0, 0.0, 0.0}, AxisX3()};
    OrientedBox3 solid = *box;
    if (!Expect(state, Intersects(ray, solid), "expected a hit")) {
        return;
    }
    Measure(state, [](Ray3& curve, OrientedBox3& value) { return Intersects(curve, value); }, ray, solid);
}
BENCHMARK(QueryIntersectsRay3OrientedBox3);

void QueryIntersectsSegment3OrientedBox3(benchmark::State& state) {
    const auto box = RotatedBox3();
    if (!box.has_value()) {
        state.SkipWithError("frame construction failed");
        return;
    }
    Segment3 segment{Point3{-2.0, 0.0, 0.0}, Point3{2.0, 0.0, 0.0}};
    OrientedBox3 solid = *box;
    if (!Expect(state, Intersects(segment, solid), "expected a hit")) {
        return;
    }
    Measure(state, [](Segment3& curve, OrientedBox3& value) { return Intersects(curve, value); }, segment, solid);
}
BENCHMARK(QueryIntersectsSegment3OrientedBox3);

void QueryDistanceSegment2(benchmark::State& state) {
    Segment2 first = SegmentAlongX();
    Segment2 second{Point2{0.0, 1.0}, Point2{1.0, 1.0}};
    if (!Expect(state, Distance(first, second) > 0.0, "expected a positive distance")) {
        return;
    }
    Measure(state, [](Segment2& a, Segment2& b) { return Distance(a, b); }, first, second);
}
BENCHMARK(QueryDistanceSegment2);

void QueryDistanceSegment3(benchmark::State& state) {
    Segment3 first{Point3{-1.0, 0.0, 0.0}, Point3{1.0, 0.0, 0.0}};
    Segment3 second{Point3{-1.0, 1.0, -1.0}, Point3{1.0, 1.0, 1.0}};
    if (!Expect(state, Distance(first, second) > 0.0, "expected a positive distance")) {
        return;
    }
    Measure(state, [](Segment3& a, Segment3& b) { return Distance(a, b); }, first, second);
}
BENCHMARK(QueryDistanceSegment3);

void QueryDistanceTriangle3(benchmark::State& state) {
    Triangle3 first = TriangleXY();
    Triangle3 second{Point3{0.0, 0.0, 1.0}, Point3{1.0, 0.0, 1.0}, Point3{0.0, 1.0, 1.0}};
    if (!Expect(state, Distance(first, second) > 0.0, "expected a positive distance")) {
        return;
    }
    Measure(state, [](Triangle3& a, Triangle3& b) { return Distance(a, b); }, first, second);
}
BENCHMARK(QueryDistanceTriangle3);

} // namespace
