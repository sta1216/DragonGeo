#include <benchmark/benchmark.h>

#include <cmath>
#include <numbers>
#include <stdexcept>
#include <vector>

#include <DragonGeo/DragonGeo.hpp>

using DragonGeo::Linear::Box2;
using DragonGeo::Linear::Box3;
using DragonGeo::Linear::Point2;
using DragonGeo::Linear::Point3;
using DragonGeo::Linear::Transform2;
using DragonGeo::Linear::Transform3;
using DragonGeo::Linear::UnitVector2;
using DragonGeo::Linear::UnitVector3;
using DragonGeo::Linear::Vector2;
using DragonGeo::Linear::Vector3;
using DragonGeo::Prim::Line2;
using DragonGeo::Prim::Line3;
using DragonGeo::Prim::Plane;
using DragonGeo::Prim::Polyline;
using DragonGeo::Prim::Polyline3;
using DragonGeo::Prim::Ray2;
using DragonGeo::Prim::Ray3;
using DragonGeo::Prim::Segment2;
using DragonGeo::Prim::Segment3;
using DragonGeo::Prim::Triangle2;
using DragonGeo::Prim::Triangle3;

namespace {

template <typename Function, typename... Arguments> void Measure(benchmark::State& state, Function&& function, Arguments&... arguments) {
    for (auto _ : state) {
        (benchmark::DoNotOptimize(arguments), ...);
        benchmark::DoNotOptimize(function(arguments...));
    }
}

UnitVector2 AxisX2() { return UnitVector2::FromNormalizedUnchecked(Vector2{1.0, 0.0}); }

UnitVector3 AxisZ3() { return UnitVector3::FromNormalizedUnchecked(Vector3{0.0, 0.0, 1.0}); }

Polyline Circle2(int count) {
    std::vector<Point2> points;
    points.reserve(static_cast<std::size_t>(count));
    const int unique = count - 1;
    for (int index = 0; index < unique; ++index) {
        const double angle = (2.0 * std::numbers::pi * index) / unique;
        points.push_back(Point2{std::cos(angle), std::sin(angle)});
    }
    points.push_back(points.front());
    auto shape = Polyline::FromPoints(points);
    if (!shape.has_value() || !shape->Contains(Point2{0.0, 0.0})) {
        throw std::logic_error("circle polyline is not a closed disk");
    }
    return std::move(*shape);
}

Polyline3 Circle3(int count) {
    std::vector<Point3> points;
    points.reserve(static_cast<std::size_t>(count));
    const int unique = count - 1;
    for (int index = 0; index < unique; ++index) {
        const double angle = (2.0 * std::numbers::pi * index) / unique;
        points.push_back(Point3{std::cos(angle), std::sin(angle), 0.0});
    }
    points.push_back(points.front());
    auto shape = Polyline3::FromPoints(points);
    if (!shape.has_value() || !shape->Contains(Point3{0.0, 0.0, 0.0})) {
        throw std::logic_error("circle polyline is not a closed disk");
    }
    return std::move(*shape);
}

template <typename Curve> Point2 EdgeMidpoint2(const Curve& curve) {
    const Point2 start = curve.Point(0);
    const Point2 end = curve.Point(1);
    return Point2{(start.X + end.X) * 0.5, (start.Y + end.Y) * 0.5};
}

template <typename Curve> Point3 EdgeMidpoint3(const Curve& curve) {
    const Point3 start = curve.Point(0);
    const Point3 end = curve.Point(1);
    return Point3{(start.X + end.X) * 0.5, (start.Y + end.Y) * 0.5, (start.Z + end.Z) * 0.5};
}

void PrimSegment2ClosestPoint(benchmark::State& state) {
    Segment2 segment{Point2{0.0, 0.0}, Point2{1.0, 0.0}};
    Point2 point{0.25, 1.0};
    Measure(state, [](Segment2& curve, Point2& query) { return curve.ClosestPoint(query); }, segment, point);
}
BENCHMARK(PrimSegment2ClosestPoint);

void PrimSegment2DistanceSquared(benchmark::State& state) {
    Segment2 segment{Point2{0.0, 0.0}, Point2{1.0, 0.0}};
    Point2 point{0.25, 1.0};
    Measure(state, [](Segment2& curve, Point2& query) { return curve.DistanceSquared(query); }, segment, point);
}
BENCHMARK(PrimSegment2DistanceSquared);

void PrimSegment2ContainsPoint(benchmark::State& state) {
    Segment2 segment{Point2{0.0, 0.0}, Point2{1.0, 0.0}};
    Point2 point{0.3, 0.0};
    Measure(state, [](Segment2& curve, Point2& query) { return curve.ContainsPoint(query); }, segment, point);
}
BENCHMARK(PrimSegment2ContainsPoint);

void PrimSegment2ParameterOf(benchmark::State& state) {
    Segment2 segment{Point2{0.0, 0.0}, Point2{1.0, 0.0}};
    Point2 point{0.3, 0.0};
    Measure(state, [](Segment2& curve, Point2& query) { return curve.ParameterOf(query); }, segment, point);
}
BENCHMARK(PrimSegment2ParameterOf);

void PrimSegment2Box(benchmark::State& state) {
    Segment2 segment{Point2{0.0, 0.0}, Point2{1.0, 0.0}};
    Measure(state, [](Segment2& curve) { return curve.Box(); }, segment);
}
BENCHMARK(PrimSegment2Box);

void PrimSegment2Transform(benchmark::State& state) {
    const Segment2 original{Point2{0.0, 0.0}, Point2{1.0, 0.0}};
    const Transform2 rotation = Transform2::RotationAbout(Point2{0.0, 0.0}, 0.3);
    Segment2 probe = original;
    if (!probe.Transform(rotation)) {
        state.SkipWithError("transform failed");
        return;
    }
    for (auto _ : state) {
        Segment2 copy = original;
        benchmark::DoNotOptimize(copy);
        benchmark::DoNotOptimize(rotation);
        benchmark::DoNotOptimize(copy.Transform(rotation));
        benchmark::DoNotOptimize(copy);
    }
}
BENCHMARK(PrimSegment2Transform);

void PrimSegment3ClosestPoint(benchmark::State& state) {
    Segment3 segment{Point3{0.0, 0.0, 0.0}, Point3{1.0, 0.0, 0.0}};
    Point3 point{0.25, 1.0, 0.0};
    Measure(state, [](Segment3& curve, Point3& query) { return curve.ClosestPoint(query); }, segment, point);
}
BENCHMARK(PrimSegment3ClosestPoint);

void PrimSegment3DistanceSquared(benchmark::State& state) {
    Segment3 segment{Point3{0.0, 0.0, 0.0}, Point3{1.0, 0.0, 0.0}};
    Point3 point{0.25, 1.0, 0.0};
    Measure(state, [](Segment3& curve, Point3& query) { return curve.DistanceSquared(query); }, segment, point);
}
BENCHMARK(PrimSegment3DistanceSquared);

void PrimSegment3ContainsPoint(benchmark::State& state) {
    Segment3 segment{Point3{0.0, 0.0, 0.0}, Point3{1.0, 0.0, 0.0}};
    Point3 point{0.3, 0.0, 0.0};
    Measure(state, [](Segment3& curve, Point3& query) { return curve.ContainsPoint(query); }, segment, point);
}
BENCHMARK(PrimSegment3ContainsPoint);

void PrimSegment3ParameterOf(benchmark::State& state) {
    Segment3 segment{Point3{0.0, 0.0, 0.0}, Point3{1.0, 0.0, 0.0}};
    Point3 point{0.3, 0.0, 0.0};
    Measure(state, [](Segment3& curve, Point3& query) { return curve.ParameterOf(query); }, segment, point);
}
BENCHMARK(PrimSegment3ParameterOf);

void PrimSegment3Box(benchmark::State& state) {
    Segment3 segment{Point3{0.0, 0.0, 0.0}, Point3{1.0, 0.0, 0.0}};
    Measure(state, [](Segment3& curve) { return curve.Box(); }, segment);
}
BENCHMARK(PrimSegment3Box);

void PrimSegment3Transform(benchmark::State& state) {
    const Segment3 original{Point3{0.0, 0.0, 0.0}, Point3{1.0, 0.0, 0.0}};
    const Transform3 rotation = Transform3::RotationAbout(Point3{0.0, 0.0, 0.0}, AxisZ3(), 0.3);
    Segment3 probe = original;
    if (!probe.Transform(rotation)) {
        state.SkipWithError("transform failed");
        return;
    }
    for (auto _ : state) {
        Segment3 copy = original;
        benchmark::DoNotOptimize(copy);
        benchmark::DoNotOptimize(rotation);
        benchmark::DoNotOptimize(copy.Transform(rotation));
        benchmark::DoNotOptimize(copy);
    }
}
BENCHMARK(PrimSegment3Transform);

void PrimRay2ClosestPoint(benchmark::State& state) {
    Ray2 ray{Point2{0.0, 0.0}, AxisX2()};
    Point2 point{0.5, 1.0};
    Measure(state, [](Ray2& curve, Point2& query) { return curve.ClosestPoint(query); }, ray, point);
}
BENCHMARK(PrimRay2ClosestPoint);

void PrimRay2DistanceSquared(benchmark::State& state) {
    Ray2 ray{Point2{0.0, 0.0}, AxisX2()};
    Point2 point{0.5, 1.0};
    Measure(state, [](Ray2& curve, Point2& query) { return curve.DistanceSquared(query); }, ray, point);
}
BENCHMARK(PrimRay2DistanceSquared);

void PrimRay2ContainsPoint(benchmark::State& state) {
    Ray2 ray{Point2{0.0, 0.0}, AxisX2()};
    Point2 point{0.5, 0.0};
    Measure(state, [](Ray2& curve, Point2& query) { return curve.ContainsPoint(query); }, ray, point);
}
BENCHMARK(PrimRay2ContainsPoint);

void PrimRay2ParameterOf(benchmark::State& state) {
    Ray2 ray{Point2{0.0, 0.0}, AxisX2()};
    Point2 point{0.5, 0.0};
    Measure(state, [](Ray2& curve, Point2& query) { return curve.ParameterOf(query); }, ray, point);
}
BENCHMARK(PrimRay2ParameterOf);

void PrimRay2Box(benchmark::State& state) {
    Ray2 ray{Point2{0.0, 0.0}, AxisX2()};
    Measure(state, [](Ray2& curve) { return curve.Box(); }, ray);
}
BENCHMARK(PrimRay2Box);

void PrimRay2Transform(benchmark::State& state) {
    const Ray2 original{Point2{0.0, 0.0}, AxisX2()};
    const Transform2 rotation = Transform2::RotationAbout(Point2{0.0, 0.0}, 0.3);
    Ray2 probe = original;
    if (!probe.Transform(rotation)) {
        state.SkipWithError("transform failed");
        return;
    }
    for (auto _ : state) {
        Ray2 copy = original;
        benchmark::DoNotOptimize(copy);
        benchmark::DoNotOptimize(rotation);
        benchmark::DoNotOptimize(copy.Transform(rotation));
        benchmark::DoNotOptimize(copy);
    }
}
BENCHMARK(PrimRay2Transform);

void PrimRay3ClosestPoint(benchmark::State& state) {
    Ray3 ray{Point3{0.0, 0.0, 0.0}, AxisZ3()};
    Point3 point{1.0, 0.0, 0.5};
    Measure(state, [](Ray3& curve, Point3& query) { return curve.ClosestPoint(query); }, ray, point);
}
BENCHMARK(PrimRay3ClosestPoint);

void PrimRay3DistanceSquared(benchmark::State& state) {
    Ray3 ray{Point3{0.0, 0.0, 0.0}, AxisZ3()};
    Point3 point{1.0, 0.0, 0.5};
    Measure(state, [](Ray3& curve, Point3& query) { return curve.DistanceSquared(query); }, ray, point);
}
BENCHMARK(PrimRay3DistanceSquared);

void PrimRay3ContainsPoint(benchmark::State& state) {
    Ray3 ray{Point3{0.0, 0.0, 0.0}, AxisZ3()};
    Point3 point{0.0, 0.0, 0.5};
    Measure(state, [](Ray3& curve, Point3& query) { return curve.ContainsPoint(query); }, ray, point);
}
BENCHMARK(PrimRay3ContainsPoint);

void PrimRay3ParameterOf(benchmark::State& state) {
    Ray3 ray{Point3{0.0, 0.0, 0.0}, AxisZ3()};
    Point3 point{0.0, 0.0, 0.5};
    Measure(state, [](Ray3& curve, Point3& query) { return curve.ParameterOf(query); }, ray, point);
}
BENCHMARK(PrimRay3ParameterOf);

void PrimRay3Box(benchmark::State& state) {
    Ray3 ray{Point3{0.0, 0.0, 0.0}, AxisZ3()};
    Measure(state, [](Ray3& curve) { return curve.Box(); }, ray);
}
BENCHMARK(PrimRay3Box);

void PrimRay3Transform(benchmark::State& state) {
    const Ray3 original{Point3{0.0, 0.0, 0.0}, AxisZ3()};
    const auto axis = UnitVector3::FromNormalizedUnchecked(Vector3{0.0, 1.0, 0.0});
    const Transform3 rotation = Transform3::RotationAbout(Point3{0.0, 0.0, 0.0}, axis, 0.3);
    Ray3 probe = original;
    if (!probe.Transform(rotation)) {
        state.SkipWithError("transform failed");
        return;
    }
    for (auto _ : state) {
        Ray3 copy = original;
        benchmark::DoNotOptimize(copy);
        benchmark::DoNotOptimize(rotation);
        benchmark::DoNotOptimize(copy.Transform(rotation));
        benchmark::DoNotOptimize(copy);
    }
}
BENCHMARK(PrimRay3Transform);

void PrimLine2ClosestPoint(benchmark::State& state) {
    Line2 line{Point2{0.0, 0.0}, AxisX2()};
    Point2 point{0.5, 1.0};
    Measure(state, [](Line2& curve, Point2& query) { return curve.ClosestPoint(query); }, line, point);
}
BENCHMARK(PrimLine2ClosestPoint);

void PrimLine2DistanceSquared(benchmark::State& state) {
    Line2 line{Point2{0.0, 0.0}, AxisX2()};
    Point2 point{0.5, 1.0};
    Measure(state, [](Line2& curve, Point2& query) { return curve.DistanceSquared(query); }, line, point);
}
BENCHMARK(PrimLine2DistanceSquared);

void PrimLine2ContainsPoint(benchmark::State& state) {
    Line2 line{Point2{0.0, 0.0}, AxisX2()};
    Point2 point{0.5, 0.0};
    Measure(state, [](Line2& curve, Point2& query) { return curve.ContainsPoint(query); }, line, point);
}
BENCHMARK(PrimLine2ContainsPoint);

void PrimLine2ParameterOf(benchmark::State& state) {
    Line2 line{Point2{0.0, 0.0}, AxisX2()};
    Point2 point{0.5, 0.0};
    Measure(state, [](Line2& curve, Point2& query) { return curve.ParameterOf(query); }, line, point);
}
BENCHMARK(PrimLine2ParameterOf);

void PrimLine2Box(benchmark::State& state) {
    Line2 line{Point2{0.0, 0.0}, AxisX2()};
    Measure(state, [](Line2& curve) { return curve.Box(); }, line);
}
BENCHMARK(PrimLine2Box);

void PrimLine2Transform(benchmark::State& state) {
    const Line2 original{Point2{0.0, 0.0}, AxisX2()};
    const Transform2 rotation = Transform2::RotationAbout(Point2{0.0, 0.0}, 0.3);
    Line2 probe = original;
    if (!probe.Transform(rotation)) {
        state.SkipWithError("transform failed");
        return;
    }
    for (auto _ : state) {
        Line2 copy = original;
        benchmark::DoNotOptimize(copy);
        benchmark::DoNotOptimize(rotation);
        benchmark::DoNotOptimize(copy.Transform(rotation));
        benchmark::DoNotOptimize(copy);
    }
}
BENCHMARK(PrimLine2Transform);

void PrimLine3ClosestPoint(benchmark::State& state) {
    Line3 line{Point3{0.0, 0.0, 0.0}, AxisZ3()};
    Point3 point{1.0, 0.0, 0.5};
    Measure(state, [](Line3& curve, Point3& query) { return curve.ClosestPoint(query); }, line, point);
}
BENCHMARK(PrimLine3ClosestPoint);

void PrimLine3DistanceSquared(benchmark::State& state) {
    Line3 line{Point3{0.0, 0.0, 0.0}, AxisZ3()};
    Point3 point{1.0, 0.0, 0.5};
    Measure(state, [](Line3& curve, Point3& query) { return curve.DistanceSquared(query); }, line, point);
}
BENCHMARK(PrimLine3DistanceSquared);

void PrimLine3ContainsPoint(benchmark::State& state) {
    Line3 line{Point3{0.0, 0.0, 0.0}, AxisZ3()};
    Point3 point{0.0, 0.0, 0.5};
    Measure(state, [](Line3& curve, Point3& query) { return curve.ContainsPoint(query); }, line, point);
}
BENCHMARK(PrimLine3ContainsPoint);

void PrimLine3ParameterOf(benchmark::State& state) {
    Line3 line{Point3{0.0, 0.0, 0.0}, AxisZ3()};
    Point3 point{0.0, 0.0, 0.5};
    Measure(state, [](Line3& curve, Point3& query) { return curve.ParameterOf(query); }, line, point);
}
BENCHMARK(PrimLine3ParameterOf);

void PrimLine3Box(benchmark::State& state) {
    Line3 line{Point3{0.0, 0.0, 0.0}, AxisZ3()};
    Measure(state, [](Line3& curve) { return curve.Box(); }, line);
}
BENCHMARK(PrimLine3Box);

void PrimLine3Transform(benchmark::State& state) {
    const Line3 original{Point3{0.0, 0.0, 0.0}, AxisZ3()};
    const auto axis = UnitVector3::FromNormalizedUnchecked(Vector3{0.0, 1.0, 0.0});
    const Transform3 rotation = Transform3::RotationAbout(Point3{0.0, 0.0, 0.0}, axis, 0.3);
    Line3 probe = original;
    if (!probe.Transform(rotation)) {
        state.SkipWithError("transform failed");
        return;
    }
    for (auto _ : state) {
        Line3 copy = original;
        benchmark::DoNotOptimize(copy);
        benchmark::DoNotOptimize(rotation);
        benchmark::DoNotOptimize(copy.Transform(rotation));
        benchmark::DoNotOptimize(copy);
    }
}
BENCHMARK(PrimLine3Transform);

void PrimTriangle2Contains(benchmark::State& state) {
    Triangle2 triangle{Point2{0.0, 0.0}, Point2{1.0, 0.0}, Point2{0.0, 1.0}};
    Point2 point{0.2, 0.2};
    Measure(state, [](Triangle2& curve, Point2& query) { return curve.Contains(query); }, triangle, point);
}
BENCHMARK(PrimTriangle2Contains);

void PrimTriangle2ContainsPoint(benchmark::State& state) {
    Triangle2 triangle{Point2{0.0, 0.0}, Point2{1.0, 0.0}, Point2{0.0, 1.0}};
    Point2 point{0.5, 0.0};
    Measure(state, [](Triangle2& curve, Point2& query) { return curve.ContainsPoint(query); }, triangle, point);
}
BENCHMARK(PrimTriangle2ContainsPoint);

void PrimTriangle2ClosestPoint(benchmark::State& state) {
    Triangle2 triangle{Point2{0.0, 0.0}, Point2{1.0, 0.0}, Point2{0.0, 1.0}};
    Point2 point{0.5, -1.0};
    Measure(state, [](Triangle2& curve, Point2& query) { return curve.ClosestPoint(query); }, triangle, point);
}
BENCHMARK(PrimTriangle2ClosestPoint);

void PrimTriangle2Area(benchmark::State& state) {
    Triangle2 triangle{Point2{0.0, 0.0}, Point2{1.0, 0.0}, Point2{0.0, 1.0}};
    Measure(state, [](Triangle2& curve) { return curve.Area(); }, triangle);
}
BENCHMARK(PrimTriangle2Area);

void PrimTriangle2ParameterOf(benchmark::State& state) {
    Triangle2 triangle{Point2{0.0, 0.0}, Point2{1.0, 0.0}, Point2{0.0, 1.0}};
    Point2 point{0.5, 0.0};
    Measure(state, [](Triangle2& curve, Point2& query) { return curve.ParameterOf(query); }, triangle, point);
}
BENCHMARK(PrimTriangle2ParameterOf);

void PrimTriangle3Contains(benchmark::State& state) {
    Triangle3 triangle{Point3{0.0, 0.0, 0.0}, Point3{1.0, 0.0, 0.0}, Point3{0.0, 1.0, 0.0}};
    Point3 point{0.2, 0.2, 0.0};
    Measure(state, [](Triangle3& curve, Point3& query) { return curve.Contains(query); }, triangle, point);
}
BENCHMARK(PrimTriangle3Contains);

void PrimTriangle3ContainsPoint(benchmark::State& state) {
    Triangle3 triangle{Point3{0.0, 0.0, 0.0}, Point3{1.0, 0.0, 0.0}, Point3{0.0, 1.0, 0.0}};
    Point3 point{0.5, 0.0, 0.0};
    Measure(state, [](Triangle3& curve, Point3& query) { return curve.ContainsPoint(query); }, triangle, point);
}
BENCHMARK(PrimTriangle3ContainsPoint);

void PrimTriangle3ClosestPoint(benchmark::State& state) {
    Triangle3 triangle{Point3{0.0, 0.0, 0.0}, Point3{1.0, 0.0, 0.0}, Point3{0.0, 1.0, 0.0}};
    Point3 point{0.5, -1.0, 0.0};
    Measure(state, [](Triangle3& curve, Point3& query) { return curve.ClosestPoint(query); }, triangle, point);
}
BENCHMARK(PrimTriangle3ClosestPoint);

void PrimTriangle3Area(benchmark::State& state) {
    Triangle3 triangle{Point3{0.0, 0.0, 0.0}, Point3{1.0, 0.0, 0.0}, Point3{0.0, 1.0, 0.0}};
    Measure(state, [](Triangle3& curve) { return curve.Area(); }, triangle);
}
BENCHMARK(PrimTriangle3Area);

void PrimTriangle3ParameterOf(benchmark::State& state) {
    Triangle3 triangle{Point3{0.0, 0.0, 0.0}, Point3{1.0, 0.0, 0.0}, Point3{0.0, 1.0, 0.0}};
    Point3 point{0.5, 0.0, 0.0};
    Measure(state, [](Triangle3& curve, Point3& query) { return curve.ParameterOf(query); }, triangle, point);
}
BENCHMARK(PrimTriangle3ParameterOf);

void PrimPlaneSignedDistance(benchmark::State& state) {
    Plane plane{Point3{0.0, 0.0, 0.0}, AxisZ3()};
    Point3 point{1.0, 2.0, 3.0};
    Measure(state, [](Plane& surface, Point3& query) { return surface.SignedDistance(query); }, plane, point);
}
BENCHMARK(PrimPlaneSignedDistance);

void PrimPlaneProject(benchmark::State& state) {
    Plane plane{Point3{0.0, 0.0, 0.0}, AxisZ3()};
    Vector3 vector{1.0, 0.0, 1.0};
    Measure(state, [](Plane& surface, Vector3& query) { return surface.Project(query); }, plane, vector);
}
BENCHMARK(PrimPlaneProject);

void PrimPlaneMirror(benchmark::State& state) {
    Plane plane{Point3{0.0, 0.0, 0.0}, AxisZ3()};
    Point3 point{1.0, 2.0, 3.0};
    Measure(state, [](Plane& surface, Point3& query) { return surface.Mirror(query); }, plane, point);
}
BENCHMARK(PrimPlaneMirror);

void PrimPlaneContains(benchmark::State& state) {
    Plane plane{Point3{0.0, 0.0, 0.0}, AxisZ3()};
    Point3 point{1.0, 2.0, 0.0};
    Measure(state, [](Plane& surface, Point3& query) { return surface.Contains(query); }, plane, point);
}
BENCHMARK(PrimPlaneContains);

void PrimPlaneClosestPoint(benchmark::State& state) {
    Plane plane{Point3{0.0, 0.0, 0.0}, AxisZ3()};
    Point3 point{1.0, 2.0, 3.0};
    Measure(state, [](Plane& surface, Point3& query) { return surface.ClosestPoint(query); }, plane, point);
}
BENCHMARK(PrimPlaneClosestPoint);

void PrimPolylineLength(benchmark::State& state) {
    const Polyline polyline = Circle2(static_cast<int>(state.range(0)));
    Measure(state, [](const Polyline& curve) { return curve.Length(); }, polyline);
}
BENCHMARK(PrimPolylineLength)->Arg(16)->Arg(1024);

void PrimPolylineContains(benchmark::State& state) {
    const Polyline polyline = Circle2(static_cast<int>(state.range(0)));
    Point2 point{0.0, 0.0};
    Measure(state, [](const Polyline& curve, Point2& query) { return curve.Contains(query); }, polyline, point);
}
BENCHMARK(PrimPolylineContains)->Arg(16)->Arg(1024);

void PrimPolylineContainsPoint(benchmark::State& state) {
    const Polyline polyline = Circle2(static_cast<int>(state.range(0)));
    Point2 point = EdgeMidpoint2(polyline);
    Measure(state, [](const Polyline& curve, Point2& query) { return curve.ContainsPoint(query); }, polyline, point);
}
BENCHMARK(PrimPolylineContainsPoint)->Arg(16)->Arg(1024);

void PrimPolylineParameterOf(benchmark::State& state) {
    const Polyline polyline = Circle2(static_cast<int>(state.range(0)));
    Point2 point = EdgeMidpoint2(polyline);
    Measure(state, [](const Polyline& curve, Point2& query) { return curve.ParameterOf(query); }, polyline, point);
}
BENCHMARK(PrimPolylineParameterOf)->Arg(16)->Arg(1024);

void PrimPolylineBox(benchmark::State& state) {
    const Polyline polyline = Circle2(static_cast<int>(state.range(0)));
    Measure(state, [](const Polyline& curve) { return curve.Box(); }, polyline);
}
BENCHMARK(PrimPolylineBox)->Arg(16)->Arg(1024);

void PrimPolylineArea(benchmark::State& state) {
    const Polyline polyline = Circle2(static_cast<int>(state.range(0)));
    Measure(state, [](const Polyline& curve) { return curve.Area(); }, polyline);
}
BENCHMARK(PrimPolylineArea)->Arg(16)->Arg(1024);

void PrimPolylineTransform(benchmark::State& state) {
    const Polyline original = Circle2(16);
    const Transform2 rotation = Transform2::RotationAbout(Point2{0.0, 0.0}, 0.3);
    Polyline probe = original;
    if (!probe.Transform(rotation)) {
        state.SkipWithError("transform failed");
        return;
    }
    for (auto _ : state) {
        Polyline copy = original;
        benchmark::DoNotOptimize(copy);
        benchmark::DoNotOptimize(rotation);
        benchmark::DoNotOptimize(copy.Transform(rotation));
        benchmark::DoNotOptimize(copy);
    }
}
BENCHMARK(PrimPolylineTransform);

void PrimPolyline3Length(benchmark::State& state) {
    const Polyline3 polyline = Circle3(static_cast<int>(state.range(0)));
    Measure(state, [](const Polyline3& curve) { return curve.Length(); }, polyline);
}
BENCHMARK(PrimPolyline3Length)->Arg(16)->Arg(1024);

void PrimPolyline3Contains(benchmark::State& state) {
    const Polyline3 polyline = Circle3(static_cast<int>(state.range(0)));
    Point3 point{0.0, 0.0, 0.0};
    Measure(state, [](const Polyline3& curve, Point3& query) { return curve.Contains(query); }, polyline, point);
}
BENCHMARK(PrimPolyline3Contains)->Arg(16)->Arg(1024);

void PrimPolyline3ContainsPoint(benchmark::State& state) {
    const Polyline3 polyline = Circle3(static_cast<int>(state.range(0)));
    Point3 point = EdgeMidpoint3(polyline);
    Measure(state, [](const Polyline3& curve, Point3& query) { return curve.ContainsPoint(query); }, polyline, point);
}
BENCHMARK(PrimPolyline3ContainsPoint)->Arg(16)->Arg(1024);

void PrimPolyline3ParameterOf(benchmark::State& state) {
    const Polyline3 polyline = Circle3(static_cast<int>(state.range(0)));
    Point3 point = EdgeMidpoint3(polyline);
    Measure(state, [](const Polyline3& curve, Point3& query) { return curve.ParameterOf(query); }, polyline, point);
}
BENCHMARK(PrimPolyline3ParameterOf)->Arg(16)->Arg(1024);

void PrimPolyline3Box(benchmark::State& state) {
    const Polyline3 polyline = Circle3(static_cast<int>(state.range(0)));
    Measure(state, [](const Polyline3& curve) { return curve.Box(); }, polyline);
}
BENCHMARK(PrimPolyline3Box)->Arg(16)->Arg(1024);

void PrimPolyline3Area(benchmark::State& state) {
    const Polyline3 polyline = Circle3(static_cast<int>(state.range(0)));
    Measure(state, [](const Polyline3& curve) { return curve.Area(); }, polyline);
}
BENCHMARK(PrimPolyline3Area)->Arg(16)->Arg(1024);

void PrimPolyline3Transform(benchmark::State& state) {
    const Polyline3 original = Circle3(16);
    const auto axis = UnitVector3::FromNormalizedUnchecked(Vector3{0.0, 1.0, 0.0});
    const Transform3 rotation = Transform3::RotationAbout(Point3{0.0, 0.0, 0.0}, axis, 0.3);
    Polyline3 probe = original;
    if (!probe.Transform(rotation)) {
        state.SkipWithError("transform failed");
        return;
    }
    for (auto _ : state) {
        Polyline3 copy = original;
        benchmark::DoNotOptimize(copy);
        benchmark::DoNotOptimize(rotation);
        benchmark::DoNotOptimize(copy.Transform(rotation));
        benchmark::DoNotOptimize(copy);
    }
}
BENCHMARK(PrimPolyline3Transform);

} // namespace
