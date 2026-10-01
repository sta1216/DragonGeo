#include <DragonGeo/Query/Intersection.hpp>

#include <DragonGeo/Detail/CurveContainment.hpp>

namespace DragonGeo::Query::Detail {

enum class CurveDomain { Line, Ray, Segment };

enum class HitKind { Miss, Point, Region };

struct SolidHit {
    HitKind Kind = HitKind::Miss;
    ParameterPoint3 Point{};
};

[[nodiscard]] inline CurveMeet2 MeetPoint(Linear::Point2 point, double onFirst, double onSecond) noexcept {
    CurveMeet2 meet;
    meet.Kind = CurveMeet::Point;
    meet.Point = point;
    meet.ParameterOnFirst = onFirst;
    meet.ParameterOnSecond = onSecond;
    return meet;
}

[[nodiscard]] inline CurveMeet2 MeetNone() noexcept {
    return {};
}

[[nodiscard]] inline CurveMeet2 MeetOverlap(Prim::Segment2 overlap) noexcept {
    CurveMeet2 meet;
    meet.Kind = CurveMeet::Overlap;
    meet.Overlap = overlap;
    return meet;
}

[[nodiscard]] inline double ParameterAlong(Linear::Point2 origin, Linear::Point2 end, Linear::Point2 point) noexcept {
    const Linear::Vector2 chord = end - origin;
    const double lengthSquared = chord.LengthSquared();
    if (!(lengthSquared > 0.0)) {
        return 0.0;
    }
    return (point - origin).Dot(chord) / lengthSquared;
}

[[nodiscard]] inline bool ParameterInUnit(double parameter) noexcept {
    return parameter >= 0.0 && parameter <= 1.0;
}

[[nodiscard]] inline CurveMeet2 CollinearSegments(Prim::Segment2 first, Prim::Segment2 second) noexcept {
    const double t0 = ParameterAlong(first.A, first.B, second.A);
    const double t1 = ParameterAlong(first.A, first.B, second.B);
    const double lo = std::min(t0, t1);
    const double hi = std::max(t0, t1);
    const double enter = std::max(lo, 0.0);
    const double exit = std::min(hi, 1.0);
    if (enter > exit) {
        return MeetNone();
    }
    const Linear::Vector2 chord = first.B - first.A;
    const Linear::Point2 start = first.A + chord * enter;
    if (!(enter < exit)) {
        return MeetPoint(start, enter, ParameterAlong(second.A, second.B, start));
    }
    return MeetOverlap(Prim::Segment2{start, first.A + chord * exit});
}

[[nodiscard]] inline CurveMeet2 PointOnSegment(Linear::Point2 point, Prim::Segment2 segment, double parameterOnPoint) noexcept {
    const double parameter = ParameterAlong(segment.A, segment.B, point);
    if (Predicates::Orient2d(segment.A, segment.B, point) != 0 || !ParameterInUnit(parameter)) {
        if (segment.A == segment.B) {
            return point == segment.A ? MeetPoint(point, parameterOnPoint, 0.0) : MeetNone();
        }
        return MeetNone();
    }
    const double onSegment = segment.A == segment.B ? 0.0 : parameter;
    return MeetPoint(point, parameterOnPoint, onSegment);
}

[[nodiscard]] inline CurveMeet2 IntersectionSegments(Prim::Segment2 first, Prim::Segment2 second) noexcept {
    if (first.A == first.B && second.A == second.B) {
        return first.A == second.A ? MeetPoint(first.A, 0.0, 0.0) : MeetNone();
    }
    if (first.A == first.B) {
        return PointOnSegment(first.A, second, 0.0);
    }
    if (second.A == second.B) {
        const double parameter = ParameterAlong(first.A, first.B, second.A);
        if (!ParameterInUnit(parameter) || Predicates::Orient2d(first.A, first.B, second.A) != 0) {
            return MeetNone();
        }
        return MeetPoint(second.A, parameter, 0.0);
    }

    const int abc = Predicates::Orient2d(first.A, first.B, second.A);
    const int abd = Predicates::Orient2d(first.A, first.B, second.B);
    if (abc == 0 && abd == 0) {
        return CollinearSegments(first, second);
    }

    const int cda = Predicates::Orient2d(second.A, second.B, first.A);
    const int cdb = Predicates::Orient2d(second.A, second.B, first.B);
    if ((abc > 0 && abd > 0) || (abc < 0 && abd < 0) || (cda > 0 && cdb > 0) || (cda < 0 && cdb < 0)) {
        return MeetNone();
    }

    if (abc == 0) {
        const double parameter = ParameterAlong(first.A, first.B, second.A);
        if (ParameterInUnit(parameter)) {
            return MeetPoint(second.A, parameter, 0.0);
        }
    }
    if (abd == 0) {
        const double parameter = ParameterAlong(first.A, first.B, second.B);
        if (ParameterInUnit(parameter)) {
            return MeetPoint(second.B, parameter, 1.0);
        }
    }
    if (cda == 0) {
        const double parameter = ParameterAlong(second.A, second.B, first.A);
        if (ParameterInUnit(parameter)) {
            return MeetPoint(first.A, 0.0, parameter);
        }
    }
    if (cdb == 0) {
        const double parameter = ParameterAlong(second.A, second.B, first.B);
        if (ParameterInUnit(parameter)) {
            return MeetPoint(first.B, 1.0, parameter);
        }
    }

    const Linear::Vector2 ab = first.B - first.A;
    const Linear::Vector2 cd = second.B - second.A;
    const Linear::Vector2 ac = second.A - first.A;
    const double denominator = ab.Cross(cd);
    if (denominator == 0.0) {
        return MeetNone();
    }
    const double onFirst = ac.Cross(cd) / denominator;
    const double onSecond = -ab.Cross(ac) / denominator;
    if (!ParameterInUnit(onFirst) || !ParameterInUnit(onSecond)) {
        return MeetNone();
    }
    return MeetPoint(first.A + ab * onFirst, onFirst, onSecond);
}

[[nodiscard]] inline CurveMeet2 IntersectionLineSegment(Prim::Line2 line, Prim::Segment2 segment) noexcept {
    const Linear::Vector2 direction = line.Direction.AsVector();
    const Linear::Point2 tip = line.Origin + direction;
    if (segment.A == segment.B) {
        if (Predicates::Orient2d(line.Origin, tip, segment.A) != 0) {
            return MeetNone();
        }
        return MeetPoint(segment.A, (segment.A - line.Origin).Dot(direction), 0.0);
    }

    const int sideA = Predicates::Orient2d(line.Origin, tip, segment.A);
    const int sideB = Predicates::Orient2d(line.Origin, tip, segment.B);
    if (sideA == 0 && sideB == 0) {
        const Linear::Vector2 chord = segment.B - segment.A;
        if (chord.Dot(direction) < 0.0) {
            return MeetOverlap(Prim::Segment2{segment.B, segment.A});
        }
        return MeetOverlap(segment);
    }
    if ((sideA > 0 && sideB > 0) || (sideA < 0 && sideB < 0)) {
        return MeetNone();
    }

    const Linear::Vector2 chord = segment.B - segment.A;
    const Linear::Vector2 offset = segment.A - line.Origin;
    const double denominator = direction.Cross(chord);
    if (denominator == 0.0) {
        return MeetNone();
    }
    const double onLine = offset.Cross(chord) / denominator;
    const double onSegment = -direction.Cross(offset) / denominator;
    if (!ParameterInUnit(onSegment) || !Core::IsFinite(onLine)) {
        return MeetNone();
    }
    return MeetPoint(line.Origin + direction * onLine, onLine, onSegment);
}

template <typename Point, typename Vector> [[nodiscard]] std::optional<std::pair<double, double>> Slab(Point origin, Vector direction, Point low,
                                                            Point high, double tEnter, double tExit, int axes) noexcept {
    for (int axis = 0; axis < axes; ++axis) {
        const double component = direction[axis];
        if (component == 0.0) {
            if (origin[axis] < low[axis] || origin[axis] > high[axis]) {
                return std::nullopt;
            }
            continue;
        }
        double t1 = (low[axis] - origin[axis]) / component;
        double t2 = (high[axis] - origin[axis]) / component;
        if (t1 > t2) {
            std::swap(t1, t2);
        }
        if (t1 > tEnter) {
            tEnter = t1;
        }
        if (t2 < tExit) {
            tExit = t2;
        }
        if (!(tEnter <= tExit)) {
            return std::nullopt;
        }
    }
    return std::pair<double, double>{tEnter, tExit};
}

template <typename Interval, typename Point, typename Vector>
[[nodiscard]] Interval AtParameters(Point origin, Vector direction, double enter, double exit) noexcept {
    Interval interval;
    interval.Enter = enter;
    interval.Exit = exit;
    interval.EnterPoint = origin + direction * enter;
    interval.ExitPoint = origin + direction * exit;
    return interval;
}

template <typename Interval, typename Point, typename Vector, typename Box>
[[nodiscard]] std::optional<Interval> IntersectBox(Point origin, Vector direction, Box box, double tEnter, double tExit, int axes) noexcept {
    if (box.IsEmpty()) {
        return std::nullopt;
    }
    const auto range = Slab(origin, direction, box.Min, box.Max, tEnter, tExit, axes);
    if (!range.has_value()) {
        return std::nullopt;
    }
    return AtParameters<Interval>(origin, direction, range->first, range->second);
}

template <typename Interval, typename Point, typename Vector, typename Corner>
[[nodiscard]] std::optional<Interval> IntersectExtents(Point origin, Vector direction, Corner low, Corner high,
                                                       double tEnter, double tExit, int axes, Point worldOrigin, Vector worldDirection) noexcept {
    const auto range = Slab(origin, direction, low, high, tEnter, tExit, axes);
    if (!range.has_value()) {
        return std::nullopt;
    }
    return AtParameters<Interval>(worldOrigin, worldDirection, range->first, range->second);
}

[[nodiscard]] inline SolidHit HitPlane(Linear::Point3 origin, Linear::Vector3 direction, CurveDomain domain, Prim::Plane plane) noexcept {
    const double alongNormal = direction.Dot(plane.Normal.AsVector());
    const double signedOrigin = plane.SignedDistance(origin);
    if (alongNormal == 0.0) {
        if (signedOrigin != 0.0) {
            return {};
        }
        if (domain == CurveDomain::Segment && direction.LengthSquared() == 0.0) {
            return SolidHit{HitKind::Point, ParameterPoint3{0.0, origin}};
        }
        return SolidHit{HitKind::Region, {}};
    }

    const double parameter = -signedOrigin / alongNormal;
    if (!Core::IsFinite(parameter)) {
        return {};
    }
    if (domain == CurveDomain::Ray && parameter < 0.0) {
        return {};
    }
    if (domain == CurveDomain::Segment && !ParameterInUnit(parameter)) {
        return {};
    }
    return SolidHit{HitKind::Point, ParameterPoint3{parameter, origin + direction * parameter}};
}

[[nodiscard]] inline bool PointOnSegment3(Linear::Point3 point, Linear::Point3 start, Linear::Point3 end) noexcept {
    const Linear::Vector3 chord = end - start;
    const Linear::Vector3 offset = point - start;
    if (chord.Cross(offset).LengthSquared() != 0.0) {
        return false;
    }
    const double lengthSquared = chord.LengthSquared();
    if (!(lengthSquared > 0.0)) {
        return point == start;
    }
    const double parameter = offset.Dot(chord) / lengthSquared;
    return ParameterInUnit(parameter);
}

/// 直线 `origin + t direction` 是否碰到线段 `start`–`end`。`direction` 在射线上是单位向量，在线段上是 `B - A`。
[[nodiscard]] inline bool CurveMeetsSegment3(Linear::Point3 origin, Linear::Vector3 direction,
                                             CurveDomain domain, Linear::Point3 start, Linear::Point3 end) noexcept {
    if (direction.LengthSquared() == 0.0) {
        return PointOnSegment3(origin, start, end);
    }

    const Linear::Vector3 edge = end - start;
    const Linear::Vector3 crossed = direction.Cross(edge);
    const double denominator = crossed.LengthSquared();
    const Linear::Vector3 offset = start - origin;
    if (denominator == 0.0) {
        if (direction.Cross(offset).LengthSquared() != 0.0) {
            return false;
        }
        if (domain == CurveDomain::Line) {
            return true;
        }
        const double lengthSquared = direction.LengthSquared();
        const double t0 = (start - origin).Dot(direction) / lengthSquared;
        const double t1 = (end - origin).Dot(direction) / lengthSquared;
        const double hi = std::max(t0, t1);
        const double lo = std::min(t0, t1);
        if (domain == CurveDomain::Ray) {
            return hi >= 0.0;
        }
        return lo <= 1.0 && hi >= 0.0;
    }

    if (offset.Dot(crossed) != 0.0) {
        return false;
    }
    const double onEdge = offset.Cross(direction).Dot(crossed) / denominator;
    if (!ParameterInUnit(onEdge)) {
        return false;
    }
    const double onCurve = offset.Cross(edge).Dot(crossed) / denominator;
    if (domain == CurveDomain::Ray && onCurve < 0.0) {
        return false;
    }
    if (domain == CurveDomain::Segment && !ParameterInUnit(onCurve)) {
        return false;
    }
    return true;
}

[[nodiscard]] inline bool CoplanarCurveHitsTriangle(Linear::Point3 origin, Linear::Vector3 direction,
                                                    CurveDomain domain, Prim::Triangle3 triangle) noexcept {
    if (domain != CurveDomain::Line && triangle.Contains(origin)) {
        return true;
    }
    if (domain == CurveDomain::Segment && triangle.Contains(origin + direction)) {
        return true;
    }
    const Linear::Point3 edges[3][2] = {{triangle.A, triangle.B}, {triangle.B, triangle.C}, {triangle.C, triangle.A}};
    for (const auto& edge : edges) {
        if (CurveMeetsSegment3(origin, direction, domain, edge[0], edge[1])) {
            return true;
        }
    }
    return false;
}

/// 丢掉法向绝对值最大的轴。并列时按 X、Y、Z 取先出现的轴，与 `Triangle3` 投影一致。
[[nodiscard]] inline int DroppedNormalAxis(Linear::Vector3 normal) noexcept {
    return DragonGeo::Detail::DominantAxis(normal);
}

[[nodiscard]] inline Linear::Point2 DropAxis(Linear::Point3 point, int axis) noexcept {
    return DragonGeo::Detail::DropAxis(point, axis);
}

/// 平面求交得到的点可能离开三角形一个 ulp。`Triangle3::Contains` 要求 `Orient3d == 0`，会把横向穿过的真实交点判掉。投影到坐标平面后，用与 `Triangle2::Contains` 相同的包含测试。
[[nodiscard]] inline bool ContainsInProjection(Prim::Triangle3 triangle, Linear::Point3 point, Linear::Vector3 normal) noexcept {
    const int dropped = DroppedNormalAxis(normal);
    const Prim::Triangle2 projected{DropAxis(triangle.A, dropped), DropAxis(triangle.B, dropped), DropAxis(triangle.C, dropped)};
    return projected.Contains(DropAxis(point, dropped));
}

[[nodiscard]] inline SolidHit HitTriangle(Linear::Point3 origin, Linear::Vector3 direction, CurveDomain domain, Prim::Triangle3 triangle) noexcept {
    if (direction.LengthSquared() == 0.0) {
        if (!triangle.Contains(origin)) {
            return {};
        }
        return SolidHit{HitKind::Point, ParameterPoint3{0.0, origin}};
    }

    const int sideOrigin = Predicates::Orient3d(triangle.A, triangle.B, triangle.C, origin);
    const int sideAhead = Predicates::Orient3d(triangle.A, triangle.B, triangle.C, origin + direction);
    if (sideOrigin == 0 && sideAhead == 0) {
        if (!CoplanarCurveHitsTriangle(origin, direction, domain, triangle)) {
            return {};
        }
        return SolidHit{HitKind::Region, {}};
    }

    const Linear::Vector3 normal = (triangle.B - triangle.A).Cross(triangle.C - triangle.A);
    const double denominator = normal.Dot(direction);
    if (denominator == 0.0) {
        return {};
    }
    const double parameter = -normal.Dot(origin - triangle.A) / denominator;
    if (!Core::IsFinite(parameter)) {
        return {};
    }
    if (domain == CurveDomain::Ray && parameter < 0.0) {
        return {};
    }
    if (domain == CurveDomain::Segment && !ParameterInUnit(parameter)) {
        return {};
    }
    const Linear::Point3 point = origin + direction * parameter;
    if (!ContainsInProjection(triangle, point, normal)) {
        return {};
    }
    return SolidHit{HitKind::Point, ParameterPoint3{parameter, point}};
}

[[nodiscard]] inline std::optional<ParameterPoint3> AsPoint(SolidHit hit) noexcept {
    if (hit.Kind != HitKind::Point) {
        return std::nullopt;
    }
    return hit.Point;
}

template <typename Point> [[nodiscard]] double SegmentDistanceSquared(Point firstStart, Point firstEnd, Point secondStart, Point secondEnd) noexcept {
    const auto u = firstEnd - firstStart;
    const auto v = secondEnd - secondStart;
    const auto w = firstStart - secondStart;
    const double a = u.Dot(u);
    const double b = u.Dot(v);
    const double c = v.Dot(v);
    const double d = u.Dot(w);
    const double e = v.Dot(w);
    // 有一条线段长度为 0 时，把那个点投到另一条线段上并夹到 [0, 1]。第二条退化为点时，通用公式会把参数留在起点，距离因此不对称。
    if (!(c > 0.0)) {
        const double s = !(a > 0.0) ? 0.0 : std::min(1.0, std::max(0.0, -d / a));
        const auto difference = w + u * s;
        return difference.LengthSquared();
    }
    if (!(a > 0.0)) {
        const double t = std::min(1.0, std::max(0.0, e / c));
        const auto difference = w - v * t;
        return difference.LengthSquared();
    }
    const double determinant = a * c - b * b;
    double sNumerator = 0.0;
    double sDenominator = determinant;
    double tNumerator = 0.0;
    double tDenominator = determinant;

    if (!(determinant > 0.0)) {
        sNumerator = 0.0;
        sDenominator = 1.0;
        tNumerator = e;
        tDenominator = c;
    } else {
        sNumerator = b * e - c * d;
        tNumerator = a * e - b * d;
        if (sNumerator < 0.0) {
            sNumerator = 0.0;
            tNumerator = e;
            tDenominator = c;
        } else if (sNumerator > sDenominator) {
            sNumerator = sDenominator;
            tNumerator = e + b;
            tDenominator = c;
        }
    }

    if (tNumerator < 0.0) {
        tNumerator = 0.0;
        if (-d < 0.0) {
            sNumerator = 0.0;
        } else if (-d > a) {
            sNumerator = sDenominator;
        } else {
            sNumerator = -d;
            sDenominator = a;
        }
    } else if (tNumerator > tDenominator) {
        tNumerator = tDenominator;
        if (-d + b < 0.0) {
            sNumerator = 0.0;
        } else if (-d + b > a) {
            sNumerator = sDenominator;
        } else {
            sNumerator = -d + b;
            sDenominator = a;
        }
    }

    const double s = sDenominator == 0.0 ? 0.0 : sNumerator / sDenominator;
    const double t = tDenominator == 0.0 ? 0.0 : tNumerator / tDenominator;
    const auto difference = w + u * s - v * t;
    return difference.LengthSquared();
}

[[nodiscard]] inline Linear::Point3 ClosestOnTriangle(Linear::Point3 point, Prim::Triangle3 triangle) noexcept {
    const Linear::Vector3 ab = triangle.B - triangle.A;
    const Linear::Vector3 ac = triangle.C - triangle.A;
    const Linear::Vector3 ap = point - triangle.A;
    const double d1 = ab.Dot(ap);
    const double d2 = ac.Dot(ap);
    if (d1 <= 0.0 && d2 <= 0.0) {
        return triangle.A;
    }

    const Linear::Vector3 bp = point - triangle.B;
    const double d3 = ab.Dot(bp);
    const double d4 = ac.Dot(bp);
    if (d3 >= 0.0 && d4 <= d3) {
        return triangle.B;
    }

    const double vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0) {
        const double v = d1 / (d1 - d3);
        return triangle.A + ab * v;
    }

    const Linear::Vector3 cp = point - triangle.C;
    const double d5 = ab.Dot(cp);
    const double d6 = ac.Dot(cp);
    if (d6 >= 0.0 && d5 <= d6) {
        return triangle.C;
    }

    const double vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0) {
        const double w = d2 / (d2 - d6);
        return triangle.A + ac * w;
    }

    const double va = d3 * d6 - d5 * d4;
    if (va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0) {
        const double w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return triangle.B + (triangle.C - triangle.B) * w;
    }

    const double denominator = 1.0 / (va + vb + vc);
    const double v = vb * denominator;
    const double w = vc * denominator;
    return triangle.A + ab * v + ac * w;
}

[[nodiscard]] inline double PointTriangleDistanceSquared(Linear::Point3 point, Prim::Triangle3 triangle) noexcept {
    return (point - ClosestOnTriangle(point, triangle)).LengthSquared();
}

struct AxisRange {
    double Lo = 0.0;
    double Hi = 0.0;
    bool Ok = false;
};

[[nodiscard]] inline double PlaneVolume(Linear::Point3 a, Linear::Point3 b, Linear::Point3 c, Linear::Point3 point) noexcept {
    return (b - a).Cross(c - a).Dot(point - a);
}

[[nodiscard]] inline AxisRange TrianglePlaneRange(Linear::Point3 v0, Linear::Point3 v1, Linear::Point3 v2,
                                                  double d0, double d1, double d2, int axis) noexcept {
    const Linear::Point3 vertices[3] = {v0, v1, v2};
    const double distances[3] = {d0, d1, d2};
    AxisRange range;
    const auto add = [&](double projected) noexcept {
        if (!range.Ok) {
            range.Lo = projected;
            range.Hi = projected;
            range.Ok = true;
            return;
        }
        range.Lo = std::min(range.Lo, projected);
        range.Hi = std::max(range.Hi, projected);
    };
    for (int index = 0; index < 3; ++index) {
        const int next = (index + 1) % 3;
        const double here = distances[index];
        const double there = distances[next];
        if (here == 0.0 && there == 0.0) {
            add(vertices[index][axis]);
            add(vertices[next][axis]);
        } else if (here == 0.0) {
            add(vertices[index][axis]);
        } else if ((here < 0.0 && there > 0.0) || (here > 0.0 && there < 0.0)) {
            const double parameter = here / (here - there);
            const Linear::Point3 point = vertices[index] + (vertices[next] - vertices[index]) * parameter;
            add(point[axis]);
        }
    }
    return range;
}

[[nodiscard]] inline bool RangesOverlap(AxisRange first, AxisRange second) noexcept {
    return first.Ok && second.Ok && first.Lo <= second.Hi && second.Lo <= first.Hi;
}

[[nodiscard]] inline bool CoplanarTriangles(Prim::Triangle3 first, Prim::Triangle3 second) noexcept {
    if (first.Contains(second.A) || first.Contains(second.B) || first.Contains(second.C)
        || second.Contains(first.A) || second.Contains(first.B) || second.Contains(first.C)) {
        return true;
    }
    const Prim::Segment3 firstEdges[3] = {{first.A, first.B}, {first.B, first.C}, {first.C, first.A}};
    const Prim::Segment3 secondEdges[3] = {{second.A, second.B}, {second.B, second.C}, {second.C, second.A}};
    for (const Prim::Segment3& edge : firstEdges) {
        for (const Prim::Segment3& other : secondEdges) {
            if (CurveMeetsSegment3(edge.A, edge.B - edge.A, CurveDomain::Segment, other.A, other.B)) {
                return true;
            }
        }
    }
    return false;
}

[[nodiscard]] inline bool TrianglesIntersect(Prim::Triangle3 first, Prim::Triangle3 second) noexcept {
    const int firstSides[3] = {
        Predicates::Orient3d(second.A, second.B, second.C, first.A),
        Predicates::Orient3d(second.A, second.B, second.C, first.B), Predicates::Orient3d(second.A, second.B, second.C, first.C),
    };
    if ((firstSides[0] > 0 && firstSides[1] > 0 && firstSides[2] > 0) || (firstSides[0] < 0 && firstSides[1] < 0 && firstSides[2] < 0)) {
        return false;
    }
    const int secondSides[3] = {
        Predicates::Orient3d(first.A, first.B, first.C, second.A),
        Predicates::Orient3d(first.A, first.B, first.C, second.B), Predicates::Orient3d(first.A, first.B, first.C, second.C),
    };
    if ((secondSides[0] > 0 && secondSides[1] > 0 && secondSides[2] > 0) || (secondSides[0] < 0 && secondSides[1] < 0 && secondSides[2] < 0)) {
        return false;
    }
    if ((firstSides[0] == 0 && firstSides[1] == 0 && firstSides[2] == 0) || (secondSides[0] == 0 && secondSides[1] == 0 && secondSides[2] == 0)) {
        return CoplanarTriangles(first, second);
    }

    const double firstDistances[3] = {
        firstSides[0] == 0 ? 0.0 : PlaneVolume(second.A, second.B, second.C, first.A),
        firstSides[1] == 0 ? 0.0 : PlaneVolume(second.A, second.B, second.C, first.B),
        firstSides[2] == 0 ? 0.0 : PlaneVolume(second.A, second.B, second.C, first.C),
    };
    const double secondDistances[3] = {
        secondSides[0] == 0 ? 0.0 : PlaneVolume(first.A, first.B, first.C, second.A),
        secondSides[1] == 0 ? 0.0 : PlaneVolume(first.A, first.B, first.C, second.B),
        secondSides[2] == 0 ? 0.0 : PlaneVolume(first.A, first.B, first.C, second.C),
    };
    const Linear::Vector3 normalFirst = (first.B - first.A).Cross(first.C - first.A);
    const Linear::Vector3 normalSecond = (second.B - second.A).Cross(second.C - second.A);
    const Linear::Vector3 direction = normalFirst.Cross(normalSecond);
    int axis = 0;
    double largest = Core::AbsoluteValue(direction.X);
    if (Core::AbsoluteValue(direction.Y) > largest) {
        largest = Core::AbsoluteValue(direction.Y);
        axis = 1;
    }
    if (Core::AbsoluteValue(direction.Z) > largest) {
        axis = 2;
    }
    if (largest == 0.0) {
        return CoplanarTriangles(first, second);
    }
    const AxisRange firstRange = TrianglePlaneRange(first.A, first.B, first.C, firstDistances[0], firstDistances[1], firstDistances[2], axis);
    const AxisRange secondRange = TrianglePlaneRange(second.A, second.B, second.C, secondDistances[0], secondDistances[1], secondDistances[2], axis);
    return RangesOverlap(firstRange, secondRange);
}

[[nodiscard]] inline double TriangleDistanceSquared(Prim::Triangle3 first, Prim::Triangle3 second) noexcept {
    if (TrianglesIntersect(first, second)) {
        return 0.0;
    }
    double best = PointTriangleDistanceSquared(first.A, second);
    best = std::min(best, PointTriangleDistanceSquared(first.B, second));
    best = std::min(best, PointTriangleDistanceSquared(first.C, second));
    best = std::min(best, PointTriangleDistanceSquared(second.A, first));
    best = std::min(best, PointTriangleDistanceSquared(second.B, first));
    best = std::min(best, PointTriangleDistanceSquared(second.C, first));

    const Prim::Segment3 firstEdges[3] = {{first.A, first.B}, {first.B, first.C}, {first.C, first.A}};
    const Prim::Segment3 secondEdges[3] = {{second.A, second.B}, {second.B, second.C}, {second.C, second.A}};
    for (const Prim::Segment3& edge : firstEdges) {
        for (const Prim::Segment3& other : secondEdges) {
            best = std::min(best, SegmentDistanceSquared(edge.A, edge.B, other.A, other.B));
        }
    }
    return best;
}

[[nodiscard]] inline double Infinity() noexcept {
    return std::numeric_limits<double>::infinity();
}

} // namespace DragonGeo::Query::Detail

namespace DragonGeo::Query {



/// 两条直线相交。平行且分离为 `None`，重合为 `Coincident`，其余为一个点。点的参数是沿各自方向的有符号距离。`noexcept`。
[[nodiscard]] CurveMeet2 Intersection(Prim::Line2 first, Prim::Line2 second) noexcept {
    const Linear::Vector2 firstDirection = first.Direction.AsVector();
    const Linear::Vector2 secondDirection = second.Direction.AsVector();
    const Linear::Point2 firstTip = first.Origin + firstDirection;
    if (Predicates::Orient2d(first.Origin, firstTip, first.Origin + secondDirection) == 0) {
        if (Predicates::Orient2d(first.Origin, firstTip, second.Origin) == 0) {
            CurveMeet2 meet;
            meet.Kind = CurveMeet::Coincident;
            return meet;
        }
        return {};
    }
    const Linear::Vector2 offset = second.Origin - first.Origin;
    const double denominator = firstDirection.Cross(secondDirection);
    const double onFirst = offset.Cross(secondDirection) / denominator;
    const double onSecond = offset.Cross(firstDirection) / denominator;
    return Detail::MeetPoint(first.Origin + firstDirection * onFirst, onFirst, onSecond);
}



/// 两条直线是否相交，包括重合。`noexcept`。
[[nodiscard]] bool Intersects(Prim::Line2 first, Prim::Line2 second) noexcept {
    return Intersection(first, second).Kind != CurveMeet::None;
}



/// 两条线段相交。只共享端点或内部交点为 `Point`。共线且重叠长度为零也是 `Point`。正长度重叠为 `Overlap`，重叠段沿第一条线段的方向。`noexcept`。
[[nodiscard]] CurveMeet2 Intersection(Prim::Segment2 first, Prim::Segment2 second) noexcept {
    return Detail::IntersectionSegments(first, second);
}



/// 两条线段是否相交，包括端点相接和共线重叠。`noexcept`。
[[nodiscard]] bool Intersects(Prim::Segment2 first, Prim::Segment2 second) noexcept {
    return Intersection(first, second).Kind != CurveMeet::None;
}



/// 直线与线段相交。线段整段落在直线上时为 `Overlap`，重叠段沿直线方向。零长度线段落在直线上时为 `Point`。`noexcept`。
[[nodiscard]] CurveMeet2 Intersection(Prim::Line2 line, Prim::Segment2 segment) noexcept {
    return Detail::IntersectionLineSegment(line, segment);
}



/// 直线与线段是否相交。`noexcept`。
[[nodiscard]] bool Intersects(Prim::Line2 line, Prim::Segment2 segment) noexcept {
    return Intersection(line, segment).Kind != CurveMeet::None;
}



/// 射线与平面。平行且不在平面上不相交。射线躺在平面上时相交，但没有单个交点。参数是沿射线方向的距离，且 `>= 0`。`noexcept`。
[[nodiscard]] std::optional<ParameterPoint3> Intersection(Prim::Ray3 ray, Prim::Plane plane) noexcept {
    return Detail::AsPoint(Detail::HitPlane(ray.Origin, ray.Direction.AsVector(), Detail::CurveDomain::Ray, plane));
}



/// 射线是否打到平面，包括整条射线躺在平面上。`noexcept`。
[[nodiscard]] bool Intersects(Prim::Ray3 ray, Prim::Plane plane) noexcept {
    return Detail::HitPlane(ray.Origin, ray.Direction.AsVector(), Detail::CurveDomain::Ray, plane).Kind != Detail::HitKind::Miss;
}



/// 线段与平面。正长度线段躺在平面上时相交，`Intersection` 为空。零长度线段落在平面上时返回该点。参数落在 `[0, 1]`。`noexcept`。
[[nodiscard]] std::optional<ParameterPoint3> Intersection(Prim::Segment3 segment, Prim::Plane plane) noexcept {
    return Detail::AsPoint(Detail::HitPlane(segment.A, segment.B - segment.A, Detail::CurveDomain::Segment, plane));
}



/// 线段是否打到平面，包括整段躺在平面上。`noexcept`。
[[nodiscard]] bool Intersects(Prim::Segment3 segment, Prim::Plane plane) noexcept {
    return Detail::HitPlane(segment.A, segment.B - segment.A, Detail::CurveDomain::Segment, plane).Kind != Detail::HitKind::Miss;
}



/// 直线与平面。躺在平面上为 `Coincident`，平行且分离为 `None`，其余为一个点。参数是沿直线方向的有符号距离。`noexcept`。
[[nodiscard]] CurveMeet3 Intersection(Prim::Line3 line, Prim::Plane plane) noexcept {
    const Detail::SolidHit hit = Detail::HitPlane(line.Origin, line.Direction.AsVector(), Detail::CurveDomain::Line, plane);
    CurveMeet3 meet;
    if (hit.Kind == Detail::HitKind::Region) {
        meet.Kind = CurveMeet::Coincident;
        return meet;
    }
    if (hit.Kind == Detail::HitKind::Point) {
        meet.Kind = CurveMeet::Point;
        meet.Point = hit.Point.Point;
        meet.ParameterOnFirst = hit.Point.Parameter;
    }
    return meet;
}



/// 直线是否打到平面，包括躺在平面上。`noexcept`。
[[nodiscard]] bool Intersects(Prim::Line3 line, Prim::Plane plane) noexcept {
    return Intersection(line, plane).Kind != CurveMeet::None;
}



/// 射线与三角形。横向穿过或点接触返回交点。共面重叠时相交，但没有单个交点。`noexcept`。
[[nodiscard]] std::optional<ParameterPoint3> Intersection(Prim::Ray3 ray, Prim::Triangle3 triangle) noexcept {
    return Detail::AsPoint(Detail::HitTriangle(ray.Origin, ray.Direction.AsVector(), Detail::CurveDomain::Ray, triangle));
}



/// 射线是否打到三角形，包括共面重叠。`noexcept`。
[[nodiscard]] bool Intersects(Prim::Ray3 ray, Prim::Triangle3 triangle) noexcept {
    return Detail::HitTriangle(ray.Origin, ray.Direction.AsVector(), Detail::CurveDomain::Ray, triangle).Kind != Detail::HitKind::Miss;
}



/// 线段与三角形。横向穿过返回交点，参数在 `[0, 1]`。共面重叠时 `Intersection` 为空。`noexcept`。
[[nodiscard]] std::optional<ParameterPoint3> Intersection(Prim::Segment3 segment, Prim::Triangle3 triangle) noexcept {
    return Detail::AsPoint(Detail::HitTriangle(segment.A, segment.B - segment.A, Detail::CurveDomain::Segment, triangle));
}



/// 线段是否打到三角形，包括共面重叠。`noexcept`。
[[nodiscard]] bool Intersects(Prim::Segment3 segment, Prim::Triangle3 triangle) noexcept {
    return Detail::HitTriangle(segment.A, segment.B - segment.A, Detail::CurveDomain::Segment, triangle).Kind != Detail::HitKind::Miss;
}



/// 直线与三角形。横向穿过返回交点。共面重叠时相交，`Intersection` 为空。`noexcept`。
[[nodiscard]] std::optional<ParameterPoint3> Intersection(Prim::Line3 line, Prim::Triangle3 triangle) noexcept {
    return Detail::AsPoint(Detail::HitTriangle(line.Origin, line.Direction.AsVector(), Detail::CurveDomain::Line, triangle));
}



/// 直线是否打到三角形，包括共面重叠。`noexcept`。
[[nodiscard]] bool Intersects(Prim::Line3 line, Prim::Triangle3 triangle) noexcept {
    return Detail::HitTriangle(line.Origin, line.Direction.AsVector(), Detail::CurveDomain::Line, triangle).Kind != Detail::HitKind::Miss;
}



/// 射线与轴对齐盒。空盒不相交。起点在体内时 `Enter` 为 0。相切时 `Enter == Exit`。两个参数都 `>= 0`。`noexcept`。
[[nodiscard]] std::optional<ParameterInterval2> Intersection(Prim::Ray2 ray, Linear::Box2 box) noexcept {
    return Detail::IntersectBox<ParameterInterval2>(ray.Origin, ray.Direction.AsVector(), box, 0.0, Detail::Infinity(), 2);
}



/// 射线是否打到轴对齐盒。空盒不相交。`noexcept`。
[[nodiscard]] bool Intersects(Prim::Ray2 ray, Linear::Box2 box) noexcept {
    return Intersection(ray, box).has_value();
}



/// 线段与轴对齐盒。参数落在 `[0, 1]`。起点在体内时 `Enter` 为 0。空盒不相交。`noexcept`。
[[nodiscard]] std::optional<ParameterInterval2> Intersection(Prim::Segment2 segment, Linear::Box2 box) noexcept {
    return Detail::IntersectBox<ParameterInterval2>(segment.A, segment.B - segment.A, box, 0.0, 1.0, 2);
}



/// 线段是否打到轴对齐盒。空盒不相交。`noexcept`。
[[nodiscard]] bool Intersects(Prim::Segment2 segment, Linear::Box2 box) noexcept {
    return Intersection(segment, box).has_value();
}



/// 直线与轴对齐盒。参数是有符号距离。空盒不相交。`noexcept`。
[[nodiscard]] std::optional<ParameterInterval2> Intersection(Prim::Line2 line, Linear::Box2 box) noexcept {
    return Detail::IntersectBox<ParameterInterval2>(line.Origin, line.Direction.AsVector(), box, -Detail::Infinity(), Detail::Infinity(), 2);
}



/// 直线是否打到轴对齐盒。空盒不相交。`noexcept`。
[[nodiscard]] bool Intersects(Prim::Line2 line, Linear::Box2 box) noexcept {
    return Intersection(line, box).has_value();
}



/// 射线与三维轴对齐盒。空盒不相交。约定与二维相同。`noexcept`。
[[nodiscard]] std::optional<ParameterInterval3> Intersection(Prim::Ray3 ray, Linear::Box3 box) noexcept {
    return Detail::IntersectBox<ParameterInterval3>(ray.Origin, ray.Direction.AsVector(), box, 0.0, Detail::Infinity(), 3);
}



/// 射线是否打到三维轴对齐盒。空盒不相交。`noexcept`。
[[nodiscard]] bool Intersects(Prim::Ray3 ray, Linear::Box3 box) noexcept {
    return Intersection(ray, box).has_value();
}



/// 线段与三维轴对齐盒。空盒不相交。`noexcept`。
[[nodiscard]] std::optional<ParameterInterval3> Intersection(Prim::Segment3 segment, Linear::Box3 box) noexcept {
    return Detail::IntersectBox<ParameterInterval3>(segment.A, segment.B - segment.A, box, 0.0, 1.0, 3);
}



/// 线段是否打到三维轴对齐盒。空盒不相交。`noexcept`。
[[nodiscard]] bool Intersects(Prim::Segment3 segment, Linear::Box3 box) noexcept {
    return Intersection(segment, box).has_value();
}



/// 直线与三维轴对齐盒。空盒不相交。`noexcept`。
[[nodiscard]] std::optional<ParameterInterval3> Intersection(Prim::Line3 line, Linear::Box3 box) noexcept {
    return Detail::IntersectBox<ParameterInterval3>(line.Origin, line.Direction.AsVector(), box, -Detail::Infinity(), Detail::Infinity(), 3);
}



/// 直线是否打到三维轴对齐盒。空盒不相交。`noexcept`。
[[nodiscard]] bool Intersects(Prim::Line3 line, Linear::Box3 box) noexcept {
    return Intersection(line, box).has_value();
}



/// 射线与二维有向盒。先用 `Coordinate.ToLocal` 变到盒的标架，再对以原点为中心、半轴为 `HalfExtent` 的盒做平板测试。不提供直线版本。`noexcept`。
[[nodiscard]] std::optional<ParameterInterval2> Intersection(Prim::Ray2 ray, const Linear::OrientedBox2& box) noexcept {
    const Linear::Point2 low{-box.HalfExtent.X, -box.HalfExtent.Y};
    const Linear::Point2 high{box.HalfExtent.X, box.HalfExtent.Y};
    return Detail::IntersectExtents<ParameterInterval2>(
        box.Coordinate.ToLocal(ray.Origin), box.Coordinate.ToLocal(ray.Direction.AsVector()), low, high, 0.0,
        Detail::Infinity(), 2, ray.Origin, ray.Direction.AsVector());
}



/// 射线是否打到二维有向盒。`noexcept`。
[[nodiscard]] bool Intersects(Prim::Ray2 ray, const Linear::OrientedBox2& box) noexcept {
    return Intersection(ray, box).has_value();
}



/// 线段与二维有向盒。参数仍是原线段上的 `[0, 1]`。`noexcept`。
[[nodiscard]] std::optional<ParameterInterval2> Intersection(Prim::Segment2 segment, const Linear::OrientedBox2& box) noexcept {
    const Linear::Point2 localStart = box.Coordinate.ToLocal(segment.A);
    const Linear::Point2 localEnd = box.Coordinate.ToLocal(segment.B);
    const Linear::Point2 low{-box.HalfExtent.X, -box.HalfExtent.Y};
    const Linear::Point2 high{box.HalfExtent.X, box.HalfExtent.Y};
    return Detail::IntersectExtents<ParameterInterval2>(localStart, localEnd - localStart, low, high, 0.0, 1.0, 2, segment.A, segment.B - segment.A);
}



/// 线段是否打到二维有向盒。`noexcept`。
[[nodiscard]] bool Intersects(Prim::Segment2 segment, const Linear::OrientedBox2& box) noexcept {
    return Intersection(segment, box).has_value();
}



/// 射线与三维有向盒。先 `Coordinate.ToLocal`，再对 `[-HalfExtent, HalfExtent]` 做平板测试。`noexcept`。
[[nodiscard]] std::optional<ParameterInterval3> Intersection(Prim::Ray3 ray, const Linear::OrientedBox3& box) noexcept {
    const Linear::Point3 low{-box.HalfExtent.X, -box.HalfExtent.Y, -box.HalfExtent.Z};
    const Linear::Point3 high{box.HalfExtent.X, box.HalfExtent.Y, box.HalfExtent.Z};
    return Detail::IntersectExtents<ParameterInterval3>(
        box.Coordinate.ToLocal(ray.Origin), box.Coordinate.ToLocal(ray.Direction.AsVector()), low, high, 0.0,
        Detail::Infinity(), 3, ray.Origin, ray.Direction.AsVector());
}



/// 射线是否打到三维有向盒。`noexcept`。
[[nodiscard]] bool Intersects(Prim::Ray3 ray, const Linear::OrientedBox3& box) noexcept {
    return Intersection(ray, box).has_value();
}



/// 线段与三维有向盒。参数仍是原线段上的 `[0, 1]`。`noexcept`。
[[nodiscard]] std::optional<ParameterInterval3> Intersection(Prim::Segment3 segment, const Linear::OrientedBox3& box) noexcept {
    const Linear::Point3 localStart = box.Coordinate.ToLocal(segment.A);
    const Linear::Point3 localEnd = box.Coordinate.ToLocal(segment.B);
    const Linear::Point3 low{-box.HalfExtent.X, -box.HalfExtent.Y, -box.HalfExtent.Z};
    const Linear::Point3 high{box.HalfExtent.X, box.HalfExtent.Y, box.HalfExtent.Z};
    return Detail::IntersectExtents<ParameterInterval3>(localStart, localEnd - localStart, low, high, 0.0, 1.0, 3, segment.A, segment.B - segment.A);
}



/// 线段是否打到三维有向盒。`noexcept`。
[[nodiscard]] bool Intersects(Prim::Segment3 segment, const Linear::OrientedBox3& box) noexcept {
    return Intersection(segment, box).has_value();
}



/// 两条线段的平方距离。相交时为 0。`noexcept`。
[[nodiscard]] double DistanceSquared(Prim::Segment2 first, Prim::Segment2 second) noexcept {
    return Detail::SegmentDistanceSquared(first.A, first.B, second.A, second.B);
}



/// 两条线段的距离，即 `DistanceSquared` 的平方根，非负。`noexcept`。
[[nodiscard]] double Distance(Prim::Segment2 first, Prim::Segment2 second) noexcept {
    return std::sqrt(DistanceSquared(first, second));
}



/// 两条三维线段的平方距离。相交时为 0。`noexcept`。
[[nodiscard]] double DistanceSquared(Prim::Segment3 first, Prim::Segment3 second) noexcept {
    return Detail::SegmentDistanceSquared(first.A, first.B, second.A, second.B);
}



/// 两条三维线段的距离，非负。`noexcept`。
[[nodiscard]] double Distance(Prim::Segment3 first, Prim::Segment3 second) noexcept {
    return std::sqrt(DistanceSquared(first, second));
}



/// 两个三角形的平方距离。相交时为 0；否则是顶点到另一三角形、以及边到另一三角形的边的最小值。`noexcept`。
[[nodiscard]] double DistanceSquared(Prim::Triangle3 first, Prim::Triangle3 second) noexcept {
    return Detail::TriangleDistanceSquared(first, second);
}



/// 两个三角形的距离，非负。相交时为 0。`noexcept`。
[[nodiscard]] double Distance(Prim::Triangle3 first, Prim::Triangle3 second) noexcept {
    return std::sqrt(DistanceSquared(first, second));
}

} // namespace DragonGeo::Query
