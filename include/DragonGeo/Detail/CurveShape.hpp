#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>
#include <span>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include <DragonGeo/Core/Numeric.hpp>
#include <DragonGeo/Detail/CurveParameter.hpp>
#include <DragonGeo/Prim/Winding.hpp>

namespace DragonGeo::Detail {

/// 有符号量对应的绕行方向。正数为逆时针，负数为顺时针，零为退化。
template <typename Scalar> [[nodiscard]] std::optional<Prim::Winding> WindingFromSign(Scalar signedValue) noexcept {
    if (signedValue > Scalar{0}) {
        return Prim::Winding::CounterClockwise;
    }
    if (signedValue < Scalar{0}) {
        return Prim::Winding::Clockwise;
    }
    return Prim::Winding::Degenerate;
}

/// 三个分量里绝对值最大的那个。并列时保留先出现的，顺序是 X、Y、Z。
template <typename Scalar> [[nodiscard]] Scalar DominantComponent(Scalar x, Scalar y, Scalar z) noexcept {
    Scalar component = x;
    Scalar magnitude = Core::AbsoluteValue(x);
    const Scalar absY = Core::AbsoluteValue(y);
    if (absY > magnitude) {
        component = y;
        magnitude = absY;
    }
    const Scalar absZ = Core::AbsoluteValue(z);
    if (absZ > magnitude) {
        component = z;
    }
    return component;
}

/// 与线段 `Direction` 相同：两端重合时为空，否则取弦的单位方向。
template <typename Point> [[nodiscard]] auto ChordDirection(Point start, Point end) noexcept {
    if (start == end) {
        return decltype((end - start).Normalized()){};
    }
    return (end - start).Normalized();
}

/// 点在弦上的无夹紧参数。零长度或方向无法单位化时为 0。
template <typename Point> [[nodiscard]] auto ChordParameter(Point start, Point end, Point query) noexcept {
    using Scalar = decltype(start.X);
    const auto direction = ChordDirection(start, end);
    if (!direction.has_value()) {
        return Scalar{0};
    }
    const Scalar length = start.DistanceTo(end);
    if (!(length > Scalar{0}) || !Core::IsFinite(length)) {
        return Scalar{0};
    }
    return ProjectParameter(start, *direction, query) / length;
}

/// 点到弦所在直线的距离。方向不可用时退化为到起点的距离。
template <typename Point> [[nodiscard]] auto DistanceToChordLine(Point start, Point end, Point query) noexcept {
    using Scalar = decltype(start.X);
    const auto direction = ChordDirection(start, end);
    const Scalar length = start.DistanceTo(end);
    if (!direction.has_value() || !(length > Scalar{0}) || !Core::IsFinite(length)) {
        return query.DistanceTo(start);
    }
    const Scalar parameter = ProjectParameter(start, *direction, query) / length;
    return query.DistanceTo(start + (end - start) * parameter);
}

/// 变换线段两端。结果非有限，或原弦能单位化而变换后的弦不能时，不修改端点并返回假。
template <typename Point, typename Transform>
[[nodiscard]] bool TryTransformChord(Point& start, Point& end, const Transform& transform) noexcept {
    const Point movedStart = transform.TransformPoint(start);
    const Point movedEnd = transform.TransformPoint(end);
    if (!CoordinatesAreFinite(movedStart) || !CoordinatesAreFinite(movedEnd)) {
        return false;
    }
    const auto chord = end - start;
    const auto transformedChord = transform.TransformVector(chord);
    if (chord.Normalized().has_value() && (!CoordinatesAreFinite(transformedChord) || !transformedChord.Normalized().has_value())) {
        return false;
    }
    start = movedStart;
    end = movedEnd;
    return true;
}

/// 变换原点与单位方向。新原点或新方向非有限，或方向无法重新单位化时，不修改并返回假。
template <typename Point, typename Direction, typename Transform>
[[nodiscard]] bool TryTransformFrame(Point& origin, Direction& direction, const Transform& transform) noexcept {
    const Point moved = transform.TransformPoint(origin);
    const auto transformedDirection = transform.TransformVector(direction.AsVector());
    if (!CoordinatesAreFinite(moved) || !CoordinatesAreFinite(transformedDirection)) {
        return false;
    }
    const auto unit = transformedDirection.Normalized();
    if (!unit.has_value()) {
        return false;
    }
    origin = moved;
    direction = *unit;
    return true;
}

/// 把单位方向送进变换后再单位化。失败时沿用未检查的结果，与射线和直线的旋转、镜像一致。
template <typename Direction, typename Transform>
[[nodiscard]] Direction RenormalizedDirection(const Direction& direction, const Transform& transform) noexcept {
    const auto transformed = transform.TransformVector(direction.AsVector());
    const auto unit = transformed.Normalized();
    if (unit.has_value()) {
        return *unit;
    }
    return Direction::FromNormalizedUnchecked(transformed);
}

/// 变换三角形三个顶点。任一结果非有限时不修改并返回假。
template <typename Point, typename Transform>
[[nodiscard]] bool TryTransformTriangle(Point& a, Point& b, Point& c, const Transform& transform) noexcept {
    const Point movedA = transform.TransformPoint(a);
    const Point movedB = transform.TransformPoint(b);
    const Point movedC = transform.TransformPoint(c);
    if (!CoordinatesAreFinite(movedA) || !CoordinatesAreFinite(movedB) || !CoordinatesAreFinite(movedC)) {
        return false;
    }
    a = movedA;
    b = movedB;
    c = movedC;
    return true;
}

template <typename Point, typename Transform>
void TransformVertices(Point& a, Point& b, Point& c, const Transform& transform) noexcept {
    a = transform.TransformPoint(a);
    b = transform.TransformPoint(b);
    c = transform.TransformPoint(c);
}

template <typename Point> [[nodiscard]] auto ChainLength(const Point* points, std::size_t segmentCount) noexcept {
    using Scalar = decltype(points[0].X);
    Scalar length{0};
    for (std::size_t index = 0; index < segmentCount; ++index) {
        length += points[index].DistanceTo(points[index + 1]);
    }
    return length;
}

/// `t == segmentCount` 时取最后一个点。调用方保证 `points` 有 `segmentCount + 1` 个点。
template <typename Point, typename Scalar>
[[nodiscard]] Point LocateOnChain(const Point* points, std::size_t segmentCount, Scalar t) noexcept {
    if (t == static_cast<Scalar>(segmentCount)) {
        return points[segmentCount];
    }
    const auto index = static_cast<std::size_t>(t);
    const Scalar local = t - static_cast<Scalar>(index);
    return points[index] + (points[index + 1] - points[index]) * local;
}

template <typename Scalar> [[nodiscard]] std::size_t EdgeIndexOnChain(std::size_t segmentCount, Scalar t) noexcept {
    if (t == static_cast<Scalar>(segmentCount)) {
        return segmentCount - 1;
    }
    return static_cast<std::size_t>(t);
}

/// 闭合时保持首尾那个重合点不动，只反转中间顶点。
template <typename Point> void ReverseChain(Point* points, std::size_t count, bool closed) noexcept {
    if (closed) {
        std::reverse(points + 1, points + count - 1);
        return;
    }
    std::reverse(points, points + count);
}

template <typename Point, typename Vector> void TranslatePoints(Point* points, std::size_t count, Vector vector) noexcept {
    for (std::size_t index = 0; index < count; ++index) {
        points[index] = points[index] + vector;
    }
}

template <typename Point, typename Transform> void ApplyTransform(Point* points, std::size_t count, const Transform& transform) noexcept {
    for (std::size_t index = 0; index < count; ++index) {
        points[index] = transform.TransformPoint(points[index]);
    }
}

/// 先检查每个点的像都有限，再写入。有一个非有限就保持原列并返回假。
template <typename Point, typename Transform>
[[nodiscard]] bool TryTransformPoints(Point* points, std::size_t count, const Transform& transform) noexcept {
    for (std::size_t index = 0; index < count; ++index) {
        if (!CoordinatesAreFinite(transform.TransformPoint(points[index]))) {
            return false;
        }
    }
    ApplyTransform(points, count, transform);
    return true;
}

/// 前 `count` 个点的坐标平均。点列至少要有一个点。
template <typename Point> [[nodiscard]] Point AverageOf(const Point* points, std::size_t count) noexcept {
    using Scalar = decltype(points[0].X);
    Scalar x{0};
    Scalar y{0};
    Scalar z{0};
    for (std::size_t index = 0; index < count; ++index) {
        x += points[index].X;
        y += points[index].Y;
        if constexpr (requires { points[0].Z; }) {
            z += points[index].Z;
        }
    }
    const Scalar divisor = static_cast<Scalar>(count);
    if constexpr (requires { points[0].Z; }) {
        return Point{x / divisor, y / divisor, z / divisor};
    } else {
        return Point{x / divisor, y / divisor};
    }
}

template <typename Segment, typename Polyline, typename Point, typename Scalar, typename Locate>
[[nodiscard]] std::optional<std::variant<Segment, Polyline>> ChainSubcurve(std::size_t segmentCount, Linear::IntervalT<Scalar> interval,
    Linear::IntervalT<Scalar> domain, const Point* points, Locate&& locate) {
    using Curve = std::variant<Segment, Polyline>;
    if (!IsFiniteSubinterval(interval, domain)) {
        return std::nullopt;
    }
    for (std::size_t edge = 0; edge < segmentCount; ++edge) {
        const Scalar edgeMin = static_cast<Scalar>(edge);
        const Scalar edgeMax = edgeMin + Scalar{1};
        if (interval.Min >= edgeMin && interval.Max <= edgeMax) {
            return Curve{Segment{locate(interval.Min), locate(interval.Max)}};
        }
    }

    std::vector<Point> corners;
    corners.push_back(locate(interval.Min));
    for (std::size_t vertex = 1; vertex < segmentCount; ++vertex) {
        const Scalar parameter = static_cast<Scalar>(vertex);
        if (parameter > interval.Min && parameter < interval.Max) {
            corners.push_back(points[vertex]);
        }
    }
    corners.push_back(locate(interval.Max));
    auto polyline = Polyline::FromPoints(corners);
    if (!polyline.has_value()) {
        return std::nullopt;
    }
    return Curve{std::move(*polyline)};
}

template <typename Segment, typename Polyline, typename Scalar, typename Locate>
[[nodiscard]] std::optional<std::variant<Segment, Polyline>> TriangleSubcurve(Linear::IntervalT<Scalar> interval,
    Linear::IntervalT<Scalar> domain, Locate&& locate) {
    using Curve = std::variant<Segment, Polyline>;
    using Point = std::remove_cvref_t<decltype(locate(interval.Min))>;
    if (!IsFiniteSubinterval(interval, domain)) {
        return std::nullopt;
    }
    for (int edge = 0; edge < 3; ++edge) {
        const Scalar edgeMin = static_cast<Scalar>(edge);
        const Scalar edgeMax = edgeMin + Scalar{1};
        if (interval.Min >= edgeMin && interval.Max <= edgeMax) {
            return Curve{Segment{locate(interval.Min), locate(interval.Max)}};
        }
    }

    Point corners[4];
    int count = 0;
    corners[count++] = locate(interval.Min);
    for (int vertex = 1; vertex <= 2; ++vertex) {
        const Scalar parameter = static_cast<Scalar>(vertex);
        if (parameter > interval.Min && parameter < interval.Max) {
            corners[count++] = locate(parameter);
        }
    }
    corners[count++] = locate(interval.Max);

    auto polyline = Polyline::FromPoints(std::span<const Point>{corners, static_cast<std::size_t>(count)});
    if (!polyline.has_value()) {
        return std::nullopt;
    }
    return Curve{std::move(*polyline)};
}

template <typename Point, typename Scalar> [[nodiscard]] Point LocateOnTriangle(Point a, Point b, Point c, Scalar t) noexcept {
    const Point vertices[]{a, b, c, a};
    const int edge = static_cast<int>(t);
    const int index = edge >= 3 ? 2 : edge;
    const Scalar local = t - static_cast<Scalar>(index);
    return vertices[index] + (vertices[index + 1] - vertices[index]) * local;
}

template <typename Scalar> [[nodiscard]] int TriangleEdgeIndex(Scalar t) noexcept {
    if (t == Scalar{3}) {
        return 2;
    }
    const int edge = static_cast<int>(t);
    return edge >= 3 ? 2 : edge;
}

/// 距离不超过容差。先开方再比较，与三角形和三维折线的边界判定一致。
template <typename Scalar> [[nodiscard]] bool DistanceWithin(Scalar distanceSquared, Scalar allowance) noexcept {
    return std::sqrt(distanceSquared) <= allowance;
}

} // namespace DragonGeo::Detail
