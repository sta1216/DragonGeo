#pragma once

#include <cstddef>

#include <DragonGeo/Core/Numeric.hpp>
#include <DragonGeo/Detail/CurveParameter.hpp>
#include <DragonGeo/Linear/Point2.hpp>
#include <DragonGeo/Linear/Point3.hpp>
#include <DragonGeo/Linear/Vector3.hpp>
#include <DragonGeo/Predicates/Predicates.hpp>
#include <DragonGeo/Prim/Segment2.hpp>
#include <DragonGeo/Prim/Segment3.hpp>

namespace DragonGeo::Detail {

/// 查询点是否精确落在二维线段上。`Orient2d == 0` 且参数在 `[0, 1]`。零长度线段只接受那个端点。
[[nodiscard]] inline bool PointOnSegment2(Linear::Point2 start, Linear::Point2 end, Linear::Point2 query) noexcept {
    if (Predicates::Orient2d(start, end, query) != 0) {
        return false;
    }
    if (start == end) {
        return query == start;
    }
    const Linear::Vector2 chord = end - start;
    const double lengthSquared = chord.LengthSquared();
    if (!(lengthSquared > 0.0)) {
        return query == start || query == end;
    }
    const double parameter = (query - start).Dot(chord) / lengthSquared;
    return parameter >= 0.0 && parameter <= 1.0;
}

/// 三点在三维空间中是否共线。三个坐标平面上的 `Orient2d` 都为零才算共线。
[[nodiscard]] inline bool PointsAreCollinear3(Linear::Point3 first, Linear::Point3 second, Linear::Point3 third) noexcept {
    const auto orient = [](double ax, double ay, double bx, double by, double cx, double cy) noexcept {
        return Predicates::Orient2d(Linear::Point2{ax, ay}, Linear::Point2{bx, by}, Linear::Point2{cx, cy});
    };
    const bool xy = orient(first.X, first.Y, second.X, second.Y, third.X, third.Y) == 0;
    const bool yz = orient(first.Y, first.Z, second.Y, second.Z, third.Y, third.Z) == 0;
    const bool zx = orient(first.Z, first.X, second.Z, second.X, third.Z, third.X) == 0;
    return xy && yz && zx;
}

/// 查询点是否精确落在三维线段上。先要求共线，再把参数夹到 `[0, 1]`。零长度线段只接受那个端点。
[[nodiscard]] inline bool PointOnSegment3(Linear::Point3 start, Linear::Point3 end, Linear::Point3 query) noexcept {
    if (start == end) {
        return query == start;
    }
    if (!PointsAreCollinear3(start, end, query)) {
        return false;
    }
    const Linear::Vector3 chord = end - start;
    const double lengthSquared = chord.LengthSquared();
    if (!(lengthSquared > 0.0)) {
        return query == start || query == end;
    }
    const double parameter = (query - start).Dot(chord) / lengthSquared;
    return parameter >= 0.0 && parameter <= 1.0;
}

/// 闭合折线的环绕数。`pointAt(i)` 是第 i 个顶点，最后一条边连到 `pointAt(edgeCount)`。非零表示在内部。
template <typename PointAt>
[[nodiscard]] int WindingNumber(std::size_t edgeCount, Linear::Point2 query, PointAt&& pointAt) noexcept {
    int winding = 0;
    for (std::size_t edge = 0; edge < edgeCount; ++edge) {
        const Linear::Point2 start = pointAt(edge);
        const Linear::Point2 end = pointAt(edge + 1);
        if (start.Y <= query.Y) {
            if (end.Y > query.Y && Predicates::Orient2d(start, end, query) > 0) {
                ++winding;
            }
        } else if (end.Y <= query.Y && Predicates::Orient2d(start, end, query) < 0) {
            --winding;
        }
    }
    return winding;
}

/// 绝对值最大的分量。并列时按 X、Y、Z 取先出现的轴。返回 0、1 或 2。
template <typename Scalar> [[nodiscard]] int DominantAxis(Linear::Vector3T<Scalar> vector) noexcept {
    const double absX = Core::AbsoluteValue(static_cast<double>(vector.X));
    const double absY = Core::AbsoluteValue(static_cast<double>(vector.Y));
    const double absZ = Core::AbsoluteValue(static_cast<double>(vector.Z));
    if (absX >= absY && absX >= absZ) {
        return 0;
    }
    if (absY >= absZ) {
        return 1;
    }
    return 2;
}

/// 丢掉 `dropped` 轴，剩下两个分量按 X、Y、Z 的顺序组成二维点。
[[nodiscard]] inline Linear::Point2 DropAxis(Linear::Point3 point, int dropped) noexcept {
    if (dropped == 0) {
        return Linear::Point2{point.Y, point.Z};
    }
    if (dropped == 1) {
        return Linear::Point2{point.X, point.Z};
    }
    return Linear::Point2{point.X, point.Y};
}

/// 边上的参数：边号加上夹到 `[0, 1]` 的局部参数。接缝折回由调用方处理。
template <typename Index, typename Point>
[[nodiscard]] auto EdgeParameter(Index edge, Point from, Point to, Point closest) noexcept {
    using Scalar = decltype(from.X);
    Scalar parameter = static_cast<Scalar>(edge);
    const Scalar length = from.DistanceTo(to);
    if (length > Scalar{0} && Core::IsFinite(length)) {
        const auto direction = (to - from).Normalized();
        if (direction.has_value()) {
            Scalar local = ProjectParameter(from, *direction, closest) / length;
            if (local < Scalar{0}) {
                local = Scalar{0};
            } else if (local > Scalar{1}) {
                local = Scalar{1};
            }
            parameter += local;
        }
    }
    return parameter;
}

template <typename Point> [[nodiscard]] auto EdgeSegment(Point from, Point to) noexcept {
    using Scalar = decltype(from.X);
    if constexpr (requires { from.Z; }) {
        return Prim::Segment3T<Scalar>{from, to};
    } else {
        return Prim::Segment2T<Scalar>{from, to};
    }
}

/// 沿相邻顶点找最近点。距离更近者优先，距离相等时参数更小者优先。`wrapSeam` 时参数上端折回 0。
template <typename Location, typename Point>
[[nodiscard]] Location ClosestOnChain(const Point* vertices, std::size_t vertexCount, Point query, bool wrapSeam) noexcept {
    using Scalar = decltype(query.X);
    Location best{};
    if (vertexCount < 2) {
        return best;
    }
    const std::size_t edges = vertexCount - 1;
    for (std::size_t edge = 0; edge < edges; ++edge) {
        const Point from = vertices[edge];
        const Point to = vertices[edge + 1];
        const Point candidate = EdgeSegment(from, to).ClosestPoint(query);
        const Scalar distanceSquared = (query - candidate).LengthSquared();
        Scalar parameter = EdgeParameter(edge, from, to, candidate);
        if (wrapSeam && parameter == static_cast<Scalar>(edges)) {
            parameter = Scalar{0};
        }
        if (edge == 0 || distanceSquared < best.DistanceSquared || (distanceSquared == best.DistanceSquared && parameter < best.Parameter)) {
            best = Location{distanceSquared, parameter, candidate};
        }
    }
    return best;
}

/// 包围盒对角线。退化或非有限时用 1，作为容差的尺度。
template <typename Box> [[nodiscard]] auto DiagonalScale(const Box& box) noexcept {
    const auto diagonal = box.Extent().Length();
    using Scalar = decltype(diagonal);
    if (diagonal > Scalar{0} && Core::IsFinite(diagonal)) {
        return diagonal;
    }
    return Scalar{1};
}

template <typename Point> [[nodiscard]] auto UnitDirection(Point from, Point to) noexcept -> decltype((to - from).Normalized()) {
    using Scalar = decltype(from.X);
    const Scalar length = from.DistanceTo(to);
    if (!(length > Scalar{0}) || !Core::IsFinite(length)) {
        return std::nullopt;
    }
    return (to - from).Normalized();
}

} // namespace DragonGeo::Detail
