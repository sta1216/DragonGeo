#pragma once

#include <cmath>
#include <concepts>
#include <optional>
#include <span>

#include <DragonGeo/Core/Numeric.hpp>
#include <DragonGeo/Core/Tolerance.hpp>
#include <DragonGeo/Detail/CurveParameter.hpp>
#include <DragonGeo/Linear/Box2.hpp>
#include <DragonGeo/Linear/Interval.hpp>
#include <DragonGeo/Linear/Point2.hpp>
#include <DragonGeo/Linear/Transform2.hpp>
#include <DragonGeo/Linear/UnitVector2.hpp>
#include <DragonGeo/Predicates/Predicates.hpp>
#include <DragonGeo/Prim/Segment2.hpp>
#include <DragonGeo/Prim/Winding.hpp>

namespace DragonGeo::Prim {

/// 二维三角形：由顶点 `A`、`B`、`C` 给出。
///
/// 参数域是 `[0, 3]`。三条边各占长度 1，顺序是 `A→B`、`B→C`、`C→A`。
/// 参数中点 `1.5` 落在第二条边的中点，不是三角形重心。
/// 非有限坐标可以存入；`IsValid` 仅在六个分量都有限时为真。
/// 零面积（三点共线）仍视为有效。
template <typename Scalar>
struct Triangle2T {
    using ScalarType = Scalar;

    // 同 Point2T / Box2T：不声明任何构造函数，以保持聚合性。
    Linear::Point2T<Scalar> A{};
    Linear::Point2T<Scalar> B{};
    Linear::Point2T<Scalar> C{};

    /// 三个顶点的六个分量均为有限值时为真。零面积（三点共线）仍视为有效。
    [[nodiscard]] constexpr bool IsValid() const noexcept {
        using Core::IsFinite;
        return IsFinite(A.X) && IsFinite(A.Y) && IsFinite(B.X) && IsFinite(B.Y) && IsFinite(C.X)
            && IsFinite(C.Y);
    }

    /// 有向面积，逆时针为正，顺时针为负，三点共线时为 0。
    /// 等于 `(B - A) × (C - A) / 2`。
    [[nodiscard]] constexpr Scalar SignedArea() const noexcept {
        return Scalar{0.5} * (B - A).Cross(C - A);
    }

    /// 参数域 `[0, 3]`。
    [[nodiscard]] constexpr Linear::IntervalT<Scalar> Domain() const noexcept {
        return {Scalar{0}, Scalar{3}};
    }

    /// `t` 不在 `[0, 3]` 或非有限时为空。
    /// `0` 与 `3` 都是顶点 `A`。`1.5` 是第二条边的中点，在边界上，不是重心。
    [[nodiscard]] constexpr std::optional<Linear::Point2T<Scalar>> PointAt(Scalar t) const noexcept {
        if (!Detail::IsAcceptedParameter(t, Domain())) {
            return std::nullopt;
        }
        return Locate(t);
    }

    /// 参数中点，即 `PointAt(1.5)`。它在边界上，不是填充区域的重心。
    [[nodiscard]] constexpr std::optional<Linear::Point2T<Scalar>> MidPoint() const noexcept {
        return PointAt(Scalar{1.5});
    }

    /// 点在三角形内（含边界）时为真。只对 `double` 提供。
    ///
    /// 面积非零时，三次 `Orient2d`（`A,B`、`B,C`、`C,A` 对查询点）全部 `>= 0`
    /// 或全部 `<= 0`。边界上至少有一次为 0，仍算内部。
    /// 面积为零（`Orient2d(A,B,C) == 0`）时，点必须落在某条退化边上：
    /// 对该边 `Orient2d` 为 0，且点在边上的参数落在 `[0, 1]`。
    /// 零长度边只包含与该顶点重合的点。
    template <typename S = Scalar>
        requires std::same_as<Scalar, double> && std::same_as<S, double>
    [[nodiscard]] bool Contains(Linear::Point2T<S> point) const noexcept {
        const auto onEdge = [](Linear::Point2 start, Linear::Point2 end,
                               Linear::Point2 query) noexcept {
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
        };

        if (Predicates::Orient2d(A, B, C) == 0) {
            return onEdge(A, B, point) || onEdge(B, C, point) || onEdge(C, A, point);
        }
        const int ab = Predicates::Orient2d(A, B, point);
        const int bc = Predicates::Orient2d(B, C, point);
        const int ca = Predicates::Orient2d(C, A, point);
        return (ab >= 0 && bc >= 0 && ca >= 0) || (ab <= 0 && bc <= 0 && ca <= 0);
    }

    /// 闭合曲线的两端都是接缝上的顶点 `A`。
    [[nodiscard]] constexpr std::optional<Linear::Point2T<Scalar>> StartPoint() const noexcept {
        return A;
    }

    /// 闭合曲线的两端都是接缝上的顶点 `A`。
    [[nodiscard]] constexpr std::optional<Linear::Point2T<Scalar>> EndPoint() const noexcept {
        return A;
    }

    /// `A→B` 的单位方向。这条边没有有限正长度时为空。
    [[nodiscard]] std::optional<Linear::UnitVector2T<Scalar>> StartTangent() const noexcept {
        return UnitEdge(A, B);
    }

    /// `C→A` 的单位方向。这条边没有有限正长度时为空。
    [[nodiscard]] std::optional<Linear::UnitVector2T<Scalar>> EndTangent() const noexcept {
        return UnitEdge(C, A);
    }

    /// `B→C` 的单位方向。这条边没有有限正长度时为空。
    [[nodiscard]] std::optional<Linear::UnitVector2T<Scalar>> MidTangent() const noexcept {
        return UnitEdge(B, C);
    }

    /// `t` 不在 `[0, 3]`、非有限，或所在边没有有限正长度时为空。
    /// `t == 3` 用边 `C→A`。其余参数用 `floor(t)`：`0` 是 `A→B`，`1` 是 `B→C`，`2` 是 `C→A`。
    [[nodiscard]] std::optional<Linear::UnitVector2T<Scalar>> TangentAt(Scalar t) const noexcept {
        if (!Detail::IsAcceptedParameter(t, Domain())) {
            return std::nullopt;
        }
        const Linear::Point2T<Scalar> vertices[]{A, B, C, A};
        const int edge = EdgeIndex(t);
        return UnitEdge(vertices[edge], vertices[edge + 1]);
    }

    [[nodiscard]] constexpr bool IsClosed() const noexcept {
        return true;
    }

    /// 周长 `|AB| + |BC| + |CA|`，非负。
    [[nodiscard]] Scalar Length() const noexcept {
        return A.DistanceTo(B) + B.DistanceTo(C) + C.DistanceTo(A);
    }

    /// 填充面积，即 `SignedArea` 的绝对值，含 0。
    [[nodiscard]] constexpr std::optional<Scalar> Area() const noexcept {
        return Core::AbsoluteValue(SignedArea());
    }

    /// 正的有符号面积是 `CounterClockwise`，负的是 `Clockwise`，恰好为 0 是 `Degenerate`。
    [[nodiscard]] constexpr std::optional<Winding> Orientation() const noexcept {
        const Scalar signedArea = SignedArea();
        if (signedArea > Scalar{0}) {
            return Winding::CounterClockwise;
        }
        if (signedArea < Scalar{0}) {
            return Winding::Clockwise;
        }
        return Winding::Degenerate;
    }

    /// 三个顶点的轴对齐包围盒。
    [[nodiscard]] Linear::Box2T<Scalar> Box() const noexcept {
        const Linear::Point2T<Scalar> vertices[]{A, B, C};
        return Linear::Box2T<Scalar>::FromPoints(std::span<const Linear::Point2T<Scalar>>{vertices});
    }

    /// `(A + B + C) / 3`。`SignedArea` 为 0 时为空。
    [[nodiscard]] constexpr std::optional<Linear::Point2T<Scalar>> Centroid() const noexcept {
        if (SignedArea() == Scalar{0}) {
            return std::nullopt;
        }
        return Linear::Point2T<Scalar>{
            (A.X + B.X + C.X) / Scalar{3},
            (A.Y + B.Y + C.Y) / Scalar{3},
        };
    }

    /// 点到边界（三条边构成的曲线）的距离不超过 `tolerance.Resolve(尺度)`。
    /// 严格落在内部、离每条边都比容差更远的点为假。
    /// 尺度是包围盒对角线；对角线不是大于 0 的有限数时用 `1`。
    [[nodiscard]] bool ContainsPoint(
        Linear::Point2T<Scalar> point,
        Core::ToleranceT<Scalar> tolerance = {}) const noexcept {
        const auto boundary = ClosestBoundary(point);
        return std::sqrt(boundary.DistanceSquared) <= tolerance.Resolve(BoundaryScale());
    }

    /// 边界上最近点的参数，落在 `[0, 3]`。`ContainsPoint` 为假时为空。
    /// 接缝上的点返回 `0`，不返回 `3`。
    [[nodiscard]] std::optional<Scalar> ParameterOf(
        Linear::Point2T<Scalar> point,
        Core::ToleranceT<Scalar> tolerance = {}) const noexcept {
        if (!ContainsPoint(point, tolerance)) {
            return std::nullopt;
        }
        return ClosestBoundary(point).Parameter;
    }

    constexpr void Translate(Linear::Vector2T<Scalar> vector) noexcept {
        A = A + vector;
        B = B + vector;
        C = C + vector;
    }

    /// 绕 `center` 逆时针旋转 `radians` 弧度。
    void Rotate(Linear::Point2T<Scalar> center, Scalar radians) noexcept {
        const Linear::Transform2T<Scalar> rotation =
            Linear::Transform2T<Scalar>::RotationAbout(center, radians);
        A = rotation.TransformPoint(A);
        B = rotation.TransformPoint(B);
        C = rotation.TransformPoint(C);
    }

    /// 关于过 `point`、法向为 `unitNormal` 的直线反射。
    constexpr void Mirror(
        Linear::Point2T<Scalar> point, Linear::UnitVector2T<Scalar> unitNormal) noexcept {
        const Linear::Transform2T<Scalar> mirror =
            Linear::Transform2T<Scalar>::Reflection(point, unitNormal);
        A = mirror.TransformPoint(A);
        B = mirror.TransformPoint(B);
        C = mirror.TransformPoint(C);
    }

    /// 变成 `{A, C, B}`。接缝仍是 `A`，`Orientation` 变号。
    constexpr void Reverse() noexcept {
        const Linear::Point2T<Scalar> vertexB = B;
        B = C;
        C = vertexB;
    }

    [[nodiscard]] constexpr Triangle2T Clone() const noexcept {
        return *this;
    }

    /// 变换三个顶点。任一结果分量非有限时返回 `false`，字段保持原样。
    /// 零面积仍然成功。
    [[nodiscard]] bool Transform(const Linear::Transform2T<Scalar>& transform) noexcept {
        const Linear::Point2T<Scalar> movedA = transform.TransformPoint(A);
        const Linear::Point2T<Scalar> movedB = transform.TransformPoint(B);
        const Linear::Point2T<Scalar> movedC = transform.TransformPoint(C);
        if (!Detail::CoordinatesAreFinite(movedA) || !Detail::CoordinatesAreFinite(movedB)
            || !Detail::CoordinatesAreFinite(movedC)) {
            return false;
        }
        A = movedA;
        B = movedB;
        C = movedC;
        return true;
    }

    /// 到填充三角形（面或边界）的平方距离。内部的点是 0。
    [[nodiscard]] Scalar DistanceSquared(Linear::Point2T<Scalar> point) const noexcept {
        return (point - ClosestPoint(point)).LengthSquared();
    }

    /// 到填充三角形的距离，即 `DistanceSquared` 的平方根。
    [[nodiscard]] Scalar Distance(Linear::Point2T<Scalar> point) const noexcept {
        return std::sqrt(DistanceSquared(point));
    }

    /// 填充三角形上的最近点。内部的点就是查询点本身；外部取三条边上的最近点。
    [[nodiscard]] Linear::Point2T<Scalar> ClosestPoint(Linear::Point2T<Scalar> point) const noexcept {
        if (ProjectsInside(point)) {
            return point;
        }
        return ClosestBoundary(point).Point;
    }

    /// 区间必须落在 `[0, 3]` 内、长度大于 0，并且整段落在同一条边上，否则为空。
    /// 端点非有限或区间倒置同样为空。不抛异常。
    ///
    /// 跨过顶点的区间需要 `Polyline`。该类型还不存在，因此不声明返回折线的重载。
    [[nodiscard]] constexpr std::optional<Segment2T<Scalar>> Subcurve(
        Linear::IntervalT<Scalar> interval) const noexcept {
        if (!Detail::IsFiniteSubinterval(interval, Domain())) {
            return std::nullopt;
        }
        for (int edge = 0; edge < 3; ++edge) {
            const Scalar edgeMin = static_cast<Scalar>(edge);
            const Scalar edgeMax = edgeMin + Scalar{1};
            if (interval.Min >= edgeMin && interval.Max <= edgeMax) {
                return Segment2T<Scalar>{Locate(interval.Min), Locate(interval.Max)};
            }
        }
        return std::nullopt;
    }

private:
    /// `t` 必须已落在 `[0, 3]` 内。`3` 映射回 `A`。
    [[nodiscard]] constexpr Linear::Point2T<Scalar> Locate(Scalar t) const noexcept {
        const Linear::Point2T<Scalar> vertices[]{A, B, C, A};
        const int edge = static_cast<int>(t);
        const int index = edge >= 3 ? 2 : edge;
        const Scalar local = t - static_cast<Scalar>(index);
        return vertices[index] + (vertices[index + 1] - vertices[index]) * local;
    }

    struct BoundaryLocation {
        Scalar DistanceSquared{};
        Scalar Parameter{};
        Linear::Point2T<Scalar> Point{};
    };

    /// `t == 3` 落在 `C→A`。其余用截断后的边号。
    [[nodiscard]] static int EdgeIndex(Scalar t) noexcept {
        if (t == Scalar{3}) {
            return 2;
        }
        const int edge = static_cast<int>(t);
        return edge >= 3 ? 2 : edge;
    }

    /// 边没有有限正长度，或方向无法归一化时为空。
    [[nodiscard]] static std::optional<Linear::UnitVector2T<Scalar>> UnitEdge(
        Linear::Point2T<Scalar> from, Linear::Point2T<Scalar> to) noexcept {
        const Scalar length = from.DistanceTo(to);
        if (!(length > Scalar{0}) || !Core::IsFinite(length)) {
            return std::nullopt;
        }
        return (to - from).Normalized();
    }

    /// 包围盒对角线。不是大于 0 的有限数时用 `1`。
    [[nodiscard]] Scalar BoundaryScale() const noexcept {
        const Scalar diagonal = Box().Extent().Length();
        if (diagonal > Scalar{0} && Core::IsFinite(diagonal)) {
            return diagonal;
        }
        return Scalar{1};
    }

    /// 查询点的重心坐标全部 `>= 0` 时，它落在填充三角形内（含边界）。
    [[nodiscard]] constexpr bool ProjectsInside(Linear::Point2T<Scalar> point) const noexcept {
        const Linear::Vector2T<Scalar> ab = B - A;
        const Linear::Vector2T<Scalar> ac = C - A;
        const Scalar denominator = ab.Cross(ac);
        if (denominator == Scalar{0} || !Core::IsFinite(denominator)) {
            return false;
        }
        const Linear::Vector2T<Scalar> ap = point - A;
        const Scalar alongB = ap.Cross(ac) / denominator;
        const Scalar alongC = ab.Cross(ap) / denominator;
        const Scalar alongA = Scalar{1} - alongB - alongC;
        return Core::IsFinite(alongA) && Core::IsFinite(alongB) && Core::IsFinite(alongC)
            && alongA >= Scalar{0} && alongB >= Scalar{0} && alongC >= Scalar{0};
    }

    /// 三条边上的最近点。距离相同取较小的参数。参数 `3` 折回接缝 `0`。
    [[nodiscard]] BoundaryLocation ClosestBoundary(Linear::Point2T<Scalar> point) const noexcept {
        const Linear::Point2T<Scalar> vertices[]{A, B, C, A};
        BoundaryLocation best{};
        for (int edge = 0; edge < 3; ++edge) {
            const Segment2T<Scalar> segment{vertices[edge], vertices[edge + 1]};
            const Linear::Point2T<Scalar> candidate = segment.ClosestPoint(point);
            const Scalar distanceSquared = (point - candidate).LengthSquared();
            const Scalar parameter =
                BoundaryParameter(edge, vertices[edge], vertices[edge + 1], candidate);
            if (edge == 0 || distanceSquared < best.DistanceSquared
                || (distanceSquared == best.DistanceSquared && parameter < best.Parameter)) {
                best = BoundaryLocation{distanceSquared, parameter, candidate};
            }
        }
        return best;
    }

    [[nodiscard]] static Scalar BoundaryParameter(
        int edge,
        Linear::Point2T<Scalar> from,
        Linear::Point2T<Scalar> to,
        Linear::Point2T<Scalar> closest) noexcept {
        Scalar parameter = static_cast<Scalar>(edge);
        const Scalar length = from.DistanceTo(to);
        if (length > Scalar{0} && Core::IsFinite(length)) {
            const auto direction = (to - from).Normalized();
            if (direction.has_value()) {
                Scalar local = Detail::ProjectParameter(from, *direction, closest) / length;
                if (local < Scalar{0}) {
                    local = Scalar{0};
                } else if (local > Scalar{1}) {
                    local = Scalar{1};
                }
                parameter += local;
            }
        }
        if (parameter == Scalar{3}) {
            parameter = Scalar{0};
        }
        return parameter;
    }
};

/// 逐字段比较。`operator!=` 由 C++20 自动生成，不手写。
template <typename Scalar>
[[nodiscard]] constexpr bool operator==(Triangle2T<Scalar> a, Triangle2T<Scalar> b) noexcept {
    return a.A == b.A && a.B == b.B && a.C == b.C;
}

using Triangle2 = Triangle2T<double>;
using Triangle2f = Triangle2T<float>;

} // namespace DragonGeo::Prim
