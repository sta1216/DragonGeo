#pragma once

#include <concepts>
#include <optional>
#include <span>
#include <utility>
#include <variant>

#include <DragonGeo/Core/Numeric.hpp>
#include <DragonGeo/Detail/CurveParameter.hpp>
#include <DragonGeo/Linear/Interval.hpp>
#include <DragonGeo/Linear/Point3.hpp>
#include <DragonGeo/Linear/Vector3.hpp>
#include <DragonGeo/Predicates/Predicates.hpp>
#include <DragonGeo/Prim/Polyline3.hpp>
#include <DragonGeo/Prim/Triangle2.hpp>

namespace DragonGeo::Prim {

/// 三维三角形：由顶点 `A`、`B`、`C` 给出。
///
/// 参数域是 `[0, 3]`。三条边各占长度 1，顺序是 `A→B`、`B→C`、`C→A`。
/// 参数中点 `1.5` 落在第二条边的中点，不是三角形重心。
/// 非有限坐标可以存入；`IsValid` 仅在九个分量都有限时为真。
/// 零面积（三点共线）仍视为有效。
template <typename Scalar>
struct Triangle3T {
    using ScalarType = Scalar;

    // 同 Point3T / Box2T：不声明任何构造函数，以保持聚合性。
    Linear::Point3T<Scalar> A{};
    Linear::Point3T<Scalar> B{};
    Linear::Point3T<Scalar> C{};

    /// 三个顶点的九个分量均为有限值时为真。零面积（三点共线）仍视为有效。
    [[nodiscard]] constexpr bool IsValid() const noexcept {
        using Core::IsFinite;
        return IsFinite(A.X) && IsFinite(A.Y) && IsFinite(A.Z) && IsFinite(B.X) && IsFinite(B.Y)
            && IsFinite(B.Z) && IsFinite(C.X) && IsFinite(C.Y) && IsFinite(C.Z);
    }

    /// 有向面积是 `(B - A) × (C - A)` 长度的一半。
    /// 符号取叉积绝对值最大的那个分量的符号；绝对值并列时按 X、Y、Z
    /// 的顺序取先出现的分量。交换两个顶点会使叉积反向，面积变号。
    /// 叉积为零时面积是 0。落在 xy 平面上时，这与二维的逆时针为正一致。
    [[nodiscard]] Scalar SignedArea() const noexcept {
        const Linear::Vector3T<Scalar> cross = (B - A).Cross(C - A);
        const Scalar length = cross.Length();
        if (length == Scalar{0}) {
            return Scalar{0};
        }
        Scalar component = cross.X;
        Scalar magnitude = Core::AbsoluteValue(cross.X);
        const Scalar absY = Core::AbsoluteValue(cross.Y);
        if (absY > magnitude) {
            component = cross.Y;
            magnitude = absY;
        }
        const Scalar absZ = Core::AbsoluteValue(cross.Z);
        if (absZ > magnitude) {
            component = cross.Z;
        }
        const Scalar sign = component < Scalar{0} ? Scalar{-1} : Scalar{1};
        return sign * (Scalar{0.5} * length);
    }

    /// 参数域 `[0, 3]`。
    [[nodiscard]] constexpr Linear::IntervalT<Scalar> Domain() const noexcept {
        return {Scalar{0}, Scalar{3}};
    }

    /// `t` 不在 `[0, 3]` 或非有限时为空。
    /// `0` 与 `3` 都是顶点 `A`。`1.5` 是第二条边的中点，在边界上，不是重心。
    [[nodiscard]] constexpr std::optional<Linear::Point3T<Scalar>> PointAt(Scalar t) const noexcept {
        if (!Detail::IsAcceptedParameter(t, Domain())) {
            return std::nullopt;
        }
        return Locate(t);
    }

    /// 参数中点，即 `PointAt(1.5)`。它在边界上，不是填充区域的重心。
    [[nodiscard]] constexpr std::optional<Linear::Point3T<Scalar>> MidPoint() const noexcept {
        return PointAt(Scalar{1.5});
    }

    /// 点在三角形内（含边界）时为真。只对 `double` 提供。
    ///
    /// 先要求 `Orient3d(A, B, C, point) == 0`。不共面就是不包含。
    /// 三点共线时，点必须落在某条三维退化边上：在 xy、yz、zx 三个投影里
    /// `Orient2d` 都为 0，且沿该边的参数落在 `[0, 1]`。零长度边只包含重合的顶点。
    /// 面积非零时，投到叉积绝对值最大的坐标平面上，丢掉对应的轴，再调用
    /// `Triangle2::Contains`。
    template <typename S = Scalar>
        requires std::same_as<Scalar, double> && std::same_as<S, double>
    [[nodiscard]] bool Contains(Linear::Point3T<S> point) const noexcept {
        if (Predicates::Orient3d(A, B, C, point) != 0) {
            return false;
        }

        const auto collinear = [](Linear::Point3 first, Linear::Point3 second,
                                  Linear::Point3 third) noexcept {
            const auto orient = [](double ax, double ay, double bx, double by, double cx,
                                   double cy) noexcept {
                return Predicates::Orient2d(
                    Linear::Point2{ax, ay}, Linear::Point2{bx, by}, Linear::Point2{cx, cy});
            };
            return orient(first.X, first.Y, second.X, second.Y, third.X, third.Y) == 0
                && orient(first.Y, first.Z, second.Y, second.Z, third.Y, third.Z) == 0
                && orient(first.Z, first.X, second.Z, second.X, third.Z, third.X) == 0;
        };
        const auto onEdge = [&](Linear::Point3 start, Linear::Point3 end,
                                Linear::Point3 query) noexcept {
            if (start == end) {
                return query == start;
            }
            if (!collinear(start, end, query)) {
                return false;
            }
            const Linear::Vector3 chord = end - start;
            const double lengthSquared = chord.LengthSquared();
            if (!(lengthSquared > 0.0)) {
                return query == start || query == end;
            }
            const double parameter = (query - start).Dot(chord) / lengthSquared;
            return parameter >= 0.0 && parameter <= 1.0;
        };
        if (collinear(A, B, C)) {
            return onEdge(A, B, point) || onEdge(B, C, point) || onEdge(C, A, point);
        }

        const int dropped = DroppedAxis();
        const Triangle2 projected{
            Project(A, dropped), Project(B, dropped), Project(C, dropped)};
        return projected.Contains(Project(point, dropped));
    }

    /// 区间必须落在 `[0, 3]` 内且长度大于 0，否则为空。端点非有限或区间倒置同样为空。
    /// 整段落在同一条边上时返回 `Segment3`，跨过顶点时返回 `Polyline3`。
    [[nodiscard]] std::optional<std::variant<Segment3T<Scalar>, Polyline3T<Scalar>>> Subcurve(
        Linear::IntervalT<Scalar> interval) const {
        using Curve = std::variant<Segment3T<Scalar>, Polyline3T<Scalar>>;
        if (!Detail::IsFiniteSubinterval(interval, Domain())) {
            return std::nullopt;
        }
        for (int edge = 0; edge < 3; ++edge) {
            const Scalar edgeMin = static_cast<Scalar>(edge);
            const Scalar edgeMax = edgeMin + Scalar{1};
            if (interval.Min >= edgeMin && interval.Max <= edgeMax) {
                return Curve{Segment3T<Scalar>{Locate(interval.Min), Locate(interval.Max)}};
            }
        }

        Linear::Point3T<Scalar> corners[4];
        int count = 0;
        corners[count++] = Locate(interval.Min);
        for (int vertex = 1; vertex <= 2; ++vertex) {
            const Scalar parameter = static_cast<Scalar>(vertex);
            if (parameter > interval.Min && parameter < interval.Max) {
                corners[count++] = Locate(parameter);
            }
        }
        corners[count++] = Locate(interval.Max);

        Segment3T<Scalar> segments[3];
        for (int index = 0; index + 1 < count; ++index) {
            segments[index] = Segment3T<Scalar>{corners[index], corners[index + 1]};
        }
        auto polyline = Polyline3T<Scalar>::FromSegments(
            std::span<const Segment3T<Scalar>>{segments, static_cast<std::size_t>(count - 1)});
        if (!polyline.has_value()) {
            return std::nullopt;
        }
        return Curve{std::move(*polyline)};
    }

private:
    /// `t` 必须已落在 `[0, 3]` 内。`3` 映射回 `A`。
    [[nodiscard]] constexpr Linear::Point3T<Scalar> Locate(Scalar t) const noexcept {
        const Linear::Point3T<Scalar> vertices[]{A, B, C, A};
        const int edge = static_cast<int>(t);
        const int index = edge >= 3 ? 2 : edge;
        const Scalar local = t - static_cast<Scalar>(index);
        return vertices[index] + (vertices[index + 1] - vertices[index]) * local;
    }

    /// 投影时丢掉的轴：0 是 X，1 是 Y，2 是 Z。只从 `double` 的 `Contains` 调用。
    [[nodiscard]] int DroppedAxis() const noexcept {
        const Linear::Vector3 cross = (B - A).Cross(C - A);
        const double absX = Core::AbsoluteValue(cross.X);
        const double absY = Core::AbsoluteValue(cross.Y);
        const double absZ = Core::AbsoluteValue(cross.Z);
        if (absX != 0.0 || absY != 0.0 || absZ != 0.0) {
            if (absX >= absY && absX >= absZ) {
                return 0;
            }
            if (absY >= absZ) {
                return 1;
            }
            return 2;
        }
        const auto extent = [](double p, double q, double r) noexcept {
            const double lo = p < q ? p : q;
            const double hi = p < q ? q : p;
            const double low = lo < r ? lo : r;
            const double high = hi > r ? hi : r;
            return high - low;
        };
        const double rangeX = extent(A.X, B.X, C.X);
        const double rangeY = extent(A.Y, B.Y, C.Y);
        const double rangeZ = extent(A.Z, B.Z, C.Z);
        if (rangeX <= rangeY && rangeX <= rangeZ) {
            return 0;
        }
        if (rangeY <= rangeZ) {
            return 1;
        }
        return 2;
    }

    [[nodiscard]] static Linear::Point2 Project(Linear::Point3 point, int dropped) noexcept {
        if (dropped == 0) {
            return Linear::Point2{point.Y, point.Z};
        }
        if (dropped == 1) {
            return Linear::Point2{point.X, point.Z};
        }
        return Linear::Point2{point.X, point.Y};
    }
};

/// 逐字段比较。`operator!=` 由 C++20 自动生成，不手写。
template <typename Scalar>
[[nodiscard]] constexpr bool operator==(Triangle3T<Scalar> a, Triangle3T<Scalar> b) noexcept {
    return a.A == b.A && a.B == b.B && a.C == b.C;
}

using Triangle3 = Triangle3T<double>;
using Triangle3f = Triangle3T<float>;

} // namespace DragonGeo::Prim
