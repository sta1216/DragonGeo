#pragma once

#include <concepts>
#include <optional>

#include <DragonGeo/Core/Numeric.hpp>
#include <DragonGeo/Detail/CurveParameter.hpp>
#include <DragonGeo/Linear/Interval.hpp>
#include <DragonGeo/Linear/Point2.hpp>
#include <DragonGeo/Predicates/Predicates.hpp>
#include <DragonGeo/Prim/Segment2.hpp>

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
};

/// 逐字段比较。`operator!=` 由 C++20 自动生成，不手写。
template <typename Scalar>
[[nodiscard]] constexpr bool operator==(Triangle2T<Scalar> a, Triangle2T<Scalar> b) noexcept {
    return a.A == b.A && a.B == b.B && a.C == b.C;
}

using Triangle2 = Triangle2T<double>;
using Triangle2f = Triangle2T<float>;

} // namespace DragonGeo::Prim
