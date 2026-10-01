#pragma once

#include <cmath>
#include <concepts>
#include <optional>
#include <span>
#include <variant>

#include <DragonGeo/Core/Numeric.hpp>
#include <DragonGeo/Core/Tolerance.hpp>
#include <DragonGeo/Detail/CurveParameter.hpp>
#include <DragonGeo/Linear/Box2.hpp>
#include <DragonGeo/Linear/Interval.hpp>
#include <DragonGeo/Linear/Point2.hpp>
#include <DragonGeo/Linear/Transform2.hpp>
#include <DragonGeo/Linear/UnitVector2.hpp>
#include <DragonGeo/Predicates/Predicates.hpp>
#include <DragonGeo/Prim/Polyline.hpp>
#include <DragonGeo/Prim/Segment2.hpp>
#include <DragonGeo/Prim/Winding.hpp>

namespace DragonGeo::Prim {

/// 二维三角形：由顶点 `A`、`B`、`C` 给出。
///
/// 参数域是 `[0, 3]`。三条边各占长度 1，顺序是 `A→B`、`B→C`、`C→A`。参数中点 `1.5` 落在第二条边的中点，不是三角形重心。非有限坐标可以存入；`IsValid` 仅在六个分量都有限时为真。零面积（三点共线）仍视为有效。
template <typename Scalar> struct Triangle2T {
    using ScalarType = Scalar;

    // 同 Point2T / Box2T：不声明任何构造函数，以保持聚合性。
    Linear::Point2T<Scalar> A{};
    Linear::Point2T<Scalar> B{};
    Linear::Point2T<Scalar> C{};

    /// 三个顶点的六个分量均为有限值时为真。零面积（三点共线）仍视为有效。
    [[nodiscard]] constexpr bool IsValid() const noexcept {
        using Core::IsFinite;
        return IsFinite(A.X) && IsFinite(A.Y) && IsFinite(B.X) && IsFinite(B.Y) && IsFinite(C.X) && IsFinite(C.Y);
    }

    /// 有向面积，逆时针为正，顺时针为负，三点共线时为 0。等于 `(B - A) × (C - A) / 2`。
    [[nodiscard]] Scalar SignedArea() const noexcept;

    /// 参数域 `[0, 3]`。
    [[nodiscard]] Linear::IntervalT<Scalar> Domain() const noexcept;

    /// `t` 不在 `[0, 3]` 或非有限时为空。 `0` 与 `3` 都是顶点 `A`。`1.5` 是第二条边的中点，在边界上，不是重心。
    [[nodiscard]] std::optional<Linear::Point2T<Scalar>> PointAt(Scalar t) const noexcept;

    /// 参数中点，即 `PointAt(1.5)`。它在边界上，不是填充区域的重心。
    [[nodiscard]] std::optional<Linear::Point2T<Scalar>> MidPoint() const noexcept;

    /// 点在三角形内（含边界）时为真。只对 `double` 提供。
    ///
    /// 面积非零时，三次 `Orient2d`（`A,B`、`B,C`、`C,A` 对查询点）全部 `>= 0` 或全部 `<= 0`。边界上至少有一次为 0，仍算内部。面积为零（`Orient2d(A,B,C) == 0`）时，点必须落在某条退化边上：
    /// 对该边 `Orient2d` 为 0，且点在边上的参数落在 `[0, 1]`。零长度边只包含与该顶点重合的点。
    template <typename S = Scalar>
        requires std::same_as<Scalar, double> && std::same_as<S, double> [[nodiscard]] bool Contains(Linear::Point2T<S> point) const noexcept;

    /// 闭合曲线的两端都是接缝上的顶点 `A`。
    [[nodiscard]] std::optional<Linear::Point2T<Scalar>> StartPoint() const noexcept;

    /// 闭合曲线的两端都是接缝上的顶点 `A`。
    [[nodiscard]] std::optional<Linear::Point2T<Scalar>> EndPoint() const noexcept;

    /// `A→B` 的单位方向。这条边没有有限正长度时为空。
    [[nodiscard]] std::optional<Linear::UnitVector2T<Scalar>> StartTangent() const noexcept;

    /// `C→A` 的单位方向。这条边没有有限正长度时为空。
    [[nodiscard]] std::optional<Linear::UnitVector2T<Scalar>> EndTangent() const noexcept;

    /// `B→C` 的单位方向。这条边没有有限正长度时为空。
    [[nodiscard]] std::optional<Linear::UnitVector2T<Scalar>> MidTangent() const noexcept;

    /// `t` 不在 `[0, 3]`、非有限，或所在边没有有限正长度时为空。 `t == 3` 用边 `C→A`。其余参数用 `floor(t)`：`0` 是 `A→B`，`1` 是 `B→C`，`2` 是 `C→A`。
    [[nodiscard]] std::optional<Linear::UnitVector2T<Scalar>> TangentAt(Scalar t) const noexcept;

    [[nodiscard]] bool IsClosed() const noexcept;

    /// 周长 `|AB| + |BC| + |CA|`，非负。
    [[nodiscard]] Scalar Length() const noexcept;

    /// 填充面积，即 `SignedArea` 的绝对值，含 0。
    [[nodiscard]] std::optional<Scalar> Area() const noexcept;

    /// 正的有符号面积是 `CounterClockwise`，负的是 `Clockwise`，恰好为 0 是 `Degenerate`。
    [[nodiscard]] std::optional<Winding> Orientation() const noexcept;

    /// 三个顶点的轴对齐包围盒。
    [[nodiscard]] Linear::Box2T<Scalar> Box() const noexcept;

    /// `(A + B + C) / 3`。`SignedArea` 为 0 时为空。
    [[nodiscard]] std::optional<Linear::Point2T<Scalar>> Centroid() const noexcept;

    /// 点到边界（三条边构成的曲线）的距离不超过 `tolerance.Resolve(尺度)`。严格落在内部、离每条边都比容差更远的点为假。尺度是包围盒对角线；对角线不是大于 0 的有限数时用 `1`。
    [[nodiscard]] bool ContainsPoint(Linear::Point2T<Scalar> point, Core::ToleranceT<Scalar> tolerance = {}) const noexcept;

    /// 边界上最近点的参数，落在 `[0, 3]`。`ContainsPoint` 为假时为空。接缝上的点返回 `0`，不返回 `3`。
    [[nodiscard]] std::optional<Scalar> ParameterOf(Linear::Point2T<Scalar> point, Core::ToleranceT<Scalar> tolerance = {}) const noexcept;

    void Translate(Linear::Vector2T<Scalar> vector) noexcept;

    /// 绕 `center` 逆时针旋转 `radians` 弧度。
    void Rotate(Linear::Point2T<Scalar> center, Scalar radians) noexcept;

    /// 关于过 `point`、法向为 `unitNormal` 的直线反射。
    void Mirror(Linear::Point2T<Scalar> point, Linear::UnitVector2T<Scalar> unitNormal) noexcept;

    /// 变成 `{A, C, B}`。接缝仍是 `A`，`Orientation` 变号。
    void Reverse() noexcept;

    [[nodiscard]] Triangle2T Clone() const noexcept;

    /// 变换三个顶点。任一结果分量非有限时返回 `false`，字段保持原样。零面积仍然成功。
    [[nodiscard]] bool Transform(const Linear::Transform2T<Scalar>& transform) noexcept;

    /// 到填充三角形（面或边界）的平方距离。内部的点是 0。
    [[nodiscard]] Scalar DistanceSquared(Linear::Point2T<Scalar> point) const noexcept;

    /// 到填充三角形的距离，即 `DistanceSquared` 的平方根。
    [[nodiscard]] Scalar Distance(Linear::Point2T<Scalar> point) const noexcept;

    /// 填充三角形上的最近点。内部的点就是查询点本身；外部取三条边上的最近点。
    [[nodiscard]] Linear::Point2T<Scalar> ClosestPoint(Linear::Point2T<Scalar> point) const noexcept;

    /// 区间必须落在 `[0, 3]` 内且长度大于 0，否则为空。端点非有限或区间倒置同样为空。整段落在同一条边上时返回 `Segment2`，跨过顶点时返回 `Polyline`。
    [[nodiscard]] std::optional<std::variant<Segment2T<Scalar>, PolylineT<Scalar>>> Subcurve(Linear::IntervalT<Scalar> interval) const;

private:
    /// `t` 必须已落在 `[0, 3]` 内。`3` 映射回 `A`。
    [[nodiscard]] Linear::Point2T<Scalar> Locate(Scalar t) const noexcept;

    struct BoundaryLocation {
        Scalar DistanceSquared{};
        Scalar Parameter{};
        Linear::Point2T<Scalar> Point{};
    };

    /// `t == 3` 落在 `C→A`。其余用截断后的边号。
    [[nodiscard]] static int EdgeIndex(Scalar t) noexcept;

    /// 边没有有限正长度，或方向无法归一化时为空。
    [[nodiscard]] static std::optional<Linear::UnitVector2T<Scalar>> UnitEdge(Linear::Point2T<Scalar> from, Linear::Point2T<Scalar> to) noexcept;

    /// 包围盒对角线。不是大于 0 的有限数时用 `1`。
    [[nodiscard]] Scalar BoundaryScale() const noexcept;

    /// 查询点的重心坐标全部 `>= 0` 时，它落在填充三角形内（含边界）。
    [[nodiscard]] bool ProjectsInside(Linear::Point2T<Scalar> point) const noexcept;

    /// 三条边上的最近点。距离相同取较小的参数。参数 `3` 折回接缝 `0`。
    [[nodiscard]] BoundaryLocation ClosestBoundary(Linear::Point2T<Scalar> point) const noexcept;

    [[nodiscard]] static Scalar BoundaryParameter(
        int edge, Linear::Point2T<Scalar> from, Linear::Point2T<Scalar> to, Linear::Point2T<Scalar> closest) noexcept;
};

/// 逐字段比较。`operator!=` 由 C++20 自动生成，不手写。
template <typename Scalar> [[nodiscard]] constexpr bool operator==(Triangle2T<Scalar> a, Triangle2T<Scalar> b) noexcept {
    return a.A == b.A && a.B == b.B && a.C == b.C;
}

using Triangle2 = Triangle2T<double>;
using Triangle2f = Triangle2T<float>;


extern template struct Triangle2T<double>;
extern template struct Triangle2T<float>;
} // namespace DragonGeo::Prim
