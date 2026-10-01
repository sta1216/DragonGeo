#pragma once

#include <cmath>
#include <concepts>
#include <optional>
#include <span>
#include <utility>
#include <variant>

#include <DragonGeo/Core/Numeric.hpp>
#include <DragonGeo/Core/Tolerance.hpp>
#include <DragonGeo/Detail/CurveParameter.hpp>
#include <DragonGeo/Linear/Box3.hpp>
#include <DragonGeo/Linear/Interval.hpp>
#include <DragonGeo/Linear/Point3.hpp>
#include <DragonGeo/Linear/Transform3.hpp>
#include <DragonGeo/Linear/UnitVector3.hpp>
#include <DragonGeo/Linear/Vector3.hpp>
#include <DragonGeo/Predicates/Predicates.hpp>
#include <DragonGeo/Prim/Polyline3.hpp>
#include <DragonGeo/Prim/Segment3.hpp>
#include <DragonGeo/Prim/Triangle2.hpp>
#include <DragonGeo/Prim/Winding.hpp>

namespace DragonGeo::Prim {

/// 三维三角形：由顶点 `A`、`B`、`C` 给出。
///
/// 参数域是 `[0, 3]`。三条边各占长度 1，顺序是 `A→B`、`B→C`、`C→A`。参数中点 `1.5` 落在第二条边的中点，不是三角形重心。非有限坐标可以存入；`IsValid` 仅在九个分量都有限时为真。零面积（三点共线）仍视为有效。
template <typename Scalar> struct Triangle3T {
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

    /// 有向面积是 `(B - A) × (C - A)` 长度的一半。符号取叉积绝对值最大的那个分量的符号；绝对值并列时按 X、Y、Z 的顺序取先出现的分量。交换两个顶点会使叉积反向，面积变号。叉积为零时面积是 0。落在 xy 平面上时，这与二维的逆时针为正一致。
    [[nodiscard]] Scalar SignedArea() const noexcept;

    /// 参数域 `[0, 3]`。
    [[nodiscard]] Linear::IntervalT<Scalar> Domain() const noexcept;

    /// `t` 不在 `[0, 3]` 或非有限时为空。 `0` 与 `3` 都是顶点 `A`。`1.5` 是第二条边的中点，在边界上，不是重心。
    [[nodiscard]] std::optional<Linear::Point3T<Scalar>> PointAt(Scalar t) const noexcept;

    /// 参数中点，即 `PointAt(1.5)`。它在边界上，不是填充区域的重心。
    [[nodiscard]] std::optional<Linear::Point3T<Scalar>> MidPoint() const noexcept;

    /// 点在三角形内（含边界）时为真。只对 `double` 提供。
    ///
    /// 先要求 `Orient3d(A, B, C, point) == 0`。不共面就是不包含。三点共线时，点必须落在某条三维退化边上：在 xy、yz、zx 三个投影里 `Orient2d` 都为 0，且沿该边的参数落在 `[0, 1]`。零长度边只包含重合的顶点。
    /// 面积非零时，投到叉积绝对值最大的坐标平面上，丢掉对应的轴，再调用 `Triangle2::Contains`。
    template <typename S = Scalar>
        requires std::same_as<Scalar, double> && std::same_as<S, double> [[nodiscard]] bool Contains(Linear::Point3T<S> point) const noexcept;

    /// 闭合曲线的两端都是接缝上的顶点 `A`。
    [[nodiscard]] std::optional<Linear::Point3T<Scalar>> StartPoint() const noexcept;

    /// 闭合曲线的两端都是接缝上的顶点 `A`。
    [[nodiscard]] std::optional<Linear::Point3T<Scalar>> EndPoint() const noexcept;

    /// `A→B` 的单位方向。这条边没有有限正长度时为空。
    [[nodiscard]] std::optional<Linear::UnitVector3T<Scalar>> StartTangent() const noexcept;

    /// `C→A` 的单位方向。这条边没有有限正长度时为空。
    [[nodiscard]] std::optional<Linear::UnitVector3T<Scalar>> EndTangent() const noexcept;

    /// `B→C` 的单位方向。这条边没有有限正长度时为空。
    [[nodiscard]] std::optional<Linear::UnitVector3T<Scalar>> MidTangent() const noexcept;

    /// `t` 不在 `[0, 3]`、非有限，或所在边没有有限正长度时为空。 `t == 3` 用边 `C→A`。其余参数用 `floor(t)`：`0` 是 `A→B`，`1` 是 `B→C`，`2` 是 `C→A`。
    [[nodiscard]] std::optional<Linear::UnitVector3T<Scalar>> TangentAt(Scalar t) const noexcept;

    [[nodiscard]] bool IsClosed() const noexcept;

    /// 周长 `|AB| + |BC| + |CA|`，非负。
    [[nodiscard]] Scalar Length() const noexcept;

    /// 填充面积，即 `SignedArea` 的绝对值，含 0。
    [[nodiscard]] std::optional<Scalar> Area() const noexcept;

    /// 正的有符号面积是 `CounterClockwise`，负的是 `Clockwise`，恰好为 0 是 `Degenerate`。符号与 `SignedArea` 相同：落在 xy 平面上时逆时针为正。
    [[nodiscard]] std::optional<Winding> Orientation() const noexcept;

    /// 三个顶点的轴对齐包围盒。
    [[nodiscard]] Linear::Box3T<Scalar> Box() const noexcept;

    /// `(A + B + C) / 3`。`SignedArea` 为 0 时为空。
    [[nodiscard]] std::optional<Linear::Point3T<Scalar>> Centroid() const noexcept;

    /// 点到边界（三条边构成的曲线）的距离不超过 `tolerance.Resolve(尺度)`。严格落在内部、离每条边都比容差更远的点为假。尺度是包围盒对角线；对角线不是大于 0 的有限数时用 `1`。
    [[nodiscard]] bool ContainsPoint(Linear::Point3T<Scalar> point, Core::ToleranceT<Scalar> tolerance = {}) const noexcept;

    /// 边界上最近点的参数，落在 `[0, 3]`。`ContainsPoint` 为假时为空。接缝上的点返回 `0`，不返回 `3`。
    [[nodiscard]] std::optional<Scalar> ParameterOf(Linear::Point3T<Scalar> point, Core::ToleranceT<Scalar> tolerance = {}) const noexcept;

    void Translate(Linear::Vector3T<Scalar> vector) noexcept;

    /// 绕过 `origin`、方向为 `axis` 的轴旋转 `radians` 弧度。
    void Rotate(Linear::Point3T<Scalar> origin, Linear::UnitVector3T<Scalar> axis, Scalar radians) noexcept;

    /// 关于过 `point`、法向为 `unitNormal` 的平面反射。
    void Mirror(Linear::Point3T<Scalar> point, Linear::UnitVector3T<Scalar> unitNormal) noexcept;

    /// 变成 `{A, C, B}`。接缝仍是 `A`，`Orientation` 变号。
    void Reverse() noexcept;

    [[nodiscard]] Triangle3T Clone() const noexcept;

    /// 变换三个顶点。任一结果分量非有限时返回 `false`，字段保持原样。零面积仍然成功。
    [[nodiscard]] bool Transform(const Linear::Transform3T<Scalar>& transform) noexcept;

    /// 到填充三角形（面或边界）的平方距离。面上的点是它到平面的距离的平方。
    [[nodiscard]] Scalar DistanceSquared(Linear::Point3T<Scalar> point) const noexcept;

    /// 到填充三角形的距离，即 `DistanceSquared` 的平方根。
    [[nodiscard]] Scalar Distance(Linear::Point3T<Scalar> point) const noexcept;

    /// 填充三角形上的最近点。投影落在面内时，结果是 `A`、`B`、`C` 的仿射组合；否则取三条边上的最近点。
    [[nodiscard]] Linear::Point3T<Scalar> ClosestPoint(Linear::Point3T<Scalar> point) const noexcept;

    /// 区间必须落在 `[0, 3]` 内且长度大于 0，否则为空。端点非有限或区间倒置同样为空。整段落在同一条边上时返回 `Segment3`，跨过顶点时返回 `Polyline3`。
    [[nodiscard]] std::optional<std::variant<Segment3T<Scalar>, Polyline3T<Scalar>>> Subcurve(Linear::IntervalT<Scalar> interval) const;

private:
    /// `t` 必须已落在 `[0, 3]` 内。`3` 映射回 `A`。
    [[nodiscard]] Linear::Point3T<Scalar> Locate(Scalar t) const noexcept;

    struct BoundaryLocation {
        Scalar DistanceSquared{};
        Scalar Parameter{};
        Linear::Point3T<Scalar> Point{};
    };

    /// `t == 3` 落在 `C→A`。其余用截断后的边号。
    [[nodiscard]] static int EdgeIndex(Scalar t) noexcept;

    /// 边没有有限正长度，或方向无法归一化时为空。
    [[nodiscard]] static std::optional<Linear::UnitVector3T<Scalar>> UnitEdge(Linear::Point3T<Scalar> from, Linear::Point3T<Scalar> to) noexcept;

    /// 包围盒对角线。不是大于 0 的有限数时用 `1`。
    [[nodiscard]] Scalar BoundaryScale() const noexcept;

    /// 投影落在面内（含边界）时，返回 `A`、`B`、`C` 的仿射组合。不把除法重建的点再交给 `Contains`：那次除法会让点离开平面。
    [[nodiscard]] std::optional<Linear::Point3T<Scalar>> FacePoint(Linear::Point3T<Scalar> point) const noexcept;

    /// 三条边上的最近点。距离相同取较小的参数。参数 `3` 折回接缝 `0`。
    [[nodiscard]] BoundaryLocation ClosestBoundary(Linear::Point3T<Scalar> point) const noexcept;

    [[nodiscard]] static Scalar BoundaryParameter(
        int edge, Linear::Point3T<Scalar> from, Linear::Point3T<Scalar> to, Linear::Point3T<Scalar> closest) noexcept;

    /// 投影时丢掉的轴：0 是 X，1 是 Y，2 是 Z。只从 `double` 的 `Contains` 调用。
    [[nodiscard]] int DroppedAxis() const noexcept;

    [[nodiscard]] static Linear::Point2 Project(Linear::Point3 point, int dropped) noexcept;
};

/// 逐字段比较。`operator!=` 由 C++20 自动生成，不手写。
template <typename Scalar> [[nodiscard]] constexpr bool operator==(Triangle3T<Scalar> a, Triangle3T<Scalar> b) noexcept {
    return a.A == b.A && a.B == b.B && a.C == b.C;
}

using Triangle3 = Triangle3T<double>;
using Triangle3f = Triangle3T<float>;


extern template struct Triangle3T<double>;
extern template struct Triangle3T<float>;
} // namespace DragonGeo::Prim
