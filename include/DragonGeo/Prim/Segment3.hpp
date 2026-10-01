#pragma once

#include <cmath>
#include <optional>

#include <DragonGeo/Core/Numeric.hpp>
#include <DragonGeo/Core/Tolerance.hpp>
#include <DragonGeo/Detail/CurveParameter.hpp>
#include <DragonGeo/Linear/Box3.hpp>
#include <DragonGeo/Linear/Point3.hpp>
#include <DragonGeo/Linear/Transform3.hpp>
#include <DragonGeo/Linear/UnitVector3.hpp>
#include <DragonGeo/Prim/Winding.hpp>

namespace DragonGeo::Prim {

template <typename Scalar>
struct Ray3T;
template <typename Scalar>
struct Line3T;

/// 三维线段：由两个端点 `A`、`B` 给出的有限曲线段。
///
/// 参数域是 `[0, 1]`，`PointAt(t) = A + t (B - A)`。`t` 不在域内或非有限时为空。
/// 零长度（`A == B`）仍然有效：长度为 0，`Direction` 为空，最近点参数是 0。
/// 非有限坐标可以存入；`IsValid` 仅在六个分量都有限时为真。
template <typename Scalar>
struct Segment3T {
    using ScalarType = Scalar;

    // 同 Point3T / Box2T：不声明任何构造函数，以保持聚合性。
    Linear::Point3T<Scalar> A{};
    Linear::Point3T<Scalar> B{};

    /// 两个端点的六个分量均为有限值时为真。零长度（`A == B`）仍视为有效。
    [[nodiscard]] constexpr bool IsValid() const noexcept {
        using Core::IsFinite;
        return IsFinite(A.X) && IsFinite(A.Y) && IsFinite(A.Z) && IsFinite(B.X) && IsFinite(B.Y)
            && IsFinite(B.Z);
    }

    /// 参数域 `[0, 1]`。
    [[nodiscard]] Linear::IntervalT<Scalar> Domain() const noexcept;

    /// `t` 不在 `[0, 1]` 或非有限时为空。
    [[nodiscard]] std::optional<Linear::Point3T<Scalar>> PointAt(Scalar t) const noexcept;

    /// 最近点把参数夹在 `[0, 1]`。零长度线段的最近点就是 `A`。
    [[nodiscard]] Linear::Point3T<Scalar> ClosestPoint(Linear::Point3T<Scalar> point) const noexcept;

    /// 点到线段的平方距离。点在线段上时为 0。
    [[nodiscard]] Scalar DistanceSquared(Linear::Point3T<Scalar> point) const noexcept;

    /// 点到线段的距离，即 `DistanceSquared` 的平方根。
    [[nodiscard]] Scalar Distance(Linear::Point3T<Scalar> point) const noexcept;

    /// 两端点距离。`A == B` 时为 0。
    [[nodiscard]] Scalar Length() const noexcept;

    /// 两端点距离的平方。`A == B` 时为 0。
    [[nodiscard]] Scalar LengthSquared() const noexcept;

    /// 从 `A` 指向 `B` 的单位方向。`A == B`，或方向无法归一化时为空。
    [[nodiscard]] std::optional<Linear::UnitVector3T<Scalar>> Direction() const noexcept;

    /// 两端点的轴对齐包围盒。
    [[nodiscard]] Linear::Box3T<Scalar> Box() const noexcept;

    [[nodiscard]] std::optional<Linear::Point3T<Scalar>> StartPoint() const noexcept;

    [[nodiscard]] std::optional<Linear::Point3T<Scalar>> EndPoint() const noexcept;

    /// 参数中点，即 `PointAt(0.5)`。
    [[nodiscard]] std::optional<Linear::Point3T<Scalar>> MidPoint() const noexcept;

    [[nodiscard]] std::optional<Linear::UnitVector3T<Scalar>> StartTangent() const noexcept;

    [[nodiscard]] std::optional<Linear::UnitVector3T<Scalar>> EndTangent() const noexcept;

    [[nodiscard]] std::optional<Linear::UnitVector3T<Scalar>> MidTangent() const noexcept;

    /// 域内且方向非零时返回单位方向。越界、非有限参数，或零长度线段为空。
    [[nodiscard]] std::optional<Linear::UnitVector3T<Scalar>> TangentAt(Scalar t) const noexcept;

    [[nodiscard]] bool IsClosed() const noexcept;

    [[nodiscard]] std::optional<Scalar> Area() const noexcept;

    [[nodiscard]] std::optional<Winding> Orientation() const noexcept;

    [[nodiscard]] std::optional<Linear::Point3T<Scalar>> Centroid() const noexcept;

    /// 线段不填充区域。
    [[nodiscard]] bool Contains(Linear::Point3T<Scalar>) const noexcept;

    /// 点到所在直线的距离不超过 `tolerance.Resolve(尺度)`，并且投影参数落在
    /// `[-e, 1 + e]` 内，其中 `e = tolerance.Resolve(1)`。
    /// 尺度是包围盒对角线，也就是线段长度。零长度线段的对角线是 0，
    /// 容差因此是绝对项 `Resolve(0)`，距离是到 `A` 的距离。
    [[nodiscard]] bool ContainsPoint(
        Linear::Point3T<Scalar> point,
        Core::ToleranceT<Scalar> tolerance = {}) const noexcept;

    /// 未夹紧的直线投影参数。同一容差下 `ContainsPoint` 为假时为空。
    /// 成功时可以略微落在 `[0, 1]` 之外。零长度线段返回 `0`。
    [[nodiscard]] std::optional<Scalar> ParameterOf(
        Linear::Point3T<Scalar> point,
        Core::ToleranceT<Scalar> tolerance = {}) const noexcept;

    void Translate(Linear::Vector3T<Scalar> vector) noexcept;

    /// 绕过 `origin`、方向为 `axis` 的轴旋转 `radians` 弧度。
    void Rotate(
        Linear::Point3T<Scalar> origin,
        Linear::UnitVector3T<Scalar> axis,
        Scalar radians) noexcept;

    /// 关于过 `point`、法向为 `unitNormal` 的平面反射。
    void Mirror(
        Linear::Point3T<Scalar> point, Linear::UnitVector3T<Scalar> unitNormal) noexcept;

    /// 交换两端，参数方向相反。
    void Reverse() noexcept;

    [[nodiscard]] Segment3T Clone() const noexcept;

    /// 变换两端点。任一结果分量非有限时返回 `false`，字段保持原样。
    /// 原本能归一化的方向在变换后不能归一化时同样失败；零长度线段没有方向，
    /// 端点有限时写入两端并返回 `true`。
    [[nodiscard]] bool Transform(const Linear::Transform3T<Scalar>& transform) noexcept;

    /// 区间必须落在 `[0, 1]` 内且长度大于 0，否则为空。
    [[nodiscard]] std::optional<Segment3T> Subcurve(
        Linear::IntervalT<Scalar> interval) const noexcept;

    /// 起点为 `A`，方向从 `A` 指向 `B`。`A == B`，或方向无法归一化时为空。
    [[nodiscard]] std::optional<Ray3T<Scalar>> AsRay() const noexcept;

    /// 起点为 `A`，方向从 `A` 指向 `B`。`A == B`，或方向无法归一化时为空。
    [[nodiscard]] std::optional<Line3T<Scalar>> AsLine() const noexcept;

private:
    [[nodiscard]] Linear::Point3T<Scalar> Locate(Scalar t) const noexcept;

    /// 直线上的未夹紧参数。零长度，或方向长度不是有限正数时返回 0。
    [[nodiscard]] Scalar SupportingParameter(Linear::Point3T<Scalar> point) const noexcept;

    /// 点到所在无限直线的距离。零长度时是到 `A` 的距离。
    [[nodiscard]] Scalar DistanceToSupportingLine(Linear::Point3T<Scalar> point) const noexcept;

    /// 零长度，或方向长度不是有限正数时返回 0。否则把投影参数夹进 `[0, 1]`。
    [[nodiscard]] Scalar ClosestParameter(Linear::Point3T<Scalar> point) const noexcept;
};

/// 逐字段比较。`operator!=` 由 C++20 自动生成，不手写。
template <typename Scalar>
[[nodiscard]] constexpr bool operator==(Segment3T<Scalar> a, Segment3T<Scalar> b) noexcept {
    return a.A == b.A && a.B == b.B;
}

using Segment3 = Segment3T<double>;
using Segment3f = Segment3T<float>;


extern template struct Segment3T<double>;
extern template struct Segment3T<float>;
} // namespace DragonGeo::Prim

#include <DragonGeo/Prim/Ray3.hpp>
#include <DragonGeo/Prim/Line3.hpp>
