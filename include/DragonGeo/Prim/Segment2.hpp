#pragma once

#include <cmath>
#include <limits>
#include <optional>

#include <DragonGeo/Core/Numeric.hpp>
#include <DragonGeo/Core/Tolerance.hpp>
#include <DragonGeo/Detail/CurveParameter.hpp>
#include <DragonGeo/Linear/Box2.hpp>
#include <DragonGeo/Linear/Point2.hpp>
#include <DragonGeo/Linear/Transform2.hpp>
#include <DragonGeo/Linear/UnitVector2.hpp>

namespace DragonGeo::Prim {

template <typename Scalar>
struct Ray2T;
template <typename Scalar>
struct Line2T;

/// 二维线段：由两个端点 `A`、`B` 给出的有限曲线段。
///
/// 参数域是 `[0, 1]`，`PointAt(t) = A + t (B - A)`。`t` 不在域内或非有限时为空。
/// 零长度（`A == B`）仍然有效：长度为 0，`Direction` 为空，最近点参数是 0。
/// 非有限坐标可以存入；`IsValid` 仅在四个分量都有限时为真。
template <typename Scalar>
struct Segment2T {
    using ScalarType = Scalar;

    // 同 Point2T / Box2T：不声明任何构造函数，以保持聚合性。
    Linear::Point2T<Scalar> A{};
    Linear::Point2T<Scalar> B{};

    /// 两个端点的四个分量均为有限值时为真。零长度（`A == B`）仍视为有效。
    [[nodiscard]] constexpr bool IsValid() const noexcept {
        using Core::IsFinite;
        return IsFinite(A.X) && IsFinite(A.Y) && IsFinite(B.X) && IsFinite(B.Y);
    }

    /// 参数域 `[0, 1]`。
    [[nodiscard]] constexpr Linear::IntervalT<Scalar> Domain() const noexcept {
        return {Scalar{0}, Scalar{1}};
    }

    /// `t` 不在 `[0, 1]` 或非有限时为空。
    [[nodiscard]] constexpr std::optional<Linear::Point2T<Scalar>> PointAt(Scalar t) const noexcept {
        if (!Detail::IsAcceptedParameter(t, Domain())) {
            return std::nullopt;
        }
        return Locate(t);
    }

    /// 最近点把参数夹在 `[0, 1]`。零长度线段的最近点就是 `A`。
    [[nodiscard]] Linear::Point2T<Scalar> ClosestPoint(Linear::Point2T<Scalar> point) const noexcept {
        return Locate(ClosestParameter(point));
    }

    /// 点到线段的平方距离。点在线段上时为 0。
    [[nodiscard]] Scalar DistanceSquared(Linear::Point2T<Scalar> point) const noexcept {
        return (point - ClosestPoint(point)).LengthSquared();
    }

    /// 点到线段的距离，即 `DistanceSquared` 的平方根。
    [[nodiscard]] Scalar Distance(Linear::Point2T<Scalar> point) const noexcept {
        return std::sqrt(DistanceSquared(point));
    }

    /// 两端点距离。`A == B` 时为 0。
    [[nodiscard]] Scalar Length() const noexcept {
        return A.DistanceTo(B);
    }

    /// 两端点距离的平方。`A == B` 时为 0。
    [[nodiscard]] constexpr Scalar LengthSquared() const noexcept {
        return (B - A).LengthSquared();
    }

    /// 从 `A` 指向 `B` 的单位方向。`A == B`，或方向无法归一化时为空。
    [[nodiscard]] std::optional<Linear::UnitVector2T<Scalar>> Direction() const noexcept {
        if (A == B) {
            return std::nullopt;
        }
        return (B - A).Normalized();
    }

    /// 两端点的轴对齐包围盒。
    [[nodiscard]] constexpr Linear::Box2T<Scalar> Bounds() const noexcept {
        return Linear::Box2T<Scalar>::FromCorners(A, B);
    }

    [[nodiscard]] constexpr std::optional<Linear::Point2T<Scalar>> StartPoint() const noexcept {
        return A;
    }

    [[nodiscard]] constexpr std::optional<Linear::Point2T<Scalar>> EndPoint() const noexcept {
        return B;
    }

    /// 参数中点，即 `PointAt(0.5)`，不是弧长以外的另一种中点。
    [[nodiscard]] constexpr std::optional<Linear::Point2T<Scalar>> MidPoint() const noexcept {
        return PointAt(Scalar{0.5});
    }

    [[nodiscard]] std::optional<Linear::UnitVector2T<Scalar>> StartTangent() const noexcept {
        return Direction();
    }

    [[nodiscard]] std::optional<Linear::UnitVector2T<Scalar>> EndTangent() const noexcept {
        return Direction();
    }

    [[nodiscard]] std::optional<Linear::UnitVector2T<Scalar>> MidTangent() const noexcept {
        return Direction();
    }

    /// 域内且方向非零时返回单位方向。越界、非有限参数，或零长度线段为空。
    [[nodiscard]] std::optional<Linear::UnitVector2T<Scalar>> TangentAt(Scalar t) const noexcept {
        if (!Detail::IsAcceptedParameter(t, Domain())) {
            return std::nullopt;
        }
        return Direction();
    }

    [[nodiscard]] constexpr bool IsClosed() const noexcept {
        return false;
    }

    [[nodiscard]] constexpr std::optional<Scalar> Area() const noexcept {
        return std::nullopt;
    }

    [[nodiscard]] constexpr std::optional<int> Orientation() const noexcept {
        return std::nullopt;
    }

    [[nodiscard]] constexpr std::optional<Linear::Point2T<Scalar>> Centroid() const noexcept {
        return std::nullopt;
    }

    /// 线段不填充区域。
    [[nodiscard]] constexpr bool Contains(Linear::Point2T<Scalar>) const noexcept {
        return false;
    }

    /// 点到线段（不是到所在无限直线）的距离不超过 `tolerance.Resolve(尺度)`。
    /// 尺度是线段长度；长度为 0 或非有限时用 `1`。
    [[nodiscard]] bool ContainsPoint(
        Linear::Point2T<Scalar> point,
        Core::ToleranceT<Scalar> tolerance = {}) const noexcept {
        const Scalar length = Length();
        const Scalar scale = length > Scalar{0} && Core::IsFinite(length) ? length : Scalar{1};
        return Distance(point) <= tolerance.Resolve(scale);
    }

    /// 最近点的参数。距离不满足同一容差下的 `ContainsPoint` 时为空。
    /// 零长度线段在点落在端点上时返回 `0`。
    [[nodiscard]] std::optional<Scalar> ParameterOf(
        Linear::Point2T<Scalar> point,
        Core::ToleranceT<Scalar> tolerance = {}) const noexcept {
        if (!ContainsPoint(point, tolerance)) {
            return std::nullopt;
        }
        return ClosestParameter(point);
    }

    [[nodiscard]] constexpr Segment2T Translated(Linear::Vector2T<Scalar> vector) const noexcept {
        return {A + vector, B + vector};
    }

    /// 绕 `center` 逆时针旋转 `radians` 弧度。
    [[nodiscard]] Segment2T Rotated(Linear::Point2T<Scalar> center, Scalar radians) const noexcept {
        const Linear::Transform2T<Scalar> rotation =
            Linear::Transform2T<Scalar>::RotationAbout(center, radians);
        return {rotation.TransformPoint(A), rotation.TransformPoint(B)};
    }

    /// 关于过 `point`、法向为 `unitNormal` 的直线反射。
    [[nodiscard]] constexpr Segment2T Mirrored(
        Linear::Point2T<Scalar> point, Linear::UnitVector2T<Scalar> unitNormal) const noexcept {
        const Linear::Transform2T<Scalar> mirror =
            Linear::Transform2T<Scalar>::Reflection(point, unitNormal);
        return {mirror.TransformPoint(A), mirror.TransformPoint(B)};
    }

    /// 交换两端，参数方向相反。
    [[nodiscard]] constexpr Segment2T Reversed() const noexcept {
        return {B, A};
    }

    [[nodiscard]] constexpr Segment2T Clone() const noexcept {
        return *this;
    }

    /// 变换两端点。任一结果分量非有限时为空。
    /// 原本能归一化的方向在变换后不能归一化时也为空；零长度线段没有方向，
    /// 端点有限时仍返回线段。
    [[nodiscard]] std::optional<Segment2T> Transformed(
        const Linear::Transform2T<Scalar>& transform) const noexcept {
        const Linear::Point2T<Scalar> movedA = transform.TransformPoint(A);
        const Linear::Point2T<Scalar> movedB = transform.TransformPoint(B);
        if (!Detail::CoordinatesAreFinite(movedA) || !Detail::CoordinatesAreFinite(movedB)) {
            return std::nullopt;
        }
        const Linear::Vector2T<Scalar> chord = B - A;
        const Linear::Vector2T<Scalar> transformedChord = transform * chord;
        if (chord.Normalized().has_value()
            && (!Detail::CoordinatesAreFinite(transformedChord)
                || !transformedChord.Normalized().has_value())) {
            return std::nullopt;
        }
        return Segment2T{movedA, movedB};
    }

    /// 区间必须落在 `[0, 1]` 内且长度大于 0，否则为空。
    [[nodiscard]] constexpr std::optional<Segment2T> Subcurve(
        Linear::IntervalT<Scalar> interval) const noexcept {
        if (!Detail::IsFiniteSubinterval(interval, Domain())) {
            return std::nullopt;
        }
        return Segment2T{Locate(interval.Min), Locate(interval.Max)};
    }

    /// 起点为 `A`，方向从 `A` 指向 `B`。`A == B`，或方向无法归一化时为空。
    [[nodiscard]] std::optional<Ray2T<Scalar>> AsRay() const noexcept;

    /// 起点为 `A`，方向从 `A` 指向 `B`。`A == B`，或方向无法归一化时为空。
    [[nodiscard]] std::optional<Line2T<Scalar>> AsLine() const noexcept;

private:
    [[nodiscard]] constexpr Linear::Point2T<Scalar> Locate(Scalar t) const noexcept {
        return A + (B - A) * t;
    }

    /// 零长度，或方向长度不是有限正数时返回 0。否则把投影参数夹进 `[0, 1]`。
    [[nodiscard]] Scalar ClosestParameter(Linear::Point2T<Scalar> point) const noexcept {
        const auto direction = Direction();
        if (!direction.has_value()) {
            return Scalar{0};
        }
        const Scalar length = Length();
        if (!(length > Scalar{0}) || !Core::IsFinite(length)) {
            return Scalar{0};
        }
        return Detail::ClampParameter(
            Detail::ProjectParameter(A, *direction, point) / length, Domain());
    }
};

/// 逐字段比较。`operator!=` 由 C++20 自动生成，不手写。
template <typename Scalar>
[[nodiscard]] constexpr bool operator==(Segment2T<Scalar> a, Segment2T<Scalar> b) noexcept {
    return a.A == b.A && a.B == b.B;
}

using Segment2 = Segment2T<double>;
using Segment2f = Segment2T<float>;

} // namespace DragonGeo::Prim
