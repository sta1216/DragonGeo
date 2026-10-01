#pragma once

#include <cmath>
#include <limits>
#include <optional>

#include <DragonGeo/Core/Numeric.hpp>
#include <DragonGeo/Core/Tolerance.hpp>
#include <DragonGeo/Detail/CurveParameter.hpp>
#include <DragonGeo/Linear/Box3.hpp>
#include <DragonGeo/Linear/Point3.hpp>
#include <DragonGeo/Linear/Transform3.hpp>
#include <DragonGeo/Linear/UnitVector3.hpp>
#include <DragonGeo/Prim/Segment3.hpp>

namespace DragonGeo::Prim {

/// 三维射线：从 `Origin` 沿 `Direction` 伸出的半直线。
///
/// 参数域是 `[0, +inf)`，`PointAt(t) = Origin + t Direction`。`t` 是沿方向的
/// 有符号距离；负参数与非有限参数为空。没有终点和中点。长度是 `+inf`，
/// 包围盒是规范空盒。
/// 非有限坐标可以存入；`IsValid` 只检查分量是否有限，不检查方向是否已归一化。
template <typename Scalar>
struct Ray3T {
    using ScalarType = Scalar;

    // 同 Point3T / Box2T：不声明任何构造函数，以保持聚合性。
    // 单位向量没有默认构造函数，故 Direction 不写默认成员初始化器。
    Linear::Point3T<Scalar> Origin{};
    Linear::UnitVector3T<Scalar> Direction;

    /// 原点与方向的每个分量均为有限值时为真。
    [[nodiscard]] constexpr bool IsValid() const noexcept {
        using Core::IsFinite;
        return IsFinite(Origin.X) && IsFinite(Origin.Y) && IsFinite(Origin.Z)
            && IsFinite(Direction.X()) && IsFinite(Direction.Y()) && IsFinite(Direction.Z());
    }

    /// 参数域 `[0, +inf)`。
    [[nodiscard]] constexpr Linear::IntervalT<Scalar> Domain() const noexcept {
        return {Scalar{0}, std::numeric_limits<Scalar>::infinity()};
    }

    /// `t < 0` 或 `t` 非有限时为空。
    [[nodiscard]] constexpr std::optional<Linear::Point3T<Scalar>> PointAt(Scalar t) const noexcept {
        if (!Detail::IsAcceptedParameter(t, Domain())) {
            return std::nullopt;
        }
        return Locate(t);
    }

    /// 最近点把参数夹在 `t >= 0`。查询点落在起点后方时，最近点是原点。
    [[nodiscard]] constexpr Linear::Point3T<Scalar> ClosestPoint(
        Linear::Point3T<Scalar> point) const noexcept {
        return Locate(ClosestParameter(point));
    }

    /// 点到射线的平方距离。
    [[nodiscard]] constexpr Scalar DistanceSquared(Linear::Point3T<Scalar> point) const noexcept {
        return (point - ClosestPoint(point)).LengthSquared();
    }

    /// 点到射线的距离，即 `DistanceSquared` 的平方根。
    [[nodiscard]] Scalar Distance(Linear::Point3T<Scalar> point) const noexcept {
        return std::sqrt(DistanceSquared(point));
    }

    /// 射线无界，长度为 `+inf`。
    [[nodiscard]] constexpr Scalar Length() const noexcept {
        return std::numeric_limits<Scalar>::infinity();
    }

    /// 射线没有有限包围盒，返回规范空盒。
    [[nodiscard]] constexpr Linear::Box3T<Scalar> Bounds() const noexcept {
        return Linear::Box3T<Scalar>::Empty();
    }

    [[nodiscard]] constexpr std::optional<Linear::Point3T<Scalar>> StartPoint() const noexcept {
        return Origin;
    }

    [[nodiscard]] constexpr std::optional<Linear::Point3T<Scalar>> EndPoint() const noexcept {
        return std::nullopt;
    }

    [[nodiscard]] constexpr std::optional<Linear::Point3T<Scalar>> MidPoint() const noexcept {
        return std::nullopt;
    }

    [[nodiscard]] constexpr std::optional<Linear::UnitVector3T<Scalar>> StartTangent() const noexcept {
        return Direction;
    }

    [[nodiscard]] constexpr std::optional<Linear::UnitVector3T<Scalar>> EndTangent() const noexcept {
        return std::nullopt;
    }

    [[nodiscard]] constexpr std::optional<Linear::UnitVector3T<Scalar>> MidTangent() const noexcept {
        return std::nullopt;
    }

    /// 域内返回存放的方向。负参数与非有限参数为空。
    [[nodiscard]] constexpr std::optional<Linear::UnitVector3T<Scalar>> TangentAt(
        Scalar t) const noexcept {
        if (!Detail::IsAcceptedParameter(t, Domain())) {
            return std::nullopt;
        }
        return Direction;
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

    [[nodiscard]] constexpr std::optional<Linear::Point3T<Scalar>> Centroid() const noexcept {
        return std::nullopt;
    }

    /// 射线不填充区域。
    [[nodiscard]] constexpr bool Contains(Linear::Point3T<Scalar>) const noexcept {
        return false;
    }

    /// 点到射线的距离不超过 `tolerance.Resolve(1)`。尺度固定为 `1`。
    [[nodiscard]] bool ContainsPoint(
        Linear::Point3T<Scalar> point,
        Core::ToleranceT<Scalar> tolerance = {}) const noexcept {
        return Distance(point) <= tolerance.Resolve(Scalar{1});
    }

    /// 最近点的参数。距离不满足同一容差下的 `ContainsPoint` 时为空。
    [[nodiscard]] std::optional<Scalar> ParameterOf(
        Linear::Point3T<Scalar> point,
        Core::ToleranceT<Scalar> tolerance = {}) const noexcept {
        if (!ContainsPoint(point, tolerance)) {
            return std::nullopt;
        }
        return ClosestParameter(point);
    }

    [[nodiscard]] constexpr Ray3T Translated(Linear::Vector3T<Scalar> vector) const noexcept {
        return {Origin + vector, Direction};
    }

    /// 绕过 `origin`、方向为 `axis` 的轴旋转 `radians` 弧度。方向随线性部分旋转后再归一化。
    [[nodiscard]] Ray3T Rotated(
        Linear::Point3T<Scalar> origin,
        Linear::UnitVector3T<Scalar> axis,
        Scalar radians) const noexcept {
        const Linear::Transform3T<Scalar> rotation =
            Linear::Transform3T<Scalar>::RotationAbout(origin, axis, radians);
        return Ray3T{rotation.TransformPoint(Origin), UnitDirection(rotation)};
    }

    /// 关于过 `point`、法向为 `unitNormal` 的平面反射。
    [[nodiscard]] Ray3T Mirrored(
        Linear::Point3T<Scalar> point, Linear::UnitVector3T<Scalar> unitNormal) const noexcept {
        const Linear::Transform3T<Scalar> mirror =
            Linear::Transform3T<Scalar>::Reflection(point, unitNormal);
        return Ray3T{mirror.TransformPoint(Origin), UnitDirection(mirror)};
    }

    /// 原点不变，方向取反。点集变成从同一原点指向另一侧的射线。
    [[nodiscard]] constexpr Ray3T Reversed() const noexcept {
        return {Origin, -Direction};
    }

    [[nodiscard]] constexpr Ray3T Clone() const noexcept {
        return *this;
    }

    /// 变换原点，方向只施加线性部分再归一化。
    /// 方向无法归一化，或结果含非有限分量时为空。
    [[nodiscard]] std::optional<Ray3T> Transformed(
        const Linear::Transform3T<Scalar>& transform) const noexcept {
        const Linear::Point3T<Scalar> moved = transform.TransformPoint(Origin);
        const Linear::Vector3T<Scalar> transformedDirection = transform * Direction.AsVector();
        if (!Detail::CoordinatesAreFinite(moved)
            || !Detail::CoordinatesAreFinite(transformedDirection)) {
            return std::nullopt;
        }
        const auto unit = transformedDirection.Normalized();
        if (!unit.has_value()) {
            return std::nullopt;
        }
        return Ray3T{moved, *unit};
    }

    /// 有限子区间是同维度线段。区间不在 `[0, +inf)` 内、长度不大于 0，或端点非有限时为空。
    [[nodiscard]] constexpr std::optional<Segment3T<Scalar>> Subcurve(
        Linear::IntervalT<Scalar> interval) const noexcept {
        if (!Detail::IsFiniteSubinterval(interval, Domain())) {
            return std::nullopt;
        }
        return Segment3T<Scalar>{Locate(interval.Min), Locate(interval.Max)};
    }

private:
    [[nodiscard]] constexpr Linear::Point3T<Scalar> Locate(Scalar t) const noexcept {
        return Origin + Direction * t;
    }

    [[nodiscard]] constexpr Scalar ClosestParameter(Linear::Point3T<Scalar> point) const noexcept {
        return Detail::ClampParameter(
            Detail::ProjectParameter(Origin, Direction, point), Domain());
    }

    /// 反射和旋转保持长度。归一化失败时保留变换后的分量，调用方仍得到一条射线。
    [[nodiscard]] Linear::UnitVector3T<Scalar> UnitDirection(
        const Linear::Transform3T<Scalar>& transform) const noexcept {
        const Linear::Vector3T<Scalar> transformed = transform * Direction.AsVector();
        const auto unit = transformed.Normalized();
        if (unit.has_value()) {
            return *unit;
        }
        return Linear::UnitVector3T<Scalar>::FromNormalizedUnchecked(transformed);
    }
};

/// 逐字段比较。`operator!=` 由 C++20 自动生成，不手写。
template <typename Scalar>
[[nodiscard]] constexpr bool operator==(Ray3T<Scalar> a, Ray3T<Scalar> b) noexcept {
    return a.Origin == b.Origin && a.Direction == b.Direction;
}

using Ray3 = Ray3T<double>;
using Ray3f = Ray3T<float>;

} // namespace DragonGeo::Prim
