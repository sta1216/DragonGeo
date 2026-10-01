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
#include <DragonGeo/Prim/Winding.hpp>

namespace DragonGeo::Prim {

template <typename Scalar>
struct Ray3T;

/// 三维直线：过 `Origin`、方向为 `Direction` 的双向直线。
///
/// 参数域是 `IntervalT::Unbounded()`，即 `[-inf, +inf]`。`PointAt(t) = Origin + t Direction`，
/// `t` 是沿方向的有符号距离。任何有限 `t` 都接受，非有限 `t` 为空。没有起点、终点和中点。
/// 长度是 `+inf`，包围盒是规范空盒。最近点不夹参数。
/// 非有限坐标可以存入；`IsValid` 只检查分量是否有限，不检查方向是否已归一化。
template <typename Scalar>
struct Line3T {
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

    /// 无界参数域 `[-inf, +inf]`。
    [[nodiscard]] constexpr Linear::IntervalT<Scalar> Domain() const noexcept {
        return Linear::IntervalT<Scalar>::Unbounded();
    }

    /// 任何有限 `t` 都有点。非有限 `t` 为空。
    [[nodiscard]] constexpr std::optional<Linear::Point3T<Scalar>> PointAt(Scalar t) const noexcept {
        if (!Detail::IsAcceptedParameter(t, Domain())) {
            return std::nullopt;
        }
        return Locate(t);
    }

    /// 最近点不夹参数，负参数保留。
    [[nodiscard]] constexpr Linear::Point3T<Scalar> ClosestPoint(
        Linear::Point3T<Scalar> point) const noexcept {
        return Locate(ClosestParameter(point));
    }

    /// 点到直线的平方距离。
    [[nodiscard]] constexpr Scalar DistanceSquared(Linear::Point3T<Scalar> point) const noexcept {
        return (point - ClosestPoint(point)).LengthSquared();
    }

    /// 点到直线的距离，即 `DistanceSquared` 的平方根。
    [[nodiscard]] Scalar Distance(Linear::Point3T<Scalar> point) const noexcept {
        return std::sqrt(DistanceSquared(point));
    }

    /// 直线无界，长度为 `+inf`。
    [[nodiscard]] constexpr Scalar Length() const noexcept {
        return std::numeric_limits<Scalar>::infinity();
    }

    /// 直线没有有限包围盒，返回规范空盒。
    [[nodiscard]] constexpr Linear::Box3T<Scalar> Box() const noexcept {
        return Linear::Box3T<Scalar>::Empty();
    }

    [[nodiscard]] constexpr std::optional<Linear::Point3T<Scalar>> StartPoint() const noexcept {
        return std::nullopt;
    }

    [[nodiscard]] constexpr std::optional<Linear::Point3T<Scalar>> EndPoint() const noexcept {
        return std::nullopt;
    }

    [[nodiscard]] constexpr std::optional<Linear::Point3T<Scalar>> MidPoint() const noexcept {
        return std::nullopt;
    }

    [[nodiscard]] constexpr std::optional<Linear::UnitVector3T<Scalar>> StartTangent() const noexcept {
        return std::nullopt;
    }

    [[nodiscard]] constexpr std::optional<Linear::UnitVector3T<Scalar>> EndTangent() const noexcept {
        return std::nullopt;
    }

    [[nodiscard]] constexpr std::optional<Linear::UnitVector3T<Scalar>> MidTangent() const noexcept {
        return std::nullopt;
    }

    /// 有限参数处返回存放的方向。非有限参数为空。
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

    [[nodiscard]] constexpr std::optional<Winding> Orientation() const noexcept {
        return std::nullopt;
    }

    [[nodiscard]] constexpr std::optional<Linear::Point3T<Scalar>> Centroid() const noexcept {
        return std::nullopt;
    }

    /// 直线不填充区域。
    [[nodiscard]] constexpr bool Contains(Linear::Point3T<Scalar>) const noexcept {
        return false;
    }

    /// 点到直线的距离不超过 `tolerance.Resolve(1)`。尺度固定为 `1`。
    [[nodiscard]] bool ContainsPoint(
        Linear::Point3T<Scalar> point,
        Core::ToleranceT<Scalar> tolerance = {}) const noexcept {
        return Distance(point) <= tolerance.Resolve(Scalar{1});
    }

    /// 最近点的参数，可以是负数。距离不满足同一容差下的 `ContainsPoint` 时为空。
    [[nodiscard]] std::optional<Scalar> ParameterOf(
        Linear::Point3T<Scalar> point,
        Core::ToleranceT<Scalar> tolerance = {}) const noexcept {
        if (!ContainsPoint(point, tolerance)) {
            return std::nullopt;
        }
        return ClosestParameter(point);
    }

    constexpr void Translate(Linear::Vector3T<Scalar> vector) noexcept {
        Origin = Origin + vector;
    }

    /// 绕过 `origin`、方向为 `axis` 的轴旋转 `radians` 弧度。方向随线性部分旋转后再归一化。
    void Rotate(
        Linear::Point3T<Scalar> origin,
        Linear::UnitVector3T<Scalar> axis,
        Scalar radians) noexcept {
        const Linear::Transform3T<Scalar> rotation =
            Linear::Transform3T<Scalar>::RotationAbout(origin, axis, radians);
        const Linear::UnitVector3T<Scalar> direction = UnitDirection(rotation);
        Origin = rotation.TransformPoint(Origin);
        Direction = direction;
    }

    /// 关于过 `point`、法向为 `unitNormal` 的平面反射。
    void Mirror(
        Linear::Point3T<Scalar> point, Linear::UnitVector3T<Scalar> unitNormal) noexcept {
        const Linear::Transform3T<Scalar> mirror =
            Linear::Transform3T<Scalar>::Reflection(point, unitNormal);
        const Linear::UnitVector3T<Scalar> direction = UnitDirection(mirror);
        Origin = mirror.TransformPoint(Origin);
        Direction = direction;
    }

    /// 原点不变，方向取反。点集不变，参数方向相反。
    constexpr void Reverse() noexcept {
        Direction = -Direction;
    }

    [[nodiscard]] constexpr Line3T Clone() const noexcept {
        return *this;
    }

    /// 变换原点，方向只施加线性部分再归一化。
    /// 方向无法归一化，或结果含非有限分量时返回 `false`，字段保持原样。
    [[nodiscard]] bool Transform(const Linear::Transform3T<Scalar>& transform) noexcept {
        const Linear::Point3T<Scalar> moved = transform.TransformPoint(Origin);
        const Linear::Vector3T<Scalar> transformedDirection = transform * Direction.AsVector();
        if (!Detail::CoordinatesAreFinite(moved)
            || !Detail::CoordinatesAreFinite(transformedDirection)) {
            return false;
        }
        const auto unit = transformedDirection.Normalized();
        if (!unit.has_value()) {
            return false;
        }
        Origin = moved;
        Direction = *unit;
        return true;
    }

    /// 有限子区间是同维度线段。端点非有限或长度不大于 0 时为空。
    [[nodiscard]] constexpr std::optional<Segment3T<Scalar>> Subcurve(
        Linear::IntervalT<Scalar> interval) const noexcept {
        if (!Detail::IsFiniteSubinterval(interval, Domain())) {
            return std::nullopt;
        }
        return Segment3T<Scalar>{Locate(interval.Min), Locate(interval.Max)};
    }

    /// 用这条直线存放的原点和方向。
    [[nodiscard]] Ray3T<Scalar> AsRay() const noexcept;

    /// 两参数均有限且不相等时，从 `PointAt(parameterStart)` 到 `PointAt(parameterEnd)` 的线段；否则为空。
    [[nodiscard]] std::optional<Segment3T<Scalar>> AsSegment(
        Scalar parameterStart, Scalar parameterEnd) const noexcept {
        if (!Core::IsFinite(parameterStart) || !Core::IsFinite(parameterEnd)
            || parameterStart == parameterEnd) {
            return std::nullopt;
        }
        const auto start = PointAt(parameterStart);
        const auto end = PointAt(parameterEnd);
        if (!start.has_value() || !end.has_value()) {
            return std::nullopt;
        }
        return Segment3T<Scalar>{*start, *end};
    }

private:
    [[nodiscard]] constexpr Linear::Point3T<Scalar> Locate(Scalar t) const noexcept {
        return Origin + Direction * t;
    }

    /// 直线不夹参数。
    [[nodiscard]] constexpr Scalar ClosestParameter(Linear::Point3T<Scalar> point) const noexcept {
        return Detail::ProjectParameter(Origin, Direction, point);
    }

    /// 反射和旋转保持长度。归一化失败时保留变换后的分量，调用方仍得到一条直线。
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
[[nodiscard]] constexpr bool operator==(Line3T<Scalar> a, Line3T<Scalar> b) noexcept {
    return a.Origin == b.Origin && a.Direction == b.Direction;
}

using Line3 = Line3T<double>;
using Line3f = Line3T<float>;

} // namespace DragonGeo::Prim

#include <DragonGeo/Prim/Ray3.hpp>

namespace DragonGeo::Prim {

template <typename Scalar>
[[nodiscard]] std::optional<Ray3T<Scalar>> Segment3T<Scalar>::AsRay() const noexcept {
    const auto unitDirection = Direction();
    if (!unitDirection.has_value()) {
        return std::nullopt;
    }
    return Ray3T<Scalar>{A, *unitDirection};
}

template <typename Scalar>
[[nodiscard]] std::optional<Line3T<Scalar>> Segment3T<Scalar>::AsLine() const noexcept {
    const auto unitDirection = Direction();
    if (!unitDirection.has_value()) {
        return std::nullopt;
    }
    return Line3T<Scalar>{A, *unitDirection};
}

template <typename Scalar>
[[nodiscard]] Ray3T<Scalar> Line3T<Scalar>::AsRay() const noexcept {
    return {Origin, Direction};
}

} // namespace DragonGeo::Prim
