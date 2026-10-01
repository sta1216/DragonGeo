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
#include <DragonGeo/Prim/Segment2.hpp>

namespace DragonGeo::Prim {

/// 二维直线：过 `Origin`、方向为 `Direction` 的双向直线。
///
/// 参数域是 `IntervalT::Unbounded()`，即 `[-inf, +inf]`。`PointAt(t) = Origin + t Direction`，
/// `t` 是沿方向的有符号距离。任何有限 `t` 都接受，非有限 `t` 为空。没有起点、终点和中点。
/// 长度是 `+inf`，包围盒是规范空盒。最近点不夹参数。
/// 非有限坐标可以存入；`IsValid` 只检查分量是否有限，不检查方向是否已归一化。
template <typename Scalar>
struct Line2T {
    using ScalarType = Scalar;

    // 同 Point2T / Box2T：不声明任何构造函数，以保持聚合性。
    // 单位向量没有默认构造函数，故 Direction 不写默认成员初始化器。
    Linear::Point2T<Scalar> Origin{};
    Linear::UnitVector2T<Scalar> Direction;

    /// 原点与方向的每个分量均为有限值时为真。
    [[nodiscard]] constexpr bool IsValid() const noexcept {
        using Core::IsFinite;
        return IsFinite(Origin.X) && IsFinite(Origin.Y) && IsFinite(Direction.X())
            && IsFinite(Direction.Y());
    }

    /// 无界参数域 `[-inf, +inf]`。
    [[nodiscard]] constexpr Linear::IntervalT<Scalar> Domain() const noexcept {
        return Linear::IntervalT<Scalar>::Unbounded();
    }

    /// 任何有限 `t` 都有点。非有限 `t` 为空。
    [[nodiscard]] constexpr std::optional<Linear::Point2T<Scalar>> PointAt(Scalar t) const noexcept {
        if (!Detail::IsAcceptedParameter(t, Domain())) {
            return std::nullopt;
        }
        return Locate(t);
    }

    /// 最近点不夹参数，负参数保留。
    [[nodiscard]] constexpr Linear::Point2T<Scalar> ClosestPoint(
        Linear::Point2T<Scalar> point) const noexcept {
        return Locate(ClosestParameter(point));
    }

    /// 点到直线的平方距离。
    [[nodiscard]] constexpr Scalar DistanceSquared(Linear::Point2T<Scalar> point) const noexcept {
        return (point - ClosestPoint(point)).LengthSquared();
    }

    /// 点到直线的距离，即 `DistanceSquared` 的平方根。
    [[nodiscard]] Scalar Distance(Linear::Point2T<Scalar> point) const noexcept {
        return std::sqrt(DistanceSquared(point));
    }

    /// 直线无界，长度为 `+inf`。
    [[nodiscard]] constexpr Scalar Length() const noexcept {
        return std::numeric_limits<Scalar>::infinity();
    }

    /// 直线没有有限包围盒，返回规范空盒。
    [[nodiscard]] constexpr Linear::Box2T<Scalar> Bounds() const noexcept {
        return Linear::Box2T<Scalar>::Empty();
    }

    [[nodiscard]] constexpr std::optional<Linear::Point2T<Scalar>> StartPoint() const noexcept {
        return std::nullopt;
    }

    [[nodiscard]] constexpr std::optional<Linear::Point2T<Scalar>> EndPoint() const noexcept {
        return std::nullopt;
    }

    [[nodiscard]] constexpr std::optional<Linear::Point2T<Scalar>> MidPoint() const noexcept {
        return std::nullopt;
    }

    [[nodiscard]] constexpr std::optional<Linear::UnitVector2T<Scalar>> StartTangent() const noexcept {
        return std::nullopt;
    }

    [[nodiscard]] constexpr std::optional<Linear::UnitVector2T<Scalar>> EndTangent() const noexcept {
        return std::nullopt;
    }

    [[nodiscard]] constexpr std::optional<Linear::UnitVector2T<Scalar>> MidTangent() const noexcept {
        return std::nullopt;
    }

    /// 有限参数处返回存放的方向。非有限参数为空。
    [[nodiscard]] constexpr std::optional<Linear::UnitVector2T<Scalar>> TangentAt(
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

    [[nodiscard]] constexpr std::optional<Linear::Point2T<Scalar>> Centroid() const noexcept {
        return std::nullopt;
    }

    /// 直线不填充区域。
    [[nodiscard]] constexpr bool Contains(Linear::Point2T<Scalar>) const noexcept {
        return false;
    }

    /// 点到直线的距离不超过 `tolerance.Resolve(1)`。尺度固定为 `1`。
    [[nodiscard]] bool ContainsPoint(
        Linear::Point2T<Scalar> point,
        Core::ToleranceT<Scalar> tolerance = {}) const noexcept {
        return Distance(point) <= tolerance.Resolve(Scalar{1});
    }

    /// 最近点的参数，可以是负数。距离不满足同一容差下的 `ContainsPoint` 时为空。
    [[nodiscard]] std::optional<Scalar> ParameterOf(
        Linear::Point2T<Scalar> point,
        Core::ToleranceT<Scalar> tolerance = {}) const noexcept {
        if (!ContainsPoint(point, tolerance)) {
            return std::nullopt;
        }
        return ClosestParameter(point);
    }

    [[nodiscard]] constexpr Line2T Translated(Linear::Vector2T<Scalar> vector) const noexcept {
        return {Origin + vector, Direction};
    }

    /// 绕 `center` 逆时针旋转 `radians` 弧度。方向随线性部分旋转后再归一化。
    [[nodiscard]] Line2T Rotated(Linear::Point2T<Scalar> center, Scalar radians) const noexcept {
        const Linear::Transform2T<Scalar> rotation =
            Linear::Transform2T<Scalar>::RotationAbout(center, radians);
        return Line2T{rotation.TransformPoint(Origin), UnitDirection(rotation)};
    }

    /// 关于过 `point`、法向为 `unitNormal` 的直线反射。
    [[nodiscard]] Line2T Mirrored(
        Linear::Point2T<Scalar> point, Linear::UnitVector2T<Scalar> unitNormal) const noexcept {
        const Linear::Transform2T<Scalar> mirror =
            Linear::Transform2T<Scalar>::Reflection(point, unitNormal);
        return Line2T{mirror.TransformPoint(Origin), UnitDirection(mirror)};
    }

    /// 原点不变，方向取反。点集不变，参数方向相反。
    [[nodiscard]] constexpr Line2T Reversed() const noexcept {
        return {Origin, -Direction};
    }

    [[nodiscard]] constexpr Line2T Clone() const noexcept {
        return *this;
    }

    /// 变换原点，方向只施加线性部分再归一化。
    /// 方向无法归一化，或结果含非有限分量时为空。
    [[nodiscard]] std::optional<Line2T> Transformed(
        const Linear::Transform2T<Scalar>& transform) const noexcept {
        const Linear::Point2T<Scalar> moved = transform.TransformPoint(Origin);
        const Linear::Vector2T<Scalar> transformedDirection = transform * Direction.AsVector();
        if (!Detail::CoordinatesAreFinite(moved)
            || !Detail::CoordinatesAreFinite(transformedDirection)) {
            return std::nullopt;
        }
        const auto unit = transformedDirection.Normalized();
        if (!unit.has_value()) {
            return std::nullopt;
        }
        return Line2T{moved, *unit};
    }

    /// 有限子区间是同维度线段。端点非有限或长度不大于 0 时为空。
    [[nodiscard]] constexpr std::optional<Segment2T<Scalar>> Subcurve(
        Linear::IntervalT<Scalar> interval) const noexcept {
        if (!Detail::IsFiniteSubinterval(interval, Domain())) {
            return std::nullopt;
        }
        return Segment2T<Scalar>{Locate(interval.Min), Locate(interval.Max)};
    }

private:
    [[nodiscard]] constexpr Linear::Point2T<Scalar> Locate(Scalar t) const noexcept {
        return Origin + Direction * t;
    }

    /// 直线不夹参数。
    [[nodiscard]] constexpr Scalar ClosestParameter(Linear::Point2T<Scalar> point) const noexcept {
        return Detail::ProjectParameter(Origin, Direction, point);
    }

    /// 反射和旋转保持长度。归一化失败时保留变换后的分量，调用方仍得到一条直线。
    [[nodiscard]] Linear::UnitVector2T<Scalar> UnitDirection(
        const Linear::Transform2T<Scalar>& transform) const noexcept {
        const Linear::Vector2T<Scalar> transformed = transform * Direction.AsVector();
        const auto unit = transformed.Normalized();
        if (unit.has_value()) {
            return *unit;
        }
        return Linear::UnitVector2T<Scalar>::FromNormalizedUnchecked(transformed);
    }
};

/// 逐字段比较。`operator!=` 由 C++20 自动生成，不手写。
template <typename Scalar>
[[nodiscard]] constexpr bool operator==(Line2T<Scalar> a, Line2T<Scalar> b) noexcept {
    return a.Origin == b.Origin && a.Direction == b.Direction;
}

using Line2 = Line2T<double>;
using Line2f = Line2T<float>;

} // namespace DragonGeo::Prim
