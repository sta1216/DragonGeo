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
#include <DragonGeo/Prim/Winding.hpp>

namespace DragonGeo::Prim {

template <typename Scalar> struct Ray2T;

/// 二维直线：过 `Origin`、方向为 `Direction` 的双向直线。
///
/// 参数域是 `IntervalT::Unbounded()`，即 `[-inf, +inf]`。`PointAt(t) = Origin + t Direction`， `t` 是沿方向的有符号距离。任何有限 `t` 都接受，非有限 `t` 为空。没有起点、终点和中点。
/// 长度是 `+inf`，包围盒是规范空盒。最近点不夹参数。非有限坐标可以存入；`IsValid` 只检查分量是否有限，不检查方向是否已归一化。
template <typename Scalar> struct Line2T {
    using ScalarType = Scalar;

    // 同 Point2T / Box2T：不声明任何构造函数，以保持聚合性。单位向量没有默认构造函数，故 Direction 不写默认成员初始化器。
    Linear::Point2T<Scalar> Origin{};
    Linear::UnitVector2T<Scalar> Direction;

    /// 原点与方向的每个分量均为有限值时为真。
    [[nodiscard]] constexpr bool IsValid() const noexcept {
        using Core::IsFinite;
        return IsFinite(Origin.X) && IsFinite(Origin.Y) && IsFinite(Direction.X()) && IsFinite(Direction.Y());
    }

    /// 无界参数域 `[-inf, +inf]`。
    [[nodiscard]] Linear::IntervalT<Scalar> Domain() const noexcept;

    /// 任何有限 `t` 都有点。非有限 `t` 为空。
    [[nodiscard]] std::optional<Linear::Point2T<Scalar>> PointAt(Scalar t) const noexcept;

    /// 最近点不夹参数，负参数保留。
    [[nodiscard]] Linear::Point2T<Scalar> ClosestPoint(Linear::Point2T<Scalar> point) const noexcept;

    /// 点到直线的平方距离。
    [[nodiscard]] Scalar DistanceSquared(Linear::Point2T<Scalar> point) const noexcept;

    /// 点到直线的距离，即 `DistanceSquared` 的平方根。
    [[nodiscard]] Scalar Distance(Linear::Point2T<Scalar> point) const noexcept;

    /// 直线无界，长度为 `+inf`。
    [[nodiscard]] Scalar Length() const noexcept;

    /// 直线没有有限包围盒，返回规范空盒。
    [[nodiscard]] Linear::Box2T<Scalar> Box() const noexcept;

    [[nodiscard]] std::optional<Linear::Point2T<Scalar>> StartPoint() const noexcept;

    [[nodiscard]] std::optional<Linear::Point2T<Scalar>> EndPoint() const noexcept;

    [[nodiscard]] std::optional<Linear::Point2T<Scalar>> MidPoint() const noexcept;

    [[nodiscard]] std::optional<Linear::UnitVector2T<Scalar>> StartTangent() const noexcept;

    [[nodiscard]] std::optional<Linear::UnitVector2T<Scalar>> EndTangent() const noexcept;

    [[nodiscard]] std::optional<Linear::UnitVector2T<Scalar>> MidTangent() const noexcept;

    /// 有限参数处返回存放的方向。非有限参数为空。
    [[nodiscard]] std::optional<Linear::UnitVector2T<Scalar>> TangentAt(Scalar t) const noexcept;

    [[nodiscard]] bool IsClosed() const noexcept;

    [[nodiscard]] std::optional<Scalar> Area() const noexcept;

    [[nodiscard]] std::optional<Winding> Orientation() const noexcept;

    [[nodiscard]] std::optional<Linear::Point2T<Scalar>> Centroid() const noexcept;

    /// 直线不填充区域。
    [[nodiscard]] bool Contains(Linear::Point2T<Scalar>) const noexcept;

    /// 点到直线的距离不超过 `tolerance.Resolve(1)`。尺度固定为 `1`。
    [[nodiscard]] bool ContainsPoint(Linear::Point2T<Scalar> point, Core::ToleranceT<Scalar> tolerance = {}) const noexcept;

    /// 最近点的参数，可以是负数。距离不满足同一容差下的 `ContainsPoint` 时为空。
    [[nodiscard]] std::optional<Scalar> ParameterOf(Linear::Point2T<Scalar> point, Core::ToleranceT<Scalar> tolerance = {}) const noexcept;

    void Translate(Linear::Vector2T<Scalar> vector) noexcept;

    /// 绕 `center` 逆时针旋转 `radians` 弧度。方向随线性部分旋转后再归一化。
    void Rotate(Linear::Point2T<Scalar> center, Scalar radians) noexcept;

    /// 关于过 `point`、法向为 `unitNormal` 的直线反射。
    void Mirror(Linear::Point2T<Scalar> point, Linear::UnitVector2T<Scalar> unitNormal) noexcept;

    /// 原点不变，方向取反。点集不变，参数方向相反。
    void Reverse() noexcept;

    [[nodiscard]] Line2T Clone() const noexcept;

    /// 变换原点，方向只施加线性部分再归一化。方向无法归一化，或结果含非有限分量时返回 `false`，字段保持原样。
    [[nodiscard]] bool Transform(const Linear::Transform2T<Scalar>& transform) noexcept;

    /// 有限子区间是同维度线段。端点非有限或长度不大于 0 时为空。
    [[nodiscard]] std::optional<Segment2T<Scalar>> Subcurve(Linear::IntervalT<Scalar> interval) const noexcept;

    /// 用这条直线存放的原点和方向。
    [[nodiscard]] Ray2T<Scalar> AsRay() const noexcept;

    /// 两参数均有限且不相等时，从 `PointAt(parameterStart)` 到 `PointAt(parameterEnd)` 的线段；否则为空。
    [[nodiscard]] std::optional<Segment2T<Scalar>> AsSegment(Scalar parameterStart, Scalar parameterEnd) const noexcept;

private:
    [[nodiscard]] Linear::Point2T<Scalar> Locate(Scalar t) const noexcept;

    /// 直线不夹参数。
    [[nodiscard]] Scalar ClosestParameter(Linear::Point2T<Scalar> point) const noexcept;

    /// 反射和旋转保持长度。归一化失败时保留变换后的分量，调用方仍得到一条直线。
    [[nodiscard]] Linear::UnitVector2T<Scalar> UnitDirection(const Linear::Transform2T<Scalar>& transform) const noexcept;
};

/// 逐字段比较。`operator!=` 由 C++20 自动生成，不手写。
template <typename Scalar> [[nodiscard]] constexpr bool operator==(Line2T<Scalar> a, Line2T<Scalar> b) noexcept {
    return a.Origin == b.Origin && a.Direction == b.Direction;
}

using Line2 = Line2T<double>;
using Line2f = Line2T<float>;


extern template struct Line2T<double>;
extern template struct Line2T<float>;
} // namespace DragonGeo::Prim
