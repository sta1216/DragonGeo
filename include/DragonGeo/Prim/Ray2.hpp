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
#include <DragonGeo/Prim/Winding.hpp>
#include <DragonGeo/Prim/Segment2.hpp>

namespace DragonGeo::Prim {

template <typename Scalar>
struct Line2T;

/// 二维射线：从 `Origin` 沿 `Direction` 伸出的半直线。
///
/// 参数域是 `[0, +inf)`，`PointAt(t) = Origin + t Direction`。`t` 是沿方向的
/// 有符号距离；负参数与非有限参数为空。没有终点和中点。长度是 `+inf`，
/// 包围盒是规范空盒。
/// 非有限坐标可以存入；`IsValid` 只检查分量是否有限，不检查方向是否已归一化。
template <typename Scalar>
struct Ray2T {
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

    /// 参数域 `[0, +inf)`。
    [[nodiscard]] Linear::IntervalT<Scalar> Domain() const noexcept;

    /// `t < 0` 或 `t` 非有限时为空。
    [[nodiscard]] std::optional<Linear::Point2T<Scalar>> PointAt(Scalar t) const noexcept;

    /// 最近点把参数夹在 `t >= 0`。查询点落在起点后方时，最近点是原点。
    [[nodiscard]] Linear::Point2T<Scalar> ClosestPoint(
        Linear::Point2T<Scalar> point) const noexcept;

    /// 点到射线的平方距离。
    [[nodiscard]] Scalar DistanceSquared(Linear::Point2T<Scalar> point) const noexcept;

    /// 点到射线的距离，即 `DistanceSquared` 的平方根。
    [[nodiscard]] Scalar Distance(Linear::Point2T<Scalar> point) const noexcept;

    /// 射线无界，长度为 `+inf`。
    [[nodiscard]] Scalar Length() const noexcept;

    /// 射线没有有限包围盒，返回规范空盒。
    [[nodiscard]] Linear::Box2T<Scalar> Box() const noexcept;

    [[nodiscard]] std::optional<Linear::Point2T<Scalar>> StartPoint() const noexcept;

    [[nodiscard]] std::optional<Linear::Point2T<Scalar>> EndPoint() const noexcept;

    [[nodiscard]] std::optional<Linear::Point2T<Scalar>> MidPoint() const noexcept;

    [[nodiscard]] std::optional<Linear::UnitVector2T<Scalar>> StartTangent() const noexcept;

    [[nodiscard]] std::optional<Linear::UnitVector2T<Scalar>> EndTangent() const noexcept;

    [[nodiscard]] std::optional<Linear::UnitVector2T<Scalar>> MidTangent() const noexcept;

    /// 域内返回存放的方向。负参数与非有限参数为空。
    [[nodiscard]] std::optional<Linear::UnitVector2T<Scalar>> TangentAt(
        Scalar t) const noexcept;

    [[nodiscard]] bool IsClosed() const noexcept;

    [[nodiscard]] std::optional<Scalar> Area() const noexcept;

    [[nodiscard]] std::optional<Winding> Orientation() const noexcept;

    [[nodiscard]] std::optional<Linear::Point2T<Scalar>> Centroid() const noexcept;

    /// 射线不填充区域。
    [[nodiscard]] bool Contains(Linear::Point2T<Scalar>) const noexcept;

    /// 点到射线的距离不超过 `tolerance.Resolve(1)`。尺度固定为 `1`。
    [[nodiscard]] bool ContainsPoint(
        Linear::Point2T<Scalar> point,
        Core::ToleranceT<Scalar> tolerance = {}) const noexcept;

    /// 最近点的参数。距离不满足同一容差下的 `ContainsPoint` 时为空。
    [[nodiscard]] std::optional<Scalar> ParameterOf(
        Linear::Point2T<Scalar> point,
        Core::ToleranceT<Scalar> tolerance = {}) const noexcept;

    void Translate(Linear::Vector2T<Scalar> vector) noexcept;

    /// 绕 `center` 逆时针旋转 `radians` 弧度。方向随线性部分旋转后再归一化。
    void Rotate(Linear::Point2T<Scalar> center, Scalar radians) noexcept;

    /// 关于过 `point`、法向为 `unitNormal` 的直线反射。
    void Mirror(
        Linear::Point2T<Scalar> point, Linear::UnitVector2T<Scalar> unitNormal) noexcept;

    /// 原点不变，方向取反。点集变成从同一原点指向另一侧的射线。
    void Reverse() noexcept;

    [[nodiscard]] Ray2T Clone() const noexcept;

    /// 变换原点，方向只施加线性部分再归一化。
    /// 方向无法归一化，或结果含非有限分量时返回 `false`，字段保持原样。
    [[nodiscard]] bool Transform(const Linear::Transform2T<Scalar>& transform) noexcept;

    /// 有限子区间是同维度线段。区间不在 `[0, +inf)` 内、长度不大于 0，或端点非有限时为空。
    [[nodiscard]] std::optional<Segment2T<Scalar>> Subcurve(
        Linear::IntervalT<Scalar> interval) const noexcept;

    /// 原点和方向原样带走。
    [[nodiscard]] Line2T<Scalar> AsLine() const noexcept;

    /// `length` 有限且大于 0 时，从原点到 `Origin + length * Direction` 的线段；否则为空。
    [[nodiscard]] std::optional<Segment2T<Scalar>> AsSegment(Scalar length) const noexcept;

private:
    [[nodiscard]] Linear::Point2T<Scalar> Locate(Scalar t) const noexcept;

    [[nodiscard]] Scalar ClosestParameter(Linear::Point2T<Scalar> point) const noexcept;

    /// 反射和旋转保持长度。归一化失败时保留变换后的分量，调用方仍得到一条射线。
    [[nodiscard]] Linear::UnitVector2T<Scalar> UnitDirection(
        const Linear::Transform2T<Scalar>& transform) const noexcept;
};

/// 逐字段比较。`operator!=` 由 C++20 自动生成，不手写。
template <typename Scalar>
[[nodiscard]] constexpr bool operator==(Ray2T<Scalar> a, Ray2T<Scalar> b) noexcept {
    return a.Origin == b.Origin && a.Direction == b.Direction;
}

using Ray2 = Ray2T<double>;
using Ray2f = Ray2T<float>;


extern template struct Ray2T<double>;
extern template struct Ray2T<float>;
} // namespace DragonGeo::Prim
