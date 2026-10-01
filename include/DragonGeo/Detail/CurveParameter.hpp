#pragma once

#include <DragonGeo/Core/Numeric.hpp>
#include <DragonGeo/Linear/Interval.hpp>
#include <DragonGeo/Linear/Point2.hpp>
#include <DragonGeo/Linear/Point3.hpp>
#include <DragonGeo/Linear/Vector2.hpp>
#include <DragonGeo/Linear/Vector3.hpp>

namespace DragonGeo::Detail {

/// 点在以 `origin` 为原点、`directionUnit` 为单位方向的轴上的有符号参数。
/// 等于 `(point - origin) · direction`。
template <typename Point, typename UnitDirection>
[[nodiscard]] constexpr auto ProjectParameter(
    Point origin, UnitDirection directionUnit, Point point) noexcept {
    return (point - origin).Dot(directionUnit.AsVector());
}

/// 把参数夹进 `domain`。
///
/// 线段传入 `[0, 1]`，射线传入 `[0, +inf)`。直线的参数不夹，调用方不要把
/// 直线域传进来。空域没有可夹紧的端点，参数原样返回。
template <typename Scalar>
[[nodiscard]] constexpr Scalar ClampParameter(
    Scalar parameter, Linear::IntervalT<Scalar> domain) noexcept {
    if (domain.IsEmpty()) {
        return parameter;
    }
    if (parameter < domain.Min) {
        return domain.Min;
    }
    if (parameter > domain.Max) {
        return domain.Max;
    }
    return parameter;
}

/// `PointAt` 接受的参数：必须有限，并且落在闭域内。
///
/// 直线的无界域包含 ±inf，但非有限参数仍然拒绝。
template <typename Scalar>
[[nodiscard]] constexpr bool IsAcceptedParameter(
    Scalar parameter, Linear::IntervalT<Scalar> domain) noexcept {
    return Core::IsFinite(parameter) && domain.Contains(parameter);
}

/// 能表示成线段的子区间：两端都有限，长度大于 0，并且整个区间落在 `domain` 内。
///
/// 长度为 0 的退化区间、倒置区间、越出参数域的区间、以及射线和直线上含无穷端点的
/// 区间都不是。参数域是凸的，两端都在域内就表示整段都在域内。
template <typename Scalar>
[[nodiscard]] constexpr bool IsFiniteSubinterval(
    Linear::IntervalT<Scalar> interval, Linear::IntervalT<Scalar> domain) noexcept {
    if (!Core::IsFinite(interval.Min) || !Core::IsFinite(interval.Max)) {
        return false;
    }
    if (!(interval.Length() > Scalar{0})) {
        return false;
    }
    return domain.Contains(interval.Min) && domain.Contains(interval.Max);
}

template <typename Scalar>
[[nodiscard]] constexpr bool CoordinatesAreFinite(Linear::Point2T<Scalar> point) noexcept {
    return Core::IsFinite(point.X) && Core::IsFinite(point.Y);
}

template <typename Scalar>
[[nodiscard]] constexpr bool CoordinatesAreFinite(Linear::Point3T<Scalar> point) noexcept {
    return Core::IsFinite(point.X) && Core::IsFinite(point.Y) && Core::IsFinite(point.Z);
}

template <typename Scalar>
[[nodiscard]] constexpr bool CoordinatesAreFinite(Linear::Vector2T<Scalar> vector) noexcept {
    return Core::IsFinite(vector.X) && Core::IsFinite(vector.Y);
}

template <typename Scalar>
[[nodiscard]] constexpr bool CoordinatesAreFinite(Linear::Vector3T<Scalar> vector) noexcept {
    return Core::IsFinite(vector.X) && Core::IsFinite(vector.Y) && Core::IsFinite(vector.Z);
}

} // namespace DragonGeo::Detail
