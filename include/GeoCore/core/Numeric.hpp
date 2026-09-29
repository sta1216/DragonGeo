#pragma once

#include <concepts>
#include <cmath>
#include <limits>

namespace GeoCore::core {

/// 绝对值。与 std::abs 的差别：本函数是 constexpr（C++20 的 std::abs
/// 对标量浮点尚不是），且对 -0.0 返回 -0.0 —— 由于 -0.0 == 0.0，
/// 这不影响任何比较结果。
template <std::floating_point Scalar>
[[nodiscard]] constexpr Scalar absolute_value(Scalar value) noexcept {
    return value < Scalar{0} ? -value : value;
}

/// 是否为有限值（既非 ±inf 也非 NaN）。
///
/// 手写而非使用 std::isfinite：后者在 C++20 尚不是 constexpr，而本层的
/// 工具函数需要能在常量表达式中求值。Tolerance 依赖它来拒绝非有限输入 ——
/// 没有这个判断，`|inf - 5| <= resolve(inf)` 会退化成 `inf <= inf`，
/// 把无穷大判成「与任何有限值相等」。
template <std::floating_point Scalar>
[[nodiscard]] constexpr bool is_finite(Scalar value) noexcept {
    return value == value
        && value != std::numeric_limits<Scalar>::infinity()
        && value != -std::numeric_limits<Scalar>::infinity();
}

/// 把 value 限制到 [low, high]。要求 low <= high。
template <std::floating_point Scalar>
[[nodiscard]] constexpr Scalar clamp(Scalar value, Scalar low, Scalar high) noexcept {
    if (value < low) {
        return low;
    }
    if (value > high) {
        return high;
    }
    return value;
}

/// 平方根，负输入返回 0。
///
/// 存在意义：浮点舍入可能让本应为零的平方和变为极小的负数，
/// 此时 std::sqrt 会返回 NaN 并污染整条计算链。
///
/// 注意：-0.0 与恰好为 0 的输入返回 0；NaN 输入返回 NaN。
template <std::floating_point Scalar>
[[nodiscard]] inline Scalar safe_sqrt(Scalar value) noexcept {
    if (value <= Scalar{0}) {
        return Scalar{0};
    }
    return std::sqrt(value);
}

} // namespace GeoCore::core
