#pragma once

#include <concepts>
#include <cmath>

namespace GeoCore::core {

/// 绝对值。与 std::abs 的差别：本函数是 constexpr（C++20 的 std::abs
/// 对标量浮点尚不是），且对 -0.0 返回 -0.0 —— 由于 -0.0 == 0.0，
/// 这不影响任何比较结果。
template <std::floating_point Scalar>
[[nodiscard]] constexpr Scalar absolute_value(Scalar value) noexcept {
    return value < Scalar{0} ? -value : value;
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
