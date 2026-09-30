#pragma once

#include <concepts>
#include <cmath>
#include <limits>

namespace DragonGeo::Core {

/// 绝对值。与 std::abs 的差别：本函数是 constexpr（C++20 的 std::abs
/// 对标量浮点尚不是），且对 -0.0 返回 -0.0 —— 由于 -0.0 == 0.0，
/// 这不影响任何比较结果。
template <std::floating_point Scalar>
[[nodiscard]] constexpr Scalar AbsoluteValue(Scalar value) noexcept {
    return value < Scalar{0} ? -value : value;
}

/// 是否为有限值（既非 ±inf 也非 NaN）。
///
/// 手写而非使用 std::isfinite：后者在 C++20 尚不是 constexpr，而本层的
/// 工具函数需要能在常量表达式中求值。Tolerance 依赖它来拒绝非有限输入 ——
/// 没有这个判断，`|inf - 5| <= Resolve(inf)` 会退化成 `inf <= inf`，
/// 把无穷大判成「与任何有限值相等」。
template <std::floating_point Scalar>
[[nodiscard]] constexpr bool IsFinite(Scalar value) noexcept {
    return value == value
        && value != std::numeric_limits<Scalar>::infinity()
        && value != -std::numeric_limits<Scalar>::infinity();
}

/// 一组分量的最大绝对值，并对非有限输入规定确定的答案。
///
/// 规则：任一分量为 ±inf ⇒ +inf；否则任一分量为 NaN ⇒ NaN；否则取最大绝对值
/// （恒为非负）。若无此规定，逐项比较遇到 NaN 时比较均返回 false，fold 会
/// 静默保留前一个值 —— 于是同一个量在不同元数的类型上给出互相矛盾的答案。
///
/// 合并是可交换、可结合的：±inf 压过 NaN，NaN 压过任何有限值。因此
/// MaxAbsOf(a, b, c) 展开成两两合并时与参数顺序无关。
template <std::floating_point Scalar>
[[nodiscard]] constexpr Scalar MaxAbsOf(Scalar a, Scalar b) noexcept {
    const Scalar absA = AbsoluteValue(a);
    const Scalar absB = AbsoluteValue(b);
    const Scalar infinity = std::numeric_limits<Scalar>::infinity();

    if (absA == infinity || absB == infinity) {
        // 必须先判 inf：与 NaN 同时出现时，规范给出的答案是 +inf。
        return infinity;
    }
    if (!IsFinite(absA) || !IsFinite(absB)) {
        return std::numeric_limits<Scalar>::quiet_NaN();   // 此刻只可能是 NaN
    }
    return absA > absB ? absA : absB;
}

template <std::floating_point Scalar>
[[nodiscard]] constexpr Scalar MaxAbsOf(Scalar a, Scalar b, Scalar c) noexcept {
    return MaxAbsOf(MaxAbsOf(a, b), c);
}

template <std::floating_point Scalar>
[[nodiscard]] constexpr Scalar MaxAbsOf(Scalar a, Scalar b, Scalar c, Scalar d) noexcept {
    return MaxAbsOf(MaxAbsOf(a, b), MaxAbsOf(c, d));
}

/// 把 value 限制到 [low, high]。要求 low <= high。
template <std::floating_point Scalar>
[[nodiscard]] constexpr Scalar Clamp(Scalar value, Scalar low, Scalar high) noexcept {
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
[[nodiscard]] inline Scalar SafeSqrt(Scalar value) noexcept {
    if (value <= Scalar{0}) {
        return Scalar{0};
    }
    return std::sqrt(value);
}

} // namespace DragonGeo::Core
