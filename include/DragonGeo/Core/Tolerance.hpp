#pragma once

#include <concepts>

#include <DragonGeo/Core/Numeric.hpp>

namespace DragonGeo::Core {

/// `double` 默认绝对项（与历史库行为一致）。
template <std::floating_point Scalar>
[[nodiscard]] constexpr Scalar DefaultToleranceAbs() noexcept {
    if constexpr (std::is_same_v<Scalar, float>) {
        // 约为 float 机器 epsilon 量级（~1.2e-7）的若干倍，供 float 管线默认使用。
        return Scalar{1e-6f};
    } else {
        return Scalar{1e-12};
    }
}

/// `double` 默认相对项（与历史库行为一致）。
template <std::floating_point Scalar>
[[nodiscard]] constexpr Scalar DefaultToleranceRel() noexcept {
    if constexpr (std::is_same_v<Scalar, float>) {
        return Scalar{1e-5f};
    } else {
        return Scalar{1e-9};
    }
}

/// 显式的容差模型，标量类型与 `VectorNT` / `MatrixT` 一致。
///
/// DragonGeo 刻意不提供全局 epsilon 常量：单一阈值在坐标尺度相差数个
/// 数量级的输入上必然失效。容差作为参数显式传递，调用者始终知道
/// 自己在何种精度下工作。
///
/// 有效容差 = Abs + Rel * |magnitude|，即「绝对项」与「随量级增长的
/// 相对项」之和。
template <std::floating_point Scalar>
struct ToleranceT {
    /// 绝对项，与量级无关的下限。
    Scalar Abs = DefaultToleranceAbs<Scalar>();

    /// 相对项，随参考量级线性增长。
    Scalar Rel = DefaultToleranceRel<Scalar>();

    /// 给出参考量级对应的有效容差。
    [[nodiscard]] constexpr Scalar Resolve(Scalar magnitude) const noexcept {
        return Abs + Rel * AbsoluteValue(magnitude);
    }

    /// 判定两个标量在该容差下是否可视为相等。
    /// 参考量级取二者绝对值中的较大者。
    ///
    /// 非有限输入另行处理：完全相等（含 ±inf 与自身）返回 true，其余任何
    /// 涉及 ±inf 或 NaN 的组合一律返回 false。
    [[nodiscard]] constexpr bool Equal(Scalar a, Scalar b) const noexcept {
        if (a == b) {
            return true;
        }
        if (!IsFinite(a) || !IsFinite(b)) {
            return false;
        }
        const Scalar scale = AbsoluteValue(a) > AbsoluteValue(b) ? AbsoluteValue(a) : AbsoluteValue(b);
        return AbsoluteValue(a - b) <= Resolve(scale);
    }

    /// 判定标量在该容差下是否可视为零。
    ///
    /// 非有限值一律不视为零。
    [[nodiscard]] constexpr bool IsZero(Scalar x) const noexcept {
        if (!IsFinite(x)) {
            return false;
        }
        return AbsoluteValue(x) <= Resolve(x);
    }
};

using Tolerance = ToleranceT<double>;
using Tolerancef = ToleranceT<float>;

} // namespace DragonGeo::Core
