#pragma once

#include <DragonGeo/Core/Numeric.hpp>

namespace DragonGeo::Core {

/// 显式的容差模型。
///
/// DragonGeo 刻意不提供全局 epsilon 常量：单一阈值在坐标尺度相差数个
/// 数量级的输入上必然失效。容差作为参数显式传递，调用者始终知道
/// 自己在何种精度下工作。
///
/// 有效容差 = Abs + Rel * |magnitude|，即「绝对项」与「随量级增长的
/// 相对项」之和。
struct Tolerance {
    /// 绝对项，与量级无关的下限。
    double Abs = 1e-12;

    /// 相对项，随参考量级线性增长。
    double Rel = 1e-9;

    /// 给出参考量级对应的有效容差。
    [[nodiscard]] constexpr double Resolve(double magnitude) const noexcept {
        return Abs + Rel * AbsoluteValue(magnitude);
    }

    /// 判定两个标量在该容差下是否可视为相等。
    /// 参考量级取二者绝对值中的较大者。
    ///
    /// 非有限输入另行处理：完全相等（含 ±inf 与自身）返回 true，其余任何
    /// 涉及 ±inf 或 NaN 的组合一律返回 false。若不特判，`|inf - 5| <=
    /// Resolve(inf)` 会退化为 `inf <= inf` 而返回 true —— 无穷大被判成
    /// 「与任何有限值相等」，这与本类型的存在目的恰好相反。
    [[nodiscard]] constexpr bool Equal(double a, double b) const noexcept {
        if (a == b) {
            return true;
        }
        if (!IsFinite(a) || !IsFinite(b)) {
            return false;
        }
        const double scale = AbsoluteValue(a) > AbsoluteValue(b)
                                 ? AbsoluteValue(a)
                                 : AbsoluteValue(b);
        return AbsoluteValue(a - b) <= Resolve(scale);
    }

    /// 判定标量在该容差下是否可视为零。
    ///
    /// 注意参考量级取的是 x 自身，故判定条件等价于
    /// |x| <= Abs / (1 - Rel)，在 Rel 远小于 1 时约等于 Abs。
    ///
    /// 非有限值一律不视为零：把溢出成 ±inf 或变成 NaN 的量静默归类为
    /// 「可忽略」，正是调用者最需要被示警时却得到放行的情形。
    [[nodiscard]] constexpr bool IsZero(double x) const noexcept {
        if (!IsFinite(x)) {
            return false;
        }
        return AbsoluteValue(x) <= Resolve(x);
    }
};

} // namespace DragonGeo::Core
