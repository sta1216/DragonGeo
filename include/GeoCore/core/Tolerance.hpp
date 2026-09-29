#pragma once

#include <GeoCore/core/Numeric.hpp>

namespace GeoCore::core {

/// 显式的容差模型。
///
/// GeoCore 刻意不提供全局 epsilon 常量：单一阈值在坐标尺度相差数个
/// 数量级的输入上必然失效。容差作为参数显式传递，调用者始终知道
/// 自己在何种精度下工作。
///
/// 有效容差 = abs + rel * |magnitude|，即「绝对项」与「随量级增长的
/// 相对项」之和。
struct Tolerance {
    /// 绝对项，与量级无关的下限。
    double abs = 1e-12;

    /// 相对项，随参考量级线性增长。
    double rel = 1e-9;

    /// 给出参考量级对应的有效容差。
    [[nodiscard]] constexpr double resolve(double magnitude) const noexcept {
        return abs + rel * absolute_value(magnitude);
    }

    /// 判定两个标量在该容差下是否可视为相等。
    /// 参考量级取二者绝对值中的较大者。
    [[nodiscard]] constexpr bool equal(double a, double b) const noexcept {
        const double scale = absolute_value(a) > absolute_value(b)
                                 ? absolute_value(a)
                                 : absolute_value(b);
        return absolute_value(a - b) <= resolve(scale);
    }

    /// 判定标量在该容差下是否可视为零。
    ///
    /// 注意参考量级取的是 x 自身，故判定条件等价于
    /// |x| <= abs / (1 - rel)，在 rel 远小于 1 时约等于 abs。
    [[nodiscard]] constexpr bool is_zero(double x) const noexcept {
        return absolute_value(x) <= resolve(x);
    }
};

} // namespace GeoCore::core
