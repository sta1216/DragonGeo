#pragma once

#include <concepts>
#include <cmath>

#include <GeoCore/core/Numeric.hpp>

namespace GeoCore::linear {

/// 二维向量：纯代数载体，不含任何几何语义。
///
/// 与 Point2 的区别是语义而非存储 —— 两个点相加没有意义，因此
/// 类型系统不允许它。（Point2 属于 prim 层。）
template <typename Scalar>
struct Vector2T {
    using scalar_type = Scalar;

    // 刻意不声明任何构造函数。C++20 起「用户声明的构造函数」——哪怕只是
    // `= default` —— 都会让类型不再是聚合，进而使 `Vector2{3.0, 4.0}` 这类
    // 聚合初始化失效。默认成员初始化器已经提供了零初始化，无需额外构造函数。
    Scalar x{};
    Scalar y{};

    [[nodiscard]] constexpr Scalar length_squared() const noexcept {
        return x * x + y * y;
    }

    /// 欧几里得长度。
    ///
    /// 先按最大分量缩放再求平方根，因此分量在 1e-200 或 1e200 这类
    /// 极端量级上都不会因中间量下溢/上溢而丢失精度。代价是两次除法，
    /// 热路径上若只需要比较长度请改用 length_squared()。
    [[nodiscard]] Scalar length() const noexcept {
        const Scalar abs_x = core::absolute_value(x);
        const Scalar abs_y = core::absolute_value(y);
        const Scalar scale = abs_x > abs_y ? abs_x : abs_y;
        if (scale == Scalar{0}) {
            return Scalar{0};
        }
        const Scalar scaled_x = x / scale;
        const Scalar scaled_y = y / scale;
        return scale * std::sqrt(scaled_x * scaled_x + scaled_y * scaled_y);
    }
};

using Vector2 = Vector2T<double>;
using Vector2f = Vector2T<float>;

// ---- 运算符 ----

template <typename Scalar>
[[nodiscard]] constexpr Vector2T<Scalar> operator+(Vector2T<Scalar> a, Vector2T<Scalar> b) noexcept {
    return Vector2T<Scalar>{a.x + b.x, a.y + b.y};
}

template <typename Scalar>
[[nodiscard]] constexpr Vector2T<Scalar> operator-(Vector2T<Scalar> v) noexcept {
    return Vector2T<Scalar>{-v.x, -v.y};
}

template <typename Scalar>
[[nodiscard]] constexpr Vector2T<Scalar> operator-(Vector2T<Scalar> a, Vector2T<Scalar> b) noexcept {
    return Vector2T<Scalar>{a.x - b.x, a.y - b.y};
}

/// 向量 × 标量。因子接受任意可转换为 Scalar 的算术类型，
/// 因此 `v * 3` 与 `v * 3.0` 都成立；传向量进去会被约束拒绝。
template <typename Scalar, typename Factor>
    requires std::convertible_to<Factor, Scalar>
[[nodiscard]] constexpr Vector2T<Scalar> operator*(Vector2T<Scalar> v, Factor factor) noexcept {
    const auto scale = static_cast<Scalar>(factor);
    return Vector2T<Scalar>{v.x * scale, v.y * scale};
}

template <typename Scalar, typename Factor>
    requires std::convertible_to<Factor, Scalar>
[[nodiscard]] constexpr Vector2T<Scalar> operator*(Factor factor, Vector2T<Scalar> v) noexcept {
    return v * factor;
}

template <typename Scalar, typename Divisor>
    requires std::convertible_to<Divisor, Scalar>
[[nodiscard]] constexpr Vector2T<Scalar> operator/(Vector2T<Scalar> v, Divisor divisor) noexcept {
    const auto scale = static_cast<Scalar>(divisor);
    return Vector2T<Scalar>{v.x / scale, v.y / scale};
}

template <typename Scalar>
[[nodiscard]] constexpr bool operator==(Vector2T<Scalar> a, Vector2T<Scalar> b) noexcept {
    return a.x == b.x && a.y == b.y;
}
// C++20 由 operator== 自动生成 operator!=，无需手写。

// ---- 几何量 ----

/// 点积。
template <typename Scalar>
[[nodiscard]] constexpr Scalar dot(Vector2T<Scalar> a, Vector2T<Scalar> b) noexcept {
    return a.x * b.x + a.y * b.y;
}

/// 二维叉积，返回标量（有向面积的两倍再取半，即 z 分量）。
/// 正值表示 b 在 a 的逆时针一侧。
template <typename Scalar>
[[nodiscard]] constexpr Scalar cross(Vector2T<Scalar> a, Vector2T<Scalar> b) noexcept {
    return a.x * b.y - a.y * b.x;
}

} // namespace GeoCore::linear
