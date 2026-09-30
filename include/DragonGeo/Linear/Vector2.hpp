#pragma once

#include <array>
#include <concepts>
#include <cmath>
#include <optional>

#include <DragonGeo/Core/Numeric.hpp>
#include <DragonGeo/Core/Tolerance.hpp>

namespace DragonGeo::Linear {

template <typename Scalar> struct UnitVector2T;

/// 二维向量：纯代数载体，不含任何几何语义。
///
/// 与 Point2 的区别是语义而非存储 —— 两个点相加没有意义，因此
/// 类型系统不允许它。（Point2 与本类型同处 linear 层。）
template <typename Scalar>
struct Vector2T {
    using ScalarType = Scalar;

    // 刻意不声明任何构造函数。C++20 起「用户声明的构造函数」——哪怕只是
    // `= default` —— 都会让类型不再是聚合，进而使 `Vector2{3.0, 4.0}` 这类
    // 聚合初始化失效。默认成员初始化器已经提供了零初始化，无需额外构造函数。
    Scalar X{};
    Scalar Y{};

    /// 零向量，各分量都是 0。加法单位元，与值初始化的向量相同。
    /// 类型在定义内部不完整，常量定义在类型之后。
    static const Vector2T Zero;

    [[nodiscard]] constexpr Scalar LengthSquared() const noexcept {
        return X * X + Y * Y;
    }

    /// 欧几里得长度。
    ///
    /// 先按最大分量缩放再求平方根，因此分量在 1e-200 或 1e200 这类
    /// 极端量级上都不会因中间量下溢/上溢而丢失精度。代价是两次除法，
    /// 热路径上若只需要比较长度请改用 LengthSquared()。
    [[nodiscard]] Scalar Length() const noexcept {
        const Scalar scale = Core::MaxAbsOf(X, Y);
        if (scale == Scalar{0}) {
            return Scalar{0};
        }
        if (!Core::IsFinite(scale)) {
            // 含 ±inf 分量时下面的 inf / inf 会算出 NaN 并污染结果 —— 缩放本是
            // 为消除溢出而引入，不能反而在无穷输入上退化。缩放系数由
            // MaxAbsOf 给出，规则是：任一无穷分量 ⇒ ±inf；否则含 NaN ⇒ NaN。
            return scale;
        }
        const Scalar scaledX = X / scale;
        const Scalar scaledY = Y / scale;
        return scale * std::sqrt(scaledX * scaledX + scaledY * scaledY);
    }

    /// 下标访问。索引 0/1 依次对应 X/Y。
    ///
    /// 越界是未定义行为 —— 与 std::array 一致，不做边界检查。
    [[nodiscard]] constexpr Scalar& operator[](int index) noexcept {
        return index == 0 ? X : Y;
    }

    [[nodiscard]] constexpr const Scalar& operator[](int index) const noexcept {
        return index == 0 ? X : Y;
    }

    /// 导出为数组，便于与外部库互操作。
    [[nodiscard]] constexpr std::array<Scalar, 2> ToArray() const noexcept {
        return {X, Y};
    }

    /// 点积。
    [[nodiscard]] constexpr Scalar Dot(Vector2T other) const noexcept {
        return X * other.X + Y * other.Y;
    }

    /// 二维叉积，返回标量（有向面积的两倍再取半，即 z 分量）。
    /// 正值表示 other 在 this 的逆时针一侧。
    [[nodiscard]] constexpr Scalar Cross(Vector2T other) const noexcept {
        return X * other.Y - Y * other.X;
    }

    /// 归一化。零向量或退化向量返回 std::nullopt。
    ///
    /// 定义在 UnitVector2.hpp —— 返回类型 UnitVector2T 在那里才完整。
    /// 因此调用者不能只包含本头文件：UnitVector2T 不完整时，返回的
    /// std::optional 无法实例化。
    [[nodiscard]] std::optional<UnitVector2T<Scalar>> Normalized(
        Core::Tolerance tolerance = {}) const noexcept;
};

template <typename Scalar>
const Vector2T<Scalar> Vector2T<Scalar>::Zero{};

using Vector2 = Vector2T<double>;
using Vector2f = Vector2T<float>;

// ---- 运算符 ----

template <typename Scalar>
[[nodiscard]] constexpr Vector2T<Scalar> operator+(Vector2T<Scalar> a, Vector2T<Scalar> b) noexcept {
    return Vector2T<Scalar>{a.X + b.X, a.Y + b.Y};
}

template <typename Scalar>
[[nodiscard]] constexpr Vector2T<Scalar> operator-(Vector2T<Scalar> v) noexcept {
    return Vector2T<Scalar>{-v.X, -v.Y};
}

template <typename Scalar>
[[nodiscard]] constexpr Vector2T<Scalar> operator-(Vector2T<Scalar> a, Vector2T<Scalar> b) noexcept {
    return Vector2T<Scalar>{a.X - b.X, a.Y - b.Y};
}

/// 向量 × 标量。因子接受任意可转换为 Scalar 的算术类型，
/// 因此 `v * 3` 与 `v * 3.0` 都成立；传向量进去会被约束拒绝。
template <typename Scalar, typename Factor>
    requires std::convertible_to<Factor, Scalar>
[[nodiscard]] constexpr Vector2T<Scalar> operator*(Vector2T<Scalar> v, Factor factor) noexcept {
    const auto scale = static_cast<Scalar>(factor);
    return Vector2T<Scalar>{v.X * scale, v.Y * scale};
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
    return Vector2T<Scalar>{v.X / scale, v.Y / scale};
}

template <typename Scalar>
[[nodiscard]] constexpr bool operator==(Vector2T<Scalar> a, Vector2T<Scalar> b) noexcept {
    return a.X == b.X && a.Y == b.Y;
}
// C++20 由 operator== 自动生成 operator!=，无需手写。

} // namespace DragonGeo::Linear
