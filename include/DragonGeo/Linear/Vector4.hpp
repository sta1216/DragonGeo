#pragma once

#include <array>
#include <concepts>
#include <cmath>

#include <DragonGeo/Core/Numeric.hpp>

namespace DragonGeo::Linear {

/// 四维向量：纯代数载体。齐次坐标与四元数的底层表示都用它，
/// 因此本层刻意不赋予它任何几何语义。
template <typename Scalar>
struct Vector4T {
    using ScalarType = Scalar;

    // 同 Vector2T：不声明任何构造函数，以保持聚合性。
    Scalar X{};
    Scalar Y{};
    Scalar Z{};
    Scalar W{};

    [[nodiscard]] constexpr Scalar LengthSquared() const noexcept {
        return X * X + Y * Y + Z * Z + W * W;
    }

    /// 欧几里得长度。先按最大分量缩放，避免中间量上溢或下溢。
    [[nodiscard]] Scalar Length() const noexcept {
        const Scalar scale = Core::MaxAbsOf(X, Y, Z, W);
        if (scale == Scalar{0}) {
            return Scalar{0};
        }
        if (!Core::IsFinite(scale)) {
            // 同 Vector2T::Length：规则是任一无穷分量 ⇒ ±inf；否则含 NaN ⇒ NaN。
            return scale;
        }
        const Scalar scaledX = X / scale;
        const Scalar scaledY = Y / scale;
        const Scalar scaledZ = Z / scale;
        const Scalar scaledW = W / scale;
        return scale * std::sqrt(scaledX * scaledX + scaledY * scaledY
                                 + scaledZ * scaledZ + scaledW * scaledW);
    }

    /// 下标访问。索引 0/1/2/3 依次对应 X/Y/Z/W。
    ///
    /// 越界是未定义行为 —— 与 std::array 一致，不做边界检查。
    [[nodiscard]] constexpr Scalar& operator[](int index) noexcept {
        return index == 0 ? X : (index == 1 ? Y : (index == 2 ? Z : W));
    }

    [[nodiscard]] constexpr const Scalar& operator[](int index) const noexcept {
        return index == 0 ? X : (index == 1 ? Y : (index == 2 ? Z : W));
    }

    /// 导出为数组，便于与外部库互操作。
    [[nodiscard]] constexpr std::array<Scalar, 4> ToArray() const noexcept {
        return {X, Y, Z, W};
    }

    /// 点积。四维没有叉积，本层也不存在 UnitVector4T，故没有 normalized。
    [[nodiscard]] constexpr Scalar Dot(Vector4T other) const noexcept {
        return X * other.X + Y * other.Y + Z * other.Z + W * other.W;
    }
};

using Vector4 = Vector4T<double>;
using Vector4f = Vector4T<float>;

// ---- 运算符 ----

template <typename Scalar>
[[nodiscard]] constexpr Vector4T<Scalar> operator+(Vector4T<Scalar> a, Vector4T<Scalar> b) noexcept {
    return Vector4T<Scalar>{a.X + b.X, a.Y + b.Y, a.Z + b.Z, a.W + b.W};
}

template <typename Scalar>
[[nodiscard]] constexpr Vector4T<Scalar> operator-(Vector4T<Scalar> v) noexcept {
    return Vector4T<Scalar>{-v.X, -v.Y, -v.Z, -v.W};
}

template <typename Scalar>
[[nodiscard]] constexpr Vector4T<Scalar> operator-(Vector4T<Scalar> a, Vector4T<Scalar> b) noexcept {
    return Vector4T<Scalar>{a.X - b.X, a.Y - b.Y, a.Z - b.Z, a.W - b.W};
}

template <typename Scalar, typename Factor>
    requires std::convertible_to<Factor, Scalar>
[[nodiscard]] constexpr Vector4T<Scalar> operator*(Vector4T<Scalar> v, Factor factor) noexcept {
    const auto scale = static_cast<Scalar>(factor);
    return Vector4T<Scalar>{v.X * scale, v.Y * scale, v.Z * scale, v.W * scale};
}

template <typename Scalar, typename Factor>
    requires std::convertible_to<Factor, Scalar>
[[nodiscard]] constexpr Vector4T<Scalar> operator*(Factor factor, Vector4T<Scalar> v) noexcept {
    return v * factor;
}

template <typename Scalar, typename Divisor>
    requires std::convertible_to<Divisor, Scalar>
[[nodiscard]] constexpr Vector4T<Scalar> operator/(Vector4T<Scalar> v, Divisor divisor) noexcept {
    const auto scale = static_cast<Scalar>(divisor);
    return Vector4T<Scalar>{v.X / scale, v.Y / scale, v.Z / scale, v.W / scale};
}

template <typename Scalar>
[[nodiscard]] constexpr bool operator==(Vector4T<Scalar> a, Vector4T<Scalar> b) noexcept {
    return a.X == b.X && a.Y == b.Y && a.Z == b.Z && a.W == b.W;
}

} // namespace DragonGeo::Linear
