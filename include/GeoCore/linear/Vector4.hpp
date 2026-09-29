#pragma once

#include <concepts>
#include <cmath>

#include <GeoCore/core/Numeric.hpp>

namespace GeoCore::linear {

/// 四维向量：纯代数载体。齐次坐标与四元数的底层表示都用它，
/// 因此本层刻意不赋予它任何几何语义。
template <typename Scalar>
struct Vector4T {
    using scalar_type = Scalar;

    // 同 Vector2T：不声明任何构造函数，以保持聚合性。
    Scalar x{};
    Scalar y{};
    Scalar z{};
    Scalar w{};

    [[nodiscard]] constexpr Scalar length_squared() const noexcept {
        return x * x + y * y + z * z + w * w;
    }

    /// 欧几里得长度。先按最大分量缩放，避免中间量上溢或下溢。
    [[nodiscard]] Scalar length() const noexcept {
        const Scalar scale = core::max_abs_of(x, y, z, w);
        if (scale == Scalar{0}) {
            return Scalar{0};
        }
        if (!core::is_finite(scale)) {
            // 同 Vector2T::length：规则是任一无穷分量 ⇒ ±inf；否则含 NaN ⇒ NaN。
            return scale;
        }
        const Scalar scaled_x = x / scale;
        const Scalar scaled_y = y / scale;
        const Scalar scaled_z = z / scale;
        const Scalar scaled_w = w / scale;
        return scale * std::sqrt(scaled_x * scaled_x + scaled_y * scaled_y
                                 + scaled_z * scaled_z + scaled_w * scaled_w);
    }
};

using Vector4 = Vector4T<double>;
using Vector4f = Vector4T<float>;

// ---- 运算符 ----

template <typename Scalar>
[[nodiscard]] constexpr Vector4T<Scalar> operator+(Vector4T<Scalar> a, Vector4T<Scalar> b) noexcept {
    return Vector4T<Scalar>{a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w};
}

template <typename Scalar>
[[nodiscard]] constexpr Vector4T<Scalar> operator-(Vector4T<Scalar> v) noexcept {
    return Vector4T<Scalar>{-v.x, -v.y, -v.z, -v.w};
}

template <typename Scalar>
[[nodiscard]] constexpr Vector4T<Scalar> operator-(Vector4T<Scalar> a, Vector4T<Scalar> b) noexcept {
    return Vector4T<Scalar>{a.x - b.x, a.y - b.y, a.z - b.z, a.w - b.w};
}

template <typename Scalar, typename Factor>
    requires std::convertible_to<Factor, Scalar>
[[nodiscard]] constexpr Vector4T<Scalar> operator*(Vector4T<Scalar> v, Factor factor) noexcept {
    const auto scale = static_cast<Scalar>(factor);
    return Vector4T<Scalar>{v.x * scale, v.y * scale, v.z * scale, v.w * scale};
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
    return Vector4T<Scalar>{v.x / scale, v.y / scale, v.z / scale, v.w / scale};
}

template <typename Scalar>
[[nodiscard]] constexpr bool operator==(Vector4T<Scalar> a, Vector4T<Scalar> b) noexcept {
    return a.x == b.x && a.y == b.y && a.z == b.z && a.w == b.w;
}

// ---- 几何量 ----

template <typename Scalar>
[[nodiscard]] constexpr Scalar dot(Vector4T<Scalar> a, Vector4T<Scalar> b) noexcept {
    return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
}

} // namespace GeoCore::linear
