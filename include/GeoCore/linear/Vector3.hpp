#pragma once

#include <concepts>
#include <cmath>

#include <GeoCore/core/Numeric.hpp>

namespace GeoCore::linear {

/// 三维向量：纯代数载体，不含任何几何语义。
template <typename Scalar>
struct Vector3T {
    using scalar_type = Scalar;

    // 同 Vector2T：不声明任何构造函数，以保持聚合性。
    Scalar x{};
    Scalar y{};
    Scalar z{};

    [[nodiscard]] constexpr Scalar length_squared() const noexcept {
        return x * x + y * y + z * z;
    }

    /// 欧几里得长度。先按最大分量缩放，避免中间量上溢或下溢。
    [[nodiscard]] Scalar length() const noexcept {
        const Scalar scale = core::max_abs_of(x, y, z);
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
        return scale * std::sqrt(scaled_x * scaled_x + scaled_y * scaled_y + scaled_z * scaled_z);
    }
};

using Vector3 = Vector3T<double>;
using Vector3f = Vector3T<float>;

// ---- 运算符 ----

template <typename Scalar>
[[nodiscard]] constexpr Vector3T<Scalar> operator+(Vector3T<Scalar> a, Vector3T<Scalar> b) noexcept {
    return Vector3T<Scalar>{a.x + b.x, a.y + b.y, a.z + b.z};
}

template <typename Scalar>
[[nodiscard]] constexpr Vector3T<Scalar> operator-(Vector3T<Scalar> v) noexcept {
    return Vector3T<Scalar>{-v.x, -v.y, -v.z};
}

template <typename Scalar>
[[nodiscard]] constexpr Vector3T<Scalar> operator-(Vector3T<Scalar> a, Vector3T<Scalar> b) noexcept {
    return Vector3T<Scalar>{a.x - b.x, a.y - b.y, a.z - b.z};
}

template <typename Scalar, typename Factor>
    requires std::convertible_to<Factor, Scalar>
[[nodiscard]] constexpr Vector3T<Scalar> operator*(Vector3T<Scalar> v, Factor factor) noexcept {
    const auto scale = static_cast<Scalar>(factor);
    return Vector3T<Scalar>{v.x * scale, v.y * scale, v.z * scale};
}

template <typename Scalar, typename Factor>
    requires std::convertible_to<Factor, Scalar>
[[nodiscard]] constexpr Vector3T<Scalar> operator*(Factor factor, Vector3T<Scalar> v) noexcept {
    return v * factor;
}

template <typename Scalar, typename Divisor>
    requires std::convertible_to<Divisor, Scalar>
[[nodiscard]] constexpr Vector3T<Scalar> operator/(Vector3T<Scalar> v, Divisor divisor) noexcept {
    const auto scale = static_cast<Scalar>(divisor);
    return Vector3T<Scalar>{v.x / scale, v.y / scale, v.z / scale};
}

template <typename Scalar>
[[nodiscard]] constexpr bool operator==(Vector3T<Scalar> a, Vector3T<Scalar> b) noexcept {
    return a.x == b.x && a.y == b.y && a.z == b.z;
}

// ---- 几何量 ----

template <typename Scalar>
[[nodiscard]] constexpr Scalar dot(Vector3T<Scalar> a, Vector3T<Scalar> b) noexcept {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

/// 三维叉积。结果垂直于两个输入，方向遵循右手定则。
/// 两向量平行（含任一为零向量）时结果为零向量。
template <typename Scalar>
[[nodiscard]] constexpr Vector3T<Scalar> cross(Vector3T<Scalar> a, Vector3T<Scalar> b) noexcept {
    return Vector3T<Scalar>{
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x,
    };
}

} // namespace GeoCore::linear
