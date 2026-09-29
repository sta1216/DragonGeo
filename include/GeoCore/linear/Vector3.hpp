#pragma once

#include <array>
#include <concepts>
#include <cmath>
#include <optional>

#include <GeoCore/core/Numeric.hpp>
#include <GeoCore/core/Tolerance.hpp>

namespace GeoCore::linear {

template <typename Scalar> class UnitVector3T;

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

    /// 下标访问。索引 0/1/2 依次对应 x/y/z。
    ///
    /// 越界是未定义行为 —— 与 std::array 一致，不做边界检查。
    [[nodiscard]] constexpr Scalar& operator[](int index) noexcept {
        return index == 0 ? x : (index == 1 ? y : z);
    }

    [[nodiscard]] constexpr const Scalar& operator[](int index) const noexcept {
        return index == 0 ? x : (index == 1 ? y : z);
    }

    /// 导出为数组，便于与外部库互操作。
    [[nodiscard]] constexpr std::array<Scalar, 3> to_array() const noexcept {
        return {x, y, z};
    }

    /// 点积。
    [[nodiscard]] constexpr Scalar dot(Vector3T other) const noexcept {
        return x * other.x + y * other.y + z * other.z;
    }

    /// 三维叉积。结果垂直于两个输入，方向遵循右手定则。
    /// 两向量平行（含任一为零向量）时结果为零向量。
    [[nodiscard]] constexpr Vector3T cross(Vector3T other) const noexcept {
        return Vector3T{
            y * other.z - z * other.y,
            z * other.x - x * other.z,
            x * other.y - y * other.x,
        };
    }

    /// 归一化。零向量或退化向量返回 std::nullopt。
    ///
    /// 定义在 UnitVector3.hpp —— 返回类型 UnitVector3T 在那里才完整。
    /// 因此调用者不能只包含本头文件：UnitVector3T 不完整时，返回的
    /// std::optional 无法实例化。
    [[nodiscard]] std::optional<UnitVector3T<Scalar>> normalized(
        core::Tolerance tolerance = {}) const noexcept;
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

} // namespace GeoCore::linear
