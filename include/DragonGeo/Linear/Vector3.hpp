#pragma once

#include <array>
#include <concepts>
#include <cmath>
#include <optional>

#include <DragonGeo/Core/Numeric.hpp>
#include <DragonGeo/Core/Tolerance.hpp>

namespace DragonGeo::Linear {

template <typename Scalar> class UnitVector3T;

/// 三维向量：纯代数载体，不含任何几何语义。
template <typename Scalar>
struct Vector3T {
    using ScalarType = Scalar;

    // 同 Vector2T：不声明任何构造函数，以保持聚合性。
    Scalar X{};
    Scalar Y{};
    Scalar Z{};

    [[nodiscard]] constexpr Scalar LengthSquared() const noexcept {
        return X * X + Y * Y + Z * Z;
    }

    /// 欧几里得长度。先按最大分量缩放，避免中间量上溢或下溢。
    [[nodiscard]] Scalar Length() const noexcept {
        const Scalar scale = Core::MaxAbsOf(X, Y, Z);
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
        return scale * std::sqrt(scaledX * scaledX + scaledY * scaledY + scaledZ * scaledZ);
    }

    /// 下标访问。索引 0/1/2 依次对应 X/Y/Z。
    ///
    /// 越界是未定义行为 —— 与 std::array 一致，不做边界检查。
    [[nodiscard]] constexpr Scalar& operator[](int index) noexcept {
        return index == 0 ? X : (index == 1 ? Y : Z);
    }

    [[nodiscard]] constexpr const Scalar& operator[](int index) const noexcept {
        return index == 0 ? X : (index == 1 ? Y : Z);
    }

    /// 导出为数组，便于与外部库互操作。
    [[nodiscard]] constexpr std::array<Scalar, 3> ToArray() const noexcept {
        return {X, Y, Z};
    }

    /// 点积。
    [[nodiscard]] constexpr Scalar Dot(Vector3T other) const noexcept {
        return X * other.X + Y * other.Y + Z * other.Z;
    }

    /// 三维叉积。结果垂直于两个输入，方向遵循右手定则。
    /// 两向量平行（含任一为零向量）时结果为零向量。
    [[nodiscard]] constexpr Vector3T Cross(Vector3T other) const noexcept {
        return Vector3T{
            Y * other.Z - Z * other.Y,
            Z * other.X - X * other.Z,
            X * other.Y - Y * other.X,
        };
    }

    /// 归一化。零向量或退化向量返回 std::nullopt。
    ///
    /// 定义在 UnitVector3.hpp —— 返回类型 UnitVector3T 在那里才完整。
    /// 因此调用者不能只包含本头文件：UnitVector3T 不完整时，返回的
    /// std::optional 无法实例化。
    [[nodiscard]] std::optional<UnitVector3T<Scalar>> Normalized(
        Core::Tolerance tolerance = {}) const noexcept;
};

using Vector3 = Vector3T<double>;
using Vector3f = Vector3T<float>;

// ---- 运算符 ----

template <typename Scalar>
[[nodiscard]] constexpr Vector3T<Scalar> operator+(Vector3T<Scalar> a, Vector3T<Scalar> b) noexcept {
    return Vector3T<Scalar>{a.X + b.X, a.Y + b.Y, a.Z + b.Z};
}

template <typename Scalar>
[[nodiscard]] constexpr Vector3T<Scalar> operator-(Vector3T<Scalar> v) noexcept {
    return Vector3T<Scalar>{-v.X, -v.Y, -v.Z};
}

template <typename Scalar>
[[nodiscard]] constexpr Vector3T<Scalar> operator-(Vector3T<Scalar> a, Vector3T<Scalar> b) noexcept {
    return Vector3T<Scalar>{a.X - b.X, a.Y - b.Y, a.Z - b.Z};
}

template <typename Scalar, typename Factor>
    requires std::convertible_to<Factor, Scalar>
[[nodiscard]] constexpr Vector3T<Scalar> operator*(Vector3T<Scalar> v, Factor factor) noexcept {
    const auto scale = static_cast<Scalar>(factor);
    return Vector3T<Scalar>{v.X * scale, v.Y * scale, v.Z * scale};
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
    return Vector3T<Scalar>{v.X / scale, v.Y / scale, v.Z / scale};
}

template <typename Scalar>
[[nodiscard]] constexpr bool operator==(Vector3T<Scalar> a, Vector3T<Scalar> b) noexcept {
    return a.X == b.X && a.Y == b.Y && a.Z == b.Z;
}

} // namespace DragonGeo::Linear
