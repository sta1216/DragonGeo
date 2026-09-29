#pragma once

#include <optional>

#include <GeoCore/core/Tolerance.hpp>
#include <GeoCore/linear/Vector3.hpp>

namespace GeoCore::linear {

/// 单位向量的三维类型。
///
/// 不变量是**弱不变量**：|v| 在浮点舍入误差内等于 1，而不是精确等于 1。
/// 它保证的是「已归一化过一次」，因此不能用作依赖精确单位长度的判定。
///
/// 运算的返回类型如实反映是否保持该不变量：
///   -u               -> UnitVector3T   （保持）
///   u * scalar       -> Vector3T       （缩放后不再是单位向量）
///   u + u            -> Vector3T       （和一般不是单位向量）
///   cross(u, u)      -> Vector3T       （平行时退化为零向量）
template <typename Scalar>
class UnitVector3T {
public:
    using scalar_type = Scalar;

    /// 前置条件：normalized 已是单位向量。命名即警告。
    ///
    /// 违反前置条件不会立即出错，但会让本类型的不变量永久失效，
    /// 后续依赖该不变量的代码将得到错误结果。请优先使用 normalize()。
    [[nodiscard]] static constexpr UnitVector3T from_normalized_unchecked(
        Vector3T<Scalar> normalized) noexcept {
        return UnitVector3T{normalized};
    }

    [[nodiscard]] constexpr Vector3T<Scalar> as_vector() const noexcept {
        return value_;
    }

    [[nodiscard]] constexpr Scalar x() const noexcept { return value_.x; }
    [[nodiscard]] constexpr Scalar y() const noexcept { return value_.y; }
    [[nodiscard]] constexpr Scalar z() const noexcept { return value_.z; }

    [[nodiscard]] constexpr bool operator==(const UnitVector3T&) const noexcept = default;

private:
    explicit constexpr UnitVector3T(Vector3T<Scalar> value) noexcept : value_(value) {}

    Vector3T<Scalar> value_;
};

using UnitVector3 = UnitVector3T<double>;
using UnitVector3f = UnitVector3T<float>;

/// 归一化。向量长度在给定容差下可视为零时返回 std::nullopt，
/// 因此调用者无法得到含 NaN 的单位向量。
template <typename Scalar>
[[nodiscard]] std::optional<UnitVector3T<Scalar>> normalize(
    Vector3T<Scalar> v, core::Tolerance tolerance = {}) noexcept {
    const Scalar length = v.length();
    if (tolerance.is_zero(static_cast<double>(length))) {
        return std::nullopt;
    }
    return UnitVector3T<Scalar>::from_normalized_unchecked(v / length);
}

// ---- 保持不变量的运算 ----

template <typename Scalar>
[[nodiscard]] constexpr UnitVector3T<Scalar> operator-(UnitVector3T<Scalar> u) noexcept {
    return UnitVector3T<Scalar>::from_normalized_unchecked(-u.as_vector());
}

// ---- 不保持不变量的运算：返回类型相应改变 ----

template <typename Scalar, typename Factor>
    requires std::convertible_to<Factor, Scalar>
[[nodiscard]] constexpr Vector3T<Scalar> operator*(UnitVector3T<Scalar> u, Factor factor) noexcept {
    return u.as_vector() * factor;
}

template <typename Scalar, typename Factor>
    requires std::convertible_to<Factor, Scalar>
[[nodiscard]] constexpr Vector3T<Scalar> operator*(Factor factor, UnitVector3T<Scalar> u) noexcept {
    return u.as_vector() * factor;
}

template <typename Scalar>
[[nodiscard]] constexpr Vector3T<Scalar> operator+(
    UnitVector3T<Scalar> a, UnitVector3T<Scalar> b) noexcept {
    return a.as_vector() + b.as_vector();
}

template <typename Scalar>
[[nodiscard]] constexpr Vector3T<Scalar> operator-(
    UnitVector3T<Scalar> a, UnitVector3T<Scalar> b) noexcept {
    return a.as_vector() - b.as_vector();
}

template <typename Scalar>
[[nodiscard]] constexpr Scalar dot(UnitVector3T<Scalar> a, UnitVector3T<Scalar> b) noexcept {
    return dot(a.as_vector(), b.as_vector());
}

/// 叉积。结果不保证是单位向量（两向量平行时为零向量），故返回 Vector3T。
template <typename Scalar>
[[nodiscard]] constexpr Vector3T<Scalar> cross(UnitVector3T<Scalar> a, UnitVector3T<Scalar> b) noexcept {
    return cross(a.as_vector(), b.as_vector());
}

} // namespace GeoCore::linear
