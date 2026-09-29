#pragma once

#include <concepts>
#include <optional>

#include <GeoCore/core/Numeric.hpp>
#include <GeoCore/core/Tolerance.hpp>
#include <GeoCore/linear/Vector2.hpp>

namespace GeoCore::linear {

/// 单位向量的二维类型。语义与 UnitVector3T 完全一致，
/// 差异仅在维度与二维叉积返回标量。
template <typename Scalar>
class UnitVector2T {
public:
    using scalar_type = Scalar;

    /// 前置条件：normalized 已是单位向量。命名即警告。
    [[nodiscard]] static constexpr UnitVector2T from_normalized_unchecked(
        Vector2T<Scalar> normalized) noexcept {
        return UnitVector2T{normalized};
    }

    [[nodiscard]] constexpr Vector2T<Scalar> as_vector() const noexcept {
        return value_;
    }

    [[nodiscard]] constexpr Scalar x() const noexcept { return value_.x; }
    [[nodiscard]] constexpr Scalar y() const noexcept { return value_.y; }

    [[nodiscard]] constexpr Scalar dot(UnitVector2T other) const noexcept {
        return value_.dot(other.value_);
    }

    /// 二维叉积，即两单位向量夹角的正弦。
    [[nodiscard]] constexpr Scalar cross(UnitVector2T other) const noexcept {
        return value_.cross(other.value_);
    }

    [[nodiscard]] constexpr bool operator==(const UnitVector2T&) const noexcept = default;

private:
    explicit constexpr UnitVector2T(Vector2T<Scalar> value) noexcept : value_(value) {}

    Vector2T<Scalar> value_;
};

using UnitVector2 = UnitVector2T<double>;
using UnitVector2f = UnitVector2T<float>;

template <typename Scalar>
[[nodiscard]] constexpr UnitVector2T<Scalar> operator-(UnitVector2T<Scalar> u) noexcept {
    return UnitVector2T<Scalar>::from_normalized_unchecked(-u.as_vector());
}

template <typename Scalar, typename Factor>
    requires std::convertible_to<Factor, Scalar>
[[nodiscard]] constexpr Vector2T<Scalar> operator*(UnitVector2T<Scalar> u, Factor factor) noexcept {
    return u.as_vector() * factor;
}

template <typename Scalar, typename Factor>
    requires std::convertible_to<Factor, Scalar>
[[nodiscard]] constexpr Vector2T<Scalar> operator*(Factor factor, UnitVector2T<Scalar> u) noexcept {
    return u.as_vector() * factor;
}

template <typename Scalar>
[[nodiscard]] constexpr Vector2T<Scalar> operator+(
    UnitVector2T<Scalar> a, UnitVector2T<Scalar> b) noexcept {
    return a.as_vector() + b.as_vector();
}

template <typename Scalar>
[[nodiscard]] constexpr Vector2T<Scalar> operator-(
    UnitVector2T<Scalar> a, UnitVector2T<Scalar> b) noexcept {
    return a.as_vector() - b.as_vector();
}

template <typename Scalar>
[[nodiscard]] std::optional<UnitVector2T<Scalar>> Vector2T<Scalar>::normalized(
    core::Tolerance tolerance) const noexcept {
    const Scalar length = this->length();
    // 非有限长度同样返回 nullopt，理由见 Vector3T::normalized。
    if (!core::is_finite(length) || tolerance.is_zero(static_cast<double>(length))) {
        return std::nullopt;
    }
    return UnitVector2T<Scalar>::from_normalized_unchecked(*this / length);
}

} // namespace GeoCore::linear
