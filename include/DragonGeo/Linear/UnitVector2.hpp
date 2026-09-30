#pragma once

#include <concepts>
#include <optional>

#include <DragonGeo/Core/Numeric.hpp>
#include <DragonGeo/Core/Tolerance.hpp>
#include <DragonGeo/Linear/Vector2.hpp>

namespace DragonGeo::Linear {

/// 单位向量的二维类型。语义与 UnitVector3T 完全一致，
/// 差异仅在维度与二维叉积返回标量。
template <typename Scalar>
class UnitVector2T {
public:
    using ScalarType = Scalar;

    /// 前置条件：normalized 已是单位向量。命名即警告。
    [[nodiscard]] static constexpr UnitVector2T FromNormalizedUnchecked(
        Vector2T<Scalar> normalized) noexcept {
        return UnitVector2T{normalized};
    }

    [[nodiscard]] constexpr Vector2T<Scalar> AsVector() const noexcept {
        return m_value;
    }

    [[nodiscard]] constexpr Scalar X() const noexcept { return m_value.X; }
    [[nodiscard]] constexpr Scalar Y() const noexcept { return m_value.Y; }

    [[nodiscard]] constexpr Scalar Dot(UnitVector2T other) const noexcept {
        return m_value.Dot(other.m_value);
    }

    /// 二维叉积，即两单位向量夹角的正弦。
    [[nodiscard]] constexpr Scalar Cross(UnitVector2T other) const noexcept {
        return m_value.Cross(other.m_value);
    }

    [[nodiscard]] constexpr bool operator==(const UnitVector2T&) const noexcept = default;

private:
    explicit constexpr UnitVector2T(Vector2T<Scalar> value) noexcept : m_value(value) {}

    Vector2T<Scalar> m_value;
};

using UnitVector2 = UnitVector2T<double>;
using UnitVector2f = UnitVector2T<float>;

template <typename Scalar>
[[nodiscard]] constexpr UnitVector2T<Scalar> operator-(UnitVector2T<Scalar> u) noexcept {
    return UnitVector2T<Scalar>::FromNormalizedUnchecked(-u.AsVector());
}

template <typename Scalar, typename Factor>
    requires std::convertible_to<Factor, Scalar>
[[nodiscard]] constexpr Vector2T<Scalar> operator*(UnitVector2T<Scalar> u, Factor factor) noexcept {
    return u.AsVector() * factor;
}

template <typename Scalar, typename Factor>
    requires std::convertible_to<Factor, Scalar>
[[nodiscard]] constexpr Vector2T<Scalar> operator*(Factor factor, UnitVector2T<Scalar> u) noexcept {
    return u.AsVector() * factor;
}

template <typename Scalar>
[[nodiscard]] constexpr Vector2T<Scalar> operator+(
    UnitVector2T<Scalar> a, UnitVector2T<Scalar> b) noexcept {
    return a.AsVector() + b.AsVector();
}

template <typename Scalar>
[[nodiscard]] constexpr Vector2T<Scalar> operator-(
    UnitVector2T<Scalar> a, UnitVector2T<Scalar> b) noexcept {
    return a.AsVector() - b.AsVector();
}

template <typename Scalar>
[[nodiscard]] std::optional<UnitVector2T<Scalar>> Vector2T<Scalar>::Normalized(
    Core::Tolerance tolerance) const noexcept {
    const Scalar length = this->Length();
    // 非有限长度同样返回 nullopt，理由见 Vector3T::normalized。
    if (!Core::IsFinite(length) || tolerance.IsZero(static_cast<double>(length))) {
        return std::nullopt;
    }
    return UnitVector2T<Scalar>::FromNormalizedUnchecked(*this / length);
}

} // namespace DragonGeo::Linear
