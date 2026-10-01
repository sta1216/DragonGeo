#pragma once

#include <cmath>
#include <concepts>
#include <optional>

#include <DragonGeo/Core/Numeric.hpp>
#include <DragonGeo/Core/Tolerance.hpp>
#include <DragonGeo/Linear/Vector2.hpp>

namespace DragonGeo::Linear {

/// 单位向量的二维类型。语义与 UnitVector3T 完全一致，
/// 差异仅在维度与二维叉积返回标量。
template <typename Scalar>
struct UnitVector2T {
public:
    using ScalarType = Scalar;

    /// 前置条件：normalized 已是单位向量。命名即警告。
    [[nodiscard]] static constexpr UnitVector2T FromNormalizedUnchecked(
        Vector2T<Scalar> normalized) noexcept {
        return UnitVector2T{normalized};
    }

    /// 正 X 轴单位向量 `(1, 0)`。
    /// 类型在定义内部不完整，常量定义在类型之后。
    static const UnitVector2T XAxis;

    /// 正 Y 轴单位向量 `(0, 1)`。它是 `XAxis` 逆时针转 90° 的结果。
    static const UnitVector2T YAxis;

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

    /// 逆时针转 90° 得到的单位垂直向量，即 `(-Y, X)`。
    ///
    /// 与 `Coordinate2T::FromXAxis` 补出的 Y 轴相同。另一侧取相反数。
    /// 分量对调不改变长度，所以结果仍是单位向量。
    [[nodiscard]] constexpr UnitVector2T Perpendicular() const noexcept {
        return FromNormalizedUnchecked(Vector2T<Scalar>{-m_value.Y, m_value.X});
    }

    /// 把 `vector` 投到本方向上：`(vector · n) n`。
    ///
    /// 结果的长度是投影长度，一般不再是单位向量，故返回 `Vector2T`。
    [[nodiscard]] constexpr Vector2T<Scalar> Projected(Vector2T<Scalar> vector) const noexcept {
        return m_value * m_value.Dot(vector);
    }

    /// 与另一单位向量的夹角，范围 `[0, π]`。
    [[nodiscard]] Scalar AngleBetween(UnitVector2T other) const noexcept {
        double cosine = static_cast<double>(Dot(other));
        if (cosine > 1.0) {
            cosine = 1.0;
        } else if (cosine < -1.0) {
            cosine = -1.0;
        }
        return static_cast<Scalar>(std::acos(cosine));
    }

    /// 从本向量到 `other` 的有符号角，范围 `(-π, π]`，逆时针为正。
    [[nodiscard]] Scalar SignedAngle(UnitVector2T other) const noexcept {
        return static_cast<Scalar>(
            std::atan2(static_cast<double>(Cross(other)), static_cast<double>(Dot(other))));
    }

    [[nodiscard]] constexpr bool operator==(const UnitVector2T&) const noexcept = default;

private:
    explicit constexpr UnitVector2T(Vector2T<Scalar> value) noexcept : m_value(value) {}

    Vector2T<Scalar> m_value;
};

template <typename Scalar>
const UnitVector2T<Scalar> UnitVector2T<Scalar>::XAxis =
    UnitVector2T<Scalar>::FromNormalizedUnchecked(Vector2T<Scalar>{Scalar{1}, Scalar{0}});

template <typename Scalar>
const UnitVector2T<Scalar> UnitVector2T<Scalar>::YAxis =
    UnitVector2T<Scalar>::FromNormalizedUnchecked(Vector2T<Scalar>{Scalar{0}, Scalar{1}});

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
