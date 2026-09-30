#pragma once

#include <concepts>
#include <optional>

#include <DragonGeo/Core/Numeric.hpp>
#include <DragonGeo/Core/Tolerance.hpp>
#include <DragonGeo/Linear/Vector3.hpp>

namespace DragonGeo::Linear {

/// 单位向量的三维类型。
///
/// 不变量是**弱不变量**：|v| 在浮点舍入误差内等于 1，而不是精确等于 1。
/// 它保证的是「已归一化过一次」，因此不能用作依赖精确单位长度的判定。
///
/// 运算的返回类型如实反映是否保持该不变量：
///   -u               -> UnitVector3T   （保持）
///   u * scalar       -> Vector3T       （缩放后不再是单位向量）
///   u + u            -> Vector3T       （和一般不是单位向量）
///   u.Cross(u)       -> Vector3T       （平行时退化为零向量）
template <typename Scalar>
class UnitVector3T {
public:
    using ScalarType = Scalar;

    /// 前置条件：normalized 已是单位向量。命名即警告。
    ///
    /// 违反前置条件不会立即出错，但会让本类型的不变量永久失效，
    /// 后续依赖该不变量的代码将得到错误结果。请优先使用 Vector3T::Normalized()。
    [[nodiscard]] static constexpr UnitVector3T FromNormalizedUnchecked(
        Vector3T<Scalar> normalized) noexcept {
        return UnitVector3T{normalized};
    }

    [[nodiscard]] constexpr Vector3T<Scalar> AsVector() const noexcept {
        return m_value;
    }

    [[nodiscard]] constexpr Scalar X() const noexcept { return m_value.X; }
    [[nodiscard]] constexpr Scalar Y() const noexcept { return m_value.Y; }
    [[nodiscard]] constexpr Scalar Z() const noexcept { return m_value.Z; }

    [[nodiscard]] constexpr Scalar Dot(UnitVector3T other) const noexcept {
        return m_value.Dot(other.m_value);
    }

    /// 叉积。结果不保证是单位向量（两向量平行时为零向量），故返回 Vector3T。
    [[nodiscard]] constexpr Vector3T<Scalar> Cross(UnitVector3T other) const noexcept {
        return m_value.Cross(other.m_value);
    }

    [[nodiscard]] constexpr bool operator==(const UnitVector3T&) const noexcept = default;

private:
    explicit constexpr UnitVector3T(Vector3T<Scalar> value) noexcept : m_value(value) {}

    Vector3T<Scalar> m_value;
};

using UnitVector3 = UnitVector3T<double>;
using UnitVector3f = UnitVector3T<float>;

// ---- 保持不变量的运算 ----

template <typename Scalar>
[[nodiscard]] constexpr UnitVector3T<Scalar> operator-(UnitVector3T<Scalar> u) noexcept {
    return UnitVector3T<Scalar>::FromNormalizedUnchecked(-u.AsVector());
}

// ---- 不保持不变量的运算：返回类型相应改变 ----

template <typename Scalar, typename Factor>
    requires std::convertible_to<Factor, Scalar>
[[nodiscard]] constexpr Vector3T<Scalar> operator*(UnitVector3T<Scalar> u, Factor factor) noexcept {
    return u.AsVector() * factor;
}

template <typename Scalar, typename Factor>
    requires std::convertible_to<Factor, Scalar>
[[nodiscard]] constexpr Vector3T<Scalar> operator*(Factor factor, UnitVector3T<Scalar> u) noexcept {
    return u.AsVector() * factor;
}

template <typename Scalar>
[[nodiscard]] constexpr Vector3T<Scalar> operator+(
    UnitVector3T<Scalar> a, UnitVector3T<Scalar> b) noexcept {
    return a.AsVector() + b.AsVector();
}

template <typename Scalar>
[[nodiscard]] constexpr Vector3T<Scalar> operator-(
    UnitVector3T<Scalar> a, UnitVector3T<Scalar> b) noexcept {
    return a.AsVector() - b.AsVector();
}

template <typename Scalar>
[[nodiscard]] std::optional<UnitVector3T<Scalar>> Vector3T<Scalar>::Normalized(
    Core::Tolerance tolerance) const noexcept {
    const Scalar length = this->Length();
    // 非有限长度同样返回 nullopt。容差判断对 ±inf 与 NaN 一律返回 false
    // （`IsZero` 刻意不把溢出量静默归类为零），若就此放行，本函数会交出一个
    // has_value() 为真、内容却是 NaN 的「单位向量」：调用者无从察觉，而 NaN
    // 会一路污染 dot / cross 与每一个容差比较 —— 那些比较对 NaN 都返回 false，
    // 下游几何代码会静默走「否」分支。NaN 输入比无穷更常见：任何上游的
    // 0/0 或 inf - inf 都会落到这里。
    if (!Core::IsFinite(length) || tolerance.IsZero(static_cast<double>(length))) {
        return std::nullopt;
    }
    return UnitVector3T<Scalar>::FromNormalizedUnchecked(*this / length);
}

} // namespace DragonGeo::Linear
