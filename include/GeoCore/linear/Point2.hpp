#pragma once

#include <array>

#include <GeoCore/linear/Vector2.hpp>

namespace GeoCore::linear {

/// 二维点：位置类型，与 Vector2T 同底层存储、不同类型语义。
///
/// 与 Vector2T 的区别是语义而非存储。点没有加法，也没有零元 —— 两个点相加没有
/// 意义，因此 `Point2 + Point2` 刻意不提供，类型系统会拒绝它。有意义的运算只有
/// 三种：点 + 向量（平移）、点 - 向量（反向平移）、点 - 点（位移向量）。
template <typename Scalar>
struct Point2T {
    using scalar_type = Scalar;

    // 同 Vector2T：不声明任何构造函数，以保持聚合性。
    Scalar x{};
    Scalar y{};

    /// 下标访问。索引 0/1 依次对应 x/y。
    ///
    /// 越界是未定义行为 —— 与 std::array 一致，不做边界检查。
    [[nodiscard]] constexpr Scalar& operator[](int index) noexcept {
        return index == 0 ? x : y;
    }

    [[nodiscard]] constexpr const Scalar& operator[](int index) const noexcept {
        return index == 0 ? x : y;
    }

    /// 导出为数组，便于与外部库互操作。
    [[nodiscard]] constexpr std::array<Scalar, 2> to_array() const noexcept {
        return {x, y};
    }

    /// 两点间的欧几里得距离。
    ///
    /// 经 Vector2T::length() 实现，因此继承它的一切行为 —— 包括**差向量**的任一
    /// 分量含 ±inf 时返回 inf 而非 NaN。那正是长度的既定语义，不为此加特判。
    ///
    /// 限定语「**差向量**」不可省：两个点在同一位上都是同一个 ±inf、**且其余
    /// 槽位的差都是有限值**时，差向量含 NaN 而不含无穷，距离随之是 NaN ——
    /// 与 `length()` 的规则一致（任一无穷分量 ⇒ ±inf；否则含 NaN ⇒ NaN），
    /// 不是漏判。
    ///
    /// 「其余槽位有限」这个附加条件不能省：若别的槽位差为 ±inf，那条规则会让
    /// inf 压过 NaN，距离是 inf —— 例如 `{+inf, 0}` 到 `{+inf, +inf}` 是 inf，
    /// 不是 NaN。
    [[nodiscard]] Scalar distance_to(Point2T other) const noexcept {
        return (*this - other).length();
    }
};

using Point2 = Point2T<double>;
using Point2f = Point2T<float>;

// ---- 运算符 ----

/// 点沿向量平移。
///
/// 刻意没有 `Point + Point`：点没有加法。缺了它编译期就会报错，而不是静默
/// 给出错误语义。
template <typename Scalar>
[[nodiscard]] constexpr Point2T<Scalar> operator+(Point2T<Scalar> p, Vector2T<Scalar> v) noexcept {
    return Point2T<Scalar>{p.x + v.x, p.y + v.y};
}

/// 点沿向量反方向平移。
template <typename Scalar>
[[nodiscard]] constexpr Point2T<Scalar> operator-(Point2T<Scalar> p, Vector2T<Scalar> v) noexcept {
    return Point2T<Scalar>{p.x - v.x, p.y - v.y};
}

/// 两点之差是位移向量，不是点。
template <typename Scalar>
[[nodiscard]] constexpr Vector2T<Scalar> operator-(Point2T<Scalar> a, Point2T<Scalar> b) noexcept {
    return Vector2T<Scalar>{a.x - b.x, a.y - b.y};
}

/// 逐分量比较。与 Vector 的约定一致：`operator!=` 由 C++20 自动生成，不手写。
template <typename Scalar>
[[nodiscard]] constexpr bool operator==(Point2T<Scalar> a, Point2T<Scalar> b) noexcept {
    return a.x == b.x && a.y == b.y;
}

} // namespace GeoCore::linear
