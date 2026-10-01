#pragma once

#include <array>

#include <DragonGeo/Linear/Vector2.hpp>

namespace DragonGeo::Linear {

/// 二维点：位置类型，与 Vector2T 同底层存储、不同类型语义。
///
/// 与 Vector2T 的区别是语义而非存储。点没有加法，因此没有加法零元 —— 两个点相加没有
/// 意义，因此 `Point2 + Point2` 刻意不提供，类型系统会拒绝它。有意义的运算只有
/// 三种：点 ± 向量（平移，反向书写 `向量 + 点` 同样成立）、点 - 点（位移向量）。
template <typename Scalar>
struct Point2T {
    using ScalarType = Scalar;

    // 同 Vector2T：不声明任何构造函数，以保持聚合性。
    Scalar X{};
    Scalar Y{};

    /// 原点，两个分量都是 0。名称与零向量的 `Zero` 一致。
    /// 与值初始化的点相同。点没有加法，它不是加法零元。
    /// 类型在定义内部不完整，常量定义在类型之后。
    static const Point2T Zero;

    /// 下标访问。索引 0/1 依次对应 X/Y。
    ///
    /// 越界是未定义行为 —— 与 std::array 一致，不做边界检查。
    [[nodiscard]] constexpr Scalar& operator[](int index) noexcept {
        return index == 0 ? X : Y;
    }

    [[nodiscard]] constexpr const Scalar& operator[](int index) const noexcept {
        return index == 0 ? X : Y;
    }

    /// 导出为数组，便于与外部库互操作。
    [[nodiscard]] constexpr std::array<Scalar, 2> ToArray() const noexcept {
        return {X, Y};
    }

    /// 两点间的欧几里得距离。
    ///
    /// 经 Vector2T::Length() 实现，因此继承它的一切行为 —— 包括**差向量**的任一
    /// 分量含 ±inf 时返回 inf 而非 NaN。那正是长度的既定语义，不为此加特判。
    ///
    /// 限定语「**差向量**」不可省：两个点在同一位上都是同一个 ±inf、**且其余
    /// 槽位的差都是有限值**时，差向量含 NaN 而不含无穷，距离随之是 NaN ——
    /// 与 `Length()` 的规则一致（任一无穷分量 ⇒ ±inf；否则含 NaN ⇒ NaN），
    /// 不是漏判。
    ///
    /// 「其余槽位有限」这个附加条件不能省：若别的槽位差为 ±inf，那条规则会让
    /// inf 压过 NaN，距离是 inf —— 例如 `{+inf, 0}` 到 `{+inf, +inf}` 是 inf，
    /// 不是 NaN。
    [[nodiscard]] Scalar DistanceTo(Point2T other) const noexcept {
        return (*this - other).Length();
    }

    /// 两点间距离的平方。
    ///
    /// 经差向量的 `LengthSquared`，不经过平方根。`(1, 1)` 这种无理距离的平方
    /// 仍是精确的分量平方和。非有限差与 `LengthSquared` 相同：差向量含 ±inf
    /// 时平方和为 inf，`inf - inf` 得到的 NaN 差则保持 NaN。
    [[nodiscard]] constexpr Scalar DistanceSquared(Point2T other) const noexcept {
        return (*this - other).LengthSquared();
    }

    /// 线性插值。`t` 不在 `[0, 1]` 时仍按公式外推，不 clamp。
    [[nodiscard]] constexpr Point2T Lerp(Point2T other, Scalar t) const noexcept {
        return *this + (other - *this) * t;
    }
};

template <typename Scalar>
const Point2T<Scalar> Point2T<Scalar>::Zero{};

using Point2 = Point2T<double>;
using Point2f = Point2T<float>;

// ---- 运算符 ----

/// 点沿向量平移。
///
/// 刻意没有 `Point + Point`：点没有加法。缺了它编译期就会报错，而不是静默
/// 给出错误语义。
template <typename Scalar>
[[nodiscard]] constexpr Point2T<Scalar> operator+(Point2T<Scalar> p, Vector2T<Scalar> v) noexcept {
    return Point2T<Scalar>{p.X + v.X, p.Y + v.Y};
}

/// 向量 + 点：同一次平移的反向书写，结果同样是点。
///
/// 平移不关心两个操作数的书写顺序，`v + p` 与 `p + v` 必须同义 —— 调用方先
/// 写出向量时不该撞上「找不到运算符」。返回的仍是点，不会与刻意不存在的
/// `Point + Point` 混淆。
template <typename Scalar>
[[nodiscard]] constexpr Point2T<Scalar> operator+(Vector2T<Scalar> v, Point2T<Scalar> p) noexcept {
    return Point2T<Scalar>{v.X + p.X, v.Y + p.Y};
}

/// 点沿向量反方向平移。
template <typename Scalar>
[[nodiscard]] constexpr Point2T<Scalar> operator-(Point2T<Scalar> p, Vector2T<Scalar> v) noexcept {
    return Point2T<Scalar>{p.X - v.X, p.Y - v.Y};
}

/// 两点之差是位移向量，不是点。
template <typename Scalar>
[[nodiscard]] constexpr Vector2T<Scalar> operator-(Point2T<Scalar> a, Point2T<Scalar> b) noexcept {
    return Vector2T<Scalar>{a.X - b.X, a.Y - b.Y};
}

/// 逐分量比较。与 Vector 的约定一致：`operator!=` 由 C++20 自动生成，不手写。
template <typename Scalar>
[[nodiscard]] constexpr bool operator==(Point2T<Scalar> a, Point2T<Scalar> b) noexcept {
    return a.X == b.X && a.Y == b.Y;
}

} // namespace DragonGeo::Linear
