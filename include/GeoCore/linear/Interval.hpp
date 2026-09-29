#pragma once

#include <limits>

namespace GeoCore::linear {

/// 一维闭区间 [min, max]。
///
/// 用途是包围盒的单分量、参数区间、投影范围这类一维量的载体；Box2/Box3 的
/// 逐分量语义与这里逐字对齐。
///
/// **空区间的规范表示是 [+inf, -inf]**，由 `empty()` 给出；无界区间是
/// [-inf, +inf]，由 `unbounded()` 给出。**凡是产出空区间的运算都必须给出这个
/// 规范形式** —— `clipped` 无交集、`expanded` 收缩过头时返回的必须是 `empty()`，
/// 不能是 [2, 0] 这种「倒置但不规范」的区间。
///
/// 理由是同一个数学量不该有两种表示：若两处都算「空」，`==`、`length()`、
/// `center()` 都会变成「看情况」，调用者无从判断手里的是哪一种。这个选择同时
/// 让谓词无需特判：`merged` 取两端外扩、`intersects` 比较 `max(min) <= min(max)`，
/// 对空区间都自然给出正确答案。
///
/// 空判定 `is_empty()` 是 `!(min <= max)` 而**不是** `min > max`：后者在 NaN 上
/// 说谎（见该成员函数）。谓词必须是全函数，否则上面那条「生产者一律规范化」的
/// 不变量会在非有限输入上被绕过。
///
/// 谓词（`is_empty` / `contains` / `intersects`）全部是**精确比较**，不含容差 ——
/// 带容差的包含会让「这个点是否在区间内」随上下文变化。需要容差的比较应由调用方
/// 显式完成，因此本类型不依赖 core::Tolerance。
template <typename Scalar>
struct IntervalT {
    using scalar_type = Scalar;

    // 同 VectorNT / PointNT：不声明任何构造函数，以保持聚合性 ——
    // `Interval{1.0, 5.0}` 与上面的工厂都依赖它。
    Scalar min{};
    Scalar max{};

    /// 空区间，规范形式 [+inf, -inf]。空集的测度是 0（见 length()）。
    [[nodiscard]] static constexpr IntervalT empty() noexcept {
        const Scalar infinity = std::numeric_limits<Scalar>::infinity();
        return {infinity, -infinity};
    }

    /// 无界区间，[-inf, +inf]。
    [[nodiscard]] static constexpr IntervalT unbounded() noexcept {
        const Scalar infinity = std::numeric_limits<Scalar>::infinity();
        return {-infinity, infinity};
    }

    /// 是否为空，实现为 `!(min <= max)`，**不是** `min > max`。
    ///
    /// 空判定必须是**全函数**：任何区间要么空、要么满足 `min <= max`，不允许存在
    /// 「两个都不是」的第三种状态。差别只在非有限值上，但正好是要命的地方 ——
    /// `min > max` 对 `{NaN, NaN}` 返回 **false**，也就是谎称自己是一个正常的非空
    /// 区间。`!(min <= max)` 对 NaN、对倒置、对规范空三种情形都返回真。
    ///
    /// 为什么这一条特别重要：`expanded` 与 `clipped` 都靠
    /// `result.is_empty() ? empty() : result` 做规范化。谓词不全时
    /// `empty().expanded(+inf)` 会算出 `{NaN, NaN}`（`+inf - (+inf)` 是 NaN）、
    /// 再被 `min > max` 判成「非空」，于是交出一个看似成功却含 NaN 的结果；
    /// 谓词改全之后它自动落回 `empty()`，**无需在 expanded 里加特判**。
    ///
    /// 退化区间 [x, x] **不是**空区间 —— 它恰好含一个点。
    [[nodiscard]] constexpr bool is_empty() const noexcept {
        return !(min <= max);
    }

    /// 闭区间包含：两端都算在内，精确比较。
    ///
    /// 空区间不含任何点（`min = +inf` 让比较自然为假），无需特判。
    [[nodiscard]] constexpr bool contains(Scalar value) const noexcept {
        return value >= min && value <= max;
    }

    /// 闭区间相交：端点相接算相交（[1, 5] 与 [5, 9] 相交于单点 5）。
    ///
    /// 返回 bool 而非区间 —— 需要交集本身请用 clipped()。空区间无需特判：
    /// [+inf, -inf] 会把 max(min) 顶到 +inf、把 min(max) 压到 -inf，
    /// 比较必然为假。
    [[nodiscard]] constexpr bool intersects(IntervalT other) const noexcept {
        const Scalar lower = other.min > min ? other.min : min;
        const Scalar upper = other.max < max ? other.max : max;
        return lower <= upper;
    }

    /// 区间长度，即一维测度（`max - min`）。
    ///
    /// **空区间返回 0**：空集的测度是 0，而原始差 `-inf - (+inf)` 是 -inf，
    /// 一个没有意义的「长度」，还会顺着加法一路传播下去。无界（含半无界）
    /// 区间返回 +inf —— 原始差本来就是 +inf，正确。
    [[nodiscard]] constexpr Scalar length() const noexcept {
        return is_empty() ? Scalar{0} : max - min;
    }

    /// 区间中点。
    ///
    /// 用 `min * 0.5 + max * 0.5` 计算，**不要**写成 `(min + max) * 0.5`（在
    /// [1e308, 1e308] 上中间和溢出成 +inf）或 `min + (max - min) * 0.5`
    /// （在 [-1e308, 1e308] 上差溢出成 +inf）；上面这个形式对两者都正确。
    ///
    /// **任一端点是 ±inf 时结果是 NaN**（`(+inf) + (-inf)`）—— 这是刻意的哨兵，
    /// 是「没有中点」的诚实答案，不是漏判。调用者应先 is_empty() 或先判端点
    /// 有限性，不要指望这里返回一个看似成功的数字。
    [[nodiscard]] constexpr Scalar center() const noexcept {
        return min * Scalar{0.5} + max * Scalar{0.5};
    }

    /// 最小包含两者的区间。
    ///
    /// 对空区间自然成立，无需特判：`merged(empty())` 取 min(…, +inf) 与
    /// max(…, -inf)，结果恰好是自身。
    [[nodiscard]] constexpr IntervalT merged(IntervalT other) const noexcept {
        const Scalar lower = other.min < min ? other.min : min;
        const Scalar upper = other.max > max ? other.max : max;
        return {lower, upper};
    }

    /// 两端各向外扩展 amount（负值即收缩）。
    ///
    /// 收缩过头时返回**规范空区间**，不是倒置的 [4, 2]；空区间扩展后仍是空区间
    /// （不会退化成 unbounded()）。
    [[nodiscard]] constexpr IntervalT expanded(Scalar amount) const noexcept {
        const IntervalT result{min - amount, max + amount};
        return result.is_empty() ? empty() : result;
    }

    /// 与 other 的交集。
    ///
    /// 不相交时返回**规范空区间**：不能把 [6, 5] 这种倒置结果直接返回 ——
    /// 它的 is_empty() 也为真，但 min/max 携带的是错误信息，于是同一个空集有了
    /// 两种表示。端点相接时结果是退化的单点区间 [5, 5]，不是空区间。
    [[nodiscard]] constexpr IntervalT clipped(IntervalT other) const noexcept {
        const Scalar lower = other.min > min ? other.min : min;
        const Scalar upper = other.max < max ? other.max : max;
        const IntervalT result{lower, upper};
        return result.is_empty() ? empty() : result;
    }
};

using Interval = IntervalT<double>;
using Intervalf = IntervalT<float>;

// ---- 运算符 ----

/// 两端都相等才算相等。与 Vector / Point 的约定一致：`operator!=` 由 C++20
/// 自动生成，不手写。
template <typename Scalar>
[[nodiscard]] constexpr bool operator==(IntervalT<Scalar> a, IntervalT<Scalar> b) noexcept {
    return a.min == b.min && a.max == b.max;
}

} // namespace GeoCore::linear
