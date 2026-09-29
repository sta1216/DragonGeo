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
/// `center()` 都会变成「看情况」，调用者无从判断手里的是哪一种。
///
/// 空判定 `is_empty()` 是 `!(min <= max)` 而**不是** `min > max`：后者在 NaN 上
/// 说谎（见该成员函数）。谓词必须是全函数，否则上面那条「生产者一律规范化」的
/// 不变量会在非有限输入上被绕过。
///
/// **注意：谓词全函数化之后，`merged` / `intersects` / `clipped` 都需要显式判空
/// —— 「规范空的表示让它们无需特判」这句话现在是假的。** 规范空 [+inf, -inf] 确实
/// 能在朴素比较下自然得解，但含 NaN 的空区间不行：比较碰上 NaN 一律返回 false，
/// 于是结果会取决于操作数顺序（`intersects` 反方向翻面、`clipped` 被空集裁剪却
/// 返回全部、`merged` 不满足交换律）。三个消费函数各有一道
/// `is_empty()` 判定，它们是「不与空集相交」与「一个空 + 一个非空时取另一个」的
/// 唯一保证（两个操作数都为空且表示不同时的残留不对称见 `merged`）。
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
    /// 两种空区间（规范空与含 NaN 的空）都不含任何点，且都不需要特判：
    /// `value >= min` 对 `min = +inf` 为假、对 `min = NaN` 也为假。
    [[nodiscard]] constexpr bool contains(Scalar value) const noexcept {
        return value >= min && value <= max;
    }

    /// 闭区间相交：端点相接算相交（[1, 5] 与 [5, 9] 相交于单点 5）。
    ///
    /// 返回 bool 而非区间 —— 需要交集本身请用 clipped()。
    ///
    /// **两个操作数都要非空才算相交，必须显式判空。** 只靠下面那两行比较是不够的：
    /// 规范空 [+inf, -inf] 确实会被顶成假，但含 NaN 的空区间不会 —— 比较碰上 NaN
    /// 一律返回 false，于是结果的真假取决于**操作数顺序**
    /// （实测：`{1,5}.intersects({NaN,NaN})` 曾为真、反方向为假）。
    /// `is_empty()` 是全函数，一句判断同时挡住两种空，并恢复对称性。
    [[nodiscard]] constexpr bool intersects(IntervalT other) const noexcept {
        if (is_empty() || other.is_empty()) {
            return false;
        }
        const Scalar lower = other.min > min ? other.min : min;
        const Scalar upper = other.max < max ? other.max : max;
        return lower <= upper;
    }

    /// 区间长度，即一维测度（`max - min`）。
    ///
    /// **空区间返回 0**：空集的测度是 0，而原始差 `-inf - (+inf)` 是 -inf，
    /// 一个没有意义的「长度」，还会顺着加法一路传播下去。无界（含半无界）
    /// 区间返回 +inf —— 原始差本来就是 +inf，正确。
    ///
    /// 但 `[+inf, +inf]` 这类**非空却没有有限长度**的区间会算出 `+inf - (+inf)`
    /// = NaN。这是与 `center()` 同类的刻意哨兵（「没有长度」的诚实答案），
    /// 不是漏判；调用者若需要有限长度，应先自行排除无穷端点。**不要**指望这里
    /// 返回一个看似成功的数字。注意它随 `is_empty()` 的定义走：`is_empty()` 若退化
    /// 成 `min > max`，`{NaN, NaN}.length()` 会从 0 变成 NaN。
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
    /// **要显式判空。** 朴素的取两端外扩对规范空恰好成立（`min(…, +inf)` 取回
    /// 自身），但对 `{NaN, NaN}` 不成立：比较碰上 NaN 返回 false，于是取到的是
    /// 自己的 NaN —— 实测 `{NaN,NaN}.merged({1,5})` 曾给自己、反方向却给 `{1,5}`。
    /// 加上这两道判断之后，**「一个空 + 一个非空」的两个方向都返回那个非空的操作数**，
    /// 交换律在这一类输入上恢复。
    ///
    /// **已知的残留不对称（记录在案，未修）**：两个操作数**都**为空、但表示不同时
    /// （例如 `{NaN, NaN}` 与倒置的 `{1.0, 0.0}`），每个方向都返回「另一个」，
    /// 于是 `a.merged(b) != b.merged(a)` —— 两者都是空集，但 `==` 比的是表示。
    /// 根源是两条原则在非规范输入上冲突：「恒等」要求原样返回另一个操作数，
    /// 「规范化」要求产出空区间时必须给 `empty()`。本实现选择恒等，因此**不保证**
    /// 两个空操作数之间对称；需要规范化请自行判空后再取。
    [[nodiscard]] constexpr IntervalT merged(IntervalT other) const noexcept {
        if (is_empty()) {
            return other;
        }
        if (other.is_empty()) {
            return *this;
        }
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
    ///
    /// **任一操作数为空则直接返回规范空区间，同样必须显式判空。** 只靠末尾那次
    /// `result.is_empty()` 规范化不够：含 NaN 的空区间与别的区间取交时，
    /// 比较碰上 NaN 会把结果算成对方的一个正常区间
    /// （实测 `{1,5}.clipped({NaN,NaN})` 曾返回整个 `{1,5}` —— 被空集裁剪却得到全部）。
    [[nodiscard]] constexpr IntervalT clipped(IntervalT other) const noexcept {
        if (is_empty() || other.is_empty()) {
            return empty();
        }
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
