#pragma once

#include <limits>

namespace DragonGeo::Linear {

/// 一维闭区间 [Min, Max]。
///
/// 用途是包围盒的单分量、参数区间、投影范围这类一维量的载体；Box2/Box3 的
/// 逐分量语义与这里逐字对齐。
///
/// **空区间的规范表示是 [+inf, -inf]**，由 `Empty()` 给出；无界区间是
/// [-inf, +inf]，由 `Unbounded()` 给出。**凡是产出空区间的运算都必须给出这个
/// 规范形式** —— `Intersection` 无交集、`expanded` 收缩过头或空进空出时返回的必须是
/// `Empty()`，不能是 [2, 0] 这种「倒置但不规范」的区间。
///
/// 理由是同一个数学量不该有两种表示：若两处都算「空」，`==`、`Length()`、
/// `Center()` 都会变成「看情况」，调用者无从判断手里的是哪一种。
///
/// 空判定 `IsEmpty()` 是 `!(Min <= Max)` 而**不是** `Min > Max`：后者在 NaN 上
/// 说谎（见该成员函数）。谓词必须是全函数，否则上面那条「生产者一律规范化」的
/// 不变量会在非有限输入上被绕过。
///
/// **注意：谓词全函数化之后，`merged` / `intersects` / `Intersection` 都需要显式判空
/// —— 「规范空的表示让它们无需特判」这句话现在是假的。** 规范空 [+inf, -inf] 确实
/// 能在朴素比较下自然得解，但含 NaN 的空区间不行：比较碰上 NaN 一律返回 false，
/// 于是结果会取决于操作数顺序（`intersects` 反方向翻面、`Intersection` 被空集裁剪却
/// 返回全部、`merged` 不满足交换律）。三个消费函数各有一道
/// `IsEmpty()` 判定，它们是「不与空集相交」与「一个空 + 一个非空时取另一个」的
/// 保证，并消除了上面那三处顺序依赖（`merged` 另需一道判空，用于两个「空」表示不同时
/// 给出规范形式；`intersects` 的自判空在结果上是冗余的，理由见该函数）。
///
/// 谓词（`IsEmpty` / `contains` / `intersects`）全部是**精确比较**，不含容差 ——
/// 带容差的包含会让「这个点是否在区间内」随上下文变化。需要容差的比较应由调用方
/// 显式完成，因此本类型不依赖 Core::Tolerance。
template <typename Scalar>
struct IntervalDifferenceT;

template <typename Scalar>
struct IntervalT {
    using ScalarType = Scalar;

    // 同 VectorNT / PointNT：不声明任何构造函数，以保持聚合性 ——
    // `Interval{1.0, 5.0}` 与上面的工厂都依赖它。
    Scalar Min{};
    Scalar Max{};

    /// 空区间，规范形式 [+inf, -inf]。空集的测度是 0（见 Length()）。
    [[nodiscard]] static constexpr IntervalT Empty() noexcept {
        const Scalar infinity = std::numeric_limits<Scalar>::infinity();
        return {infinity, -infinity};
    }

    /// 无界区间，[-inf, +inf]。
    [[nodiscard]] static constexpr IntervalT Unbounded() noexcept {
        const Scalar infinity = std::numeric_limits<Scalar>::infinity();
        return {-infinity, infinity};
    }

    /// 是否为空，实现为 `!(Min <= Max)`，**不是** `Min > Max`。
    ///
    /// 空判定必须是**全函数**：任何区间要么空、要么满足 `Min <= Max`，不允许存在
    /// 「两个都不是」的第三种状态。差别只在非有限值上，但正好是要命的地方 ——
    /// `Min > Max` 对 `{NaN, NaN}` 返回 **false**，也就是谎称自己是一个正常的非空
    /// 区间。`!(Min <= Max)` 对 NaN、对倒置、对规范空三种情形都返回真。
    ///
    /// 为什么这一条特别重要：`expanded` 与 `Intersection` 都靠
    /// `result.IsEmpty() ? Empty() : result` 做规范化。谓词不全时，
    /// **有限区间的 NaN 增量**会算出 `{NaN, NaN}`（`1 - NaN` 是 NaN）、
    /// 再被 `Min > Max` 判成「非空」，于是交出一个看似成功却含 NaN 的结果；
    /// 谓词改全之后 `{NaN, NaN}` 会被认成空，那一格自动落回 `Empty()`。
    ///
    /// **举例不能用 `Empty().Expanded(+inf)`**：那一格被**输入侧**的判空先接住了
    /// （空区间直接返回 `Empty()`，根本走不到结果侧），所以它对谓词是否全函数
    /// 不敏感 —— 三个类型都把 `IsEmpty()` 退回 `Min > Max` 时，它照样返回规范空。
    /// 存活的例子是 `Interval{1.0, 5.0}.Expanded(NaN)`（见 `expanded` 与测试）。
    ///
    /// 但这**不等于**消费方可以省掉输入侧的判空：谓词全函数只保证「问得出正确
    /// 答案」，不保证「有人问」。`expanded` 就曾经只查结果 —— 规范空靠结果侧
    /// 恰好得解，非规范空 `[1, 0]` 却会膨胀出一个**非空**的 `[0, 1]`。
    /// 判空是每个消费输入的操作自己的责任（见 `expanded`）。
    ///
    /// 退化区间 [x, x] **不是**空区间 —— 它恰好含一个点。
    [[nodiscard]] constexpr bool IsEmpty() const noexcept {
        return !(Min <= Max);
    }

    /// 闭区间包含：两端都算在内，精确比较。
    ///
    /// 两种空区间（规范空与含 NaN 的空）都不含任何点，且都不需要特判：
    /// `value >= Min` 对 `Min = +inf` 为假、对 `Min = NaN` 也为假。
    [[nodiscard]] constexpr bool Contains(Scalar value) const noexcept {
        return value >= Min && value <= Max;
    }

    /// 闭区间相交：端点相接算相交（[1, 5] 与 [5, 9] 相交于单点 5）。
    ///
    /// 返回 bool 而非区间 —— 需要交集本身请用 Intersection()。
    ///
    /// **两个操作数都要非空才算相交，必须显式判空。** 只靠下面那两行比较是不够的：
    /// 规范空 [+inf, -inf] 确实会被顶成假，但含 NaN 的空区间不会 —— 比较碰上 NaN
    /// 一律返回 false，于是结果的真假取决于**操作数顺序**
    /// （实测：`{1,5}.Intersects({NaN,NaN})` 曾为真、反方向为假）。
    /// `IsEmpty()` 是全函数，一句判断同时挡住两种空，并恢复对称性。
    ///
    /// 其中 `IsEmpty()`（自判空）这一半在**结果上是冗余的、防御性的**，不是必需的：
    /// 「自己空、对方非空」时朴素比较必然为假（规范空给出 `lower = +inf`、
    /// `upper = -inf`；NaN 空给出 `lower = NaN` 或 `upper = NaN`）。已用 121×121 组
    /// 含 NaN / ±inf / 非规范空的差分探针实测：去掉它以后**逐字节相同**（同一装置对
    /// 「整块去掉判空」报出 848 行差异，证明装置有效）。**删掉它的是等价变异体，
    /// 不改变任何结果，后续轮次不必再追。** 保留它的理由是与 `Intersection` / `merged`
    /// 的写法对称、可读，且比较式将来一改它就会重新变得必要。
    [[nodiscard]] constexpr bool Intersects(IntervalT other) const noexcept {
        if (IsEmpty() || other.IsEmpty()) {
            return false;
        }
        const Scalar lower = other.Min > Min ? other.Min : Min;
        const Scalar upper = other.Max < Max ? other.Max : Max;
        return lower <= upper;
    }

    /// 区间长度，即一维测度（`Max - Min`）。
    ///
    /// **空区间返回 0**：空集的测度是 0，而原始差 `-inf - (+inf)` 是 -inf，
    /// 一个没有意义的「长度」，还会顺着加法一路传播下去。无界（含半无界）
    /// 区间返回 +inf —— 原始差本来就是 +inf，正确。
    ///
    /// 但 `[+inf, +inf]` 这类**非空却没有有限长度**的区间会算出 `+inf - (+inf)`
    /// = NaN。这是与 `Center()` 同类的刻意哨兵（「没有长度」的诚实答案），
    /// 不是漏判；调用者若需要有限长度，应先自行排除无穷端点。**不要**指望这里
    /// 返回一个看似成功的数字。注意它随 `IsEmpty()` 的定义走：`IsEmpty()` 若退化
    /// 成 `Min > Max`，`{NaN, NaN}.Length()` 会从 0 变成 NaN。
    [[nodiscard]] constexpr Scalar Length() const noexcept {
        return IsEmpty() ? Scalar{0} : Max - Min;
    }

    /// 区间中点。
    ///
    /// 用 `Min * 0.5 + Max * 0.5` 计算，**不要**写成 `(Min + Max) * 0.5`（在
    /// [1e308, 1e308] 上中间和溢出成 +inf）或 `Min + (Max - Min) * 0.5`
    /// （在 [-1e308, 1e308] 上差溢出成 +inf）；上面这个形式对两者都正确。
    ///
    /// **两端点为异号无穷时结果是 NaN**（`(+inf) + (-inf)`；规范空区间与无界
    /// 区间都是这一形态）—— 这是刻意的哨兵，是「没有中点」的诚实答案，不是
    /// 漏判。**非规范空区间不在此列**：`{2, 0}`（`IsEmpty()` 为真，聚合初始化
    /// 随手就能造出来）的中点是有限的 `1.0`，一个看似完全正常的数字 ——
    /// 本函数不判空，区分不了两种「空」的表示，这正是「生产者一律规范化」的
    /// 用途之一。调用者应先 `IsEmpty()` 或先判端点有限性，不要指望这里返回
    /// 一个看似成功的数字。
    [[nodiscard]] constexpr Scalar Center() const noexcept {
        return Min * Scalar{0.5} + Max * Scalar{0.5};
    }

    /// 最小包含两者的区间。
    ///
    /// **要显式判空。** 朴素的取两端外扩对规范空恰好成立（`Min(…, +inf)` 取回
    /// 自身），但对 `{NaN, NaN}` 不成立：比较碰上 NaN 返回 false，于是取到的是
    /// 自己的 NaN —— 实测 `{NaN,NaN}.Merged({1,5})` 曾给自己、反方向却给 `{1,5}`。
    /// 加上这两道判断之后，**「一个空 + 一个非空」的两个方向都返回那个非空的操作数**，
    /// 交换律在这一类输入上恢复。
    ///
    /// 注意第一支里的 `other.IsEmpty() ? Empty() : other`：**规范化优先于恒等律**。
    /// 两者在**非规范空**上冲突 —— 恒等律说 `Merged(∅, x) = x`，规范化说产出空
    /// 就必须是 `Empty()`。本项目明写的规则是后者（「凡是产出空区间的运算都必须
    /// 给出这个规范形式」，理由是一个数学量不该有两种表示），恒等律只是该表示对
    /// **规范输入**的推论，不是能压过它的公理。实测：不做这一步时 `merged` 在
    /// 4140/14641 组输入上不对称（两侧都返回「另一个」，两者都是空集但 `==` 比表示），
    /// 做了之后归零、整套测试全绿。**交换律是这一步的结果，不是它的目的。**
    [[nodiscard]] constexpr IntervalT Merged(IntervalT other) const noexcept {
        if (IsEmpty()) {
            return other.IsEmpty() ? Empty() : other;
        }
        if (other.IsEmpty()) {
            return *this;
        }
        const Scalar lower = other.Min < Min ? other.Min : Min;
        const Scalar upper = other.Max > Max ? other.Max : Max;
        return {lower, upper};
    }

    /// 两端各向外扩展 amount（负值即收缩）。
    ///
    /// **任何空区间（含非规范表示）膨胀后仍是规范空区间。**
    ///
    /// 开头先判输入，与末尾的「结果为空则规范化」**各管一段、不可互相替代**：
    ///
    /// - 开头这一句管**输入侧**：只靠末尾那次规范化，只有在输入是**规范**空时才
    ///   兑现「空进空出」；非规范空 `[1, 0]` 膨胀 1 会得到 `[0, 1]` ——
    ///   一个看起来完全正常的**非空**区间从一个空输入产生。此外，每个消费输入的
    ///   操作都该查一次 `IsEmpty()`（`intersects` / `merged` / `Intersection` 都查），
    ///   `expanded` 不该是例外。
    /// - 末尾那一句管**结果侧**：收缩过头（`[1, 5]` 扩 -3 → `[4, 2]`）与
    ///   **有限区间的 NaN 增量**（`[1, 5].Expanded(NaN)` → `{NaN, NaN}`）都靠它
    ///   规范化，输入侧那句拦不住这些：输入非空，照样走到这里。
    ///   （注意不能拿 `Empty().Expanded(+inf)` 当例子 —— 那一格被输入侧先接住，
    ///   走不到结果侧，因此对结果侧用哪个谓词不敏感。见 `IsEmpty()` 的说明。）
    ///
    /// **返回 `Empty()` 而不是 `*this`**：后者也满足「空进空出」，但返回的是
    /// 非规范的那个空表示 —— 本项目已裁定「产出空必须给规范形式」，
    /// 让同一个空集有两种表示正是那条规则要排除的。
    [[nodiscard]] constexpr IntervalT Expanded(Scalar amount) const noexcept {
        if (IsEmpty()) {
            return Empty();
        }
        const IntervalT result{Min - amount, Max + amount};
        return result.IsEmpty() ? Empty() : result;
    }

    /// 与 other 的交集。
    ///
    /// 不相交时返回**规范空区间**：不能把 [6, 5] 这种倒置结果直接返回 ——
    /// 它的 IsEmpty() 也为真，但 Min/Max 携带的是错误信息，于是同一个空集有了
    /// 两种表示。端点相接时结果是退化的单点区间 [5, 5]，不是空区间。
    ///
    /// **任一操作数为空则直接返回规范空区间，同样必须显式判空。** 只靠末尾那次
    /// `result.IsEmpty()` 规范化不够：含 NaN 的空区间与别的区间取交时，
    /// 比较碰上 NaN 会把结果算成对方的一个正常区间
    /// （实测 `{1,5}.Intersection({NaN,NaN})` 曾返回整个 `{1,5}` —— 被空集裁剪却得到全部）。
    [[nodiscard]] constexpr IntervalT Intersection(IntervalT other) const noexcept {
        if (IsEmpty() || other.IsEmpty()) {
            return Empty();
        }
        const Scalar lower = other.Min > Min ? other.Min : Min;
        const Scalar upper = other.Max < Max ? other.Max : Max;
        const IntervalT result{lower, upper};
        return result.IsEmpty() ? Empty() : result;
    }

    /// 差集：属于自身、不属于 other 的部分。
    ///
    /// 闭区间的差集不一定仍是闭区间，也不一定仍是**一段**。本函数返回差集的
    /// **闭包**，最多两段，装在 `IntervalDifferenceT` 里：
    ///
    /// - `Below` 是挖掉的那段左侧（较小坐标），没有则为规范空区间；
    /// - `Above` 是挖掉的那段右侧，没有则为规范空区间。
    ///
    /// 被挖掉的重叠若只有一个点（或根本不相交），闭包仍是原来的整段，放在
    /// `Below`，`Above` 为空 —— 去掉一个点不改变闭区间的闭包，因此**不会**拆成
    /// 两段。只有重叠的长度大于 0 时才会裁掉一截或从中间挖开。
    ///
    /// 裁切留下的端点仍算在结果里（闭包）。因此 `Below` / `Above` 可能与 other
    /// 在单个端点上 `Intersects`。这是闭区间表达不了半开区间的结果，不是漏判。
    ///
    /// 任一操作数为空时与 `Intersection` 同一套判空：自身为空则两段都是规范空；
    /// other 为空则差集就是自身（非规范空先收成 `Empty()`）。
    [[nodiscard]] constexpr IntervalDifferenceT<Scalar> Difference(IntervalT other) const noexcept;
};

/// `IntervalT::Difference` 的结果：最多两段闭区间，按坐标从低到高放在
/// `Below` 与 `Above`。缺的那一段是规范空区间。
template <typename Scalar>
struct IntervalDifferenceT {
    IntervalT<Scalar> Below = IntervalT<Scalar>::Empty();
    IntervalT<Scalar> Above = IntervalT<Scalar>::Empty();
};

template <typename Scalar>
constexpr IntervalDifferenceT<Scalar> IntervalT<Scalar>::Difference(IntervalT other) const noexcept {
    const IntervalT overlap = Intersection(other);
    if (overlap.IsEmpty() || !(overlap.Length() > Scalar{0})) {
        return {IsEmpty() ? Empty() : *this, Empty()};
    }

    IntervalDifferenceT<Scalar> result;
    if (Min < overlap.Min) {
        result.Below = IntervalT{Min, overlap.Min};
    }
    if (Max > overlap.Max) {
        result.Above = IntervalT{overlap.Max, Max};
    }
    return result;
}

using Interval = IntervalT<double>;
using Intervalf = IntervalT<float>;
using IntervalDifference = IntervalDifferenceT<double>;
using IntervalDifferencef = IntervalDifferenceT<float>;

// ---- 运算符 ----

/// 两端都相等才算相等。与 Vector / Point 的约定一致：`operator!=` 由 C++20
/// 自动生成，不手写。
template <typename Scalar>
[[nodiscard]] constexpr bool operator==(IntervalT<Scalar> a, IntervalT<Scalar> b) noexcept {
    return a.Min == b.Min && a.Max == b.Max;
}

template <typename Scalar>
[[nodiscard]] constexpr bool operator==(
    IntervalDifferenceT<Scalar> a, IntervalDifferenceT<Scalar> b) noexcept {
    return a.Below == b.Below && a.Above == b.Above;
}

} // namespace DragonGeo::Linear
