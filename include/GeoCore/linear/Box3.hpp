#pragma once

#include <limits>

#include <GeoCore/linear/Point3.hpp>
#include <GeoCore/linear/Vector3.hpp>

namespace GeoCore::linear {

/// 三维轴对齐包围盒（AABB），由两个角点 `min` / `max` 给出。
///
/// 逐分量语义与 `IntervalT` 完全对齐 —— 本类型就是三个一维闭区间的笛卡尔积
/// （`contains(Point3T)` 是三个一维包含的合取，`intersects` 是三个一维相交的
/// 合取，`merged` 是三个一维合并）。Interval 那边已经裁定的规则这里不重新讨论，
/// 只写本类型特有的约定。
///
/// **对齐的是语义，不是接口面**：一维测度在 `IntervalT` 里叫 `length()`，在本
/// 类型里叫 `extent()` / `half_extent()`；`unbounded()` / `clipped()` /
/// `contains(IntervalT)` 在本类型**没有对应物**（求交与裁剪归后续的 `query` 层，
/// 见 spec §5.2）。不要为了「整齐」在本阶段补这些 API —— 它们的名字与归属是
/// 后续阶段的决定，提前长出来只会把接口面钉错。
///
/// **空盒的规范表示是 `min` 的每个分量为 +inf、`max` 的每个分量为 -inf**，
/// 由 `empty()` 给出。**凡是产出空盒的运算都必须给出这个规范形式**
/// （`expanded` 收缩过头、`from_corners` 遇到 NaN 角点时都是例子），
/// 不能给「只有 x 分量倒置」的盒。
/// 理由是同一个数学量不该有两种表示：若两处都算「空」，`==`、`extent()`、
/// `center()` 都会变成「看情况」，调用者无从判断手里的是哪一种。
///
/// **这个表示是后续 `OrientedBox` 的既定依赖，不得改动** —— 它使「从空盒开始
/// 逐分量取 min/max」自然成立，无需对空盒特判。
///
/// 空判定 `is_empty()` 是 `!(min <= max)` 逐分量取或，而**不是** `min > max`：
/// 后者在 NaN 上说谎（见该成员函数）。谓词必须是全函数，否则上面那条
/// 「生产者一律规范化」的不变量会在非有限输入上被绕过。
///
/// **注意：谓词全函数化之后，`intersects` / `contains(Box3T)` / `merged` 都需要
/// 显式判空 —— 「规范空的表示让它们无需特判」这句话现在是假的。** 规范空
/// [+inf, -inf]³ 确实能在朴素比较下自然得解，但另外两种空不行：
///
/// - **含 NaN 的盒**：比较碰上 NaN 一律返回 false，于是结果会取决于操作数顺序；
/// - **非规范空盒**（例如 `{0,0,0}`–`{-1,-1,-1}`，聚合初始化随手就能造出来）：
///   朴素的逐分量比较会让它「相交」任何横跨 [0, -1] 的盒，也会让一个更大的盒
///   「包含」它。这是 `is_empty()` 真的**载重**的地方，不是防御性冗余。
///
/// 谓词（`is_empty` / `contains` / `intersects`）全部是**精确比较**，不含容差 ——
/// 带容差的包含会让「这个点是否在盒内」随上下文变化。需要容差的比较应由调用方
/// 显式完成，因此本类型不依赖 `core::Tolerance`。
template <typename Scalar>
struct Box3T {
    using scalar_type = Scalar;

    // 同 VectorNT / PointNT / IntervalT：不声明任何构造函数，以保持聚合性 ——
    // `Box3{Point3{...}, Point3{...}}` 与下面的工厂都依赖它。
    Point3T<Scalar> min{};
    Point3T<Scalar> max{};

    /// 空盒，规范形式 `[+inf, -inf]³`。空集的测度是 0（见 `extent()`）。
    [[nodiscard]] static constexpr Box3T empty() noexcept {
        const Scalar infinity = std::numeric_limits<Scalar>::infinity();
        return Box3T{Point3T<Scalar>{infinity, infinity, infinity},
                     Point3T<Scalar>{-infinity, -infinity, -infinity}};
    }

    /// 由两个角点构造，**接受任意顺序** —— 名字说的是「角点」不是「min/max」，
    /// 调用方不该被迫先自己排一遍。逐分量取 min / max。
    ///
    /// 「逐分量」是接口的一部分：`from_corners({1, 0}, {0, 2})` 得到
    /// `[{0, 0}, {1, 2}]`，而**不是**按某个分量决定要不要整体交换两个点。
    ///
    /// **任一角点含 NaN 分量时返回规范空盒**（`empty()`）；**±inf 不在此列** ——
    /// `from_corners((-inf,-inf,-inf), (+inf,+inf,+inf))` 得到整个空间，语义正确。
    /// NaN 必须**显式检测**，不能只靠出口的 `is_empty()` 兜底：逐分量比较碰上
    /// NaN 一律返回 false，于是朴素实现的结果取决于实参顺序 —— 一个顺序得到含
    /// NaN 的盒（`is_empty()` 为真、却不等于 `empty()`），交换实参后 NaN 被静默
    /// 丢给另一个操作数、得到一个**看似完全正常**的盒。空盒是「不含任何点」的
    /// 诚实答案，与 `OrientedBox3T::to_axis_aligned()` 对 NaN 角点的处置同源。
    [[nodiscard]] static constexpr Box3T from_corners(Point3T<Scalar> a,
                                                      Point3T<Scalar> b) noexcept {
        // NaN 检测用 `v != v`（NaN 是唯一不等于自身的值）—— 同 core::is_finite
        // 里 `value == value` 的写法。只查 NaN：±inf 是合法端点。
        if (a.x != a.x || a.y != a.y || a.z != a.z
            || b.x != b.x || b.y != b.y || b.z != b.z) {
            return empty();
        }
        return Box3T{Point3T<Scalar>{a.x < b.x ? a.x : b.x,
                                     a.y < b.y ? a.y : b.y,
                                     a.z < b.z ? a.z : b.z},
                     Point3T<Scalar>{a.x > b.x ? a.x : b.x,
                                     a.y > b.y ? a.y : b.y,
                                     a.z > b.z ? a.z : b.z}};
    }

    /// 是否为空，实现为 `!(min <= max)` 逐分量取或，**不是** `min > max`。
    ///
    /// 空判定必须是**全函数**：任何盒要么空、要么满足 `min <= max`，不允许存在
    /// 「两个都不是」的第三种状态。差别只在非有限值上，但正好是要命的地方 ——
    /// `min > max` 对含 NaN 的盒返回 **false**，也就是谎称自己是一个正常的非空盒。
    ///
    /// 为什么这一条特别重要：`expanded` 靠 `result.is_empty() ? empty() : result`
    /// 做规范化。谓词不全时，**有限盒的 NaN 增量**会算出 `{NaN}³` 这种
    /// 「看似成功却含 NaN」的结果，而不是干净地落回 `empty()`。
    ///
    /// **举例不能用 `empty().expanded(+inf)`**：那一格被 `expanded` **输入侧**的
    /// 判空先接住（空盒直接返回 `empty()`，根本走不到结果侧），所以它对谓词是否
    /// 全函数不敏感 —— 把 `is_empty()` 退回 `min > max`，它照样返回规范空。
    /// 存活的例子是有限盒 `[{0,0,0},{1,1,1}].expanded(NaN)`。
    ///
    /// 退化的盒 `[p, p]`（三个分量都取等）**不是**空盒 —— 它恰好含一个点。
    [[nodiscard]] constexpr bool is_empty() const noexcept {
        return !(min.x <= max.x) || !(min.y <= max.y) || !(min.z <= max.z);
    }

    /// 闭盒包含：三个轴上都要 `min <= p <= max`，六个比较都算边界，精确比较。
    ///
    /// 两种空的盒都不含任何点，且都不需要特判：`p.x >= min.x` 对 `min.x = +inf`
    /// 为假、对 `min.x = NaN` 也为假。这正是选这个空表示的理由。
    ///
    /// 注意六个比较必须**直接**写出来，不要「化简」成双重否定
    /// （`!(p.x < min.x) && !(p.x > max.x)`）—— 后者在 NaN 上两边都为真。
    [[nodiscard]] constexpr bool contains(Point3T<Scalar> point) const noexcept {
        return point.x >= min.x && point.x <= max.x
            && point.y >= min.y && point.y <= max.y
            && point.z >= min.z && point.z <= max.z;
    }

    /// 盒包含盒（闭的：`a.contains(a)` 为真）。
    ///
    /// **任一方为空盒时返回 `false`。** 数学上 `∅ ⊆ B` 是空真，但那会让
    /// `if (a.contains(b))` 在 `b` 为空时通过，是每个调用者都会踩的坑；
    /// `intersects` 已经采用「空盒与任何盒都不相交」，`contains` 保持同向。
    /// **这是刻意的约定，不要把它当 bug 改掉。**
    ///
    /// 这个判断不能省：非规范空盒（`{0,0,0}`–`{-1,-1,-1}`）在朴素比较下会被
    /// 任何一个「横跨它」的盒判为「被包含」—— `min <= 0` 与 `max >= -1` 同时成立。
    /// 载重的是 `other.is_empty()` 这一半（实测：去掉整道判定后两条断言失败）。
    ///
    /// 反过来，`is_empty()`（自判空）那一半是**可证不可观测的**：逐分量的
    /// `min <= other.min <= other.max <= max` 蕴含 `min <= max`，所以一个倒置的
    /// `*this` 不可能在朴素比较下返回 true（NaN 同理，比较一律为假）。保留它只为
    /// 了与 `intersects` / `merged` 的写法对称 —— 实测这一半去掉后没有任何断言失败。
    /// 注意这与 `intersects` 相反：那边两半都载重（交叉操作数的缘故）。
    [[nodiscard]] constexpr bool contains(Box3T<Scalar> other) const noexcept {
        if (is_empty() || other.is_empty()) {
            return false;
        }
        return min.x <= other.min.x && max.x >= other.max.x
            && min.y <= other.min.y && max.y >= other.max.y
            && min.z <= other.min.z && max.z >= other.max.z;
    }

    /// 闭盒相交：面 / 边 / 角相接都算相交（共享至少一个点）。
    ///
    /// **两个操作数都要非空才算相交，必须显式判空。** 只靠下面那六行比较是不够的：
    /// 规范空确实会被顶成假，但含 NaN 的盒会让结果取决于操作数顺序，而**非规范
    /// 空盒**更直接 —— `spanning.intersects({0,0,0},{-1,-1,-1})` 的朴素比较
    /// 会把一个空的集合判成与别人相交。
    ///
    /// **这道判定里的两半都是载重的。** `IntervalT::intersects` 把两端各自归约成
    /// 一个标量再比较，自判空在结果上是冗余的（那边实测过）；本类型是逐分量
    /// AND 链，用的是**交叉操作数**（`min <= other.max && max >= other.min`），
    /// 于是自判空同样是必需的：一个倒置的盒只要碰上横跨 `[max, min]` 的另一个盒，
    /// 朴素比较就返回 true —— 实测把 `is_empty()` 那一半去掉后，
    /// `spanning.intersects(non_canonical_empty)` 变成真（它该是假）。
    /// **不要照搬 Interval 的结论把自判空删掉**，两个方向各有一格实测证据。
    [[nodiscard]] constexpr bool intersects(Box3T<Scalar> other) const noexcept {
        if (is_empty() || other.is_empty()) {
            return false;
        }
        return min.x <= other.max.x && max.x >= other.min.x
            && min.y <= other.max.y && max.y >= other.min.y
            && min.z <= other.max.z && max.z >= other.min.z;
    }

    /// 三轴尺寸（`max - min` 逐分量）。
    ///
    /// **空盒返回零向量**：空集的测度是 0，而原始差 `-inf - (+inf)` 是 -inf，
    /// 一个没有意义的「尺寸」，还会顺着加法一路传播下去。
    ///
    /// 但 `[+inf, +inf]³` 这类**非空却没有有限尺寸**的盒会算出 `inf - inf` = NaN。
    /// 这是与 `center()` 同类的刻意哨兵（「没有尺寸」的诚实答案），不是漏判；
    /// 调用者若需要有限尺寸，应先自行排除无穷端点。
    [[nodiscard]] constexpr Vector3T<Scalar> extent() const noexcept {
        if (is_empty()) {
            return Vector3T<Scalar>{};
        }
        return max - min;
    }

    /// 三轴半尺寸，即 `extent() * 0.5`。空盒返回零向量（空集的测度是 0）。
    [[nodiscard]] constexpr Vector3T<Scalar> half_extent() const noexcept {
        return extent() * Scalar{0.5};
    }

    /// 盒中心。
    ///
    /// 逐分量写成 `min * 0.5 + max * 0.5`，**不要**写成 `(min + max) * 0.5`
    /// （在 [1e308, 1e308] 上中间和溢出成 +inf）或 `min + (max - min) * 0.5`
    /// （在 [-1e308, 1e308] 上差溢出成 +inf）；上面这个形式对两者都正确。
    ///
    /// 注意这里不能直写 `min * 0.5 + max * 0.5` 这种向量式 —— `Point3T` 刻意
    /// 没有 `operator*`（标量）也没有 `Point3T + Point3T`，必须逐分量做标量算术。
    ///
    /// **规范空盒、以及任一端点无穷的盒，结果是 NaN**（`(+inf) + (-inf)`）——
    /// 这是刻意的哨兵，是「没有中心」的诚实答案，不是漏判。**非规范空盒不在
    /// 此列**：`{2,0,0}`–`{0,1,1}`（`is_empty()` 为真，聚合初始化随手就能造出来）
    /// 的中心是有限的 `(1, 0.5, 0.5)`，一个看似完全正常的点 —— 本函数不判空，
    /// 区分不了两种「空」的表示，这正是「生产者一律规范化」的用途之一。
    /// 调用者应先 `is_empty()` 或先判端点有限性，不要指望这里返回一个看似成功的点。
    [[nodiscard]] constexpr Point3T<Scalar> center() const noexcept {
        return Point3T<Scalar>{min.x * Scalar{0.5} + max.x * Scalar{0.5},
                               min.y * Scalar{0.5} + max.y * Scalar{0.5},
                               min.z * Scalar{0.5} + max.z * Scalar{0.5}};
    }

    /// 最小包含两者的盒（逐分量取 min / max）。
    ///
    /// **要显式判空。** 朴素的取两端外扩对**规范**空恰好成立
    /// （`min(…, +inf)` 取回自身），但对含 NaN 的盒、对非规范空盒都不成立
    /// （比较碰上 NaN 返回 false，于是取到的是自己的 NaN）。
    ///
    /// 注意第一支里的 `other.is_empty() ? empty() : other`：**规范化优先于恒等律**。
    /// 两者在**非规范空**上冲突 —— 恒等律说 `merged(∅, x) = x`，规范化说产出空
    /// 就必须是 `empty()`。本项目明写的规则是后者（「凡是产出空盒的运算都必须
    /// 给出这个规范形式」，理由是一个数学量不该有两种表示），恒等律只是该表示对
    /// **规范输入**的推论，不是能压过它的公理。少了这一步会让 `merged` 在两个
    /// 「空」表示不同时不对称（两侧各自返回「另一个」，都是空集但 `==` 比表示）。
    /// **交换律是这一步的结果，不是它的目的。**
    [[nodiscard]] constexpr Box3T merged(Box3T<Scalar> other) const noexcept {
        if (is_empty()) {
            return other.is_empty() ? empty() : other;
        }
        if (other.is_empty()) {
            return *this;
        }
        return Box3T{Point3T<Scalar>{min.x < other.min.x ? min.x : other.min.x,
                                     min.y < other.min.y ? min.y : other.min.y,
                                     min.z < other.min.z ? min.z : other.min.z},
                     Point3T<Scalar>{max.x > other.max.x ? max.x : other.max.x,
                                     max.y > other.max.y ? max.y : other.max.y,
                                     max.z > other.max.z ? max.z : other.max.z}};
    }

    /// 三轴各向两侧扩 amount（负值即收缩）。
    ///
    /// 收缩过头时返回**规范空盒**，不是分量倒置的盒。
    ///
    /// **任何空盒扩展后仍是规范空盒。** 开头那道 `is_empty()` 判定同时挡住三种空：
    /// 规范空、含 NaN 的空、以及非规范表示的空（`{0,0,0}`–`{-1,-1,-1}`）——
    /// 空集的测度是 0，它没有端点可以往外扩；而且 `+inf - amount` 仍是 +inf、
    /// `-inf + amount` 仍是 -inf，不会退化成整个空间。
    ///
    /// 这道判定让 `expanded` 与 `intersects` / `contains(Box3T)` / `merged` 归入
    /// 同一类：**消费输入的操作都先问一句 `is_empty()`**。缺了它，非规范空盒会被
    /// 逐端点外扩成一个**非空**盒 —— `{0,0,0}`–`{-1,-1,-1}` 扩展 1.0 曾得到
    /// `{-1,-1,-1}`–`{0,0,0}`，一个看起来完全正常、尺寸却来自无意义端点的盒。
    ///
    /// **用 `result.is_empty()` 判、不要用 `min > max` 判** —— 前者是全函数
    /// （含 NaN），后者会把**有限盒的 NaN 增量**算出的 `{NaN}³` 冒充成非空盒
    /// （`min > max` 碰上 NaN 一律返回 false），于是交出一个看似成功却含 NaN 的盒。
    /// （举例**不能**用 `empty().expanded(+inf)`：那一格被输入侧的判空先接住，
    /// 根本走不到结果侧，因此对这里用哪个谓词不敏感。）
    ///
    /// `amount` 为 **NaN** 时逐分量算出 NaN（`0 - NaN` 是 NaN），全函数谓词判中间
    /// 结果为真，于是同样落回**规范空盒**：**凡是中间结果为空的情形，出口都是
    /// 同一个 `empty()`**，调用者不必为 amount 特判。
    /// （实测：有限盒 `[{0,0,0},{1,1,1}].expanded(NaN)` 的中间结果是全 NaN 的盒，
    /// `is_empty()` 为真，最终返回 `[+inf,-inf]³`。）
    ///
    /// **注意这不等于「非有限 amount 一律产出空盒」**：有限盒 `expanded(+inf)`
    /// 得到的是整个空间 `[-inf,+inf]³`，非空 —— 中间结果为空才有上面那条出口。
    [[nodiscard]] constexpr Box3T expanded(Scalar amount) const noexcept {
        if (is_empty()) {
            return empty();
        }
        const Box3T result{Point3T<Scalar>{min.x - amount, min.y - amount, min.z - amount},
                           Point3T<Scalar>{max.x + amount, max.y + amount, max.z + amount}};
        return result.is_empty() ? empty() : result;
    }

    /// 第 `index` 个角点，`index` 取 0..7。
    ///
    /// **索引约定：bit 0 / bit 1 / bit 2 依次选择 x / y / z 取 `min` 还是 `max`，
    /// 置位取 `max`。** 于是 `corner(0)` 是 `min`（三位全 0）、`corner(7)` 是
    /// `max`（三位全 1）、`corner(1) = {max.x, min.y, min.z}`、
    /// `corner(2) = {min.x, max.y, min.z}`、`corner(4) = {min.x, min.y, max.z}`。
    ///
    /// 该含义是接口的一部分：后续的网格与包围体代码会按 `for (int i = 0; i < 8; ++i)`
    /// 的顺序遍历角点，改约定等于改语义。
    ///
    /// 索引越界是未定义行为 —— 与 `Point3T::operator[]` 一致，不做边界检查。
    /// 空盒上调用得到由 ±inf / NaN 混成的点，无意义；调用方应先 `is_empty()`。
    [[nodiscard]] constexpr Point3T<Scalar> corner(int index) const noexcept {
        return Point3T<Scalar>{(index & 1) != 0 ? max.x : min.x,
                               (index & 2) != 0 ? max.y : min.y,
                               (index & 4) != 0 ? max.z : min.z};
    }
};

using Box3 = Box3T<double>;
using Box3f = Box3T<float>;

// ---- 运算符 ----

/// 两个角点都相等才算相等（六个标量）。与 Vector / Point / Interval 的约定
/// 一致：`operator!=` 由 C++20 自动生成，不手写。
template <typename Scalar>
[[nodiscard]] constexpr bool operator==(Box3T<Scalar> a, Box3T<Scalar> b) noexcept {
    return a.min == b.min && a.max == b.max;
}

} // namespace GeoCore::linear
