#pragma once

#include <DragonGeo/Core/Numeric.hpp>
#include <DragonGeo/Core/Tolerance.hpp>
#include <DragonGeo/Linear/Box2.hpp>
#include <DragonGeo/Linear/Coordinate2.hpp>
#include <DragonGeo/Linear/Point2.hpp>
#include <DragonGeo/Linear/Vector2.hpp>

namespace DragonGeo::Linear {

/// 二维有向包围盒（OBB）：一个坐标系（标架）加两个**非负**半轴长度。
///
/// 语义与 `OrientedBox3T` 完全一致（标架的局部盒子 `[-HalfExtent, +HalfExtent]` 经 `Coordinate` 送到父坐标系；**没有「空」的概念**；有向盒必须有一个标架，因此
/// 没有默认构造），差异仅在维度。二维与三维的这些约定是**同一套**：本文件里每一条都可以在 `OrientedBox3.hpp` 找到对应的那一条，反之亦然。
///
/// 两处由维度带来的必然差异只有：
///
/// - `corner` 有 4 个索引（bit 0 / bit 1 依次选择 x / y），约定与 `Box2T::corner` 完全一致； - `ToAxisAligned` 返回 `Box2T`，极值取自四个角。
///
/// **求交与合并尚未提供**（`intersects` / `merged`）：按 spec §5.2，求交归后续的 `query` 层。三维同此。
///
/// `Coordinate` 是强不变量类型 `Coordinate2T`（私有构造 + 校验过的工厂）。 `HalfExtent` 的非负是弱不变量，`expanded` 是把它修回来的公开路径。
///
/// 所有容差都由调用者显式传入，默认值为 `Core::ToleranceT<Scalar>{}`，函数体内不硬编码阈值。默认容差按 double 定标；float 实例化请显式传入与该精度相称的容差。
template <typename Scalar> struct OrientedBox2T {
    using ScalarType = Scalar;

    /// 标架：原点即盒中心，两根轴即两条棱的方向。
    Coordinate2T<Scalar> Coordinate;

    /// 两个方向的半轴长度，逐分量为非负。
    Vector2T<Scalar> HalfExtent{};

    /// 盒中心，即标架原点。
    ///
    /// `Box2T::Center()` 对空盒返回 NaN（「没有中心」的哨兵）；本类型没有空，永远有中心 —— 差别来自两个类型的定义域，不是实现风格的差异。
    [[nodiscard]] constexpr Point2T<Scalar> Center() const noexcept { return Coordinate.Origin(); }

    /// 闭包含：把点送进局部坐标，再逐轴比较 `|local.i| <= HalfExtent.i`，边界算在内。**容差加在比较的右侧**： `|local.i| <= HalfExtent.i + tolerance.Resolve(HalfExtent.i)`。
    ///
    /// 为什么这里要容差而 `Box2T::contains` 不要：点经 `ToLocal` 要先减去原点、再与两根轴做点积（四次乘加），一个恰好落在角点上的点在往返之后可能偏出约 1 ulp —— 没有容差时它判「不在盒里」，而它恰恰是盒子自己的角点。
    /// 默认容差（相对项 1e-9）远大于这个浮点噪声、又远小于任何几何特征量级。 **测试里有一格把这个误差的符号钉住为正**（原点 (100,50)、x 轴 45°、半轴 (1,1) 的盒：角 1 与角 2 在零容差下判在盒外、默认容差下判在盒内，
    /// 而角 0 与角 3 的误差为负 —— 同一个标架下两种符号并存）。那一格的余量约 36 ulp，**余量是刻意留的**：断言一旦绑在舍入误差的符号上，余量就是它全部的安全带（三维同名用例把原点取到量级 1000，是同一手法）。
    ///
    /// 容差的**参考量级取右端**（半轴分量），而不是左端（局部坐标）：接口上写的就是「加在比较的右侧」，参考量级跟着那一侧走；边界附近两者只差一个 ulp 量级，这是**规格**上的选择而非可观测的差异（与三维逐条相同）。
    ///
    /// 半轴含 NaN、或点含 NaN 时不特判：比较碰上 NaN 一律为假，于是「什么都装不下」。这是诚实的答案 —— 与 `ToAxisAligned()` 对 NaN 给出规范空盒同源（见该成员）。
    ///
    /// **半轴为 ±inf 同样不特判**：`+inf` 让右端恒大于任何有限的 `|local.i|`，于是「什么都装得下」；`-inf` 让右端算出 NaN，于是「什么都装不下」。两者都是**契约外**输入：`+inf` 与 `ToAxisAligned()` 给出的结果没有
    /// 几何意义上的对应物（那一边的结果由标架决定）—— 见 `ToAxisAligned` 与 `expanded` 的非有限半轴一节。
    [[nodiscard]] constexpr bool Contains(Point2T<Scalar> point, Core::ToleranceT<Scalar> tolerance = {}) const noexcept {
        const Point2T<Scalar> local = Coordinate.ToLocal(point);
        return Core::AbsoluteValue(local.X) <= HalfExtent.X + tolerance.Resolve(HalfExtent.X)
            && Core::AbsoluteValue(local.Y) <= HalfExtent.Y + tolerance.Resolve(HalfExtent.Y);
    }

    /// 第 `index` 个角点，`index` 取 0..3。
    ///
    /// **索引约定与 `Box2T::corner` 完全一致**：bit 0 / bit 1 依次选择 x / y 取 `-HalfExtent` 还是 `+HalfExtent`，置位取 `+HalfExtent`（即 `Box2T`
    /// 那边的 `max` 一角）。先在**局部**坐标取角，再用 `Coordinate.ToParent` 送到父坐标系。
    ///
    /// 越界是未定义行为（与 `Box2T::corner` 一致，不做边界检查）。
    [[nodiscard]] constexpr Point2T<Scalar> Corner(int index) const noexcept {
        const Point2T<Scalar> local{(index & 1) != 0 ? HalfExtent.X : -HalfExtent.X, (index & 2) != 0 ? HalfExtent.Y : -HalfExtent.Y};
        return Coordinate.ToParent(local);
    }

    /// 紧致的轴对齐包围盒。
    ///
    /// 取**四个角**在父坐标系下的实际 min/max；**结果为空则返回规范空盒** （`Box2T` 的不变量：凡是产出空盒的运算都必须给出规范形式 —— 与 `Box2T::expanded` / `IntervalT` 同一条规则）。出口那一步规范化不是装饰：
    /// 四个角的极值在非有限输入下会留下**非规范**的端点组合（实测 `min = (-inf,+inf)` / `max = (+inf,-inf)`：`IsEmpty()` 为真而 `!= Empty()`）。
    ///
    /// 两种常见替代写法都是错的，且**错的方向相反**，测试对两者都有检出能力 （各由不同的断言杀死）： - 用「包围球半径」（半轴向量长度 `|HalfExtent|`；边长 2 的正方形绕原点转 45° 时是 √2）：旋转后**过松**；
    /// - 只把中心变换过去、半轴沿用局部值（同一例子里是 1）：**过紧** —— 盒子框不住自己的角点，是更危险的那种错。正确结果是 √2 ≈ 1.41421356，与三维同值。
    ///
    /// 起点用 `Box2T::Empty()`（其 `min = +inf` / `max = -inf`，使首轮比较自然成立）。**不能改用零盒起步**：整体落在正卦限的盒子会得到 `min = 0`。
    ///
    /// **±inf 半轴是契约外输入**（`Expanded(+inf)` 是一条可达路径）：角点里会出现 NaN 或 ±inf，极值取决于标架，结果不保证几何意义 —— 可能为空（此时按上面的规则给规范空盒）、也可能是整个空间。**不要**拿它跟
    /// `Box2T::Expanded(+inf)`（那边给**整个空间**）对齐：同一个词在两种类型上没有对等的语义，因为「无限大的有向盒」本身没有良定义。本成员只保证一件事：返回的永远是一个守规矩的 `Box2T`。
    ///
    /// 半轴含 NaN 时四个角全是 NaN、逐分量比较一律为假，min/max 各自留在初始的 ±inf 上（即规范空盒）——「不含任何点」的诚实答案，与 `contains` 对 NaN 恒假同源。
    [[nodiscard]] constexpr Box2T<Scalar> ToAxisAligned() const noexcept {
        Box2T<Scalar> result = Box2T<Scalar>::Empty();
        for (int i = 0; i < 4; ++i) {
            const Point2T<Scalar> c = Corner(i);
            result.Min.X = c.X < result.Min.X ? c.X : result.Min.X;
            result.Min.Y = c.Y < result.Min.Y ? c.Y : result.Min.Y;
            result.Max.X = c.X > result.Max.X ? c.X : result.Max.X;
            result.Max.Y = c.Y > result.Max.Y ? c.Y : result.Max.Y;
        }
        return result.IsEmpty() ? Box2T<Scalar>::Empty() : result;
    }

    /// 逐分量把半轴加上 amount 并**夹到非负**：`max(0, HalfExtent.i + amount)`。
    ///
    /// 与 `Box2T::expanded` 的对照：那边的负 amount 收缩过头时产出**规范空盒** （空集的测度是 0）；这里没有空可以落回，语义是逐分量夹取 —— 于是
    /// `Expanded(-10)` 是一个半轴全 0 的**退化盒（一个点）**，`ToAxisAligned()` 给出 `[center, center]`、`IsEmpty()` 为假、`Contains(center)` 为真。
    /// 两条规则不同不是疏漏，是两个类型的定义域不同（与三维逐条相同）。
    ///
    /// 夹取针对的是**和**（`HalfExtent.i + amount`），不是 amount 的符号：聚合初始化造出的负半轴（非法状态）经 `Expanded(0.5)` 也会被修回非负。放负半轴过去会让 `ToAxisAligned()` 给出一个倒置的、悄悄错的盒子 —— 角点按
    /// `±HalfExtent` 取、极值对 `|HalfExtent|` 成立，而 `contains` 用 `|local.i| <= HalfExtent.i` 对负半轴恒假，同一个盒子的两个说法互相矛盾。
    ///
    /// **非有限半轴的三种形态**（都不特判，只把下游规定清楚）：
    ///
    /// - `amount` 为 NaN（或半轴已是 NaN）：结果逐分量为 NaN，**不**静默夹成 0 —— 本类型没有空盒可以落回，把非有限输入伪装成一个完全正常的退化盒比留下 NaN 更危险。下游是确定的：`contains` 全假、`ToAxisAligned()`
    /// 给出规范空盒（见各自的文档）； - `amount = +inf`：这是一条**可达路径**（有限盒 + inf 增量），半轴全变成 +inf。这样的盒 `contains` **什么都收**，而 `ToAxisAligned()` 的结果
    /// 由标架决定（角点含 NaN / ±inf：可能为空 —— 此时按上面的规则给**规范** 空盒 —— 也可能是整个平面），没有几何意义。**不要**拿它跟 `Box2T::Expanded(+inf)` 对齐：那边给出的是**整个空间** `[-inf,+inf]²`，
    /// 而「无限大的有向盒」没有良定义（见 `ToAxisAligned` 的契约外输入一节）； - `amount = -inf`：逐分量算出 `-inf`，夹取后全是 0 —— 与「收缩过头」同一条规则，不再单列。
    ///
    /// 标架原样保留 —— 本运算只动半轴。
    [[nodiscard]] constexpr OrientedBox2T Expanded(Scalar amount) const noexcept {
        const Scalar x = HalfExtent.X + amount;
        const Scalar y = HalfExtent.Y + amount;
        return OrientedBox2T{Coordinate, Vector2T<Scalar>{x < Scalar{0} ? Scalar{0} : x, y < Scalar{0} ? Scalar{0} : y}};
    }
};

using OrientedBox2 = OrientedBox2T<double>;
using OrientedBox2f = OrientedBox2T<float>;

// ---- 运算符 ----

/// **五个**标量都相等才算相等（`Coordinate` 的原点 + 两根轴，加 `HalfExtent` 的两个分量）。与 Vector / Point / Box / Coordinate 的约定一致：自由函数，
/// `operator!=` 由 C++20 自动生成，不手写；逐分量精确比较，不带容差。与三维同一条约定（两个头文件独立编写，这一条两边各自有测试）。
template <typename Scalar> [[nodiscard]] constexpr bool operator==(const OrientedBox2T<Scalar>& a, const OrientedBox2T<Scalar>& b) noexcept {
    return a.Coordinate == b.Coordinate && a.HalfExtent == b.HalfExtent;
}

} // namespace DragonGeo::Linear
