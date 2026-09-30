#pragma once

#include <GeoCore/core/Numeric.hpp>
#include <GeoCore/core/Tolerance.hpp>
#include <GeoCore/linear/Box3.hpp>
#include <GeoCore/linear/Coordinate3.hpp>
#include <GeoCore/linear/Point3.hpp>
#include <GeoCore/linear/Vector3.hpp>

namespace GeoCore::linear {

/// 三维有向包围盒（OBB）：一个坐标系（标架）加三个**非负**半轴长度。
///
/// 盒子是标架的局部盒子 `[-half_extent, +half_extent]` 经 `frame` 送到父坐标系
/// 后的像：中心在 `frame.origin()`，三条棱沿三根轴。与 `Box3T` 一样是**闭**盒
/// （边界算在内），但两个类型的定义域不同，几条规则也**刻意不同**：
///
///   - **本类型没有「空」的概念。** 半轴恒非负，退化只到「一个点」（半轴全 0），
///     而那个点仍然是一个合法的有向盒。于是 `Box3T` 的「空进空出」「凡是产出空
///     都必须给出规范形式」「规范化优先于恒等律」在这里**没有对应物**，不要
///     照搬 —— 最直接的例子是 `expanded`：那边收缩过头产出规范空盒，这边逐分量
///     夹到 0（见该成员）。
///   - `contains` **接受容差**，`Box3T::contains` 不接受。理由见该成员：点经
///     `to_local` 往返一次就会偏出 1 ulp 量级，没有容差时「角点不在自己所在的
///     盒里」。两个类型的理由都写在各自的成员上，不要为了「一致」抹平。
///
/// `frame` 是**强不变量**类型 `Coordinate3T`（私有构造 + 校验过的工厂），于是
/// 「有向盒一定挂在一个正交单位标架上」是类型层面的保证，本类型不做第二次校验。
/// `half_extent` 的非负则是**弱不变量**：聚合初始化随时能写出负数，`expanded`
/// 是把它修回来的公开路径（见该成员的文档）。
///
/// 本类型是聚合体（`struct` + 两个公开数据成员，不声明任何构造函数），
/// `OrientedBox3T{frame, half_extent}` 是全部的构造方式。**没有默认构造** ——
/// `Coordinate3T` 没有，于是「有向盒必须有一个标架」不需要运行期检查。
///
/// 所有容差都由调用者显式传入，默认值来自 `core::Tolerance`，函数体内不硬编码
/// 阈值。默认容差按 double 定标；float 实例化请显式传入与该精度相称的容差。
template <typename Scalar>
struct OrientedBox3T {
    using scalar_type = Scalar;

    /// 标架：原点即盒中心，三根轴即三条棱的方向。
    Coordinate3T<Scalar> frame;

    /// 三个方向的半轴长度，逐分量为非负。
    Vector3T<Scalar> half_extent{};

    /// 盒中心，即标架原点。
    ///
    /// `Box3T::center()` 对空盒返回 NaN（「没有中心」的哨兵）；本类型没有空，
    /// 永远有中心 —— 差别来自两个类型的定义域，不是实现风格的差异。
    [[nodiscard]] constexpr Point3T<Scalar> center() const noexcept { return frame.origin(); }

    /// 闭包含：把点送进局部坐标，再逐轴比较 `|local.i| <= half_extent.i`，
    /// 边界算在内。**容差加在比较的右侧**：
    /// `|local.i| <= half_extent.i + tolerance.resolve(half_extent.i)`。
    ///
    /// 为什么这里要容差而 `Box3T::contains` 不要：点经 `to_local` 要先减去原点、
    /// 再与三根轴做点积（六次乘加），一个恰好落在角点上的点在往返之后可能偏出
    /// 约 1 ulp —— 没有容差时它判「不在盒里」，而它恰恰是盒子自己的角点。
    /// 默认容差（相对项 1e-9）远大于这个浮点噪声、又远小于任何几何特征量级。
    /// **测试里有一格把这个误差的符号钉住为正**：z 轴绕 (1,1,1) 补全、半轴
    /// (1,2,3) 的盒，八个角在零容差下全判在盒外、在默认容差下全判在盒内。
    ///
    /// 容差的**参考量级取右端**（半轴分量），而不是左端（局部坐标）：接口上写
    /// 的就是「加在比较的右侧」，参考量级跟着那一侧走；边界附近两者只差一个
    /// ulp 量级，这是**规格**上的选择而非可观测的差异。
    ///
    /// 半轴含 NaN、或点含 NaN 时不特判：比较碰上 NaN 一律为假，于是「什么都装
    /// 不下」。这是诚实的答案 —— 与 `to_axis_aligned()` 对 NaN 给出规范空盒
    /// 同源（见该成员）。
    [[nodiscard]] constexpr bool contains(Point3T<Scalar> point,
                                          core::Tolerance tolerance = {}) const noexcept {
        const Point3T<Scalar> local = frame.to_local(point);
        return core::absolute_value(local.x) <= half_extent.x + tolerance.resolve(half_extent.x)
            && core::absolute_value(local.y) <= half_extent.y + tolerance.resolve(half_extent.y)
            && core::absolute_value(local.z) <= half_extent.z + tolerance.resolve(half_extent.z);
    }

    /// 第 `index` 个角点，`index` 取 0..7。
    ///
    /// **索引约定与 `Box3T::corner` 完全一致**：bit 0 / bit 1 / bit 2 依次选择
    /// x / y / z 取 `-half_extent` 还是 `+half_extent`，置位取 `+half_extent`
    /// （即 `Box3T` 那边的 `max` 一角）。先在**局部**坐标取角，再用
    /// `frame.to_parent` 送到父坐标系 —— 顺序反了（先变换后取角）是另一回事，
    /// 这里的局部盒子以标架原点为中心。
    ///
    /// 「索引位与 `Box3T` 相同」是接口的一部分：`to_axis_aligned()` 与后续的
    /// 网格代码都按 `for (int i = 0; i < 8; ++i)` 遍历角点。越界是未定义行为
    /// （与 `Box3T::corner` 一致，不做边界检查）。
    [[nodiscard]] constexpr Point3T<Scalar> corner(int index) const noexcept {
        const Point3T<Scalar> local{(index & 1) != 0 ? half_extent.x : -half_extent.x,
                                    (index & 2) != 0 ? half_extent.y : -half_extent.y,
                                    (index & 4) != 0 ? half_extent.z : -half_extent.z};
        return frame.to_parent(local);
    }

    /// 紧致的轴对齐包围盒。
    ///
    /// 取**八个角**在父坐标系下的实际 min/max。两种常见替代写法都是错的，且
    /// **错的方向相反**，测试对两者都有检出能力（各由不同的断言杀死）：
    ///   - 用「包围球半径」（半轴向量长度 `|half_extent|`；边长 2 的立方体绕 z
    ///     转 45° 时是 √3 ≈ 1.732）：旋转后**过松**；
    ///   - 只把中心变换过去、半轴沿用局部值（同一例子里是 1）：**过紧** ——
    ///     盒子框不住自己的角点，是更危险的那种错。
    /// 正确结果是 √2 ≈ 1.41421356。
    ///
    /// 起点用 `Box3T::empty()`（其 `min = +inf` / `max = -inf`，使首轮比较自然
    /// 成立）。**不能改用零盒起步**：整体落在正卦限的盒子会得到 `min = 0`。
    ///
    /// 半轴含 NaN 时八个角全是 NaN、逐分量比较一律为假，每一个 min/max 分量都
    /// 留在初始的 ±inf 上，于是返回**规范空盒** —— 「不含任何点」的诚实答案，
    /// 与 `contains` 对 NaN 恒假同源。`expanded(NaN)` 因此也有确定的下游。
    [[nodiscard]] constexpr Box3T<Scalar> to_axis_aligned() const noexcept {
        Box3T<Scalar> result = Box3T<Scalar>::empty();
        for (int i = 0; i < 8; ++i) {
            const Point3T<Scalar> c = corner(i);
            result.min.x = c.x < result.min.x ? c.x : result.min.x;
            result.min.y = c.y < result.min.y ? c.y : result.min.y;
            result.min.z = c.z < result.min.z ? c.z : result.min.z;
            result.max.x = c.x > result.max.x ? c.x : result.max.x;
            result.max.y = c.y > result.max.y ? c.y : result.max.y;
            result.max.z = c.z > result.max.z ? c.z : result.max.z;
        }
        return result;
    }

    /// 逐分量把半轴加上 amount 并**夹到非负**：`max(0, half_extent.i + amount)`。
    ///
    /// 与 `Box3T::expanded` 的对照：那边的负 amount 收缩过头时产出**规范空盒**
    /// （空集的测度是 0）；这里没有空可以落回，语义是逐分量夹取 —— 于是
    /// `expanded(-10)` 是一个半轴全 0 的**退化盒（一个点）**，`to_axis_aligned()`
    /// 给出 `[center, center]`、`is_empty()` 为假、`contains(center)` 为真。
    /// 两条规则不同不是疏漏，是两个类型的定义域不同（见类文档）。
    ///
    /// 夹取针对的是**和**（`half_extent.i + amount`），不是 amount 的符号：聚合
    /// 初始化造出的负半轴（非法状态）经 `expanded(0.5)` 也会被修回非负。放负
    /// 半轴过去会让 `to_axis_aligned()` 给出一个倒置的、悄悄错的盒子 —— 角点按
    /// `±half_extent` 取、极值对 `|half_extent|` 成立，而 `contains` 用
    /// `|local.i| <= half_extent.i` 对负半轴恒假，同一个盒子的两个说法互相矛盾。
    ///
    /// `amount` 为 NaN（或半轴已是 NaN）时结果逐分量为 NaN，**不**静默夹成 0：
    /// 本类型没有空盒可以落回，把非有限输入伪装成一个完全正常的退化盒比留下
    /// NaN 更危险。NaN 的下游是确定的：`contains` 全假、`to_axis_aligned()`
    /// 给出规范空盒（见各自的文档）。
    ///
    /// 标架原样保留 —— 本运算只动半轴。
    [[nodiscard]] constexpr OrientedBox3T expanded(Scalar amount) const noexcept {
        const Scalar x = half_extent.x + amount;
        const Scalar y = half_extent.y + amount;
        const Scalar z = half_extent.z + amount;
        return OrientedBox3T{frame,
                             Vector3T<Scalar>{x < Scalar{0} ? Scalar{0} : x,
                                              y < Scalar{0} ? Scalar{0} : y,
                                              z < Scalar{0} ? Scalar{0} : z}};
    }
};

using OrientedBox3 = OrientedBox3T<double>;
using OrientedBox3f = OrientedBox3T<float>;

// ---- 运算符 ----

/// **七个**标量都相等才算相等（`frame` 的原点 + 三根轴，加 `half_extent` 的三个
/// 分量）。与 Vector / Point / Box / Coordinate 的约定一致：自由函数，
/// `operator!=` 由 C++20 自动生成，不手写；逐分量精确比较，不带容差 —— 两个
/// 「几乎相同」的有向盒不相等，这一点是刻意的。
template <typename Scalar>
[[nodiscard]] constexpr bool operator==(const OrientedBox3T<Scalar>& a,
                                        const OrientedBox3T<Scalar>& b) noexcept {
    return a.frame == b.frame && a.half_extent == b.half_extent;
}

} // namespace GeoCore::linear
