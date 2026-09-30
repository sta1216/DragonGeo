#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <limits>
#include <type_traits>
#include <utility>

#include <DragonGeo/Core/Tolerance.hpp>
#include <DragonGeo/Linear/Box3.hpp>
#include <DragonGeo/Linear/Coordinate3.hpp>
#include <DragonGeo/Linear/OrientedBox3.hpp>
#include <DragonGeo/Linear/Point3.hpp>
#include <DragonGeo/Linear/UnitVector3.hpp>
#include <DragonGeo/Linear/Vector3.hpp>

using Catch::Approx;

using DragonGeo::Core::Tolerance;
using DragonGeo::Linear::Box3;
using DragonGeo::Linear::Box3T;
using DragonGeo::Linear::Coordinate3;
using DragonGeo::Linear::Coordinate3T;
using DragonGeo::Linear::OrientedBox3;
using DragonGeo::Linear::OrientedBox3f;
using DragonGeo::Linear::OrientedBox3T;
using DragonGeo::Linear::Point3;
using DragonGeo::Linear::Point3T;
using DragonGeo::Linear::UnitVector3;
using DragonGeo::Linear::UnitVector3T;
using DragonGeo::Linear::Vector3;
using DragonGeo::Linear::Vector3T;

namespace {

/// 世界 z 轴。与 coordinate3Test.cpp / transform3Test.cpp 同一种写法。
const UnitVector3 zAxis = UnitVector3::FromNormalizedUnchecked(Vector3{0.0, 0.0, 1.0});
const UnitVector3 ex = UnitVector3::FromNormalizedUnchecked(Vector3{1.0, 0.0, 0.0});
const UnitVector3 ey = UnitVector3::FromNormalizedUnchecked(Vector3{0.0, 1.0, 0.0});

/// 轴取标准基、原点在 `origin` 的标架。
///
/// `Coordinate3T::Identity()` 的原点固定在 (0,0,0)，而本文件多处要的是
/// 「轴不动、只挪原点」—— 那是把「原点参与运算」从「轴参与运算」里分离出来的
/// 唯一办法。标准基必然通过校验（长度 1、两两正交、右手），失败即测试装置
/// 自身出错，所以这里照常 REQUIRE。
Coordinate3 FrameAt(Point3 origin) {
    const auto frame = Coordinate3::FromAxes(origin, ex, ey, zAxis);
    REQUIRE(frame.has_value());
    return *frame;
}

} // namespace

TEST_CASE("the float alias really is the float instantiation",
          "[linear][orientedbox3]") {
    STATIC_REQUIRE(std::is_same_v<OrientedBox3f, OrientedBox3T<float>>);
    STATIC_REQUIRE(std::is_same_v<OrientedBox3, OrientedBox3T<double>>);
    STATIC_REQUIRE(std::is_same_v<OrientedBox3T<float>::ScalarType, float>);
    STATIC_REQUIRE(std::is_same_v<OrientedBox3T<double>::ScalarType, double>);

    // 聚合性是 Interfaces 明写的形态（`struct { frame; HalfExtent; }`），也是
    // 本文件每一处初始化在用的东西。加一个构造函数（哪怕 `= default`）就悄悄
    // 丢掉它 —— 成员函数不影响聚合性，构造函数影响。
    STATIC_REQUIRE(std::is_aggregate_v<OrientedBox3T<double>>);
    STATIC_REQUIRE(std::is_aggregate_v<OrientedBox3T<float>>);

    // 没有默认构造：`frame` 是强不变量类型 `Coordinate3T`，它没有默认构造，
    // 于是「有向盒必须有一个标架」是**类型层面**的要求，不需要运行期检查。
    // 给 `frame` 补一个恒等标架的默认成员初始化值会让下面两条由真变假。
    STATIC_REQUIRE(!std::is_default_constructible_v<OrientedBox3T<double>>);
    STATIC_REQUIRE(!std::is_default_constructible_v<OrientedBox3T<float>>);

    // 两个数据成员的名字与类型本身就是接口（下游按名字读它们）。
    STATIC_REQUIRE(std::is_same_v<decltype(OrientedBox3T<double>::Coordinate), Coordinate3T<double>>);
    STATIC_REQUIRE(std::is_same_v<decltype(OrientedBox3T<double>::HalfExtent), Vector3T<double>>);
    STATIC_REQUIRE(std::is_same_v<decltype(OrientedBox3T<float>::Coordinate), Coordinate3T<float>>);
    STATIC_REQUIRE(std::is_same_v<decltype(OrientedBox3T<float>::HalfExtent), Vector3T<float>>);
}

TEST_CASE("rotating a box grows its axis-aligned bounding box",
          "[linear][orientedbox3]") {
    const auto frame = Coordinate3::FromZAxis(
        Point3{0.0, 0.0, 0.0},
        UnitVector3::FromNormalizedUnchecked(Vector3{0.0, 0.0, 1.0}));
    REQUIRE(frame.has_value());

    const OrientedBox3 box{*frame, Vector3{1.0, 1.0, 1.0}};   // 边长 2 的立方体
    const Box3 aabb = box.ToAxisAligned();

    // 未旋转时紧包围盒就是自身
    CHECK(aabb.Min.X == Approx(-1.0));
    CHECK(aabb.Max.X == Approx(1.0));

    // 绕 z 转 45° 后，x/y 方向的紧包围盒必须扩展到 √2
    const auto x45 = Vector3{1.0, 1.0, 0.0}.Normalized();
    const auto y45 = Vector3{-1.0, 1.0, 0.0}.Normalized();
    REQUIRE(x45.has_value());
    REQUIRE(y45.has_value());

    const auto rotatedFrame = Coordinate3::FromAxes(
        Point3{0.0, 0.0, 0.0}, *x45, *y45, zAxis);
    REQUIRE(rotatedFrame.has_value());

    const OrientedBox3 rotated{*rotatedFrame, Vector3{1.0, 1.0, 1.0}};
    const Box3 rotatedAabb = rotated.ToAxisAligned();

    CHECK(rotatedAabb.Max.X == Approx(std::sqrt(2.0)).margin(1e-12));
    CHECK(rotatedAabb.Min.X == Approx(-std::sqrt(2.0)).margin(1e-12));
    CHECK(rotatedAabb.Max.X > aabb.Max.X);   // 真的变大了
    CHECK(rotatedAabb.Max.Z == Approx(1.0)); // z 方向不变

    // 两端都要真的变大：只钉 max 那一侧时，「min 沿用局部值」的实现无处可藏
    // 的另一半 —— 45° 旋转把 x/y 对称化，y 方向也必须扩展到 √2。
    CHECK(rotatedAabb.Min.X < aabb.Min.X);
    CHECK(rotatedAabb.Max.Y == Approx(std::sqrt(2.0)).margin(1e-12));
    CHECK(rotatedAabb.Min.Y == Approx(-std::sqrt(2.0)).margin(1e-12));
    CHECK(rotatedAabb.Min.Z == Approx(-1.0));
    // 尺寸（派生量，独立于上面那两条端点断言）：2√2，而不是包围球直径 2√3。
    CHECK(rotatedAabb.Extent().X == Approx(2.0 * std::sqrt(2.0)).margin(1e-12));

    // **定义性质**：八个角都在紧包围盒里。这一组是「过紧」写法（只把中心变换
    // 过去、半轴沿用局部值 = 1）的直接死亡证明 —— 它会让 (0,±√2,±1) 这类角点
    // 落到盒外；上面那两条 √2 断言则是同一个变异体的另一种证据。
    for (int i = 0; i < 8; ++i) {
        CHECK(rotatedAabb.Contains(rotated.Corner(i)));
        CHECK(aabb.Contains(box.Corner(i)));
    }
}

TEST_CASE("a non-cubic oriented box pins every axis separately",
          "[linear][orientedbox3]") {
    // 上面的用例是立方体 —— 半轴三个分量相等，于是任何把 x/y/z 弄混、
    // 或把 corner 的索引位弄反的实现都看不出来。半轴取 (1,2,3) 之后，
    // 每个轴各自可辨。
    const auto frame = Coordinate3::FromZAxis(
        Point3{0.0, 0.0, 0.0},
        UnitVector3::FromNormalizedUnchecked(Vector3{0.0, 0.0, 1.0}));
    REQUIRE(frame.has_value());

    // 先把标架本身钉住：`FromZAxis` 对世界 z 轴的补全**不是**单位标架，而是
    // 绕 z 的一个 90° 旋转（x = (1,0,0) × z = (0,-1,0)，y = z × x = (1,0,0)）。
    // 下面每一格的期望值都由它算出 —— 局部 (a,b,c) 送到世界是 (b,-a,c)。
    CHECK(frame->XAxis().X() == 0.0);
    CHECK(frame->XAxis().Y() == -1.0);
    CHECK(frame->XAxis().Z() == 0.0);
    CHECK(frame->YAxis().X() == 1.0);
    CHECK(frame->YAxis().Y() == 0.0);
    CHECK(frame->YAxis().Z() == 0.0);
    CHECK(frame->ZAxis().Z() == 1.0);

    const OrientedBox3 box{*frame, Vector3{1.0, 2.0, 3.0}};
    const Box3 aabb = box.ToAxisAligned();

    CHECK(aabb.Min == Point3{-2.0, -1.0, -3.0});
    CHECK(aabb.Max == Point3{2.0, 1.0, 3.0});

    // corner 的 bit0/bit1/bit2 依次选 x/y/z，置位取 max；先在**局部**取角，
    // 再经标架送到世界。0 与 7 是「全 min」「全 max」，三个位的两两互换在它们
    // 身上看不出来：1/2/4 各置一位、3/5/6 各置两位，每一位都独立钉死。
    CHECK(box.Corner(0) == Point3{-2.0, 1.0, -3.0});
    CHECK(box.Corner(1) == Point3{-2.0, -1.0, -3.0});
    CHECK(box.Corner(2) == Point3{2.0, 1.0, -3.0});
    CHECK(box.Corner(3) == Point3{2.0, -1.0, -3.0});
    CHECK(box.Corner(4) == Point3{-2.0, 1.0, 3.0});
    CHECK(box.Corner(5) == Point3{-2.0, -1.0, 3.0});
    CHECK(box.Corner(6) == Point3{2.0, 1.0, 3.0});
    CHECK(box.Corner(7) == Point3{2.0, -1.0, 3.0});

    CHECK(box.Center() == Point3{0.0, 0.0, 0.0});

    // 角点全部落在紧包围盒里（定义性质）。半轴 (1,2,3) 的三个值互不相同，
    // 任何一个轴被换到别的轴上都会让上面若干条失败。
    for (int i = 0; i < 8; ++i) {
        CHECK(aabb.Contains(box.Corner(i)));
    }
}

TEST_CASE("oriented box containment follows the frame", "[linear][orientedbox3]") {
    const auto x45 = Vector3{1.0, 1.0, 0.0}.Normalized();
    const auto y45 = Vector3{-1.0, 1.0, 0.0}.Normalized();
    REQUIRE(x45.has_value());
    REQUIRE(y45.has_value());

    const auto frame = Coordinate3::FromAxes(Point3{1.0, 1.0, 0.0}, *x45, *y45, zAxis);
    REQUIRE(frame.has_value());

    const OrientedBox3 box{*frame, Vector3{1.0, 1.0, 1.0}};

    CHECK(box.Contains(Point3{1.0, 1.0, 0.0}));          // 中心
    CHECK(box.Contains(Point3{1.0, 1.0, 1.0}));          // 局部 z 的面上
    CHECK_FALSE(box.Contains(Point3{1.0, 1.0, 1.5}));

    // 角落方向：局部 (1,1,1) 在世界里是 rotatedFrame 作用后的点。
    // 先确认「局部 (2,0,0)」这个明显在外面的点被拒绝，再确认边界点被接受。
    CHECK_FALSE(box.Contains(Point3{1.0 + 2.0 * x45->X(), 1.0 + 2.0 * x45->Y(), 0.0}));

    // expanded 之后同一个点应当被接受。
    CHECK(box.Expanded(1.5).Contains(
        Point3{1.0 + 2.0 * x45->X(), 1.0 + 2.0 * x45->Y(), 0.0}));

    // 「角点必须在盒里」—— 对**旋转过的**标架，这一组才是有内容的：一个
    // 忘掉 `ToLocal`（拿世界坐标直接与半轴比）的实现会在 (1,1,1) 那一格上
    // 碰巧通过，却把 x45 方向的角点（世界 (1+√2, 1, 1)）判在盒外。
    // 顺带证明默认容差覆盖了往返一次的舍入（偏出约 1 ulp）。
    for (int i = 0; i < 8; ++i) {
        CHECK(box.Contains(box.Corner(i)));
    }
}

TEST_CASE("expanded never produces a negative half extent",
          "[linear][orientedbox3]") {
    const auto frame = Coordinate3::FromZAxis(
        Point3{0.0, 0.0, 0.0},
        UnitVector3::FromNormalizedUnchecked(Vector3{0.0, 0.0, 1.0}));
    REQUIRE(frame.has_value());

    const OrientedBox3 box{*frame, Vector3{1.0, 2.0, 3.0}};

    CHECK(box.Expanded(1.0).HalfExtent == Vector3{2.0, 3.0, 4.0});
    CHECK(box.Expanded(-0.5).HalfExtent == Vector3{0.5, 1.5, 2.5});

    // 收缩过头：逐分量夹到 0，而不是留下负的半轴 —— 负半轴会让
    // ToAxisAligned() 给出一个倒置的、悄悄错的盒子。
    CHECK(box.Expanded(-10.0).HalfExtent == Vector3{0.0, 0.0, 0.0});

    // 三维都为 0 时，紧包围盒退化成一个点（即中心），不是空盒。
    const Box3 degenerate = box.Expanded(-10.0).ToAxisAligned();
    CHECK_FALSE(degenerate.IsEmpty());
    CHECK(degenerate.Min == Point3{0.0, 0.0, 0.0});
    CHECK(degenerate.Max == Point3{0.0, 0.0, 0.0});

    // 标架原样保留：`expanded` 只动半轴（四个成员都要真的搬过去）。
    CHECK(box.Expanded(1.0).Coordinate == box.Coordinate);
    CHECK(box.Expanded(-10.0).Coordinate == box.Coordinate);
    CHECK(box.Expanded(0.0) == box);            // 零增量是恒等

    // 逐分量的夹取证据：让「只有 x 被夹」「只有 y 被夹」「只有 z 被夹」三种
    // 半残实现各自现形。三个和恰好落在 0 两侧的不同位置。
    CHECK(box.Expanded(-1.5).HalfExtent == Vector3{0.0, 0.5, 1.5});    // x 夹、y/z 不夹
    CHECK(box.Expanded(-2.0).HalfExtent == Vector3{0.0, 0.0, 1.0});    // x/y 夹、z 不夹
    const OrientedBox3 tall{*frame, Vector3{3.0, 1.0, 2.0}};
    CHECK(tall.Expanded(-1.5).HalfExtent == Vector3{1.5, 0.0, 0.5});   // y 夹、x/z 不夹

    // 半轴本身为负（聚合初始化就能造出的非法状态）时，夹取针对的是**和**
    // （`HalfExtent.i + amount`），不是 amount 的符号。
    const OrientedBox3 inverted{*frame, Vector3{-1.0, 2.0, 3.0}};
    CHECK(inverted.Expanded(0.5).HalfExtent == Vector3{0.0, 2.5, 3.5});

    // `amount` 为 NaN：结果逐分量为 NaN，**不**被静默夹成 0 —— 本类型没有
    // 「空」可以落回，把非有限输入伪装成一个完全正常的退化盒比留下 NaN
    // 更危险。NaN 的下游是确定的（下面两条）：contains 全假、紧包围盒是
    // 规范空盒（角点全是 NaN，逐分量比较一律为假，min/max 各自留在 ±inf）。
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const Vector3 nanHalfExtent = box.Expanded(nan).HalfExtent;
    CHECK(std::isnan(nanHalfExtent.X));
    CHECK(std::isnan(nanHalfExtent.Y));
    CHECK(std::isnan(nanHalfExtent.Z));
    CHECK(box.Expanded(nan).ToAxisAligned() == Box3::Empty());
    CHECK_FALSE(box.Expanded(nan).Contains(Point3{0.0, 0.0, 0.0}));

    // **非有限半轴的第二种形态：±inf。** 与 NaN 一样按「不特判、暴露矛盾」
    // 处理，这里钉住当前行为（语义**不改**）：
    //   - `Expanded(+inf)` 是一条**公开路径**（有限盒 + inf 增量），给出半轴
    //     全 +inf；
    //   - 这样的盒 `contains` **什么都收**（右端 `inf + inf` 恒大于任何有限
    //     的 `|local.i|`），而 `ToAxisAligned()` 给出**规范空盒** —— 角点里
    //     `0 * inf` 是 NaN，八个角全被毒掉。**两个说法互相矛盾**，这正是
    //     「±inf 半轴不是合法状态」的直接后果；
    //   - 兄弟类型 `Box3T::Expanded(+inf)` 承诺的是**整个空间**（`[-inf,+inf]³`）
    //     —— 同一个语义在两个类型上给出相反答案。差别来自本类型**没有空盒**：
    //     那边可以用「全空间」回答，这边只能把矛盾暴露出来。
    const double infinity = std::numeric_limits<double>::infinity();
    const OrientedBox3 unbounded = box.Expanded(infinity);
    CHECK(unbounded.HalfExtent.X == infinity);
    CHECK(unbounded.HalfExtent.Y == infinity);
    CHECK(unbounded.HalfExtent.Z == infinity);
    CHECK(unbounded.Contains(Point3{123.0, -456.0, 789.0}));
    CHECK(unbounded.ToAxisAligned() == Box3::Empty());
    // 反方向：`h.i + (-inf) = -inf`，夹取后全是 0 —— 与「收缩过头」同一条规则。
    CHECK(box.Expanded(-infinity).HalfExtent == Vector3{0.0, 0.0, 0.0});

    // **`ToAxisAligned()` 的出口规范化**：上面那个标架的每一行都含 0
    // （`0 * inf` 把角点毒成 NaN），八个角全 NaN、min/max 留在初始的 ±inf 上，
    // 于是**恰好**是规范空盒 —— 那是运气。一般标架不是这样：实测本用例的
    // `FromZAxis(o, (1,1,1)/√3)` 给出 `min = (+inf,-inf,-inf)` /
    // `max = (-inf,+inf,+inf)`，`IsEmpty()` 为真而 `!= Empty()`。`Box3T`
    // 的不变量是「凡是产出空盒的运算都必须给出规范形式」（`Box3T::expanded`
    // / `IntervalT` 同一条规则），所以出口要显式规范化一次。
    const auto generalZ = Vector3{1.0, 1.0, 1.0}.Normalized();
    REQUIRE(generalZ.has_value());
    const auto generalFrame = Coordinate3::FromZAxis(Point3{0.0, 0.0, 0.0}, *generalZ);
    REQUIRE(generalFrame.has_value());
    const OrientedBox3 generalInf{*generalFrame, Vector3{infinity, infinity, infinity}};
    CHECK(generalInf.ToAxisAligned().IsEmpty());
    CHECK(generalInf.ToAxisAligned() == Box3::Empty());
}

TEST_CASE("corner maps the local box through the frame",
          "[linear][orientedbox3]") {
    // 轴置换标架：xAxis = (0,1,0)、yAxis = (0,0,1)、zAxis = (1,0,0)
    // （右手：x × y = (1,0,0) = z）。分量全是 0/1，全部算术精确。
    // 局部 (a,b,c) 送到世界是 (c,a,b)，于是半轴 (1,2,3) 会被送到世界的
    // z/x/y 方向上 —— 「半轴与轴对错位」的实现在这里无处可藏。
    const auto frame = Coordinate3::FromAxes(
        Point3{2.0, 0.0, -1.0},
        UnitVector3::FromNormalizedUnchecked(Vector3{0.0, 1.0, 0.0}),
        UnitVector3::FromNormalizedUnchecked(Vector3{0.0, 0.0, 1.0}),
        UnitVector3::FromNormalizedUnchecked(Vector3{1.0, 0.0, 0.0}));
    REQUIRE(frame.has_value());

    const OrientedBox3 box{*frame, Vector3{1.0, 2.0, 3.0}};

    // 中心是**标架原点**（不是局部原点，也不是 (0,0,0)）。
    CHECK(box.Center() == Point3{2.0, 0.0, -1.0});

    CHECK(box.Corner(0) == Point3{-1.0, -1.0, -3.0});
    CHECK(box.Corner(1) == Point3{-1.0, 1.0, -3.0});
    CHECK(box.Corner(2) == Point3{-1.0, -1.0, 1.0});
    CHECK(box.Corner(3) == Point3{-1.0, 1.0, 1.0});
    CHECK(box.Corner(4) == Point3{5.0, -1.0, -3.0});
    CHECK(box.Corner(5) == Point3{5.0, 1.0, -3.0});
    CHECK(box.Corner(6) == Point3{5.0, -1.0, 1.0});
    CHECK(box.Corner(7) == Point3{5.0, 1.0, 1.0});
}

TEST_CASE("ToAxisAligned is the tight box of the corners",
          "[linear][orientedbox3]") {
    // 同一个轴置换标架：紧包围盒的每一端都由不同角点取胜，且三轴尺寸
    // (6,2,4) = (2·3, 2·1, 2·2) 是半轴的**置换** —— 轴向互换可辨。
    const auto frame = Coordinate3::FromAxes(
        Point3{2.0, 0.0, -1.0},
        UnitVector3::FromNormalizedUnchecked(Vector3{0.0, 1.0, 0.0}),
        UnitVector3::FromNormalizedUnchecked(Vector3{0.0, 0.0, 1.0}),
        UnitVector3::FromNormalizedUnchecked(Vector3{1.0, 0.0, 0.0}));
    REQUIRE(frame.has_value());

    const OrientedBox3 box{*frame, Vector3{1.0, 2.0, 3.0}};
    const Box3 aabb = box.ToAxisAligned();

    CHECK(aabb.Min == Point3{-1.0, -1.0, -3.0});
    CHECK(aabb.Max == Point3{5.0, 1.0, 1.0});
    CHECK(aabb.Extent() == Vector3{6.0, 2.0, 4.0});

    // 紧：半轴最长者 3、包围球半径 |h| = √14 ≈ 3.74。包围球写法给出的是边长
    // 2√14 ≈ 7.48 的立方体；把半轴直接当世界半宽的写法给出 (2,4,6)。两者都
    // 被上面的 `Extent() == (6,2,4)` 排除，下面三条再从**每轴的实际跨度**
    // 复核一次（不经由 `Extent()` 的实现）。
    CHECK(aabb.Max.X - aabb.Min.X == 6.0);
    CHECK(aabb.Max.Y - aabb.Min.Y == 2.0);
    CHECK(aabb.Max.Z - aabb.Min.Z == 4.0);
    for (int i = 0; i < 8; ++i) {
        CHECK(aabb.Contains(box.Corner(i)));
    }

    // 整体落在正卦限：正确的 min 是 (8,9,7)。从**零盒**起步取极值的实现会
    // 把它算成 (0,0,0) —— 上面那些跨原点的盒子对这一点不敏感（它们的 min
    // 本来就 <= 0）。这是 `Box3T::Empty()` 那个起点的死亡证明。
    const auto positiveFrame = Coordinate3::FromZAxis(Point3{10.0, 10.0, 10.0}, zAxis);
    REQUIRE(positiveFrame.has_value());
    const Box3 positiveAabb =
        OrientedBox3{*positiveFrame, Vector3{1.0, 2.0, 3.0}}.ToAxisAligned();
    CHECK(positiveAabb.Min == Point3{8.0, 9.0, 7.0});
    CHECK(positiveAabb.Max == Point3{12.0, 11.0, 13.0});

    // 反方向的卦限（整体为负）：max 才是「从零盒起步」会算错的那一端。
    const auto negativeFrame = Coordinate3::FromZAxis(Point3{-10.0, -10.0, -10.0}, zAxis);
    REQUIRE(negativeFrame.has_value());
    const Box3 negativeAabb =
        OrientedBox3{*negativeFrame, Vector3{1.0, 2.0, 3.0}}.ToAxisAligned();
    CHECK(negativeAabb.Min == Point3{-12.0, -11.0, -13.0});
    CHECK(negativeAabb.Max == Point3{-8.0, -9.0, -7.0});
}

TEST_CASE("containment pins every axis and both ends", "[linear][orientedbox3]") {
    // 轴对齐、非立方、带原点偏移：中心 (5,-3,2)，半轴 (1,2,3)。
    // 三个轴的边界值互不相同，任何一格的交叉引用都会现形。
    const Coordinate3 frame = FrameAt(Point3{5.0, -3.0, 2.0});
    const OrientedBox3 box{frame, Vector3{1.0, 2.0, 3.0}};

    CHECK(box.Center() == Point3{5.0, -3.0, 2.0});

    // 闭区间：min 角与 max 角都算在内，内部点当然算。
    CHECK(box.Contains(Point3{5.0, -3.0, 2.0}));
    CHECK(box.Contains(Point3{6.0, -1.0, 5.0}));    // Corner(7)
    CHECK(box.Contains(Point3{4.0, -5.0, -1.0}));   // Corner(0)

    // 六个方向各有一格「只在该轴的一端越界」的证据：`|local.i| <= h.i` 的
    // 三个比较被拆成六个方向，少写任何一个轴的比较、或把某一轴的上界写成
    // 另一轴的，都会在对应格上现形。
    CHECK_FALSE(box.Contains(Point3{6.5, -3.0, 2.0}));   // 越 +x
    CHECK_FALSE(box.Contains(Point3{5.0, -0.5, 2.0}));   // 越 +y
    CHECK_FALSE(box.Contains(Point3{5.0, -3.0, 5.5}));   // 越 +z
    CHECK_FALSE(box.Contains(Point3{3.5, -3.0, 2.0}));   // 越 -x
    CHECK_FALSE(box.Contains(Point3{5.0, -5.5, 2.0}));   // 越 -y
    CHECK_FALSE(box.Contains(Point3{5.0, -3.0, -1.5}));  // 越 -z

    // **闭区间本身需要独立的见证。** 容差为正时 `<=` 与 `<` 不可分：右端是
    // `h + tol > h`，边界点两边都为真（实测：下面这组是它们的**唯一**死亡
    // 证明 —— 在此之前，把三处 `<=` 全改成 `<` 的变异体在全部 422 条断言下
    // 存活）。把容差显式设为 0 之后右端恰好等于 h，「边界算在内」只剩 `<=`
    // 能满足 —— 三个轴各给一格，单轴变异体各自现形（Corner(7) 那格三轴同时
    // 取等，杀不了单轴变异体）。
    const Tolerance exact{0.0, 0.0};
    CHECK(box.Contains(Point3{6.0, -3.0, 2.0}, exact));    // 只 x 取上界
    CHECK(box.Contains(Point3{5.0, -1.0, 2.0}, exact));    // 只 y 取上界
    CHECK(box.Contains(Point3{5.0, -3.0, 5.0}, exact));    // 只 z 取上界
    CHECK(box.Contains(Point3{4.0, -5.0, -1.0}, exact));   // Corner(0)：三轴都取下界
    CHECK_FALSE(box.Contains(Point3{6.0 + 1e-6, -3.0, 2.0}, exact));

    // 八个角全部落在盒里，且紧包围盒就是 [center - h, center + h]：
    // 同一格把**原点参与运算**钉住（把原点漏掉的实现会得到一个以 (0,0,0)
    // 为中心或不含原点的盒子）。
    for (int i = 0; i < 8; ++i) {
        CHECK(box.Contains(box.Corner(i)));
    }
    const Box3 aabb = box.ToAxisAligned();
    CHECK(aabb.Min == Point3{4.0, -5.0, -1.0});
    CHECK(aabb.Max == Point3{6.0, -1.0, 5.0});
    CHECK(box.Corner(0) == Point3{4.0, -5.0, -1.0});
    CHECK(box.Corner(7) == Point3{6.0, -1.0, 5.0});
}

TEST_CASE("containment threads its tolerance through every axis",
          "[linear][orientedbox3]") {
    const Coordinate3 frame = FrameAt(Point3{0.0, 0.0, 0.0});
    const OrientedBox3 box{frame, Vector3{1.0, 2.0, 3.0}};

    // 三个轴各一格：点在边界外 1e-3。默认容差（rel 1e-9）拒绝；
    // 显式传入的 1e-2 接受。容差若只穿到某一个轴上，另外两格会失败。
    const Point3 outsideX{1.001, 0.0, 0.0};
    const Point3 outsideY{0.0, 2.001, 0.0};
    const Point3 outsideZ{0.0, 0.0, 3.001};
    CHECK_FALSE(box.Contains(outsideX));
    CHECK_FALSE(box.Contains(outsideY));
    CHECK_FALSE(box.Contains(outsideZ));

    const Tolerance loose{1e-2, 0.0};
    CHECK(box.Contains(outsideX, loose));
    CHECK(box.Contains(outsideY, loose));
    CHECK(box.Contains(outsideZ, loose));

    // 容差不是「全接受」：显式 1e-2 之下 0.5 的越界仍被拒（三个轴各一格）。
    CHECK_FALSE(box.Contains(Point3{1.5, 0.0, 0.0}, loose));
    CHECK_FALSE(box.Contains(Point3{0.0, 2.5, 0.0}, loose));
    CHECK_FALSE(box.Contains(Point3{0.0, 0.0, 3.5}, loose));

    // 盒内的点在显式容差下同样被接受（反向自证：上面三格不是「恒真」的）。
    CHECK(box.Contains(Point3{0.999, 0.0, 0.0}, loose));
    CHECK(box.Contains(Point3{0.999, 1.999, 2.999}, loose));
}

TEST_CASE("containment's tolerance absorbs the local round trip",
          "[linear][orientedbox3]") {
    // `contains` 带容差而 `Box3T::contains` 不带，理由不是「有向盒更模糊」，
    // 而是**点必须经 `ToLocal` 往返一次**：先减原点、再与三根轴做点积
    // （六次乘加），一个恰好落在角点上的点会带上误差。
    // 这一格让误差**足以把角点判到盒外**（角点按精确比较在自己的盒外），
    // 而断言一旦绑在舍入误差的符号上，**余量就是它全部的安全带**：
    //
    //   原点是刻意的：误差随坐标量级线性增长。原点取量级 1000 时，八个角
    //   在各自「决定它在外」的那根轴上都有约 **36 ulp** 的余量（实测）；
    //   原点取 (0,0,0) 时只有 1 ulp —— 把 `ToLocal` 的结果整体下移 1 ulp
    //   （GCC/Clang 默认把 `x*ox + y*oy + z*oz` 收缩成 FMA，正是这个量级的
    //   差异）就会让一个**完全正确**的实现在那一版上失败（实测：修前版本的
    //   `orientedBox3Test.cpp:432`）。本版对 35 ulp 的整体系下移仍然通过
    //   （K=36 才失败 —— 余量就是这 36 ulp）。
    const auto z = Vector3{1.0, 1.0, 1.0}.Normalized();
    REQUIRE(z.has_value());
    const auto frame = Coordinate3::FromZAxis(Point3{1000.0, 1000.0, 1000.0}, *z);
    REQUIRE(frame.has_value());

    const OrientedBox3 box{*frame, Vector3{1.0, 2.0, 3.0}};

    // 每个角点都有**至少一根轴**的余量为正 —— 这正是 `Contains(…, 零容差)`
    // 为假所依赖的**全部**性质。**不**逐轴断言「三根轴都为正」：那是坐标靠近
    // 原点时的巧合（大坐标下三根轴正负混合，实测），把断言绑到它上面等于把
    // 用例绑到一个与实现无关的舍入细节上。判据与 2D 同名用例相同（那边在
    // (100,50) 这一格上两根轴的余量都为正，所以直接逐轴断言）。
    for (int i = 0; i < 8; ++i) {
        const Point3 local = frame->ToLocal(box.Corner(i));
        const double excessX = std::abs(local.X) - box.HalfExtent.X;
        const double excessY = std::abs(local.Y) - box.HalfExtent.Y;
        const double excessZ = std::abs(local.Z) - box.HalfExtent.Z;
        const double largest = excessX > excessY ? excessX : excessY;
        const double excess = largest > excessZ ? largest : excessZ;
        REQUIRE(excess > 0.0);
    }

    // 零容差把八个角判在盒外，默认容差把它们吸收掉 —— 两条合起来说明容差
    // 参数不是装饰，且默认值恰好够用（往返误差 ~1e-14 远小于默认的相对项
    // 1e-9，两者相差 5 个数量级）。
    for (int i = 0; i < 8; ++i) {
        CHECK_FALSE(box.Contains(box.Corner(i), Tolerance{0.0, 0.0}));
        CHECK(box.Contains(box.Corner(i)));
    }
}

TEST_CASE("oriented box equality has evidence in every member",
          "[linear][orientedbox3]") {
    // `==` 要比较**七个**标量：frame 的原点 + 三根轴，以及 HalfExtent 的三个
    // 分量。每一个都要有独立的不同证据 —— 只写一条「整体相等」时，漏掉任何
    // 一个成员都看不出来。
    const auto baseFrame = Coordinate3::FromAxes(Point3{0.0, 0.0, 0.0}, ex, ey, zAxis);
    REQUIRE(baseFrame.has_value());
    const OrientedBox3 base{*baseFrame, Vector3{1.0, 2.0, 3.0}};

    CHECK(base == OrientedBox3{*baseFrame, Vector3{1.0, 2.0, 3.0}});

    // 原点：同一组轴、不同原点。
    const auto movedFrame = Coordinate3::FromAxes(Point3{1.0, 0.0, 0.0}, ex, ey, zAxis);
    REQUIRE(movedFrame.has_value());
    CHECK(base != OrientedBox3{*movedFrame, Vector3{1.0, 2.0, 3.0}});

    // 三根轴各一格。右手正交标架里任意两条轴唯一决定第三条，所以「精确只差
    // 一根轴」的另一组标架不**存在** —— 用 1e-10 的偏移造出「只在那一根轴上
    // 不同」的标架（长度偏差 1e-20、点积偏差 1e-10 都落在显式放宽容差的松弛
    // 区内），于是每一格只由那一根轴自己的比较决定。
    const Tolerance loose{1e-6, 1e-6};
    const auto tiltedX = Coordinate3::FromAxes(
        Point3{0.0, 0.0, 0.0},
        UnitVector3::FromNormalizedUnchecked(Vector3{1.0, 1e-10, 0.0}), ey, zAxis, loose);
    REQUIRE(tiltedX.has_value());
    CHECK(base != OrientedBox3{*tiltedX, Vector3{1.0, 2.0, 3.0}});

    const auto tiltedY = Coordinate3::FromAxes(
        Point3{0.0, 0.0, 0.0}, ex,
        UnitVector3::FromNormalizedUnchecked(Vector3{0.0, 1.0, 1e-10}), zAxis, loose);
    REQUIRE(tiltedY.has_value());
    CHECK(base != OrientedBox3{*tiltedY, Vector3{1.0, 2.0, 3.0}});

    const auto tiltedZ = Coordinate3::FromAxes(
        Point3{0.0, 0.0, 0.0}, ex, ey,
        UnitVector3::FromNormalizedUnchecked(Vector3{1e-10, 0.0, 1.0}), loose);
    REQUIRE(tiltedZ.has_value());
    CHECK(base != OrientedBox3{*tiltedZ, Vector3{1.0, 2.0, 3.0}});

    // HalfExtent 的三个分量各一格。
    CHECK(base != OrientedBox3{*baseFrame, Vector3{9.0, 2.0, 3.0}});
    CHECK(base != OrientedBox3{*baseFrame, Vector3{1.0, 9.0, 3.0}});
    CHECK(base != OrientedBox3{*baseFrame, Vector3{1.0, 2.0, 9.0}});

    // 相等那一条也要在**每个成员都相同**时才成立（反向自证）。
    CHECK_FALSE(base != OrientedBox3{*baseFrame, Vector3{1.0, 2.0, 3.0}});
}

TEST_CASE("the implicitly generated special members carry the frame and the half extent",
          "[linear][orientedbox3]") {
    const Coordinate3 frame = FrameAt(Point3{1.0, -2.0, 3.0});
    const OrientedBox3 source{frame, Vector3{1.0, 2.0, 3.0}};

    // 拷贝赋值：只赋 HalfExtent、或只赋 frame 的手臂实现会在这里现形。
    OrientedBox3 target{FrameAt(Point3{9.0, 9.0, 9.0}), Vector3{9.0, 9.0, 9.0}};
    target = source;
    CHECK(target.Coordinate == source.Coordinate);
    CHECK(target.HalfExtent == Vector3{1.0, 2.0, 3.0});
    CHECK(target.Center() == Point3{1.0, -2.0, 3.0});
    CHECK(target.Corner(7) == source.Corner(7));

    // 拷贝构造。
    const OrientedBox3 copied = target;
    CHECK(copied.Coordinate == source.Coordinate);
    CHECK(copied.HalfExtent == Vector3{1.0, 2.0, 3.0});
    CHECK(copied.Corner(7) == source.Corner(7));

    // 移动构造。
    OrientedBox3 movedSource{frame, Vector3{4.0, 5.0, 6.0}};
    OrientedBox3 moved = std::move(movedSource);
    CHECK(moved.Coordinate == frame);
    CHECK(moved.HalfExtent == Vector3{4.0, 5.0, 6.0});

    // 移动赋值。
    OrientedBox3 movedInto{FrameAt(Point3{0.0, 0.0, 0.0}), Vector3{0.0, 0.0, 0.0}};
    movedInto = std::move(moved);
    CHECK(movedInto.Coordinate == frame);
    CHECK(movedInto.HalfExtent == Vector3{4.0, 5.0, 6.0});
    CHECK(movedInto.Corner(7) == Point3{5.0, 3.0, 9.0});

    // 析构与拷贝/移动的「无行为」是这一类型的规格：这些成员都是隐式生成的。
    // 这**不是**用特征代理行为（本类型没有可观测的析构副作用），而是直接
    // 陈述「这些成员是平凡/隐式的」这一条规格本身（写成 `= default` 同样为真，
    // 证明逐位搬运的是上面那四组断言）。
    STATIC_REQUIRE(std::is_trivially_copyable_v<OrientedBox3T<double>>);
    STATIC_REQUIRE(std::is_trivially_destructible_v<OrientedBox3T<double>>);
}

TEST_CASE("every declared callable is noexcept", "[linear][orientedbox3]") {
    constexpr Coordinate3T<double> frame = Coordinate3T<double>::Identity();
    const OrientedBox3T<double> box{frame, Vector3T<double>{1.0, 2.0, 3.0}};

    // 各条之间无依赖，去掉**任意一处** noexcept 都会单独失败 —— 最后一条
    // `!=` 除外，理由见它自己的注释。
    STATIC_REQUIRE(noexcept(box.Center()));
    STATIC_REQUIRE(noexcept(box.Contains(Point3T<double>{})));
    STATIC_REQUIRE(noexcept(box.Contains(Point3T<double>{}, Tolerance{})));
    STATIC_REQUIRE(noexcept(box.Corner(0)));
    STATIC_REQUIRE(noexcept(box.ToAxisAligned()));
    STATIC_REQUIRE(noexcept(box.Expanded(0.0)));
    STATIC_REQUIRE(noexcept(box == box));
    // 这一条相对上一条是**零独立证据**：C++20 的 `!=` 是 `!(a == b)` 的重写，
    // noexcept 规格继承自被重写的 `operator==`。保留它只为把「生成的 `!=`
    // 也被真的用过」写出来，不要把它计入覆盖率。
    STATIC_REQUIRE(noexcept(box != box));
}

TEST_CASE("every callable is usable in a constant expression",
          "[linear][orientedbox3]") {
    // `Identity()` 是 constexpr 工厂，于是整条链都能在常量表达式里求值。
    // 逐个删掉对应成员的 `constexpr`，这里就编译不过。
    constexpr Coordinate3T<double> frame = Coordinate3T<double>::Identity();
    constexpr OrientedBox3T<double> box{frame, Vector3T<double>{1.0, 2.0, 3.0}};
    constexpr Box3T<double> aabb = box.ToAxisAligned();

    STATIC_REQUIRE(box.Center() == Point3T<double>{0.0, 0.0, 0.0});
    STATIC_REQUIRE(box.Corner(0) == Point3T<double>{-1.0, -2.0, -3.0});
    STATIC_REQUIRE(box.Corner(3) == Point3T<double>{1.0, 2.0, -3.0});
    STATIC_REQUIRE(box.Corner(5) == Point3T<double>{1.0, -2.0, 3.0});
    STATIC_REQUIRE(box.Corner(6) == Point3T<double>{-1.0, 2.0, 3.0});
    STATIC_REQUIRE(aabb.Min == Point3T<double>{-1.0, -2.0, -3.0});
    STATIC_REQUIRE(aabb.Max == Point3T<double>{1.0, 2.0, 3.0});
    STATIC_REQUIRE(box.Contains(Point3T<double>{1.0, 2.0, 3.0}));
    STATIC_REQUIRE_FALSE(box.Contains(Point3T<double>{1.0, 2.0, 3.5}));
    STATIC_REQUIRE(box.Expanded(1.0).HalfExtent == Vector3T<double>{2.0, 3.0, 4.0});
    STATIC_REQUIRE(box.Expanded(-10.0).HalfExtent == Vector3T<double>{0.0, 0.0, 0.0});
    STATIC_REQUIRE_FALSE(box.Expanded(-10.0).Contains(Point3T<double>{0.5, 0.0, 0.0}));
    STATIC_REQUIRE(box == OrientedBox3T<double>{frame, Vector3T<double>{1.0, 2.0, 3.0}});
    STATIC_REQUIRE(box != box.Expanded(1.0));
}

TEST_CASE("the float instantiation is usable", "[linear][orientedbox3]") {
    const auto frame = Coordinate3T<float>::Identity();
    const OrientedBox3f box{frame, Vector3T<float>{1.0f, 2.0f, 3.0f}};

    CHECK(box.Center() == Point3T<float>{0.0f, 0.0f, 0.0f});
    CHECK(box.Corner(3) == Point3T<float>{1.0f, 2.0f, -3.0f});
    CHECK(box.ToAxisAligned().Min == Point3T<float>{-1.0f, -2.0f, -3.0f});
    CHECK(box.ToAxisAligned().Max == Point3T<float>{1.0f, 2.0f, 3.0f});
    CHECK(box.Expanded(1.0f).HalfExtent == Vector3T<float>{2.0f, 3.0f, 4.0f});

    // 默认容差按 double 定标（rel 1e-9），float 调用者必须显式给容差 ——
    // 与 Coordinate / Vector 那边同一条口径。1e-3 的越界在两条路径上
    // 一拒一收，顺带证明 float 上的容差参数确实被穿到底。
    const Point3T<float> outside{1.001f, 0.0f, 0.0f};
    CHECK_FALSE(box.Contains(outside));
    CHECK(box.Contains(outside, Tolerance{1e-2, 0.0}));
}
