#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <limits>
#include <type_traits>
#include <utility>

#include <DragonGeo/Core/Tolerance.hpp>
#include <DragonGeo/Linear/Box2.hpp>
#include <DragonGeo/Linear/Coordinate2.hpp>
#include <DragonGeo/Linear/OrientedBox2.hpp>
#include <DragonGeo/Linear/Point2.hpp>
#include <DragonGeo/Linear/UnitVector2.hpp>
#include <DragonGeo/Linear/Vector2.hpp>

using Catch::Approx;

using DragonGeo::Core::Tolerance;
using DragonGeo::Core::Tolerancef;
using DragonGeo::Linear::Box2;
using DragonGeo::Linear::Box2T;
using DragonGeo::Linear::Coordinate2;
using DragonGeo::Linear::Coordinate2T;
using DragonGeo::Linear::OrientedBox2;
using DragonGeo::Linear::OrientedBox2f;
using DragonGeo::Linear::OrientedBox2T;
using DragonGeo::Linear::Point2;
using DragonGeo::Linear::Point2T;
using DragonGeo::Linear::UnitVector2;
using DragonGeo::Linear::UnitVector2T;
using DragonGeo::Linear::Vector2;
using DragonGeo::Linear::Vector2T;

namespace {

/// 世界 x 轴。二维的补全以 x 为主轴（`FromXAxis`），与三维的 `FromZAxis` 对应 —— 两个文件的这个常量因此也不同名（与 coordinate2Test.cpp 一致）。
const UnitVector2 xAxis = UnitVector2::FromNormalizedUnchecked(Vector2{1.0, 0.0});
const UnitVector2 ey = UnitVector2::FromNormalizedUnchecked(Vector2{0.0, 1.0});

/// 轴取标准基、原点在 `origin` 的标架（三维文件的 `FrameAt` 的二维版）。
///
/// `Coordinate2T::Identity()` 的原点固定在 (0,0)，而本文件多处要的是 「轴不动、只挪原点」—— 那是把「原点参与运算」从「轴参与运算」里分离出来的唯一办法。标准基必然通过校验（长度 1、正交、逆时针），失败即测试装置自身出错，所以这里照常 REQUIRE。
Coordinate2 FrameAt(Point2 origin) {
    const auto frame = Coordinate2::FromAxes(origin, xAxis, ey);
    REQUIRE(frame.has_value());
    return *frame;
}

} // namespace

TEST_CASE("the float alias really is the float instantiation", "[linear][orientedbox2]") {
    STATIC_REQUIRE(std::is_same_v<OrientedBox2f, OrientedBox2T<float>>);
    STATIC_REQUIRE(std::is_same_v<OrientedBox2, OrientedBox2T<double>>);
    STATIC_REQUIRE(std::is_same_v<OrientedBox2T<float>::ScalarType, float>);
    STATIC_REQUIRE(std::is_same_v<OrientedBox2T<double>::ScalarType, double>);

    // 聚合性（与三维逐条对齐）：Interfaces 写的就是 `struct { frame; HalfExtent; }`，聚合初始化是本类型唯一的构造路径。
    STATIC_REQUIRE(std::is_aggregate_v<OrientedBox2T<double>>);
    STATIC_REQUIRE(std::is_aggregate_v<OrientedBox2T<float>>);

    // 没有默认构造：`frame` 是强不变量类型 `Coordinate2T`，它没有默认构造。
    STATIC_REQUIRE(!std::is_default_constructible_v<OrientedBox2T<double>>);
    STATIC_REQUIRE(!std::is_default_constructible_v<OrientedBox2T<float>>);

    // 两个数据成员的名字与类型。
    STATIC_REQUIRE(std::is_same_v<decltype(OrientedBox2T<double>::Coordinate), Coordinate2T<double>>);
    STATIC_REQUIRE(std::is_same_v<decltype(OrientedBox2T<double>::HalfExtent), Vector2T<double>>);
    STATIC_REQUIRE(std::is_same_v<decltype(OrientedBox2T<float>::Coordinate), Coordinate2T<float>>);
    STATIC_REQUIRE(std::is_same_v<decltype(OrientedBox2T<float>::HalfExtent), Vector2T<float>>);
}

TEST_CASE("rotating a box grows its axis-aligned bounding box", "[linear][orientedbox2]") {
    const auto frame = Coordinate2::FromXAxis(Point2{0.0, 0.0}, UnitVector2::FromNormalizedUnchecked(Vector2{1.0, 0.0}));
    REQUIRE(frame.has_value());

    const OrientedBox2 box{*frame, Vector2{1.0, 1.0}};   // 边长 2 的正方形
    const Box2 aabb = box.ToAxisAligned();

    // 未旋转时紧包围盒就是自身
    CHECK(aabb.Min.X == Approx(-1.0));
    CHECK(aabb.Max.X == Approx(1.0));

    // 绕原点转 45° 后，x/y 方向的紧包围盒必须扩展到 √2
    const auto x45 = Vector2{1.0, 1.0}.Normalized();
    const auto y45 = Vector2{-1.0, 1.0}.Normalized();
    REQUIRE(x45.has_value());
    REQUIRE(y45.has_value());

    const auto rotatedFrame = Coordinate2::FromAxes(Point2{0.0, 0.0}, *x45, *y45);
    REQUIRE(rotatedFrame.has_value());

    const OrientedBox2 rotated{*rotatedFrame, Vector2{1.0, 1.0}};
    const Box2 rotatedAabb = rotated.ToAxisAligned();

    CHECK(rotatedAabb.Max.X == Approx(std::sqrt(2.0)).margin(1e-12));
    CHECK(rotatedAabb.Min.X == Approx(-std::sqrt(2.0)).margin(1e-12));
    CHECK(rotatedAabb.Max.X > aabb.Max.X);   // 真的变大了
    CHECK(rotatedAabb.Min.X < aabb.Min.X);   // 两端都要真的变大

    // 45° 旋转把 x/y 对称化：y 方向也必须扩展到 √2（三维的 z 方向则不变，二维没有第三个方向）。
    CHECK(rotatedAabb.Max.Y == Approx(std::sqrt(2.0)).margin(1e-12));
    CHECK(rotatedAabb.Min.Y == Approx(-std::sqrt(2.0)).margin(1e-12));
    // 尺寸（派生量，独立于上面那两条端点断言）：2√2，而不是包围球直径 2√2 之外的某个更松的值。
    CHECK(rotatedAabb.Extent().X == Approx(2.0 * std::sqrt(2.0)).margin(1e-12));

    // 非正方形（半轴 1 与 2）：45° 之后两轴的半宽都是 (1+2)/√2 = 3/√2。只用一个轴的半轴、或把另一个轴漏掉，得到的 1/√2 与 2/√2 都不等于它。
    const OrientedBox2 stretched{*rotatedFrame, Vector2{1.0, 2.0}};
    const Box2 stretchedAabb = stretched.ToAxisAligned();
    CHECK(stretchedAabb.Max.X == Approx(3.0 / std::sqrt(2.0)).margin(1e-12));
    CHECK(stretchedAabb.Min.X == Approx(-3.0 / std::sqrt(2.0)).margin(1e-12));
    CHECK(stretchedAabb.Max.Y == Approx(3.0 / std::sqrt(2.0)).margin(1e-12));
    CHECK(stretchedAabb.Min.Y == Approx(-3.0 / std::sqrt(2.0)).margin(1e-12));

    // **定义性质**：四个角都在紧包围盒里。这一组是「过紧」写法（只把中心变换过去、半轴沿用局部值）的直接死亡证明，与上面的数值断言互相独立。
    for (int i = 0; i < 4; ++i) {
        CHECK(rotatedAabb.Contains(rotated.Corner(i)));
        CHECK(aabb.Contains(box.Corner(i)));
        CHECK(stretchedAabb.Contains(stretched.Corner(i)));
    }
}

TEST_CASE("a non-cubic oriented box pins every axis separately", "[linear][orientedbox2]") {
    // 上面的用例是正方形 —— 半轴两个分量相等，于是任何把 x/y 弄混、或把 corner 的索引位弄反的实现都看不出来。半轴取 (1,2) 之后，每个轴各自可辨。
    const auto frame = Coordinate2::FromXAxis(Point2{0.0, 0.0}, UnitVector2::FromNormalizedUnchecked(Vector2{1.0, 0.0}));
    REQUIRE(frame.has_value());

    // 先把标架本身钉住：二维的补全 `y = (-x.Y, x.X)` 对 x = (1,0) **就是** 单位标架（与三维不同 —— 那边的 `FromZAxis` 把世界 z 补成绕 z 的 90° 旋转）。于是下面的期望值直接就是局部值。
    CHECK(frame->XAxis().X() == 1.0);
    CHECK(frame->XAxis().Y() == 0.0);
    CHECK(frame->YAxis().X() == 0.0);
    CHECK(frame->YAxis().Y() == 1.0);

    const OrientedBox2 box{*frame, Vector2{1.0, 2.0}};
    const Box2 aabb = box.ToAxisAligned();

    CHECK(aabb.Min == Point2{-1.0, -2.0});
    CHECK(aabb.Max == Point2{1.0, 2.0});

    // corner 的 bit0/bit1 依次选 x/y，置位取 max；先在**局部**取角，再经标架送到世界。四个索引全部钉死（0/3 是「全 min」「全 max」，1/2 各置一位）。
    CHECK(box.Corner(0) == Point2{-1.0, -2.0});
    CHECK(box.Corner(1) == Point2{1.0, -2.0});
    CHECK(box.Corner(2) == Point2{-1.0, 2.0});
    CHECK(box.Corner(3) == Point2{1.0, 2.0});

    CHECK(box.Center() == Point2{0.0, 0.0});

    // 角点全部落在紧包围盒里（定义性质）；半轴 1 与 2 不同，任何一个轴被换到另一个轴上都会让上面若干条失败。
    for (int i = 0; i < 4; ++i) {
        CHECK(aabb.Contains(box.Corner(i)));
    }
}

TEST_CASE("oriented box containment follows the frame", "[linear][orientedbox2]") {
    const auto x45 = Vector2{1.0, 1.0}.Normalized();
    const auto y45 = Vector2{-1.0, 1.0}.Normalized();
    REQUIRE(x45.has_value());
    REQUIRE(y45.has_value());

    const auto frame = Coordinate2::FromAxes(Point2{1.0, 1.0}, *x45, *y45);
    REQUIRE(frame.has_value());

    const OrientedBox2 box{*frame, Vector2{1.0, 1.0}};

    CHECK(box.Contains(Point2{1.0, 1.0}));                                    // 中心
    CHECK(box.Contains(Point2{1.0 + x45->X(), 1.0 + x45->Y()}));              // 局部 (1,0) 的面上
    CHECK(box.Contains(Point2{1.0 - y45->X(), 1.0 - y45->Y()}));              // 局部 (0,-1) 的面上
    CHECK_FALSE(box.Contains(Point2{1.0 + 1.5 * x45->X(), 1.0 + 1.5 * x45->Y()}));

    // 局部 (2,0) 这个明显在外面的点被拒绝，再确认 expanded 之后同一个点被接受。
    const Point2 outside{1.0 + 2.0 * x45->X(), 1.0 + 2.0 * x45->Y()};
    CHECK_FALSE(box.Contains(outside));
    CHECK(box.Expanded(1.5).Contains(outside));

    // 「角点必须在盒里」—— 对**旋转过的**标架，这一组才是有内容的：一个忘掉 `ToLocal`（拿世界坐标直接与半轴比）的实现会把 x45 方向的角点 （世界 (1+√2, 1+√2)）判在盒外。顺带证明默认容差覆盖了往返一次的舍入。
    for (int i = 0; i < 4; ++i) {
        CHECK(box.Contains(box.Corner(i)));
    }
}

TEST_CASE("expanded never produces a negative half extent", "[linear][orientedbox2]") {
    // 标架**刻意取一个非单位标架**（x = (0,1) ⇒ y = (-1,0)，绕原点 90°）。用单位标架的话（`FromXAxis(o, (1,0))`），「expanded 丢掉标架、返回单位标架」的实现在下面每一条都通过 —— 实测：把本用例的标架换成
    // `(1,0)` 之后，`expanded` 恒返回 `Identity()` 的变异体存活。
    const auto frame = Coordinate2::FromXAxis(Point2{0.0, 0.0}, UnitVector2::FromNormalizedUnchecked(Vector2{0.0, 1.0}));
    REQUIRE(frame.has_value());

    const OrientedBox2 box{*frame, Vector2{1.0, 2.0}};

    CHECK(box.Expanded(1.0).HalfExtent == Vector2{2.0, 3.0});
    CHECK(box.Expanded(-0.5).HalfExtent == Vector2{0.5, 1.5});

    // 收缩过头：逐分量夹到 0，而不是留下负的半轴 —— 负半轴会让 ToAxisAligned() 给出一个倒置的、悄悄错的盒子。
    CHECK(box.Expanded(-10.0).HalfExtent == Vector2{0.0, 0.0});

    // 两个半轴都为 0 时，紧包围盒退化成一个点（即中心），不是空盒。
    const Box2 degenerate = box.Expanded(-10.0).ToAxisAligned();
    CHECK_FALSE(degenerate.IsEmpty());
    CHECK(degenerate.Min == Point2{0.0, 0.0});
    CHECK(degenerate.Max == Point2{0.0, 0.0});

    // 标架原样保留：`expanded` 只动半轴（三个成员都要真的搬过去）。
    CHECK(box.Expanded(1.0).Coordinate == box.Coordinate);
    CHECK(box.Expanded(-10.0).Coordinate == box.Coordinate);
    CHECK(box.Expanded(0.0) == box);            // 零增量是恒等

    // 逐分量的夹取证据：一个让 x 与 y 各自被夹、而另一个不被夹。
    CHECK(box.Expanded(-1.5).HalfExtent == Vector2{0.0, 0.5});          // x 夹、y 不夹
    const OrientedBox2 wide{*frame, Vector2{3.0, 1.0}};
    CHECK(wide.Expanded(-1.5).HalfExtent == Vector2{1.5, 0.0});         // y 夹、x 不夹

    // 半轴本身为负（聚合初始化就能造出的非法状态）时，夹取针对的是**和** （`HalfExtent.i + amount`），不是 amount 的符号。
    const OrientedBox2 inverted{*frame, Vector2{-1.0, 2.0}};
    CHECK(inverted.Expanded(0.5).HalfExtent == Vector2{0.0, 2.5});

    // `amount` 为 NaN：结果逐分量为 NaN，**不**被静默夹成 0 —— 本类型没有 「空」可以落回，把非有限输入伪装成一个完全正常的退化盒比留下 NaN 更危险。NaN 的下游是确定的（下面两条）：contains 全假、紧包围盒是
    // 规范空盒（角点全是 NaN，逐分量比较一律为假，min/max 各自留在 ±inf）。
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const Vector2 nanHalfExtent = box.Expanded(nan).HalfExtent;
    CHECK(std::isnan(nanHalfExtent.X));
    CHECK(std::isnan(nanHalfExtent.Y));
    CHECK(box.Expanded(nan).ToAxisAligned() == Box2::Empty());
    CHECK_FALSE(box.Expanded(nan).Contains(Point2{0.0, 0.0}));

    // **非有限半轴的第二种形态：±inf**（与三维逐条相同）。`Expanded(+inf)` 是公开路径，给出半轴全 +inf；这样的盒 `contains` 什么都收，而 `ToAxisAligned()` 是**规范空盒**（角点里 `0 * inf` 是 NaN）——
    // 两个说法互相矛盾，这正是「±inf 半轴不是合法状态」的直接后果。兄弟类型 `Box2T::Expanded(+inf)` 给出的是**整个空间**（`[-inf,+inf]²`）。
    const double infinity = std::numeric_limits<double>::infinity();
    const OrientedBox2 unbounded = box.Expanded(infinity);
    CHECK(unbounded.HalfExtent.X == infinity);
    CHECK(unbounded.HalfExtent.Y == infinity);
    CHECK(unbounded.Contains(Point2{123.0, -456.0}));
    CHECK(unbounded.ToAxisAligned() == Box2::Empty());
    // 反方向：`h.i + (-inf) = -inf`，夹取后全是 0 —— 与「收缩过头」同一条规则。
    CHECK(box.Expanded(-infinity).HalfExtent == Vector2{0.0, 0.0});

    // **`ToAxisAligned()` 的出口规范化**（与三维同名用例逐条对应）：上面那个标架每一行都含 0（`0 * inf` 把角点毒成 NaN），得到的是**规范**空盒 —— 那是运气。单位标架 + 半轴 `(+inf, 2)` 给的是
    // `min = (-inf,+inf)` / `max = (+inf,-inf)`：`IsEmpty()` 为真而 `!= Empty()`；`Box2T` 的不变量要求出口给出规范形式。
    const OrientedBox2 mixedInf{Coordinate2::Identity(), Vector2{infinity, 2.0}};
    CHECK(mixedInf.ToAxisAligned().IsEmpty());
    CHECK(mixedInf.ToAxisAligned() == Box2::Empty());

    // 另一个极端（规范化**不**该动的情形）：45° 标架 + 全 +inf 半轴给出的是 **整个平面**（实测 `min = (-inf,-inf)` / `max = (+inf,+inf)`）， `IsEmpty()` 为假 → 原样返回；规范化只处理「空」这一种出口。
    const auto x45 = Vector2{1.0, 1.0}.Normalized();
    REQUIRE(x45.has_value());
    const auto frame45 = Coordinate2::FromXAxis(Point2{0.0, 0.0}, *x45);
    REQUIRE(frame45.has_value());
    const Box2 wholePlane = OrientedBox2{*frame45, Vector2{infinity, infinity}}.ToAxisAligned();
    CHECK_FALSE(wholePlane.IsEmpty());
    CHECK(wholePlane.Min.X == -infinity);
    CHECK(wholePlane.Max.X == infinity);
}

TEST_CASE("corner maps the local box through the frame", "[linear][orientedbox2]") {
    // 90° 标架：xAxis = (0,1)、yAxis = (-1,0)（逆时针：x × y = 1 > 0）。分量全是 0/1，全部算术精确。局部 (a,b) 送到世界是 (-b,a)，于是半轴 (1,3) 会被送到世界的 y/x 方向上 —— 「半轴与轴对错位」在这里无处可藏。
    const auto frame = Coordinate2::FromAxes(
        Point2{2.0, -1.0}, UnitVector2::FromNormalizedUnchecked(Vector2{0.0, 1.0}), UnitVector2::FromNormalizedUnchecked(Vector2{-1.0, 0.0}));
    REQUIRE(frame.has_value());

    const OrientedBox2 box{*frame, Vector2{1.0, 3.0}};

    // 中心是**标架原点**（不是局部原点，也不是 (0,0)）。
    CHECK(box.Center() == Point2{2.0, -1.0});

    CHECK(box.Corner(0) == Point2{5.0, -2.0});
    CHECK(box.Corner(1) == Point2{5.0, 0.0});
    CHECK(box.Corner(2) == Point2{-1.0, -2.0});
    CHECK(box.Corner(3) == Point2{-1.0, 0.0});
}

TEST_CASE("ToAxisAligned is the tight box of the corners", "[linear][orientedbox2]") {
    // 同一个 90° 标架：紧包围盒的每一端都由不同角点取胜，且两轴尺寸 (6,2) = (2·3, 2·1) 是半轴的**置换** —— 轴向互换可辨。
    const auto frame = Coordinate2::FromAxes(
        Point2{2.0, -1.0}, UnitVector2::FromNormalizedUnchecked(Vector2{0.0, 1.0}), UnitVector2::FromNormalizedUnchecked(Vector2{-1.0, 0.0}));
    REQUIRE(frame.has_value());

    const OrientedBox2 box{*frame, Vector2{1.0, 3.0}};
    const Box2 aabb = box.ToAxisAligned();

    CHECK(aabb.Min == Point2{-1.0, -2.0});
    CHECK(aabb.Max == Point2{5.0, 0.0});
    CHECK(aabb.Extent() == Vector2{6.0, 2.0});

    // 紧：半轴最长者 3、包围球半径 |h| = √10 ≈ 3.16。包围球写法给出的是边长 2√10 ≈ 6.32 的方形；把半轴直接当世界半宽的写法给出 (2,6)。两者都被上面的 `Extent() == (6,2)` 排除，下面两条再从**每轴的实际跨度**复核
    // 一次（不经由 `Extent()` 的实现）。
    CHECK(aabb.Max.X - aabb.Min.X == 6.0);
    CHECK(aabb.Max.Y - aabb.Min.Y == 2.0);
    for (int i = 0; i < 4; ++i) {
        CHECK(aabb.Contains(box.Corner(i)));
    }

    // 整体落在正卦限：正确的 min 是 (9,7)。从**零盒**起步取极值的实现会把它算成 (0,0) —— 上面那些跨原点的盒子对这一点不敏感。这是 `Box2T::Empty()` 那个起点的死亡证明。
    const OrientedBox2 positive{FrameAt(Point2{10.0, 10.0}), Vector2{1.0, 3.0}};
    const Box2 positiveAabb = positive.ToAxisAligned();
    CHECK(positiveAabb.Min == Point2{9.0, 7.0});
    CHECK(positiveAabb.Max == Point2{11.0, 13.0});

    // 反方向的卦限（整体为负）：max 才是「从零盒起步」会算错的那一端。
    const OrientedBox2 negative{FrameAt(Point2{-10.0, -10.0}), Vector2{1.0, 3.0}};
    const Box2 negativeAabb = negative.ToAxisAligned();
    CHECK(negativeAabb.Min == Point2{-11.0, -13.0});
    CHECK(negativeAabb.Max == Point2{-9.0, -7.0});
}

TEST_CASE("containment pins every axis and both ends", "[linear][orientedbox2]") {
    // 轴对齐、非立方、带原点偏移：中心 (5,-3)，半轴 (1,2)。
    const Coordinate2 frame = FrameAt(Point2{5.0, -3.0});
    const OrientedBox2 box{frame, Vector2{1.0, 2.0}};

    CHECK(box.Center() == Point2{5.0, -3.0});

    // 闭区间：min 角与 max 角都算在内，内部点当然算。
    CHECK(box.Contains(Point2{5.0, -3.0}));
    CHECK(box.Contains(Point2{6.0, -1.0}));    // Corner(3)
    CHECK(box.Contains(Point2{4.0, -5.0}));    // Corner(0)

    // 四个方向各有一格「只在该轴的一端越界」的证据：`|local.i| <= h.i` 的两个比较被拆成四个方向，少写任何一个轴的比较、或把某一轴的上界写成另一轴的，都会在对应格上现形。
    CHECK_FALSE(box.Contains(Point2{6.5, -3.0}));   // 越 +x
    CHECK_FALSE(box.Contains(Point2{5.0, -0.5}));   // 越 +y
    CHECK_FALSE(box.Contains(Point2{3.5, -3.0}));   // 越 -x
    CHECK_FALSE(box.Contains(Point2{5.0, -5.5}));   // 越 -y

    // **闭区间本身需要独立的见证。** 容差为正时 `<=` 与 `<` 不可分：右端是 `h + tol > h`，边界点两边都为真（实测：下面这组是它的**唯一**死亡证明 —— 在此之前，把两处 `<=` 全改成 `<` 的变异体在全部 422 条断言下存活）。
    // 把容差显式设为 0 之后右端恰好等于 h，「边界算在内」只剩 `<=` 能满足 —— 两个轴各给一格（Corner(3) 那格两轴同时取等，杀不了单轴变异体）。
    const Tolerance exact{0.0, 0.0};
    CHECK(box.Contains(Point2{6.0, -3.0}, exact));    // 只 x 取上界
    CHECK(box.Contains(Point2{5.0, -1.0}, exact));    // 只 y 取上界
    CHECK(box.Contains(Point2{4.0, -5.0}, exact));    // Corner(0)：两轴都取下界
    CHECK_FALSE(box.Contains(Point2{6.0 + 1e-6, -3.0}, exact));

    // 四个角全部落在盒里，且紧包围盒就是 [center - h, center + h]：同一格把**原点参与运算**钉住。
    for (int i = 0; i < 4; ++i) {
        CHECK(box.Contains(box.Corner(i)));
    }
    const Box2 aabb = box.ToAxisAligned();
    CHECK(aabb.Min == Point2{4.0, -5.0});
    CHECK(aabb.Max == Point2{6.0, -1.0});
    CHECK(box.Corner(0) == Point2{4.0, -5.0});
    CHECK(box.Corner(3) == Point2{6.0, -1.0});
}

TEST_CASE("containment threads its tolerance through every axis", "[linear][orientedbox2]") {
    const Coordinate2 frame = FrameAt(Point2{0.0, 0.0});
    const OrientedBox2 box{frame, Vector2{1.0, 2.0}};

    // 两个轴各一格：点在边界外 1e-3。默认容差（rel 1e-9）拒绝；显式传入的 1e-2 接受。容差若只穿到某一个轴上，另一格会失败。
    const Point2 outsideX{1.001, 0.0};
    const Point2 outsideY{0.0, 2.001};
    CHECK_FALSE(box.Contains(outsideX));
    CHECK_FALSE(box.Contains(outsideY));

    const Tolerance loose{1e-2, 0.0};
    CHECK(box.Contains(outsideX, loose));
    CHECK(box.Contains(outsideY, loose));

    // 容差不是「全接受」：显式 1e-2 之下 0.5 的越界仍被拒（两个轴各一格）。
    CHECK_FALSE(box.Contains(Point2{1.5, 0.0}, loose));
    CHECK_FALSE(box.Contains(Point2{0.0, 2.5}, loose));

    // 盒内的点在显式容差下同样被接受（反向自证：上面两格不是「恒真」的）。
    CHECK(box.Contains(Point2{0.999, 0.0}, loose));
    CHECK(box.Contains(Point2{0.999, 1.999}, loose));
}

TEST_CASE("containment's tolerance absorbs the local round trip", "[linear][orientedbox2]") {
    // 与三维同名用例逐条对应：点经 `ToLocal` 往返一次（先减原点、再与两根轴做点积）会带上误差，这一格让误差的符号**为正**（角点按精确比较落在盒外）。原点取 (100,50)：误差随坐标量级线性增长，这一格因此比「原点在 (0,0)」
    // 的格子稳定（实测 ±8e-15 对 ±2e-15，约 36 ulp 对 1 ulp）。**「让误差为正的断言必须带余量」这条与三维是同一手法**（三维那边换成量级 1000 的原点）：断言绑在舍入误差的符号上时，余量就是它全部的安全带。
    // 这里可以直接逐轴断言，是因为 (100,50) 这一格上两根轴的余量都为正；三维在同样量级的原点上三根轴正负混合，那边因此写成「至少一根轴为正」的聚合形式。两边的判据相同。
    const auto x = Vector2{1.0, 1.0}.Normalized();
    REQUIRE(x.has_value());
    const auto frame = Coordinate2::FromXAxis(Point2{100.0, 50.0}, *x);
    REQUIRE(frame.has_value());

    const OrientedBox2 box{*frame, Vector2{1.0, 1.0}};

    const Point2 local = frame->ToLocal(box.Corner(1));
    REQUIRE(std::abs(local.X) > box.HalfExtent.X);
    REQUIRE(std::abs(local.Y) > box.HalfExtent.Y);

    // 零容差把角点判在盒外，默认容差把它吸收掉 —— 两条合起来说明容差参数不是装饰，且默认值恰好够用。
    CHECK_FALSE(box.Contains(box.Corner(1), Tolerance{0.0, 0.0}));
    CHECK(box.Contains(box.Corner(1)));

    // 角 1 与角 2 的误差为正、角 0 与角 3 为负 —— 同一个标架下两种符号并存，这正是「不能用精确比较」的直接证据。
    for (int i = 1; i <= 2; ++i) {
        CHECK_FALSE(box.Contains(box.Corner(i), Tolerance{0.0, 0.0}));
        CHECK(box.Contains(box.Corner(i)));
    }
}

TEST_CASE("oriented box equality has evidence in every member", "[linear][orientedbox2]") {
    // `==` 要比较**五个**标量：frame 的原点 + 两根轴，以及 HalfExtent 的两个分量。每一个都要有独立的不同证据 —— 只写一条「整体相等」时，漏掉任何一个成员都看不出来。
    const auto baseFrame = Coordinate2::FromAxes(Point2{0.0, 0.0}, xAxis, ey);
    REQUIRE(baseFrame.has_value());
    const OrientedBox2 base{*baseFrame, Vector2{1.0, 2.0}};

    CHECK(base == OrientedBox2{*baseFrame, Vector2{1.0, 2.0}});

    // 原点：同一组轴、不同原点。
    const auto movedFrame = Coordinate2::FromAxes(Point2{1.0, 0.0}, xAxis, ey);
    REQUIRE(movedFrame.has_value());
    CHECK(base != OrientedBox2{*movedFrame, Vector2{1.0, 2.0}});

    // 两根轴各一格。二维的「精确只差一根轴」用 1e-10 的偏移造出（点积偏差 1e-10 落在显式放宽容差的松弛区内），于是每一格只由那一根轴自己的比较决定 —— 定向检查是纯符号判断，偏移不改变符号。
    const Tolerance loose{1e-6, 1e-6};
    const auto tiltedX = Coordinate2::FromAxes(Point2{0.0, 0.0}, UnitVector2::FromNormalizedUnchecked(Vector2{1.0, 1e-10}), ey, loose);
    REQUIRE(tiltedX.has_value());
    CHECK(base != OrientedBox2{*tiltedX, Vector2{1.0, 2.0}});

    const auto tiltedY = Coordinate2::FromAxes(Point2{0.0, 0.0}, xAxis, UnitVector2::FromNormalizedUnchecked(Vector2{1e-10, 1.0}), loose);
    REQUIRE(tiltedY.has_value());
    CHECK(base != OrientedBox2{*tiltedY, Vector2{1.0, 2.0}});

    // HalfExtent 的两个分量各一格。
    CHECK(base != OrientedBox2{*baseFrame, Vector2{9.0, 2.0}});
    CHECK(base != OrientedBox2{*baseFrame, Vector2{1.0, 9.0}});

    // 相等那一条也要在**每个成员都相同**时才成立（反向自证）。
    CHECK_FALSE(base != OrientedBox2{*baseFrame, Vector2{1.0, 2.0}});
}

TEST_CASE("the implicitly generated special members carry the frame and the half extent", "[linear][orientedbox2]") {
    const Coordinate2 frame = FrameAt(Point2{1.0, -2.0});
    const OrientedBox2 source{frame, Vector2{1.0, 2.0}};

    // 拷贝赋值：只赋 HalfExtent、或只赋 frame 的手臂实现会在这里现形。
    OrientedBox2 target{FrameAt(Point2{9.0, 9.0}), Vector2{9.0, 9.0}};
    target = source;
    CHECK(target.Coordinate == source.Coordinate);
    CHECK(target.HalfExtent == Vector2{1.0, 2.0});
    CHECK(target.Center() == Point2{1.0, -2.0});
    CHECK(target.Corner(3) == source.Corner(3));

    // 拷贝构造。
    const OrientedBox2 copied = target;
    CHECK(copied.Coordinate == source.Coordinate);
    CHECK(copied.HalfExtent == Vector2{1.0, 2.0});
    CHECK(copied.Corner(3) == source.Corner(3));

    // 移动构造。
    OrientedBox2 movedSource{frame, Vector2{4.0, 5.0}};
    OrientedBox2 moved = std::move(movedSource);
    CHECK(moved.Coordinate == frame);
    CHECK(moved.HalfExtent == Vector2{4.0, 5.0});

    // 移动赋值。
    OrientedBox2 movedInto{FrameAt(Point2{0.0, 0.0}), Vector2{0.0, 0.0}};
    movedInto = std::move(moved);
    CHECK(movedInto.Coordinate == frame);
    CHECK(movedInto.HalfExtent == Vector2{4.0, 5.0});
    CHECK(movedInto.Corner(3) == Point2{5.0, 3.0});

    // 析构与拷贝/移动的「无行为」是这一类型的规格：这些成员都是隐式生成的。这**不是**用特征代理行为（本类型没有可观测的析构副作用），而是直接陈述「这些成员是平凡/隐式的」这一条规格本身（写成 `= default` 同样为真，证明逐位搬运的是上面那四组断言）。
    STATIC_REQUIRE(std::is_trivially_copyable_v<OrientedBox2T<double>>);
    STATIC_REQUIRE(std::is_trivially_destructible_v<OrientedBox2T<double>>);
}

TEST_CASE("every declared callable is noexcept", "[linear][orientedbox2]") {
    constexpr Coordinate2T<double> frame = Coordinate2T<double>::Identity();
    const OrientedBox2T<double> box{frame, Vector2T<double>{1.0, 2.0}};

    // 各条之间无依赖，去掉**任意一处** noexcept 都会单独失败 —— 最后一条 `!=` 除外，理由见它自己的注释。
    STATIC_REQUIRE(noexcept(box.Center()));
    STATIC_REQUIRE(noexcept(box.Contains(Point2T<double>{})));
    STATIC_REQUIRE(noexcept(box.Contains(Point2T<double>{}, Tolerance{})));
    STATIC_REQUIRE(noexcept(box.Corner(0)));
    STATIC_REQUIRE(noexcept(box.ToAxisAligned()));
    STATIC_REQUIRE(noexcept(box.Expanded(0.0)));
    STATIC_REQUIRE(noexcept(box == box));
    // 这一条相对上一条是**零独立证据**：C++20 的 `!=` 是 `!(a == b)` 的重写， noexcept 规格继承自被重写的 `operator==`。保留它只为把「生成的 `!=` 也被真的用过」写出来，不要把它计入覆盖率。
    STATIC_REQUIRE(noexcept(box != box));
}

TEST_CASE("every callable is usable in a constant expression", "[linear][orientedbox2]") {
    // `Identity()` 是 constexpr 工厂，于是整条链都能在常量表达式里求值。逐个删掉对应成员的 `constexpr`，这里就编译不过。
    constexpr Coordinate2T<double> frame = Coordinate2T<double>::Identity();
    constexpr OrientedBox2T<double> box{frame, Vector2T<double>{1.0, 2.0}};
    constexpr Box2T<double> aabb = box.ToAxisAligned();

    STATIC_REQUIRE(box.Center() == Point2T<double>{0.0, 0.0});
    STATIC_REQUIRE(box.Corner(0) == Point2T<double>{-1.0, -2.0});
    STATIC_REQUIRE(box.Corner(1) == Point2T<double>{1.0, -2.0});
    STATIC_REQUIRE(box.Corner(2) == Point2T<double>{-1.0, 2.0});
    STATIC_REQUIRE(aabb.Min == Point2T<double>{-1.0, -2.0});
    STATIC_REQUIRE(aabb.Max == Point2T<double>{1.0, 2.0});
    STATIC_REQUIRE(box.Contains(Point2T<double>{1.0, 2.0}));
    STATIC_REQUIRE_FALSE(box.Contains(Point2T<double>{1.0, 2.5}));
    STATIC_REQUIRE(box.Expanded(1.0).HalfExtent == Vector2T<double>{2.0, 3.0});
    STATIC_REQUIRE(box.Expanded(-10.0).HalfExtent == Vector2T<double>{0.0, 0.0});
    STATIC_REQUIRE_FALSE(box.Expanded(-10.0).Contains(Point2T<double>{0.5, 0.0}));
    STATIC_REQUIRE(box == OrientedBox2T<double>{frame, Vector2T<double>{1.0, 2.0}});
    STATIC_REQUIRE(box != box.Expanded(1.0));
}

TEST_CASE("the float instantiation is usable", "[linear][orientedbox2]") {
    const auto frame = Coordinate2T<float>::Identity();
    const OrientedBox2f box{frame, Vector2T<float>{1.0f, 2.0f}};

    CHECK(box.Center() == Point2T<float>{0.0f, 0.0f});
    CHECK(box.Corner(1) == Point2T<float>{1.0f, -2.0f});
    CHECK(box.ToAxisAligned().Min == Point2T<float>{-1.0f, -2.0f});
    CHECK(box.ToAxisAligned().Max == Point2T<float>{1.0f, 2.0f});
    CHECK(box.Expanded(1.0f).HalfExtent == Vector2T<float>{2.0f, 3.0f});

    // 默认容差按 double 定标（rel 1e-9），float 调用者必须显式给容差 —— 与 Coordinate / Vector 那边同一条口径。1e-3 的越界在两条路径上一拒一收，顺带证明 float 上的容差参数确实被穿到底。
    const Point2T<float> outside{1.001f, 0.0f};
    CHECK_FALSE(box.Contains(outside));
    CHECK(box.Contains(outside, Tolerancef{1e-2f, 0.0f}));
}
