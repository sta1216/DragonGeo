#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <limits>
#include <type_traits>
#include <utility>

#include <GeoCore/core/Tolerance.hpp>
#include <GeoCore/linear/Box3.hpp>
#include <GeoCore/linear/Coordinate3.hpp>
#include <GeoCore/linear/OrientedBox3.hpp>
#include <GeoCore/linear/Point3.hpp>
#include <GeoCore/linear/UnitVector3.hpp>
#include <GeoCore/linear/Vector3.hpp>

using Catch::Approx;

using GeoCore::core::Tolerance;
using GeoCore::linear::Box3;
using GeoCore::linear::Box3T;
using GeoCore::linear::Coordinate3;
using GeoCore::linear::Coordinate3T;
using GeoCore::linear::OrientedBox3;
using GeoCore::linear::OrientedBox3f;
using GeoCore::linear::OrientedBox3T;
using GeoCore::linear::Point3;
using GeoCore::linear::Point3T;
using GeoCore::linear::UnitVector3;
using GeoCore::linear::UnitVector3T;
using GeoCore::linear::Vector3;
using GeoCore::linear::Vector3T;

namespace {

/// 世界 z 轴。与 coordinate3_test.cpp / transform3_test.cpp 同一种写法。
const UnitVector3 z_axis = UnitVector3::from_normalized_unchecked(Vector3{0.0, 0.0, 1.0});
const UnitVector3 ex = UnitVector3::from_normalized_unchecked(Vector3{1.0, 0.0, 0.0});
const UnitVector3 ey = UnitVector3::from_normalized_unchecked(Vector3{0.0, 1.0, 0.0});

/// 轴取标准基、原点在 `origin` 的标架。
///
/// `Coordinate3T::identity()` 的原点固定在 (0,0,0)，而本文件多处要的是
/// 「轴不动、只挪原点」—— 那是把「原点参与运算」从「轴参与运算」里分离出来的
/// 唯一办法。标准基必然通过校验（长度 1、两两正交、右手），失败即测试装置
/// 自身出错，所以这里照常 REQUIRE。
Coordinate3 frame_at(Point3 origin) {
    const auto frame = Coordinate3::from_axes(origin, ex, ey, z_axis);
    REQUIRE(frame.has_value());
    return *frame;
}

} // namespace

TEST_CASE("the float alias really is the float instantiation",
          "[linear][orientedbox3]") {
    STATIC_REQUIRE(std::is_same_v<OrientedBox3f, OrientedBox3T<float>>);
    STATIC_REQUIRE(std::is_same_v<OrientedBox3, OrientedBox3T<double>>);
    STATIC_REQUIRE(std::is_same_v<OrientedBox3T<float>::scalar_type, float>);
    STATIC_REQUIRE(std::is_same_v<OrientedBox3T<double>::scalar_type, double>);

    // 聚合性是 Interfaces 明写的形态（`struct { frame; half_extent; }`），也是
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
    STATIC_REQUIRE(std::is_same_v<decltype(OrientedBox3T<double>::frame), Coordinate3T<double>>);
    STATIC_REQUIRE(std::is_same_v<decltype(OrientedBox3T<double>::half_extent), Vector3T<double>>);
    STATIC_REQUIRE(std::is_same_v<decltype(OrientedBox3T<float>::frame), Coordinate3T<float>>);
    STATIC_REQUIRE(std::is_same_v<decltype(OrientedBox3T<float>::half_extent), Vector3T<float>>);
}

TEST_CASE("rotating a box grows its axis-aligned bounding box",
          "[linear][orientedbox3]") {
    const auto frame = Coordinate3::from_z_axis(
        Point3{0.0, 0.0, 0.0},
        UnitVector3::from_normalized_unchecked(Vector3{0.0, 0.0, 1.0}));
    REQUIRE(frame.has_value());

    const OrientedBox3 box{*frame, Vector3{1.0, 1.0, 1.0}};   // 边长 2 的立方体
    const Box3 aabb = box.to_axis_aligned();

    // 未旋转时紧包围盒就是自身
    CHECK(aabb.min.x == Approx(-1.0));
    CHECK(aabb.max.x == Approx(1.0));

    // 绕 z 转 45° 后，x/y 方向的紧包围盒必须扩展到 √2
    const auto x45 = Vector3{1.0, 1.0, 0.0}.normalized();
    const auto y45 = Vector3{-1.0, 1.0, 0.0}.normalized();
    REQUIRE(x45.has_value());
    REQUIRE(y45.has_value());

    const auto rotated_frame = Coordinate3::from_axes(
        Point3{0.0, 0.0, 0.0}, *x45, *y45, z_axis);
    REQUIRE(rotated_frame.has_value());

    const OrientedBox3 rotated{*rotated_frame, Vector3{1.0, 1.0, 1.0}};
    const Box3 rotated_aabb = rotated.to_axis_aligned();

    CHECK(rotated_aabb.max.x == Approx(std::sqrt(2.0)).margin(1e-12));
    CHECK(rotated_aabb.min.x == Approx(-std::sqrt(2.0)).margin(1e-12));
    CHECK(rotated_aabb.max.x > aabb.max.x);   // 真的变大了
    CHECK(rotated_aabb.max.z == Approx(1.0)); // z 方向不变

    // 两端都要真的变大：只钉 max 那一侧时，「min 沿用局部值」的实现无处可藏
    // 的另一半 —— 45° 旋转把 x/y 对称化，y 方向也必须扩展到 √2。
    CHECK(rotated_aabb.min.x < aabb.min.x);
    CHECK(rotated_aabb.max.y == Approx(std::sqrt(2.0)).margin(1e-12));
    CHECK(rotated_aabb.min.y == Approx(-std::sqrt(2.0)).margin(1e-12));
    CHECK(rotated_aabb.min.z == Approx(-1.0));
    // 尺寸（派生量，独立于上面那两条端点断言）：2√2，而不是包围球直径 2√3。
    CHECK(rotated_aabb.extent().x == Approx(2.0 * std::sqrt(2.0)).margin(1e-12));

    // **定义性质**：八个角都在紧包围盒里。这一组是「过紧」写法（只把中心变换
    // 过去、半轴沿用局部值 = 1）的直接死亡证明 —— 它会让 (0,±√2,±1) 这类角点
    // 落到盒外；上面那两条 √2 断言则是同一个变异体的另一种证据。
    for (int i = 0; i < 8; ++i) {
        CHECK(rotated_aabb.contains(rotated.corner(i)));
        CHECK(aabb.contains(box.corner(i)));
    }
}

TEST_CASE("a non-cubic oriented box pins every axis separately",
          "[linear][orientedbox3]") {
    // 上面的用例是立方体 —— 半轴三个分量相等，于是任何把 x/y/z 弄混、
    // 或把 corner 的索引位弄反的实现都看不出来。半轴取 (1,2,3) 之后，
    // 每个轴各自可辨。
    const auto frame = Coordinate3::from_z_axis(
        Point3{0.0, 0.0, 0.0},
        UnitVector3::from_normalized_unchecked(Vector3{0.0, 0.0, 1.0}));
    REQUIRE(frame.has_value());

    // 先把标架本身钉住：`from_z_axis` 对世界 z 轴的补全**不是**单位标架，而是
    // 绕 z 的一个 90° 旋转（x = (1,0,0) × z = (0,-1,0)，y = z × x = (1,0,0)）。
    // 下面每一格的期望值都由它算出 —— 局部 (a,b,c) 送到世界是 (b,-a,c)。
    CHECK(frame->x_axis().x() == 0.0);
    CHECK(frame->x_axis().y() == -1.0);
    CHECK(frame->x_axis().z() == 0.0);
    CHECK(frame->y_axis().x() == 1.0);
    CHECK(frame->y_axis().y() == 0.0);
    CHECK(frame->y_axis().z() == 0.0);
    CHECK(frame->z_axis().z() == 1.0);

    const OrientedBox3 box{*frame, Vector3{1.0, 2.0, 3.0}};
    const Box3 aabb = box.to_axis_aligned();

    CHECK(aabb.min == Point3{-2.0, -1.0, -3.0});
    CHECK(aabb.max == Point3{2.0, 1.0, 3.0});

    // corner 的 bit0/bit1/bit2 依次选 x/y/z，置位取 max；先在**局部**取角，
    // 再经标架送到世界。0 与 7 是「全 min」「全 max」，三个位的两两互换在它们
    // 身上看不出来：1/2/4 各置一位、3/5/6 各置两位，每一位都独立钉死。
    CHECK(box.corner(0) == Point3{-2.0, 1.0, -3.0});
    CHECK(box.corner(1) == Point3{-2.0, -1.0, -3.0});
    CHECK(box.corner(2) == Point3{2.0, 1.0, -3.0});
    CHECK(box.corner(3) == Point3{2.0, -1.0, -3.0});
    CHECK(box.corner(4) == Point3{-2.0, 1.0, 3.0});
    CHECK(box.corner(5) == Point3{-2.0, -1.0, 3.0});
    CHECK(box.corner(6) == Point3{2.0, 1.0, 3.0});
    CHECK(box.corner(7) == Point3{2.0, -1.0, 3.0});

    CHECK(box.center() == Point3{0.0, 0.0, 0.0});

    // 角点全部落在紧包围盒里（定义性质）。半轴 (1,2,3) 的三个值互不相同，
    // 任何一个轴被换到别的轴上都会让上面若干条失败。
    for (int i = 0; i < 8; ++i) {
        CHECK(aabb.contains(box.corner(i)));
    }
}

TEST_CASE("oriented box containment follows the frame", "[linear][orientedbox3]") {
    const auto x45 = Vector3{1.0, 1.0, 0.0}.normalized();
    const auto y45 = Vector3{-1.0, 1.0, 0.0}.normalized();
    REQUIRE(x45.has_value());
    REQUIRE(y45.has_value());

    const auto frame = Coordinate3::from_axes(Point3{1.0, 1.0, 0.0}, *x45, *y45, z_axis);
    REQUIRE(frame.has_value());

    const OrientedBox3 box{*frame, Vector3{1.0, 1.0, 1.0}};

    CHECK(box.contains(Point3{1.0, 1.0, 0.0}));          // 中心
    CHECK(box.contains(Point3{1.0, 1.0, 1.0}));          // 局部 z 的面上
    CHECK_FALSE(box.contains(Point3{1.0, 1.0, 1.5}));

    // 角落方向：局部 (1,1,1) 在世界里是 rotated_frame 作用后的点。
    // 先确认「局部 (2,0,0)」这个明显在外面的点被拒绝，再确认边界点被接受。
    CHECK_FALSE(box.contains(Point3{1.0 + 2.0 * x45->x(), 1.0 + 2.0 * x45->y(), 0.0}));

    // expanded 之后同一个点应当被接受。
    CHECK(box.expanded(1.5).contains(
        Point3{1.0 + 2.0 * x45->x(), 1.0 + 2.0 * x45->y(), 0.0}));

    // 「角点必须在盒里」—— 对**旋转过的**标架，这一组才是有内容的：一个
    // 忘掉 `to_local`（拿世界坐标直接与半轴比）的实现会在 (1,1,1) 那一格上
    // 碰巧通过，却把 x45 方向的角点（世界 (1+√2, 1, 1)）判在盒外。
    // 顺带证明默认容差覆盖了往返一次的舍入（偏出约 1 ulp）。
    for (int i = 0; i < 8; ++i) {
        CHECK(box.contains(box.corner(i)));
    }
}

TEST_CASE("expanded never produces a negative half extent",
          "[linear][orientedbox3]") {
    const auto frame = Coordinate3::from_z_axis(
        Point3{0.0, 0.0, 0.0},
        UnitVector3::from_normalized_unchecked(Vector3{0.0, 0.0, 1.0}));
    REQUIRE(frame.has_value());

    const OrientedBox3 box{*frame, Vector3{1.0, 2.0, 3.0}};

    CHECK(box.expanded(1.0).half_extent == Vector3{2.0, 3.0, 4.0});
    CHECK(box.expanded(-0.5).half_extent == Vector3{0.5, 1.5, 2.5});

    // 收缩过头：逐分量夹到 0，而不是留下负的半轴 —— 负半轴会让
    // to_axis_aligned() 给出一个倒置的、悄悄错的盒子。
    CHECK(box.expanded(-10.0).half_extent == Vector3{0.0, 0.0, 0.0});

    // 三维都为 0 时，紧包围盒退化成一个点（即中心），不是空盒。
    const Box3 degenerate = box.expanded(-10.0).to_axis_aligned();
    CHECK_FALSE(degenerate.is_empty());
    CHECK(degenerate.min == Point3{0.0, 0.0, 0.0});
    CHECK(degenerate.max == Point3{0.0, 0.0, 0.0});

    // 标架原样保留：`expanded` 只动半轴（四个成员都要真的搬过去）。
    CHECK(box.expanded(1.0).frame == box.frame);
    CHECK(box.expanded(-10.0).frame == box.frame);
    CHECK(box.expanded(0.0) == box);            // 零增量是恒等

    // 逐分量的夹取证据：让「只有 x 被夹」「只有 y 被夹」「只有 z 被夹」三种
    // 半残实现各自现形。三个和恰好落在 0 两侧的不同位置。
    CHECK(box.expanded(-1.5).half_extent == Vector3{0.0, 0.5, 1.5});    // x 夹、y/z 不夹
    CHECK(box.expanded(-2.0).half_extent == Vector3{0.0, 0.0, 1.0});    // x/y 夹、z 不夹
    const OrientedBox3 tall{*frame, Vector3{3.0, 1.0, 2.0}};
    CHECK(tall.expanded(-1.5).half_extent == Vector3{1.5, 0.0, 0.5});   // y 夹、x/z 不夹

    // 半轴本身为负（聚合初始化就能造出的非法状态）时，夹取针对的是**和**
    // （`half_extent.i + amount`），不是 amount 的符号。
    const OrientedBox3 inverted{*frame, Vector3{-1.0, 2.0, 3.0}};
    CHECK(inverted.expanded(0.5).half_extent == Vector3{0.0, 2.5, 3.5});

    // `amount` 为 NaN：结果逐分量为 NaN，**不**被静默夹成 0 —— 本类型没有
    // 「空」可以落回，把非有限输入伪装成一个完全正常的退化盒比留下 NaN
    // 更危险。NaN 的下游是确定的（下面两条）：contains 全假、紧包围盒是
    // 规范空盒（角点全是 NaN，逐分量比较一律为假，min/max 各自留在 ±inf）。
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const Vector3 nan_half_extent = box.expanded(nan).half_extent;
    CHECK(std::isnan(nan_half_extent.x));
    CHECK(std::isnan(nan_half_extent.y));
    CHECK(std::isnan(nan_half_extent.z));
    CHECK(box.expanded(nan).to_axis_aligned() == Box3::empty());
    CHECK_FALSE(box.expanded(nan).contains(Point3{0.0, 0.0, 0.0}));
}

TEST_CASE("corner maps the local box through the frame",
          "[linear][orientedbox3]") {
    // 轴置换标架：x_axis = (0,1,0)、y_axis = (0,0,1)、z_axis = (1,0,0)
    // （右手：x × y = (1,0,0) = z）。分量全是 0/1，全部算术精确。
    // 局部 (a,b,c) 送到世界是 (c,a,b)，于是半轴 (1,2,3) 会被送到世界的
    // z/x/y 方向上 —— 「半轴与轴对错位」的实现在这里无处可藏。
    const auto frame = Coordinate3::from_axes(
        Point3{2.0, 0.0, -1.0},
        UnitVector3::from_normalized_unchecked(Vector3{0.0, 1.0, 0.0}),
        UnitVector3::from_normalized_unchecked(Vector3{0.0, 0.0, 1.0}),
        UnitVector3::from_normalized_unchecked(Vector3{1.0, 0.0, 0.0}));
    REQUIRE(frame.has_value());

    const OrientedBox3 box{*frame, Vector3{1.0, 2.0, 3.0}};

    // 中心是**标架原点**（不是局部原点，也不是 (0,0,0)）。
    CHECK(box.center() == Point3{2.0, 0.0, -1.0});

    CHECK(box.corner(0) == Point3{-1.0, -1.0, -3.0});
    CHECK(box.corner(1) == Point3{-1.0, 1.0, -3.0});
    CHECK(box.corner(2) == Point3{-1.0, -1.0, 1.0});
    CHECK(box.corner(3) == Point3{-1.0, 1.0, 1.0});
    CHECK(box.corner(4) == Point3{5.0, -1.0, -3.0});
    CHECK(box.corner(5) == Point3{5.0, 1.0, -3.0});
    CHECK(box.corner(6) == Point3{5.0, -1.0, 1.0});
    CHECK(box.corner(7) == Point3{5.0, 1.0, 1.0});
}

TEST_CASE("to_axis_aligned is the tight box of the corners",
          "[linear][orientedbox3]") {
    // 同一个轴置换标架：紧包围盒的每一端都由不同角点取胜，且三轴尺寸
    // (6,2,4) = (2·3, 2·1, 2·2) 是半轴的**置换** —— 轴向互换可辨。
    const auto frame = Coordinate3::from_axes(
        Point3{2.0, 0.0, -1.0},
        UnitVector3::from_normalized_unchecked(Vector3{0.0, 1.0, 0.0}),
        UnitVector3::from_normalized_unchecked(Vector3{0.0, 0.0, 1.0}),
        UnitVector3::from_normalized_unchecked(Vector3{1.0, 0.0, 0.0}));
    REQUIRE(frame.has_value());

    const OrientedBox3 box{*frame, Vector3{1.0, 2.0, 3.0}};
    const Box3 aabb = box.to_axis_aligned();

    CHECK(aabb.min == Point3{-1.0, -1.0, -3.0});
    CHECK(aabb.max == Point3{5.0, 1.0, 1.0});
    CHECK(aabb.extent() == Vector3{6.0, 2.0, 4.0});

    // 紧：半轴最长者 3、包围球半径 |h| = √14 ≈ 3.74。包围球写法给出的是边长
    // 2√14 ≈ 7.48 的立方体；把半轴直接当世界半宽的写法给出 (2,4,6)。两者都
    // 被上面的 `extent() == (6,2,4)` 排除，下面三条再从**每轴的实际跨度**
    // 复核一次（不经由 `extent()` 的实现）。
    CHECK(aabb.max.x - aabb.min.x == 6.0);
    CHECK(aabb.max.y - aabb.min.y == 2.0);
    CHECK(aabb.max.z - aabb.min.z == 4.0);
    for (int i = 0; i < 8; ++i) {
        CHECK(aabb.contains(box.corner(i)));
    }

    // 整体落在正卦限：正确的 min 是 (8,9,7)。从**零盒**起步取极值的实现会
    // 把它算成 (0,0,0) —— 上面那些跨原点的盒子对这一点不敏感（它们的 min
    // 本来就 <= 0）。这是 `Box3T::empty()` 那个起点的死亡证明。
    const auto positive_frame = Coordinate3::from_z_axis(Point3{10.0, 10.0, 10.0}, z_axis);
    REQUIRE(positive_frame.has_value());
    const Box3 positive_aabb =
        OrientedBox3{*positive_frame, Vector3{1.0, 2.0, 3.0}}.to_axis_aligned();
    CHECK(positive_aabb.min == Point3{8.0, 9.0, 7.0});
    CHECK(positive_aabb.max == Point3{12.0, 11.0, 13.0});

    // 反方向的卦限（整体为负）：max 才是「从零盒起步」会算错的那一端。
    const auto negative_frame = Coordinate3::from_z_axis(Point3{-10.0, -10.0, -10.0}, z_axis);
    REQUIRE(negative_frame.has_value());
    const Box3 negative_aabb =
        OrientedBox3{*negative_frame, Vector3{1.0, 2.0, 3.0}}.to_axis_aligned();
    CHECK(negative_aabb.min == Point3{-12.0, -11.0, -13.0});
    CHECK(negative_aabb.max == Point3{-8.0, -9.0, -7.0});
}

TEST_CASE("containment pins every axis and both ends", "[linear][orientedbox3]") {
    // 轴对齐、非立方、带原点偏移：中心 (5,-3,2)，半轴 (1,2,3)。
    // 三个轴的边界值互不相同，任何一格的交叉引用都会现形。
    const Coordinate3 frame = frame_at(Point3{5.0, -3.0, 2.0});
    const OrientedBox3 box{frame, Vector3{1.0, 2.0, 3.0}};

    CHECK(box.center() == Point3{5.0, -3.0, 2.0});

    // 闭区间：min 角与 max 角都算在内，内部点当然算。
    CHECK(box.contains(Point3{5.0, -3.0, 2.0}));
    CHECK(box.contains(Point3{6.0, -1.0, 5.0}));    // corner(7)
    CHECK(box.contains(Point3{4.0, -5.0, -1.0}));   // corner(0)

    // 六个方向各有一格「只在该轴的一端越界」的证据：`|local.i| <= h.i` 的
    // 三个比较被拆成六个方向，少写任何一个轴的比较、或把某一轴的上界写成
    // 另一轴的，都会在对应格上现形。
    CHECK_FALSE(box.contains(Point3{6.5, -3.0, 2.0}));   // 越 +x
    CHECK_FALSE(box.contains(Point3{5.0, -0.5, 2.0}));   // 越 +y
    CHECK_FALSE(box.contains(Point3{5.0, -3.0, 5.5}));   // 越 +z
    CHECK_FALSE(box.contains(Point3{3.5, -3.0, 2.0}));   // 越 -x
    CHECK_FALSE(box.contains(Point3{5.0, -5.5, 2.0}));   // 越 -y
    CHECK_FALSE(box.contains(Point3{5.0, -3.0, -1.5}));  // 越 -z

    // **闭区间本身需要独立的见证。** 容差为正时 `<=` 与 `<` 不可分：右端是
    // `h + tol > h`，边界点两边都为真（实测：下面这组是它们的**唯一**死亡
    // 证明 —— 在此之前，把三处 `<=` 全改成 `<` 的变异体在全部 422 条断言下
    // 存活）。把容差显式设为 0 之后右端恰好等于 h，「边界算在内」只剩 `<=`
    // 能满足 —— 三个轴各给一格，单轴变异体各自现形（corner(7) 那格三轴同时
    // 取等，杀不了单轴变异体）。
    const Tolerance exact{0.0, 0.0};
    CHECK(box.contains(Point3{6.0, -3.0, 2.0}, exact));    // 只 x 取上界
    CHECK(box.contains(Point3{5.0, -1.0, 2.0}, exact));    // 只 y 取上界
    CHECK(box.contains(Point3{5.0, -3.0, 5.0}, exact));    // 只 z 取上界
    CHECK(box.contains(Point3{4.0, -5.0, -1.0}, exact));   // corner(0)：三轴都取下界
    CHECK_FALSE(box.contains(Point3{6.0 + 1e-6, -3.0, 2.0}, exact));

    // 八个角全部落在盒里，且紧包围盒就是 [center - h, center + h]：
    // 同一格把**原点参与运算**钉住（把原点漏掉的实现会得到一个以 (0,0,0)
    // 为中心或不含原点的盒子）。
    for (int i = 0; i < 8; ++i) {
        CHECK(box.contains(box.corner(i)));
    }
    const Box3 aabb = box.to_axis_aligned();
    CHECK(aabb.min == Point3{4.0, -5.0, -1.0});
    CHECK(aabb.max == Point3{6.0, -1.0, 5.0});
    CHECK(box.corner(0) == Point3{4.0, -5.0, -1.0});
    CHECK(box.corner(7) == Point3{6.0, -1.0, 5.0});
}

TEST_CASE("containment threads its tolerance through every axis",
          "[linear][orientedbox3]") {
    const Coordinate3 frame = frame_at(Point3{0.0, 0.0, 0.0});
    const OrientedBox3 box{frame, Vector3{1.0, 2.0, 3.0}};

    // 三个轴各一格：点在边界外 1e-3。默认容差（rel 1e-9）拒绝；
    // 显式传入的 1e-2 接受。容差若只穿到某一个轴上，另外两格会失败。
    const Point3 outside_x{1.001, 0.0, 0.0};
    const Point3 outside_y{0.0, 2.001, 0.0};
    const Point3 outside_z{0.0, 0.0, 3.001};
    CHECK_FALSE(box.contains(outside_x));
    CHECK_FALSE(box.contains(outside_y));
    CHECK_FALSE(box.contains(outside_z));

    const Tolerance loose{1e-2, 0.0};
    CHECK(box.contains(outside_x, loose));
    CHECK(box.contains(outside_y, loose));
    CHECK(box.contains(outside_z, loose));

    // 容差不是「全接受」：显式 1e-2 之下 0.5 的越界仍被拒（三个轴各一格）。
    CHECK_FALSE(box.contains(Point3{1.5, 0.0, 0.0}, loose));
    CHECK_FALSE(box.contains(Point3{0.0, 2.5, 0.0}, loose));
    CHECK_FALSE(box.contains(Point3{0.0, 0.0, 3.5}, loose));

    // 盒内的点在显式容差下同样被接受（反向自证：上面三格不是「恒真」的）。
    CHECK(box.contains(Point3{0.999, 0.0, 0.0}, loose));
    CHECK(box.contains(Point3{0.999, 1.999, 2.999}, loose));
}

TEST_CASE("containment's tolerance absorbs the local round trip",
          "[linear][orientedbox3]") {
    // `contains` 带容差而 `Box3T::contains` 不带，理由不是「有向盒更模糊」，
    // 而是**点必须经 `to_local` 往返一次**：先减原点、再与三根轴做点积
    // （六次乘加），一个恰好落在角点上的点会带上 1 ulp 量级的误差。
    // 这一格刻意让误差的**符号为正**（角点按精确比较落在盒外）—— 标架取
    // 世界 z 轴绕 (1,1,1) 补全、半轴 (1,2,3)，八个角都有若干轴的误差为正。
    const auto z = Vector3{1.0, 1.0, 1.0}.normalized();
    REQUIRE(z.has_value());
    const auto frame = Coordinate3::from_z_axis(Point3{0.0, 0.0, 0.0}, *z);
    REQUIRE(frame.has_value());

    const OrientedBox3 box{*frame, Vector3{1.0, 2.0, 3.0}};

    const Point3 local = frame->to_local(box.corner(0));
    REQUIRE(std::abs(local.x) > box.half_extent.x);
    REQUIRE(std::abs(local.y) > box.half_extent.y);
    REQUIRE(std::abs(local.z) > box.half_extent.z);

    // 零容差把角点判在盒外，默认容差把它吸收掉 —— 两条合起来说明容差参数
    // 不是装饰，且默认值恰好够用（误差 ~1e-15 远小于默认的相对项 1e-9）。
    CHECK_FALSE(box.contains(box.corner(0), Tolerance{0.0, 0.0}));
    CHECK(box.contains(box.corner(0)));

    // 八个角都是这样。
    for (int i = 0; i < 8; ++i) {
        CHECK_FALSE(box.contains(box.corner(i), Tolerance{0.0, 0.0}));
        CHECK(box.contains(box.corner(i)));
    }
}

TEST_CASE("oriented box equality has evidence in every member",
          "[linear][orientedbox3]") {
    // `==` 要比较**七个**标量：frame 的原点 + 三根轴，以及 half_extent 的三个
    // 分量。每一个都要有独立的不同证据 —— 只写一条「整体相等」时，漏掉任何
    // 一个成员都看不出来。
    const auto base_frame = Coordinate3::from_axes(Point3{0.0, 0.0, 0.0}, ex, ey, z_axis);
    REQUIRE(base_frame.has_value());
    const OrientedBox3 base{*base_frame, Vector3{1.0, 2.0, 3.0}};

    CHECK(base == OrientedBox3{*base_frame, Vector3{1.0, 2.0, 3.0}});

    // 原点：同一组轴、不同原点。
    const auto moved_frame = Coordinate3::from_axes(Point3{1.0, 0.0, 0.0}, ex, ey, z_axis);
    REQUIRE(moved_frame.has_value());
    CHECK(base != OrientedBox3{*moved_frame, Vector3{1.0, 2.0, 3.0}});

    // 三根轴各一格。右手正交标架里任意两条轴唯一决定第三条，所以「精确只差
    // 一根轴」的另一组标架不**存在** —— 用 1e-10 的偏移造出「只在那一根轴上
    // 不同」的标架（长度偏差 1e-20、点积偏差 1e-10 都落在显式放宽容差的松弛
    // 区内），于是每一格只由那一根轴自己的比较决定。
    const Tolerance loose{1e-6, 1e-6};
    const auto tilted_x = Coordinate3::from_axes(
        Point3{0.0, 0.0, 0.0},
        UnitVector3::from_normalized_unchecked(Vector3{1.0, 1e-10, 0.0}), ey, z_axis, loose);
    REQUIRE(tilted_x.has_value());
    CHECK(base != OrientedBox3{*tilted_x, Vector3{1.0, 2.0, 3.0}});

    const auto tilted_y = Coordinate3::from_axes(
        Point3{0.0, 0.0, 0.0}, ex,
        UnitVector3::from_normalized_unchecked(Vector3{0.0, 1.0, 1e-10}), z_axis, loose);
    REQUIRE(tilted_y.has_value());
    CHECK(base != OrientedBox3{*tilted_y, Vector3{1.0, 2.0, 3.0}});

    const auto tilted_z = Coordinate3::from_axes(
        Point3{0.0, 0.0, 0.0}, ex, ey,
        UnitVector3::from_normalized_unchecked(Vector3{1e-10, 0.0, 1.0}), loose);
    REQUIRE(tilted_z.has_value());
    CHECK(base != OrientedBox3{*tilted_z, Vector3{1.0, 2.0, 3.0}});

    // half_extent 的三个分量各一格。
    CHECK(base != OrientedBox3{*base_frame, Vector3{9.0, 2.0, 3.0}});
    CHECK(base != OrientedBox3{*base_frame, Vector3{1.0, 9.0, 3.0}});
    CHECK(base != OrientedBox3{*base_frame, Vector3{1.0, 2.0, 9.0}});

    // 相等那一条也要在**每个成员都相同**时才成立（反向自证）。
    CHECK_FALSE(base != OrientedBox3{*base_frame, Vector3{1.0, 2.0, 3.0}});
}

TEST_CASE("the implicitly generated special members carry the frame and the half extent",
          "[linear][orientedbox3]") {
    const Coordinate3 frame = frame_at(Point3{1.0, -2.0, 3.0});
    const OrientedBox3 source{frame, Vector3{1.0, 2.0, 3.0}};

    // 拷贝赋值：只赋 half_extent、或只赋 frame 的手臂实现会在这里现形。
    OrientedBox3 target{frame_at(Point3{9.0, 9.0, 9.0}), Vector3{9.0, 9.0, 9.0}};
    target = source;
    CHECK(target.frame == source.frame);
    CHECK(target.half_extent == Vector3{1.0, 2.0, 3.0});
    CHECK(target.center() == Point3{1.0, -2.0, 3.0});
    CHECK(target.corner(7) == source.corner(7));

    // 拷贝构造。
    const OrientedBox3 copied = target;
    CHECK(copied.frame == source.frame);
    CHECK(copied.half_extent == Vector3{1.0, 2.0, 3.0});
    CHECK(copied.corner(7) == source.corner(7));

    // 移动构造。
    OrientedBox3 moved_source{frame, Vector3{4.0, 5.0, 6.0}};
    OrientedBox3 moved = std::move(moved_source);
    CHECK(moved.frame == frame);
    CHECK(moved.half_extent == Vector3{4.0, 5.0, 6.0});

    // 移动赋值。
    OrientedBox3 moved_into{frame_at(Point3{0.0, 0.0, 0.0}), Vector3{0.0, 0.0, 0.0}};
    moved_into = std::move(moved);
    CHECK(moved_into.frame == frame);
    CHECK(moved_into.half_extent == Vector3{4.0, 5.0, 6.0});
    CHECK(moved_into.corner(7) == Point3{5.0, 3.0, 9.0});

    // 析构与拷贝/移动的「无行为」是这一类型的规格：这些成员都是隐式生成的。
    // 这**不是**用特征代理行为（本类型没有可观测的析构副作用），而是直接
    // 陈述「这些成员是平凡/隐式的」这一条规格本身（写成 `= default` 同样为真，
    // 证明逐位搬运的是上面那四组断言）。
    STATIC_REQUIRE(std::is_trivially_copyable_v<OrientedBox3T<double>>);
    STATIC_REQUIRE(std::is_trivially_destructible_v<OrientedBox3T<double>>);
}

TEST_CASE("every declared callable is noexcept", "[linear][orientedbox3]") {
    constexpr Coordinate3T<double> frame = Coordinate3T<double>::identity();
    const OrientedBox3T<double> box{frame, Vector3T<double>{1.0, 2.0, 3.0}};

    // 各条之间无依赖，去掉**任意一处** noexcept 都会单独失败 —— 最后一条
    // `!=` 除外，理由见它自己的注释。
    STATIC_REQUIRE(noexcept(box.center()));
    STATIC_REQUIRE(noexcept(box.contains(Point3T<double>{})));
    STATIC_REQUIRE(noexcept(box.contains(Point3T<double>{}, Tolerance{})));
    STATIC_REQUIRE(noexcept(box.corner(0)));
    STATIC_REQUIRE(noexcept(box.to_axis_aligned()));
    STATIC_REQUIRE(noexcept(box.expanded(0.0)));
    STATIC_REQUIRE(noexcept(box == box));
    // 这一条相对上一条是**零独立证据**：C++20 的 `!=` 是 `!(a == b)` 的重写，
    // noexcept 规格继承自被重写的 `operator==`。保留它只为把「生成的 `!=`
    // 也被真的用过」写出来，不要把它计入覆盖率。
    STATIC_REQUIRE(noexcept(box != box));
}

TEST_CASE("every callable is usable in a constant expression",
          "[linear][orientedbox3]") {
    // `identity()` 是 constexpr 工厂，于是整条链都能在常量表达式里求值。
    // 逐个删掉对应成员的 `constexpr`，这里就编译不过。
    constexpr Coordinate3T<double> frame = Coordinate3T<double>::identity();
    constexpr OrientedBox3T<double> box{frame, Vector3T<double>{1.0, 2.0, 3.0}};
    constexpr Box3T<double> aabb = box.to_axis_aligned();

    STATIC_REQUIRE(box.center() == Point3T<double>{0.0, 0.0, 0.0});
    STATIC_REQUIRE(box.corner(0) == Point3T<double>{-1.0, -2.0, -3.0});
    STATIC_REQUIRE(box.corner(3) == Point3T<double>{1.0, 2.0, -3.0});
    STATIC_REQUIRE(box.corner(5) == Point3T<double>{1.0, -2.0, 3.0});
    STATIC_REQUIRE(box.corner(6) == Point3T<double>{-1.0, 2.0, 3.0});
    STATIC_REQUIRE(aabb.min == Point3T<double>{-1.0, -2.0, -3.0});
    STATIC_REQUIRE(aabb.max == Point3T<double>{1.0, 2.0, 3.0});
    STATIC_REQUIRE(box.contains(Point3T<double>{1.0, 2.0, 3.0}));
    STATIC_REQUIRE_FALSE(box.contains(Point3T<double>{1.0, 2.0, 3.5}));
    STATIC_REQUIRE(box.expanded(1.0).half_extent == Vector3T<double>{2.0, 3.0, 4.0});
    STATIC_REQUIRE(box.expanded(-10.0).half_extent == Vector3T<double>{0.0, 0.0, 0.0});
    STATIC_REQUIRE_FALSE(box.expanded(-10.0).contains(Point3T<double>{0.5, 0.0, 0.0}));
    STATIC_REQUIRE(box == OrientedBox3T<double>{frame, Vector3T<double>{1.0, 2.0, 3.0}});
    STATIC_REQUIRE(box != box.expanded(1.0));
}

TEST_CASE("the float instantiation is usable", "[linear][orientedbox3]") {
    const auto frame = Coordinate3T<float>::identity();
    const OrientedBox3f box{frame, Vector3T<float>{1.0f, 2.0f, 3.0f}};

    CHECK(box.center() == Point3T<float>{0.0f, 0.0f, 0.0f});
    CHECK(box.corner(3) == Point3T<float>{1.0f, 2.0f, -3.0f});
    CHECK(box.to_axis_aligned().min == Point3T<float>{-1.0f, -2.0f, -3.0f});
    CHECK(box.to_axis_aligned().max == Point3T<float>{1.0f, 2.0f, 3.0f});
    CHECK(box.expanded(1.0f).half_extent == Vector3T<float>{2.0f, 3.0f, 4.0f});

    // 默认容差按 double 定标（rel 1e-9），float 调用者必须显式给容差 ——
    // 与 Coordinate / Vector 那边同一条口径。1e-3 的越界在两条路径上
    // 一拒一收，顺带证明 float 上的容差参数确实被穿到底。
    const Point3T<float> outside{1.001f, 0.0f, 0.0f};
    CHECK_FALSE(box.contains(outside));
    CHECK(box.contains(outside, Tolerance{1e-2, 0.0}));
}
