#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <limits>
#include <type_traits>
#include <utility>

#include <GeoCore/linear/Box3.hpp>

using GeoCore::linear::Box3;
using GeoCore::linear::Box3f;
using GeoCore::linear::Box3T;
using GeoCore::linear::Point3;
using GeoCore::linear::Point3T;
using GeoCore::linear::Vector3;

TEST_CASE("the float alias really is the float instantiation", "[linear][box3]") {
    STATIC_REQUIRE(std::is_same_v<Box3f, Box3T<float>>);

    // >>> sweep-add
    // 与 Task 4/5 同一条判据：**两个实例化都要钉**。只钉 float 时，一个把
    // `Box3` 绑到 `Box3T<float>`（或反之）的写法照样全绿。
    STATIC_REQUIRE(std::is_same_v<Box3, Box3T<double>>);
    // 成员别名也必须被**命名**过 —— 未被命名的绑定写错时连一次实例化都不会发生。
    STATIC_REQUIRE(std::is_same_v<Box3T<float>::scalar_type, float>);
    STATIC_REQUIRE(std::is_same_v<Box3T<double>::scalar_type, double>);
    // 聚合性是头文件明写的承诺（「不声明任何构造函数，以保持聚合性」）——
    // 加一个构造函数就悄悄丢掉它，而本文件里处处都在用聚合初始化。
    STATIC_REQUIRE(std::is_aggregate_v<Box3T<double>>);
    STATIC_REQUIRE(std::is_aggregate_v<Box3T<float>>);
    // <<< sweep-add
}

TEST_CASE("an empty box contains nothing and merges as identity",
          "[linear][box3]") {
    const Box3 empty = Box3::empty();
    const Box3 box{Point3{0.0, 0.0, 0.0}, Point3{1.0, 1.0, 1.0}};

    CHECK(empty.is_empty());
    CHECK_FALSE(empty.contains(Point3{0.0, 0.0, 0.0}));
    CHECK_FALSE(empty.intersects(box));
    CHECK(empty.merged(box) == box);
    CHECK(box.merged(empty) == box);
    CHECK(empty.merged(empty) == empty);
}

TEST_CASE("a degenerate box (a point) is not empty", "[linear][box3]") {
    const Box3 point{Point3{1.0, 2.0, 3.0}, Point3{1.0, 2.0, 3.0}};

    CHECK_FALSE(point.is_empty());
    CHECK(point.contains(Point3{1.0, 2.0, 3.0}));
    CHECK(point.extent() == Vector3{0.0, 0.0, 0.0});
}

TEST_CASE("box corners pin every index bit", "[linear][box3]") {
    const Box3 box{Point3{0.0, 0.0, 0.0}, Point3{1.0, 2.0, 4.0}};

    // 索引的 bit0/bit1/bit2 依次选择 x/y/z 取 min 还是 max，置位取 max。
    // 只测 0 与 7 是不够的：那两端恰好是「全 min」与「全 max」，三个轴的
    // 位若被两两互换，0 和 7 仍然都对得上。中间三个索引各只置一位，
    // 把每一位独立钉死。
    CHECK(box.corner(0) == Point3{0.0, 0.0, 0.0});
    CHECK(box.corner(1) == Point3{1.0, 0.0, 0.0});
    CHECK(box.corner(2) == Point3{0.0, 2.0, 0.0});
    CHECK(box.corner(4) == Point3{0.0, 0.0, 4.0});
    CHECK(box.corner(7) == Point3{1.0, 2.0, 4.0});

    CHECK(box.center() == Point3{0.5, 1.0, 2.0});
    CHECK(box.half_extent() == Vector3{0.5, 1.0, 2.0});
    CHECK(box.extent() == Vector3{1.0, 2.0, 4.0});

    // >>> sweep-add
    // 余下三个索引（3/5/6）是**两位同时置位**、且不为 0 也不为 7 的三格，
    // 上面五条一条也钉不住它们：一个「bit1 只在 bit0 为 0 时才生效」的实现
    // （等价于刻表时第 3 项写错）在 0/1/2/4/7 上全部正确，只有 corner(3)
    // 会返回 {max.x, min.y, max.z}。补满八格之后，每个索引的返回值都被钉死。
    CHECK(box.corner(3) == Point3{1.0, 2.0, 0.0});
    CHECK(box.corner(5) == Point3{1.0, 0.0, 4.0});
    CHECK(box.corner(6) == Point3{0.0, 2.0, 4.0});
    // 非均匀盒（1,2,4）让轴向互换、甚至单个分量的交叉引用都无处可藏。
    CHECK(box.corner(7) == box.max);
    CHECK(box.corner(0) == box.min);
    // <<< sweep-add
}

TEST_CASE("box overlap is closed and empty boxes intersect nothing",
          "[linear][box3]") {
    const Box3 a{Point3{0.0, 0.0, 0.0}, Point3{2.0, 2.0, 2.0}};
    const Box3 touching{Point3{2.0, 0.0, 0.0}, Point3{4.0, 2.0, 2.0}};
    const Box3 apart{Point3{3.0, 0.0, 0.0}, Point3{4.0, 2.0, 2.0}};

    CHECK(a.intersects(touching));            // 共享一个面，算相交
    CHECK_FALSE(a.intersects(apart));
    CHECK(a.contains(Box3{Point3{0.5, 0.5, 0.5}, Point3{1.5, 1.5, 1.5}}));
    CHECK_FALSE(a.contains(touching));

    // 空盒与任何盒都不相交，也不包含任何盒，且不被任何盒包含。
    CHECK_FALSE(Box3::empty().intersects(a));
    CHECK_FALSE(a.intersects(Box3::empty()));
    CHECK_FALSE(a.contains(Box3::empty()));
    CHECK_FALSE(Box3::empty().contains(a));
}

TEST_CASE("box extent and center are defined on the empty box",
          "[linear][box3]") {
    // 空盒的测度是 0，不是 -inf。
    CHECK(Box3::empty().extent() == Vector3{0.0, 0.0, 0.0});

    // 没有中心：(+inf) + (-inf) 逐分量为 NaN。刻意如此，不是漏判。
    CHECK(std::isnan(Box3::empty().center().x));
    CHECK(std::isnan(Box3::empty().center().y));
    CHECK(std::isnan(Box3::empty().center().z));

    // center 用 min*0.5 + max*0.5，这两种极端值都能算对。
    CHECK(Box3{Point3{1e308, 0.0, 0.0}, Point3{1e308, 0.0, 0.0}}.center().x == 1e308);
    CHECK(Box3{Point3{-1e308, 0.0, 0.0}, Point3{1e308, 0.0, 0.0}}.center().x == 0.0);

    // >>> sweep-add
    // 上面两条只钉住 x。`(min+max)*0.5` 与 `min+(max-min)*0.5` 这两个溢出陷阱
    // 在 y、z 上是**独立的**代码路径（逐分量写出来就是三段独立的算术），
    // 一条 x 的证据不能替另外两条提供证据 —— 只把 y 写成溢出式照样全绿。
    CHECK(Box3{Point3{0.0, 1e308, 0.0}, Point3{0.0, 1e308, 0.0}}.center().y == 1e308);
    CHECK(Box3{Point3{0.0, -1e308, 0.0}, Point3{0.0, 1e308, 0.0}}.center().y == 0.0);
    CHECK(Box3{Point3{0.0, 0.0, 1e308}, Point3{0.0, 0.0, 1e308}}.center().z == 1e308);
    CHECK(Box3{Point3{0.0, 0.0, -1e308}, Point3{0.0, 0.0, 1e308}}.center().z == 0.0);

    // 空盒的半测度同样是 0（空集的测度是 0，它的一半也是 0）。
    // 少了 `extent()` 那道判空而直写 `(max - min) * 0.5` 会得到 -inf。
    CHECK(Box3::empty().half_extent() == Vector3{0.0, 0.0, 0.0});
    // <<< sweep-add
}

TEST_CASE("merged and expanded keep the canonical empty box",
          "[linear][box3]") {
    const Box3 a{Point3{0.0, 0.0, 0.0}, Point3{1.0, 1.0, 1.0}};
    const Box3 b{Point3{2.0, 2.0, 2.0}, Point3{3.0, 3.0, 3.0}};

    CHECK(a.merged(b) == Box3{Point3{0.0, 0.0, 0.0}, Point3{3.0, 3.0, 3.0}});

    CHECK(a.expanded(1.0) == Box3{Point3{-1.0, -1.0, -1.0}, Point3{2.0, 2.0, 2.0}});

    // 收缩过头得到规范空盒，不是「分量倒置但非规范」的形式。
    CHECK(a.expanded(-2.0) == Box3::empty());

    // 空盒膨胀后仍是空盒，不会变成整个空间。
    CHECK(Box3::empty().expanded(1.0) == Box3::empty());

    // >>> sweep-add
    // 上面 `a.merged(b)` 的两个操作数都是**关于原点对称**的（min = -max），
    // 于是六个结果分量里每一对都各自相等，某个分量取错操作数完全看不出来。
    // 下面这组：a 与 b 的六个分量互不相同、且**每一项的赢家混合来自双方**
    // （min 的 x/z 来自 a、y 来自 b；max 的 x/z 来自 a、y 来自 b），
    // 任何一格的取反、取错操作数、或忘记赋值（落到 `Box3T{}` 的 0）都会失败。
    const Box3 p{Point3{1.0, 2.0, 3.0}, Point3{40.0, 50.0, 60.0}};
    const Box3 q{Point3{10.0, -20.0, 30.0}, Point3{39.0, 55.0, 59.0}};
    const Box3 expected{Point3{1.0, -20.0, 3.0}, Point3{40.0, 55.0, 60.0}};
    CHECK(p.merged(q) == expected);
    CHECK(q.merged(p) == expected);   // 交换律：两条路径不同，结果必须相同
    CHECK(p.merged(p) == p);

    // 同一个非均匀、非零起点的盒把四个派生量一起钉住。上面 corners 用例里的
    // 盒 min 全为 0，`max + min` 与 `max - min` 在那里**无法区分**。
    CHECK(p.extent() == Vector3{39.0, 48.0, 57.0});
    CHECK(p.half_extent() == Vector3{19.5, 24.0, 28.5});
    CHECK(p.center() == Point3{20.5, 26.0, 31.5});
    CHECK(p.expanded(10.0) == Box3{Point3{-9.0, -8.0, -7.0}, Point3{50.0, 60.0, 70.0}});

    // 负值即收缩，且**两端各自的方向都要对**：全部减到 min.x 上、
    // 或把 `- amount` 写成 `+ amount`，都会在下面这一条上现形。
    CHECK(Box3{Point3{0.0, 0.0, 0.0}, Point3{1.0, 1.0, 1.0}}.expanded(-0.25) ==
          Box3{Point3{0.25, 0.25, 0.25}, Point3{0.75, 0.75, 0.75}});
    // <<< sweep-add
}

TEST_CASE("from_corners accepts the two corners in either order",
          "[linear][box3]") {
    const Box3 forward = Box3::from_corners(Point3{0.0, 0.0, 0.0}, Point3{1.0, 2.0, 4.0});
    const Box3 reversed = Box3::from_corners(Point3{1.0, 2.0, 4.0}, Point3{0.0, 0.0, 0.0});

    CHECK(forward == reversed);
    CHECK(forward.min == Point3{0.0, 0.0, 0.0});
    CHECK(forward.max == Point3{1.0, 2.0, 4.0});

    // >>> sweep-add
    // 上面两条只覆盖「整个点整体顺序正确 / 整体反序」两种输入，它们**区分不出**
    // 「按某一个分量决定要不要交换两个点」（整点交换）与「逐分量取 min/max」：
    // 一个 `a.x <= b.x ? Box3{a,b} : Box3{b,a}` 的实现让上面三条全部通过。
    // 混合顺序的输入才把这个差别暴露出来 —— 它的三个分量没有统一的「谁在前」。
    const Box3 mixed = Box3::from_corners(Point3{1.0, 0.0, 4.0}, Point3{0.0, 2.0, 0.0});
    CHECK_FALSE(mixed.is_empty());   // 整点交换会得到一个 y 倒置的空盒
    CHECK(mixed.min == Point3{0.0, 0.0, 0.0});
    CHECK(mixed.max == Point3{1.0, 2.0, 4.0});
    // <<< sweep-add
}

TEST_CASE("box equality has evidence in every component",
          "[linear][box3]") {
    // `==` 要比较**六个**标量（min/max 各三个），每一个都要有独立的不同证据。
    // 只写一条「整体相等」是不够的：漏掉任何一个分量都看不出来。
    // （Task 4 的教训：相等断言不能替不相等断言提供证据，反之亦然。）
    const Box3 base{Point3{1.0, 2.0, 3.0}, Point3{4.0, 5.0, 6.0}};

    CHECK(base == Box3{Point3{1.0, 2.0, 3.0}, Point3{4.0, 5.0, 6.0}});

    CHECK(base != Box3{Point3{9.0, 2.0, 3.0}, Point3{4.0, 5.0, 6.0}});  // min.x
    CHECK(base != Box3{Point3{1.0, 9.0, 3.0}, Point3{4.0, 5.0, 6.0}});  // min.y
    CHECK(base != Box3{Point3{1.0, 2.0, 9.0}, Point3{4.0, 5.0, 6.0}});  // min.z
    CHECK(base != Box3{Point3{1.0, 2.0, 3.0}, Point3{9.0, 5.0, 6.0}});  // max.x
    CHECK(base != Box3{Point3{1.0, 2.0, 3.0}, Point3{4.0, 9.0, 6.0}});  // max.y
    CHECK(base != Box3{Point3{1.0, 2.0, 3.0}, Point3{4.0, 5.0, 9.0}});  // max.z
}

TEST_CASE("box copy assignment carries both corners", "[linear][box3]") {
    // 隐式拷贝赋值也要被真的用一次，否则它从未被实例化 —— Task 4 实测过：
    // obj 里连拷贝赋值的符号都不存在，一个只赋 min 的手写 operator= 完全静默。
    const Box3 source{Point3{1.0, 2.0, 3.0}, Point3{4.0, 5.0, 6.0}};
    Box3 target{Point3{0.0, 0.0, 0.0}, Point3{0.0, 0.0, 0.0}};
    target = source;

    CHECK(target.min == Point3{1.0, 2.0, 3.0});
    CHECK(target.max == Point3{4.0, 5.0, 6.0});
}

TEST_CASE("the empty predicate is total, including on non-finite input",
          "[linear][box3][degenerate]") {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double infinity = std::numeric_limits<double>::infinity();

    // `min > max` 对 NaN 返回 false，等于谎称这是一个正常的非空盒；
    // `!(min <= max)` 才是全函数。
    CHECK(Box3{Point3{nan, 0.0, 0.0}, Point3{1.0, 1.0, 1.0}}.is_empty());

    // 部分倒置也算空。
    CHECK(Box3{Point3{5.0, 0.0, 0.0}, Point3{1.0, 1.0, 1.0}}.is_empty());

    // 空盒被 +inf 膨胀：`+inf - (+inf)` 是 NaN。谓词改全之后自动落回 empty()。
    CHECK(Box3::empty().expanded(infinity) == Box3::empty());

    // 有限盒被 +inf 膨胀得到整个空间，**不是**空盒。
    CHECK_FALSE(Box3{Point3{1.0, 2.0, 3.0}, Point3{4.0, 5.0, 6.0}}
                    .expanded(infinity)
                    .is_empty());

    // `{+inf}³` 不是空盒（它含 +inf 一个点），但 extent 是 `inf - inf` = NaN。
    // 与 Interval 的 length 同类，是「非空却没有有限尺寸」的哨兵，文档写明并钉住。
    const Box3 at_infinity{Point3{infinity, infinity, infinity},
                           Point3{infinity, infinity, infinity}};
    CHECK_FALSE(at_infinity.is_empty());
    CHECK(std::isnan(at_infinity.extent().x));

    // >>> sweep-add
    // 上面两条只钉住 x 分量的 NaN。谓词是三个分量的**析取**，把另外两个
    // 分量漏掉（`!(min.x <= max.x) || !(min.y <= max.y)`）在 x 那一条上照样
    // 是真的 —— 而漏判的后果是这一格谎称「非空」。
    CHECK(Box3{Point3{0.0, nan, 0.0}, Point3{1.0, 1.0, 1.0}}.is_empty());
    CHECK(Box3{Point3{0.0, 0.0, nan}, Point3{1.0, 1.0, 1.0}}.is_empty());
    CHECK(Box3{Point3{0.0, 5.0, 0.0}, Point3{1.0, 1.0, 1.0}}.is_empty());
    CHECK(Box3{Point3{0.0, 0.0, 5.0}, Point3{1.0, 1.0, 1.0}}.is_empty());
    // 三个分量同时成立才算非空：`&&` 写成了 `||` 在这里现形。
    CHECK_FALSE(Box3{Point3{0.0, 0.0, 0.0}, Point3{1.0, 1.0, 1.0}}.is_empty());

    // 闭区间的包含在 +inf 上仍然成立（它不是「无穷大的盒」而是含 +inf 一个点）。
    CHECK(at_infinity.contains(Point3{infinity, infinity, infinity}));
    // 但任何有限点都不在里面 —— `>= min` 这一半必须真的在。
    CHECK_FALSE(at_infinity.contains(Point3{1.0, 1.0, 1.0}));

    // 有限盒被 -inf 「膨胀」得到的是规范空盒（min 变 +inf、max 变 -inf），
    // 与 `expanded(-2.0)` 同类，同样是规范化出口。
    CHECK(Box3{Point3{1.0, 2.0, 3.0}, Point3{4.0, 5.0, 6.0}}.expanded(-infinity) ==
          Box3::empty());

    // amount 是 NaN 时逐分量算出 NaN（`0 - NaN`），全函数谓词判中间结果为真，
    // 于是**同样落回规范空盒** —— 与 `+inf` 那一格同源，是「所有非有限输入都走
    // 同一条规范化出口」的推论。下面这条是本文件里唯一走「有限盒 + NaN 增量」
    // 这条路的断言（`empty().expanded(inf)` 走的是另一个入口）。
    CHECK(Box3{Point3{1.0, 2.0, 3.0}, Point3{4.0, 5.0, 6.0}}.expanded(nan) ==
          Box3::empty());
    // <<< sweep-add
}

TEST_CASE("the consumers agree with the total empty predicate",
          "[linear][box3][degenerate]") {
    // `is_empty()` 说含 NaN 的盒是空 —— 这是对的，它不含任何点。
    // 三个消费者必须跟着这么认为，否则谓词只是装饰，而且换方向答案会翻面。
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const Box3 nan_box{Point3{nan, 0.0, 0.0}, Point3{nan, 0.0, 0.0}};
    const Box3 normal{Point3{1.0, 2.0, 3.0}, Point3{4.0, 5.0, 6.0}};

    REQUIRE(nan_box.is_empty());

    // 相交：两个方向都必须为假（对称性）。
    CHECK_FALSE(normal.intersects(nan_box));
    CHECK_FALSE(nan_box.intersects(normal));

    // 包含：空盒不被任何盒包含，也不包含任何盒。
    CHECK_FALSE(normal.contains(nan_box));
    CHECK_FALSE(nan_box.contains(normal));

    // 合并：换方向答案相同（交换律）。
    CHECK(nan_box.merged(normal) == normal);
    CHECK(normal.merged(nan_box) == normal);

    // 两个「空」的表示不同时，**规范化优先于恒等律**（同 Interval）。
    const Box3 non_canonical_empty{Point3{0.0, 0.0, 0.0}, Point3{-1.0, -1.0, -1.0}};
    CHECK(non_canonical_empty.is_empty());
    CHECK(nan_box.merged(non_canonical_empty) == Box3::empty());
    CHECK(non_canonical_empty.merged(nan_box) == Box3::empty());

    // >>> sweep-add
    // 空盒的测度是 0，含 NaN 的空盒也一样 —— `extent()` 的守卫若从全函数
    // 谓词退化成 `min.x > max.x`，这一格会返回 `NaN - NaN` 而不是 0。
    CHECK(nan_box.extent() == Vector3{0.0, 0.0, 0.0});

    // `contains(Point3T)` 没有（也不需要）判空守卫：逐分量比较碰上 NaN 一律
    // 为假。但**写成双重否定就会翻面** —— `!(p.x < min.x) && !(p.x > max.x)`
    // 这种「不小于下界且不大于上界」的常见写法在 NaN 上两边都为真。
    CHECK_FALSE(nan_box.contains(Point3{0.0, 0.0, 0.0}));
    CHECK_FALSE(normal.contains(Point3{nan, 2.0, 3.0}));

    // 恒等律的另一半：非空的盒与非规范空合并，必须原样返回自己。
    // 少了 `if (other.is_empty()) return *this;` 会退化成逐分量取外扩，
    // 于是把 `{-1,-1,-1}` 这个**假的** max 端吃进结果里。
    CHECK(normal.merged(non_canonical_empty) == normal);
    // <<< sweep-add
}

// >>> sweep-add
TEST_CASE("the canonical empty box is pinned component by component",
          "[linear][box3]") {
    // 规范表示是本任务的关键约定（Task 8 明写依赖它）。只断言
    // `x == Box3::empty()` 是**两边一起动**的：empty() 自己被改成别的形式时，
    // 全部等价断言照样通过。所以六个端点值要逐条钉死，不经由另一个可能
    // 一起变的值来比较。
    const double infinity = std::numeric_limits<double>::infinity();
    const Box3 empty = Box3::empty();

    CHECK(empty.min.x == infinity);
    CHECK(empty.min.y == infinity);
    CHECK(empty.min.z == infinity);
    CHECK(empty.max.x == -infinity);
    CHECK(empty.max.y == -infinity);
    CHECK(empty.max.z == -infinity);

    // 反证：一个把 min 与 max 对调的空盒不是空盒。
    CHECK_FALSE(Box3{Point3{-infinity, -infinity, -infinity},
                     Point3{infinity, infinity, infinity}}
                    .is_empty());

    // 生产者必须给出**同一个**规范形式。`expanded` 是唯一会「收缩过头」的
    // 生产者，它的六个端点也逐条钉死（`== Box3::empty()` 是自指的）。
    const Box3 shrunk = Box3{Point3{0.0, 0.0, 0.0}, Point3{1.0, 1.0, 1.0}}.expanded(-2.0);
    CHECK(shrunk.min.x == infinity);
    CHECK(shrunk.min.y == infinity);
    CHECK(shrunk.min.z == infinity);
    CHECK(shrunk.max.x == -infinity);
    CHECK(shrunk.max.y == -infinity);
    CHECK(shrunk.max.z == -infinity);

    // 非规范空的输入不是「另一种空」，而是一个 is_empty() 为真的**输入**。
    // 与**非空**的盒合并时恒等律成立：结果就是那个非空的盒。规范化只约束
    // 「产出空」的那条路径，不是「见到空的输入就返回 empty()」。
    const Box3 non_canonical_empty{Point3{0.0, 0.0, 0.0}, Point3{-1.0, -1.0, -1.0}};
    const Box3 other{Point3{2.0, 2.0, 2.0}, Point3{3.0, 3.0, 3.0}};
    CHECK(non_canonical_empty.merged(other) == other);
    CHECK(other.merged(non_canonical_empty) == other);

    // 而两个**表示不同**的空盒（非规范空 + 规范空）合并时，产出必须是规范的
    // 六个分量：**规范化优先于恒等律**。只断言 `== Box3::empty()` 是自指的
    // （右式会跟着左式一起变），所以逐分量钉。
    const Box3 canon = non_canonical_empty.merged(Box3::empty());
    CHECK(canon.min.x == infinity);
    CHECK(canon.min.y == infinity);
    CHECK(canon.min.z == infinity);
    CHECK(canon.max.x == -infinity);
    CHECK(canon.max.y == -infinity);
    CHECK(canon.max.z == -infinity);
    // 反方向同样如此（交换律）。
    CHECK(Box3::empty().merged(non_canonical_empty) == canon);
}

TEST_CASE("point containment pins each comparison and closes both ends",
          "[linear][box3]") {
    // 非均匀盒：x/y/z 的六个边界值互不相同，任何一格的交叉引用都现形。
    const Box3 box{Point3{0.0, 0.0, 0.0}, Point3{1.0, 2.0, 4.0}};

    // 闭区间：min 角与 max 角都算在内，内部点当然算。
    CHECK(box.contains(Point3{0.0, 0.0, 0.0}));
    CHECK(box.contains(Point3{1.0, 2.0, 4.0}));
    CHECK(box.contains(Point3{0.5, 1.0, 2.0}));

    // 六个边界各有一格越界证据。`p <= max` 那一半若整条缺失（只留 `p >= min`），
    // 前两格照样为真 —— 三条「越上界」是它唯一的证据；反过来，`p >= min` 若
    // 缺失，则靠三条「越下界」。
    CHECK_FALSE(box.contains(Point3{1.5, 2.0, 4.0}));   // 越 max.x
    CHECK_FALSE(box.contains(Point3{1.0, 2.5, 4.0}));   // 越 max.y
    CHECK_FALSE(box.contains(Point3{1.0, 2.0, 4.5}));   // 越 max.z
    CHECK_FALSE(box.contains(Point3{-0.5, 0.0, 0.0}));  // 越 min.x
    CHECK_FALSE(box.contains(Point3{0.0, -0.5, 0.0}));  // 越 min.y
    CHECK_FALSE(box.contains(Point3{0.0, 0.0, -0.5}));  // 越 min.z

    // 空盒（两种）都不含任何点，且不需要特判 —— 这正是选这个空表示的理由。
    CHECK_FALSE(Box3::empty().contains(Point3{0.0, 0.0, 0.0}));
    const double nan = std::numeric_limits<double>::quiet_NaN();
    CHECK_FALSE(Box3{Point3{nan, nan, nan}, Point3{nan, nan, nan}}
                    .contains(Point3{nan, nan, nan}));
}

TEST_CASE("box containment pins each comparison and refuses non-canonical empties",
          "[linear][box3]") {
    // 非均匀盒，六个边界互不相同。
    const Box3 a{Point3{0.0, 0.0, 0.0}, Point3{1.0, 2.0, 4.0}};

    // 自己包含自己（闭区间），这是「`<` 写成 `<=`」的唯一证据。
    CHECK(a.contains(Box3{Point3{0.0, 0.0, 0.0}, Point3{1.0, 2.0, 4.0}}));

    // `max >= other.max` 若整条缺失（只留 `min <= other.min`），下面三格全都
    // 会翻成 true；`min <= other.min` 若缺失，则由另外三格兜住。
    CHECK_FALSE(a.contains(Box3{Point3{0.0, 0.0, 0.0}, Point3{1.5, 2.0, 4.0}}));
    CHECK_FALSE(a.contains(Box3{Point3{0.0, 0.0, 0.0}, Point3{1.0, 2.5, 4.0}}));
    CHECK_FALSE(a.contains(Box3{Point3{0.0, 0.0, 0.0}, Point3{1.0, 2.0, 4.5}}));
    CHECK_FALSE(a.contains(Box3{Point3{-0.5, 0.0, 0.0}, Point3{1.0, 2.0, 4.0}}));
    CHECK_FALSE(a.contains(Box3{Point3{0.0, -0.5, 0.0}, Point3{1.0, 2.0, 4.0}}));
    CHECK_FALSE(a.contains(Box3{Point3{0.0, 0.0, -0.5}, Point3{1.0, 2.0, 4.0}}));

    // **非规范空盒必须被显式判空挡掉。** 朴素逐分量比较在它身上会给出 true
    // （`min <= 0` 与 `max >= -1` 同时成立），也就是「盒里装着空集」。
    const Box3 non_canonical_empty{Point3{0.0, 0.0, 0.0}, Point3{-1.0, -1.0, -1.0}};
    REQUIRE(non_canonical_empty.is_empty());
    CHECK_FALSE(a.contains(non_canonical_empty));
    CHECK_FALSE(non_canonical_empty.contains(a));
}

TEST_CASE("intersects is per-axis and refuses non-canonical empties",
          "[linear][box3]") {
    const Box3 a{Point3{0.0, 0.0, 0.0}, Point3{1.0, 2.0, 4.0}};

    // 在一个角上相接，算相交（闭盒）。`<=` 写成 `<` 只由这一条抓得到。
    CHECK(a.intersects(Box3{Point3{1.0, 2.0, 4.0}, Point3{2.0, 3.0, 5.0}}));

    // 三个轴各一格「只在这一点上分开」的证据。少写任何一个轴的比较，
    // 对应的那一格就会翻成 true —— 计划的 apart 盒只在 x 上分开。
    const Box3 apart_x{Point3{1.5, 0.0, 0.0}, Point3{2.5, 3.0, 5.0}};
    const Box3 apart_y{Point3{0.0, 2.5, 0.0}, Point3{1.0, 3.5, 5.0}};
    const Box3 apart_z{Point3{0.0, 0.0, 4.5}, Point3{1.0, 2.0, 5.5}};
    CHECK_FALSE(a.intersects(apart_x));
    CHECK_FALSE(a.intersects(apart_y));
    CHECK_FALSE(a.intersects(apart_z));

    // 反方向。`a.intersects(b)` 的两条半比较各管一个方向，只留一条时
    // 正向可能被 apart 盒抓到、反向却翻面 —— 上面三条对调操作数即可。
    CHECK_FALSE(apart_x.intersects(a));
    CHECK_FALSE(apart_y.intersects(a));
    CHECK_FALSE(apart_z.intersects(a));

    // 完全在另一侧（不含相接）也算不相交。
    const Box3 left{Point3{-2.0, -1.0, -1.0}, Point3{-1.0, 1.0, 1.0}};
    CHECK_FALSE(a.intersects(left));
    CHECK_FALSE(left.intersects(a));

    // **非规范空盒与任何盒都不相交，必须显式判空。** 朴素的逐分量比较会
    // 返回 true：这个空盒的 min/max 是「倒置」的 {0}–{-1}，一个横跨它两端的
    // 盒会同时满足 `min <= other.max` 与 `max >= other.min` —— 空集与别人
    // 「相交」了。`spanning` 就是这样的盒（它含 [0, -1] 这段），`a` 不是
    // （a.min.x = 0 不 <= -1），所以 `a` 上这一格看不出差别。
    const Box3 non_canonical_empty{Point3{0.0, 0.0, 0.0}, Point3{-1.0, -1.0, -1.0}};
    const Box3 spanning{Point3{-5.0, -5.0, -5.0}, Point3{5.0, 5.0, 5.0}};
    REQUIRE(non_canonical_empty.is_empty());
    CHECK_FALSE(spanning.intersects(non_canonical_empty));
    CHECK_FALSE(non_canonical_empty.intersects(spanning));
    CHECK_FALSE(a.intersects(non_canonical_empty));
    CHECK_FALSE(non_canonical_empty.intersects(a));
}

TEST_CASE("the implicitly generated special members carry both corners",
          "[linear][box3]") {
    // 默认构造 + 成员初始化值 `Point3T<Scalar> min{}` —— **用常量求值钉，
    // 不要用类型特征**。常量表达式里读不确定值是编译错误，六个分量逐个钉死。
    // `default_box` 不能写花括号 —— 写了就是聚合的值初始化，有没有初始化值
    // 都会清零。
    constexpr Box3T<double> default_box;
    STATIC_REQUIRE(default_box.min.x == 0.0);
    STATIC_REQUIRE(default_box.min.y == 0.0);
    STATIC_REQUIRE(default_box.min.z == 0.0);
    STATIC_REQUIRE(default_box.max.x == 0.0);
    STATIC_REQUIRE(default_box.max.y == 0.0);
    STATIC_REQUIRE(default_box.max.z == 0.0);

    // 另外两个工厂是 Interfaces 明文的 `static constexpr`：在常量表达式里
    // 求值是最直接的证据，去掉 constexpr 就编不过。
    constexpr Box3T<double> at_origin = Box3T<double>::empty().merged(
        Box3T<double>{Point3T<double>{1.0, 2.0, 3.0}, Point3T<double>{4.0, 5.0, 6.0}});
    STATIC_REQUIRE(at_origin.min.x == 1.0);
    STATIC_REQUIRE(at_origin.min.y == 2.0);
    STATIC_REQUIRE(at_origin.min.z == 3.0);
    STATIC_REQUIRE(at_origin.max.x == 4.0);
    STATIC_REQUIRE(at_origin.max.y == 5.0);
    STATIC_REQUIRE(at_origin.max.z == 6.0);

    constexpr Box3T<double> sorted = Box3T<double>::from_corners(
        Point3T<double>{4.0, 2.0, 6.0}, Point3T<double>{1.0, 5.0, 3.0});
    STATIC_REQUIRE(sorted.min.x == 1.0);
    STATIC_REQUIRE(sorted.min.y == 2.0);
    STATIC_REQUIRE(sorted.min.z == 3.0);
    STATIC_REQUIRE(sorted.max.x == 4.0);
    STATIC_REQUIRE(sorted.max.y == 5.0);
    STATIC_REQUIRE(sorted.max.z == 6.0);

    constexpr Point3T<double> corner_one =
        Box3T<double>{Point3T<double>{1.0, 2.0, 3.0}, Point3T<double>{4.0, 5.0, 6.0}}.corner(1);
    STATIC_REQUIRE(corner_one.x == 4.0);
    STATIC_REQUIRE(corner_one.y == 2.0);
    STATIC_REQUIRE(corner_one.z == 3.0);

    // 隐式拷贝/移动构造与移动赋值同样要真的用一次（Task 4 的教训：从仓库测试
    // 编出的 obj 里连拷贝赋值的符号都不存在，手写一个只赋 min 的实现完全静默）。
    // 拷贝构造与移动构造若被手写替换会破坏聚合性（响亮），但**赋值运算符
    // 不会** —— 这两格是它们唯一的证据。
    const Box3 source{Point3{1.0, 2.0, 3.0}, Point3{4.0, 5.0, 6.0}};
    Box3 target{Point3{0.0, 0.0, 0.0}, Point3{0.0, 0.0, 0.0}};

    target = source;                     // 拷贝赋值
    CHECK(target.min == Point3{1.0, 2.0, 3.0});
    CHECK(target.max == Point3{4.0, 5.0, 6.0});

    Box3 copied = target;                // 拷贝构造
    CHECK(copied.min == Point3{1.0, 2.0, 3.0});
    CHECK(copied.max == Point3{4.0, 5.0, 6.0});

    Box3 moved = std::move(copied);      // 移动构造
    CHECK(moved.min == Point3{1.0, 2.0, 3.0});
    CHECK(moved.max == Point3{4.0, 5.0, 6.0});

    Box3 moved_into{Point3{9.0, 9.0, 9.0}, Point3{9.0, 9.0, 9.0}};
    moved_into = std::move(moved);       // 移动赋值
    CHECK(moved_into.min == Point3{1.0, 2.0, 3.0});
    CHECK(moved_into.max == Point3{4.0, 5.0, 6.0});

    // 析构与拷贝/移动的「无行为」是这一类型的规格：三个类别都必须是隐式的。
    // 这**不是**用特征代理行为（本类型没有可观测的析构/拷贝副作用），
    // 而是直接陈述「这些成员是隐式生成的」这一条规格本身。
    STATIC_REQUIRE(std::is_trivially_copyable_v<Box3T<double>>);
    STATIC_REQUIRE(std::is_trivially_destructible_v<Box3T<double>>);
}

TEST_CASE("every declared callable is noexcept", "[linear][box3]") {
    // Interfaces 对本类型的两个工厂明文写了 noexcept，成员一律 noexcept 是全库
    // 惯例。各条之间无依赖，去掉**任意一处** noexcept 都会单独失败 ——
    // 最后一条 `!=` 除外，理由见它自己的注释。
    STATIC_REQUIRE(noexcept(Box3T<double>::empty()));
    STATIC_REQUIRE(noexcept(Box3T<double>::from_corners(Point3T<double>{}, Point3T<double>{})));
    STATIC_REQUIRE(noexcept(Box3T<double>{}.is_empty()));
    STATIC_REQUIRE(noexcept(Box3T<double>{}.contains(Point3T<double>{})));
    STATIC_REQUIRE(noexcept(Box3T<double>{}.contains(Box3T<double>{})));
    STATIC_REQUIRE(noexcept(Box3T<double>{}.intersects(Box3T<double>{})));
    STATIC_REQUIRE(noexcept(Box3T<double>{}.center()));
    STATIC_REQUIRE(noexcept(Box3T<double>{}.extent()));
    STATIC_REQUIRE(noexcept(Box3T<double>{}.half_extent()));
    STATIC_REQUIRE(noexcept(Box3T<double>{}.merged(Box3T<double>{})));
    STATIC_REQUIRE(noexcept(Box3T<double>{}.expanded(0.0)));
    STATIC_REQUIRE(noexcept(Box3T<double>{}.corner(0)));
    STATIC_REQUIRE(noexcept(Box3T<double>{} == Box3T<double>{}));
    // 这一条相对上一条是**零独立证据**：C++20 的 `!=` 是 `!(a == b)` 的重写，
    // noexcept 规格继承自被重写的 `operator==`，于是删掉 `operator==` 的 noexcept
    // 时上面一条与这一条**同时**失败。保留它只为把「生成的 `!=` 也被真的用过」
    // 写出来，不要把它计入覆盖率。（真要把 `!=` 的 noexcept 单独钉死，
    // 只能手写一个 `operator!=`，而全库约定是不手写。）
    STATIC_REQUIRE(noexcept(Box3T<double>{} != Box3T<double>{}));
}

TEST_CASE("every callable is usable in a constant expression",
          "[linear][box3]") {
    // 13 个可调用实体里，`empty` / `from_corners` / `merged` / `corner` / `is_empty`
    // 在别处已经被常量求值钉住；**其余八个在补这一格之前没有任何见证** ——
    // 逐个删掉 `constexpr` 后它们全部存活（实测 8/8 存活：`contains` 的两个重载、
    // `intersects`、`extent`、`half_extent`、`center`、`expanded`、`operator==`）。
    // 也就是说那些 `constexpr` 当时是无人见证的承诺。这一格把它们一次性放进
    // 常量表达式：删掉**任意一个**的 `constexpr`，这里就编译不过。
    //
    // 关于花括号里的逗号：`STATIC_REQUIRE` 在本版 Catch2 里是变参宏
    // （`static_assert( __VA_ARGS__, #__VA_ARGS__ )`），所以
    // `Point3T<double>{2.0, 0.0, 0.0}` 直接写在括号里**不会**被拆成多个实参 ——
    // 这一点是实测的（另建 TU 编过），不是推测。下面仍然先把比较值存进具名常量：
    // 对单参数宏那才是安全写法，也更好读。
    constexpr Box3T<double> box{Point3T<double>{0.0, 0.0, 0.0},
                                Point3T<double>{2.0, 4.0, 6.0}};
    constexpr Point3T<double> inner_min{};
    constexpr Point3T<double> inner_max{1.0, 1.0, 1.0};
    constexpr Box3T<double> inner{inner_min, inner_max};
    constexpr Box3T<double> canonical_empty = Box3T<double>::empty();
    constexpr Point3T<double> corner_one{2.0, 0.0, 0.0};

    STATIC_REQUIRE_FALSE(box.is_empty());
    STATIC_REQUIRE(box.contains(inner_min));
    STATIC_REQUIRE(box.contains(inner));
    STATIC_REQUIRE(box.intersects(box));
    STATIC_REQUIRE(box.extent().x == 2.0);
    STATIC_REQUIRE(box.half_extent().x == 1.0);
    STATIC_REQUIRE(box.center().x == 1.0);
    STATIC_REQUIRE(box.merged(box) == box);
    STATIC_REQUIRE(box.expanded(1.0).min.x == -1.0);
    STATIC_REQUIRE(box.corner(1) == corner_one);
    STATIC_REQUIRE(canonical_empty.is_empty());
    STATIC_REQUIRE(Box3T<double>::from_corners(inner_min, inner_max).max.y == 1.0);
    STATIC_REQUIRE(box == box);
}
// <<< sweep-add
