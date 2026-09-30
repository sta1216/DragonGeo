#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <limits>
#include <type_traits>

#include <DragonGeo/Linear/Interval.hpp>

using DragonGeo::Linear::Interval;
using DragonGeo::Linear::IntervalDifference;
using DragonGeo::Linear::IntervalDifferencef;
using DragonGeo::Linear::IntervalDifferenceT;
using DragonGeo::Linear::IntervalT;
using DragonGeo::Linear::Intervalf;

TEST_CASE("the float alias really is the float instantiation",
          "[linear][interval]") {
    STATIC_REQUIRE(std::is_same_v<Intervalf, IntervalT<float>>);
    STATIC_REQUIRE(std::is_same_v<IntervalDifferencef, IntervalDifferenceT<float>>);

    // 别名与成员别名都必须被**命名**过 —— 未被命名的绑定写错时连一次实例化
    // 都不会发生（Task 4 实测：把 float 别名绑成 double 实例化，全库 134 个
    // 用例全绿、退出码 0）。与 Task 4 同一条判据：**两个实例化都要钉**，
    // 只钉 float 时一个 `using ScalarType = float;` 的硬编码照样全绿（实测存活）。
    STATIC_REQUIRE(std::is_same_v<Interval, IntervalT<double>>);
    STATIC_REQUIRE(std::is_same_v<IntervalT<float>::ScalarType, float>);
    STATIC_REQUIRE(std::is_same_v<IntervalT<double>::ScalarType, double>);
}

TEST_CASE("merging with the empty interval is the identity",
          "[linear][interval]") {
    const Interval a{1.0, 5.0};
    CHECK(a.Merged(Interval::Empty()) == a);
    CHECK(Interval::Empty().Merged(a) == a);
    CHECK(Interval::Empty().Merged(Interval::Empty()) == Interval::Empty());

    // 「空的**非规范**表示」也是合法输入（`Interval{5.0, 1.0}` 就是个聚合初始化出来的
    // 倒置区间，`IsEmpty()` 明说对任何倒置区间都安全）。任一为空就要返回另一个，
    // 两个方向都必须如此 —— 只留自判空、不留对方判空的实现会在这一格返回
    // 「拿自己的端点去和空区间取外扩」的结果（实测 `{0,0}.Merged({5,1})` 曾给出 {0,1}）。
    const Interval invertedEmpty{5.0, 1.0};
    CHECK(Interval{0.0, 0.0}.Merged(invertedEmpty) == Interval{0.0, 0.0});
    CHECK(invertedEmpty.Merged(Interval{0.0, 0.0}) == Interval{0.0, 0.0});
}

TEST_CASE("a degenerate interval contains exactly one point",
          "[linear][interval]") {
    const Interval point{3.0, 3.0};

    CHECK_FALSE(point.IsEmpty());
    CHECK(point.Contains(3.0));
    CHECK_FALSE(point.Contains(3.0 + 1e-15));
    CHECK(point.Length() == 0.0);
}

TEST_CASE("contains is closed at both ends", "[linear][interval]") {
    const Interval a{1.0, 5.0};

    CHECK(a.Contains(1.0));
    CHECK(a.Contains(5.0));
    CHECK(a.Contains(3.0));
    CHECK_FALSE(a.Contains(0.999));
    CHECK_FALSE(a.Contains(5.001));

    // 空集不含任何点，无界区间含一切有限点 —— 两个哨兵在谓词下必须安全可用
    // （选这个空表示的理由正是让谓词无需特判）。
    CHECK_FALSE(Interval::Empty().Contains(3.0));
    CHECK(Interval::Unbounded().Contains(3.0));
}

TEST_CASE("length and center are defined on the sentinel intervals",
          "[linear][interval]") {
    // 空区间的测度是 0；max - min 会给出 -inf，那是没有意义的长度。
    CHECK(Interval::Empty().Length() == 0.0);

    // 无界区间的测度确实是 +inf。
    CHECK(Interval::Unbounded().Length() ==
          std::numeric_limits<double>::infinity());

    CHECK(Interval{1.0, 5.0}.Length() == 4.0);
    CHECK(Interval{1.0, 5.0}.Center() == 3.0);

    // center 用 min*0.5 + max*0.5：这两种极端值都能算对，
    // (min+max)*0.5 与 min+(max-min)*0.5 各会在其中一个上溢出。
    CHECK(Interval{1e308, 1e308}.Center() == 1e308);
    CHECK(Interval{-1e308, 1e308}.Center() == 0.0);

    // 端点无穷时没有「中点」：(+inf) + (-inf) 是 NaN。这是刻意的哨兵，
    // 不是漏判 —— 调用者应先 IsEmpty() 或判端点有限性。
    CHECK(std::isnan(Interval::Empty().Center()));
    CHECK(std::isnan(Interval::Unbounded().Center()));
}

TEST_CASE("the sentinels carry the canonical endpoints", "[linear][interval]") {
    // 规范表示是本任务的关键约定。只断言 `x == Interval::Empty()` 是**两边
    // 一起动**的：Empty() 自己被改成别的形式时，全部等价断言照样通过（实测：
    // 改成 {+inf, 0} 之后，计划里的断言只有 Center() 的 NaN 那一条抓得到）。
    // 所以端点值要逐条钉死，而不是经由另一个可能一起变的值来比较。
    CHECK(Interval::Empty().Min == std::numeric_limits<double>::infinity());
    CHECK(Interval::Empty().Max == -std::numeric_limits<double>::infinity());
    CHECK(Interval::Empty().IsEmpty());

    CHECK(Interval::Unbounded().Min == -std::numeric_limits<double>::infinity());
    CHECK(Interval::Unbounded().Max == std::numeric_limits<double>::infinity());
    CHECK_FALSE(Interval::Unbounded().IsEmpty());

    // 生产者必须给出**同一个**规范形式，而不是「倒置但不规范」的 {6, 5}：
    // 后者 `IsEmpty()` 同样为真，但 min/max 携带的是错误信息 —— 一个数学量
    // 两种表示，`==`/`Length()`/`Center()` 随之变成「看情况」。
    const Interval emptyIntersection = Interval{1.0, 5.0}.Intersection(Interval{6.0, 9.0});
    CHECK(emptyIntersection.Min == std::numeric_limits<double>::infinity());
    CHECK(emptyIntersection.Max == -std::numeric_limits<double>::infinity());

    const Interval shrunkEmpty = Interval{1.0, 5.0}.Expanded(-3.0);
    CHECK(shrunkEmpty.Min == std::numeric_limits<double>::infinity());
    CHECK(shrunkEmpty.Max == -std::numeric_limits<double>::infinity());
}

TEST_CASE("intersects and intersection agree, and intersection canonicalises",
          "[linear][interval]") {
    const Interval a{1.0, 5.0};
    const Interval b{4.0, 8.0};
    const Interval disjoint{6.0, 9.0};

    CHECK(a.Intersects(b));
    CHECK_FALSE(a.Intersects(disjoint));
    CHECK(a.Intersection(b) == Interval{4.0, 5.0});

    // 不相交时必须是规范空区间，不能是 [6.0, 5.0] 那种倒置形式 ——
    // 后者 IsEmpty() 也为真，但 min/max 携带的是错误信息。
    CHECK(a.Intersection(disjoint) == Interval::Empty());
    CHECK_FALSE(a.Intersection(disjoint).Intersects(a));

    // 闭区间：端点相接算相交 —— 改成半开比较（`<`）会把它判成不相交，而计划里
    // 的两条用例（{1,5} vs {4,8}、vs {6,9}）**在两种写法下结果完全相同**，
    // 抓不到这个差异。相接时 `Intersection` 的结果是退化的单点区间，不是空区间。
    CHECK(a.Intersects(Interval{5.0, 9.0}));
    CHECK(a.Intersection(Interval{5.0, 9.0}) == Interval{5.0, 5.0});
}

TEST_CASE("empty intervals stay empty under intersects and intersection",
          "[linear][interval]") {
    const Interval a{1.0, 5.0};

    CHECK_FALSE(Interval::Empty().Intersects(a));
    CHECK_FALSE(a.Intersects(Interval::Empty()));
    CHECK(Interval::Empty().Intersection(a) == Interval::Empty());
    CHECK(a.Intersection(Interval::Empty()) == Interval::Empty());
    CHECK(Interval::Empty().Intersection(Interval::Empty()) == Interval::Empty());
}

TEST_CASE("expanded grows both ends, and a negative amount shrinks",
          "[linear][interval]") {
    CHECK(Interval{1.0, 5.0}.Expanded(2.0) == Interval{-1.0, 7.0});
    CHECK(Interval{1.0, 5.0}.Expanded(-1.0) == Interval{2.0, 4.0});

    // 收缩过头得到空区间 —— 同样是规范形式，不是 [4.0, 2.0]。
    CHECK(Interval{1.0, 5.0}.Expanded(-3.0) == Interval::Empty());

    // 空区间膨胀后仍是空区间，不会变成 [-inf, +inf]。
    CHECK(Interval::Empty().Expanded(1.0) == Interval::Empty());

    // 无界区间膨胀后仍然无界。
    CHECK(Interval::Unbounded().Expanded(1.0) == Interval::Unbounded());

    // 非规范空也是空（`Interval{1.0, 0.0}` 一个聚合初始化就造得出来）。「空进空出」
    // 必须在**两种表示**上都成立 —— 旧实现只在规范空上兑现：它只靠末尾那次
    // 「结果为空则规范化」，而 `{1, 0}.Expanded(1)` 的结果是 `{0, 1}`，
    // **一个看起来完全正常的非空区间，从一个空输入产生**。
    //
    // 下面两条**各管一段，不是重复**：
    //   第一条只要求「还是空」—— 「开头先判输入」整块去掉时它就失败（结果非空）；
    //   第二条要求「是**规范**空」—— 把开头那句写成 `return *this;` 时，
    //   第一条**照样通过**（它确实是空的），只有这一条抓得到：非规范表示被原样返回。
    const Interval nonCanonicalEmptyForExpand{1.0, 0.0};
    CHECK(nonCanonicalEmptyForExpand.IsEmpty());
    CHECK(nonCanonicalEmptyForExpand.Expanded(1.0).IsEmpty());
    CHECK(nonCanonicalEmptyForExpand.Expanded(1.0) == Interval::Empty());
}

TEST_CASE("the empty predicate is total, including on non-finite input",
          "[linear][interval][degenerate]") {
    // `min > max` 对 NaN 返回 false，等于谎称这是一个正常的非空区间；
    // `!(min <= max)` 才是全函数。这一条是下面两个出口能自动规范化的前提。
    //
    // 用 `quiet_NaN()` 而不是 `0.0 / 0.0`：后者在常量表达式里被零除，
    // MSVC 直接报 **C2124，是编译错误**（标准层面的非良构，不是编译器脾气；
    // 已用四行独立 TU 复现）。库内 `core/Numeric.hpp` 本来就用 `quiet_NaN()`。
    const double nan = std::numeric_limits<double>::quiet_NaN();
    CHECK(Interval{nan, nan}.IsEmpty());

    // 空区间被 +inf 膨胀：`+inf - (+inf)` 是 NaN，若不把谓词改全，
    // 这里会返回 {NaN, NaN} 且 IsEmpty() 报 false —— 一个看似成功、
    // 实则含 NaN 的「区间」，正是本项目那条原则要禁止的东西。
    const double infinity = std::numeric_limits<double>::infinity();
    CHECK(Interval::Empty().Expanded(infinity) == Interval::Empty());
    CHECK(Interval::Empty().Expanded(-infinity) == Interval::Empty());

    // 有限区间的这两种极端膨胀都有正确的归宿。
    CHECK(Interval{1.0, 5.0}.Expanded(-infinity) == Interval::Empty());
    CHECK(Interval{1.0, 5.0}.Expanded(infinity) == Interval::Unbounded());

    // **这一条是「结果侧必须用全函数谓词」的活例，不能省。**
    // 中间结果 {NaN, NaN} 不是空的（`!(NaN <= NaN)` 虽为真，但那是全函数谓词
    // 才判得出 —— 用 `min > max` 会判成非空），所以必须靠结果侧那句全函数谓词
    // 才落得回 `Empty()`。缺了它，把结果侧写成 `min > max` 会全绿通过，
    // 而 `expanded` 会开始交出 `{NaN, NaN}` —— `IsEmpty()` 为真、`== Empty()`
    // 为假，正是本项目禁止的「第二个空表示」。
    CHECK(Interval{1.0, 5.0}.Expanded(nan) == Interval::Empty());

    // `{+inf, +inf}` **不是**空区间（它含 +inf 一个点），但它的长度是
    // `inf - inf` = NaN。这是「非空却没有有限长度」的哨兵值，与 center 同类，
    // 文档写明并在此钉住 —— 不要让一个看似成功的长度悄悄是 NaN。
    CHECK_FALSE(Interval{infinity, infinity}.IsEmpty());
    CHECK(std::isnan(Interval{infinity, infinity}.Length()));
}

TEST_CASE("the consumers agree with the total empty predicate",
          "[linear][interval][degenerate]") {
    // `IsEmpty()` 说 {NaN,NaN} 是空 —— 这是对的，它不含任何点。
    // 但三个消费者必须跟着这么认为，否则谓词只是装饰，而且同一对参数
    // 换方向会给出不同答案。
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const Interval nanInterval{nan, nan};
    const Interval normal{1.0, 5.0};

    REQUIRE(nanInterval.IsEmpty());

    // 相交：两个方向都必须为假（对称性）。
    CHECK_FALSE(normal.Intersects(nanInterval));
    CHECK_FALSE(nanInterval.Intersects(normal));

    // 被空区间裁剪，返回的是规范空区间，不是「全部」。
    CHECK(normal.Intersection(nanInterval) == Interval::Empty());
    CHECK(nanInterval.Intersection(normal) == Interval::Empty());

    // 合并：换方向答案相同（交换律）。
    CHECK(nanInterval.Merged(normal) == normal);
    CHECK(normal.Merged(nanInterval) == normal);

    // **两个「空」的表示不同时，规范化优先于恒等律。**
    // `{NaN,NaN}` 与 `{1,0}` 都是空集，但表示不同。若第一支写成
    // `if (IsEmpty()) return other;`，两侧会各自返回「另一个」——
    // 结果都是空集，`==` 却因为比表示而判不等，于是 `merged` 不对称
    // （实测 4140/14641 组输入）。correct 的第一支是
    // `return other.IsEmpty() ? Empty() : other;`。
    const Interval nonCanonicalEmpty{1.0, 0.0};
    CHECK(nonCanonicalEmpty.IsEmpty());
    CHECK(nanInterval.Merged(nonCanonicalEmpty) == Interval::Empty());
    CHECK(nonCanonicalEmpty.Merged(nanInterval) == Interval::Empty());

    // `Length()` 的 `IsEmpty()` 守卫也要走全函数谓词 —— 否则退化成
    // `min > max` 就会算出 NaN，而这一格没有任何别的断言看得见。
    CHECK(nanInterval.Length() == 0.0);

    // 聚合性是头文件明写的承诺（「不声明任何构造函数，以保持聚合性」），
    // 加一个构造函数就会悄悄丢掉它。
    STATIC_REQUIRE(std::is_aggregate_v<IntervalT<double>>);
}


TEST_CASE("equality compares both endpoints, and != agrees",
          "[linear][interval]") {
    // 计划里全部比较断言都是 `==`，而每个期望值的**两端都同时正确** ——
    // 实测：一个只比较 min 的 operator== 让计划 Step 1 的全部 36 条断言
    // 全绿、退出码 0（同一套 shadow，只换测试文件即复现）；只比较 max 的
    // 同样全绿。两个方向各要一条独立证据，不能靠一条相等断言顺带覆盖
    // （断言会传染：被依赖方失效时，依赖它的那条一起哑掉）。
    // 这两条同时命名了 C++20 自动生成的 operator!=（本库不手写它）。
    CHECK(Interval{1.0, 5.0} != Interval{1.0, 6.0});   // 只有 max 不同
    CHECK(Interval{1.0, 5.0} != Interval{2.0, 5.0});   // 只有 min 不同
    CHECK(Interval{1.0, 5.0} == Interval{1.0, 5.0});
}

TEST_CASE("the implicitly generated special members carry both endpoints",
          "[linear][interval]") {
    // 默认成员初始化值 `Scalar min{}` —— **用常量求值钉，不要用类型特征**
    // （Task 4 实测那条 `!is_trivially_default_constructible_v` 是恒真的）。
    // 常量表达式里读不确定值是**编译错误**（C2737 / C2131），既零 UB、
    // 又能检出**任意一个**端点的 NSDMI 缺失。`defaultInterval` 不能写花括号
    // —— 写了就是聚合的值初始化，有没有 NSDMI 都会清零。
    constexpr IntervalT<double> defaultInterval;
    STATIC_REQUIRE(defaultInterval.Min == 0.0);
    STATIC_REQUIRE(defaultInterval.Max == 0.0);

    // 两个工厂是 Interfaces 明文的 `static constexpr`：在常量表达式里求值
    // 是最直接的证据，去掉 constexpr 就编不过。
    constexpr IntervalT<double> mergedSentinel =
        IntervalT<double>::Empty().Merged(IntervalT<double>{1.0, 5.0});
    STATIC_REQUIRE(mergedSentinel.Min == 1.0);
    STATIC_REQUIRE(mergedSentinel.Max == 5.0);
    // 修复轮 2：`merged` 加了空判定守卫之后，上面两条**不再经过取两端外扩的算术
    // 路径**（空区间直接返回 other），`merged` 的 min/max 取错由「编译失败」变成了
    // **存活**（实测 89/89 全绿）。非空的合并路径因此需要自己的证据：期望值的两端
    // 都取自 other（{0,9}），任何一端取错或整体返回自身都会失败。
    constexpr IntervalT<double> mergedFinite =
        IntervalT<double>{1.0, 5.0}.Merged(IntervalT<double>{0.0, 9.0});
    STATIC_REQUIRE(mergedFinite.Min == 0.0);
    STATIC_REQUIRE(mergedFinite.Max == 9.0);

    // 隐式拷贝赋值也必须被真的用一次，否则它从未被实例化 —— 一个只赋 min 的
    // 手写 operator= 会完全静默（Task 4 实测过同族形态：从仓库测试编出的 obj
    // 里连拷贝赋值的符号都不存在）。
    const Interval source{4.0, 5.0};
    Interval target{0.0, 0.0};
    target = source;

    CHECK(target.Min == 4.0);
    CHECK(target.Max == 5.0);
}

TEST_CASE("difference returns the closure of the set difference",
          "[linear][interval]") {
    const Interval whole{1.0, 10.0};

    // 中间挖开：闭包把切点留在两侧。
    const IntervalDifference hole = whole.Difference(Interval{3.0, 5.0});
    CHECK(hole.Below == Interval{1.0, 3.0});
    CHECK(hole.Above == Interval{5.0, 10.0});
    CHECK(hole.Below.Intersects(Interval{3.0, 5.0}));

    // 只裁掉左端 / 右端：剩下的一段落在对应的槽里。
    const IntervalDifference cutLeft = whole.Difference(Interval{0.0, 4.0});
    CHECK(cutLeft.Below == Interval::Empty());
    CHECK(cutLeft.Above == Interval{4.0, 10.0});

    const IntervalDifference cutRight = whole.Difference(Interval{7.0, 12.0});
    CHECK(cutRight.Below == Interval{1.0, 7.0});
    CHECK(cutRight.Above == Interval::Empty());

    // 完全盖住，以及与自身相减，差集为空。
    const IntervalDifference covered = whole.Difference(Interval{0.0, 10.0});
    CHECK(covered.Below == Interval::Empty());
    CHECK(covered.Above == Interval::Empty());
    CHECK(whole.Difference(whole) == covered);

    // 不相交，或只重叠一个点：闭包仍是整段，不拆开。
    CHECK(whole.Difference(Interval{11.0, 12.0}).Below == whole);
    CHECK(whole.Difference(Interval{11.0, 12.0}).Above == Interval::Empty());
    CHECK(whole.Difference(Interval{-2.0, -1.0}).Below == whole);
    CHECK(whole.Difference(Interval{4.0, 4.0}).Below == whole);
    CHECK(whole.Difference(Interval{4.0, 4.0}).Above == Interval::Empty());

    // 空操作数：自身为空则两段都空；对方为空则差集是自身。非规范空先收成规范空。
    const Interval inverted{2.0, 0.0};
    CHECK(Interval::Empty().Difference(whole).Below == Interval::Empty());
    CHECK(Interval::Empty().Difference(whole).Above == Interval::Empty());
    CHECK(inverted.Difference(whole).Below == Interval::Empty());
    CHECK(whole.Difference(Interval::Empty()).Below == whole);
    CHECK(whole.Difference(inverted).Below == whole);
    CHECK(whole.Difference(inverted).Above == Interval::Empty());

    const double nan = std::numeric_limits<double>::quiet_NaN();
    CHECK(whole.Difference(Interval{nan, nan}).Below == whole);

    constexpr IntervalDifference finite =
        IntervalT<double>{1.0, 10.0}.Difference(IntervalT<double>{3.0, 5.0});
    STATIC_REQUIRE(finite.Below.Min == 1.0);
    STATIC_REQUIRE(finite.Below.Max == 3.0);
    STATIC_REQUIRE(finite.Above.Min == 5.0);
    STATIC_REQUIRE(finite.Above.Max == 10.0);
}

TEST_CASE("every declared callable is noexcept", "[linear][interval]") {
    // Interfaces 对本类型的两个工厂明文写了 noexcept，成员一律 noexcept 是全库
    // 惯例。Task 4 把「其余 noexcept 无证据」留成了显式取舍；这里一次钉全 ——
    // 各条之间无依赖，去掉**任意一处** noexcept 都会单独失败。
    STATIC_REQUIRE(noexcept(IntervalT<double>::Empty()));
    STATIC_REQUIRE(noexcept(IntervalT<double>::Unbounded()));
    STATIC_REQUIRE(noexcept(IntervalT<double>{}.IsEmpty()));
    STATIC_REQUIRE(noexcept(IntervalT<double>{}.Contains(0.0)));
    STATIC_REQUIRE(noexcept(IntervalT<double>{}.Intersects(IntervalT<double>{})));
    STATIC_REQUIRE(noexcept(IntervalT<double>{}.Length()));
    STATIC_REQUIRE(noexcept(IntervalT<double>{}.Center()));
    STATIC_REQUIRE(noexcept(IntervalT<double>{}.Merged(IntervalT<double>{})));
    STATIC_REQUIRE(noexcept(IntervalT<double>{}.Expanded(0.0)));
    STATIC_REQUIRE(noexcept(IntervalT<double>{}.Intersection(IntervalT<double>{})));
    STATIC_REQUIRE(noexcept(IntervalT<double>{}.Difference(IntervalT<double>{})));
    STATIC_REQUIRE(noexcept(IntervalDifferenceT<double>{} == IntervalDifferenceT<double>{}));
    STATIC_REQUIRE(noexcept(IntervalT<double>{} == IntervalT<double>{}));
}
