#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <limits>
#include <type_traits>

#include <GeoCore/linear/Interval.hpp>

using GeoCore::linear::Interval;
using GeoCore::linear::IntervalT;
using GeoCore::linear::Intervalf;

TEST_CASE("the float alias really is the float instantiation",
          "[linear][interval]") {
    STATIC_REQUIRE(std::is_same_v<Intervalf, IntervalT<float>>);

    // 别名与成员别名都必须被**命名**过 —— 未被命名的绑定写错时连一次实例化
    // 都不会发生（Task 4 实测：把 float 别名绑成 double 实例化，全库 134 个
    // 用例全绿、退出码 0）。与 Task 4 同一条判据：**两个实例化都要钉**，
    // 只钉 float 时一个 `using scalar_type = float;` 的硬编码照样全绿（实测存活）。
    STATIC_REQUIRE(std::is_same_v<Interval, IntervalT<double>>);
    STATIC_REQUIRE(std::is_same_v<IntervalT<float>::scalar_type, float>);
    STATIC_REQUIRE(std::is_same_v<IntervalT<double>::scalar_type, double>);
}

TEST_CASE("merging with the empty interval is the identity",
          "[linear][interval]") {
    const Interval a{1.0, 5.0};
    CHECK(a.merged(Interval::empty()) == a);
    CHECK(Interval::empty().merged(a) == a);
    CHECK(Interval::empty().merged(Interval::empty()) == Interval::empty());

    // 「空的**非规范**表示」也是合法输入（`Interval{5.0, 1.0}` 就是个聚合初始化出来的
    // 倒置区间，`is_empty()` 明说对任何倒置区间都安全）。任一为空就要返回另一个，
    // 两个方向都必须如此 —— 只留自判空、不留对方判空的实现会在这一格返回
    // 「拿自己的端点去和空区间取外扩」的结果（实测 `{0,0}.merged({5,1})` 曾给出 {0,1}）。
    const Interval inverted_empty{5.0, 1.0};
    CHECK(Interval{0.0, 0.0}.merged(inverted_empty) == Interval{0.0, 0.0});
    CHECK(inverted_empty.merged(Interval{0.0, 0.0}) == Interval{0.0, 0.0});
}

TEST_CASE("a degenerate interval contains exactly one point",
          "[linear][interval]") {
    const Interval point{3.0, 3.0};

    CHECK_FALSE(point.is_empty());
    CHECK(point.contains(3.0));
    CHECK_FALSE(point.contains(3.0 + 1e-15));
    CHECK(point.length() == 0.0);
}

TEST_CASE("contains is closed at both ends", "[linear][interval]") {
    const Interval a{1.0, 5.0};

    CHECK(a.contains(1.0));
    CHECK(a.contains(5.0));
    CHECK(a.contains(3.0));
    CHECK_FALSE(a.contains(0.999));
    CHECK_FALSE(a.contains(5.001));

    // 空集不含任何点，无界区间含一切有限点 —— 两个哨兵在谓词下必须安全可用
    // （选这个空表示的理由正是让谓词无需特判）。
    CHECK_FALSE(Interval::empty().contains(3.0));
    CHECK(Interval::unbounded().contains(3.0));
}

TEST_CASE("length and center are defined on the sentinel intervals",
          "[linear][interval]") {
    // 空区间的测度是 0；max - min 会给出 -inf，那是没有意义的长度。
    CHECK(Interval::empty().length() == 0.0);

    // 无界区间的测度确实是 +inf。
    CHECK(Interval::unbounded().length() ==
          std::numeric_limits<double>::infinity());

    CHECK(Interval{1.0, 5.0}.length() == 4.0);
    CHECK(Interval{1.0, 5.0}.center() == 3.0);

    // center 用 min*0.5 + max*0.5：这两种极端值都能算对，
    // (min+max)*0.5 与 min+(max-min)*0.5 各会在其中一个上溢出。
    CHECK(Interval{1e308, 1e308}.center() == 1e308);
    CHECK(Interval{-1e308, 1e308}.center() == 0.0);

    // 端点无穷时没有「中点」：(+inf) + (-inf) 是 NaN。这是刻意的哨兵，
    // 不是漏判 —— 调用者应先 is_empty() 或判端点有限性。
    CHECK(std::isnan(Interval::empty().center()));
    CHECK(std::isnan(Interval::unbounded().center()));
}

TEST_CASE("the sentinels carry the canonical endpoints", "[linear][interval]") {
    // 规范表示是本任务的关键约定。只断言 `x == Interval::empty()` 是**两边
    // 一起动**的：empty() 自己被改成别的形式时，全部等价断言照样通过（实测：
    // 改成 {+inf, 0} 之后，计划里的断言只有 center() 的 NaN 那一条抓得到）。
    // 所以端点值要逐条钉死，而不是经由另一个可能一起变的值来比较。
    CHECK(Interval::empty().min == std::numeric_limits<double>::infinity());
    CHECK(Interval::empty().max == -std::numeric_limits<double>::infinity());
    CHECK(Interval::empty().is_empty());

    CHECK(Interval::unbounded().min == -std::numeric_limits<double>::infinity());
    CHECK(Interval::unbounded().max == std::numeric_limits<double>::infinity());
    CHECK_FALSE(Interval::unbounded().is_empty());

    // 生产者必须给出**同一个**规范形式，而不是「倒置但不规范」的 {6, 5}：
    // 后者 `is_empty()` 同样为真，但 min/max 携带的是错误信息 —— 一个数学量
    // 两种表示，`==`/`length()`/`center()` 随之变成「看情况」。
    const Interval clipped_empty = Interval{1.0, 5.0}.clipped(Interval{6.0, 9.0});
    CHECK(clipped_empty.min == std::numeric_limits<double>::infinity());
    CHECK(clipped_empty.max == -std::numeric_limits<double>::infinity());

    const Interval shrunk_empty = Interval{1.0, 5.0}.expanded(-3.0);
    CHECK(shrunk_empty.min == std::numeric_limits<double>::infinity());
    CHECK(shrunk_empty.max == -std::numeric_limits<double>::infinity());
}

TEST_CASE("intersects and clipped agree, and clipped canonicalises",
          "[linear][interval]") {
    const Interval a{1.0, 5.0};
    const Interval b{4.0, 8.0};
    const Interval disjoint{6.0, 9.0};

    CHECK(a.intersects(b));
    CHECK_FALSE(a.intersects(disjoint));
    CHECK(a.clipped(b) == Interval{4.0, 5.0});

    // 不相交时必须是规范空区间，不能是 [6.0, 5.0] 那种倒置形式 ——
    // 后者 is_empty() 也为真，但 min/max 携带的是错误信息。
    CHECK(a.clipped(disjoint) == Interval::empty());
    CHECK_FALSE(a.clipped(disjoint).intersects(a));

    // 闭区间：端点相接算相交 —— 改成半开比较（`<`）会把它判成不相交，而计划里
    // 的两条用例（{1,5} vs {4,8}、vs {6,9}）**在两种写法下结果完全相同**，
    // 抓不到这个差异。相接时 `clipped` 的结果是退化的单点区间，不是空区间。
    CHECK(a.intersects(Interval{5.0, 9.0}));
    CHECK(a.clipped(Interval{5.0, 9.0}) == Interval{5.0, 5.0});
}

TEST_CASE("empty intervals stay empty under intersects and clipped",
          "[linear][interval]") {
    const Interval a{1.0, 5.0};

    CHECK_FALSE(Interval::empty().intersects(a));
    CHECK_FALSE(a.intersects(Interval::empty()));
    CHECK(Interval::empty().clipped(a) == Interval::empty());
    CHECK(a.clipped(Interval::empty()) == Interval::empty());
    CHECK(Interval::empty().clipped(Interval::empty()) == Interval::empty());
}

TEST_CASE("expanded grows both ends, and a negative amount shrinks",
          "[linear][interval]") {
    CHECK(Interval{1.0, 5.0}.expanded(2.0) == Interval{-1.0, 7.0});
    CHECK(Interval{1.0, 5.0}.expanded(-1.0) == Interval{2.0, 4.0});

    // 收缩过头得到空区间 —— 同样是规范形式，不是 [4.0, 2.0]。
    CHECK(Interval{1.0, 5.0}.expanded(-3.0) == Interval::empty());

    // 空区间膨胀后仍是空区间，不会变成 [-inf, +inf]。
    CHECK(Interval::empty().expanded(1.0) == Interval::empty());

    // 无界区间膨胀后仍然无界。
    CHECK(Interval::unbounded().expanded(1.0) == Interval::unbounded());

    // 非规范空也是空（`Interval{1.0, 0.0}` 一个聚合初始化就造得出来）。「空进空出」
    // 必须在**两种表示**上都成立 —— 旧实现只在规范空上兑现：它只靠末尾那次
    // 「结果为空则规范化」，而 `{1, 0}.expanded(1)` 的结果是 `{0, 1}`，
    // **一个看起来完全正常的非空区间，从一个空输入产生**。
    //
    // 下面两条**各管一段，不是重复**：
    //   第一条只要求「还是空」—— 「开头先判输入」整块去掉时它就失败（结果非空）；
    //   第二条要求「是**规范**空」—— 把开头那句写成 `return *this;` 时，
    //   第一条**照样通过**（它确实是空的），只有这一条抓得到：非规范表示被原样返回。
    const Interval non_canonical_empty_for_expand{1.0, 0.0};
    CHECK(non_canonical_empty_for_expand.is_empty());
    CHECK(non_canonical_empty_for_expand.expanded(1.0).is_empty());
    CHECK(non_canonical_empty_for_expand.expanded(1.0) == Interval::empty());
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
    CHECK(Interval{nan, nan}.is_empty());

    // 空区间被 +inf 膨胀：`+inf - (+inf)` 是 NaN，若不把谓词改全，
    // 这里会返回 {NaN, NaN} 且 is_empty() 报 false —— 一个看似成功、
    // 实则含 NaN 的「区间」，正是本项目那条原则要禁止的东西。
    const double infinity = std::numeric_limits<double>::infinity();
    CHECK(Interval::empty().expanded(infinity) == Interval::empty());
    CHECK(Interval::empty().expanded(-infinity) == Interval::empty());

    // 有限区间的这两种极端膨胀都有正确的归宿。
    CHECK(Interval{1.0, 5.0}.expanded(-infinity) == Interval::empty());
    CHECK(Interval{1.0, 5.0}.expanded(infinity) == Interval::unbounded());

    // **这一条是「结果侧必须用全函数谓词」的活例，不能省。**
    // 中间结果 {NaN, NaN} 不是空的（`!(NaN <= NaN)` 虽为真，但那是全函数谓词
    // 才判得出 —— 用 `min > max` 会判成非空），所以必须靠结果侧那句全函数谓词
    // 才落得回 `empty()`。缺了它，把结果侧写成 `min > max` 会全绿通过，
    // 而 `expanded` 会开始交出 `{NaN, NaN}` —— `is_empty()` 为真、`== empty()`
    // 为假，正是本项目禁止的「第二个空表示」。
    CHECK(Interval{1.0, 5.0}.expanded(nan) == Interval::empty());

    // `{+inf, +inf}` **不是**空区间（它含 +inf 一个点），但它的长度是
    // `inf - inf` = NaN。这是「非空却没有有限长度」的哨兵值，与 center 同类，
    // 文档写明并在此钉住 —— 不要让一个看似成功的长度悄悄是 NaN。
    CHECK_FALSE(Interval{infinity, infinity}.is_empty());
    CHECK(std::isnan(Interval{infinity, infinity}.length()));
}

TEST_CASE("the consumers agree with the total empty predicate",
          "[linear][interval][degenerate]") {
    // `is_empty()` 说 {NaN,NaN} 是空 —— 这是对的，它不含任何点。
    // 但三个消费者必须跟着这么认为，否则谓词只是装饰，而且同一对参数
    // 换方向会给出不同答案。
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const Interval nan_interval{nan, nan};
    const Interval normal{1.0, 5.0};

    REQUIRE(nan_interval.is_empty());

    // 相交：两个方向都必须为假（对称性）。
    CHECK_FALSE(normal.intersects(nan_interval));
    CHECK_FALSE(nan_interval.intersects(normal));

    // 被空区间裁剪，返回的是规范空区间，不是「全部」。
    CHECK(normal.clipped(nan_interval) == Interval::empty());
    CHECK(nan_interval.clipped(normal) == Interval::empty());

    // 合并：换方向答案相同（交换律）。
    CHECK(nan_interval.merged(normal) == normal);
    CHECK(normal.merged(nan_interval) == normal);

    // **两个「空」的表示不同时，规范化优先于恒等律。**
    // `{NaN,NaN}` 与 `{1,0}` 都是空集，但表示不同。若第一支写成
    // `if (is_empty()) return other;`，两侧会各自返回「另一个」——
    // 结果都是空集，`==` 却因为比表示而判不等，于是 `merged` 不对称
    // （实测 4140/14641 组输入）。correct 的第一支是
    // `return other.is_empty() ? empty() : other;`。
    const Interval non_canonical_empty{1.0, 0.0};
    CHECK(non_canonical_empty.is_empty());
    CHECK(nan_interval.merged(non_canonical_empty) == Interval::empty());
    CHECK(non_canonical_empty.merged(nan_interval) == Interval::empty());

    // `length()` 的 `is_empty()` 守卫也要走全函数谓词 —— 否则退化成
    // `min > max` 就会算出 NaN，而这一格没有任何别的断言看得见。
    CHECK(nan_interval.length() == 0.0);

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
    // 又能检出**任意一个**端点的 NSDMI 缺失。`default_interval` 不能写花括号
    // —— 写了就是聚合的值初始化，有没有 NSDMI 都会清零。
    constexpr IntervalT<double> default_interval;
    STATIC_REQUIRE(default_interval.min == 0.0);
    STATIC_REQUIRE(default_interval.max == 0.0);

    // 两个工厂是 Interfaces 明文的 `static constexpr`：在常量表达式里求值
    // 是最直接的证据，去掉 constexpr 就编不过。
    constexpr IntervalT<double> merged_sentinel =
        IntervalT<double>::empty().merged(IntervalT<double>{1.0, 5.0});
    STATIC_REQUIRE(merged_sentinel.min == 1.0);
    STATIC_REQUIRE(merged_sentinel.max == 5.0);
    // 修复轮 2：`merged` 加了空判定守卫之后，上面两条**不再经过取两端外扩的算术
    // 路径**（空区间直接返回 other），`merged` 的 min/max 取错由「编译失败」变成了
    // **存活**（实测 89/89 全绿）。非空的合并路径因此需要自己的证据：期望值的两端
    // 都取自 other（{0,9}），任何一端取错或整体返回自身都会失败。
    constexpr IntervalT<double> merged_finite =
        IntervalT<double>{1.0, 5.0}.merged(IntervalT<double>{0.0, 9.0});
    STATIC_REQUIRE(merged_finite.min == 0.0);
    STATIC_REQUIRE(merged_finite.max == 9.0);

    // 隐式拷贝赋值也必须被真的用一次，否则它从未被实例化 —— 一个只赋 min 的
    // 手写 operator= 会完全静默（Task 4 实测过同族形态：从仓库测试编出的 obj
    // 里连拷贝赋值的符号都不存在）。
    const Interval source{4.0, 5.0};
    Interval target{0.0, 0.0};
    target = source;

    CHECK(target.min == 4.0);
    CHECK(target.max == 5.0);
}

TEST_CASE("every declared callable is noexcept", "[linear][interval]") {
    // Interfaces 对本类型的两个工厂明文写了 noexcept，成员一律 noexcept 是全库
    // 惯例。Task 4 把「其余 noexcept 无证据」留成了显式取舍；这里一次钉全 ——
    // 各条之间无依赖，去掉**任意一处** noexcept 都会单独失败。
    STATIC_REQUIRE(noexcept(IntervalT<double>::empty()));
    STATIC_REQUIRE(noexcept(IntervalT<double>::unbounded()));
    STATIC_REQUIRE(noexcept(IntervalT<double>{}.is_empty()));
    STATIC_REQUIRE(noexcept(IntervalT<double>{}.contains(0.0)));
    STATIC_REQUIRE(noexcept(IntervalT<double>{}.intersects(IntervalT<double>{})));
    STATIC_REQUIRE(noexcept(IntervalT<double>{}.length()));
    STATIC_REQUIRE(noexcept(IntervalT<double>{}.center()));
    STATIC_REQUIRE(noexcept(IntervalT<double>{}.merged(IntervalT<double>{})));
    STATIC_REQUIRE(noexcept(IntervalT<double>{}.expanded(0.0)));
    STATIC_REQUIRE(noexcept(IntervalT<double>{}.clipped(IntervalT<double>{})));
    STATIC_REQUIRE(noexcept(IntervalT<double>{} == IntervalT<double>{}));
}
