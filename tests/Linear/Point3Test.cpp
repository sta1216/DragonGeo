#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cmath>
#include <limits>
#include <type_traits>

#include <DragonGeo/Linear/Point3.hpp>

using Catch::Approx;
using DragonGeo::Linear::Point3;
using DragonGeo::Linear::Point3T;
using DragonGeo::Linear::Point3f;
using DragonGeo::Linear::Vector3;

namespace {
/// 「能否相加」这个判断必须包在概念里，见下面 static_assert 处的说明。
template <typename T> concept addable = requires(T p, T q) { p + q; };
}

TEST_CASE("the float alias really is the float instantiation", "[linear][point3]") {
    // 别名若从未被任何测试命名过，绑定写错时成员连一次实例化都不会发生 —— 实测：把 Point3f 绑成 Point3T<double>，全库 134 个用例全绿、退出码 0。 float 用户会静默拿到 double 存储，精度与内存占用都不是承诺的样子。
    STATIC_REQUIRE(std::is_same_v<Point3f, Point3T<float>>);
}

TEST_CASE("Point3 supports subscript and array export", "[linear][point3]") {
    const Point3 p{1.0, 2.0, 3.0};

    CHECK(p[0] == 1.0);
    CHECK(p[1] == 2.0);
    CHECK(p[2] == 3.0);

    // 三个槽位**逐个**查。只查末槽是不够的：`{y, x, z}`、`{0, 0, z}` 这类错位实现都能通过（已用变异测试实测存活）。ToArray() 是公开的互操作接口（喂给外部库 / GPU buffer），槽位错位就是静默的坐标错乱。
    // 同目录的 vector3Test.cpp 正是三个下标全查的，这里与它对齐。
    const std::array<double, 3> arr = p.ToArray();
    CHECK(arr[0] == 1.0);
    CHECK(arr[1] == 2.0);
    CHECK(arr[2] == 3.0);
}

TEST_CASE("the mutable subscript writes the component it names", "[linear][point3]") {
    // **非 const 重载必须被真的用一次。** 本任务此前所有下标访问都作用在 const 对象上，因此 `Scalar& operator[](int)` 从未被实例化 —— 实测把它写成分量对调后，全库 136 个用例全绿、零警告，甚至从测试编出的 obj 里
    // 连这个重载的符号都不存在。后果是经由可变下标写入会静默落进别的分量，那是几何数据损坏，不是精度问题。同目录的 vector3Test.cpp 正是这样用 Vector 的可变下标的，与它对齐。
    Point3 p{0.0, 0.0, 0.0};
    p[0] = 1.0;
    p[1] = 2.0;
    p[2] = 3.0;

    CHECK(p.X == 1.0);
    CHECK(p.Y == 2.0);
    CHECK(p.Z == 3.0);
}

TEST_CASE("Point3 keeps its declaration-level guarantees", "[linear][point3]") {
    // 分量别名 —— **两个实例化都要钉**。只钉 float 时，一个把 `using ScalarType = float;` 硬编码的实现照样全绿（实测存活）；全库其余类型都是 `Scalar`，硬编码是实打实的缺陷。
    STATIC_REQUIRE(std::is_same_v<Point3T<float>::ScalarType, float>);
    STATIC_REQUIRE(std::is_same_v<Point3T<double>::ScalarType, double>);

    // noexcept 是 Interfaces 的明文承诺，也要有证据。
    STATIC_REQUIRE(noexcept(Point3{}.DistanceTo(Point3{})));

    // 同 point2Test.cpp：其余 8 处 noexcept 也要有证据，逐条都有死亡证明。
    const Point3 sample{1.0, 2.0, 3.0};
    STATIC_REQUIRE(noexcept(Point3{}.operator[](0)));   // 非 const 重载
    STATIC_REQUIRE(noexcept(sample[0]));                // const 重载
    STATIC_REQUIRE(noexcept(Point3{}.ToArray()));
    STATIC_REQUIRE(noexcept(sample + Vector3{1.0, 2.0, 3.0}));
    STATIC_REQUIRE(noexcept(Vector3{1.0, 2.0, 3.0} + sample));
    STATIC_REQUIRE(noexcept(sample - Vector3{1.0, 2.0, 3.0}));
    STATIC_REQUIRE(noexcept(sample - sample));
    STATIC_REQUIRE(noexcept(sample == sample));

    // 默认成员初始化值 `Scalar x{}` —— **用常量求值钉，不要用类型特征**。 `!is_trivially_default_constructible_v` 看着聪明，实际是**恒真**的：去掉一个分量的 NSDMI 之后，另外两个还在，构造函数依然非平凡，断言照样
    // 通过（已实测：去掉全部三个才变）。那正是它要防的变异体。
    //
    // 常量表达式里读一个不确定值**不是 UB，是编译错误**（C2131 / C2737），所以下面这段既零 UB、又能检出**任意一个**分量的 NSDMI 缺失。
    //
    // 注意 `defaultPoint` **不能写花括号**：写了就是聚合的值初始化，有没有 NSDMI 都会清零，什么也测不出来。
    constexpr Point3T<double> defaultPoint;
    STATIC_REQUIRE(defaultPoint.X == 0.0);
    STATIC_REQUIRE(defaultPoint.Y == 0.0);
    STATIC_REQUIRE(defaultPoint.Z == 0.0);
}

TEST_CASE("copy assignment carries every component", "[linear][point3]") {
    // 隐式拷贝赋值也必须被真的用一次。否则它从未被实例化，一个只赋前两个分量的手写赋值运算符会完全静默 —— 已实测：从仓库测试编出的 obj 里连拷贝赋值的符号（`??4`）都不存在。
    const Point3 source{4.0, 5.0, 6.0};
    Point3 target{0.0, 0.0, 0.0};
    target = source;

    CHECK(target.X == 4.0);
    CHECK(target.Y == 5.0);
    CHECK(target.Z == 6.0);
}

TEST_CASE("point and vector arithmetic follows affine rules", "[linear][point3]") {
    const Point3 a{1.0, 2.0, 3.0};
    const Point3 b{4.0, 6.0, 8.0};
    const Vector3 v{10.0, 20.0, 30.0};

    // 点 + 向量 = 点
    STATIC_REQUIRE(std::is_same_v<decltype(a + v), Point3>);
    CHECK(a + v == Point3{11.0, 22.0, 33.0});

    // 向量 + 点 = 点 —— 上一条的反向书写。平移不关心两个操作数的书写顺序，两种写法必须同义；调用方先写出向量时不该撞上「找不到运算符」。（此运算符用户裁定补上，详见计划的「遗留决策」第 2 条。）
    STATIC_REQUIRE(std::is_same_v<decltype(v + a), Point3>);
    CHECK(v + a == Point3{11.0, 22.0, 33.0});
    CHECK(v + a == a + v);

    // 点 - 向量 = 点
    STATIC_REQUIRE(std::is_same_v<decltype(a - v), Point3>);
    CHECK(a - v == Point3{-9.0, -18.0, -27.0});

    // 点 - 点 = 向量
    STATIC_REQUIRE(std::is_same_v<decltype(b - a), Vector3>);
    CHECK(b - a == Vector3{3.0, 4.0, 5.0});

    // 两个点只要有一个分量不同就必须不相等 —— **三个方向各要一条**。上面所有期望值的 x 分量都恰好与左操作数相同（11、-9 都是从 a.X=1 算出来的），于是一个「只比较 x」的 operator== 会让本文件全部断言静默通过
    // （已用变异测试实测：把 == 改成只比 x，21 条断言全过）。
    //
    // 而只补两个方向同样不够：三条断言若写成 {y 不同, z 不同, 相等}，一个 **忽略 x** 的实现照样全绿（实测存活）。更糟的是它会**连带**抹掉 `operator+` 的 x 槽证据 —— 下面 `a + v == Point3{11,22,33}` 的 x 方向
    // 正是经由 `==` 传递的，于是「operator+ 把 x 写成 0」也一起存活。一条断言被架空，另一条断言就跟着失效。
    CHECK(Point3{1.0, 2.0, 3.0} != Point3{1.0, 9.0, 3.0});   // y 不同
    CHECK(Point3{1.0, 2.0, 3.0} != Point3{1.0, 2.0, 9.0});   // z 不同
    CHECK(Point3{1.0, 2.0, 3.0} != Point3{9.0, 2.0, 3.0});   // x 不同
    CHECK(Point3{1.0, 2.0, 3.0} == Point3{1.0, 2.0, 3.0});

    // 点 + 点不存在 —— 这是本任务的核心不变量（Review Focus 第 1 条）：写错了不会报错，只会静默给出错误语义。必须真的断言，不能只注释掉。
    //
    // 不能直接写 `static_assert(!requires(Point3 p, Point3 q) { p + q; });` —— 那不是「求值为假」，而是**非良构的 C++**。requirements 里的非法表达式
    // 只有在「替换模板实参」的语境下才求值为 false；`Point3` 是具体类型， `p + q` 不依赖任何模板参数，因此是硬错误（MSVC 报 C2676，编译器不会给你 false）。必须包一层概念，让失败发生在替换上下文里 —— 见文件顶部
    // 匿名命名空间的 `addable`。这一点与 Catch2 无关，也与用不用 STATIC_REQUIRE 无关。
    static_assert(!addable<Point3>, "Point + Point must not be well-formed");
}

TEST_CASE("Point3 DistanceTo", "[linear][point3]") {
    // 返回类型也要钉住。收窄成 float 之后 42 条断言全绿、exit 0 （/W4 下会出 C4244，但测试套件检不出 —— 别指望编译器的警告代替断言）。
    STATIC_REQUIRE(std::is_same_v<decltype(Point3{}.DistanceTo(Point3{})), double>);

    CHECK(Point3{0.0, 0.0, 0.0}.DistanceTo(Point3{3.0, 4.0, 0.0}) == Approx(5.0));

    // 上一条的 Δz 是 0，一个「只算 x/y」的实现照样通过（已用变异测试实测：丢掉 z 之后本文件 11 条断言全过）。补一条三个分量都非零的，并把 z 单独钉一次。
    CHECK(Point3{0.0, 0.0, 0.0}.DistanceTo(Point3{0.0, 0.0, 5.0}) == Approx(5.0));
    CHECK(Point3{1.0, 2.0, 3.0}.DistanceTo(Point3{2.0, 4.0, 5.0}) == Approx(3.0));

    // 非有限输入：文档里的规则说的是**差向量**。异号无穷 → 差向量含 ±inf → inf；两端是同一个无穷 → 差向量是 NaN（inf - inf）→ NaN。两句各要一条证据。
    const double infinity = std::numeric_limits<double>::infinity();
    CHECK(Point3{-infinity, 0.0, 0.0}.DistanceTo(Point3{infinity, 0.0, 0.0}) == infinity);
    CHECK(std::isnan(Point3{infinity, 0.0, 0.0}.DistanceTo(Point3{infinity, 0.0, 0.0})));
}

TEST_CASE("Point3 DistanceSquared is the sum of squared deltas", "[linear][point3]") {
    STATIC_REQUIRE(std::is_same_v<decltype(Point3{}.DistanceSquared(Point3{})), double>);

    // 三个分量都非零，丢掉 z 就不是 9。两端都离开原点，减法写成加法也对不上。
    CHECK(Point3{1.0, 2.0, 3.0}.DistanceSquared(Point3{2.0, 4.0, 5.0}) == 9.0);
    CHECK(Point3{2.0, 4.0, 5.0}.DistanceSquared(Point3{1.0, 2.0, 3.0}) == 9.0);
    CHECK(Point3{0.0, 0.0, 0.0}.DistanceSquared(Point3{0.0, 0.0, 5.0}) == 25.0);
    CHECK(Point3{1.0, 2.0, 3.0}.DistanceSquared(Point3{1.0, 2.0, 3.0}) == 0.0);

    // (1,1,1) 的距离是 √3。平方和必须精确是 3，不能先开方再平方。
    CHECK(Point3{0.0, 0.0, 0.0}.DistanceSquared(Point3{1.0, 1.0, 1.0}) == 3.0);

    constexpr Point3 a{1.0, 2.0, 3.0};
    constexpr Point3 b{2.0, 4.0, 5.0};
    STATIC_REQUIRE(a.DistanceSquared(b) == 9.0);
    STATIC_REQUIRE(noexcept(a.DistanceSquared(b)));

    const double infinity = std::numeric_limits<double>::infinity();
    CHECK(Point3{-infinity, 0.0, 0.0}.DistanceSquared(Point3{infinity, 0.0, 0.0}) == infinity);
    CHECK(std::isnan(Point3{infinity, 0.0, 0.0}.DistanceSquared(Point3{infinity, 0.0, 0.0})));
}

TEST_CASE("Point3 zero is the origin", "[linear][point3]") {
    CHECK(Point3::Zero.X == 0.0);
    CHECK(Point3::Zero.Y == 0.0);
    CHECK(Point3::Zero.Z == 0.0);
    CHECK(Point3::Zero == Point3{});
    CHECK(Point3::Zero.DistanceSquared(Point3{1.0, 2.0, 2.0}) == 9.0);
    CHECK(Point3f::Zero.X == 0.0f);
    CHECK(Point3f::Zero.Y == 0.0f);
    CHECK(Point3f::Zero.Z == 0.0f);
}

TEST_CASE("Point3 Lerp moves along the segment", "[linear][point3]") {
    const Point3 a{0.0, 0.0, 0.0};
    const Point3 b{2.0, 4.0, 6.0};
    CHECK(a.Lerp(b, 0.5) == Point3{1.0, 2.0, 3.0});
}
