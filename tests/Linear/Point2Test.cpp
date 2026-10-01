#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cmath>
#include <limits>
#include <type_traits>

#include <DragonGeo/Linear/Point2.hpp>

using Catch::Approx;
using DragonGeo::Linear::Point2;
using DragonGeo::Linear::Point2T;
using DragonGeo::Linear::Point2f;
using DragonGeo::Linear::Vector2;

namespace {
/// 同 point3Test.cpp：判断「能否相加」必须包在概念里。
template <typename T>
concept addable = requires(T p, T q) { p + q; };
}

TEST_CASE("the float alias really is the float instantiation",
          "[linear][point2]") {
    // 同 point3Test.cpp。二维此前**同样没有任何防线**：实测只把别名 using
    // 进来（不补断言）、把 Point2f 绑成 Point2T<double>，编译通过、13 条断言
    // 全过、退出码 0。所以下面这条不是「加固一道已有的防线」，它是这里**唯一**
    // 的证据。
    STATIC_REQUIRE(std::is_same_v<Point2f, Point2T<float>>);
}

TEST_CASE("Point2 supports subscript and array export", "[linear][point2]") {
    const Point2 p{1.0, 2.0};

    CHECK(p[0] == 1.0);
    CHECK(p[1] == 2.0);

    // 同 point3Test.cpp：两个槽位逐个查，只查末槽会让 `{0, y}` 这类
    // 错位实现静默通过。
    const std::array<double, 2> arr = p.ToArray();
    CHECK(arr[0] == 1.0);
    CHECK(arr[1] == 2.0);
}

TEST_CASE("the mutable subscript writes the component it names",
          "[linear][point2]") {
    // 同 point3Test.cpp：非 const 重载必须被真的用一次，否则它从未被实例化，
    // 分量对调也无人察觉。
    Point2 p{0.0, 0.0};
    p[0] = 4.0;
    p[1] = 5.0;

    CHECK(p.X == 4.0);
    CHECK(p.Y == 5.0);
}

TEST_CASE("Point2 keeps its declaration-level guarantees",
          "[linear][point2]") {
    STATIC_REQUIRE(std::is_same_v<Point2T<float>::ScalarType, float>);
    STATIC_REQUIRE(std::is_same_v<Point2T<double>::ScalarType, double>);
    STATIC_REQUIRE(noexcept(Point2{}.DistanceTo(Point2{})));

    // 其余 8 处 noexcept 同样要有证据（Task 4 曾记录后放行，这里与 Interval
    // 的 11 处口径统一）。逐条都有死亡证明：删掉声明上的 noexcept，该条即
    // 编译失败。const 重载只能由具名 const 对象选中，故先声明 sample。
    const Point2 sample{1.0, 2.0};
    STATIC_REQUIRE(noexcept(Point2{}.operator[](0)));   // 非 const 重载
    STATIC_REQUIRE(noexcept(sample[0]));                // const 重载
    STATIC_REQUIRE(noexcept(Point2{}.ToArray()));
    STATIC_REQUIRE(noexcept(sample + Vector2{1.0, 2.0}));
    STATIC_REQUIRE(noexcept(Vector2{1.0, 2.0} + sample));
    STATIC_REQUIRE(noexcept(sample - Vector2{1.0, 2.0}));
    STATIC_REQUIRE(noexcept(sample - sample));
    STATIC_REQUIRE(noexcept(sample == sample));

    // 同 point3Test.cpp：常量求值 + **不带花括号**的默认初始化。
    constexpr Point2T<double> defaultPoint;
    STATIC_REQUIRE(defaultPoint.X == 0.0);
    STATIC_REQUIRE(defaultPoint.Y == 0.0);
}

TEST_CASE("copy assignment carries every component", "[linear][point2]") {
    const Point2 source{4.0, 5.0};
    Point2 target{0.0, 0.0};
    target = source;

    CHECK(target.X == 4.0);
    CHECK(target.Y == 5.0);
}

TEST_CASE("Point2 arithmetic follows the same affine rules as Point3",
          "[linear][point2]") {
    const Point2 a{1.0, 2.0};
    const Point2 b{4.0, 6.0};
    const Vector2 v{10.0, 20.0};

    STATIC_REQUIRE(std::is_same_v<decltype(a + v), Point2>);
    CHECK(a + v == Point2{11.0, 22.0});

    // 向量 + 点 = 点 —— 上一条的反向书写，同 3D 侧。平移不关心书写顺序，
    // 两种写法必须同义；调用方先写出向量时不该撞上「找不到运算符」。
    STATIC_REQUIRE(std::is_same_v<decltype(v + a), Point2>);
    CHECK(v + a == Point2{11.0, 22.0});
    CHECK(v + a == a + v);

    STATIC_REQUIRE(std::is_same_v<decltype(a - v), Point2>);
    CHECK(a - v == Point2{-9.0, -18.0});

    STATIC_REQUIRE(std::is_same_v<decltype(b - a), Vector2>);
    CHECK(b - a == Vector2{3.0, 4.0});

    // 两个分量**各要一条**不同方向的证据。只钉 y 是不够的：一个
    // 「只比较 y」的 operator== 会让本文件全绿（已实测存活），
    // 于是 Point2{1,2} == Point2{5,2} 被静默判为相等。
    CHECK(Point2{1.0, 2.0} != Point2{1.0, 9.0});   // y 不同
    CHECK(Point2{1.0, 2.0} != Point2{9.0, 2.0});   // x 不同
    CHECK(Point2{1.0, 2.0} == Point2{1.0, 2.0});

    static_assert(!addable<Point2>, "Point + Point must not be well-formed");
}

TEST_CASE("Point2 DistanceTo", "[linear][point2]") {
    STATIC_REQUIRE(std::is_same_v<decltype(Point2{}.DistanceTo(Point2{})), double>);

    // **两个操作数都不能是原点。** 用原点做操作数时，`x - other.X` 与
    // `x + other.X` 平方后相同，把 dx 变号也能存活（实测：2D 侧存活，
    // 3D 侧正因为下面第三条用了非原点对才被杀掉）。
    CHECK(Point2{0.0, 0.0}.DistanceTo(Point2{3.0, 4.0}) == Approx(5.0));
    CHECK(Point2{1.0, 2.0}.DistanceTo(Point2{4.0, 6.0}) == Approx(5.0));

    // 非有限输入：文档里的规则说的是**差向量**。异号无穷 → 差向量含 ±inf → inf；
    // 两端是同一个无穷 → 差向量是 NaN（inf - inf）→ NaN。两句各要一条证据。
    const double infinity = std::numeric_limits<double>::infinity();
    CHECK(Point2{-infinity, 0.0}.DistanceTo(Point2{infinity, 0.0}) == infinity);
    CHECK(std::isnan(Point2{infinity, 0.0}.DistanceTo(Point2{infinity, 0.0})));
}

TEST_CASE("Point2 DistanceSquared is the sum of squared deltas", "[linear][point2]") {
    STATIC_REQUIRE(std::is_same_v<decltype(Point2{}.DistanceSquared(Point2{})), double>);

    // 两端都离开原点：把减法写成加法时，平方后不再相同。
    CHECK(Point2{1.0, 2.0}.DistanceSquared(Point2{4.0, 6.0}) == 25.0);
    CHECK(Point2{4.0, 6.0}.DistanceSquared(Point2{1.0, 2.0}) == 25.0);
    CHECK(Point2{1.0, 2.0}.DistanceSquared(Point2{1.0, 2.0}) == 0.0);

    // (1,1) 的距离是 √2。先开方再平方一般回不到 2，平方和必须精确是 2。
    CHECK(Point2{0.0, 0.0}.DistanceSquared(Point2{1.0, 1.0}) == 2.0);

    constexpr Point2 a{1.0, 2.0};
    constexpr Point2 b{4.0, 6.0};
    STATIC_REQUIRE(a.DistanceSquared(b) == 25.0);
    STATIC_REQUIRE(noexcept(a.DistanceSquared(b)));

    const double infinity = std::numeric_limits<double>::infinity();
    CHECK(Point2{-infinity, 0.0}.DistanceSquared(Point2{infinity, 0.0}) == infinity);
    CHECK(std::isnan(Point2{infinity, 0.0}.DistanceSquared(Point2{infinity, 0.0})));
}

TEST_CASE("Point2 zero is the origin", "[linear][point2]") {
    CHECK(Point2::Zero.X == 0.0);
    CHECK(Point2::Zero.Y == 0.0);
    CHECK(Point2::Zero == Point2{});
    CHECK(Point2::Zero.DistanceSquared(Point2{3.0, 4.0}) == 25.0);
    CHECK(Point2f::Zero.X == 0.0f);
    CHECK(Point2f::Zero.Y == 0.0f);
}

TEST_CASE("Point2 Lerp moves along the segment", "[linear][point2]") {
    const Point2 a{0.0, 0.0};
    const Point2 b{10.0, 20.0};

    CHECK(a.Lerp(b, 0.0) == a);
    CHECK(a.Lerp(b, 1.0) == b);
    CHECK(a.Lerp(b, 0.5) == Point2{5.0, 10.0});

    constexpr Point2 ca{1.0, 1.0};
    constexpr Point2 cb{3.0, 5.0};
    STATIC_REQUIRE(ca.Lerp(cb, 0.5) == Point2{2.0, 3.0});
}
