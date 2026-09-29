#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <array>
#include <type_traits>

#include <GeoCore/linear/Point2.hpp>

using Catch::Approx;
using GeoCore::linear::Point2;
using GeoCore::linear::Point2T;
using GeoCore::linear::Point2f;
using GeoCore::linear::Vector2;

namespace {
/// 同 point3_test.cpp：判断「能否相加」必须包在概念里。
template <typename T>
concept addable = requires(T p, T q) { p + q; };

template <typename A, typename B>
concept addable_pair = requires(A a, B b) { a + b; };
}

TEST_CASE("the float alias really is the float instantiation",
          "[linear][point2]") {
    // 同 point3_test.cpp。二维此前**同样没有任何防线**：实测只把别名 using
    // 进来（不补断言）、把 Point2f 绑成 Point2T<double>，编译通过、13 条断言
    // 全过、退出码 0。所以下面这条不是「加固一道已有的防线」，它是这里**唯一**
    // 的证据。
    STATIC_REQUIRE(std::is_same_v<Point2f, Point2T<float>>);
}

TEST_CASE("Point2 supports subscript and array export", "[linear][point2]") {
    const Point2 p{1.0, 2.0};

    CHECK(p[0] == 1.0);
    CHECK(p[1] == 2.0);

    // 同 point3_test.cpp：两个槽位逐个查，只查末槽会让 `{0, y}` 这类
    // 错位实现静默通过。
    const std::array<double, 2> arr = p.to_array();
    CHECK(arr[0] == 1.0);
    CHECK(arr[1] == 2.0);
}

TEST_CASE("the mutable subscript writes the component it names",
          "[linear][point2]") {
    // 同 point3_test.cpp：非 const 重载必须被真的用一次，否则它从未被实例化，
    // 分量对调也无人察觉。
    Point2 p{0.0, 0.0};
    p[0] = 4.0;
    p[1] = 5.0;

    CHECK(p.x == 4.0);
    CHECK(p.y == 5.0);
}

TEST_CASE("Point2 keeps its declaration-level guarantees",
          "[linear][point2]") {
    STATIC_REQUIRE(std::is_same_v<Point2T<float>::scalar_type, float>);
    STATIC_REQUIRE(std::is_same_v<Point2T<double>::scalar_type, double>);
    STATIC_REQUIRE(noexcept(Point2{}.distance_to(Point2{})));

    // 同 point3_test.cpp：常量求值 + **不带花括号**的默认初始化。
    constexpr Point2T<double> default_point;
    STATIC_REQUIRE(default_point.x == 0.0);
    STATIC_REQUIRE(default_point.y == 0.0);
}

TEST_CASE("copy assignment carries every component", "[linear][point2]") {
    const Point2 source{4.0, 5.0};
    Point2 target{0.0, 0.0};
    target = source;

    CHECK(target.x == 4.0);
    CHECK(target.y == 5.0);
}

TEST_CASE("Point2 arithmetic follows the same affine rules as Point3",
          "[linear][point2]") {
    const Point2 a{1.0, 2.0};
    const Point2 b{4.0, 6.0};
    const Vector2 v{10.0, 20.0};

    STATIC_REQUIRE(std::is_same_v<decltype(a + v), Point2>);
    CHECK(a + v == Point2{11.0, 22.0});

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
    static_assert(!addable_pair<Vector2, Point2>,
                  "Vector + Point must not be well-formed");
}

TEST_CASE("Point2 distance_to", "[linear][point2]") {
    STATIC_REQUIRE(std::is_same_v<decltype(Point2{}.distance_to(Point2{})), double>);

    // **两个操作数都不能是原点。** 用原点做操作数时，`x - other.x` 与
    // `x + other.x` 平方后相同，把 dx 变号也能存活（实测：2D 侧存活，
    // 3D 侧正因为下面第三条用了非原点对才被杀掉）。
    CHECK(Point2{0.0, 0.0}.distance_to(Point2{3.0, 4.0}) == Approx(5.0));
    CHECK(Point2{1.0, 2.0}.distance_to(Point2{4.0, 6.0}) == Approx(5.0));
}
