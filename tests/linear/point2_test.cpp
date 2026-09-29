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
}

TEST_CASE("the float alias really is the float instantiation",
          "[linear][point2]") {
    // 同 point3_test.cpp。二维的「绑定写错」在列表初始化收窄检查下会编译失败，
    // 但那条**偶然**的防线不该被依赖 —— 断言本身才是承诺。
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

    // 同 point3_test.cpp：== 的 y 分量必须有独立证据，否则一个「只比较 x」
    // 的实现会让本文件全部断言静默通过。
    CHECK(Point2{1.0, 2.0} != Point2{1.0, 9.0});
    CHECK(Point2{1.0, 2.0} == Point2{1.0, 2.0});

    static_assert(!addable<Point2>, "Point + Point must not be well-formed");
}

TEST_CASE("Point2 distance_to", "[linear][point2]") {
    CHECK(Point2{0.0, 0.0}.distance_to(Point2{3.0, 4.0}) == Approx(5.0));
}
