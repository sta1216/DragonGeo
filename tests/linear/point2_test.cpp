#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <type_traits>

#include <GeoCore/linear/Point2.hpp>

using Catch::Approx;
using GeoCore::linear::Point2;
using GeoCore::linear::Vector2;

namespace {

/// `T + T` 是否良构？用来把「不存在 Point + Point」这条不变量写成编译期断言。
///
/// 为什么包一层概念、而不直接写
/// `static_assert(!requires(Point2 p, Point2 q) { p + q; }, ...)`：
/// 按 [expr.prim.req.general] 的 Note，requirements 中出现非法表达式、而该
/// requires-expression 又不在「模板化实体」的声明中时，**程序本身即非良构**
/// （MSVC 报 C2676 no operator found），而不是求值为 false —— 直写会连测试
/// 都编不过，且诊断发生在断言之外的地方。
///
/// 放进概念后，`Point2` 代入产生的失败是寻常的替换失败，概念取值为 false，
/// `static_assert` 才真正拿到它要断言的那个 false。这仍然是编译期、由编译器
/// 亲自判定的检查 —— 不是「目测没问题」。
template <typename T>
concept addable = requires(T p, T q) { p + q; };

} // namespace

TEST_CASE("Point2 supports subscript and array export", "[linear][point2]") {
    const Point2 p{1.0, 2.0};

    CHECK(p[0] == 1.0);
    CHECK(p[1] == 2.0);

    const std::array<double, 2> arr = p.to_array();
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

    // `addable` 见文件顶部的匿名命名空间 —— 包一层概念是**编译期**要求，
    // 不是风格偏好，原因见那里的注释。
    static_assert(!addable<Point2>,
                  "Point + Point must not be well-formed");
}

TEST_CASE("Point2 distance_to", "[linear][point2]") {
    CHECK(Point2{0.0, 0.0}.distance_to(Point2{3.0, 4.0}) == Approx(5.0));
}
