#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <type_traits>

#include <GeoCore/linear/Point3.hpp>

using Catch::Approx;
using GeoCore::linear::Point3;
using GeoCore::linear::Vector3;

namespace {

/// `T + T` 是否良构？用来把「不存在 Point + Point」这条不变量写成编译期断言。
///
/// 为什么包一层概念、而不直接写
/// `static_assert(!requires(Point3 p, Point3 q) { p + q; }, ...)`：
/// 按 [expr.prim.req.general] 的 Note，requirements 中出现非法表达式、而该
/// requires-expression 又不在「模板化实体」的声明中时，**程序本身即非良构**
/// （MSVC 报 C2676 no operator found），而不是求值为 false —— 直写会连测试
/// 都编不过，且诊断发生在断言之外的地方。
///
/// 放进概念后，`Point3` 代入产生的失败是寻常的替换失败，概念取值为 false，
/// `static_assert` 才真正拿到它要断言的那个 false。这仍然是编译期、由编译器
/// 亲自判定的检查 —— 不是「目测没问题」。
template <typename T>
concept addable = requires(T p, T q) { p + q; };

} // namespace

TEST_CASE("Point3 supports subscript and array export", "[linear][point3]") {
    const Point3 p{1.0, 2.0, 3.0};

    CHECK(p[0] == 1.0);
    CHECK(p[1] == 2.0);
    CHECK(p[2] == 3.0);

    const std::array<double, 3> arr = p.to_array();
    CHECK(arr[2] == 3.0);
}

TEST_CASE("point and vector arithmetic follows affine rules",
          "[linear][point3]") {
    const Point3 a{1.0, 2.0, 3.0};
    const Point3 b{4.0, 6.0, 8.0};
    const Vector3 v{10.0, 20.0, 30.0};

    // 点 + 向量 = 点
    STATIC_REQUIRE(std::is_same_v<decltype(a + v), Point3>);
    CHECK(a + v == Point3{11.0, 22.0, 33.0});

    // 点 - 向量 = 点
    STATIC_REQUIRE(std::is_same_v<decltype(a - v), Point3>);
    CHECK(a - v == Point3{-9.0, -18.0, -27.0});

    // 点 - 点 = 向量
    STATIC_REQUIRE(std::is_same_v<decltype(b - a), Vector3>);
    CHECK(b - a == Vector3{3.0, 4.0, 5.0});

    // 点 + 点不存在 —— 这是本任务的核心不变量（Review Focus 第 1 条）：
    // 写错了不会报错，只会静默给出错误语义。必须真的断言，不能只注释掉。
    //
    // 用普通 static_assert 而非 Catch2 的 STATIC_REQUIRE：后者要把表达式分解成
    // 左右操作数，而 concept 没有可分解的运算符。块作用域的 static_assert 无此问题。
    //
    // `addable` 见文件顶部的匿名命名空间 —— 包一层概念是**编译期**要求，
    // 不是风格偏好，原因见那里的注释。
    static_assert(!addable<Point3>,
                  "Point + Point must not be well-formed");
}

TEST_CASE("Point3 distance_to", "[linear][point3]") {
    CHECK(Point3{0.0, 0.0, 0.0}.distance_to(Point3{3.0, 4.0, 0.0}) == Approx(5.0));
}
