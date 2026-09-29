#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <array>
#include <type_traits>

#include <GeoCore/linear/Point3.hpp>

using Catch::Approx;
using GeoCore::linear::Point3;
using GeoCore::linear::Point3T;
using GeoCore::linear::Point3f;
using GeoCore::linear::Vector3;

namespace {
/// 「能否相加」这个判断必须包在概念里，见下面 static_assert 处的说明。
template <typename T>
concept addable = requires(T p, T q) { p + q; };
}

TEST_CASE("the float alias really is the float instantiation",
          "[linear][point3]") {
    // 别名若从未被任何测试命名过，绑定写错时成员连一次实例化都不会发生 ——
    // 实测：把 Point3f 绑成 Point3T<double>，全库 134 个用例全绿、退出码 0。
    // float 用户会静默拿到 double 存储，精度与内存占用都不是承诺的样子。
    STATIC_REQUIRE(std::is_same_v<Point3f, Point3T<float>>);
}

TEST_CASE("Point3 supports subscript and array export", "[linear][point3]") {
    const Point3 p{1.0, 2.0, 3.0};

    CHECK(p[0] == 1.0);
    CHECK(p[1] == 2.0);
    CHECK(p[2] == 3.0);

    // 三个槽位**逐个**查。只查末槽是不够的：`{y, x, z}`、`{0, 0, z}` 这类
    // 错位实现都能通过（已用变异测试实测存活）。to_array() 是公开的互操作
    // 接口（喂给外部库 / GPU buffer），槽位错位就是静默的坐标错乱。
    // 同目录的 vector3_test.cpp 正是三个下标全查的，这里与它对齐。
    const std::array<double, 3> arr = p.to_array();
    CHECK(arr[0] == 1.0);
    CHECK(arr[1] == 2.0);
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

    // 两个点只要有一个分量不同就必须不相等。**这一条不能省** ——
    // 上面所有期望值的 x 分量都恰好与左操作数相同（11、-9 都是从 a.x=1 算出来的），
    // 于是一个「只比较 x」的 operator== 会让本文件全部断言静默通过（已用变异测试
    // 实测：把 == 改成只比 x，21 条断言全过）。
    CHECK(Point3{1.0, 2.0, 3.0} != Point3{1.0, 9.0, 3.0});
    CHECK(Point3{1.0, 2.0, 3.0} != Point3{1.0, 2.0, 9.0});
    CHECK(Point3{1.0, 2.0, 3.0} == Point3{1.0, 2.0, 3.0});

    // 点 + 点不存在 —— 这是本任务的核心不变量（Review Focus 第 1 条）：
    // 写错了不会报错，只会静默给出错误语义。必须真的断言，不能只注释掉。
    //
    // 不能直接写 `static_assert(!requires(Point3 p, Point3 q) { p + q; });`
    // —— 那不是「求值为假」，而是**非良构的 C++**。requirements 里的非法表达式
    // 只有在「替换模板实参」的语境下才求值为 false；`Point3` 是具体类型，
    // `p + q` 不依赖任何模板参数，因此是硬错误（MSVC 报 C2676，编译器不会
    // 给你 false）。必须包一层概念，让失败发生在替换上下文里 —— 见文件顶部
    // 匿名命名空间的 `addable`。这一点与 Catch2 无关，也与用不用
    // STATIC_REQUIRE 无关。
    static_assert(!addable<Point3>, "Point + Point must not be well-formed");
}

TEST_CASE("Point3 distance_to", "[linear][point3]") {
    CHECK(Point3{0.0, 0.0, 0.0}.distance_to(Point3{3.0, 4.0, 0.0}) == Approx(5.0));

    // 上一条的 Δz 是 0，一个「只算 x/y」的实现照样通过（已用变异测试实测：
    // 丢掉 z 之后本文件 11 条断言全过）。补一条三个分量都非零的，
    // 并把 z 单独钉一次。
    CHECK(Point3{0.0, 0.0, 0.0}.distance_to(Point3{0.0, 0.0, 5.0}) == Approx(5.0));
    CHECK(Point3{1.0, 2.0, 3.0}.distance_to(Point3{2.0, 4.0, 5.0}) == Approx(3.0));
}
