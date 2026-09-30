#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cmath>
#include <limits>
#include <type_traits>

#include <GeoCore/linear/UnitVector3.hpp>
#include <GeoCore/linear/Vector3.hpp>

using Catch::Approx;

using GeoCore::linear::Vector3;
using GeoCore::linear::Vector3T;
using GeoCore::linear::Vector3f;

TEST_CASE("Vector3 arithmetic is component-wise", "[linear][vector3]") {
    const Vector3 a{1.0, 2.0, 3.0};
    const Vector3 b{4.0, 5.0, 6.0};

    CHECK(a + b == Vector3{5.0, 7.0, 9.0});
    CHECK(b - a == Vector3{3.0, 3.0, 3.0});
    CHECK(-a == Vector3{-1.0, -2.0, -3.0});
    CHECK(a * 2.0 == Vector3{2.0, 4.0, 6.0});
    CHECK(b / 2.0 == Vector3{2.0, 2.5, 3.0});
}

TEST_CASE("dot is symmetric and matches the definition", "[linear][vector3]") {
    const Vector3 a{1.0, 2.0, 3.0};
    const Vector3 b{4.0, -5.0, 6.0};

    CHECK(a.dot(b) == 1.0 * 4.0 + 2.0 * -5.0 + 3.0 * 6.0);
    CHECK(a.dot(b) == b.dot(a));
}

TEST_CASE("cross is anticommutative and right-handed", "[linear][vector3]") {
    const Vector3 x_axis{1.0, 0.0, 0.0};
    const Vector3 y_axis{0.0, 1.0, 0.0};
    const Vector3 z_axis{0.0, 0.0, 1.0};

    CHECK(x_axis.cross(y_axis) == z_axis);
    CHECK(y_axis.cross(x_axis) == -z_axis);
    CHECK(x_axis.cross(x_axis) == Vector3{0.0, 0.0, 0.0});

    const Vector3 a{1.0, 2.0, 3.0};
    const Vector3 b{4.0, 5.0, 6.0};
    CHECK(a.cross(b) == -b.cross(a));

    // 上面这些断言全是输出的线性性质，任何固定线性映射都能满足它们 ——
    // 反交换性对任意 S 都成立（S(b×a) = -S(a×b)），零结果对线性映射也
    // 仍是零。因此形如 diag(s1, s2, 1)·(a×b) 的实现能通过全部断言，
    // 包括 s1 = s2 = 0（x、y 分量恒为零）。下面用三个轴的循环和一个
    // 一般对，把每个分量各自钉死。
    CHECK(y_axis.cross(z_axis) == x_axis);
    CHECK(z_axis.cross(x_axis) == y_axis);
    CHECK(a.cross(b) == Vector3{-3.0, 6.0, -3.0});
}

TEST_CASE("cross of parallel vectors is the zero vector", "[linear][vector3]") {
    const Vector3 a{1.0, 2.0, 3.0};
    const Vector3 b{2.0, 4.0, 6.0};
    CHECK(a.cross(b) == Vector3{0.0, 0.0, 0.0});
}

TEST_CASE("length survives extreme magnitudes", "[linear][vector3][degenerate]") {
    const Vector3 huge{1e200, 1e200, 0.0};
    CHECK(std::isfinite(huge.length()));
    CHECK(huge.length() == Approx(1.4142135623730951e200).epsilon(1e-12));

    const Vector3 tiny{1e-200, 1e-200, 0.0};
    CHECK(tiny.length() > 0.0);
    CHECK(tiny.length() == Approx(1.4142135623730951e-200).epsilon(1e-12));
}

TEST_CASE("length of the 1-2-2 vector is 3", "[linear][vector3]") {
    const Vector3 v{1.0, 2.0, 2.0};
    CHECK(v.length_squared() == 9.0);
    CHECK(v.length() == 3.0);
}

TEST_CASE("Vector3T is usable with float", "[linear][vector3]") {
    // 同 vector2_test.cpp：`v.length() == 3.0f` 抓不住误绑定 —— 两种精度下
    // 都得 3.0，且 float 字面量与 double 比较时会提升。钉的是绑定本身。
    STATIC_REQUIRE(std::is_same_v<Vector3f, Vector3T<float>>);

    const Vector3f v{1.0f, 2.0f, 2.0f};
    CHECK(v.length() == 3.0f);
}

TEST_CASE("length of a vector containing infinity is infinity, not NaN",
          "[linear][vector3][degenerate]") {
    const double infinity = std::numeric_limits<double>::infinity();

    // 同 Vector2T：缩放写法若不特判，inf / inf 会算出 NaN。
    CHECK(Vector3{infinity, 1.0, 1.0}.length() == infinity);
    CHECK(Vector3{1.0, infinity, 1.0}.length() == infinity);
    CHECK(Vector3{infinity, infinity, infinity}.length() == infinity);
}

TEST_CASE("length of a vector containing NaN is NaN whatever the slot order",
          "[linear][vector3][degenerate]") {
    const double infinity = std::numeric_limits<double>::infinity();
    const double nan = std::numeric_limits<double>::quiet_NaN();

    // 这里的嵌套三目曾经让 {5, NaN, 0} 返回 0：`5 > NaN` 与 `NaN > 0` 都是
    // false，fold 于是保留了零槽的值，再被 scale == 0 的提前返回放大成
    // 「长度为零」。规则改为：任一无穷分量 ⇒ ±inf；否则含 NaN ⇒ NaN。
    CHECK(std::isnan(Vector3{5.0, nan, 0.0}.length()));
    CHECK(std::isnan(Vector3{nan, 5.0, 0.0}.length()));
    CHECK(std::isnan(Vector3{0.0, nan, 5.0}.length()));
    // 无 NaN 的输入不受这条规则影响
    CHECK(Vector3{5.0, 0.0, 0.0}.length() == 5.0);

    // 无穷压过 NaN，且与槽位无关 —— 旧实现里 {NaN, inf, 1} 与 {inf, NaN, 1}
    // 给出的答案不同（nan 与 inf）。
    CHECK(Vector3{infinity, nan, 1.0}.length() == infinity);
    CHECK(Vector3{nan, infinity, 1.0}.length() == infinity);
    CHECK(Vector3{1.0, nan, infinity}.length() == infinity);
}

TEST_CASE("Vector3 members: dot, cross, subscript and array export",
          "[linear][vector3]") {
    const Vector3 a{1.0, 2.0, 3.0};
    const Vector3 b{4.0, 5.0, 6.0};

    // 成员形式的 dot/cross
    CHECK(a.dot(b) == 32.0);
    CHECK(a.cross(b) == Vector3{-3.0, 6.0, -3.0});

    // 下标访问：索引 0/1/2 对应 x/y/z
    CHECK(a[0] == 1.0);
    CHECK(a[1] == 2.0);
    CHECK(a[2] == 3.0);

    // 非 const 下标可写
    Vector3 mutable_v{};
    mutable_v[0] = 7.0;
    mutable_v[1] = 8.0;
    mutable_v[2] = 9.0;
    CHECK(mutable_v == Vector3{7.0, 8.0, 9.0});

    // 数组导出
    const std::array<double, 3> arr = a.to_array();
    CHECK(arr[0] == 1.0);
    CHECK(arr[1] == 2.0);
    CHECK(arr[2] == 3.0);
}

TEST_CASE("Vector3 normalized is a member returning optional",
          "[linear][vector3][degenerate]") {
    const auto unit = Vector3{3.0, 4.0, 0.0}.normalized();

    REQUIRE(unit.has_value());
    CHECK(unit->x() == Approx(0.6));
    CHECK(unit->y() == Approx(0.8));

    // 零向量仍然拒绝，且与旧自由函数 normalize 语义一致
    CHECK_FALSE(Vector3{0.0, 0.0, 0.0}.normalized().has_value());
}
