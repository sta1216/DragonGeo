#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <limits>
#include <type_traits>

#include <GeoCore/linear/Vector2.hpp>

using Catch::Approx;

using GeoCore::linear::Vector2;
using GeoCore::linear::Vector2T;
using GeoCore::linear::Vector2f;

TEST_CASE("Vector2 supports aggregate initialization", "[linear][vector2]") {
    const Vector2 v{3.0, 4.0};
    CHECK(v.x == 3.0);
    CHECK(v.y == 4.0);
}

TEST_CASE("default-constructed Vector2 is the zero vector", "[linear][vector2]") {
    const Vector2 v{};
    CHECK(v.x == 0.0);
    CHECK(v.y == 0.0);
    CHECK(v.length() == 0.0);
}

TEST_CASE("Vector2 arithmetic is component-wise", "[linear][vector2]") {
    const Vector2 a{1.0, 2.0};
    const Vector2 b{3.0, 5.0};

    CHECK(a + b == Vector2{4.0, 7.0});
    CHECK(b - a == Vector2{2.0, 3.0});
    CHECK(-a == Vector2{-1.0, -2.0});
    CHECK(a * 2.0 == Vector2{2.0, 4.0});
    CHECK(2.0 * a == Vector2{2.0, 4.0});
    CHECK(b / 2.0 == Vector2{1.5, 2.5});
}

TEST_CASE("scalar multiplication accepts integer factors", "[linear][vector2]") {
    const Vector2 a{1.0, 2.0};
    CHECK(a * 3 == Vector2{3.0, 6.0});
}

TEST_CASE("dot and cross match their definitions", "[linear][vector2]") {
    const Vector2 a{1.0, 0.0};
    const Vector2 b{0.0, 1.0};

    CHECK(a.dot(b) == 0.0);
    CHECK(a.dot(a) == 1.0);
    CHECK(a.cross(b) == 1.0);
    CHECK(b.cross(a) == -1.0);
    CHECK(a.cross(a) == 0.0);

    // {1,0}·{0,1} 恒为 0，第二项上任何符号错误都看不出来。用一对各分量
    // 都非零的通用输入把两项的符号钉住：正确 1*3 + 2*5 = 13，第二项符号
    // 错为 1*3 - 2*5 = -7。
    CHECK(Vector2{1.0, 2.0}.dot(Vector2{3.0, 5.0}) == 13.0);
    CHECK(Vector2{1.0, 2.0}.cross(Vector2{3.0, 5.0}) == -1.0);
}

TEST_CASE("length of the 3-4-5 triangle is exact", "[linear][vector2]") {
    const Vector2 v{3.0, 4.0};
    CHECK(v.length_squared() == 25.0);
    CHECK(v.length() == 5.0);
}

TEST_CASE("length survives components near the overflow threshold",
          "[linear][vector2][degenerate]") {
    // 朴素实现会先算出 1e200 * 1e200 == inf
    const Vector2 v{1e200, 1e200};
    const double length = v.length();

    CHECK(std::isfinite(length));
    CHECK(length == Approx(1.4142135623730951e200).epsilon(1e-12));
}

TEST_CASE("length survives components near the underflow threshold",
          "[linear][vector2][degenerate]") {
    // 朴素实现会先算出 1e-200 * 1e-200 == 0
    const Vector2 v{1e-200, 1e-200};
    const double length = v.length();

    CHECK(length > 0.0);
    CHECK(length == Approx(1.4142135623730951e-200).epsilon(1e-12));
}

TEST_CASE("length of a vector containing NaN does not crash",
          "[linear][vector2][degenerate]") {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const Vector2 v{nan, 1.0};
    CHECK(std::isnan(v.length()));
}

TEST_CASE("Vector2T is usable with float", "[linear][vector2]") {
    // 被用到不等于被钉住：`v.length() == 5.0f` 在 Vector2T<float> 与
    // Vector2T<double> 下都成立（float 字面量比较时会提升），把 Vector2f 绑成
    // Vector2T<double> 它照样通过。对**绑定本身**的断言只能是 is_same_v。
    STATIC_REQUIRE(std::is_same_v<Vector2f, Vector2T<float>>);

    const Vector2f v{3.0f, 4.0f};
    CHECK(v.length() == 5.0f);
    CHECK(v.x == 3.0f);
}

TEST_CASE("length of a vector containing infinity is infinity, not NaN",
          "[linear][vector2][degenerate]") {
    const double infinity = std::numeric_limits<double>::infinity();

    // 缩放写法若不特判，inf / inf 会算出 NaN 并污染整条计算链 ——
    // 缩放本是为消除溢出而引入，不能反而在无穷输入上退化。
    CHECK(Vector2{infinity, 1.0}.length() == infinity);
    CHECK(Vector2{1.0, infinity}.length() == infinity);
    CHECK(Vector2{infinity, infinity}.length() == infinity);
}

TEST_CASE("length of a vector containing NaN is NaN whatever the slot order",
          "[linear][vector2][degenerate]") {
    const double infinity = std::numeric_limits<double>::infinity();
    const double nan = std::numeric_limits<double>::quiet_NaN();

    // 逐项取最大绝对值时，与 NaN 的比较一律返回 false，fold 会静默保留前一个
    // 值，答案于是取决于 NaN 落在哪一槽。规则固定为：任一无穷分量 ⇒ ±inf；
    // 否则含 NaN ⇒ NaN。槽位与元数都不再影响结果。
    CHECK(std::isnan(Vector2{5.0, nan}.length()));
    CHECK(std::isnan(Vector2{nan, 5.0}.length()));

    // 无穷压过 NaN：这一条把上面那条规则的优先级也钉住
    CHECK(Vector2{infinity, nan}.length() == infinity);
    CHECK(Vector2{nan, infinity}.length() == infinity);
}

TEST_CASE("Vector2 members: dot, cross and subscript", "[linear][vector2]") {
    const Vector2 a{1.0, 2.0};
    const Vector2 b{3.0, 5.0};

    CHECK(a.dot(b) == 13.0);          // 1*3 + 2*5
    CHECK(a.cross(b) == -1.0);        // 1*5 - 2*3
    CHECK(a[0] == 1.0);
    CHECK(a[1] == 2.0);
}
