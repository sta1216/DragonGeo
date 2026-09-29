#include <catch2/catch_test_macros.hpp>

#include <limits>

#include <GeoCore/linear/Vector4.hpp>

using GeoCore::linear::Vector4;

TEST_CASE("Vector4 arithmetic is component-wise", "[linear][vector4]") {
    const Vector4 a{1.0, 2.0, 3.0, 4.0};
    const Vector4 b{5.0, 6.0, 7.0, 8.0};

    CHECK(a + b == Vector4{6.0, 8.0, 10.0, 12.0});
    CHECK(b - a == Vector4{4.0, 4.0, 4.0, 4.0});
    CHECK(a * 2.0 == Vector4{2.0, 4.0, 6.0, 8.0});
}

TEST_CASE("Vector4 dot includes the w component", "[linear][vector4]") {
    const Vector4 a{1.0, 2.0, 3.0, 4.0};
    const Vector4 b{1.0, 1.0, 1.0, 1.0};
    CHECK(dot(a, b) == 10.0);
}

TEST_CASE("Vector4 length includes the w component", "[linear][vector4]") {
    const Vector4 v{1.0, 2.0, 2.0, 4.0};
    CHECK(v.length_squared() == 25.0);
    CHECK(v.length() == 5.0);
}

TEST_CASE("default-constructed Vector4 is zero", "[linear][vector4]") {
    const Vector4 v{};
    CHECK(v.w == 0.0);
    CHECK(v.length() == 0.0);
}

TEST_CASE("length of a vector containing infinity is infinity, not NaN",
          "[linear][vector4][degenerate]") {
    const double infinity = std::numeric_limits<double>::infinity();

    // 同 Vector2T：缩放写法若不特判，inf / inf 会算出 NaN。
    CHECK(Vector4{infinity, 1.0, 1.0, 1.0}.length() == infinity);
    CHECK(Vector4{1.0, 1.0, 1.0, infinity}.length() == infinity);
    CHECK(Vector4{infinity, infinity, infinity, infinity}.length() == infinity);
}

TEST_CASE("length of a vector containing NaN is NaN whatever the slot order",
          "[linear][vector4][degenerate]") {
    const double infinity = std::numeric_limits<double>::infinity();
    const double nan = std::numeric_limits<double>::quiet_NaN();

    // 规则与 Vector2T/Vector3T 相同：任一无穷分量 ⇒ ±inf；否则含 NaN ⇒ NaN。
    // 旧实现里 {inf, NaN, 1, 1} 得到 nan，而 Vector3 的同形输入得到 inf ——
    // 同一个量在相邻元数的类型上互相矛盾，正是这条规则要消除的。
    CHECK(std::isnan(Vector4{5.0, nan, 0.0, 0.0}.length()));
    CHECK(std::isnan(Vector4{nan, 5.0, 0.0, 0.0}.length()));
    CHECK(std::isnan(Vector4{0.0, nan, 5.0, 0.0}.length()));
    CHECK(std::isnan(Vector4{0.0, 0.0, 0.0, nan}.length()));

    CHECK(Vector4{infinity, nan, 1.0, 1.0}.length() == infinity);
    CHECK(Vector4{nan, infinity, 1.0, 1.0}.length() == infinity);
    CHECK(Vector4{1.0, 1.0, nan, infinity}.length() == infinity);
}
