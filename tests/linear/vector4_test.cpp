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
