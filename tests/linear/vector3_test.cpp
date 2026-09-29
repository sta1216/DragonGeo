#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <limits>

#include <GeoCore/linear/Vector3.hpp>

using Catch::Approx;

using GeoCore::linear::Vector3;
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

    CHECK(dot(a, b) == 1.0 * 4.0 + 2.0 * -5.0 + 3.0 * 6.0);
    CHECK(dot(a, b) == dot(b, a));
}

TEST_CASE("cross is anticommutative and right-handed", "[linear][vector3]") {
    const Vector3 x_axis{1.0, 0.0, 0.0};
    const Vector3 y_axis{0.0, 1.0, 0.0};
    const Vector3 z_axis{0.0, 0.0, 1.0};

    CHECK(cross(x_axis, y_axis) == z_axis);
    CHECK(cross(y_axis, x_axis) == -z_axis);
    CHECK(cross(x_axis, x_axis) == Vector3{0.0, 0.0, 0.0});

    const Vector3 a{1.0, 2.0, 3.0};
    const Vector3 b{4.0, 5.0, 6.0};
    CHECK(cross(a, b) == -cross(b, a));

    // 上面这些断言全是输出的线性性质，任何固定线性映射都能满足它们 ——
    // 反交换性对任意 S 都成立（S(b×a) = -S(a×b)），零结果对线性映射也
    // 仍是零。因此形如 diag(s1, s2, 1)·(a×b) 的实现能通过全部断言，
    // 包括 s1 = s2 = 0（x、y 分量恒为零）。下面用三个轴的循环和一个
    // 一般对，把每个分量各自钉死。
    CHECK(cross(y_axis, z_axis) == x_axis);
    CHECK(cross(z_axis, x_axis) == y_axis);
    CHECK(cross(a, b) == Vector3{-3.0, 6.0, -3.0});
}

TEST_CASE("cross of parallel vectors is the zero vector", "[linear][vector3]") {
    const Vector3 a{1.0, 2.0, 3.0};
    const Vector3 b{2.0, 4.0, 6.0};
    CHECK(cross(a, b) == Vector3{0.0, 0.0, 0.0});
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
