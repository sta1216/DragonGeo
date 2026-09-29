#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <limits>
#include <optional>
#include <type_traits>

#include <GeoCore/linear/UnitVector3.hpp>

using Catch::Approx;

using GeoCore::core::Tolerance;
using GeoCore::linear::normalize;
using GeoCore::linear::UnitVector3;
using GeoCore::linear::Vector3;

TEST_CASE("normalize produces a unit-length vector", "[linear][unitvector3]") {
    const auto result = normalize(Vector3{3.0, 4.0, 0.0});

    REQUIRE(result.has_value());
    CHECK(result->x() == Approx(0.6));
    CHECK(result->y() == Approx(0.8));
    CHECK(result->z() == Approx(0.0));
    CHECK(result->as_vector().length() == Approx(1.0));
}

TEST_CASE("normalize rejects the zero vector", "[linear][unitvector3][degenerate]") {
    const auto result = normalize(Vector3{0.0, 0.0, 0.0});

    // 必须返回 nullopt，而不是含 NaN 的单位向量
    CHECK_FALSE(result.has_value());
}

TEST_CASE("normalize rejects vectors below the tolerance",
          "[linear][unitvector3][degenerate]") {
    CHECK_FALSE(normalize(Vector3{1e-15, 0.0, 0.0}).has_value());
}

TEST_CASE("a zero tolerance rejects only the exact zero vector",
          "[linear][unitvector3][degenerate]") {
    const Tolerance exact{0.0, 0.0};

    CHECK_FALSE(normalize(Vector3{0.0, 0.0, 0.0}, exact).has_value());
    CHECK(normalize(Vector3{1e-300, 0.0, 0.0}, exact).has_value());
}

TEST_CASE("normalize rejects vectors below the absolute tolerance",
          "[linear][unitvector3][degenerate]") {
    // 1e-200 的长度远小于默认绝对容差 1e-12，因此按「视为零」处理。
    // 需要在这个量级上工作时，必须显式传入更小的容差 —— 这正是
    // 容差显式传参原则的预期后果。
    CHECK_FALSE(normalize(Vector3{1e-200, 0.0, 0.0}).has_value());
}

TEST_CASE("normalize scales correctly at extreme magnitudes",
          "[linear][unitvector3][degenerate]") {
    // 关闭容差判断，单独考察重缩放后的计算本身是否上溢或下溢
    const Tolerance exact{0.0, 0.0};

    const auto huge = normalize(Vector3{1e200, 1e200, 0.0}, exact);
    REQUIRE(huge.has_value());
    CHECK(huge->x() == Approx(0.7071067811865476));
    CHECK(huge->y() == Approx(0.7071067811865476));

    const auto tiny = normalize(Vector3{1e-200, 1e-200, 0.0}, exact);
    REQUIRE(tiny.has_value());
    CHECK(tiny->x() == Approx(0.7071067811865476));
    CHECK(tiny->y() == Approx(0.7071067811865476));
}

TEST_CASE("negation preserves the unit invariant", "[linear][unitvector3]") {
    const auto u = normalize(Vector3{1.0, 2.0, 2.0});
    REQUIRE(u.has_value());

    const UnitVector3 negated = -*u;
    CHECK(negated.x() == Approx(-1.0 / 3.0));
    CHECK(negated.y() == Approx(-2.0 / 3.0));
    CHECK(negated.z() == Approx(-2.0 / 3.0));
}

TEST_CASE("scaling a unit vector yields a plain Vector3",
          "[linear][unitvector3]") {
    const auto u = normalize(Vector3{0.0, 0.0, 1.0});
    REQUIRE(u.has_value());

    // 关键的类型契约：缩放后不再是单位向量，返回类型必须改变
    STATIC_REQUIRE(std::is_same_v<decltype(*u * 2.0), Vector3>);
    CHECK(*u * 2.0 == Vector3{0.0, 0.0, 2.0});
    CHECK(2.0 * *u == Vector3{0.0, 0.0, 2.0});
}

TEST_CASE("sums and differences of unit vectors are plain Vector3",
          "[linear][unitvector3]") {
    const auto x = normalize(Vector3{1.0, 0.0, 0.0});
    const auto z = normalize(Vector3{0.0, 0.0, 1.0});
    REQUIRE(x.has_value());
    REQUIRE(z.has_value());

    // 规范 §4.4 点名了 u + u 与 u - u：和与差一般都不是单位向量，返回类型
    // 必须如实反映这一点，否则不变量会被静默破坏。
    STATIC_REQUIRE(std::is_same_v<decltype(*x + *z), Vector3>);
    STATIC_REQUIRE(std::is_same_v<decltype(*x - *z), Vector3>);

    CHECK(*x + *z == Vector3{1.0, 0.0, 1.0});
    CHECK(*x - *z == Vector3{1.0, 0.0, -1.0});
}

TEST_CASE("dot of two unit vectors is the cosine of the angle",
          "[linear][unitvector3]") {
    const auto a = normalize(Vector3{1.0, 0.0, 0.0});
    const auto b = normalize(Vector3{1.0, 1.0, 0.0});
    REQUIRE(a.has_value());
    REQUIRE(b.has_value());

    CHECK(dot(*a, *b) == Approx(0.7071067811865476));
    CHECK(dot(*a, *a) == Approx(1.0));
}

TEST_CASE("cross of two unit vectors is a plain Vector3",
          "[linear][unitvector3]") {
    const auto x = normalize(Vector3{1.0, 0.0, 0.0});
    const auto y = normalize(Vector3{0.0, 1.0, 0.0});
    REQUIRE(x.has_value());
    REQUIRE(y.has_value());

    // 数学上叉积仍是单位向量，但两向量接近平行时长度会退化到 0，
    // 无法维持不变量，故返回类型刻意是 Vector3。
    STATIC_REQUIRE(std::is_same_v<decltype(cross(*x, *y)), Vector3>);
    CHECK(cross(*x, *y).z == Approx(1.0));

    const auto parallel = normalize(Vector3{1.0, 0.0, 0.0});
    REQUIRE(parallel.has_value());
    CHECK(cross(*x, *parallel).length() == Approx(0.0));
}

TEST_CASE("normalize rejects non-finite input instead of returning a NaN unit vector",
          "[linear][unitvector3][degenerate]") {
    const double infinity = std::numeric_limits<double>::infinity();
    const double not_a_number = std::numeric_limits<double>::quiet_NaN();

    // 一个 has_value() 为真、内容却是 NaN 的「单位向量」是本库最不该交出的
    // 返回值：调用者无从察觉，而 NaN 会一路污染 dot / cross 与所有容差判断。
    CHECK_FALSE(normalize(Vector3{infinity, 1.0, 1.0}).has_value());
    CHECK_FALSE(normalize(Vector3{1.0, infinity, 1.0}).has_value());
    CHECK_FALSE(normalize(Vector3{infinity, infinity, infinity}).has_value());

    // NaN 输入比无穷更常见：任何上游的 0/0 或 inf - inf 都会落到这里
    CHECK_FALSE(normalize(Vector3{not_a_number, 1.0, 1.0}).has_value());
    CHECK_FALSE(normalize(Vector3{not_a_number, not_a_number, not_a_number}).has_value());
}
