#include <catch2/catch_test_macros.hpp>

#include <limits>

#include <GeoCore/core/Numeric.hpp>

using GeoCore::core::absolute_value;
using GeoCore::core::clamp;
using GeoCore::core::is_finite;
using GeoCore::core::max_abs_of;
using GeoCore::core::safe_sqrt;

TEST_CASE("absolute_value returns the magnitude", "[core][numeric]") {
    CHECK(absolute_value(-3.0) == 3.0);
    CHECK(absolute_value(3.0) == 3.0);
    CHECK(absolute_value(0.0) == 0.0);
}

TEST_CASE("absolute_value propagates NaN rather than returning garbage",
          "[core][numeric][degenerate]") {
    CHECK(std::isnan(absolute_value(std::numeric_limits<double>::quiet_NaN())));
}

TEST_CASE("clamp bounds the value on both sides", "[core][numeric]") {
    CHECK(clamp(-1.0, 0.0, 1.0) == 0.0);
    CHECK(clamp(0.5, 0.0, 1.0) == 0.5);
    CHECK(clamp(2.0, 0.0, 1.0) == 1.0);
}

TEST_CASE("safe_sqrt maps negative input to zero", "[core][numeric][degenerate]") {
    CHECK(safe_sqrt(4.0) == 2.0);
    CHECK(safe_sqrt(0.0) == 0.0);
    // Rounding can push a squared length a hair below zero; that must not
    // produce NaN.
    CHECK(safe_sqrt(-1e-18) == 0.0);
}

TEST_CASE("is_finite is usable in a constant expression", "[core][numeric]") {
    // 手写 is_finite 的唯一理由就是 std::isfinite 在 C++20 尚不是 constexpr
    // （见 Numeric.hpp 的注释），而这条约束此前没有任何测试固定：把它换回
    // std::isfinite 时，只有那些在常量表达式中调用它的源码才会报错。
    constexpr double finite_value = 1.0;
    constexpr double infinity = std::numeric_limits<double>::infinity();
    constexpr double not_a_number = std::numeric_limits<double>::quiet_NaN();

    STATIC_REQUIRE(is_finite(finite_value));
    STATIC_REQUIRE_FALSE(is_finite(infinity));
    STATIC_REQUIRE_FALSE(is_finite(-infinity));
    STATIC_REQUIRE_FALSE(is_finite(not_a_number));
}

TEST_CASE("max_abs_of picks the largest magnitude", "[core][numeric]") {
    CHECK(max_abs_of(3.0, -4.0) == 4.0);
    CHECK(max_abs_of(-4.0, 3.0) == 4.0);
    CHECK(max_abs_of(3.0, -4.0, 2.0) == 4.0);
    CHECK(max_abs_of(3.0, -4.0, 2.0, -6.0) == 6.0);
    CHECK(max_abs_of(0.0, 0.0, 0.0, 0.0) == 0.0);
    CHECK(max_abs_of(-0.0, -0.0) == 0.0);
}

TEST_CASE("max_abs_of gives a slot- and arity-independent answer on non-finite input",
          "[core][numeric][degenerate]") {
    const double infinity = std::numeric_limits<double>::infinity();
    const double nan = std::numeric_limits<double>::quiet_NaN();

    // 规则：任一 ±inf ⇒ +inf；否则任一 NaN ⇒ NaN；否则最大绝对值。
    // ±inf 必须先于 NaN 判定：两者同时出现时答案是 +inf，而不是 NaN。
    CHECK(max_abs_of(infinity, nan) == infinity);
    CHECK(max_abs_of(nan, infinity) == infinity);
    CHECK(max_abs_of(-infinity, nan) == infinity);
    CHECK(max_abs_of(nan, -infinity) == infinity);
    CHECK(max_abs_of(nan, infinity, 1.0) == infinity);
    CHECK(max_abs_of(1.0, nan, infinity) == infinity);
    CHECK(max_abs_of(nan, 1.0, 1.0, infinity) == infinity);
    CHECK(max_abs_of(infinity, nan, 1.0, 1.0) == infinity);

    CHECK(std::isnan(max_abs_of(nan, 5.0)));
    CHECK(std::isnan(max_abs_of(5.0, nan)));
    CHECK(std::isnan(max_abs_of(nan, 5.0, 0.0)));
    CHECK(std::isnan(max_abs_of(5.0, nan, 0.0)));
    CHECK(std::isnan(max_abs_of(0.0, 5.0, nan)));
    CHECK(std::isnan(max_abs_of(nan, 5.0, 0.0, 0.0)));
    CHECK(std::isnan(max_abs_of(5.0, nan, 0.0, 0.0)));
    CHECK(std::isnan(max_abs_of(0.0, 0.0, nan, 5.0)));
    CHECK(std::isnan(max_abs_of(0.0, 0.0, 5.0, nan)));
}
