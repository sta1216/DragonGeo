#include <catch2/catch_test_macros.hpp>

#include <limits>

#include <DragonGeo/Core/Numeric.hpp>

using DragonGeo::Core::AbsoluteValue;
using DragonGeo::Core::Clamp;
using DragonGeo::Core::IsFinite;
using DragonGeo::Core::MaxAbsOf;
using DragonGeo::Core::SafeSqrt;

TEST_CASE("AbsoluteValue returns the magnitude", "[core][numeric]") {
    CHECK(AbsoluteValue(-3.0) == 3.0);
    CHECK(AbsoluteValue(3.0) == 3.0);
    CHECK(AbsoluteValue(0.0) == 0.0);
}

TEST_CASE("AbsoluteValue propagates NaN rather than returning garbage", "[core][numeric][degenerate]") {
    CHECK(std::isnan(AbsoluteValue(std::numeric_limits<double>::quiet_NaN())));
}

TEST_CASE("clamp bounds the value on both sides", "[core][numeric]") {
    CHECK(Clamp(-1.0, 0.0, 1.0) == 0.0);
    CHECK(Clamp(0.5, 0.0, 1.0) == 0.5);
    CHECK(Clamp(2.0, 0.0, 1.0) == 1.0);
}

TEST_CASE("SafeSqrt maps negative input to zero", "[core][numeric][degenerate]") {
    CHECK(SafeSqrt(4.0) == 2.0);
    CHECK(SafeSqrt(0.0) == 0.0);
    CHECK(SafeSqrt(-0.0) == 0.0);
    // 舍入可能让平方长度略小于零；这种情况不得产生 NaN。
    CHECK(SafeSqrt(-1e-18) == 0.0);
    CHECK(SafeSqrt(std::numeric_limits<double>::infinity()) == std::numeric_limits<double>::infinity());
    CHECK(std::isnan(SafeSqrt(std::numeric_limits<double>::quiet_NaN())));
}

TEST_CASE("IsFinite is usable in a constant expression", "[core][numeric]") {
    // 手写 IsFinite 的唯一理由就是 std::isfinite 在 C++20 尚不是 constexpr （见 Numeric.hpp 的注释），而这条约束此前没有任何测试固定：把它换回 std::isfinite 时，只有那些在常量表达式中调用它的源码才会报错。
    constexpr double finiteValue = 1.0;
    constexpr double infinity = std::numeric_limits<double>::infinity();
    constexpr double notANumber = std::numeric_limits<double>::quiet_NaN();

    STATIC_REQUIRE(IsFinite(finiteValue));
    STATIC_REQUIRE_FALSE(IsFinite(infinity));
    STATIC_REQUIRE_FALSE(IsFinite(-infinity));
    STATIC_REQUIRE_FALSE(IsFinite(notANumber));
}

TEST_CASE("MaxAbsOf picks the largest magnitude", "[core][numeric]") {
    CHECK(MaxAbsOf(3.0, -4.0) == 4.0);
    CHECK(MaxAbsOf(-4.0, 3.0) == 4.0);
    CHECK(MaxAbsOf(3.0, -4.0, 2.0) == 4.0);
    CHECK(MaxAbsOf(3.0, -4.0, 2.0, -6.0) == 6.0);
    CHECK(MaxAbsOf(0.0, 0.0, 0.0, 0.0) == 0.0);
    CHECK(MaxAbsOf(-0.0, -0.0) == 0.0);
}

TEST_CASE("MaxAbsOf gives a slot- and arity-independent answer on non-finite input", "[core][numeric][degenerate]") {
    const double infinity = std::numeric_limits<double>::infinity();
    const double nan = std::numeric_limits<double>::quiet_NaN();

    // 规则：任一 ±inf ⇒ +inf；否则任一 NaN ⇒ NaN；否则最大绝对值。 ±inf 必须先于 NaN 判定：两者同时出现时答案是 +inf，而不是 NaN。
    CHECK(MaxAbsOf(infinity, nan) == infinity);
    CHECK(MaxAbsOf(nan, infinity) == infinity);
    CHECK(MaxAbsOf(-infinity, nan) == infinity);
    CHECK(MaxAbsOf(nan, -infinity) == infinity);
    CHECK(MaxAbsOf(nan, infinity, 1.0) == infinity);
    CHECK(MaxAbsOf(1.0, nan, infinity) == infinity);
    CHECK(MaxAbsOf(nan, 1.0, 1.0, infinity) == infinity);
    CHECK(MaxAbsOf(infinity, nan, 1.0, 1.0) == infinity);

    CHECK(std::isnan(MaxAbsOf(nan, 5.0)));
    CHECK(std::isnan(MaxAbsOf(5.0, nan)));
    CHECK(std::isnan(MaxAbsOf(nan, 5.0, 0.0)));
    CHECK(std::isnan(MaxAbsOf(5.0, nan, 0.0)));
    CHECK(std::isnan(MaxAbsOf(0.0, 5.0, nan)));
    CHECK(std::isnan(MaxAbsOf(nan, 5.0, 0.0, 0.0)));
    CHECK(std::isnan(MaxAbsOf(5.0, nan, 0.0, 0.0)));
    CHECK(std::isnan(MaxAbsOf(0.0, 0.0, nan, 5.0)));
    CHECK(std::isnan(MaxAbsOf(0.0, 0.0, 5.0, nan)));
}
