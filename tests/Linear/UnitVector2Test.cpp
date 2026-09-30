#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <limits>
#include <type_traits>

#include <DragonGeo/Linear/UnitVector2.hpp>

using Catch::Approx;

using DragonGeo::Linear::UnitVector2T;
using DragonGeo::Linear::UnitVector2f;
using DragonGeo::Linear::Vector2;

TEST_CASE("the float alias really is the float instantiation",
          "[linear][unitvector2]") {
    // 同 vector4Test.cpp：UnitVector2f 此前零命中，误绑定不会被断言发现。
    STATIC_REQUIRE(std::is_same_v<UnitVector2f, UnitVector2T<float>>);
}

TEST_CASE("normalize produces a unit-length 2D vector", "[linear][unitvector2]") {
    const auto result = Vector2{3.0, 4.0}.Normalized();

    REQUIRE(result.has_value());
    CHECK(result->X() == Approx(0.6));
    CHECK(result->Y() == Approx(0.8));
    CHECK(result->AsVector().Length() == Approx(1.0));
}

TEST_CASE("normalize rejects the 2D zero vector", "[linear][unitvector2][degenerate]") {
    CHECK_FALSE(Vector2{0.0, 0.0}.Normalized().has_value());
}

TEST_CASE("2D cross of unit vectors is the sine of the angle",
          "[linear][unitvector2]") {
    const auto x = Vector2{1.0, 0.0}.Normalized();
    const auto y = Vector2{0.0, 1.0}.Normalized();
    REQUIRE(x.has_value());
    REQUIRE(y.has_value());

    CHECK(x->Cross(*y) == Approx(1.0));
    CHECK(y->Cross(*x) == Approx(-1.0));
    CHECK(x->Cross(*x) == Approx(0.0));
}

TEST_CASE("normalize rejects non-finite input instead of returning a NaN unit vector",
          "[linear][unitvector2][degenerate]") {
    const double infinity = std::numeric_limits<double>::infinity();
    const double notANumber = std::numeric_limits<double>::quiet_NaN();

    // 理由同 UnitVector3T：交出一个内容为 NaN 的「单位向量」比返回 nullopt 危险得多。
    CHECK_FALSE(Vector2{infinity, 1.0}.Normalized().has_value());
    CHECK_FALSE(Vector2{1.0, infinity}.Normalized().has_value());
    CHECK_FALSE(Vector2{infinity, infinity}.Normalized().has_value());

    CHECK_FALSE(Vector2{notANumber, 1.0}.Normalized().has_value());
    CHECK_FALSE(Vector2{notANumber, notANumber}.Normalized().has_value());
}
