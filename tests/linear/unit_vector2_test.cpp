#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>

#include <GeoCore/linear/UnitVector2.hpp>

using Catch::Approx;

using GeoCore::linear::normalize;
using GeoCore::linear::Vector2;

TEST_CASE("normalize produces a unit-length 2D vector", "[linear][unitvector2]") {
    const auto result = normalize(Vector2{3.0, 4.0});

    REQUIRE(result.has_value());
    CHECK(result->x() == Approx(0.6));
    CHECK(result->y() == Approx(0.8));
    CHECK(result->as_vector().length() == Approx(1.0));
}

TEST_CASE("normalize rejects the 2D zero vector", "[linear][unitvector2][degenerate]") {
    CHECK_FALSE(normalize(Vector2{0.0, 0.0}).has_value());
}

TEST_CASE("2D cross of unit vectors is the sine of the angle",
          "[linear][unitvector2]") {
    const auto x = normalize(Vector2{1.0, 0.0});
    const auto y = normalize(Vector2{0.0, 1.0});
    REQUIRE(x.has_value());
    REQUIRE(y.has_value());

    CHECK(cross(*x, *y) == Approx(1.0));
    CHECK(cross(*y, *x) == Approx(-1.0));
    CHECK(cross(*x, *x) == Approx(0.0));
}
