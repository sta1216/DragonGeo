#include <catch2/catch_test_macros.hpp>

#include <limits>

#include <GeoCore/core/Numeric.hpp>

using GeoCore::core::absolute_value;
using GeoCore::core::clamp;
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
