#include <catch2/catch_test_macros.hpp>

#include <numbers>

#include <DragonGeo/DragonGeo.hpp>

using DragonGeo::Core::HALF_PI;
using DragonGeo::Core::PI;
using DragonGeo::Core::QUARTER_PI;
using DragonGeo::Core::SQRT_TWO;
using DragonGeo::Core::TWO_PI;

TEST_CASE("DragonGeo headers can be included and version macros are defined", "[smoke]") {
    STATIC_REQUIRE(DRAGONGEO_VERSION_MAJOR >= 0);
    STATIC_REQUIRE(DRAGONGEO_VERSION_MINOR >= 1);
    SUCCEED("DragonGeo.hpp included successfully");

    STATIC_REQUIRE(PI == std::numbers::pi);
    STATIC_REQUIRE(HALF_PI == PI / 2.0);
    STATIC_REQUIRE(TWO_PI == PI * 2.0);
    STATIC_REQUIRE(QUARTER_PI == PI / 4.0);
    STATIC_REQUIRE(SQRT_TWO == std::numbers::sqrt2);
}
