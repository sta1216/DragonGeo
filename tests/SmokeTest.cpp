#include <catch2/catch_test_macros.hpp>

#include <DragonGeo/DragonGeo.hpp>

TEST_CASE("DragonGeo headers can be included and version macros are defined", "[smoke]") {
    STATIC_REQUIRE(DRAGONGEO_VERSION_MAJOR >= 0);
    STATIC_REQUIRE(DRAGONGEO_VERSION_MINOR >= 1);
    SUCCEED("DragonGeo.hpp included successfully");
}
