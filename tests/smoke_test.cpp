#include <catch2/catch_test_macros.hpp>

#include <GeoCore/GeoCore.hpp>

TEST_CASE("GeoCore headers can be included and version macros are defined", "[smoke]") {
    STATIC_REQUIRE(GEOCORE_VERSION_MAJOR >= 0);
    STATIC_REQUIRE(GEOCORE_VERSION_MINOR >= 1);
    SUCCEED("GeoCore.hpp included successfully");
}
