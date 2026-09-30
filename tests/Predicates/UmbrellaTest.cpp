#include <catch2/catch_test_macros.hpp>

#include <DragonGeo/DragonGeo.hpp>

TEST_CASE("the umbrella header exports Orient2d", "[predicates][orient2d]") {
    CHECK(DragonGeo::Predicates::Orient2d(DragonGeo::Linear::Point2{0.0, 0.0},
                                          DragonGeo::Linear::Point2{1.0, 0.0},
                                          DragonGeo::Linear::Point2{0.0, 1.0}) == 1);
}
