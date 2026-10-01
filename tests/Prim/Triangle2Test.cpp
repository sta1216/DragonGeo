#include <catch2/catch_test_macros.hpp>

#include <limits>
#include <type_traits>

#include <DragonGeo/Prim/Triangle2.hpp>

using DragonGeo::Linear::Point2;
using DragonGeo::Prim::Triangle2;
using DragonGeo::Prim::Triangle2T;
using DragonGeo::Prim::Triangle2f;

TEST_CASE("Triangle2 is an aggregate of three points", "[prim][triangle2]") {
    const Triangle2 triangle{Point2{0.0, 0.0}, Point2{1.0, 0.0}, Point2{0.0, 1.0}};
    CHECK(triangle.A == Point2{0.0, 0.0});
    CHECK(triangle.B == Point2{1.0, 0.0});
    CHECK(triangle.C == Point2{0.0, 1.0});
    CHECK(triangle.IsValid());
    CHECK(triangle == Triangle2{Point2{0.0, 0.0}, Point2{1.0, 0.0}, Point2{0.0, 1.0}});
    CHECK(triangle != Triangle2{Point2{9.0, 0.0}, Point2{1.0, 0.0}, Point2{0.0, 1.0}});
    CHECK(triangle != Triangle2{Point2{0.0, 0.0}, Point2{9.0, 0.0}, Point2{0.0, 1.0}});
    CHECK(triangle != Triangle2{Point2{0.0, 0.0}, Point2{1.0, 0.0}, Point2{9.0, 0.0}});
    STATIC_REQUIRE(std::is_aggregate_v<Triangle2>);
    STATIC_REQUIRE(std::is_same_v<Triangle2f, Triangle2T<float>>);
}

TEST_CASE("a zero-area Triangle2 is valid", "[prim][triangle2]") {
    const Triangle2 triangle{Point2{0.0, 0.0}, Point2{1.0, 0.0}, Point2{2.0, 0.0}};
    CHECK(triangle.IsValid());
}

TEST_CASE("Triangle2 with a non-finite coordinate is invalid", "[prim][triangle2]") {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const Triangle2 triangle{Point2{nan, 0.0}, Point2{1.0, 0.0}, Point2{0.0, 1.0}};
    CHECK_FALSE(triangle.IsValid());
}
