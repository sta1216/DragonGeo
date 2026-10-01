#include <catch2/catch_test_macros.hpp>

#include <limits>
#include <type_traits>

#include <DragonGeo/Prim/Triangle3.hpp>

using DragonGeo::Linear::Point3;
using DragonGeo::Prim::Triangle3;
using DragonGeo::Prim::Triangle3T;
using DragonGeo::Prim::Triangle3f;

TEST_CASE("Triangle3 is an aggregate of three points", "[prim][triangle3]") {
    const Triangle3 triangle{
        Point3{0.0, 0.0, 0.0}, Point3{1.0, 0.0, 0.0}, Point3{0.0, 1.0, 0.0}};
    CHECK(triangle.A == Point3{0.0, 0.0, 0.0});
    CHECK(triangle.B == Point3{1.0, 0.0, 0.0});
    CHECK(triangle.C == Point3{0.0, 1.0, 0.0});
    CHECK(triangle.IsValid());
    CHECK(triangle == Triangle3{Point3{0.0, 0.0, 0.0}, Point3{1.0, 0.0, 0.0}, Point3{0.0, 1.0, 0.0}});
    CHECK(triangle != Triangle3{Point3{9.0, 0.0, 0.0}, Point3{1.0, 0.0, 0.0}, Point3{0.0, 1.0, 0.0}});
    CHECK(triangle != Triangle3{Point3{0.0, 0.0, 0.0}, Point3{9.0, 0.0, 0.0}, Point3{0.0, 1.0, 0.0}});
    CHECK(triangle != Triangle3{Point3{0.0, 0.0, 0.0}, Point3{1.0, 0.0, 0.0}, Point3{0.0, 9.0, 0.0}});
    STATIC_REQUIRE(std::is_aggregate_v<Triangle3>);
    STATIC_REQUIRE(std::is_same_v<Triangle3f, Triangle3T<float>>);
}

TEST_CASE("a zero-area Triangle3 is valid", "[prim][triangle3]") {
    const Triangle3 triangle{
        Point3{0.0, 0.0, 0.0}, Point3{1.0, 0.0, 0.0}, Point3{2.0, 0.0, 0.0}};
    CHECK(triangle.IsValid());
}

TEST_CASE("Triangle3 with a non-finite coordinate is invalid", "[prim][triangle3]") {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const Triangle3 triangle{
        Point3{0.0, 0.0, nan}, Point3{1.0, 0.0, 0.0}, Point3{0.0, 1.0, 0.0}};
    CHECK_FALSE(triangle.IsValid());
}
