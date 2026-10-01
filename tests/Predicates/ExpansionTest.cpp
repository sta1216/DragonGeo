#include <catch2/catch_test_macros.hpp>

#include <cmath>

#include <Predicates/Expansion.hpp>

using DragonGeo::Predicates::Detail::ScaleShift;
using DragonGeo::Predicates::Detail::Sign;
using DragonGeo::Predicates::Detail::TwoProduct;
using DragonGeo::Predicates::Detail::TwoSum;

TEST_CASE("TwoSum keeps the roundoff of a half-ulp add", "[predicates][expansion]") {
    const auto sum = TwoSum(1.0, std::ldexp(1.0, -53));
    CHECK(sum.Hi == 1.0);
    CHECK(sum.Lo == std::ldexp(1.0, -53));
}

TEST_CASE("TwoProduct keeps the square of one plus one ulp", "[predicates][expansion]") {
    const double u = 1.0 + std::ldexp(1.0, -52);
    const auto product = TwoProduct(u, u);
    CHECK(product.Hi == 1.0 + std::ldexp(1.0, -51));
    CHECK(product.Lo == std::ldexp(1.0, -104));
}

TEST_CASE("Sign reads the leading non-zero component", "[predicates][expansion]") {
    CHECK(Sign({0.0, -2.0}) == -1);
    CHECK(Sign({3.0, 0.0}) == 1);
    CHECK(Sign({0.0, 0.0}) == 0);
    CHECK(Sign({}) == 0);
}

TEST_CASE("ScaleShift brings the max absolute value into the unit binade", "[predicates][expansion]") {
    CHECK(ScaleShift(0.0) == 0);
    CHECK(ScaleShift(1.0) == 0);
    CHECK(ScaleShift(3.0) == -1);
    CHECK(std::ldexp(3.0, ScaleShift(3.0)) == 1.5);
}
