#include <catch2/catch_test_macros.hpp>

#include <GeoCore/core/Tolerance.hpp>

using GeoCore::core::Tolerance;

TEST_CASE("Tolerance::resolve grows with magnitude", "[core][tolerance]") {
    const Tolerance tolerance{1e-12, 1e-9};

    // 量级远小于绝对项时，有效容差就是绝对项
    CHECK(tolerance.resolve(0.0) == 1e-12);

    // 量级足够大时，相对项主导
    CHECK(tolerance.resolve(1.0) == 1e-12 + 1e-9);
    CHECK(tolerance.resolve(1e6) == 1e-12 + 1e-9 * 1e6);
}

TEST_CASE("Tolerance::equal respects scale", "[core][tolerance]") {
    const Tolerance tolerance{1e-12, 1e-9};

    CHECK(tolerance.equal(1.0, 1.0));
    CHECK(tolerance.equal(1.0, 1.0 + 1e-12));
    CHECK_FALSE(tolerance.equal(1.0, 1.0 + 1e-6));

    // 大尺度下，1e-3 的差异在相对容差之内
    CHECK(tolerance.equal(1e7, 1e7 + 1e-3));
    // 小尺度下，同样的绝对差异过大
    CHECK_FALSE(tolerance.equal(1e-6, 1e-6 + 1e-3));
}

TEST_CASE("Tolerance::is_zero rejects only genuinely tiny values",
          "[core][tolerance]") {
    const Tolerance tolerance{1e-12, 1e-9};

    CHECK(tolerance.is_zero(0.0));
    CHECK(tolerance.is_zero(1e-15));
    CHECK_FALSE(tolerance.is_zero(1e-6));
    CHECK_FALSE(tolerance.is_zero(-1.0));
}

TEST_CASE("a zero tolerance rejects only exact zero", "[core][tolerance]") {
    const Tolerance exact{0.0, 0.0};

    CHECK(exact.is_zero(0.0));
    CHECK_FALSE(exact.is_zero(1e-300));
}
