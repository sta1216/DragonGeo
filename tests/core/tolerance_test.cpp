#include <catch2/catch_test_macros.hpp>

#include <limits>

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

TEST_CASE("Tolerance::resolve is symmetric in the sign of the magnitude",
          "[core][tolerance]") {
    const Tolerance tolerance{1e-12, 1e-9};

    // 量级一律取绝对值，符号不应有任何影响。
    //
    // 这条断言专门盯住一个不写绝对值就完全无法察觉的回归：若 resolve 误写成
    // `abs + rel * magnitude`，正量级的用例会全部照过（文件里其余量级都是正数），
    // 只有负量级会得到一个更小的容差。这是该公式最可能的一次写错。
    CHECK(tolerance.resolve(-1e6) == tolerance.resolve(1e6));
    CHECK(tolerance.resolve(-1.0) == tolerance.resolve(1.0));
    CHECK(tolerance.resolve(-1e6) > tolerance.resolve(0.0));
}

TEST_CASE("non-finite values are neither zero nor approximately equal",
          "[core][tolerance][degenerate]") {
    const Tolerance tolerance{1e-12, 1e-9};
    const double infinity = std::numeric_limits<double>::infinity();
    const double not_a_number = std::numeric_limits<double>::quiet_NaN();

    // 溢出成无穷大的量绝不能被判为零 —— 它恰恰是调用者最该被示警的情形
    CHECK_FALSE(tolerance.is_zero(infinity));
    CHECK_FALSE(tolerance.is_zero(-infinity));

    // 无穷大不等于任何有限值
    CHECK_FALSE(tolerance.equal(infinity, 5.0));
    CHECK_FALSE(tolerance.equal(infinity, 1e300));
    CHECK_FALSE(tolerance.equal(-infinity, 1e300));

    // 但「完全相等」仍是相等，无穷大也不例外
    CHECK(tolerance.equal(infinity, infinity));

    // NaN 不等于任何东西，包括它自己
    CHECK_FALSE(tolerance.equal(not_a_number, not_a_number));
    CHECK_FALSE(tolerance.is_zero(not_a_number));
}
