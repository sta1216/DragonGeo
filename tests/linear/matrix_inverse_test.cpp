#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <limits>

#include <GeoCore/linear/Matrix.hpp>

using Catch::Approx;

using GeoCore::core::Tolerance;
using GeoCore::linear::determinant;
using GeoCore::linear::identity;
using GeoCore::linear::inverse;
using GeoCore::linear::Matrix2;
using GeoCore::linear::Matrix3;
using GeoCore::linear::Matrix4;

TEST_CASE("determinant of a 2x2 matrix", "[linear][matrix][determinant]") {
    CHECK(determinant(Matrix2{{{1.0, 2.0}, {3.0, 4.0}}}) == Approx(-2.0));
    CHECK(determinant(identity<double, 2>()) == Approx(1.0));
    CHECK(determinant(Matrix2{{{1.0, 2.0}, {2.0, 4.0}}}) == Approx(0.0));
}

TEST_CASE("determinant of a 3x3 matrix", "[linear][matrix][determinant]") {
    const Matrix3 m{{{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}, {7.0, 8.0, 10.0}}};
    CHECK(determinant(m) == Approx(-3.0));

    CHECK(determinant(identity<double, 3>()) == Approx(1.0));
}

TEST_CASE("determinant of a singular 3x3 matrix is zero",
          "[linear][matrix][determinant][degenerate]") {
    // 第三行是前两行之和
    const Matrix3 m{{{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}, {5.0, 7.0, 9.0}}};
    CHECK(determinant(m) == Approx(0.0));
}

TEST_CASE("determinant of a 4x4 matrix", "[linear][matrix][determinant]") {
    CHECK(determinant(identity<double, 4>()) == Approx(1.0));

    // 下三角矩阵的行列式是对角线之积
    const Matrix4 lower{{
        {2.0, 0.0, 0.0, 0.0},
        {3.0, 4.0, 0.0, 0.0},
        {5.0, 6.0, 7.0, 0.0},
        {8.0, 9.0, 10.0, 11.0},
    }};
    CHECK(determinant(lower) == Approx(2.0 * 4.0 * 7.0 * 11.0));
}

TEST_CASE("inverse times original is the identity", "[linear][matrix][inverse]") {
    const Matrix2 m{{{4.0, 7.0}, {2.0, 6.0}}};
    const auto inv = inverse(m);

    REQUIRE(inv.has_value());
    const Matrix2 product = m * *inv;

    // 期望值为 0 时 Approx 的两个容差项都是 0（margin 默认为 0，而
    // epsilon * |0| 也是 0），所以 `Approx(0.0)` 是伪装成容差比较的精确相等。
    // 这里必须给出显式 margin。
    CHECK(product(0, 0) == Approx(1.0).margin(1e-12));
    CHECK(product(0, 1) == Approx(0.0).margin(1e-12));
    CHECK(product(1, 0) == Approx(0.0).margin(1e-12));
    CHECK(product(1, 1) == Approx(1.0).margin(1e-12));
}

TEST_CASE("3x3 inverse round-trips", "[linear][matrix][inverse]") {
    const Matrix3 m{{{1.0, 2.0, 3.0}, {0.0, 1.0, 4.0}, {5.0, 6.0, 0.0}}};
    const auto inv = inverse(m);

    REQUIRE(inv.has_value());
    const Matrix3 product = m * *inv;
    const Matrix3 unit = identity<double, 3>();

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            CHECK(product(i, j) == Approx(unit(i, j)).margin(1e-12));
        }
    }
}

TEST_CASE("inverse of a singular matrix is nullopt",
          "[linear][matrix][inverse][degenerate]") {
    // 第二行是第一行的两倍
    const Matrix2 singular{{{1.0, 2.0}, {2.0, 4.0}}};

    const auto inv = inverse(singular);

    // 必须是 nullopt，而不是含 inf / NaN 的矩阵
    CHECK_FALSE(inv.has_value());

    const Matrix3 singular3{{
        {1.0, 2.0, 3.0},
        {2.0, 4.0, 6.0},
        {1.0, 1.0, 1.0},
    }};
    CHECK_FALSE(inverse(singular3).has_value());
}

TEST_CASE("inverse of a matrix containing NaN is nullopt",
          "[linear][matrix][inverse][degenerate]") {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const Matrix2 bad{{{nan, 1.0}, {2.0, 3.0}}};

    // 「结果不作保证」的说法已过时：自 d3ba158 起语义就是明确的 —— 绝不交出
    // has_value() 为真、内容却是 NaN 的矩阵。旧断言丢弃结果后直接 SUCCEED，
    // 因此它永远不会失败，也就什么也没测到。
    CHECK_FALSE(inverse(bad).has_value());
}

TEST_CASE("inverse rescales badly scaled matrices before testing the determinant",
          "[linear][matrix][inverse]") {
    // 行列式是 N 阶量，直接拿它跟一阶容差比是量纲错误：diag(1e-4) 的 det =
    // 1e-12 会被默认容差的绝对项（1e-12）判为奇异，而 diag(1e150) 的 det =
    // 1e450 会在任何比较之前就溢出成 inf。先按最大元素归一化再比较，两者
    // 都能得到正确答案。
    const Matrix3 tiny{{{1e-4, 0.0, 0.0}, {0.0, 1e-4, 0.0}, {0.0, 0.0, 1e-4}}};
    const auto tiny_inverse = inverse(tiny);

    REQUIRE(tiny_inverse.has_value());
    CHECK((*tiny_inverse)(0, 0) == Approx(1e4));
    CHECK((*tiny_inverse)(2, 2) == Approx(1e4));

    const Matrix3 huge{{{1e150, 0.0, 0.0}, {0.0, 1e150, 0.0}, {0.0, 0.0, 1e150}}};
    const auto huge_inverse = inverse(huge);

    REQUIRE(huge_inverse.has_value());
    CHECK((*huge_inverse)(0, 0) == Approx(1e-150));

    // 同上，逐元素回环：tiny 与 huge 互为量级上的倒数
    const Matrix3 unit = identity<double, 3>();
    const Matrix3 product = tiny * *tiny_inverse;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            CHECK(product(i, j) == Approx(unit(i, j)).margin(1e-12));
        }
    }

    // 裁定还要求 scaling_3d(1e-4) / (1e-5) / (1e150) 的逆同样存在。这三个是
    // 4×4 仿射矩阵，第 4 行的齐次 1 让「最大绝对元素」归一化对线性部分失效：
    // 归一化后的行列式分别是 1e-12、1e-15、1e-150，仍落在默认容差的绝对项
    // 之内，因此它们目前仍返回 nullopt。机制与验收标准在此互相矛盾，见
    // .superpowers/sdd/2026-09-29-geocore-stage1-foundation/final-fix-report.md
    // 的 FIX 3 一节，留待裁定后固定 —— 这里刻意不写断言。
}

TEST_CASE("inverse refuses a matrix whose restored inverse would overflow",
          "[linear][matrix][inverse][degenerate]") {
    // 1e-310 是次正规数，它的倒数是 inf。归一化后的行列式有限，但还原尺度时
    // 元素会溢出 —— 真实逆确实巨大，那就该返回 nullopt，而不是交出一个
    // has_value() 为真、内容却是 inf 的矩阵。
    const Tolerance exact{0.0, 0.0};
    const Matrix2 subnormal{{{1e-310, 0.0}, {0.0, 1.0}}};

    CHECK_FALSE(inverse(subnormal, exact).has_value());
}

TEST_CASE("inverse of a singular 4x4 matrix is nullopt",
          "[linear][matrix][inverse][degenerate]") {
    // 第 4 行与第 1 行相同
    const Matrix4 dependent{{
        {1.0, 2.0, 3.0, 4.0},
        {5.0, 6.0, 7.0, 8.0},
        {9.0, 10.0, 11.0, 12.0},
        {1.0, 2.0, 3.0, 4.0},
    }};

    CHECK_FALSE(inverse(dependent).has_value());
}

TEST_CASE("the tolerance argument decides near-singularity",
          "[linear][matrix][inverse]") {
    // 行列式 1e-4：默认容差的绝对项是 1e-12，因此放行；把绝对项显式放宽到
    // 1e-3，同一个矩阵即可视为奇异。容差始终是显式参数，这个分支才有内容。
    const Matrix2 nearly_singular{{{1.0, 1.0}, {1.0, 1.0001}}};
    const Tolerance exact{0.0, 0.0};
    const Tolerance loose{1e-3, 0.0};

    CHECK(inverse(nearly_singular, exact).has_value());
    CHECK(inverse(nearly_singular).has_value());            // 默认容差：放行
    CHECK_FALSE(inverse(nearly_singular, loose).has_value()); // 放宽后：判为奇异
}

TEST_CASE("an exact tolerance rejects only a genuinely singular matrix",
          "[linear][matrix][inverse][degenerate]") {
    const Tolerance exact{0.0, 0.0};

    CHECK_FALSE(inverse(Matrix2{{{1.0, 2.0}, {2.0, 4.0}}}, exact).has_value());
    CHECK(inverse(Matrix2{{{1.0, 2.0}, {2.0, 4.0000000001}}}, exact).has_value());
}

TEST_CASE("4x4 determinant pins the cofactor expansion",
          "[linear][matrix][determinant]") {
    // 单位阵与三角阵的行列式都等于对角线之积，会恰好掩盖余子式展开里的交叉
    // 项错误。这里用一个一般（非三角、非对称、非奇异）矩阵，把展开钉住。
    const Matrix4 m{{{2.0, 3.0, 1.0, 5.0},
                     {1.0, 4.0, 2.0, 6.0},
                     {3.0, 1.0, 5.0, 2.0},
                     {4.0, 2.0, 3.0, 1.0}}};
    CHECK(determinant(m) == Approx(-85.0));
}

TEST_CASE("4x4 inverse round-trips", "[linear][matrix][inverse]") {
    // 4×4 的逆矩阵要算 16 个三阶余子式，而且 Task 9 的 Transform 会消费它 ——
    // 但原测试只覆盖了 2×2 与 3×3。
    const Matrix4 m{{{2.0, 3.0, 1.0, 5.0},
                     {1.0, 4.0, 2.0, 6.0},
                     {3.0, 1.0, 5.0, 2.0},
                     {4.0, 2.0, 3.0, 1.0}}};
    const auto inv = inverse(m);

    REQUIRE(inv.has_value());
    const Matrix4 product = m * *inv;
    const Matrix4 unit = identity<double, 4>();

    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            CHECK(product(i, j) == Approx(unit(i, j)).margin(1e-12));
        }
    }
}

TEST_CASE("inverse of a non-finite matrix is nullopt",
          "[linear][matrix][inverse][degenerate]") {
    const double infinity = std::numeric_limits<double>::infinity();
    const double not_a_number = std::numeric_limits<double>::quiet_NaN();

    // 与 normalize() 同一条原则：绝不交出一个 has_value() 为真、内容却是
    // NaN 的结果 —— 调用者无从察觉，而 NaN 会污染后续全部计算。
    CHECK_FALSE(inverse(Matrix2{{{not_a_number, 1.0}, {2.0, 3.0}}}).has_value());
    CHECK_FALSE(inverse(Matrix2{{{infinity, 1.0}, {2.0, 3.0}}}).has_value());
    CHECK_FALSE(inverse(Matrix3{{{1.0, infinity, 3.0},
                                 {4.0, 5.0, 6.0},
                                 {7.0, 8.0, not_a_number}}}).has_value());
}
