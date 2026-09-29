#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <limits>

#include <GeoCore/core/Constants.hpp>
#include <GeoCore/linear/Quaternion.hpp>

using Catch::Approx;

using GeoCore::core::half_pi;
using GeoCore::core::Tolerance;
using GeoCore::linear::conjugate;
using GeoCore::linear::dot;
using GeoCore::linear::from_axis_angle;
using GeoCore::linear::normalize;
using GeoCore::linear::Quaternion;
using GeoCore::linear::rotate;
using GeoCore::linear::to_matrix;
using GeoCore::linear::UnitVector3;
using GeoCore::linear::Vector3;

namespace {
const UnitVector3 z_axis = UnitVector3::from_normalized_unchecked(Vector3{0.0, 0.0, 1.0});
}

TEST_CASE("default-constructed quaternion is the identity rotation",
          "[linear][quaternion]") {
    const Quaternion q{};

    CHECK(q.w == 1.0);
    CHECK(q.x == 0.0);
    CHECK(q.y == 0.0);
    CHECK(q.z == 0.0);

    // 单位旋转不改变任何向量
    CHECK(rotate(q, Vector3{1.0, 2.0, 3.0}) == Vector3{1.0, 2.0, 3.0});
}

TEST_CASE("axis-angle construction produces a unit quaternion",
          "[linear][quaternion]") {
    const Quaternion q = from_axis_angle(z_axis, half_pi);

    CHECK(q.w == Approx(std::cos(half_pi / 2.0)));
    CHECK(q.z == Approx(std::sin(half_pi / 2.0)));
    CHECK(norm(q) == Approx(1.0));
}

TEST_CASE("rotation about z by 90 degrees maps x to y",
          "[linear][quaternion]") {
    const Quaternion q = from_axis_angle(z_axis, half_pi);

    const Vector3 rotated = rotate(q, Vector3{1.0, 0.0, 0.0});
    CHECK(rotated.x == Approx(0.0).margin(1e-15));
    CHECK(rotated.y == Approx(1.0));
    CHECK(rotated.z == Approx(0.0).margin(1e-15));
}

TEST_CASE("rotation preserves length", "[linear][quaternion]") {
    const Quaternion q = from_axis_angle(z_axis, 0.7);
    const Vector3 v{3.0, 4.0, 0.0};

    CHECK(rotate(q, v).length() == Approx(v.length()));
}

TEST_CASE("conjugate undoes the rotation", "[linear][quaternion]") {
    const Quaternion q = from_axis_angle(z_axis, 0.7);
    const Vector3 v{1.0, 2.0, 3.0};

    const Vector3 there_and_back = rotate(conjugate(q), rotate(q, v));
    CHECK(there_and_back.x == Approx(v.x));
    CHECK(there_and_back.y == Approx(v.y));
    CHECK(there_and_back.z == Approx(v.z));
}

TEST_CASE("quaternion multiplication composes rotations",
          "[linear][quaternion]") {
    const Quaternion half = from_axis_angle(z_axis, half_pi / 2.0);
    const Quaternion full = half * half;

    const Vector3 rotated = rotate(full, Vector3{1.0, 0.0, 0.0});
    CHECK(rotated.x == Approx(0.0).margin(1e-15));
    CHECK(rotated.y == Approx(1.0));
}

TEST_CASE("dot of a unit quaternion with itself is 1", "[linear][quaternion]") {
    const Quaternion q = from_axis_angle(z_axis, 0.7);
    CHECK(dot(q, q) == Approx(1.0));
}

TEST_CASE("normalize rejects the zero quaternion",
          "[linear][quaternion][degenerate]") {
    const Quaternion zero{0.0, 0.0, 0.0, 0.0};

    CHECK_FALSE(normalize(zero).has_value());
    CHECK(normalize(Quaternion{}).has_value());
}

TEST_CASE("to_matrix agrees with rotate", "[linear][quaternion]") {
    const Quaternion q = from_axis_angle(z_axis, half_pi);
    const GeoCore::linear::Matrix3 m = to_matrix(q);

    const Vector3 v{1.0, 2.0, 3.0};
    const Vector3 by_matrix = m * v;
    const Vector3 by_quaternion = rotate(q, v);

    CHECK(by_matrix.x == Approx(by_quaternion.x));
    CHECK(by_matrix.y == Approx(by_quaternion.y));
    CHECK(by_matrix.z == Approx(by_quaternion.z));
}

TEST_CASE("norm and normalize handle non-finite input consistently",
          "[linear][quaternion][degenerate]") {
    const double infinity = std::numeric_limits<double>::infinity();
    const double not_a_number = std::numeric_limits<double>::quiet_NaN();

    // norm 与 Vector3T::length 语义一致：无穷输入的模长就是无穷，不是 NaN
    CHECK(norm(Quaternion{infinity, 0.0, 0.0, 0.0}) == infinity);
    CHECK(norm(Quaternion{0.0, 0.0, infinity, 0.0}) == infinity);

    // normalize 与 UnitVector::normalize / Matrix::inverse 同一条原则：
    // 绝不交出一个 has_value() 为真、内容却是 NaN 的结果。
    CHECK_FALSE(normalize(Quaternion{infinity, 0.0, 0.0, 0.0}).has_value());
    CHECK_FALSE(normalize(Quaternion{0.0, infinity, 0.0, 0.0}).has_value());
    CHECK_FALSE(normalize(Quaternion{not_a_number, 0.0, 0.0, 0.0}).has_value());
    CHECK_FALSE(normalize(Quaternion{not_a_number, not_a_number, not_a_number, not_a_number}).has_value());
}

TEST_CASE("norm gives a slot-independent answer on non-finite input",
          "[linear][quaternion][degenerate]") {
    const double infinity = std::numeric_limits<double>::infinity();
    const double nan = std::numeric_limits<double>::quiet_NaN();

    // norm 与 VectorNT::length 共用 core::max_abs_of，规则相同：
    // 任一无穷分量 ⇒ +inf；否则含 NaN ⇒ NaN。槽位不影响答案。
    CHECK(std::isnan(norm(Quaternion{5.0, nan, 0.0, 0.0})));
    CHECK(std::isnan(norm(Quaternion{nan, 5.0, 0.0, 0.0})));
    CHECK(std::isnan(norm(Quaternion{0.0, 0.0, nan, 5.0})));
    CHECK(std::isnan(norm(Quaternion{5.0, 0.0, 0.0, nan})));

    CHECK(norm(Quaternion{infinity, nan, 0.0, 0.0}) == infinity);
    CHECK(norm(Quaternion{nan, infinity, 0.0, 0.0}) == infinity);
    CHECK(norm(Quaternion{1.0, 1.0, nan, infinity}) == infinity);

    // 无 NaN 的输入不受影响
    CHECK(norm(Quaternion{5.0, 0.0, 0.0, 0.0}) == 5.0);
}

TEST_CASE("normalize rejects input whose reciprocal magnitude overflows",
          "[linear][quaternion][degenerate]") {
    const Tolerance exact{0.0, 0.0};
    const double smallest = std::numeric_limits<double>::denorm_min();   // 5e-324

    // 模长有限并不保证倒数有限：最小的正次正规数作模长时 1/|q| 直接溢出成
    // inf，分量乘上去即得 inf。用精确容差排除「模长视为零」这条分支，单独
    // 考察溢出这一条 —— 曾经这里会返回 has_value() 为真、w == inf 的四元数。
    const auto result = normalize(Quaternion{smallest, 0.0, 0.0, 0.0}, exact);

    CHECK_FALSE(result.has_value());
}

TEST_CASE("rotation about x by 90 degrees maps y to z", "[linear][quaternion]") {
    // 上面所有用例都绕 z 轴，因此四元数的 x、y 分量恒为零 —— 那些分量上的
    // 符号或系数错误会在 0 = 0 里消失。换一个轴把它们逼出来。
    const UnitVector3 x_axis = UnitVector3::from_normalized_unchecked(Vector3{1.0, 0.0, 0.0});
    const Quaternion q = from_axis_angle(x_axis, half_pi);

    const Vector3 rotated = rotate(q, Vector3{0.0, 1.0, 0.0});
    CHECK(rotated.x == Approx(0.0).margin(1e-15));
    CHECK(rotated.y == Approx(0.0).margin(1e-15));
    CHECK(rotated.z == Approx(1.0));
}

TEST_CASE("to_matrix and rotate agree on a general axis", "[linear][quaternion]") {
    // 绕 z 轴时 xz = wy = yz = wx = 0，于是 m02/m20/m12/m21 恒为零，任何
    // 局限于这两块的符号错误都不可见。用一个一般轴让它们全部非零。
    const UnitVector3 axis = UnitVector3::from_normalized_unchecked(
        Vector3{1.0, 2.0, 3.0} / std::sqrt(14.0));
    const Quaternion q = from_axis_angle(axis, 0.7);
    const GeoCore::linear::Matrix3 m = to_matrix(q);

    // 先证明这次确实覆盖了那四个条目
    CHECK(m(0, 2) != 0.0);
    CHECK(m(2, 0) != 0.0);
    CHECK(m(1, 2) != 0.0);
    CHECK(m(2, 1) != 0.0);

    // 两条独立推导路径必须给出同一答案
    const Vector3 v{0.3, -0.7, 1.1};
    const Vector3 by_matrix = m * v;
    const Vector3 by_quaternion = rotate(q, v);

    CHECK(by_matrix.x == Approx(by_quaternion.x));
    CHECK(by_matrix.y == Approx(by_quaternion.y));
    CHECK(by_matrix.z == Approx(by_quaternion.z));
}

TEST_CASE("scalar multiplication and negation are available", "[linear][quaternion]") {
    const Quaternion q{1.0, 2.0, 3.0, 4.0};

    // 与 Vector / Matrix 的接口保持一致：两种写法都要成立
    CHECK(q * 2.0 == Quaternion{2.0, 4.0, 6.0, 8.0});
    CHECK(2.0 * q == Quaternion{2.0, 4.0, 6.0, 8.0});

    CHECK(-q == Quaternion{-1.0, -2.0, -3.0, -4.0});

    // q 与 -q 是同一个旋转：对同一个向量作用的结果必须一致
    const Quaternion unit = from_axis_angle(z_axis, half_pi);
    const Vector3 v{1.0, 2.0, 3.0};
    const Vector3 by_q = rotate(unit, v);
    const Vector3 by_negated = rotate(-unit, v);

    CHECK(by_negated.x == Approx(by_q.x));
    CHECK(by_negated.y == Approx(by_q.y));
    CHECK(by_negated.z == Approx(by_q.z));
}

TEST_CASE("normalize scales a non-unit quaternion", "[linear][quaternion]") {
    // 原用例只用 has_value() 检查 normalize，因此一个「对非零模长原样返回」
    // 的实现也能全部通过。这里数值验证缩放路径本身。
    const auto axis_aligned = normalize(Quaternion{2.0, 0.0, 0.0, 0.0});
    REQUIRE(axis_aligned.has_value());
    CHECK(axis_aligned->w == Approx(1.0));

    const auto general = normalize(Quaternion{1.0, 2.0, 3.0, 4.0});
    REQUIRE(general.has_value());
    CHECK(norm(*general) == Approx(1.0));
    CHECK(general->w == Approx(1.0 / std::sqrt(30.0)));
    CHECK(general->x == Approx(2.0 / std::sqrt(30.0)));
    CHECK(general->y == Approx(3.0 / std::sqrt(30.0)));
    CHECK(general->z == Approx(4.0 / std::sqrt(30.0)));
}

TEST_CASE("quaternion product composes rotations in the documented order",
          "[linear][quaternion]") {
    // 全库只有一处调用 operator*（half * half），而那是自乘积、操作数又是 z 轴
    // 四元数（x = y = 0），因此组合顺序从未被区分过 —— 自乘积对顺序不敏感。
    // 这里乘两个不同的半转，乘积不可交换，且下面同时验证它与逐次旋转一致。
    //
    // 注意：这两个操作数是轴对齐的，各有零分量，因而 16 个乘积项里只有 4 项
    // 是活的。钉住全部项的是紧随其后的那个一般轴用例。
    const UnitVector3 x_axis = UnitVector3::from_normalized_unchecked(Vector3{1.0, 0.0, 0.0});
    const UnitVector3 y_axis = UnitVector3::from_normalized_unchecked(Vector3{0.0, 1.0, 0.0});

    const Quaternion a = from_axis_angle(x_axis, half_pi);   // 绕 x 轴 90°
    const Quaternion b = from_axis_angle(y_axis, half_pi);   // 绕 y 轴 90°

    // 手算：a = (√2/2, √2/2, 0, 0)，b = (√2/2, 0, √2/2, 0)
    const Quaternion ab = a * b;
    CHECK(ab.w == Approx(0.5));
    CHECK(ab.x == Approx(0.5));
    CHECK(ab.y == Approx(0.5));
    CHECK(ab.z == Approx(0.5));

    // 交换相乘后 z 分量变号 —— 顺序确实有区别，且这个区别被钉住
    const Quaternion ba = b * a;
    CHECK(ba.w == Approx(0.5));
    CHECK(ba.x == Approx(0.5));
    CHECK(ba.y == Approx(0.5));
    CHECK(ba.z == Approx(-0.5));

    // 且必须与逐次旋转一致：a * b 表示先施加 b 再施加 a
    const Vector3 v{1.0, 2.0, 3.0};
    const Vector3 composed = rotate(ab, v);
    const Vector3 sequential = rotate(a, rotate(b, v));
    CHECK(composed.x == Approx(sequential.x));
    CHECK(composed.y == Approx(sequential.y));
    CHECK(composed.z == Approx(sequential.z));
}

TEST_CASE("quaternion product with general operands pins all sixteen terms",
          "[linear][quaternion]") {
    // 上一个用例的两个操作数轴对齐、各有零分量，16 个乘积项里只有 4 项是活的：
    // a.z*b.y、a.z*b.x、a.x*b.x 都退化成 0·□，那些项上的符号错误依然看不见。
    // 这里两个操作数都取一般轴，四个分量全部非零，十六项全部参与运算。
    const UnitVector3 axis_a = UnitVector3::from_normalized_unchecked(
        Vector3{1.0, 2.0, 3.0} / std::sqrt(14.0));
    const UnitVector3 axis_b = UnitVector3::from_normalized_unchecked(
        Vector3{-2.0, 1.0, 0.5} / std::sqrt(4.0 + 1.0 + 0.25));

    const Quaternion a = from_axis_angle(axis_a, 0.7);
    const Quaternion b = from_axis_angle(axis_b, 1.1);

    // 先证明这次确实没有零分量可供退化
    CHECK(a.x != 0.0);
    CHECK(a.y != 0.0);
    CHECK(a.z != 0.0);
    CHECK(b.x != 0.0);
    CHECK(b.y != 0.0);
    CHECK(b.z != 0.0);

    // 期望值由参考实现（逐项按 Hamilton 规则）独立算出
    const Quaternion ab = a * b;
    CHECK(ab.w == Approx(0.7694798520425258));
    CHECK(ab.x == Approx(-0.39226136832113895));
    CHECK(ab.y == Approx(0.23465896735876354));
    CHECK(ab.z == Approx(0.446057109865496));

    // 不可交换：w 相同，其余三个分量各不相同
    const Quaternion ba = b * a;
    CHECK(ba.w == Approx(0.7694798520425258));
    CHECK(ba.x == Approx(-0.30863891228533297));
    CHECK(ba.y == Approx(0.5064319494751331));
    CHECK(ba.z == Approx(0.23700096977598095));
}
