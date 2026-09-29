#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>

#include <GeoCore/core/Constants.hpp>
#include <GeoCore/linear/Transform3.hpp>

using Catch::Approx;

using GeoCore::core::half_pi;
using GeoCore::linear::Transform3;
using GeoCore::linear::UnitVector3;
using GeoCore::linear::Vector3;

namespace {
const UnitVector3 z_axis = UnitVector3::from_normalized_unchecked(Vector3{0.0, 0.0, 1.0});
}

TEST_CASE("identity transform leaves a position unchanged", "[linear][transform3]") {
    const Transform3 t = Transform3::identity();

    CHECK(t.apply(Vector3{1.0, 2.0, 3.0}) == Vector3{1.0, 2.0, 3.0});
}

TEST_CASE("translation moves a position but not a direction",
          "[linear][transform3]") {
    const Transform3 t = Transform3::translation(Vector3{10.0, 20.0, 30.0});

    // 位置被平移
    CHECK(t.apply(Vector3{1.0, 2.0, 3.0}) == Vector3{11.0, 22.0, 33.0});

    // 方向不受平移影响 —— 这正是 operator* 与 apply 的分工
    CHECK(t * Vector3{1.0, 2.0, 3.0} == Vector3{1.0, 2.0, 3.0});
}

TEST_CASE("uniform scaling scales both positions and directions",
          "[linear][transform3]") {
    const Transform3 t = Transform3::scaling(2.0);

    CHECK(t.apply(Vector3{1.0, 2.0, 3.0}) == Vector3{2.0, 4.0, 6.0});
    CHECK(t * Vector3{1.0, 2.0, 3.0} == Vector3{2.0, 4.0, 6.0});
}

TEST_CASE("non-uniform scaling scales each axis independently",
          "[linear][transform3]") {
    const Transform3 t = Transform3::scaling(Vector3{2.0, 3.0, 4.0});

    CHECK(t.apply(Vector3{1.0, 1.0, 1.0}) == Vector3{2.0, 3.0, 4.0});

    // (1,1,1) 是全对称输入：「写对角线」与「写第一列」得到的矩阵在它上面
    // 给出相同结果，单靠这一行分不开两者。换成各分量互不相等的输入，第一列
    // 写入会算成 (2*1, 2*2, 2*5) 之类的错误值。
    CHECK(t.apply(Vector3{1.0, 2.0, 5.0}) == Vector3{2.0, 6.0, 20.0});
    CHECK(t * Vector3{1.0, 2.0, 5.0} == Vector3{2.0, 6.0, 20.0});
}

TEST_CASE("rotation about z by 90 degrees", "[linear][transform3]") {
    const Transform3 t = Transform3::rotation(z_axis, half_pi);

    const Vector3 rotated = t.apply(Vector3{1.0, 0.0, 0.0});
    CHECK(rotated.x == Approx(0.0).margin(1e-15));
    CHECK(rotated.y == Approx(1.0));
    CHECK(rotated.z == Approx(0.0).margin(1e-15));
}

TEST_CASE("composition applies the right operand first", "[linear][transform3]") {
    const Transform3 move = Transform3::translation(Vector3{1.0, 0.0, 0.0});
    const Transform3 spin = Transform3::rotation(z_axis, half_pi);

    // 绕 z 的 90° 旋转在浮点下含约 1e-16 的误差，故用 margin 而非精确相等
    // 先平移再旋转： (1,0,0) -> (2,0,0) -> (0,2,0)
    const Vector3 rotate_after = (spin * move).apply(Vector3{1.0, 0.0, 0.0});
    CHECK(rotate_after.x == Approx(0.0).margin(1e-15));
    CHECK(rotate_after.y == Approx(2.0));
    CHECK(rotate_after.z == Approx(0.0).margin(1e-15));

    // 先旋转再平移： (1,0,0) -> (0,1,0) -> (1,1,0)
    const Vector3 translate_after = (move * spin).apply(Vector3{1.0, 0.0, 0.0});
    CHECK(translate_after.x == Approx(1.0));
    CHECK(translate_after.y == Approx(1.0));
    CHECK(translate_after.z == Approx(0.0).margin(1e-15));
}

TEST_CASE("inverse undoes the transform", "[linear][transform3]") {
    const Transform3 t = Transform3::translation(Vector3{5.0, -3.0, 2.0})
                       * Transform3::rotation(z_axis, 0.7)
                       * Transform3::scaling(2.0);

    const auto inv = t.inverse();

    REQUIRE(inv.has_value());
    const Vector3 original{1.0, 2.0, 3.0};
    const Vector3 round_trip = inv->apply(t.apply(original));

    CHECK(round_trip.x == Approx(original.x).margin(1e-12));
    CHECK(round_trip.y == Approx(original.y).margin(1e-12));
    CHECK(round_trip.z == Approx(original.z).margin(1e-12));
}

TEST_CASE("inverse of a degenerate transform is nullopt",
          "[linear][transform3][degenerate]") {
    const Transform3 flatten = Transform3::scaling(Vector3{1.0, 0.0, 1.0});

    CHECK_FALSE(flatten.inverse().has_value());
}

TEST_CASE("Transform3 members and static factories", "[linear][transform3]") {
    const Transform3 t = Transform3::translation(Vector3{10.0, 20.0, 30.0});

    CHECK(t.apply(Vector3{1.0, 2.0, 3.0}) == Vector3{11.0, 22.0, 33.0});
    CHECK(t * Vector3{1.0, 2.0, 3.0} == Vector3{1.0, 2.0, 3.0});

    const Transform3 unit = Transform3::identity();
    CHECK(unit.apply(Vector3{1.0, 2.0, 3.0}) == Vector3{1.0, 2.0, 3.0});

    // 逆走的是行列平衡 + 除法还原，这条路径不承诺精确舍入；本文件既有的
    // "inverse undoes the transform" 用的就是 margin(1e-12)。沿用同一强度 ——
    // 断言精确相等会把正常的浮点舍入当成缺陷，换个编译器/libm 就会碎。
    const auto inv = t.inverse();
    REQUIRE(inv.has_value());
    const Vector3 back = inv->apply(Vector3{11.0, 22.0, 33.0});
    CHECK(back.x == Approx(1.0).margin(1e-12));
    CHECK(back.y == Approx(2.0).margin(1e-12));
    CHECK(back.z == Approx(3.0).margin(1e-12));
}
