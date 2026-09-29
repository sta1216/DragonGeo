#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <GeoCore/core/Constants.hpp>
#include <GeoCore/linear/Transform2.hpp>

using Catch::Approx;

using GeoCore::core::half_pi;
using GeoCore::core::quarter_pi;
using GeoCore::linear::apply;
using GeoCore::linear::inverse;
using GeoCore::linear::rotation_2d;
using GeoCore::linear::scaling_2d;
using GeoCore::linear::Transform2;
using GeoCore::linear::translation_2d;
using GeoCore::linear::Vector2;

TEST_CASE("2D translation moves positions only", "[linear][transform2]") {
    const Transform2 t = translation_2d(Vector2{5.0, 7.0});

    CHECK(apply(t, Vector2{1.0, 2.0}) == Vector2{6.0, 9.0});
    CHECK(t * Vector2{1.0, 2.0} == Vector2{1.0, 2.0});
}

TEST_CASE("2D rotation by 90 degrees maps x to y", "[linear][transform2]") {
    const Transform2 t = rotation_2d(half_pi);

    const Vector2 rotated = apply(t, Vector2{1.0, 0.0});
    CHECK(rotated.x == Approx(0.0).margin(1e-15));
    CHECK(rotated.y == Approx(1.0));

    // operator* 只施加线性部分：方向同样被旋转，且不引入平移
    const Vector2 direction = t * Vector2{1.0, 0.0};
    CHECK(direction.x == Approx(0.0).margin(1e-15));
    CHECK(direction.y == Approx(1.0));
}

TEST_CASE("2D scaling scales both positions and directions", "[linear][transform2]") {
    // 两个重载：各轴相同与各轴独立
    const Transform2 uniform = scaling_2d(2.0);

    CHECK(apply(uniform, Vector2{1.0, 3.0}) == Vector2{2.0, 6.0});
    CHECK(uniform * Vector2{1.0, 3.0} == Vector2{2.0, 6.0});

    const Transform2 non_uniform = scaling_2d(Vector2{2.0, 3.0});

    // 各轴输入刻意不相等：对称输入下「写入对角线」与「写入第一列」给出
    // 相同结果，无法互相区分。
    CHECK(apply(non_uniform, Vector2{5.0, 7.0}) == Vector2{10.0, 21.0});
    CHECK(non_uniform * Vector2{5.0, 7.0} == Vector2{10.0, 21.0});
}

TEST_CASE("2D composition applies the right operand first", "[linear][transform2]") {
    const Transform2 move = translation_2d(Vector2{1.0, 0.0});
    const Transform2 scale = scaling_2d(2.0);

    // 先平移再缩放：(1,1) -> (2,1) -> (4,2)
    CHECK(apply(scale * move, Vector2{1.0, 1.0}) == Vector2{4.0, 2.0});

    // 先缩放再平移：(1,1) -> (2,2) -> (3,2)
    CHECK(apply(move * scale, Vector2{1.0, 1.0}) == Vector2{3.0, 2.0});
}

TEST_CASE("inverse undoes a 2D transform", "[linear][transform2]") {
    const Transform2 t = translation_2d(Vector2{5.0, -3.0})
                       * rotation_2d(quarter_pi)
                       * scaling_2d(Vector2{2.0, 3.0});

    const auto inv = inverse(t);

    REQUIRE(inv.has_value());
    const Vector2 original{1.0, 2.0};
    const Vector2 round_trip = apply(*inv, apply(t, original));

    CHECK(round_trip.x == Approx(original.x).margin(1e-12));
    CHECK(round_trip.y == Approx(original.y).margin(1e-12));
}

TEST_CASE("inverse of a degenerate 2D transform is nullopt",
          "[linear][transform2][degenerate]") {
    // 把 y 轴压扁：行列式为 0，不存在逆变换
    const Transform2 flatten = scaling_2d(Vector2{1.0, 0.0});

    CHECK_FALSE(inverse(flatten).has_value());
}
