#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <type_traits>          // 本任务新用 STATIC_REQUIRE(std::is_same_v<...>)

#include <DragonGeo/Core/Constants.hpp>
#include <DragonGeo/Linear/Coordinate2.hpp>
#include <DragonGeo/Linear/Transform2.hpp>
#include <DragonGeo/Linear/UnitVector2.hpp>

using Catch::Approx;

using DragonGeo::Core::HALF_PI;
using DragonGeo::Core::QUARTER_PI;
using DragonGeo::Linear::Coordinate2;
using DragonGeo::Linear::Point2;
using DragonGeo::Linear::Transform2;
using DragonGeo::Linear::UnitVector2;
using DragonGeo::Linear::Transform2T;
using DragonGeo::Linear::Transform2f;
using DragonGeo::Linear::Vector2;

TEST_CASE("the float alias really is the float instantiation",
          "[linear][transform2]") {
    // 同 vector4Test.cpp：Transform2f 此前零命中，误绑定不会被断言发现。
    STATIC_REQUIRE(std::is_same_v<Transform2f, Transform2T<float>>);
}

TEST_CASE("2D translation moves positions only", "[linear][transform2]") {
    const Transform2 t = Transform2::Translation(Vector2{5.0, 7.0});

    CHECK(t.TransformPoint(Point2{1.0, 2.0}) == Point2{6.0, 9.0});
    CHECK(t * Vector2{1.0, 2.0} == Vector2{1.0, 2.0});
}

TEST_CASE("2D rotation by 90 degrees maps x to y", "[linear][transform2]") {
    const Transform2 t = Transform2::Rotation(HALF_PI);

    const Point2 rotated = t.TransformPoint(Point2{1.0, 0.0});
    CHECK(rotated.X == Approx(0.0).margin(1e-15));
    CHECK(rotated.Y == Approx(1.0));

    // operator* 只施加线性部分：方向同样被旋转，且不引入平移
    const Vector2 direction = t * Vector2{1.0, 0.0};
    CHECK(direction.X == Approx(0.0).margin(1e-15));
    CHECK(direction.Y == Approx(1.0));
}

TEST_CASE("2D scaling scales both positions and directions", "[linear][transform2]") {
    // 两个重载：各轴相同与各轴独立
    const Transform2 uniform = Transform2::Scaling(2.0);

    CHECK(uniform.TransformPoint(Point2{1.0, 3.0}) == Point2{2.0, 6.0});
    CHECK(uniform * Vector2{1.0, 3.0} == Vector2{2.0, 6.0});

    const Transform2 nonUniform = Transform2::Scaling(Vector2{2.0, 3.0});

    // 各轴输入刻意不相等：对称输入下「写入对角线」与「写入第一列」给出
    // 相同结果，无法互相区分。
    CHECK(nonUniform.TransformPoint(Point2{5.0, 7.0}) == Point2{10.0, 21.0});
    CHECK(nonUniform * Vector2{5.0, 7.0} == Vector2{10.0, 21.0});
}

TEST_CASE("2D composition applies the right operand first", "[linear][transform2]") {
    const Transform2 move = Transform2::Translation(Vector2{1.0, 0.0});
    const Transform2 scale = Transform2::Scaling(2.0);

    // 先平移再缩放：(1,1) -> (2,1) -> (4,2)
    CHECK((scale * move).TransformPoint(Point2{1.0, 1.0}) == Point2{4.0, 2.0});

    // 先缩放再平移：(1,1) -> (2,2) -> (3,2)
    CHECK((move * scale).TransformPoint(Point2{1.0, 1.0}) == Point2{3.0, 2.0});
}

TEST_CASE("inverse undoes a 2D transform", "[linear][transform2]") {
    const Transform2 t = Transform2::Translation(Vector2{5.0, -3.0})
                       * Transform2::Rotation(QUARTER_PI)
                       * Transform2::Scaling(Vector2{2.0, 3.0});

    const auto inv = t.Inverse();

    REQUIRE(inv.has_value());
    const Point2 original{1.0, 2.0};
    const Point2 roundTrip = inv->TransformPoint(t.TransformPoint(original));

    CHECK(roundTrip.X == Approx(original.X).margin(1e-12));
    CHECK(roundTrip.Y == Approx(original.Y).margin(1e-12));
}

TEST_CASE("inverse of a degenerate 2D transform is nullopt",
          "[linear][transform2][degenerate]") {
    // 把 y 轴压扁：行列式为 0，不存在逆变换
    const Transform2 flatten = Transform2::Scaling(Vector2{1.0, 0.0});

    CHECK_FALSE(flatten.Inverse().has_value());
}

TEST_CASE("Transform2 identity leaves a position unchanged", "[linear][transform2]") {
    const Transform2 unit = Transform2::Identity();

    CHECK(unit.TransformPoint(Point2{1.0, 2.0}) == Point2{1.0, 2.0});
    CHECK(unit == Transform2{});
    CHECK(Transform2::Translation(Vector2{1.0, 0.0}) != Transform2::Translation(Vector2{0.0, 1.0}));
    CHECK(Transform2::Scaling(-1.0).TransformPoint(Point2{2.0, 3.0}) == Point2{-2.0, -3.0});
}

TEST_CASE("a 2D transform carries points, not just vectors",
          "[linear][transform2]") {
    const Transform2 t = Transform2::Translation(Vector2{10.0, 0.0});
    const Point2 p{1.0, 2.0};

    // 点被平移
    STATIC_REQUIRE(std::is_same_v<decltype(t * p), Point2>);
    CHECK(t * p == Point2{11.0, 2.0});
    CHECK(t.TransformPoint(p) == Point2{11.0, 2.0});

    // 方向不被平移 —— 这是 operator* 与 TransformPoint 的分工
    CHECK(t * Vector2{1.0, 2.0} == Vector2{1.0, 2.0});
}

TEST_CASE("2D TransformPoint is the explicit spelling of carrying a position",
          "[linear][transform2]") {
    // 两轴互不相等的缩放 + 两轴互不相等的平移：(x,y) -> (2x+1, 3y+10)。
    // 这些数全部精确可表示，== 是安全的。轴不同、平移列不同，因此任一处轴
    // 错位、系数张冠李戴、丢掉平移列或丢掉线性部分，都会改变某个分量。
    const Transform2 t = Transform2::Translation(Vector2{1.0, 10.0})
                       * Transform2::Scaling(Vector2{2.0, 3.0});

    // 返回类型是 Point2（不是 Vector2）—— 类型层面的分工也钉住
    STATIC_REQUIRE(std::is_same_v<decltype(t.TransformPoint(Point2{1.0, 2.0})), Point2>);

    CHECK(t.TransformPoint(Point2{1.0, 2.0}) == Point2{3.0, 16.0});

    // operator*(Point2) 与 TransformPoint 是同一件事的两种拼写
    CHECK(t * Point2{1.0, 2.0} == t.TransformPoint(Point2{1.0, 2.0}));

    // 同一个矩阵、同一组数字：点吃平移、方向不吃，两者之差恰是平移量。
    CHECK((t * Point2{1.0, 2.0}) - (t * Vector2{1.0, 2.0}) == Point2{1.0, 10.0});
}

TEST_CASE("a 2D rotation carries points off the diagonal", "[linear][transform2]") {
    // 45° 旋转把非对角系数牵进来：对 (1,2) 有 (x,y) -> ((x-y)/√2, (x+y)/√2)。
    // 纯对角矩阵的用例看不见转置或轴错位，这一格专门喂非对角项。
    //
    // 余量：cos/sin(π/4) 的浮点值各带 1 ulp，这条路径与 3D 的 quaternion → matrix
    // 一样不承诺精确舍入 —— 与文件里 inverse 往返（margin 1e-12）同一理由。
    // 实测：x 偏 1 ulp（1.11e-16）→ 余量 9007 倍；y 偏 1 ulp of 2.12（4.44e-16）
    // → 2252 倍。真正的语义错误（转置、轴错位、丢平移）差的是 O(1)。
    const Transform2 spin = Transform2::Rotation(QUARTER_PI);
    const Point2 p{1.0, 2.0};

    const Point2 carried = spin.TransformPoint(p);
    CHECK(carried.X == Approx(-0.7071067811865476).margin(1e-12));
    CHECK(carried.Y == Approx(2.1213203435596424).margin(1e-12));

    const Point2 carriedByOperator = spin * p;
    CHECK(carriedByOperator.X == Approx(-0.7071067811865476).margin(1e-12));
    CHECK(carriedByOperator.Y == Approx(2.1213203435596424).margin(1e-12));
}

TEST_CASE("2D TransformPoint is constexpr and noexcept",
          "[linear][transform2]") {
    // TransformPoint 与 operator*(Point2) 都要能用于常量表达式。
    constexpr Transform2 t = Transform2::Translation(Vector2{1.0, 10.0})
                           * Transform2::Scaling(Vector2{2.0, 3.0});
    constexpr Point2 p{1.0, 2.0};

    STATIC_REQUIRE(t.TransformPoint(p) == Point2{3.0, 16.0});
    STATIC_REQUIRE(t * p == Point2{3.0, 16.0});

    STATIC_REQUIRE(noexcept(t.TransformPoint(p)));
    STATIC_REQUIRE(noexcept(t * p));

    // 单位变换的两种拼写都原样送回原点。
    CHECK(Transform2::Identity().TransformPoint(p) == p);
    CHECK(Transform2::Identity() * p == p);
}

TEST_CASE("composing 2D transforms agrees with applying TransformPoint in order",
          "[linear][transform2]") {
    // a 与 b 都带平移与缩放。点走 TransformPoint / operator*(Point2)，复合后
    // 一次施加与先 b 再 a 是同一件事。
    const Transform2 a = Transform2::Translation(Vector2{100.0, 0.0})
                       * Transform2::Scaling(Vector2{2.0, 3.0});
    const Transform2 b = Transform2::Translation(Vector2{0.0, 10.0})
                       * Transform2::Scaling(Vector2{5.0, 7.0});
    const Point2 p{1.0, 2.0};

    // b(p) = (5, 24)；a(b(p)) = (2*5+100, 3*24) = (110, 72)，全部精确
    CHECK(a * b.TransformPoint(p) == (a * b).TransformPoint(p));
    CHECK((a * b).TransformPoint(p) == Point2{110.0, 72.0});
}

TEST_CASE("reflecting across a coordinate axis flips the other coordinate",
          "[linear][transform2]") {
    // ReflectionX 保住 X 轴，法向是 +Y：(x, y) -> (x, -y)。
    const Transform2 acrossX = Transform2::ReflectionX();
    CHECK(acrossX.TransformPoint(Point2{3.0, 4.0}) == Point2{3.0, -4.0});
    CHECK(acrossX * Vector2{3.0, 4.0} == Vector2{3.0, -4.0});
    CHECK(acrossX == Transform2::Scaling(Vector2{1.0, -1.0}));
    CHECK(acrossX == Transform2::Reflection(
              Point2{}, UnitVector2::FromNormalizedUnchecked(Vector2{0.0, 1.0})));
    CHECK(acrossX * acrossX == Transform2::Identity());

    // ReflectionY 保住 Y 轴，法向是 +X：(x, y) -> (-x, y)。
    const Transform2 acrossY = Transform2::ReflectionY();
    CHECK(acrossY.TransformPoint(Point2{3.0, 4.0}) == Point2{-3.0, 4.0});
    CHECK(acrossY * Vector2{3.0, 4.0} == Vector2{-3.0, 4.0});
    CHECK(acrossY == Transform2::Scaling(Vector2{-1.0, 1.0}));
    CHECK(acrossY == Transform2::Reflection(
              Point2{}, UnitVector2::FromNormalizedUnchecked(Vector2{1.0, 0.0})));
    CHECK(acrossY * acrossY == Transform2::Identity());

    constexpr Transform2 axis = Transform2::ReflectionX();
    STATIC_REQUIRE(axis.TransformPoint(Point2{3.0, 4.0}) == Point2{3.0, -4.0});
    STATIC_REQUIRE(noexcept(Transform2::ReflectionX()));
    STATIC_REQUIRE(noexcept(Transform2::ReflectionY()));
    STATIC_REQUIRE(noexcept(Transform2::Reflection(
        Point2{}, UnitVector2::FromNormalizedUnchecked(Vector2{0.0, 1.0}))));

    // 正交且行列式为 −1，不能成为右手标架。
    CHECK_FALSE(Coordinate2::FromTransform(acrossX).has_value());
}

TEST_CASE("a 2D reflection is the same mirror for either normal sign",
          "[linear][transform2]") {
    const UnitVector2 normal = *Vector2{3.0, -4.0}.Normalized();
    const Point2 point{2.0, -5.0};

    CHECK(Transform2::Reflection(point, normal) == Transform2::Reflection(point, -normal));
}

TEST_CASE("reflecting across a line that misses the origin",
          "[linear][transform2]") {
    // 直线 y = 4，法向 +Y。线上的点不动；(2, 6) 落到 (2, 2)。
    const UnitVector2 normal = UnitVector2::FromNormalizedUnchecked(Vector2{0.0, 1.0});
    const Transform2 mirror = Transform2::Reflection(Point2{1.0, 4.0}, normal);
    const Transform2 sameLine = Transform2::Reflection(Point2{7.0, 4.0}, normal);

    CHECK(mirror == sameLine);
    CHECK(mirror.TransformPoint(Point2{2.0, 4.0}) == Point2{2.0, 4.0});
    CHECK(mirror.TransformPoint(Point2{2.0, 6.0}) == Point2{2.0, 2.0});
    CHECK(mirror.TransformPoint(mirror.TransformPoint(Point2{2.0, 9.0})) == Point2{2.0, 9.0});

    // 方向只走线性部分，镜面离原点的平移不改变方向。
    CHECK(mirror * Vector2{0.0, 3.0} == Vector2{0.0, -3.0});
    CHECK(mirror * Vector2{5.0, 0.0} == Vector2{5.0, 0.0});
    CHECK(mirror * Vector2{0.0, 3.0}
          == Transform2::Reflection(Point2{}, normal) * Vector2{0.0, 3.0});

    const auto inverse = mirror.Inverse();
    REQUIRE(inverse.has_value());
    CHECK(inverse->TransformPoint(Point2{2.0, 9.0}) == mirror.TransformPoint(Point2{2.0, 9.0}));
}

TEST_CASE("reflecting across a diagonal line flips the normal component",
          "[linear][transform2]") {
    // 法向 (1, 1)/√2。直线方向是 (1, -1)，其上的 (1, -1) 不动。
    // (1, 0) 落到 (0, -1)。非对角元写错时，这一格不再成立。
    const UnitVector2 normal = *Vector2{1.0, 1.0}.Normalized();
    const Transform2 mirror = Transform2::Reflection(Point2{}, normal);
    const Point2 image = mirror.TransformPoint(Point2{1.0, 0.0});

    CHECK(mirror.TransformPoint(Point2{1.0, -1.0}) == Point2{1.0, -1.0});
    CHECK(image.X == Approx(0.0).margin(1e-12));
    CHECK(image.Y == Approx(-1.0).margin(1e-12));
    CHECK(mirror.TransformPoint(image).X == Approx(1.0).margin(1e-12));
    CHECK(mirror.TransformPoint(image).Y == Approx(0.0).margin(1e-12));
}
