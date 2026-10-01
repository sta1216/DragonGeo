#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <type_traits>          // 本任务新用 STATIC_REQUIRE(std::is_same_v<...>)

#include <DragonGeo/Core/Constants.hpp>
#include <DragonGeo/Linear/Coordinate3.hpp>
#include <DragonGeo/Linear/Transform3.hpp>

using Catch::Approx;

using DragonGeo::Core::HALF_PI;
using DragonGeo::Linear::Coordinate3;
using DragonGeo::Linear::Point3;
using DragonGeo::Linear::Transform3;
using DragonGeo::Linear::Transform3T;
using DragonGeo::Linear::Transform3f;
using DragonGeo::Linear::UnitVector3;
using DragonGeo::Linear::Vector3;

namespace {
const UnitVector3 xAxis = UnitVector3::FromNormalizedUnchecked(Vector3{1.0, 0.0, 0.0});
const UnitVector3 zAxis = UnitVector3::FromNormalizedUnchecked(Vector3{0.0, 0.0, 1.0});
}

TEST_CASE("the float alias really is the float instantiation",
          "[linear][transform3]") {
    // 同 vector4Test.cpp：Transform3f 此前零命中，误绑定不会被断言发现。
    STATIC_REQUIRE(std::is_same_v<Transform3f, Transform3T<float>>);
}

TEST_CASE("identity transform leaves a position unchanged", "[linear][transform3]") {
    const Transform3 t = Transform3::Identity();

    CHECK(t.TransformPoint(Point3{1.0, 2.0, 3.0}) == Point3{1.0, 2.0, 3.0});
}

TEST_CASE("translation moves a position but not a direction",
          "[linear][transform3]") {
    const Transform3 t = Transform3::Translation(Vector3{10.0, 20.0, 30.0});

    // 位置被平移
    CHECK(t.TransformPoint(Point3{1.0, 2.0, 3.0}) == Point3{11.0, 22.0, 33.0});

    // 方向不受平移影响 —— 这正是 operator* 与 TransformPoint 的分工
    CHECK(t * Vector3{1.0, 2.0, 3.0} == Vector3{1.0, 2.0, 3.0});
}

TEST_CASE("uniform scaling scales both positions and directions",
          "[linear][transform3]") {
    const Transform3 t = Transform3::Scaling(2.0);

    CHECK(t.TransformPoint(Point3{1.0, 2.0, 3.0}) == Point3{2.0, 4.0, 6.0});
    CHECK(t * Vector3{1.0, 2.0, 3.0} == Vector3{2.0, 4.0, 6.0});
}

TEST_CASE("non-uniform scaling scales each axis independently",
          "[linear][transform3]") {
    const Transform3 t = Transform3::Scaling(Vector3{2.0, 3.0, 4.0});

    CHECK(t.TransformPoint(Point3{1.0, 1.0, 1.0}) == Point3{2.0, 3.0, 4.0});

    // (1,1,1) 是全对称输入：「写对角线」与「写第一列」得到的矩阵在它上面
    // 给出相同结果，单靠这一行分不开两者。换成各分量互不相等的输入，第一列
    // 写入会算成 (2*1, 2*2, 2*5) 之类的错误值。
    CHECK(t.TransformPoint(Point3{1.0, 2.0, 5.0}) == Point3{2.0, 6.0, 20.0});
    CHECK(t * Vector3{1.0, 2.0, 5.0} == Vector3{2.0, 6.0, 20.0});
}

TEST_CASE("rotation about z by 90 degrees", "[linear][transform3]") {
    const Transform3 t = Transform3::Rotation(zAxis, HALF_PI);

    const Point3 rotated = t.TransformPoint(Point3{1.0, 0.0, 0.0});
    CHECK(rotated.X == Approx(0.0).margin(1e-15));
    CHECK(rotated.Y == Approx(1.0));
    CHECK(rotated.Z == Approx(0.0).margin(1e-15));
}

TEST_CASE("composition applies the right operand first", "[linear][transform3]") {
    const Transform3 move = Transform3::Translation(Vector3{1.0, 0.0, 0.0});
    const Transform3 spin = Transform3::Rotation(zAxis, HALF_PI);

    // 绕 z 的 90° 旋转在浮点下含约 1e-16 的误差，故用 margin 而非精确相等
    // 先平移再旋转： (1,0,0) -> (2,0,0) -> (0,2,0)
    const Point3 rotateAfter = (spin * move).TransformPoint(Point3{1.0, 0.0, 0.0});
    CHECK(rotateAfter.X == Approx(0.0).margin(1e-15));
    CHECK(rotateAfter.Y == Approx(2.0));
    CHECK(rotateAfter.Z == Approx(0.0).margin(1e-15));

    // 先旋转再平移： (1,0,0) -> (0,1,0) -> (1,1,0)
    const Point3 translateAfter = (move * spin).TransformPoint(Point3{1.0, 0.0, 0.0});
    CHECK(translateAfter.X == Approx(1.0));
    CHECK(translateAfter.Y == Approx(1.0));
    CHECK(translateAfter.Z == Approx(0.0).margin(1e-15));
}

TEST_CASE("inverse undoes the transform", "[linear][transform3]") {
    const Transform3 t = Transform3::Translation(Vector3{5.0, -3.0, 2.0})
                       * Transform3::Rotation(zAxis, 0.7)
                       * Transform3::Scaling(2.0);

    const auto inv = t.Inverse();

    REQUIRE(inv.has_value());
    const Point3 original{1.0, 2.0, 3.0};
    const Point3 roundTrip = inv->TransformPoint(t.TransformPoint(original));

    CHECK(roundTrip.X == Approx(original.X).margin(1e-12));
    CHECK(roundTrip.Y == Approx(original.Y).margin(1e-12));
    CHECK(roundTrip.Z == Approx(original.Z).margin(1e-12));
}

TEST_CASE("inverse of a degenerate transform is nullopt",
          "[linear][transform3][degenerate]") {
    const Transform3 flatten = Transform3::Scaling(Vector3{1.0, 0.0, 1.0});

    CHECK_FALSE(flatten.Inverse().has_value());
}

TEST_CASE("Transform3 members and static factories", "[linear][transform3]") {
    const Transform3 t = Transform3::Translation(Vector3{10.0, 20.0, 30.0});

    CHECK(t.TransformPoint(Point3{1.0, 2.0, 3.0}) == Point3{11.0, 22.0, 33.0});
    CHECK(t * Vector3{1.0, 2.0, 3.0} == Vector3{1.0, 2.0, 3.0});

    const Transform3 unit = Transform3::Identity();
    CHECK(unit.TransformPoint(Point3{1.0, 2.0, 3.0}) == Point3{1.0, 2.0, 3.0});

    // 逆走的是行列平衡 + 除法还原，这条路径不承诺精确舍入；本文件既有的
    // "inverse undoes the transform" 用的就是 margin(1e-12)。沿用同一强度 ——
    // 断言精确相等会把正常的浮点舍入当成缺陷，换个编译器/libm 就会碎。
    const auto inv = t.Inverse();
    REQUIRE(inv.has_value());
    const Point3 back = inv->TransformPoint(Point3{11.0, 22.0, 33.0});
    CHECK(back.X == Approx(1.0).margin(1e-12));
    CHECK(back.Y == Approx(2.0).margin(1e-12));
    CHECK(back.Z == Approx(3.0).margin(1e-12));
}

TEST_CASE("a transform carries points, not just vectors",
          "[linear][transform3]") {
    const Transform3 t = Transform3::Translation(Vector3{10.0, 0.0, 0.0});
    const Point3 p{1.0, 2.0, 3.0};

    // 点被平移
    STATIC_REQUIRE(std::is_same_v<decltype(t * p), Point3>);
    CHECK(t * p == Point3{11.0, 2.0, 3.0});
    CHECK(t.TransformPoint(p) == Point3{11.0, 2.0, 3.0});

    // 方向不被平移 —— 这是 operator* 与 TransformPoint 的分工
    CHECK(t * Vector3{1.0, 2.0, 3.0} == Vector3{1.0, 2.0, 3.0});
}

TEST_CASE("TransformPoint is the explicit spelling of carrying a position",
          "[linear][transform3]") {
    // 各轴互不相等的缩放 + 各轴互不相等的平移：(x,y,z) -> (2x+1, 3y+10, 4z+100)。
    // 这些数全部精确可表示，== 是安全的。三根轴各不同、平移列各不同，因此
    // 任一处轴错位、系数张冠李戴、丢掉平移列或丢掉线性部分，都会改变某个分量。
    const Transform3 t = Transform3::Translation(Vector3{1.0, 10.0, 100.0})
                       * Transform3::Scaling(Vector3{2.0, 3.0, 4.0});

    // 返回类型是 Point3（不是 Vector3）—— 类型层面的分工也钉住
    STATIC_REQUIRE(std::is_same_v<decltype(t.TransformPoint(Point3{1.0, 2.0, 3.0})), Point3>);

    CHECK(t.TransformPoint(Point3{1.0, 2.0, 3.0}) == Point3{3.0, 16.0, 112.0});

    // operator*(Point3) 与 TransformPoint 是同一件事的两种拼写
    CHECK(t * Point3{1.0, 2.0, 3.0} == t.TransformPoint(Point3{1.0, 2.0, 3.0}));

    // 同一个矩阵、同一组数字：点吃平移、方向不吃，两者之差恰是平移量。
    // 若 Point 重载悄悄走了 Vector 那条只施加线性部分的路，这一条立刻失败。
    CHECK((t * Point3{1.0, 2.0, 3.0}) - (t * Vector3{1.0, 2.0, 3.0})
          == Point3{1.0, 10.0, 100.0});
}

TEST_CASE("a rotation carries points off the diagonal", "[linear][transform3]") {
    // 绕 z 的 90° 旋转把非对角系数牵进来：对 (1,2,3) 有 (x,y,z) -> (-y, x, z)。
    // 纯对角矩阵的用例看不见转置或轴错位，这一格专门喂非对角项。
    //
    // 余量：quaternion → matrix 的条目各带约 1 ulp（实测 m01 = -1.0000000000000002、
    // m00 = -2.22e-16 而非 6.12e-17），这条路径不承诺精确舍入 —— 与文件里
    // inverse 往返（margin 1e-12）同一理由，这里也用 margin(1e-12)。
    // 实测：x 偏 2 ulp of 2（8.88e-16）→ 余量 1126 倍；y 偏 2 ulp（2.22e-16）
    // → 4504 倍；z 精确。真正的语义错误（转置、轴错位、丢平移）差的是 O(1)，
    // 放宽到这个余量不损失检出。（margin 1e-15 时 x 只剩 1.13 倍余量，
    // 换 libm/编译器就可能假失败 —— 这正是 Task 8 那条余量规则的同一类问题。）
    const Transform3 spin = Transform3::Rotation(zAxis, HALF_PI);
    const Point3 p{1.0, 2.0, 3.0};

    const Point3 carried = spin.TransformPoint(p);
    CHECK(carried.X == Approx(-2.0).margin(1e-12));
    CHECK(carried.Y == Approx(1.0).margin(1e-12));
    CHECK(carried.Z == Approx(3.0).margin(1e-12));

    const Point3 carriedByOperator = spin * p;
    CHECK(carriedByOperator.X == Approx(-2.0).margin(1e-12));
    CHECK(carriedByOperator.Y == Approx(1.0).margin(1e-12));
    CHECK(carriedByOperator.Z == Approx(3.0).margin(1e-12));
}

TEST_CASE("TransformPoint is constexpr and noexcept",
          "[linear][transform3]") {
    constexpr Transform3 t = Transform3::Translation(Vector3{1.0, 10.0, 100.0})
                           * Transform3::Scaling(Vector3{2.0, 3.0, 4.0});
    constexpr Point3 p{1.0, 2.0, 3.0};

    STATIC_REQUIRE(t.TransformPoint(p) == Point3{3.0, 16.0, 112.0});
    STATIC_REQUIRE(t * p == Point3{3.0, 16.0, 112.0});

    STATIC_REQUIRE(noexcept(t.TransformPoint(p)));
    STATIC_REQUIRE(noexcept(t * p));

    // 单位变换的两种拼写都原样送回原点。
    CHECK(Transform3::Identity().TransformPoint(p) == p);
    CHECK(Transform3::Identity() * p == p);
}

TEST_CASE("a composed transform and chained member calls are different spellings",
          "[linear][transform3]") {
    // 位置走 Point。若还留着 Apply(Vector)，`a * b.Apply(v)` 能编译，
    // 但 operator* 对 Vector 只施加线性部分，a 的平移会被丢掉。
    //
    // 这一格让 a 与 b 都带平移与缩放。`a * b.TransformPoint(p)` 与
    // `(a * b).TransformPoint(p)` 是同一件事。
    const Transform3 a = Transform3::Translation(Vector3{100.0, 0.0, 0.0})
                       * Transform3::Scaling(Vector3{2.0, 3.0, 4.0});
    const Transform3 b = Transform3::Translation(Vector3{0.0, 10.0, 20.0})
                       * Transform3::Scaling(Vector3{5.0, 6.0, 7.0});
    const Point3 p{1.0, 2.0, 3.0};
    CHECK(a * b.TransformPoint(p) == (a * b).TransformPoint(p));
    CHECK((a * b).TransformPoint(p) == Point3{110.0, 66.0, 164.0});
}

TEST_CASE("reflecting across a coordinate plane flips the remaining axis",
          "[linear][transform3]") {
    const UnitVector3 axisX = UnitVector3::FromNormalizedUnchecked(Vector3{1.0, 0.0, 0.0});
    const UnitVector3 axisY = UnitVector3::FromNormalizedUnchecked(Vector3{0.0, 1.0, 0.0});

    // YZ 平面，法向 +X：(x, y, z) -> (-x, y, z)。
    const Transform3 acrossYZ = Transform3::ReflectionYZ();
    CHECK(acrossYZ.TransformPoint(Point3{3.0, 4.0, 5.0}) == Point3{-3.0, 4.0, 5.0});
    CHECK(acrossYZ * Vector3{3.0, 4.0, 5.0} == Vector3{-3.0, 4.0, 5.0});
    CHECK(acrossYZ == Transform3::Scaling(Vector3{-1.0, 1.0, 1.0}));
    CHECK(acrossYZ == Transform3::Reflection(Point3{}, axisX));
    CHECK(acrossYZ * acrossYZ == Transform3::Identity());

    // ZX 平面，法向 +Y。
    const Transform3 acrossZX = Transform3::ReflectionZX();
    CHECK(acrossZX.TransformPoint(Point3{3.0, 4.0, 5.0}) == Point3{3.0, -4.0, 5.0});
    CHECK(acrossZX == Transform3::Scaling(Vector3{1.0, -1.0, 1.0}));
    CHECK(acrossZX == Transform3::Reflection(Point3{}, axisY));
    CHECK(acrossZX * acrossZX == Transform3::Identity());

    // XY 平面，法向 +Z。
    const Transform3 acrossXY = Transform3::ReflectionXY();
    CHECK(acrossXY.TransformPoint(Point3{3.0, 4.0, 5.0}) == Point3{3.0, 4.0, -5.0});
    CHECK(acrossXY == Transform3::Scaling(Vector3{1.0, 1.0, -1.0}));
    CHECK(acrossXY == Transform3::Reflection(Point3{}, zAxis));
    CHECK(acrossXY * acrossXY == Transform3::Identity());

    constexpr Transform3 plane = Transform3::ReflectionXY();
    STATIC_REQUIRE(plane.TransformPoint(Point3{3.0, 4.0, 5.0}) == Point3{3.0, 4.0, -5.0});
    STATIC_REQUIRE(noexcept(Transform3::ReflectionYZ()));
    STATIC_REQUIRE(noexcept(Transform3::ReflectionZX()));
    STATIC_REQUIRE(noexcept(Transform3::ReflectionXY()));
    STATIC_REQUIRE(noexcept(Transform3::Reflection(Point3{}, zAxis)));

    CHECK_FALSE(Coordinate3::FromTransform(acrossXY).has_value());
}

TEST_CASE("a 3D reflection is the same mirror for either normal sign",
          "[linear][transform3]") {
    const UnitVector3 normal = *Vector3{1.0, -2.0, 2.0}.Normalized();
    const Point3 point{4.0, -1.0, 6.0};

    CHECK(Transform3::Reflection(point, normal) == Transform3::Reflection(point, -normal));
}

TEST_CASE("reflecting across a plane that misses the origin",
          "[linear][transform3]") {
    // 平面 z = 2。面上的点不动；(1, 4, 5) 落到 (1, 4, -1)。
    const Transform3 mirror = Transform3::Reflection(Point3{0.0, 0.0, 2.0}, zAxis);
    const Transform3 samePlane = Transform3::Reflection(Point3{9.0, 8.0, 2.0}, zAxis);

    CHECK(mirror == samePlane);
    CHECK(mirror.TransformPoint(Point3{1.0, 4.0, 2.0}) == Point3{1.0, 4.0, 2.0});
    CHECK(mirror.TransformPoint(Point3{1.0, 4.0, 5.0}) == Point3{1.0, 4.0, -1.0});
    CHECK(mirror.TransformPoint(mirror.TransformPoint(Point3{1.0, 4.0, 5.0}))
          == Point3{1.0, 4.0, 5.0});

    CHECK(mirror * Vector3{0.0, 0.0, 3.0} == Vector3{0.0, 0.0, -3.0});
    CHECK(mirror * Vector3{1.0, 2.0, 0.0} == Vector3{1.0, 2.0, 0.0});
    CHECK(mirror * Vector3{0.0, 0.0, 3.0}
          == Transform3::Reflection(Point3{}, zAxis) * Vector3{0.0, 0.0, 3.0});

    const auto inverse = mirror.Inverse();
    REQUIRE(inverse.has_value());
    CHECK(inverse->TransformPoint(Point3{1.0, 4.0, 5.0})
          == mirror.TransformPoint(Point3{1.0, 4.0, 5.0}));
}

TEST_CASE("reflecting across a diagonal plane flips only the normal component",
          "[linear][transform3]") {
    // 法向 (1, 1, 0)/√2。Z 不在法向里，必须原样留下。
    // (1, 0, 5) 落到 (0, -1, 5)。对角元写成单位矩阵时，这一格失败。
    const UnitVector3 normal = *Vector3{1.0, 1.0, 0.0}.Normalized();
    const Transform3 mirror = Transform3::Reflection(Point3{}, normal);
    const Point3 image = mirror.TransformPoint(Point3{1.0, 0.0, 5.0});

    CHECK(image.X == Approx(0.0).margin(1e-12));
    CHECK(image.Y == Approx(-1.0).margin(1e-12));
    CHECK(image.Z == Approx(5.0).margin(1e-12));
    const Point3 back = mirror.TransformPoint(image);
    CHECK(back.X == Approx(1.0).margin(1e-12));
    CHECK(back.Y == Approx(0.0).margin(1e-12));
    CHECK(back.Z == Approx(5.0).margin(1e-12));
}

TEST_CASE("Transform3 RotationAbout fixes the pivot", "[linear][transform3]") {
    using DragonGeo::Core::HALF_PI;

    const Point3 pivot{1.0, 2.0, 3.0};
    const Transform3 about = Transform3::RotationAbout(pivot, zAxis, HALF_PI);
    CHECK(about.TransformPoint(pivot) == pivot);

    const Point3 tip{2.0, 2.0, 3.0};
    const Point3 rotated = about.TransformPoint(tip);
    CHECK(rotated.X == Approx(1.0).margin(1e-12));
    CHECK(rotated.Y == Approx(3.0).margin(1e-12));
    CHECK(rotated.Z == Approx(3.0).margin(1e-12));
}

TEST_CASE("Transform3 TransformNormal uses inverse transpose", "[linear][transform3]") {
    const Transform3 uniform = Transform3::Scaling(2.0);
    const auto normal = uniform.TransformNormal(zAxis);
    REQUIRE(normal.has_value());
    CHECK(normal->AsVector().X == Approx(0.0).margin(1e-12));
    CHECK(normal->AsVector().Y == Approx(0.0).margin(1e-12));
    CHECK(normal->AsVector().Z == Approx(1.0).margin(1e-12));
    CHECK((uniform * zAxis.AsVector()).Z == Approx(2.0));

    const Transform3 singular = Transform3::Scaling(Vector3{0.0, 1.0, 1.0});
    CHECK_FALSE(singular.TransformNormal(zAxis).has_value());

    const Transform3 nonUniform = Transform3::Scaling(Vector3{2.0, 1.0, 1.0});
    const auto n = nonUniform.TransformNormal(xAxis);
    REQUIRE(n.has_value());
    const auto inverse = nonUniform.Inverse();
    REQUIRE(inverse.has_value());
    const auto roundTrip = inverse->TransformNormal(*n);
    REQUIRE(roundTrip.has_value());
    CHECK(roundTrip->AsVector().X == Approx(1.0).margin(1e-9));
    CHECK(roundTrip->AsVector().Y == Approx(0.0).margin(1e-9));
    CHECK(roundTrip->AsVector().Z == Approx(0.0).margin(1e-9));
}

TEST_CASE("Transform3 RotationQuaternion reads the linear rotation", "[linear][transform3]") {
    const Transform3 rotate = Transform3::Rotation(zAxis, HALF_PI);
    const auto q = rotate.RotationQuaternion();
    REQUIRE(q.has_value());
    const Vector3 tip{1.0, 0.0, 0.0};
    CHECK(q->Rotate(tip).Y == Approx(1.0).margin(1e-12));
    CHECK(q->Rotate(tip).X == Approx(0.0).margin(1e-12));

    const Transform3 scaled = Transform3::Scaling(2.0);
    CHECK_FALSE(scaled.RotationQuaternion().has_value());
}
