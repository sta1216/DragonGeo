#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <type_traits>          // 本任务新用 STATIC_REQUIRE(std::is_same_v<...>)

#include <GeoCore/core/Constants.hpp>
#include <GeoCore/linear/Transform3.hpp>

using Catch::Approx;

using GeoCore::core::half_pi;
using GeoCore::linear::Point3;
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

TEST_CASE("a transform carries points, not just vectors",
          "[linear][transform3]") {
    const Transform3 t = Transform3::translation(Vector3{10.0, 0.0, 0.0});
    const Point3 p{1.0, 2.0, 3.0};

    // 点被平移
    STATIC_REQUIRE(std::is_same_v<decltype(t * p), Point3>);
    CHECK(t * p == Point3{11.0, 2.0, 3.0});
    CHECK(t.transform_point(p) == Point3{11.0, 2.0, 3.0});

    // 方向不被平移 —— 这是 operator* 与 transform_point 的分工
    CHECK(t * Vector3{1.0, 2.0, 3.0} == Vector3{1.0, 2.0, 3.0});
}

TEST_CASE("transform_point is the explicit spelling of carrying a position",
          "[linear][transform3]") {
    // 各轴互不相等的缩放 + 各轴互不相等的平移：(x,y,z) -> (2x+1, 3y+10, 4z+100)。
    // 这些数全部精确可表示，== 是安全的。三根轴各不同、平移列各不同，因此
    // 任一处轴错位、系数张冠李戴、丢掉平移列或丢掉线性部分，都会改变某个分量。
    const Transform3 t = Transform3::translation(Vector3{1.0, 10.0, 100.0})
                       * Transform3::scaling(Vector3{2.0, 3.0, 4.0});

    // 返回类型是 Point3（不是 Vector3）—— 类型层面的分工也钉住
    STATIC_REQUIRE(std::is_same_v<decltype(t.transform_point(Point3{1.0, 2.0, 3.0})), Point3>);

    CHECK(t.transform_point(Point3{1.0, 2.0, 3.0}) == Point3{3.0, 16.0, 112.0});

    // operator*(Point3) 与 transform_point 是同一件事的两种拼写
    CHECK(t * Point3{1.0, 2.0, 3.0} == t.transform_point(Point3{1.0, 2.0, 3.0}));

    // 历史入口 apply() 在相同数值上给出相同的数（只是类型仍是 Vector3）
    CHECK(t.apply(Vector3{1.0, 2.0, 3.0}) == Vector3{3.0, 16.0, 112.0});

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
    const Transform3 spin = Transform3::rotation(z_axis, half_pi);
    const Point3 p{1.0, 2.0, 3.0};

    const Point3 carried = spin.transform_point(p);
    CHECK(carried.x == Approx(-2.0).margin(1e-12));
    CHECK(carried.y == Approx(1.0).margin(1e-12));
    CHECK(carried.z == Approx(3.0).margin(1e-12));

    const Point3 carried_by_operator = spin * p;
    CHECK(carried_by_operator.x == Approx(-2.0).margin(1e-12));
    CHECK(carried_by_operator.y == Approx(1.0).margin(1e-12));
    CHECK(carried_by_operator.z == Approx(3.0).margin(1e-12));
}

TEST_CASE("transform_point is constexpr and noexcept, like apply",
          "[linear][transform3]") {
    // apply() 是 constexpr；数学内容相同的 transform_point 没有理由不能用于
    // 常量表达式 —— 这一格把口径钉住。
    constexpr Transform3 t = Transform3::translation(Vector3{1.0, 10.0, 100.0})
                           * Transform3::scaling(Vector3{2.0, 3.0, 4.0});
    constexpr Point3 p{1.0, 2.0, 3.0};

    STATIC_REQUIRE(t.transform_point(p) == Point3{3.0, 16.0, 112.0});
    STATIC_REQUIRE(t * p == Point3{3.0, 16.0, 112.0});

    STATIC_REQUIRE(noexcept(t.transform_point(p)));
    STATIC_REQUIRE(noexcept(t * p));

    // 单位变换的两种拼写都原样送回原点。
    CHECK(Transform3::identity().transform_point(p) == p);
    CHECK(Transform3::identity() * p == p);
}

TEST_CASE("a composed transform and chained member calls are different spellings",
          "[linear][transform3]") {
    // 成员化陷阱（计划 Review Focus 第 6 条）：`apply(a * b, v)` 直译成
    // `a * b.apply(v)` 会**静默编译**，但它算的是 `a * (b.apply(v))` ——
    // operator* 对 Vector 只施加线性部分，**a 的平移被丢掉**。
    //
    // 这一格让 a 与 b **都**带平移与缩放（不像 "composition applies the right
    // operand first" 里的 spin 没有平移 —— 那里两种写法恰好同值，分不开两者）。
    const Transform3 a = Transform3::translation(Vector3{100.0, 0.0, 0.0})
                       * Transform3::scaling(Vector3{2.0, 3.0, 4.0});
    const Transform3 b = Transform3::translation(Vector3{0.0, 10.0, 20.0})
                       * Transform3::scaling(Vector3{5.0, 6.0, 7.0});
    const Vector3 v{1.0, 2.0, 3.0};

    // b(v) = (5, 22, 41)；a(b(v)) = (2*5+100, 3*22, 4*41) = (110, 66, 164)，全部精确
    CHECK((a * b).apply(v) == Vector3{110.0, 66.0, 164.0});

    // 陷阱形能编译，但语义不同：a * b.apply(v) 只施加 a 的**线性部分**
    // （b.apply(v) 先按位置算完整仿射，随后 a 当方向对待），
    // 得到 (10, 66, 164) —— 恰好差 a 的平移 (100, 0, 0)。
    CHECK(a * b.apply(v) == Vector3{10.0, 66.0, 164.0});
    CHECK((a * b).apply(v) != a * b.apply(v));

    // Point 侧没有这个陷阱：`a * b.transform_point(p)` 先算 b.transform_point(p)
    // （完整仿射），再被 operator*(Point3T) 当**位置**施加完整仿射 —— 与复合后的
    // (a * b).transform_point(p) 恒等。本任务新增的 Point 重载把这条路径上的
    // 括号敏感性去掉了（陷阱只剩 Vector 路径）。
    const Point3 p{1.0, 2.0, 3.0};
    CHECK(a * b.transform_point(p) == (a * b).transform_point(p));
    CHECK((a * b).transform_point(p) == Point3{110.0, 66.0, 164.0});
}
