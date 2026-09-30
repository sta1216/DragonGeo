#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <type_traits>          // 本任务新用 STATIC_REQUIRE(std::is_same_v<...>)

#include <GeoCore/core/Constants.hpp>
#include <GeoCore/linear/Transform2.hpp>

using Catch::Approx;

using GeoCore::core::half_pi;
using GeoCore::core::quarter_pi;
using GeoCore::linear::Point2;
using GeoCore::linear::Transform2;
using GeoCore::linear::Transform2T;
using GeoCore::linear::Transform2f;
using GeoCore::linear::Vector2;

TEST_CASE("the float alias really is the float instantiation",
          "[linear][transform2]") {
    // 同 vector4_test.cpp：Transform2f 此前零命中，误绑定不会被断言发现。
    STATIC_REQUIRE(std::is_same_v<Transform2f, Transform2T<float>>);
}

TEST_CASE("2D translation moves positions only", "[linear][transform2]") {
    const Transform2 t = Transform2::translation(Vector2{5.0, 7.0});

    CHECK(t.apply(Vector2{1.0, 2.0}) == Vector2{6.0, 9.0});
    CHECK(t * Vector2{1.0, 2.0} == Vector2{1.0, 2.0});
}

TEST_CASE("2D rotation by 90 degrees maps x to y", "[linear][transform2]") {
    const Transform2 t = Transform2::rotation(half_pi);

    const Vector2 rotated = t.apply(Vector2{1.0, 0.0});
    CHECK(rotated.x == Approx(0.0).margin(1e-15));
    CHECK(rotated.y == Approx(1.0));

    // operator* 只施加线性部分：方向同样被旋转，且不引入平移
    const Vector2 direction = t * Vector2{1.0, 0.0};
    CHECK(direction.x == Approx(0.0).margin(1e-15));
    CHECK(direction.y == Approx(1.0));
}

TEST_CASE("2D scaling scales both positions and directions", "[linear][transform2]") {
    // 两个重载：各轴相同与各轴独立
    const Transform2 uniform = Transform2::scaling(2.0);

    CHECK(uniform.apply(Vector2{1.0, 3.0}) == Vector2{2.0, 6.0});
    CHECK(uniform * Vector2{1.0, 3.0} == Vector2{2.0, 6.0});

    const Transform2 non_uniform = Transform2::scaling(Vector2{2.0, 3.0});

    // 各轴输入刻意不相等：对称输入下「写入对角线」与「写入第一列」给出
    // 相同结果，无法互相区分。
    CHECK(non_uniform.apply(Vector2{5.0, 7.0}) == Vector2{10.0, 21.0});
    CHECK(non_uniform * Vector2{5.0, 7.0} == Vector2{10.0, 21.0});
}

TEST_CASE("2D composition applies the right operand first", "[linear][transform2]") {
    const Transform2 move = Transform2::translation(Vector2{1.0, 0.0});
    const Transform2 scale = Transform2::scaling(2.0);

    // 先平移再缩放：(1,1) -> (2,1) -> (4,2)
    CHECK((scale * move).apply(Vector2{1.0, 1.0}) == Vector2{4.0, 2.0});

    // 先缩放再平移：(1,1) -> (2,2) -> (3,2)
    CHECK((move * scale).apply(Vector2{1.0, 1.0}) == Vector2{3.0, 2.0});
}

TEST_CASE("inverse undoes a 2D transform", "[linear][transform2]") {
    const Transform2 t = Transform2::translation(Vector2{5.0, -3.0})
                       * Transform2::rotation(quarter_pi)
                       * Transform2::scaling(Vector2{2.0, 3.0});

    const auto inv = t.inverse();

    REQUIRE(inv.has_value());
    const Vector2 original{1.0, 2.0};
    const Vector2 round_trip = inv->apply(t.apply(original));

    CHECK(round_trip.x == Approx(original.x).margin(1e-12));
    CHECK(round_trip.y == Approx(original.y).margin(1e-12));
}

TEST_CASE("inverse of a degenerate 2D transform is nullopt",
          "[linear][transform2][degenerate]") {
    // 把 y 轴压扁：行列式为 0，不存在逆变换
    const Transform2 flatten = Transform2::scaling(Vector2{1.0, 0.0});

    CHECK_FALSE(flatten.inverse().has_value());
}

TEST_CASE("Transform2 identity leaves a position unchanged", "[linear][transform2]") {
    const Transform2 unit = Transform2::identity();

    CHECK(unit.apply(Vector2{1.0, 2.0}) == Vector2{1.0, 2.0});
}

TEST_CASE("a 2D transform carries points, not just vectors",
          "[linear][transform2]") {
    const Transform2 t = Transform2::translation(Vector2{10.0, 0.0});
    const Point2 p{1.0, 2.0};

    // 点被平移
    STATIC_REQUIRE(std::is_same_v<decltype(t * p), Point2>);
    CHECK(t * p == Point2{11.0, 2.0});
    CHECK(t.transform_point(p) == Point2{11.0, 2.0});

    // 方向不被平移 —— 这是 operator* 与 transform_point 的分工
    CHECK(t * Vector2{1.0, 2.0} == Vector2{1.0, 2.0});
}

TEST_CASE("2D transform_point is the explicit spelling of carrying a position",
          "[linear][transform2]") {
    // 两轴互不相等的缩放 + 两轴互不相等的平移：(x,y) -> (2x+1, 3y+10)。
    // 这些数全部精确可表示，== 是安全的。轴不同、平移列不同，因此任一处轴
    // 错位、系数张冠李戴、丢掉平移列或丢掉线性部分，都会改变某个分量。
    const Transform2 t = Transform2::translation(Vector2{1.0, 10.0})
                       * Transform2::scaling(Vector2{2.0, 3.0});

    // 返回类型是 Point2（不是 Vector2）—— 类型层面的分工也钉住
    STATIC_REQUIRE(std::is_same_v<decltype(t.transform_point(Point2{1.0, 2.0})), Point2>);

    CHECK(t.transform_point(Point2{1.0, 2.0}) == Point2{3.0, 16.0});

    // operator*(Point2) 与 transform_point 是同一件事的两种拼写
    CHECK(t * Point2{1.0, 2.0} == t.transform_point(Point2{1.0, 2.0}));

    // 历史入口 apply() 在相同数值上给出相同的数（只是类型仍是 Vector2）
    CHECK(t.apply(Vector2{1.0, 2.0}) == Vector2{3.0, 16.0});

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
    const Transform2 spin = Transform2::rotation(quarter_pi);
    const Point2 p{1.0, 2.0};

    const Point2 carried = spin.transform_point(p);
    CHECK(carried.x == Approx(-0.7071067811865476).margin(1e-12));
    CHECK(carried.y == Approx(2.1213203435596424).margin(1e-12));

    const Point2 carried_by_operator = spin * p;
    CHECK(carried_by_operator.x == Approx(-0.7071067811865476).margin(1e-12));
    CHECK(carried_by_operator.y == Approx(2.1213203435596424).margin(1e-12));
}

TEST_CASE("2D transform_point is constexpr and noexcept, like apply",
          "[linear][transform2]") {
    // apply() 是 constexpr；数学内容相同的 transform_point 没有理由不能用于
    // 常量表达式 —— 这一格把口径钉住。
    constexpr Transform2 t = Transform2::translation(Vector2{1.0, 10.0})
                           * Transform2::scaling(Vector2{2.0, 3.0});
    constexpr Point2 p{1.0, 2.0};

    STATIC_REQUIRE(t.transform_point(p) == Point2{3.0, 16.0});
    STATIC_REQUIRE(t * p == Point2{3.0, 16.0});

    STATIC_REQUIRE(noexcept(t.transform_point(p)));
    STATIC_REQUIRE(noexcept(t * p));

    // 单位变换的两种拼写都原样送回原点。
    CHECK(Transform2::identity().transform_point(p) == p);
    CHECK(Transform2::identity() * p == p);
}

TEST_CASE("a composed 2D transform and chained member calls are different spellings",
          "[linear][transform2]") {
    // 成员化陷阱（计划 Review Focus 第 6 条）：`apply(a * b, v)` 直译成
    // `a * b.apply(v)` 会**静默编译**，但它算的是 `a * (b.apply(v))` ——
    // operator* 对 Vector 只施加线性部分，**a 的平移被丢掉**。
    //
    // 这一格让 a 与 b **都**带平移与缩放（不像 "2D composition applies the right
    // operand first" 里的缩放没有平移 —— 那里两种写法恰好同值，分不开两者）。
    const Transform2 a = Transform2::translation(Vector2{100.0, 0.0})
                       * Transform2::scaling(Vector2{2.0, 3.0});
    const Transform2 b = Transform2::translation(Vector2{0.0, 10.0})
                       * Transform2::scaling(Vector2{5.0, 7.0});
    const Vector2 v{1.0, 2.0};

    // b(v) = (5, 24)；a(b(v)) = (2*5+100, 3*24) = (110, 72)，全部精确
    CHECK((a * b).apply(v) == Vector2{110.0, 72.0});

    // 陷阱形能编译，但语义不同：a * b.apply(v) 只施加 a 的**线性部分**，
    // 得到 (10, 72) —— 恰好差 a 的平移 (100, 0)。
    CHECK(a * b.apply(v) == Vector2{10.0, 72.0});
    CHECK((a * b).apply(v) != a * b.apply(v));

    // Point 侧没有这个陷阱：`a * b.transform_point(p)` 先算 b.transform_point(p)
    // （完整仿射），再被 operator*(Point2T) 当**位置**施加完整仿射 —— 与复合后的
    // (a * b).transform_point(p) 恒等。本任务新增的 Point 重载把这条路径上的
    // 括号敏感性去掉了（陷阱只剩 Vector 路径）。
    const Point2 p{1.0, 2.0};
    CHECK(a * b.transform_point(p) == (a * b).transform_point(p));
    CHECK((a * b).transform_point(p) == Point2{110.0, 72.0});
}
