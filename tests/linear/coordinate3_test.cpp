#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <limits>
#include <optional>
#include <type_traits>
#include <utility>

#include <GeoCore/core/Constants.hpp>
#include <GeoCore/core/Tolerance.hpp>
#include <GeoCore/linear/Coordinate3.hpp>
#include <GeoCore/linear/Transform3.hpp>

using Catch::Approx;

using GeoCore::core::half_pi;
using GeoCore::core::Tolerance;
using GeoCore::core::two_pi;
using GeoCore::linear::Coordinate3;
using GeoCore::linear::Coordinate3f;
using GeoCore::linear::Coordinate3T;
using GeoCore::linear::Point3;
using GeoCore::linear::Point3T;
using GeoCore::linear::Transform3;
using GeoCore::linear::Transform3T;
using GeoCore::linear::UnitVector3;
using GeoCore::linear::UnitVector3T;
using GeoCore::linear::Vector3;
using GeoCore::linear::Vector3T;

namespace {

/// 世界 z 轴。与 transform3_test.cpp 同一种写法。
const UnitVector3 z_axis = UnitVector3::from_normalized_unchecked(Vector3{0.0, 0.0, 1.0});

/// 由分量直接造单位向量。
///
/// 只用来表达「假设这已经是单位向量」的输入。本文件有多处刻意传入**非**单位
/// 向量（长度校验一节），那时用的是同一个入口 —— `from_normalized_unchecked`
/// 的名字已经把风险写在脸上，而它正是「长度校验必须有」的理由。
UnitVector3 unit(double x, double y, double z) {
    return UnitVector3::from_normalized_unchecked(Vector3{x, y, z});
}

} // namespace

TEST_CASE("the float alias really is the float instantiation", "[linear][coordinate3]") {
    // Review Focus 第 8 条：别名必须被**命名**过，否则绑定写错时连一次实例化都
    // 不会发生（Task 4 实测：`Point3f` 绑成 `Point3T<double>`，全库 134 个用例
    // 全绿）。两个实例化都要钉。
    STATIC_REQUIRE(std::is_same_v<Coordinate3f, Coordinate3T<float>>);
    STATIC_REQUIRE(std::is_same_v<Coordinate3, Coordinate3T<double>>);
    STATIC_REQUIRE(std::is_same_v<Coordinate3T<float>::scalar_type, float>);
    STATIC_REQUIRE(std::is_same_v<Coordinate3T<double>::scalar_type, double>);

    // 本类型是本阶段唯一的**强不变量**类型（Review Focus 第 2 条）：正交性是
    // 坐标系的定义性质，非正交的标架会让经它表达的变换静默地拉伸几何，所以
    // 「没有公开的构造路径」本身就是接口契约，与值语义一样要钉住：
    //   - 没有公开的默认构造；
    //   - 不是聚合体（私有数据成员），逐成员初始化不可达；
    //   - 私有的四元构造 `(origin, x, y, z)` 从外部不可达。
    // 把构造函数改成 public 会让下面每一条由真变假 —— 那就是它们的死亡证明。
    STATIC_REQUIRE(!std::is_default_constructible_v<Coordinate3T<double>>);
    STATIC_REQUIRE(!std::is_aggregate_v<Coordinate3T<double>>);
    STATIC_REQUIRE(!std::is_constructible_v<Coordinate3T<double>, Point3T<double>,
                                            UnitVector3T<double>, UnitVector3T<double>,
                                            UnitVector3T<double>>);

    // float 实例化上同样要钉（Task 4 的教训：只钉一个实例化会漏掉另一个）。
    STATIC_REQUIRE(!std::is_default_constructible_v<Coordinate3T<float>>);
    STATIC_REQUIRE(!std::is_aggregate_v<Coordinate3T<float>>);
    STATIC_REQUIRE(!std::is_constructible_v<Coordinate3T<float>, Point3T<float>,
                                            UnitVector3T<float>, UnitVector3T<float>,
                                            UnitVector3T<float>>);
}

TEST_CASE("identity() is the standard orthonormal frame", "[linear][coordinate3]") {
    // 常量求值，不用类型特征（Review Focus：类型特征代理不了行为）。
    // 去掉 `identity()` 的 constexpr，这一整格直接编译不过。
    constexpr Coordinate3T<double> identity = Coordinate3T<double>::identity();

    STATIC_REQUIRE(identity.origin().x == 0.0);
    STATIC_REQUIRE(identity.origin().y == 0.0);
    STATIC_REQUIRE(identity.origin().z == 0.0);

    STATIC_REQUIRE(identity.x_axis().x() == 1.0);
    STATIC_REQUIRE(identity.x_axis().y() == 0.0);
    STATIC_REQUIRE(identity.x_axis().z() == 0.0);

    STATIC_REQUIRE(identity.y_axis().x() == 0.0);
    STATIC_REQUIRE(identity.y_axis().y() == 1.0);
    STATIC_REQUIRE(identity.y_axis().z() == 0.0);

    STATIC_REQUIRE(identity.z_axis().x() == 0.0);
    STATIC_REQUIRE(identity.z_axis().y() == 0.0);
    STATIC_REQUIRE(identity.z_axis().z() == 1.0);

    // 标准标架下两个方向都是恒等 —— 这条单独钉不住任何东西（全零的常量
    // 写法也能通过），它的价值在于把「identity 就是恒等」这句话写进断言。
    const Point3 point{1.0, 2.0, 3.0};
    CHECK(identity.to_parent(point) == point);
    CHECK(identity.to_local(point) == point);
    CHECK(identity.to_parent(Vector3{4.0, 5.0, 6.0}) == Vector3{4.0, 5.0, 6.0});
    CHECK(identity.to_local(Vector3{4.0, 5.0, 6.0}) == Vector3{4.0, 5.0, 6.0});

    // 同一个标架也可以由 from_axes 造出来，两者必须相等（`==` 被用过）。
    const auto made_by_axes = Coordinate3::from_axes(
        Point3{0.0, 0.0, 0.0}, unit(1.0, 0.0, 0.0), unit(0.0, 1.0, 0.0), unit(0.0, 0.0, 1.0));
    REQUIRE(made_by_axes.has_value());
    CHECK(identity == *made_by_axes);
}

TEST_CASE("a coordinate frame cannot be built from non-orthogonal axes",
          "[linear][coordinate3][degenerate]") {
    const Point3 o{0.0, 0.0, 0.0};
    const auto x = unit(1.0, 0.0, 0.0);
    const auto y = unit(0.0, 1.0, 0.0);
    const auto z = unit(0.0, 0.0, 1.0);

    // 正交 —— 成功
    CHECK(Coordinate3::from_axes(o, x, y, z).has_value());

    // 斜交 —— 必须被拒绝，而不是构造出一个会拉伸几何的“坐标系”
    const auto skewed = unit(0.6, 0.8, 0.0);
    CHECK_FALSE(Coordinate3::from_axes(o, x, skewed, z).has_value());

    // 左手系 —— 同样拒绝
    CHECK_FALSE(Coordinate3::from_axes(o, x, y, -z).has_value());

    // >>> sweep-add
    // 三对点积是三条独立的子表达式，只有 x·y 这一对有证据是不够的：
    // 与 x 斜交的第三条轴、与 y 斜交的第三条轴，各来一份。
    CHECK_FALSE(Coordinate3::from_axes(o, x, y, unit(0.6, 0.0, 0.8)).has_value());
    CHECK_FALSE(Coordinate3::from_axes(o, x, y, unit(0.0, 0.6, 0.8)).has_value());
    // 两轴重合是最极端的斜交：x·y = 1。
    CHECK_FALSE(Coordinate3::from_axes(o, x, x, z).has_value());

    // 轴的**长度**也要校验。`UnitVector3T` 的不变量是弱不变量，
    // `from_normalized_unchecked` 是公开的 —— 不校验长度，`from_axes` 就是一条
    // 公开的、能造出「把几何拉伸 2 倍」的坐标系的路径，而这正是本任务要堵死
    // 的那类失败模式。下面这一对刻意让两边的缩放互相补回叉积长度
    // （x 拉伸 2 倍、y 压缩 2 倍 ⇒ `x × y` 仍是 `z`）：两两点积、定向检查都
    // 看不见它，只有长度校验能拦下。
    CHECK_FALSE(Coordinate3::from_axes(o, unit(2.0, 0.0, 0.0), unit(0.0, 0.5, 0.0), z)
                    .has_value());

    // 三条长度子表达式各自的见证。`1 + 8e-10` 是刻意的：定向检查比较的是
    // `(x × y) · z ≈ 1`，容差 abs + rel·1 ≈ 1e-9；而 |x|² 偏离 1 的幅度是
    // 单轴偏差的两倍（(1+ε)² = 1 + 2ε）。ε = 8e-10 时定向检查通过
    // （8e-10 < 1e-9）、长度检查拒绝（1.6e-9 > 1e-9），于是每一格都只有
    // **一条**子表达式会拒它。
    const double scaled = 1.0 + 8e-10;
    CHECK_FALSE(Coordinate3::from_axes(o, unit(scaled, 0.0, 0.0), y, z).has_value());
    CHECK_FALSE(Coordinate3::from_axes(o, x, unit(0.0, scaled, 0.0), z).has_value());
    CHECK_FALSE(Coordinate3::from_axes(o, x, y, unit(0.0, 0.0, scaled)).has_value());

    // 三条点积子表达式也各要一格**只有它**会拒的证据。上面那些斜交同时破坏了
    // 定向检查（斜交轴自己就偏离了 `x × y`），于是只能证明「至少有一条在干活」。
    // 1e-7 的斜交刚好落在定向检查的盲区里：`(x × y) · z` 对 x·z = δ 的偏差只有
    // δ²/2 ≈ 5e-15，远小于容差 1e-9；长度也仍是 1。三格各自只由对应的点积拒。
    const double sine = 1e-7;
    const double cosine = std::sqrt(1.0 - sine * sine);
    CHECK_FALSE(Coordinate3::from_axes(o, x, unit(sine, cosine, 0.0), z).has_value());
    CHECK_FALSE(Coordinate3::from_axes(o, x, y, unit(sine, 0.0, cosine)).has_value());
    CHECK_FALSE(Coordinate3::from_axes(o, x, y, unit(0.0, sine, cosine)).has_value());
    // 反证：同一组输入把容差放宽到 1e-6 就应当被接受 —— 否则上面三格可能只是
    // 被别的检查（比如某个过紧的阈值）拒掉的。
    const Tolerance loose{1e-6, 1e-6};
    CHECK(Coordinate3::from_axes(o, x, unit(sine, cosine, 0.0), z, loose).has_value());
    CHECK(Coordinate3::from_axes(o, x, y, unit(sine, 0.0, cosine), loose).has_value());
    CHECK(Coordinate3::from_axes(o, x, y, unit(0.0, sine, cosine), loose).has_value());
    // <<< sweep-add
}

TEST_CASE("the orientation check rejects every left-handed arrangement",
          "[linear][coordinate3][degenerate]") {
    // 上面 `-z` 那一格只覆盖一种左手排列。定向检查若写成「取绝对值」或者
    // 「比较 (x × y) · z 的平方」，下面每一格都会被接受。
    const Point3 o{0.0, 0.0, 0.0};
    const auto x = unit(1.0, 0.0, 0.0);
    const auto y = unit(0.0, 1.0, 0.0);
    const auto z = unit(0.0, 0.0, 1.0);

    CHECK_FALSE(Coordinate3::from_axes(o, x, y, -z).has_value());   // z 取反
    CHECK_FALSE(Coordinate3::from_axes(o, y, x, z).has_value());    // x/y 对调
    CHECK_FALSE(Coordinate3::from_axes(o, z, y, x).has_value());    // x/z 对调

    // 反证：它们的右手版本都成功（否则上面三条可能只是被别的检查拒掉）。
    // 循环置换 (x,y,z) → (y,z,x) 是旋转，det = +1，必须成功；
    // 而上面第二格的对调（x/y 互换）才是 det = -1。
    CHECK(Coordinate3::from_axes(o, x, y, z).has_value());
    CHECK(Coordinate3::from_axes(o, y, z, x).has_value());
}

TEST_CASE("coordinate frame round-trips a point", "[linear][coordinate3]") {
    const auto frame = Coordinate3::from_z_axis(
        Point3{10.0, 0.0, 0.0},
        UnitVector3::from_normalized_unchecked(Vector3{0.0, 0.0, 1.0}));
    REQUIRE(frame.has_value());

    const Point3 local{1.0, 2.0, 3.0};
    const Point3 round_trip = frame->to_local(frame->to_parent(local));

    CHECK(round_trip.x == Approx(local.x).margin(1e-12));
    CHECK(round_trip.y == Approx(local.y).margin(1e-12));
    CHECK(round_trip.z == Approx(local.z).margin(1e-12));
}

TEST_CASE("from_z_axis produces a right-handed orthonormal frame",
          "[linear][coordinate3]") {
    // 上面那条往返用例的 z 是世界 z 轴，补全出来的恰好是标准基 ——
    // 于是「旋转矩阵转置了」「左手系」这两种实现都能通过它。这一条改用
    // 非轴向的 z，并把补全出来的轴逐分量钉死。
    //
    // 期望值是手算的：z = (1,1,1)/√3 时三个分量的绝对值并列，选择器取
    // x 方向作参考向量，得 x = (0,-1,1)/√2、y = (2,-1,-1)/√6。
    const auto z = Vector3{1.0, 1.0, 1.0}.normalized();
    REQUIRE(z.has_value());

    const auto frame = Coordinate3::from_z_axis(Point3{0.0, 0.0, 0.0}, *z);
    REQUIRE(frame.has_value());

    const double s2 = std::sqrt(2.0);
    const double s3 = std::sqrt(3.0);
    const double s6 = std::sqrt(6.0);

    CHECK(frame->x_axis().x() == Approx(0.0).margin(1e-15));
    CHECK(frame->x_axis().y() == Approx(-1.0 / s2));
    CHECK(frame->x_axis().z() == Approx(1.0 / s2));

    CHECK(frame->y_axis().x() == Approx(2.0 / s6));
    CHECK(frame->y_axis().y() == Approx(-1.0 / s6));
    CHECK(frame->y_axis().z() == Approx(-1.0 / s6));

    CHECK(frame->z_axis().x() == Approx(1.0 / s3));
    CHECK(frame->z_axis().y() == Approx(1.0 / s3));
    CHECK(frame->z_axis().z() == Approx(1.0 / s3));

    // 直接钉住定义性质，不依赖上面那组手算值。
    const Vector3 xy = frame->x_axis().as_vector().cross(frame->y_axis().as_vector());
    CHECK(xy.x == Approx(frame->z_axis().x()));
    CHECK(xy.y == Approx(frame->z_axis().y()));
    CHECK(xy.z == Approx(frame->z_axis().z()));
}

TEST_CASE("from_z_axis picks a reference axis that cannot degenerate",
          "[linear][coordinate3]") {
    // 补全公式是 x = normalize(reference × z)、y = z × x，而 reference 取
    // 「z 的绝对值最小的那个分量方向」。三条分支各要一格见证：把参考轴固定
    // 成某一根的实现，在另外两格上会给出**另一组仍然正交**的轴 —— 不变量拦不住
    // 它（那不是非正交，只是另一个合法的旋转），只有逐分量钉期望值才看得见。
    const Point3 o{0.0, 0.0, 0.0};

    // 分支 2：|y| 最小 ⇒ reference = (0,1,0)。
    // z = (0.6, 0, 0.8) ⇒ x = (0.8, 0, -0.6)、y = (0, 1, 0)。
    {
        const auto frame = Coordinate3::from_z_axis(o, unit(0.6, 0.0, 0.8));
        REQUIRE(frame.has_value());

        CHECK(frame->x_axis().x() == Approx(0.8).margin(1e-15));
        CHECK(frame->x_axis().y() == Approx(0.0).margin(1e-15));
        CHECK(frame->x_axis().z() == Approx(-0.6).margin(1e-15));

        CHECK(frame->y_axis().x() == Approx(0.0).margin(1e-15));
        CHECK(frame->y_axis().y() == Approx(1.0).margin(1e-15));
        CHECK(frame->y_axis().z() == Approx(0.0).margin(1e-15));
    }

    // 分支 3：|z| 最小 ⇒ reference = (0,0,1)。
    // z = (0.6, 0.8, 0) ⇒ x = (-0.8, 0.6, 0)、y = (0, 0, 1)。
    {
        const auto frame = Coordinate3::from_z_axis(o, unit(0.6, 0.8, 0.0));
        REQUIRE(frame.has_value());

        CHECK(frame->x_axis().x() == Approx(-0.8).margin(1e-15));
        CHECK(frame->x_axis().y() == Approx(0.6).margin(1e-15));
        CHECK(frame->x_axis().z() == Approx(0.0).margin(1e-15));

        CHECK(frame->y_axis().x() == Approx(0.0).margin(1e-15));
        CHECK(frame->y_axis().y() == Approx(0.0).margin(1e-15));
        CHECK(frame->y_axis().z() == Approx(1.0).margin(1e-15));
    }

    // 分支 1 的等号：|x| 与 |y| 并列最小 ⇒ 取 x 方向，而不是 y 方向。
    // z = (1,1,2)/√6 ⇒ x = (0, -2, 1)/√5、y = (5, -1, -2)/√30。
    // 把第一个条件写成 `ax < ay` 会落到分支 2（reference = y），得到
    // x = (2, 0, -1)/√5 —— 与下面的期望值差得很远。
    {
        const auto z = Vector3{1.0, 1.0, 2.0}.normalized();
        REQUIRE(z.has_value());
        const auto frame = Coordinate3::from_z_axis(o, *z);
        REQUIRE(frame.has_value());

        const double s5 = std::sqrt(5.0);
        const double s30 = std::sqrt(30.0);

        CHECK(frame->x_axis().x() == Approx(0.0).margin(1e-15));
        CHECK(frame->x_axis().y() == Approx(-2.0 / s5));
        CHECK(frame->x_axis().z() == Approx(1.0 / s5));

        CHECK(frame->y_axis().x() == Approx(5.0 / s30));
        CHECK(frame->y_axis().y() == Approx(-1.0 / s30));
        CHECK(frame->y_axis().z() == Approx(-2.0 / s30));
    }

    // 第二个条件的等号：|y| 与 |z| 并列最小 ⇒ 取 y 方向，而不是 z 方向。
    // z = (2,1,1)/√6 ⇒ reference = (0,1,0) ⇒ x = (1, 0, -2)/√5、
    // y = (-2, 5, -1)/√30。把第二个条件写成 `ay < az` 会落到分支 3，
    // 得到 x = (-1, 2, 0)/√5、y = (-2, -1, 5)/√30。
    {
        const auto z = Vector3{2.0, 1.0, 1.0}.normalized();
        REQUIRE(z.has_value());
        const auto frame = Coordinate3::from_z_axis(o, *z);
        REQUIRE(frame.has_value());

        const double s5 = std::sqrt(5.0);
        const double s30 = std::sqrt(30.0);

        CHECK(frame->x_axis().x() == Approx(1.0 / s5));
        CHECK(frame->x_axis().y() == Approx(0.0).margin(1e-15));
        CHECK(frame->x_axis().z() == Approx(-2.0 / s5));

        CHECK(frame->y_axis().x() == Approx(-2.0 / s30));
        CHECK(frame->y_axis().y() == Approx(5.0 / s30));
        CHECK(frame->y_axis().z() == Approx(-1.0 / s30));
    }

    // 近轴输入：z 几乎与 z 轴重合（|x| 与 |y| 并列最小 ⇒ 取 x 作参考）。
    // 期望值是精确的：z = (1e-9, 1e-9, 1).normalized() 就是 (1e-9, 1e-9, 1)
    // （长度在 double 下恰好是 1），于是 x = (1,0,0) × z = (0,-1,1e-9)。
    // 固定用 (0,0,1) 作参考会得到 (-1e-9, 1e-9, 0) 归一化 —— 一个长度只有
    // 1.4e-9 的叉积，方向完全不同（x 的 y 分量翻成 +0.707）。z 恰好等于
    // (0,0,1) 时更直接：叉积为零向量，`normalized` 返回 nullopt（见往返用例）。
    {
        const auto z = Vector3{1e-9, 1e-9, 1.0}.normalized();
        REQUIRE(z.has_value());
        const auto frame = Coordinate3::from_z_axis(o, *z);
        REQUIRE(frame.has_value());

        CHECK(frame->x_axis().x() == 0.0);
        CHECK(frame->x_axis().y() == -1.0);
        CHECK(frame->x_axis().z() == 1e-9);
    }
}

TEST_CASE("to_parent and to_local carry the whole frame", "[linear][coordinate3]") {
    // 一个原点非零、三轴都不是标准基的标架。轴取 0/±1 的置换，于是所有
    // 期望值都精确可表示 —— 断言可以要求精确相等。
    //   x = (0,1,0)、y = (-1,0,0)（x 逆时针转 90°）、z = (0,0,1)。
    const auto frame =
        Coordinate3::from_axes(Point3{10.0, 20.0, 30.0}, unit(0.0, 1.0, 0.0),
                               unit(-1.0, 0.0, 0.0), unit(0.0, 0.0, 1.0));
    REQUIRE(frame.has_value());

    // 点：origin + x·lx + y·ly + z·lz。三个分量分别被拉到不同的轴上，
    // 每一分量都必须单独钉（一个把 y 轴写成 x 轴的实现只会在其中一条上现形）。
    CHECK(frame->to_parent(Point3{1.0, 2.0, 3.0}) == Point3{8.0, 21.0, 33.0});
    CHECK(frame->to_parent(Point3{0.0, 0.0, 0.0}) == Point3{10.0, 20.0, 30.0});
    CHECK(frame->to_parent(Point3{1.0, 0.0, 0.0}) == Point3{10.0, 21.0, 30.0});
    CHECK(frame->to_parent(Point3{0.0, 1.0, 0.0}) == Point3{9.0, 20.0, 30.0});
    CHECK(frame->to_parent(Point3{0.0, 0.0, 1.0}) == Point3{10.0, 20.0, 31.0});

    // 点：反方向。注意这里必须减掉 origin —— 不减的话 (8,21,33) 会算出 (21,-8,33)。
    CHECK(frame->to_local(Point3{8.0, 21.0, 33.0}) == Point3{1.0, 2.0, 3.0});
    CHECK(frame->to_local(Point3{10.0, 20.0, 30.0}) == Point3{0.0, 0.0, 0.0});
    CHECK(frame->to_local(Point3{10.0, 21.0, 30.0}) == Point3{1.0, 0.0, 0.0});
    CHECK(frame->to_local(Point3{9.0, 20.0, 30.0}) == Point3{0.0, 1.0, 0.0});
    CHECK(frame->to_local(Point3{10.0, 20.0, 31.0}) == Point3{0.0, 0.0, 1.0});

    // 向量：只有线性部分，**平移不生效**。这里的期望值与上面点的期望值不同
    // 正是证据：把 origin 也加进去的实现会得到 (8,21,33)。
    CHECK(frame->to_parent(Vector3{1.0, 2.0, 3.0}) == Vector3{-2.0, 1.0, 3.0});
    CHECK(frame->to_parent(Vector3{0.0, 0.0, 0.0}) == Vector3{0.0, 0.0, 0.0});
    CHECK(frame->to_parent(Vector3{1.0, 0.0, 0.0}) == Vector3{0.0, 1.0, 0.0});
    CHECK(frame->to_parent(Vector3{0.0, 1.0, 0.0}) == Vector3{-1.0, 0.0, 0.0});
    CHECK(frame->to_parent(Vector3{0.0, 0.0, 1.0}) == Vector3{0.0, 0.0, 1.0});

    // 同上，反方向：不减 origin（减了就得到 (1,2,3)）。
    CHECK(frame->to_local(Vector3{8.0, 21.0, 33.0}) == Vector3{21.0, -8.0, 33.0});
    CHECK(frame->to_local(Vector3{1.0, 0.0, 0.0}) == Vector3{0.0, -1.0, 0.0});
    CHECK(frame->to_local(Vector3{0.0, 1.0, 0.0}) == Vector3{1.0, 0.0, 0.0});
    CHECK(frame->to_local(Vector3{0.0, 0.0, 1.0}) == Vector3{0.0, 0.0, 1.0});

    // 往返：两个方向、两种输入都要。
    const Point3 local{0.25, -1.5, 4.0};
    CHECK(frame->to_parent(frame->to_local(local)) == local);
    const Point3 parent{1.0, 2.0, 3.0};
    CHECK(frame->to_local(frame->to_parent(parent)) == parent);
    const Vector3 local_direction{0.25, -1.5, 4.0};
    CHECK(frame->to_parent(frame->to_local(local_direction)) == local_direction);
    const Vector3 parent_direction{1.0, 2.0, 3.0};
    CHECK(frame->to_local(frame->to_parent(parent_direction)) == parent_direction);

    // >>> sweep-add
    // 上面那个标架有一半系数的**零**：x = (0,1,0)、y = (-1,0,0)、z = (0,0,1)。
    // 于是 `to_parent` 的九项里，像 `z_.x() * local.z` 这样的项恒为零 ——
    // 整个删掉也看不出来（实测：删掉 row x 的 z 项，全文件只有这一处能抓，
    // 而它在补这一格之前**确实是活的**）。换一个九个分量都非零的标架，
    // 它由 (1,1,1)/√3 补全得到：x = (0,-1,1)/√2、y = (2,-1,-1)/√6、
    // z = (1,1,1)/√3。原点取零（原点相关的证据由上面那个标架提供）。
    const auto dense_z = Vector3{1.0, 1.0, 1.0}.normalized();
    REQUIRE(dense_z.has_value());
    const auto dense = Coordinate3::from_z_axis(Point3{0.0, 0.0, 0.0}, *dense_z);
    REQUIRE(dense.has_value());

    const double s2 = std::sqrt(2.0);
    const double s3 = std::sqrt(3.0);
    const double s6 = std::sqrt(6.0);
    // to_parent(1,2,3)：逐行是 x/y/z 三根轴按 local 分量加权之和。
    CHECK(dense->to_parent(Point3{1.0, 2.0, 3.0}).x == Approx(4.0 / s6 + 3.0 / s3));
    CHECK(dense->to_parent(Point3{1.0, 2.0, 3.0}).y ==
          Approx(-1.0 / s2 - 2.0 / s6 + 3.0 / s3));
    CHECK(dense->to_parent(Point3{1.0, 2.0, 3.0}).z ==
          Approx(1.0 / s2 - 2.0 / s6 + 3.0 / s3));
    // to_local(1,2,3)：与三根轴的点积。
    CHECK(dense->to_local(Point3{1.0, 2.0, 3.0}).x == Approx(1.0 / s2));
    CHECK(dense->to_local(Point3{1.0, 2.0, 3.0}).y == Approx(-3.0 / s6));
    CHECK(dense->to_local(Point3{1.0, 2.0, 3.0}).z == Approx(6.0 / s3));
    // 方向重载在同一个标架上同样逐行钉（它的九项与点的九项是各自独立的代码）。
    CHECK(dense->to_parent(Vector3{1.0, 2.0, 3.0}).x == Approx(4.0 / s6 + 3.0 / s3));
    CHECK(dense->to_parent(Vector3{1.0, 2.0, 3.0}).y == Approx(-1.0 / s2 - 2.0 / s6 + 3.0 / s3));
    CHECK(dense->to_parent(Vector3{1.0, 2.0, 3.0}).z == Approx(1.0 / s2 - 2.0 / s6 + 3.0 / s3));
    CHECK(dense->to_local(Vector3{1.0, 2.0, 3.0}).x == Approx(1.0 / s2));
    CHECK(dense->to_local(Vector3{1.0, 2.0, 3.0}).y == Approx(-3.0 / s6));
    CHECK(dense->to_local(Vector3{1.0, 2.0, 3.0}).z == Approx(6.0 / s3));
    // 往返（这个标架下两个方向都要走一遍）。
    const Point3 dense_local{1.0, 2.0, 3.0};
    CHECK(dense->to_parent(dense->to_local(dense_local)).x == Approx(1.0));
    CHECK(dense->to_parent(dense->to_local(dense_local)).y == Approx(2.0));
    CHECK(dense->to_parent(dense->to_local(dense_local)).z == Approx(3.0));
    // <<< sweep-add
}

TEST_CASE("only a rigid transform is a coordinate frame",
          "[linear][coordinate3][degenerate]") {
    // from_transform 若返回裸值，这里就会静默得到一个会拉伸几何的「坐标系」。
    CHECK_FALSE(Coordinate3::from_transform(
                    Transform3::scaling(Vector3{2.0, 3.0, 4.0})).has_value());

    // 均匀缩放同样不是标架 —— 注意不能靠「先把三列归一化再校验」来判：
    // diag(2,3,4) 与 diag(2,2,2) 归一化之后都变成标准基，看起来完全正交，
    // 于是非均匀缩放会静默通过。判据必须是线性部分**本身**正交，而不是
    // 归一化之后的三个方向正交。
    CHECK_FALSE(Coordinate3::from_transform(Transform3::scaling(2.0)).has_value());

    // 刚体变换才是标架。
    const auto rigid = Coordinate3::from_transform(
        Transform3::translation(Vector3{1.0, 2.0, 3.0})
        * Transform3::rotation(z_axis, half_pi));
    REQUIRE(rigid.has_value());
    CHECK(rigid->origin() == Point3{1.0, 2.0, 3.0});

    CHECK(Coordinate3::from_transform(Transform3::identity()).has_value());

    // >>> sweep-add
    // 只有 origin 被钉住是不够的：一个「原点取平移列、三轴直接给标准基」的
    // 实现能通过上面全部四条。下面把旋转出来的轴逐分量钉死。
    REQUIRE(rigid.has_value());
    CHECK(rigid->x_axis().x() == Approx(0.0).margin(1e-15));
    CHECK(rigid->x_axis().y() == Approx(1.0));
    CHECK(rigid->x_axis().z() == Approx(0.0).margin(1e-15));
    CHECK(rigid->y_axis().x() == Approx(-1.0));
    CHECK(rigid->y_axis().y() == Approx(0.0).margin(1e-15));
    CHECK(rigid->y_axis().z() == Approx(0.0).margin(1e-15));
    CHECK(rigid->z_axis().x() == Approx(0.0).margin(1e-15));
    CHECK(rigid->z_axis().y() == Approx(0.0).margin(1e-15));
    CHECK(rigid->z_axis().z() == Approx(1.0));

    // 绕非坐标轴转 120°：x→y→z→x。轴是**列**而不是行，转置与列序错误在这里
    // 都无处可藏（转置会把 x_axis 变成 (0,0,1)）。
    const auto one_third = Vector3{1.0, 1.0, 1.0}.normalized();
    REQUIRE(one_third.has_value());
    const auto turn = Coordinate3::from_transform(
        Transform3::rotation(*one_third, two_pi / 3.0));
    REQUIRE(turn.has_value());
    CHECK(turn->x_axis().x() == Approx(0.0).margin(1e-15));
    CHECK(turn->x_axis().y() == Approx(1.0));
    CHECK(turn->x_axis().z() == Approx(0.0).margin(1e-15));
    CHECK(turn->y_axis().x() == Approx(0.0).margin(1e-15));
    CHECK(turn->y_axis().y() == Approx(0.0).margin(1e-15));
    CHECK(turn->y_axis().z() == Approx(1.0));
    CHECK(turn->z_axis().x() == Approx(1.0));
    CHECK(turn->z_axis().y() == Approx(0.0).margin(1e-15));
    CHECK(turn->z_axis().z() == Approx(0.0).margin(1e-15));

    // 180° 绕 z 轴也是一组正交但非平凡的线性部分（det = +1 ⇒ 是旋转）。
    const auto half_turn = Coordinate3::from_transform(
        Transform3::scaling(Vector3{-1.0, -1.0, 1.0}));
    REQUIRE(half_turn.has_value());
    CHECK(half_turn->x_axis().x() == -1.0);
    CHECK(half_turn->x_axis().y() == 0.0);
    CHECK(half_turn->x_axis().z() == 0.0);
    CHECK(half_turn->y_axis().x() == 0.0);
    CHECK(half_turn->y_axis().y() == -1.0);
    CHECK(half_turn->y_axis().z() == 0.0);
    CHECK(half_turn->z_axis().x() == 0.0);
    CHECK(half_turn->z_axis().y() == 0.0);
    CHECK(half_turn->z_axis().z() == 1.0);

    // 反射：线性部分**是**正交矩阵（AᵀA = I），但 det = -1，不是刚体变换。
    // 若这里放行，`from_transform` 就成了一条公开的、通往左手标架的路径 ——
    // 而 `from_axes` 明确拒绝左手系，同一个不变量不能有两个说法。
    CHECK_FALSE(Coordinate3::from_transform(
                    Transform3::scaling(Vector3{1.0, -1.0, 1.0})).has_value());

    // 剪切：同样是正交之外的一种，且它的三个方向不互相垂直。
    Transform3 shear{};
    shear.matrix.data[0][1] = 0.5;
    CHECK_FALSE(Coordinate3::from_transform(shear).has_value());

    // 非均匀缩放的复合（刚体 ⊗ 缩放不是刚体）。
    CHECK_FALSE(Coordinate3::from_transform(
                    Transform3::translation(Vector3{5.0, 0.0, 0.0})
                    * Transform3::rotation(z_axis, half_pi)
                    * Transform3::scaling(2.0)).has_value());

    // 平移本身是刚体（线性部分是恒等），必须成功，且原点就是平移量。
    const auto moved = Coordinate3::from_transform(
        Transform3::translation(Vector3{-4.0, 7.0, 0.5}));
    REQUIRE(moved.has_value());
    CHECK(moved->origin() == Point3{-4.0, 7.0, 0.5});
    CHECK(moved->x_axis().x() == 1.0);
    CHECK(moved->y_axis().y() == 1.0);
    CHECK(moved->z_axis().z() == 1.0);
    // <<< sweep-add
}

TEST_CASE("from_axes and from_transform thread their tolerance through",
          "[linear][coordinate3][degenerate]") {
    // 容差必须显式传参、不得在函数体内硬编码阈值。这两格的输入相同、
    // 只有容差不同，结果必须不同 —— 阈值一旦被写死，两格就会同向。
    const Point3 o{0.0, 0.0, 0.0};
    const auto x = unit(1.0, 0.0, 0.0);
    const auto y = unit(0.0, 1.0, 0.0);
    const auto z = unit(0.0, 0.0, 1.0);

    // 与 x 夹角 1e-7 的「y 轴」：默认容差（abs = 1e-12）必须拒绝。
    const double sine = 1e-7;
    const double cosine = std::sqrt(1.0 - sine * sine);
    const auto nearly_y = unit(sine, cosine, 0.0);

    CHECK_FALSE(Coordinate3::from_axes(o, x, nearly_y, z).has_value());
    // 显式放宽容差之后同一份输入必须被接受。
    const Tolerance loose{1e-6, 1e-6};
    CHECK(Coordinate3::from_axes(o, x, nearly_y, z, loose).has_value());
    // 默认实参被真的调用过（不传容差的那条路径）—— 上面两格与别处都有。

    // from_transform 同样：线性部分的两列被扰动成 1e-7 的斜交，默认拒绝、
    // 放宽容差后接受。（只改 `data[0][1]` 一处：单位阵的其余项原样保留，
    // 于是两列仍是单位向量 —— 拒绝的理由只有「不正交」这一条。）
    Transform3 perturbed{};
    perturbed.matrix.data[0][1] = -1e-7;

    CHECK_FALSE(Coordinate3::from_transform(perturbed).has_value());
    CHECK(Coordinate3::from_transform(perturbed, loose).has_value());

    // 容差确实被转发到 `Vector3T::normalized`：把单位长度也一并视为零的
    // 荒谬容差会让补全失败，而不是悄悄产出一个标架。
    CHECK_FALSE(Coordinate3::from_z_axis(o, z, Tolerance{1e9, 0.0}).has_value());
    CHECK(Coordinate3::from_z_axis(o, z, loose).has_value());
}

TEST_CASE("a degenerate axis or a non-finite transform is rejected",
          "[linear][coordinate3][degenerate]") {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double infinity = std::numeric_limits<double>::infinity();

    const Point3 o{0.0, 0.0, 0.0};
    const auto x = unit(1.0, 0.0, 0.0);
    const auto y = unit(0.0, 1.0, 0.0);
    const auto z = unit(0.0, 0.0, 1.0);

    // 零向量冒充单位向量：长度检查必须拦下它（否则这个「标架」把几何压成零）。
    CHECK_FALSE(Coordinate3::from_axes(o, unit(0.0, 0.0, 0.0), y, z).has_value());

    // NaN / ±inf 一律拒绝。容差比较碰上非有限值返回 false，于是三条检查
    // 全部失败 —— 但这条路径要真的走过一次才知道。
    CHECK_FALSE(Coordinate3::from_axes(o, unit(nan, 0.0, 0.0), y, z).has_value());
    CHECK_FALSE(Coordinate3::from_axes(o, x, unit(0.0, nan, 0.0), z).has_value());
    CHECK_FALSE(Coordinate3::from_axes(o, x, y, unit(0.0, 0.0, nan)).has_value());
    CHECK_FALSE(Coordinate3::from_axes(o, unit(infinity, 0.0, 0.0), y, z).has_value());
    CHECK_FALSE(Coordinate3::from_axes(o, x, unit(-infinity, 0.0, 0.0), z).has_value());

    // from_z_axis 收到非有限轴时同样返回 nullopt（不是含 NaN 的标架）。
    CHECK_FALSE(Coordinate3::from_z_axis(o, unit(nan, nan, nan)).has_value());
    CHECK_FALSE(Coordinate3::from_z_axis(o, unit(infinity, 0.0, 1.0)).has_value());
    // 非单位的 z 轴也必须被拒：补全出来的 x/y 会被归一化成单位向量，于是
    // 「不校验 z」只剩这一格看得出来 —— 而它就是另一条通往会拉伸几何的标架的
    // 公开路径（补全这一步自身不会失败，这正是必须让 from_axes 兜底的理由）。
    CHECK_FALSE(Coordinate3::from_z_axis(o, unit(0.0, 0.0, 2.0)).has_value());

    // from_transform：线性部分含 NaN。
    Transform3 broken{};
    broken.matrix.data[1][2] = nan;
    CHECK_FALSE(Coordinate3::from_transform(broken).has_value());
}

TEST_CASE("coordinate frame equality has evidence in every member",
          "[linear][coordinate3]") {
    const Point3 o{1.0, 2.0, 3.0};
    const auto x = unit(1.0, 0.0, 0.0);
    const auto y = unit(0.0, 1.0, 0.0);
    const auto z = unit(0.0, 0.0, 1.0);

    const auto base = Coordinate3::from_axes(o, x, y, z);
    REQUIRE(base.has_value());

    CHECK(*base == *base);
    CHECK_FALSE(*base != *base);

    // origin 分量。
    const auto other_origin = Coordinate3::from_axes(Point3{1.0, 2.0, 4.0}, x, y, z);
    REQUIRE(other_origin.has_value());
    CHECK_FALSE(*base == *other_origin);
    CHECK(*base != *other_origin);   // C++20 生成的 != 也要真的被用过

    // 三条轴各要一格「只有它不同」的证据。难点在于：右手正交标架里，任意两条轴
    // 就唯一决定了第三条，所以**精确**只差一条轴的另一个标架并不存在 ——
    // 若只拿「换一组轴」当证据，`==` 里漏掉任意一条轴的比较都看不出来
    // （另外两条不同就足以让结果不等）。
    // 容差松弛区里有这种标架：1e-10 量级的偏移会通过全部校验
    // （长度偏差 2e-10 < 1e-9），但 `==` 是逐分量精确比较，差这一点就不相等。
    // 下面三格分别只动 x / y / z，每格只由那一条轴的比较决定。
    const double slack = 1e-10;
    const auto other_x = Coordinate3::from_axes(o, unit(1.0 + slack, 0.0, 0.0), y, z);
    REQUIRE(other_x.has_value());
    CHECK_FALSE(*base == *other_x);

    const auto other_y = Coordinate3::from_axes(o, x, unit(0.0, 1.0 + slack, 0.0), z);
    REQUIRE(other_y.has_value());
    CHECK_FALSE(*base == *other_y);

    const auto other_z = Coordinate3::from_axes(o, x, y, unit(0.0, 0.0, 1.0 + slack));
    REQUIRE(other_z.has_value());
    CHECK_FALSE(*base == *other_z);
}

TEST_CASE("the implicitly generated special members carry the whole frame",
          "[linear][coordinate3]") {
    const auto source = Coordinate3::from_axes(Point3{1.0, 2.0, 3.0},
                                               unit(0.0, 1.0, 0.0), unit(-1.0, 0.0, 0.0),
                                               unit(0.0, 0.0, 1.0));
    REQUIRE(source.has_value());

    // 拷贝构造。
    const Coordinate3 copied = *source;
    CHECK(copied.origin() == Point3{1.0, 2.0, 3.0});
    CHECK(copied.x_axis() == unit(0.0, 1.0, 0.0));
    CHECK(copied.y_axis() == unit(-1.0, 0.0, 0.0));
    CHECK(copied.z_axis() == unit(0.0, 0.0, 1.0));

    // 移动构造。
    Coordinate3 movable = *source;
    const Coordinate3 moved = std::move(movable);
    CHECK(moved.origin() == Point3{1.0, 2.0, 3.0});
    CHECK(moved.x_axis() == unit(0.0, 1.0, 0.0));
    CHECK(moved.y_axis() == unit(-1.0, 0.0, 0.0));
    CHECK(moved.z_axis() == unit(0.0, 0.0, 1.0));

    // 拷贝赋值：**唯一能被静默弄坏的那个**（Task 4 实测：obj 里连拷贝赋值的
    // 符号都不存在，一个只赋前两个成员的手写 `operator=` 完全静默）。
    // 四个成员各自断言，不能靠 `==` 顺带覆盖。
    const auto target_frame = Coordinate3::from_axes(Point3{9.0, 9.0, 9.0},
                                                     unit(1.0, 0.0, 0.0), unit(0.0, 1.0, 0.0),
                                                     unit(0.0, 0.0, 1.0));
    REQUIRE(target_frame.has_value());
    Coordinate3 assigned = *target_frame;
    assigned = *source;
    CHECK(assigned.origin() == Point3{1.0, 2.0, 3.0});
    CHECK(assigned.x_axis() == unit(0.0, 1.0, 0.0));
    CHECK(assigned.y_axis() == unit(-1.0, 0.0, 0.0));
    CHECK(assigned.z_axis() == unit(0.0, 0.0, 1.0));

    // 移动赋值。
    Coordinate3 move_assigned = *target_frame;
    move_assigned = std::move(assigned);
    CHECK(move_assigned.origin() == Point3{1.0, 2.0, 3.0});
    CHECK(move_assigned.x_axis() == unit(0.0, 1.0, 0.0));
    CHECK(move_assigned.y_axis() == unit(-1.0, 0.0, 0.0));
    CHECK(move_assigned.z_axis() == unit(0.0, 0.0, 1.0));

    // 五个特殊成员的**无行为**是这一类型的规格：本类型没有可观测的析构/拷贝
    // 副作用，所以这几条是**直接陈述规格**，不是用特征代理行为。
    // 措辞收紧（Minor-8）：`= default` 的成员同样会让这几条为真，所以它们钉的是
    // 「平凡 / 可构造 / 可赋值」，不是字面上的「隐式生成」。真正证明「成员被逐位
    // 搬运」的是上面那四组断言；手写一个只赋 `origin_` 的拷贝赋值会被
    // `is_trivially_copyable` 与那四组断言**两路**拦下（实测 S13/J5）。
    STATIC_REQUIRE(std::is_trivially_copyable_v<Coordinate3T<double>>);
    STATIC_REQUIRE(std::is_trivially_destructible_v<Coordinate3T<double>>);
    STATIC_REQUIRE(std::is_copy_constructible_v<Coordinate3T<double>>);
    STATIC_REQUIRE(std::is_move_constructible_v<Coordinate3T<double>>);
    STATIC_REQUIRE(std::is_copy_assignable_v<Coordinate3T<double>>);
    STATIC_REQUIRE(std::is_move_assignable_v<Coordinate3T<double>>);
}

TEST_CASE("the float instantiation is usable", "[linear][coordinate3]") {
    // 别名绑定断言只钉住「名字」，不实例化任何成员。这里把每个成员都在
    // float 上真的用一次 —— 否则 float 实例化的成员连一次编译都不会发生。
    constexpr Coordinate3T<float> identity = Coordinate3T<float>::identity();
    STATIC_REQUIRE(identity.x_axis().x() == 1.0f);
    STATIC_REQUIRE(identity.y_axis().y() == 1.0f);
    STATIC_REQUIRE(identity.z_axis().z() == 1.0f);
    STATIC_REQUIRE(identity.origin().x == 0.0f);
    STATIC_REQUIRE(identity.origin().y == 0.0f);
    STATIC_REQUIRE(identity.origin().z == 0.0f);

    const Point3T<float> origin{1.0f, 2.0f, 3.0f};
    const UnitVector3T<float> fx =
        UnitVector3T<float>::from_normalized_unchecked(Vector3T<float>{0.0f, 1.0f, 0.0f});
    const UnitVector3T<float> fy =
        UnitVector3T<float>::from_normalized_unchecked(Vector3T<float>{-1.0f, 0.0f, 0.0f});
    const UnitVector3T<float> fz =
        UnitVector3T<float>::from_normalized_unchecked(Vector3T<float>{0.0f, 0.0f, 1.0f});

    const auto frame = Coordinate3T<float>::from_axes(origin, fx, fy, fz);
    REQUIRE(frame.has_value());
    CHECK(frame->origin() == origin);
    CHECK(frame->x_axis() == fx);
    CHECK(frame->y_axis() == fy);
    CHECK(frame->z_axis() == fz);
    CHECK(frame->to_parent(Point3T<float>{1.0f, 2.0f, 3.0f}) == Point3T<float>{-1.0f, 3.0f, 6.0f});
    CHECK(frame->to_local(Point3T<float>{-1.0f, 3.0f, 6.0f}) == Point3T<float>{1.0f, 2.0f, 3.0f});
    CHECK(frame->to_parent(Vector3T<float>{1.0f, 2.0f, 3.0f}) == Vector3T<float>{-2.0f, 1.0f, 3.0f});
    CHECK(frame->to_local(Vector3T<float>{-2.0f, 1.0f, 3.0f}) == Vector3T<float>{1.0f, 2.0f, 3.0f});
    CHECK(frame->origin().x == 1.0f);

    const auto from_z = Coordinate3T<float>::from_z_axis(
        origin, UnitVector3T<float>::from_normalized_unchecked(Vector3T<float>{0.0f, 0.0f, 1.0f}));
    REQUIRE(from_z.has_value());
    CHECK(from_z->z_axis().z() == 1.0f);

    const auto from_xform =
        Coordinate3T<float>::from_transform(Transform3T<float>::translation(
            Vector3T<float>{1.0f, 2.0f, 3.0f}));
    REQUIRE(from_xform.has_value());
    CHECK(from_xform->origin() == origin);

    // 默认容差是按 double 定标的：float 上算出来的旋转矩阵（长度² 偏差 ~1e-7）
    // 会被它拒绝。要用 float 标架就显式给出与该精度相称的容差 —— 容差显式传参
    // 的预期后果。这里同时证明了 from_transform 的容差确实生效。
    //
    // 注意下面这条**否定的**断言钉的是**库级**的容差口径（`core::Tolerance` 的
    // 默认值是按 double 定标的），不是 `Coordinate3T` 自己的规格：它记录 1e-7
    // 量级的重建误差落在默认阈值之外这个事实。若将来默认容差改成随标量自适应，
    // 这条失败是**信号**（说明口径变了），不是噪声 —— 到时要改的是一整族 float
    // 调用点，而不只是这一条。另一半（放宽容差后接受）是这一格真正要证明的
    // 「容差被转发」。
    const UnitVector3T<float> fzw =
        UnitVector3T<float>::from_normalized_unchecked(Vector3T<float>{0.0f, 0.0f, 1.0f});
    const Transform3T<float> spin = Transform3T<float>::rotation(fzw, 1.5707963267948966f);
    CHECK_FALSE(Coordinate3T<float>::from_transform(spin).has_value());
    const auto loose = Coordinate3T<float>::from_transform(spin, Tolerance{1e-5, 1e-5});
    REQUIRE(loose.has_value());
    CHECK(loose->x_axis().x() == Approx(0.0f).margin(1e-6f));
    CHECK(loose->x_axis().y() == Approx(1.0f).margin(1e-6f));
    CHECK(loose->y_axis().x() == Approx(-1.0f).margin(1e-6f));
}

TEST_CASE("every declared callable is noexcept", "[linear][coordinate3]") {
    // 三个工厂与全部成员都是 noexcept：它们只做标量算术并返回 optional，
    // 没有任何可能抛出的操作。去掉**任意一处** noexcept 都会单独失败。
    // （`UnitVector3T` 没有公开的默认构造，实参只能靠 `unit()` 造出来。）
    const Point3 arg_origin{};
    const UnitVector3 unit_x = unit(1.0, 0.0, 0.0);
    const UnitVector3 unit_y = unit(0.0, 1.0, 0.0);
    const UnitVector3 unit_z = unit(0.0, 0.0, 1.0);

    STATIC_REQUIRE(noexcept(Coordinate3T<double>::identity()));
    STATIC_REQUIRE(noexcept(Coordinate3T<double>::from_axes(arg_origin, unit_x, unit_y, unit_z)));
    STATIC_REQUIRE(noexcept(
        Coordinate3T<double>::from_axes(arg_origin, unit_x, unit_y, unit_z, Tolerance{})));
    STATIC_REQUIRE(noexcept(Coordinate3T<double>::from_z_axis(arg_origin, unit_z)));
    STATIC_REQUIRE(noexcept(Coordinate3T<double>::from_z_axis(arg_origin, unit_z, Tolerance{})));
    STATIC_REQUIRE(noexcept(Coordinate3T<double>::from_transform(Transform3T<double>{})));
    STATIC_REQUIRE(noexcept(Coordinate3T<double>::from_transform(Transform3T<double>{},
                                                                 Tolerance{})));
    STATIC_REQUIRE(noexcept(Coordinate3T<double>::identity().origin()));
    STATIC_REQUIRE(noexcept(Coordinate3T<double>::identity().x_axis()));
    STATIC_REQUIRE(noexcept(Coordinate3T<double>::identity().y_axis()));
    STATIC_REQUIRE(noexcept(Coordinate3T<double>::identity().z_axis()));
    STATIC_REQUIRE(noexcept(Coordinate3T<double>::identity().to_parent(Point3T<double>{})));
    STATIC_REQUIRE(noexcept(Coordinate3T<double>::identity().to_local(Point3T<double>{})));
    STATIC_REQUIRE(noexcept(Coordinate3T<double>::identity().to_parent(Vector3T<double>{})));
    STATIC_REQUIRE(noexcept(Coordinate3T<double>::identity().to_local(Vector3T<double>{})));
    STATIC_REQUIRE(noexcept(Coordinate3T<double>::identity() == Coordinate3T<double>::identity()));
    // 这一条相对上一条是**零独立证据**：C++20 的 `!=` 是 `!(a == b)` 的重写，
    // noexcept 规格继承自被重写的 `operator==`。保留它只为把「生成的 != 真的
    // 被用过」写出来，不要把它计入覆盖率。
    STATIC_REQUIRE(noexcept(Coordinate3T<double>::identity() != Coordinate3T<double>::identity()));
}

TEST_CASE("every callable is usable in a constant expression",
          "[linear][coordinate3]") {
    // 除了 `from_z_axis`（它经 `Vector3T::normalized`，那条路径不是 constexpr），
    // 其余可调用实体都能在常量表达式里求值。删掉**任意一处** constexpr，
    // 这一格就编译不过 —— 这是那些 constexpr 唯一的见证。
    constexpr Point3T<double> origin{1.0, 2.0, 3.0};
    constexpr UnitVector3T<double> cx =
        UnitVector3T<double>::from_normalized_unchecked(Vector3T<double>{0.0, 1.0, 0.0});
    constexpr UnitVector3T<double> cy =
        UnitVector3T<double>::from_normalized_unchecked(Vector3T<double>{-1.0, 0.0, 0.0});
    constexpr UnitVector3T<double> cz =
        UnitVector3T<double>::from_normalized_unchecked(Vector3T<double>{0.0, 0.0, 1.0});

    constexpr auto frame = Coordinate3T<double>::from_axes(origin, cx, cy, cz);
    STATIC_REQUIRE(frame.has_value());
    STATIC_REQUIRE(frame->origin().x == 1.0);
    STATIC_REQUIRE(frame->x_axis().y() == 1.0);
    STATIC_REQUIRE(frame->y_axis().x() == -1.0);
    STATIC_REQUIRE(frame->z_axis().z() == 1.0);

    // 被拒的那一支也要在常量表达式里走一次（`nullopt` 也是常量表达式的结果）。
    constexpr auto refused = Coordinate3T<double>::from_axes(origin, cx, cy, -cz);
    STATIC_REQUIRE_FALSE(refused.has_value());

    STATIC_REQUIRE(frame->to_parent(Point3T<double>{1.0, 2.0, 3.0}) ==
                   Point3T<double>{-1.0, 3.0, 6.0});
    STATIC_REQUIRE(frame->to_local(Point3T<double>{-1.0, 3.0, 6.0}) ==
                   Point3T<double>{1.0, 2.0, 3.0});
    STATIC_REQUIRE(frame->to_parent(Vector3T<double>{1.0, 2.0, 3.0}) ==
                   Vector3T<double>{-2.0, 1.0, 3.0});
    STATIC_REQUIRE(frame->to_local(Vector3T<double>{-2.0, 1.0, 3.0}) ==
                   Vector3T<double>{1.0, 2.0, 3.0});
    STATIC_REQUIRE(*frame == *frame);

    constexpr auto xform = Coordinate3T<double>::from_transform(Transform3T<double>::identity());
    STATIC_REQUIRE(xform.has_value());
    STATIC_REQUIRE(xform->origin().x == 0.0);

    constexpr auto moved =
        Coordinate3T<double>::from_transform(Transform3T<double>::translation(
            Vector3T<double>{4.0, 5.0, 6.0}));
    STATIC_REQUIRE(moved.has_value());
    STATIC_REQUIRE(moved->origin().z == 6.0);
}
