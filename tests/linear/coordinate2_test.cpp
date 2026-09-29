#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <limits>
#include <optional>
#include <type_traits>
#include <utility>

#include <GeoCore/core/Constants.hpp>
#include <GeoCore/core/Tolerance.hpp>
#include <GeoCore/linear/Coordinate2.hpp>
#include <GeoCore/linear/Transform2.hpp>

using Catch::Approx;

using GeoCore::core::half_pi;
using GeoCore::core::Tolerance;
using GeoCore::linear::Coordinate2;
using GeoCore::linear::Coordinate2f;
using GeoCore::linear::Coordinate2T;
using GeoCore::linear::Point2;
using GeoCore::linear::Point2T;
using GeoCore::linear::Transform2;
using GeoCore::linear::Transform2T;
using GeoCore::linear::UnitVector2;
using GeoCore::linear::UnitVector2T;
using GeoCore::linear::Vector2;
using GeoCore::linear::Vector2T;

namespace {

/// 世界 x 轴。二维的补全以 x 为主轴（`from_x_axis`），与三维的 `from_z_axis`
/// 对应 —— 两个文件的这个常量因此也不同名。
const UnitVector2 x_axis = UnitVector2::from_normalized_unchecked(Vector2{1.0, 0.0});

/// 由分量直接造单位向量（同 coordinate3_test.cpp）。
UnitVector2 unit(double x, double y) {
    return UnitVector2::from_normalized_unchecked(Vector2{x, y});
}

} // namespace

TEST_CASE("the float alias really is the float instantiation", "[linear][coordinate2]") {
    // 与三维逐条对齐（两个头文件独立编写，强度容易一边倒：Task 4 的 3D `==`
    // 有三个方向的断言、2D 只有一个，这个差异存在了四轮）。
    STATIC_REQUIRE(std::is_same_v<Coordinate2f, Coordinate2T<float>>);
    STATIC_REQUIRE(std::is_same_v<Coordinate2, Coordinate2T<double>>);
    STATIC_REQUIRE(std::is_same_v<Coordinate2T<float>::scalar_type, float>);
    STATIC_REQUIRE(std::is_same_v<Coordinate2T<double>::scalar_type, double>);

    // 强不变量类型：没有任何公开的构造路径。
    STATIC_REQUIRE(!std::is_default_constructible_v<Coordinate2T<double>>);
    STATIC_REQUIRE(!std::is_aggregate_v<Coordinate2T<double>>);
    STATIC_REQUIRE(!std::is_constructible_v<Coordinate2T<double>, Point2T<double>,
                                            UnitVector2T<double>, UnitVector2T<double>>);

    STATIC_REQUIRE(!std::is_default_constructible_v<Coordinate2T<float>>);
    STATIC_REQUIRE(!std::is_aggregate_v<Coordinate2T<float>>);
    STATIC_REQUIRE(!std::is_constructible_v<Coordinate2T<float>, Point2T<float>,
                                            UnitVector2T<float>, UnitVector2T<float>>);
}

TEST_CASE("identity() is the standard orthonormal frame", "[linear][coordinate2]") {
    constexpr Coordinate2T<double> identity = Coordinate2T<double>::identity();

    STATIC_REQUIRE(identity.origin().x == 0.0);
    STATIC_REQUIRE(identity.origin().y == 0.0);

    STATIC_REQUIRE(identity.x_axis().x() == 1.0);
    STATIC_REQUIRE(identity.x_axis().y() == 0.0);

    STATIC_REQUIRE(identity.y_axis().x() == 0.0);
    STATIC_REQUIRE(identity.y_axis().y() == 1.0);

    const Point2 point{1.0, 2.0};
    CHECK(identity.to_parent(point) == point);
    CHECK(identity.to_local(point) == point);
    CHECK(identity.to_parent(Vector2{3.0, 4.0}) == Vector2{3.0, 4.0});
    CHECK(identity.to_local(Vector2{3.0, 4.0}) == Vector2{3.0, 4.0});

    const auto made_by_axes =
        Coordinate2::from_axes(Point2{0.0, 0.0}, unit(1.0, 0.0), unit(0.0, 1.0));
    REQUIRE(made_by_axes.has_value());
    CHECK(identity == *made_by_axes);
}

TEST_CASE("a coordinate frame cannot be built from non-orthogonal axes",
          "[linear][coordinate2][degenerate]") {
    const Point2 o{0.0, 0.0};
    const auto x = unit(1.0, 0.0);
    const auto y = unit(0.0, 1.0);

    // 正交 —— 成功
    CHECK(Coordinate2::from_axes(o, x, y).has_value());

    // 斜交 —— 必须被拒绝，而不是构造出一个会拉伸几何的“坐标系”
    const auto skewed = unit(0.6, 0.8);
    CHECK_FALSE(Coordinate2::from_axes(o, x, skewed).has_value());

    // 左手系（顺时针）—— 同样拒绝。二维没有第三根轴，等价的说法是
    // `x × y` 的标量叉积为负。三种排列各一条，与三维那边的三种对齐。
    CHECK_FALSE(Coordinate2::from_axes(o, x, -y).has_value());
    CHECK_FALSE(Coordinate2::from_axes(o, -x, y).has_value());
    CHECK_FALSE(Coordinate2::from_axes(o, y, x).has_value());

    // 两轴重合是最极端的斜交：x·y = 1。
    CHECK_FALSE(Coordinate2::from_axes(o, x, x).has_value());

    // >>> sweep-add
    // 轴的**长度**也要校验（`from_normalized_unchecked` 是公开的，不校验就等于
    // 留了一条能造出「拉伸 2 倍」的坐标系的路径）。这一对刻意让叉积保持为正：
    // 二维的叉积检查只看符号，拉伸/压缩在它眼里完全一样 —— 只有长度校验能拦下。
    CHECK_FALSE(Coordinate2::from_axes(o, unit(2.0, 0.0), unit(0.0, 0.5)).has_value());

    // 两条长度子表达式各自的见证：1e-6 的偏移让 |x|² 偏离 1 达 2e-6
    // （远大于长度校验 abs + rel ≈ 1e-9），而叉积仍然为正 —— 每格只有一条
    // 子表达式会拒它。二维这一格比三维更好钉：叉积是纯符号判断，不像三维的
    // `(x × y) · z ≈ 1` 那样会跟着长度一起变，所以偏移量不必卡在两个容差之间。
    const double scaled = 1.0 + 1e-6;
    CHECK_FALSE(Coordinate2::from_axes(o, unit(scaled, 0.0), y).has_value());
    CHECK_FALSE(Coordinate2::from_axes(o, x, unit(0.0, scaled)).has_value());
    // <<< sweep-add
}

TEST_CASE("coordinate frame round-trips a point", "[linear][coordinate2]") {
    const auto frame = Coordinate2::from_x_axis(Point2{10.0, 0.0}, x_axis);
    const Point2 local{1.0, 2.0};
    const Point2 round_trip = frame.to_local(frame.to_parent(local));

    CHECK(round_trip.x == Approx(local.x).margin(1e-12));
    CHECK(round_trip.y == Approx(local.y).margin(1e-12));
}

TEST_CASE("from_x_axis completes y as x rotated 90 degrees counter-clockwise",
          "[linear][coordinate2]") {
    // 上面那条往返用例的 x 是世界 x 轴，补全出来的恰好是标准基 ——
    // 于是「y 写成 (-x.y, -x.x)（顺时针）」「y 就是 x」这两种实现都能通过它。
    // 这一条改用非轴向的 x，并把补全出来的轴逐分量钉死。
    //
    // 期望值：x = (0.6, 0.8) ⇒ y = (-x.y, x.x) = (-0.8, 0.6)。
    const auto frame = Coordinate2::from_x_axis(Point2{0.0, 0.0}, unit(0.6, 0.8));

    CHECK(frame.x_axis().x() == Approx(0.6));
    CHECK(frame.x_axis().y() == Approx(0.8));
    CHECK(frame.y_axis().x() == Approx(-0.8));
    CHECK(frame.y_axis().y() == Approx(0.6));

    // 定义性质，不依赖上面那组值：x × y = +1。
    CHECK(frame.x_axis().cross(frame.y_axis()) == Approx(1.0));

    // 原点必须原样带过来（不能总是 (0,0)）。
    const auto moved = Coordinate2::from_x_axis(Point2{10.0, 20.0}, unit(1.0, 0.0));
    CHECK(moved.origin() == Point2{10.0, 20.0});
    CHECK(moved.y_axis().x() == Approx(0.0));
    CHECK(moved.y_axis().y() == Approx(1.0));

    // 由 x 轴负方向出发（x = (-1,0)）⇒ y = (0,-1)：这是「y 恒等于 (0,1)」
    // 这类实现唯一的一格反例。
    const auto flipped = Coordinate2::from_x_axis(Point2{0.0, 0.0}, unit(-1.0, 0.0));
    CHECK(flipped.x_axis().x() == Approx(-1.0));
    CHECK(flipped.x_axis().y() == Approx(0.0).margin(1e-15));
    CHECK(flipped.y_axis().x() == Approx(0.0).margin(1e-15));
    CHECK(flipped.y_axis().y() == Approx(-1.0));
}

TEST_CASE("to_parent and to_local carry the whole frame", "[linear][coordinate2]") {
    // 原点非零、两轴都不是标准基的标架；轴取 0/±1 的置换，期望值精确可表示。
    //   x = (0,1)、y = (-1,0)（x 逆时针转 90°）。
    const auto frame =
        Coordinate2::from_axes(Point2{10.0, 20.0}, unit(0.0, 1.0), unit(-1.0, 0.0));
    REQUIRE(frame.has_value());

    // 点：origin + x·lx + y·ly。两个分量分别被拉到不同的轴上，各要一条证据
    // （把 y 轴写成 x 轴的实现只会在其中一条上现形）。
    CHECK(frame->to_parent(Point2{1.0, 2.0}) == Point2{8.0, 21.0});
    CHECK(frame->to_parent(Point2{0.0, 0.0}) == Point2{10.0, 20.0});
    CHECK(frame->to_parent(Point2{1.0, 0.0}) == Point2{10.0, 21.0});
    CHECK(frame->to_parent(Point2{0.0, 1.0}) == Point2{9.0, 20.0});

    // 反方向必须减掉 origin（不减的话 (8,21) 会算出 (21,-8)）。
    CHECK(frame->to_local(Point2{8.0, 21.0}) == Point2{1.0, 2.0});
    CHECK(frame->to_local(Point2{10.0, 20.0}) == Point2{0.0, 0.0});
    CHECK(frame->to_local(Point2{10.0, 21.0}) == Point2{1.0, 0.0});
    CHECK(frame->to_local(Point2{9.0, 20.0}) == Point2{0.0, 1.0});

    // 向量：只有线性部分，**平移不生效**（期望值与点的不同正是证据）。
    CHECK(frame->to_parent(Vector2{1.0, 2.0}) == Vector2{-2.0, 1.0});
    CHECK(frame->to_parent(Vector2{0.0, 0.0}) == Vector2{0.0, 0.0});
    CHECK(frame->to_parent(Vector2{1.0, 0.0}) == Vector2{0.0, 1.0});
    CHECK(frame->to_parent(Vector2{0.0, 1.0}) == Vector2{-1.0, 0.0});

    CHECK(frame->to_local(Vector2{8.0, 21.0}) == Vector2{21.0, -8.0});
    CHECK(frame->to_local(Vector2{1.0, 0.0}) == Vector2{0.0, -1.0});
    CHECK(frame->to_local(Vector2{0.0, 1.0}) == Vector2{1.0, 0.0});

    // 往返：两个方向、两种输入都要。
    const Point2 local{0.25, -1.5};
    CHECK(frame->to_parent(frame->to_local(local)) == local);
    const Point2 parent{1.0, 2.0};
    CHECK(frame->to_local(frame->to_parent(parent)) == parent);
    const Vector2 local_direction{0.25, -1.5};
    CHECK(frame->to_parent(frame->to_local(local_direction)) == local_direction);
    const Vector2 parent_direction{1.0, 2.0};
    CHECK(frame->to_local(frame->to_parent(parent_direction)) == parent_direction);

    // >>> sweep-add
    // 上面那个标架的两根轴各有一个零分量（x = (0,1)、y = (-1,0)），于是
    // `to_parent` 四项里的 `x_.x() * local.x` 与 `y_.y() * local.y` 恒为零 ——
    // 整个删掉也看不出来。换一个四个系数都非零的标架（x = (0.6,0.8)，
    // y = (-0.8,0.6) 由补全得到，原点 (10,20)）。
    const auto dense = Coordinate2::from_x_axis(Point2{10.0, 20.0}, unit(0.6, 0.8));

    // 10 + 0.6·1 + (-0.8)·2 = 9 ／ 20 + 0.8·1 + 0.6·2 = 22
    CHECK(dense.to_parent(Point2{1.0, 2.0}).x == Approx(9.0));
    CHECK(dense.to_parent(Point2{1.0, 2.0}).y == Approx(22.0));
    // 反方向：offset = (9,22) - (10,20) = (-1,2) ⇒ (1,2)
    CHECK(dense.to_local(Point2{9.0, 22.0}).x == Approx(1.0));
    CHECK(dense.to_local(Point2{9.0, 22.0}).y == Approx(2.0));
    // 方向重载：⊥ 平移。0.6·1 - 0.8·2 = -1 ／ 0.8·1 + 0.6·2 = 2
    CHECK(dense.to_parent(Vector2{1.0, 2.0}).x == Approx(-1.0));
    CHECK(dense.to_parent(Vector2{1.0, 2.0}).y == Approx(2.0));
    // 0.6·9 + 0.8·22 = 23 ／ -0.8·9 + 0.6·22 = 6
    CHECK(dense.to_local(Vector2{9.0, 22.0}).x == Approx(23.0));
    CHECK(dense.to_local(Vector2{9.0, 22.0}).y == Approx(6.0));
    // 往返（这个标架下两个方向都要走一遍）。
    CHECK(dense.to_parent(dense.to_local(Point2{1.0, 2.0})).x == Approx(1.0));
    CHECK(dense.to_parent(dense.to_local(Point2{1.0, 2.0})).y == Approx(2.0));
    // <<< sweep-add
}

TEST_CASE("only a rigid transform is a coordinate frame",
          "[linear][coordinate2][degenerate]") {
    // from_transform 若返回裸值，这里就会静默得到一个会拉伸几何的「坐标系」。
    CHECK_FALSE(Coordinate2::from_transform(Transform2::scaling(Vector2{2.0, 3.0})).has_value());

    // 均匀缩放同样不是标架。二维这一格特别要紧：**叉积检查只看符号**，
    // 均匀缩放不会改变符号，于是「先把两列归一化再校验」的实现照样放行 ——
    // 判据必须是线性部分**本身**正交。
    CHECK_FALSE(Coordinate2::from_transform(Transform2::scaling(2.0)).has_value());

    // 刚体变换才是标架。
    const auto rigid = Coordinate2::from_transform(
        Transform2::translation(Vector2{1.0, 2.0}) * Transform2::rotation(half_pi));
    REQUIRE(rigid.has_value());
    CHECK(rigid->origin() == Point2{1.0, 2.0});

    CHECK(Coordinate2::from_transform(Transform2::identity()).has_value());

    // >>> sweep-add
    // 只有 origin 被钉住是不够的：一个「原点取平移列、两轴直接给标准基」的实现
    // 能通过上面全部四条。把旋转出来的轴逐分量钉死。
    CHECK(rigid->x_axis().x() == Approx(0.0).margin(1e-15));
    CHECK(rigid->x_axis().y() == Approx(1.0));
    CHECK(rigid->y_axis().x() == Approx(-1.0));
    CHECK(rigid->y_axis().y() == Approx(0.0).margin(1e-15));

    // 180° 旋转（det = +1、坐标全为 ±1，精确可表示）。
    const auto half_turn = Coordinate2::from_transform(Transform2::scaling(Vector2{-1.0, -1.0}));
    REQUIRE(half_turn.has_value());
    CHECK(half_turn->x_axis().x() == -1.0);
    CHECK(half_turn->x_axis().y() == 0.0);
    CHECK(half_turn->y_axis().x() == 0.0);
    CHECK(half_turn->y_axis().y() == -1.0);

    // 反射：线性部分**是**正交矩阵（AᵀA = I），但 det = -1，不是刚体变换。
    // 若这里放行，`from_transform` 就成了一条公开的、通往左手标架的路径 ——
    // 而 `from_axes` 明确拒绝左手系，同一个不变量不能有两个说法。
    CHECK_FALSE(Coordinate2::from_transform(Transform2::scaling(Vector2{1.0, -1.0})).has_value());

    // 剪切：同样在正交之外，且两列不互相垂直。
    Transform2 shear{};
    shear.matrix.data[0][1] = 0.5;
    CHECK_FALSE(Coordinate2::from_transform(shear).has_value());

    // 非均匀缩放的复合（刚体 ⊗ 缩放不是刚体）。
    CHECK_FALSE(Coordinate2::from_transform(Transform2::translation(Vector2{5.0, 0.0})
                                            * Transform2::rotation(half_pi)
                                            * Transform2::scaling(2.0))
                    .has_value());

    // 平移本身是刚体（线性部分是恒等），必须成功，且原点就是平移量。
    const auto moved = Coordinate2::from_transform(Transform2::translation(Vector2{-4.0, 7.0}));
    REQUIRE(moved.has_value());
    CHECK(moved->origin() == Point2{-4.0, 7.0});
    CHECK(moved->x_axis().x() == 1.0);
    CHECK(moved->y_axis().y() == 1.0);
    // <<< sweep-add
}

TEST_CASE("from_axes and from_transform thread their tolerance through",
          "[linear][coordinate2][degenerate]") {
    // 容差必须显式传参、不得在函数体内硬编码阈值。同一份输入、只有容差不同，
    // 结果必须不同 —— 阈值一旦被写死，两格就会同向。
    const Point2 o{0.0, 0.0};
    const auto x = unit(1.0, 0.0);
    const auto y = unit(0.0, 1.0);

    const double sine = 1e-7;
    const double cosine = std::sqrt(1.0 - sine * sine);
    const auto nearly_y = unit(sine, cosine);

    CHECK_FALSE(Coordinate2::from_axes(o, x, nearly_y).has_value());
    const Tolerance loose{1e-6, 1e-6};
    CHECK(Coordinate2::from_axes(o, x, nearly_y, loose).has_value());

    // from_transform 同样：线性部分的两列被扰动成 1e-7 的斜交，默认拒绝、
    // 放宽容差后接受。（只改 `data[0][1]` 一处：单位阵的其余项原样保留，
    // 于是两列仍是单位向量 —— 拒绝的理由只有「不正交」这一条。）
    Transform2 perturbed{};
    perturbed.matrix.data[0][1] = -1e-7;

    CHECK_FALSE(Coordinate2::from_transform(perturbed).has_value());
    CHECK(Coordinate2::from_transform(perturbed, loose).has_value());

    // `from_x_axis` 是 noexcept 返回裸值的那一个：它没有容差参数（也不需要），
    // 任何单位向量都能补全出一组正交的 x/y —— 这里顺带钉住「它不校验也不失败」。
    CHECK(Coordinate2::from_x_axis(o, nearly_y).origin() == o);
}

TEST_CASE("a degenerate axis or a non-finite transform is rejected",
          "[linear][coordinate2][degenerate]") {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double infinity = std::numeric_limits<double>::infinity();

    const Point2 o{0.0, 0.0};
    const auto x = unit(1.0, 0.0);
    const auto y = unit(0.0, 1.0);

    // 零向量冒充单位向量：长度检查必须拦下它。
    CHECK_FALSE(Coordinate2::from_axes(o, unit(0.0, 0.0), y).has_value());
    CHECK_FALSE(Coordinate2::from_axes(o, x, unit(0.0, 0.0)).has_value());

    // NaN / ±inf 一律拒绝。
    CHECK_FALSE(Coordinate2::from_axes(o, unit(nan, 0.0), y).has_value());
    CHECK_FALSE(Coordinate2::from_axes(o, x, unit(0.0, nan)).has_value());
    CHECK_FALSE(Coordinate2::from_axes(o, unit(infinity, 0.0), y).has_value());
    CHECK_FALSE(Coordinate2::from_axes(o, x, unit(0.0, -infinity)).has_value());

    // from_transform：线性部分含 NaN。
    Transform2 broken{};
    broken.matrix.data[1][1] = nan;
    CHECK_FALSE(Coordinate2::from_transform(broken).has_value());
}

TEST_CASE("coordinate frame equality has evidence in every member",
          "[linear][coordinate2]") {
    const Point2 o{1.0, 2.0};
    const auto x = unit(1.0, 0.0);
    const auto y = unit(0.0, 1.0);

    const auto base = Coordinate2::from_axes(o, x, y);
    REQUIRE(base.has_value());

    CHECK(*base == *base);
    CHECK_FALSE(*base != *base);

    // origin。
    const auto other_origin = Coordinate2::from_axes(Point2{1.0, 3.0}, x, y);
    REQUIRE(other_origin.has_value());
    CHECK_FALSE(*base == *other_origin);
    CHECK(*base != *other_origin);   // C++20 生成的 != 也要真的被用过

    // 两条轴各要一格「只有它不同」的证据。二维里 y 由 x 唯一决定
    // （逆时针 90°），所以**精确**只差一条轴的另一个标架同样不存在；
    // 容差松弛区里有：1e-10 量级的偏移会通过全部校验，但 `==` 是精确比较。
    const double slack = 1e-10;
    const auto other_x = Coordinate2::from_axes(o, unit(1.0 + slack, 0.0), y);
    REQUIRE(other_x.has_value());
    CHECK_FALSE(*base == *other_x);

    const auto other_y = Coordinate2::from_axes(o, x, unit(0.0, 1.0 + slack));
    REQUIRE(other_y.has_value());
    CHECK_FALSE(*base == *other_y);
}

TEST_CASE("the implicitly generated special members carry the whole frame",
          "[linear][coordinate2]") {
    const auto source =
        Coordinate2::from_axes(Point2{1.0, 2.0}, unit(0.0, 1.0), unit(-1.0, 0.0));
    REQUIRE(source.has_value());

    // 拷贝构造。
    const Coordinate2 copied = *source;
    CHECK(copied.origin() == Point2{1.0, 2.0});
    CHECK(copied.x_axis() == unit(0.0, 1.0));
    CHECK(copied.y_axis() == unit(-1.0, 0.0));

    // 移动构造。
    Coordinate2 movable = *source;
    const Coordinate2 moved = std::move(movable);
    CHECK(moved.origin() == Point2{1.0, 2.0});
    CHECK(moved.x_axis() == unit(0.0, 1.0));
    CHECK(moved.y_axis() == unit(-1.0, 0.0));

    // 拷贝赋值：三个成员各自断言，不能靠 `==` 顺带覆盖。
    const auto target_frame = Coordinate2::from_axes(Point2{9.0, 9.0}, unit(1.0, 0.0),
                                                     unit(0.0, 1.0));
    REQUIRE(target_frame.has_value());
    Coordinate2 assigned = *target_frame;
    assigned = *source;
    CHECK(assigned.origin() == Point2{1.0, 2.0});
    CHECK(assigned.x_axis() == unit(0.0, 1.0));
    CHECK(assigned.y_axis() == unit(-1.0, 0.0));

    // 移动赋值。
    Coordinate2 move_assigned = *target_frame;
    move_assigned = std::move(assigned);
    CHECK(move_assigned.origin() == Point2{1.0, 2.0});
    CHECK(move_assigned.x_axis() == unit(0.0, 1.0));
    CHECK(move_assigned.y_axis() == unit(-1.0, 0.0));

    // 五个特殊成员都必须是隐式生成的（直接陈述规格，不用类型特征代理行为）。
    STATIC_REQUIRE(std::is_trivially_copyable_v<Coordinate2T<double>>);
    STATIC_REQUIRE(std::is_trivially_destructible_v<Coordinate2T<double>>);
    STATIC_REQUIRE(std::is_copy_constructible_v<Coordinate2T<double>>);
    STATIC_REQUIRE(std::is_move_constructible_v<Coordinate2T<double>>);
    STATIC_REQUIRE(std::is_copy_assignable_v<Coordinate2T<double>>);
    STATIC_REQUIRE(std::is_move_assignable_v<Coordinate2T<double>>);
}

TEST_CASE("the float instantiation is usable", "[linear][coordinate2]") {
    constexpr Coordinate2T<float> identity = Coordinate2T<float>::identity();
    STATIC_REQUIRE(identity.x_axis().x() == 1.0f);
    STATIC_REQUIRE(identity.y_axis().y() == 1.0f);
    STATIC_REQUIRE(identity.origin().x == 0.0f);
    STATIC_REQUIRE(identity.origin().y == 0.0f);

    const Point2T<float> origin{1.0f, 2.0f};
    const UnitVector2T<float> fx =
        UnitVector2T<float>::from_normalized_unchecked(Vector2T<float>{0.0f, 1.0f});
    const UnitVector2T<float> fy =
        UnitVector2T<float>::from_normalized_unchecked(Vector2T<float>{-1.0f, 0.0f});

    const auto frame = Coordinate2T<float>::from_axes(origin, fx, fy);
    REQUIRE(frame.has_value());
    CHECK(frame->origin() == origin);
    CHECK(frame->x_axis() == fx);
    CHECK(frame->y_axis() == fy);
    CHECK(frame->to_parent(Point2T<float>{1.0f, 2.0f}) == Point2T<float>{-1.0f, 3.0f});
    CHECK(frame->to_local(Point2T<float>{-1.0f, 3.0f}) == Point2T<float>{1.0f, 2.0f});
    CHECK(frame->to_parent(Vector2T<float>{1.0f, 2.0f}) == Vector2T<float>{-2.0f, 1.0f});
    CHECK(frame->to_local(Vector2T<float>{-2.0f, 1.0f}) == Vector2T<float>{1.0f, 2.0f});
    CHECK(frame->origin().x == 1.0f);

    const auto from_x = Coordinate2T<float>::from_x_axis(
        origin, UnitVector2T<float>::from_normalized_unchecked(Vector2T<float>{1.0f, 0.0f}));
    CHECK(from_x.x_axis().x() == 1.0f);
    CHECK(from_x.y_axis().y() == 1.0f);

    const auto from_xform = Coordinate2T<float>::from_transform(
        Transform2T<float>::translation(Vector2T<float>{1.0f, 2.0f}));
    REQUIRE(from_xform.has_value());
    CHECK(from_xform->origin() == origin);

    // 默认容差是按 double 定标的（abs = 1e-12），所以在 float 上要显式给出与该
    // 精度相称的容差 —— 容差显式传参的预期后果。这里同时证明了 from_transform
    // 的容差确实生效：同一份输入，默认拒绝、放宽容差后接受。
    //
    // 注意这一格在二维**不能**用 `rotation()` 来构造：float 下的 90° 旋转矩阵
    // 恰好是精确正交的（两列的点积逐位为 0、长度逐位为 1），默认容差也会接受它。
    // 用一处显式的扰动，让「不正交」这件事在 float 上真实存在。
    Transform2T<float> perturbed{};
    perturbed.matrix.data[0][1] = -1e-4f;
    CHECK_FALSE(Coordinate2T<float>::from_transform(perturbed).has_value());
    const auto loose = Coordinate2T<float>::from_transform(perturbed, Tolerance{1e-2, 1e-2});
    REQUIRE(loose.has_value());
    CHECK(loose->x_axis().x() == 1.0f);
    CHECK(loose->x_axis().y() == 0.0f);
    CHECK(loose->y_axis().x() == -1e-4f);
    CHECK(loose->y_axis().y() == 1.0f);
}

TEST_CASE("every declared callable is noexcept", "[linear][coordinate2]") {
    // 三个工厂与全部成员都是 noexcept。`from_x_axis` 是 Interfaces 明文写了
    // noexcept 的那一个（二维没有退化情形，所以它返回裸值）。
    const Point2 arg_origin{};
    const UnitVector2 unit_x = unit(1.0, 0.0);
    const UnitVector2 unit_y = unit(0.0, 1.0);

    STATIC_REQUIRE(noexcept(Coordinate2T<double>::identity()));
    STATIC_REQUIRE(noexcept(Coordinate2T<double>::from_axes(arg_origin, unit_x, unit_y)));
    STATIC_REQUIRE(noexcept(Coordinate2T<double>::from_axes(arg_origin, unit_x, unit_y, Tolerance{})));
    STATIC_REQUIRE(noexcept(Coordinate2T<double>::from_x_axis(arg_origin, unit_x)));
    STATIC_REQUIRE(noexcept(Coordinate2T<double>::from_transform(Transform2T<double>{})));
    STATIC_REQUIRE(noexcept(Coordinate2T<double>::from_transform(Transform2T<double>{}, Tolerance{})));
    STATIC_REQUIRE(noexcept(Coordinate2T<double>::identity().origin()));
    STATIC_REQUIRE(noexcept(Coordinate2T<double>::identity().x_axis()));
    STATIC_REQUIRE(noexcept(Coordinate2T<double>::identity().y_axis()));
    STATIC_REQUIRE(noexcept(Coordinate2T<double>::identity().to_parent(Point2T<double>{})));
    STATIC_REQUIRE(noexcept(Coordinate2T<double>::identity().to_local(Point2T<double>{})));
    STATIC_REQUIRE(noexcept(Coordinate2T<double>::identity().to_parent(Vector2T<double>{})));
    STATIC_REQUIRE(noexcept(Coordinate2T<double>::identity().to_local(Vector2T<double>{})));
    STATIC_REQUIRE(noexcept(Coordinate2T<double>::identity() == Coordinate2T<double>::identity()));
    // 零独立证据的一条：`!=` 的 noexcept 继承自被重写的 `==`。
    STATIC_REQUIRE(noexcept(Coordinate2T<double>::identity() != Coordinate2T<double>::identity()));
}

TEST_CASE("every callable is usable in a constant expression",
          "[linear][coordinate2]") {
    // 与三维不同，二维**没有**非 constexpr 的工厂（`from_x_axis` 不做归一化），
    // 所以四个可调用实体全部能在常量表达式里求值。
    constexpr Point2T<double> origin{1.0, 2.0};
    constexpr UnitVector2T<double> cx =
        UnitVector2T<double>::from_normalized_unchecked(Vector2T<double>{0.0, 1.0});
    constexpr UnitVector2T<double> cy =
        UnitVector2T<double>::from_normalized_unchecked(Vector2T<double>{-1.0, 0.0});

    constexpr auto frame = Coordinate2T<double>::from_axes(origin, cx, cy);
    STATIC_REQUIRE(frame.has_value());
    STATIC_REQUIRE(frame->origin().x == 1.0);
    STATIC_REQUIRE(frame->x_axis().y() == 1.0);
    STATIC_REQUIRE(frame->y_axis().x() == -1.0);

    constexpr auto refused = Coordinate2T<double>::from_axes(origin, cx, -cy);
    STATIC_REQUIRE_FALSE(refused.has_value());

    constexpr auto completed = Coordinate2T<double>::from_x_axis(origin, cx);
    STATIC_REQUIRE(completed.y_axis().x() == -1.0);
    STATIC_REQUIRE(completed.y_axis().y() == 0.0);

    STATIC_REQUIRE(frame->to_parent(Point2T<double>{1.0, 2.0}) == Point2T<double>{-1.0, 3.0});
    STATIC_REQUIRE(frame->to_local(Point2T<double>{-1.0, 3.0}) == Point2T<double>{1.0, 2.0});
    STATIC_REQUIRE(frame->to_parent(Vector2T<double>{1.0, 2.0}) == Vector2T<double>{-2.0, 1.0});
    STATIC_REQUIRE(frame->to_local(Vector2T<double>{-2.0, 1.0}) == Vector2T<double>{1.0, 2.0});
    STATIC_REQUIRE(*frame == *frame);

    constexpr auto xform = Coordinate2T<double>::from_transform(Transform2T<double>::identity());
    STATIC_REQUIRE(xform.has_value());
    STATIC_REQUIRE(xform->origin().y == 0.0);

    constexpr auto moved = Coordinate2T<double>::from_transform(
        Transform2T<double>::translation(Vector2T<double>{4.0, 5.0}));
    STATIC_REQUIRE(moved.has_value());
    STATIC_REQUIRE(moved->origin().y == 5.0);
}
