#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <limits>
#include <optional>
#include <type_traits>
#include <utility>

#include <DragonGeo/Core/Constants.hpp>
#include <DragonGeo/Core/Tolerance.hpp>
#include <DragonGeo/Linear/Coordinate2.hpp>
#include <DragonGeo/Linear/Transform2.hpp>

using Catch::Approx;

using DragonGeo::Core::HALF_PI;
using DragonGeo::Core::Tolerance;
using DragonGeo::Linear::Coordinate2;
using DragonGeo::Linear::Coordinate2f;
using DragonGeo::Linear::Coordinate2T;
using DragonGeo::Linear::Point2;
using DragonGeo::Linear::Point2T;
using DragonGeo::Linear::Transform2;
using DragonGeo::Linear::Transform2T;
using DragonGeo::Linear::UnitVector2;
using DragonGeo::Linear::UnitVector2T;
using DragonGeo::Linear::Vector2;
using DragonGeo::Linear::Vector2T;

namespace {

/// 世界 x 轴。二维的补全以 x 为主轴（`FromXAxis`），与三维的 `FromZAxis`
/// 对应 —— 两个文件的这个常量因此也不同名。
const UnitVector2 xAxis = UnitVector2::FromNormalizedUnchecked(Vector2{1.0, 0.0});

/// 由分量直接造单位向量（同 coordinate3Test.cpp）。
UnitVector2 unit(double x, double y) {
    return UnitVector2::FromNormalizedUnchecked(Vector2{x, y});
}

} // namespace

TEST_CASE("the float alias really is the float instantiation", "[linear][coordinate2]") {
    // 与三维逐条对齐（两个头文件独立编写，强度容易一边倒：Task 4 的 3D `==`
    // 有三个方向的断言、2D 只有一个，这个差异存在了四轮）。
    STATIC_REQUIRE(std::is_same_v<Coordinate2f, Coordinate2T<float>>);
    STATIC_REQUIRE(std::is_same_v<Coordinate2, Coordinate2T<double>>);
    STATIC_REQUIRE(std::is_same_v<Coordinate2T<float>::ScalarType, float>);
    STATIC_REQUIRE(std::is_same_v<Coordinate2T<double>::ScalarType, double>);

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

TEST_CASE("Identity() is the standard orthonormal frame", "[linear][coordinate2]") {
    constexpr Coordinate2T<double> identity = Coordinate2T<double>::Identity();

    STATIC_REQUIRE(identity.Origin().X == 0.0);
    STATIC_REQUIRE(identity.Origin().Y == 0.0);

    STATIC_REQUIRE(identity.XAxis().X() == 1.0);
    STATIC_REQUIRE(identity.XAxis().Y() == 0.0);

    STATIC_REQUIRE(identity.YAxis().X() == 0.0);
    STATIC_REQUIRE(identity.YAxis().Y() == 1.0);

    const Point2 point{1.0, 2.0};
    CHECK(identity.ToParent(point) == point);
    CHECK(identity.ToLocal(point) == point);
    CHECK(identity.ToParent(Vector2{3.0, 4.0}) == Vector2{3.0, 4.0});
    CHECK(identity.ToLocal(Vector2{3.0, 4.0}) == Vector2{3.0, 4.0});

    const auto madeByAxes =
        Coordinate2::FromAxes(Point2{0.0, 0.0}, unit(1.0, 0.0), unit(0.0, 1.0));
    REQUIRE(madeByAxes.has_value());
    CHECK(identity == *madeByAxes);
}

TEST_CASE("a coordinate frame cannot be built from non-orthogonal axes",
          "[linear][coordinate2][degenerate]") {
    const Point2 o{0.0, 0.0};
    const auto x = unit(1.0, 0.0);
    const auto y = unit(0.0, 1.0);

    // 正交 —— 成功
    CHECK(Coordinate2::FromAxes(o, x, y).has_value());

    // 斜交 —— 必须被拒绝，而不是构造出一个会拉伸几何的“坐标系”
    const auto skewed = unit(0.6, 0.8);
    CHECK_FALSE(Coordinate2::FromAxes(o, x, skewed).has_value());

    // 左手系（顺时针）—— 同样拒绝。二维没有第三根轴，等价的说法是
    // `x × y` 的标量叉积为负。三种排列各一条，与三维那边的三种对齐。
    CHECK_FALSE(Coordinate2::FromAxes(o, x, -y).has_value());
    CHECK_FALSE(Coordinate2::FromAxes(o, -x, y).has_value());
    CHECK_FALSE(Coordinate2::FromAxes(o, y, x).has_value());

    // 两轴重合是最极端的斜交：x·y = 1。
    CHECK_FALSE(Coordinate2::FromAxes(o, x, x).has_value());

    // 轴的**长度**也要校验（`FromNormalizedUnchecked` 是公开的，不校验就等于
    // 留了一条能造出「拉伸 2 倍」的坐标系的路径）。这一对刻意让叉积保持为正：
    // 二维的叉积检查只看符号，拉伸/压缩在它眼里完全一样 —— 只有长度校验能拦下。
    CHECK_FALSE(Coordinate2::FromAxes(o, unit(2.0, 0.0), unit(0.0, 0.5)).has_value());

    // 两条长度子表达式各自的见证：1e-6 的偏移让 |x|² 偏离 1 达 2e-6
    // （远大于长度校验 abs + rel ≈ 1e-9），而叉积仍然为正 —— 每格只有一条
    // 子表达式会拒它。二维这一格比三维更好钉：叉积是纯符号判断，不像三维的
    // `(x × y) · z ≈ 1` 那样会跟着长度一起变，所以偏移量不必卡在两个容差之间。
    const double scaled = 1.0 + 1e-6;
    CHECK_FALSE(Coordinate2::FromAxes(o, unit(scaled, 0.0), y).has_value());
    CHECK_FALSE(Coordinate2::FromAxes(o, x, unit(0.0, scaled)).has_value());

    // 定向判据 `x × y > 0` 里的**严格性**（`>` 不是 `>=`）也要有证据。
    // 二者只在一处分开：叉积**恰好为零** —— 也就是有一根轴是零向量。
    // 零轴在正常容差下早就被长度检查拒了，所以这一格必须把容差放大到
    // 「零也算在 1 的容差内」（abs + rel ≥ 1，本例取 1.0）才能**只**由
    // 符号判据决定：此时长度与正交两项都通过（实测 `Equal(0, 1)` 在 1.0 下为真），
    // 唯一拒它的就是 `0 > 0`。
    // 实测：把 `>` 放宽成 `>=` 的变异体在补这一格之前**整套 224 个用例全绿**；
    // 补上之后它死在这里 —— 而那个变异体接受的是一个 x 轴为零向量的「标架」
    // （xx = 0、cross = 0），正是本任务要堵死的那类退化产物。
    const Tolerance absurd{1.0, 1.0};
    CHECK_FALSE(Coordinate2::FromAxes(o, unit(0.0, 0.0), y, absurd).has_value());
    // 反证：同一档容差下一个正常的单位标架照样被接受 —— 否则上面那一格可能
    // 只是「容差大到什么都拒」的副作用。
    CHECK(Coordinate2::FromAxes(o, x, y, absurd).has_value());
}

TEST_CASE("coordinate frame round-trips a point", "[linear][coordinate2]") {
    const auto frame = Coordinate2::FromXAxis(Point2{10.0, 0.0}, xAxis);
    REQUIRE(frame.has_value());

    const Point2 local{1.0, 2.0};
    const Point2 roundTrip = frame->ToLocal(frame->ToParent(local));

    CHECK(roundTrip.X == Approx(local.X).margin(1e-12));
    CHECK(roundTrip.Y == Approx(local.Y).margin(1e-12));
}

TEST_CASE("FromXAxis completes y as x rotated 90 degrees counter-clockwise",
          "[linear][coordinate2]") {
    // 上面那条往返用例的 x 是世界 x 轴，补全出来的恰好是标准基 ——
    // 于是「y 写成 (-x.Y, -x.X)（顺时针）」「y 就是 x」这两种实现都能通过它。
    // 这一条改用非轴向的 x，并把补全出来的轴逐分量钉死。
    //
    // 期望值：x = (0.6, 0.8) ⇒ y = (-x.Y, x.X) = (-0.8, 0.6)。
    const auto frame = Coordinate2::FromXAxis(Point2{0.0, 0.0}, unit(0.6, 0.8));
    REQUIRE(frame.has_value());

    CHECK(frame->XAxis().X() == Approx(0.6));
    CHECK(frame->XAxis().Y() == Approx(0.8));
    CHECK(frame->YAxis().X() == Approx(-0.8));
    CHECK(frame->YAxis().Y() == Approx(0.6));

    // 定义性质，不依赖上面那组值：x × y = +1。
    CHECK(frame->XAxis().Cross(frame->YAxis()) == Approx(1.0));

    // 原点必须原样带过来（不能总是 (0,0)）。
    const auto moved = Coordinate2::FromXAxis(Point2{10.0, 20.0}, unit(1.0, 0.0));
    REQUIRE(moved.has_value());
    CHECK(moved->Origin() == Point2{10.0, 20.0});
    CHECK(moved->YAxis().X() == Approx(0.0));
    CHECK(moved->YAxis().Y() == Approx(1.0));

    // 由 x 轴负方向出发（x = (-1,0)）⇒ y = (0,-1)：这是「y 恒等于 (0,1)」
    // 这类实现唯一的一格反例。
    const auto flipped = Coordinate2::FromXAxis(Point2{0.0, 0.0}, unit(-1.0, 0.0));
    REQUIRE(flipped.has_value());
    CHECK(flipped->XAxis().X() == Approx(-1.0));
    CHECK(flipped->XAxis().Y() == Approx(0.0).margin(1e-15));
    CHECK(flipped->YAxis().X() == Approx(0.0).margin(1e-15));
    CHECK(flipped->YAxis().Y() == Approx(-1.0));

    // **非单位的 x 必须被拒绝。** 补全公式只用得到 x 的方向，对长度一无所知：
    // 不放行校验，`FromNormalizedUnchecked({2,0})`（公开接口）就会补出一组
    // 长度都是 2 的轴 —— 一个把几何拉伸 2 倍的「坐标系」，正是本任务要堵死的那类
    // 路径。三维那边的 `FromZAxis` 对非单位的 z 同样拒绝（那里有一格）。
    // 这一格是「FromXAxis 真的走了 FromAxes」的唯一见证。
    CHECK_FALSE(Coordinate2::FromXAxis(Point2{0.0, 0.0}, unit(2.0, 0.0)).has_value());
    CHECK_FALSE(Coordinate2::FromXAxis(Point2{0.0, 0.0}, unit(0.0, 0.5)).has_value());
    // 反向自证：同一组输入在放宽容差后必须被接受 —— 否则上面两格可能只是被
    // 别的检查（比如某个过紧的阈值）拒掉的。
    const Tolerance loose{1.0, 1.0};
    CHECK(Coordinate2::FromXAxis(Point2{0.0, 0.0}, unit(2.0, 0.0), loose).has_value());
    CHECK(Coordinate2::FromXAxis(Point2{0.0, 0.0}, unit(0.0, 0.5), loose).has_value());
    // 容差参数确实被转发进 `FromAxes` 的长度检查（默认判据下 1e-6 的偏差被拒、
    // 放宽容差后接受）。
    CHECK_FALSE(Coordinate2::FromXAxis(Point2{0.0, 0.0}, unit(1.0 + 1e-6, 0.0)).has_value());
    CHECK(Coordinate2::FromXAxis(Point2{0.0, 0.0}, unit(1.0 + 1e-6, 0.0),
                                   Tolerance{1e-3, 1e-3})
              .has_value());
    // 非有限轴同样拒绝。
    const double nan = std::numeric_limits<double>::quiet_NaN();
    CHECK_FALSE(Coordinate2::FromXAxis(Point2{0.0, 0.0}, unit(nan, 0.0)).has_value());
}

TEST_CASE("ToParent and ToLocal carry the whole frame", "[linear][coordinate2]") {
    // 原点非零、两轴都不是标准基的标架；轴取 0/±1 的置换，期望值精确可表示。
    //   x = (0,1)、y = (-1,0)（x 逆时针转 90°）。
    const auto frame =
        Coordinate2::FromAxes(Point2{10.0, 20.0}, unit(0.0, 1.0), unit(-1.0, 0.0));
    REQUIRE(frame.has_value());

    // 点：origin + x·lx + y·ly。两个分量分别被拉到不同的轴上，各要一条证据
    // （把 y 轴写成 x 轴的实现只会在其中一条上现形）。
    CHECK(frame->ToParent(Point2{1.0, 2.0}) == Point2{8.0, 21.0});
    CHECK(frame->ToParent(Point2{0.0, 0.0}) == Point2{10.0, 20.0});
    CHECK(frame->ToParent(Point2{1.0, 0.0}) == Point2{10.0, 21.0});
    CHECK(frame->ToParent(Point2{0.0, 1.0}) == Point2{9.0, 20.0});

    // 反方向必须减掉 origin（不减的话 (8,21) 会算出 (21,-8)）。
    CHECK(frame->ToLocal(Point2{8.0, 21.0}) == Point2{1.0, 2.0});
    CHECK(frame->ToLocal(Point2{10.0, 20.0}) == Point2{0.0, 0.0});
    CHECK(frame->ToLocal(Point2{10.0, 21.0}) == Point2{1.0, 0.0});
    CHECK(frame->ToLocal(Point2{9.0, 20.0}) == Point2{0.0, 1.0});

    // 向量：只有线性部分，**平移不生效**（期望值与点的不同正是证据）。
    CHECK(frame->ToParent(Vector2{1.0, 2.0}) == Vector2{-2.0, 1.0});
    CHECK(frame->ToParent(Vector2{0.0, 0.0}) == Vector2{0.0, 0.0});
    CHECK(frame->ToParent(Vector2{1.0, 0.0}) == Vector2{0.0, 1.0});
    CHECK(frame->ToParent(Vector2{0.0, 1.0}) == Vector2{-1.0, 0.0});

    CHECK(frame->ToLocal(Vector2{8.0, 21.0}) == Vector2{21.0, -8.0});
    CHECK(frame->ToLocal(Vector2{1.0, 0.0}) == Vector2{0.0, -1.0});
    CHECK(frame->ToLocal(Vector2{0.0, 1.0}) == Vector2{1.0, 0.0});

    // 往返：两个方向、两种输入都要。
    const Point2 local{0.25, -1.5};
    CHECK(frame->ToParent(frame->ToLocal(local)) == local);
    const Point2 parent{1.0, 2.0};
    CHECK(frame->ToLocal(frame->ToParent(parent)) == parent);
    const Vector2 localDirection{0.25, -1.5};
    CHECK(frame->ToParent(frame->ToLocal(localDirection)) == localDirection);
    const Vector2 parentDirection{1.0, 2.0};
    CHECK(frame->ToLocal(frame->ToParent(parentDirection)) == parentDirection);

    // 上面那个标架的两根轴各有一个零分量（x = (0,1)、y = (-1,0)），于是
    // `ToParent` 四项里的 `m_x.X() * local.X` 与 `m_y.Y() * local.Y` 恒为零 ——
    // 整个删掉也看不出来。换一个四个系数都非零的标架（x = (0.6,0.8)，
    // y = (-0.8,0.6) 由补全得到，原点 (10,20)）。
    const auto dense = Coordinate2::FromXAxis(Point2{10.0, 20.0}, unit(0.6, 0.8));
    REQUIRE(dense.has_value());

    // 10 + 0.6·1 + (-0.8)·2 = 9 ／ 20 + 0.8·1 + 0.6·2 = 22
    CHECK(dense->ToParent(Point2{1.0, 2.0}).X == Approx(9.0));
    CHECK(dense->ToParent(Point2{1.0, 2.0}).Y == Approx(22.0));
    // 反方向：offset = (9,22) - (10,20) = (-1,2) ⇒ (1,2)
    CHECK(dense->ToLocal(Point2{9.0, 22.0}).X == Approx(1.0));
    CHECK(dense->ToLocal(Point2{9.0, 22.0}).Y == Approx(2.0));
    // 方向重载：⊥ 平移。0.6·1 - 0.8·2 = -1 ／ 0.8·1 + 0.6·2 = 2
    CHECK(dense->ToParent(Vector2{1.0, 2.0}).X == Approx(-1.0));
    CHECK(dense->ToParent(Vector2{1.0, 2.0}).Y == Approx(2.0));
    // 0.6·9 + 0.8·22 = 23 ／ -0.8·9 + 0.6·22 = 6
    CHECK(dense->ToLocal(Vector2{9.0, 22.0}).X == Approx(23.0));
    CHECK(dense->ToLocal(Vector2{9.0, 22.0}).Y == Approx(6.0));
    // 往返（这个标架下两个方向都要走一遍）。
    CHECK(dense->ToParent(dense->ToLocal(Point2{1.0, 2.0})).X == Approx(1.0));
    CHECK(dense->ToParent(dense->ToLocal(Point2{1.0, 2.0})).Y == Approx(2.0));
}

TEST_CASE("only a rigid transform is a coordinate frame",
          "[linear][coordinate2][degenerate]") {
    // FromTransform 若返回裸值，这里就会静默得到一个会拉伸几何的「坐标系」。
    CHECK_FALSE(Coordinate2::FromTransform(Transform2::Scaling(Vector2{2.0, 3.0})).has_value());

    // 均匀缩放同样不是标架。二维这一格特别要紧：**叉积检查只看符号**，
    // 均匀缩放不会改变符号，于是「先把两列归一化再校验」的实现照样放行 ——
    // 判据必须是线性部分**本身**正交。
    CHECK_FALSE(Coordinate2::FromTransform(Transform2::Scaling(2.0)).has_value());

    // 刚体变换才是标架。
    const auto rigid = Coordinate2::FromTransform(
        Transform2::Translation(Vector2{1.0, 2.0}) * Transform2::Rotation(HALF_PI));
    REQUIRE(rigid.has_value());
    CHECK(rigid->Origin() == Point2{1.0, 2.0});

    CHECK(Coordinate2::FromTransform(Transform2::Identity()).has_value());

    // 只有 origin 被钉住是不够的：一个「原点取平移列、两轴直接给标准基」的实现
    // 能通过上面全部四条。把旋转出来的轴逐分量钉死。
    CHECK(rigid->XAxis().X() == Approx(0.0).margin(1e-15));
    CHECK(rigid->XAxis().Y() == Approx(1.0));
    CHECK(rigid->YAxis().X() == Approx(-1.0));
    CHECK(rigid->YAxis().Y() == Approx(0.0).margin(1e-15));

    // 180° 旋转（det = +1、坐标全为 ±1，精确可表示）。
    const auto halfTurn = Coordinate2::FromTransform(Transform2::Scaling(Vector2{-1.0, -1.0}));
    REQUIRE(halfTurn.has_value());
    CHECK(halfTurn->XAxis().X() == -1.0);
    CHECK(halfTurn->XAxis().Y() == 0.0);
    CHECK(halfTurn->YAxis().X() == 0.0);
    CHECK(halfTurn->YAxis().Y() == -1.0);

    // 反射：线性部分**是**正交矩阵（AᵀA = I），但 det = -1，不是刚体变换。
    // 若这里放行，`FromTransform` 就成了一条公开的、通往左手标架的路径 ——
    // 而 `FromAxes` 明确拒绝左手系，同一个不变量不能有两个说法。
    CHECK_FALSE(Coordinate2::FromTransform(Transform2::Scaling(Vector2{1.0, -1.0})).has_value());

    // 剪切：同样在正交之外，且两列不互相垂直。
    Transform2 shear{};
    shear.Matrix.Data[0][1] = 0.5;
    CHECK_FALSE(Coordinate2::FromTransform(shear).has_value());

    // 非均匀缩放的复合（刚体 ⊗ 缩放不是刚体）。
    CHECK_FALSE(Coordinate2::FromTransform(Transform2::Translation(Vector2{5.0, 0.0})
                                            * Transform2::Rotation(HALF_PI)
                                            * Transform2::Scaling(2.0))
                    .has_value());

    // 平移本身是刚体（线性部分是恒等），必须成功，且原点就是平移量。
    const auto moved = Coordinate2::FromTransform(Transform2::Translation(Vector2{-4.0, 7.0}));
    REQUIRE(moved.has_value());
    CHECK(moved->Origin() == Point2{-4.0, 7.0});
    CHECK(moved->XAxis().X() == 1.0);
    CHECK(moved->YAxis().Y() == 1.0);
}

TEST_CASE("FromAxes and FromTransform thread their tolerance through",
          "[linear][coordinate2][degenerate]") {
    // 容差必须显式传参、不得在函数体内硬编码阈值。同一份输入、只有容差不同，
    // 结果必须不同 —— 阈值一旦被写死，两格就会同向。
    const Point2 o{0.0, 0.0};
    const auto x = unit(1.0, 0.0);
    const auto y = unit(0.0, 1.0);

    const double sine = 1e-7;
    const double cosine = std::sqrt(1.0 - sine * sine);
    const auto nearlyY = unit(sine, cosine);

    CHECK_FALSE(Coordinate2::FromAxes(o, x, nearlyY).has_value());
    const Tolerance loose{1e-6, 1e-6};
    CHECK(Coordinate2::FromAxes(o, x, nearlyY, loose).has_value());

    // FromTransform 同样：线性部分的两列被扰动成 1e-7 的斜交，默认拒绝、
    // 放宽容差后接受。（只改 `data[0][1]` 一处：单位阵的其余项原样保留，
    // 于是两列仍是单位向量 —— 拒绝的理由只有「不正交」这一条。）
    Transform2 perturbed{};
    perturbed.Matrix.Data[0][1] = -1e-7;

    CHECK_FALSE(Coordinate2::FromTransform(perturbed).has_value());
    CHECK(Coordinate2::FromTransform(perturbed, loose).has_value());

    // `FromXAxis` 现在与 `FromZAxis` 同形：补全之后照样过 `FromAxes`，
    // 容差参数因此也贯穿它的长度检查（正交与定向对补全出来的轴恒成立，
    // 唯一会用到容差的就是长度那一条）。
    CHECK(Coordinate2::FromXAxis(o, nearlyY).has_value());
    const auto stretched = unit(1.0 + 1e-7, 0.0);
    CHECK_FALSE(Coordinate2::FromXAxis(o, stretched).has_value());
    CHECK(Coordinate2::FromXAxis(o, stretched, loose).has_value());
}

TEST_CASE("a degenerate axis or a non-finite transform is rejected",
          "[linear][coordinate2][degenerate]") {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double infinity = std::numeric_limits<double>::infinity();

    const Point2 o{0.0, 0.0};
    const auto x = unit(1.0, 0.0);
    const auto y = unit(0.0, 1.0);

    // 零向量冒充单位向量：长度检查必须拦下它。
    CHECK_FALSE(Coordinate2::FromAxes(o, unit(0.0, 0.0), y).has_value());
    CHECK_FALSE(Coordinate2::FromAxes(o, x, unit(0.0, 0.0)).has_value());

    // NaN / ±inf 一律拒绝。
    CHECK_FALSE(Coordinate2::FromAxes(o, unit(nan, 0.0), y).has_value());
    CHECK_FALSE(Coordinate2::FromAxes(o, x, unit(0.0, nan)).has_value());
    CHECK_FALSE(Coordinate2::FromAxes(o, unit(infinity, 0.0), y).has_value());
    CHECK_FALSE(Coordinate2::FromAxes(o, x, unit(0.0, -infinity)).has_value());

    // FromTransform：线性部分含 NaN。
    Transform2 broken{};
    broken.Matrix.Data[1][1] = nan;
    CHECK_FALSE(Coordinate2::FromTransform(broken).has_value());
}

TEST_CASE("coordinate frame equality has evidence in every member",
          "[linear][coordinate2]") {
    const Point2 o{1.0, 2.0};
    const auto x = unit(1.0, 0.0);
    const auto y = unit(0.0, 1.0);

    const auto base = Coordinate2::FromAxes(o, x, y);
    REQUIRE(base.has_value());

    CHECK(*base == *base);
    CHECK_FALSE(*base != *base);

    // origin。
    const auto otherOrigin = Coordinate2::FromAxes(Point2{1.0, 3.0}, x, y);
    REQUIRE(otherOrigin.has_value());
    CHECK_FALSE(*base == *otherOrigin);
    CHECK(*base != *otherOrigin);   // C++20 生成的 != 也要真的被用过

    // 两条轴各要一格「只有它不同」的证据。二维里 y 由 x 唯一决定
    // （逆时针 90°），所以**精确**只差一条轴的另一个标架同样不存在；
    // 容差松弛区里有：1e-10 量级的偏移会通过全部校验，但 `==` 是精确比较。
    const double slack = 1e-10;
    const auto otherX = Coordinate2::FromAxes(o, unit(1.0 + slack, 0.0), y);
    REQUIRE(otherX.has_value());
    CHECK_FALSE(*base == *otherX);

    const auto otherY = Coordinate2::FromAxes(o, x, unit(0.0, 1.0 + slack));
    REQUIRE(otherY.has_value());
    CHECK_FALSE(*base == *otherY);
}

TEST_CASE("the implicitly generated special members carry the whole frame",
          "[linear][coordinate2]") {
    const auto source =
        Coordinate2::FromAxes(Point2{1.0, 2.0}, unit(0.0, 1.0), unit(-1.0, 0.0));
    REQUIRE(source.has_value());

    // 拷贝构造。
    const Coordinate2 copied = *source;
    CHECK(copied.Origin() == Point2{1.0, 2.0});
    CHECK(copied.XAxis() == unit(0.0, 1.0));
    CHECK(copied.YAxis() == unit(-1.0, 0.0));

    // 移动构造。
    Coordinate2 movable = *source;
    const Coordinate2 moved = std::move(movable);
    CHECK(moved.Origin() == Point2{1.0, 2.0});
    CHECK(moved.XAxis() == unit(0.0, 1.0));
    CHECK(moved.YAxis() == unit(-1.0, 0.0));

    // 拷贝赋值：三个成员各自断言，不能靠 `==` 顺带覆盖。
    const auto targetFrame = Coordinate2::FromAxes(Point2{9.0, 9.0}, unit(1.0, 0.0),
                                                     unit(0.0, 1.0));
    REQUIRE(targetFrame.has_value());
    Coordinate2 assigned = *targetFrame;
    assigned = *source;
    CHECK(assigned.Origin() == Point2{1.0, 2.0});
    CHECK(assigned.XAxis() == unit(0.0, 1.0));
    CHECK(assigned.YAxis() == unit(-1.0, 0.0));

    // 移动赋值。
    Coordinate2 moveAssigned = *targetFrame;
    moveAssigned = std::move(assigned);
    CHECK(moveAssigned.Origin() == Point2{1.0, 2.0});
    CHECK(moveAssigned.XAxis() == unit(0.0, 1.0));
    CHECK(moveAssigned.YAxis() == unit(-1.0, 0.0));

    // 五个特殊成员的无行为是这一类型的规格（直接陈述规格，不用类型特征代理行为）。
    // 措辞收紧（Minor-8）：这几条钉的是「平凡 / 可构造 / 可赋值」—— `= default`
    // 的成员同样为真；证明「成员被逐位搬运」的是上面那四组断言（实测 J5）。
    STATIC_REQUIRE(std::is_trivially_copyable_v<Coordinate2T<double>>);
    STATIC_REQUIRE(std::is_trivially_destructible_v<Coordinate2T<double>>);
    STATIC_REQUIRE(std::is_copy_constructible_v<Coordinate2T<double>>);
    STATIC_REQUIRE(std::is_move_constructible_v<Coordinate2T<double>>);
    STATIC_REQUIRE(std::is_copy_assignable_v<Coordinate2T<double>>);
    STATIC_REQUIRE(std::is_move_assignable_v<Coordinate2T<double>>);
}

TEST_CASE("the float instantiation is usable", "[linear][coordinate2]") {
    constexpr Coordinate2T<float> identity = Coordinate2T<float>::Identity();
    STATIC_REQUIRE(identity.XAxis().X() == 1.0f);
    STATIC_REQUIRE(identity.YAxis().Y() == 1.0f);
    STATIC_REQUIRE(identity.Origin().X == 0.0f);
    STATIC_REQUIRE(identity.Origin().Y == 0.0f);

    const Point2T<float> origin{1.0f, 2.0f};
    const UnitVector2T<float> fx =
        UnitVector2T<float>::FromNormalizedUnchecked(Vector2T<float>{0.0f, 1.0f});
    const UnitVector2T<float> fy =
        UnitVector2T<float>::FromNormalizedUnchecked(Vector2T<float>{-1.0f, 0.0f});

    const auto frame = Coordinate2T<float>::FromAxes(origin, fx, fy);
    REQUIRE(frame.has_value());
    CHECK(frame->Origin() == origin);
    CHECK(frame->XAxis() == fx);
    CHECK(frame->YAxis() == fy);
    CHECK(frame->ToParent(Point2T<float>{1.0f, 2.0f}) == Point2T<float>{-1.0f, 3.0f});
    CHECK(frame->ToLocal(Point2T<float>{-1.0f, 3.0f}) == Point2T<float>{1.0f, 2.0f});
    CHECK(frame->ToParent(Vector2T<float>{1.0f, 2.0f}) == Vector2T<float>{-2.0f, 1.0f});
    CHECK(frame->ToLocal(Vector2T<float>{-2.0f, 1.0f}) == Vector2T<float>{1.0f, 2.0f});
    CHECK(frame->Origin().X == 1.0f);

    const auto fromX = Coordinate2T<float>::FromXAxis(
        origin, UnitVector2T<float>::FromNormalizedUnchecked(Vector2T<float>{1.0f, 0.0f}));
    REQUIRE(fromX.has_value());
    CHECK(fromX->XAxis().X() == 1.0f);
    CHECK(fromX->YAxis().Y() == 1.0f);

    const auto fromXform = Coordinate2T<float>::FromTransform(
        Transform2T<float>::Translation(Vector2T<float>{1.0f, 2.0f}));
    REQUIRE(fromXform.has_value());
    CHECK(fromXform->Origin() == origin);

    // 默认容差是按 double 定标的（abs = 1e-12），所以在 float 上要显式给出与该
    // 精度相称的容差 —— 容差显式传参的预期后果。这里同时证明了 FromTransform
    // 的容差确实生效：同一份输入，默认拒绝、放宽容差后接受。
    //
    // 注意这一格在二维**不能**用 `Rotation()` 来构造：float 下的 90° 旋转矩阵
    // 恰好是精确正交的（两列的点积逐位为 0、长度逐位为 1），默认容差也会接受它。
    // 用一处显式的扰动，让「不正交」这件事在 float 上真实存在。
    Transform2T<float> perturbed{};
    perturbed.Matrix.Data[0][1] = -1e-4f;
    CHECK_FALSE(Coordinate2T<float>::FromTransform(perturbed).has_value());
    const auto loose = Coordinate2T<float>::FromTransform(perturbed, Tolerance{1e-2, 1e-2});
    REQUIRE(loose.has_value());
    CHECK(loose->XAxis().X() == 1.0f);
    CHECK(loose->XAxis().Y() == 0.0f);
    CHECK(loose->YAxis().X() == -1e-4f);
    CHECK(loose->YAxis().Y() == 1.0f);
}

TEST_CASE("every declared callable is noexcept", "[linear][coordinate2]") {
    // 三个工厂与全部成员都是 noexcept —— 它们只做标量算术并返回 optional，没有
    // 任何可能抛出的操作。四个工厂里只有 `FromAxes` 与 `FromTransform` 在
    // Interfaces 里被明文标注，`identity` / `FromXAxis` 是家族惯例（与三维一致）。
    const Point2 argOrigin{};
    const UnitVector2 unitX = unit(1.0, 0.0);
    const UnitVector2 unitY = unit(0.0, 1.0);

    STATIC_REQUIRE(noexcept(Coordinate2T<double>::Identity()));
    STATIC_REQUIRE(noexcept(Coordinate2T<double>::FromAxes(argOrigin, unitX, unitY)));
    STATIC_REQUIRE(noexcept(Coordinate2T<double>::FromAxes(argOrigin, unitX, unitY, Tolerance{})));
    STATIC_REQUIRE(noexcept(Coordinate2T<double>::FromXAxis(argOrigin, unitX)));
    STATIC_REQUIRE(
        noexcept(Coordinate2T<double>::FromXAxis(argOrigin, unitX, Tolerance{})));
    STATIC_REQUIRE(noexcept(Coordinate2T<double>::FromTransform(Transform2T<double>{})));
    STATIC_REQUIRE(noexcept(Coordinate2T<double>::FromTransform(Transform2T<double>{}, Tolerance{})));
    STATIC_REQUIRE(noexcept(Coordinate2T<double>::Identity().Origin()));
    STATIC_REQUIRE(noexcept(Coordinate2T<double>::Identity().XAxis()));
    STATIC_REQUIRE(noexcept(Coordinate2T<double>::Identity().YAxis()));
    STATIC_REQUIRE(noexcept(Coordinate2T<double>::Identity().ToParent(Point2T<double>{})));
    STATIC_REQUIRE(noexcept(Coordinate2T<double>::Identity().ToLocal(Point2T<double>{})));
    STATIC_REQUIRE(noexcept(Coordinate2T<double>::Identity().ToParent(Vector2T<double>{})));
    STATIC_REQUIRE(noexcept(Coordinate2T<double>::Identity().ToLocal(Vector2T<double>{})));
    STATIC_REQUIRE(noexcept(Coordinate2T<double>::Identity() == Coordinate2T<double>::Identity()));
    // 零独立证据的一条：`!=` 的 noexcept 继承自被重写的 `==`。
    STATIC_REQUIRE(noexcept(Coordinate2T<double>::Identity() != Coordinate2T<double>::Identity()));
}

TEST_CASE("every callable is usable in a constant expression",
          "[linear][coordinate2]") {
    // 与三维不同，二维**没有**非 constexpr 的工厂 —— `FromXAxis` 只做一次
    // 90° 旋转（没有 `Vector::normalized` 那条非 constexpr 的路径），所以四个
    // 工厂全部能在常量表达式里求值。
    constexpr Point2T<double> origin{1.0, 2.0};
    constexpr UnitVector2T<double> cx =
        UnitVector2T<double>::FromNormalizedUnchecked(Vector2T<double>{0.0, 1.0});
    constexpr UnitVector2T<double> cy =
        UnitVector2T<double>::FromNormalizedUnchecked(Vector2T<double>{-1.0, 0.0});

    constexpr auto frame = Coordinate2T<double>::FromAxes(origin, cx, cy);
    STATIC_REQUIRE(frame.has_value());
    STATIC_REQUIRE(frame->Origin().X == 1.0);
    STATIC_REQUIRE(frame->XAxis().Y() == 1.0);
    STATIC_REQUIRE(frame->YAxis().X() == -1.0);

    constexpr auto refused = Coordinate2T<double>::FromAxes(origin, cx, -cy);
    STATIC_REQUIRE_FALSE(refused.has_value());

    constexpr auto completed = Coordinate2T<double>::FromXAxis(origin, cx);
    STATIC_REQUIRE(completed.has_value());
    STATIC_REQUIRE(completed->YAxis().X() == -1.0);
    STATIC_REQUIRE(completed->YAxis().Y() == 0.0);
    // 被拒的那一支也要在常量表达式里走一次。
    constexpr auto refusedX = Coordinate2T<double>::FromXAxis(
        origin, UnitVector2T<double>::FromNormalizedUnchecked(Vector2T<double>{2.0, 0.0}));
    STATIC_REQUIRE_FALSE(refusedX.has_value());

    STATIC_REQUIRE(frame->ToParent(Point2T<double>{1.0, 2.0}) == Point2T<double>{-1.0, 3.0});
    STATIC_REQUIRE(frame->ToLocal(Point2T<double>{-1.0, 3.0}) == Point2T<double>{1.0, 2.0});
    STATIC_REQUIRE(frame->ToParent(Vector2T<double>{1.0, 2.0}) == Vector2T<double>{-2.0, 1.0});
    STATIC_REQUIRE(frame->ToLocal(Vector2T<double>{-2.0, 1.0}) == Vector2T<double>{1.0, 2.0});
    STATIC_REQUIRE(*frame == *frame);

    constexpr auto xform = Coordinate2T<double>::FromTransform(Transform2T<double>::Identity());
    STATIC_REQUIRE(xform.has_value());
    STATIC_REQUIRE(xform->Origin().Y == 0.0);

    constexpr auto moved = Coordinate2T<double>::FromTransform(
        Transform2T<double>::Translation(Vector2T<double>{4.0, 5.0}));
    STATIC_REQUIRE(moved.has_value());
    STATIC_REQUIRE(moved->Origin().Y == 5.0);
}
