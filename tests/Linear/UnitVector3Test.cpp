#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <limits>
#include <optional>
#include <type_traits>

#include <DragonGeo/Linear/Coordinate3.hpp>
#include <DragonGeo/Linear/UnitVector3.hpp>

using Catch::Approx;

using DragonGeo::Core::Tolerance;
using DragonGeo::Linear::Coordinate3;
using DragonGeo::Linear::Point3;
using DragonGeo::Linear::UnitVector3;
using DragonGeo::Linear::UnitVector3T;
using DragonGeo::Linear::UnitVector3f;
using DragonGeo::Linear::Vector3;

TEST_CASE("the float alias really is the float instantiation", "[linear][unitvector3]") {
    // 同 vector4Test.cpp：UnitVector3f 此前零命中，误绑定不会被断言发现。
    STATIC_REQUIRE(std::is_same_v<UnitVector3f, UnitVector3T<float>>);
}

TEST_CASE("normalize produces a unit-length vector", "[linear][unitvector3]") {
    const auto result = Vector3{3.0, 4.0, 0.0}.Normalized();

    REQUIRE(result.has_value());
    CHECK(result->X() == Approx(0.6));
    CHECK(result->Y() == Approx(0.8));
    CHECK(result->Z() == Approx(0.0));
    CHECK(result->AsVector().Length() == Approx(1.0));
}

TEST_CASE("normalize rejects the zero vector", "[linear][unitvector3][degenerate]") {
    const auto result = Vector3{0.0, 0.0, 0.0}.Normalized();

    // 必须返回 nullopt，而不是含 NaN 的单位向量
    CHECK_FALSE(result.has_value());
}

TEST_CASE("normalize rejects vectors below the tolerance", "[linear][unitvector3][degenerate]") {
    CHECK_FALSE(Vector3{1e-15, 0.0, 0.0}.Normalized().has_value());
}

TEST_CASE("a zero tolerance rejects only the exact zero vector", "[linear][unitvector3][degenerate]") {
    const Tolerance exact{0.0, 0.0};

    CHECK_FALSE(Vector3{0.0, 0.0, 0.0}.Normalized(exact).has_value());
    CHECK(Vector3{1e-300, 0.0, 0.0}.Normalized(exact).has_value());
}

TEST_CASE("normalize rejects vectors below the absolute tolerance", "[linear][unitvector3][degenerate]") {
    // 1e-200 的长度远小于默认绝对容差 1e-12，因此按「视为零」处理。需要在这个量级上工作时，必须显式传入更小的容差 —— 这正是容差显式传参原则的预期后果。
    CHECK_FALSE(Vector3{1e-200, 0.0, 0.0}.Normalized().has_value());
}

TEST_CASE("normalize scales correctly at extreme magnitudes", "[linear][unitvector3][degenerate]") {
    // 关闭容差判断，单独考察重缩放后的计算本身是否上溢或下溢
    const Tolerance exact{0.0, 0.0};

    const auto huge = Vector3{1e200, 1e200, 0.0}.Normalized(exact);
    REQUIRE(huge.has_value());
    CHECK(huge->X() == Approx(0.7071067811865476));
    CHECK(huge->Y() == Approx(0.7071067811865476));

    const auto tiny = Vector3{1e-200, 1e-200, 0.0}.Normalized(exact);
    REQUIRE(tiny.has_value());
    CHECK(tiny->X() == Approx(0.7071067811865476));
    CHECK(tiny->Y() == Approx(0.7071067811865476));
}

TEST_CASE("negation preserves the unit invariant", "[linear][unitvector3]") {
    const auto u = Vector3{1.0, 2.0, 2.0}.Normalized();
    REQUIRE(u.has_value());

    const UnitVector3 negated = -*u;
    CHECK(negated.X() == Approx(-1.0 / 3.0));
    CHECK(negated.Y() == Approx(-2.0 / 3.0));
    CHECK(negated.Z() == Approx(-2.0 / 3.0));
}

TEST_CASE("scaling a unit vector yields a plain Vector3", "[linear][unitvector3]") {
    const auto u = Vector3{0.0, 0.0, 1.0}.Normalized();
    REQUIRE(u.has_value());

    // 关键的类型契约：缩放后不再是单位向量，返回类型必须改变
    STATIC_REQUIRE(std::is_same_v<decltype(*u * 2.0), Vector3>);
    CHECK(*u * 2.0 == Vector3{0.0, 0.0, 2.0});
    CHECK(2.0 * *u == Vector3{0.0, 0.0, 2.0});
}

TEST_CASE("sums and differences of unit vectors are plain Vector3", "[linear][unitvector3]") {
    const auto x = Vector3{1.0, 0.0, 0.0}.Normalized();
    const auto z = Vector3{0.0, 0.0, 1.0}.Normalized();
    REQUIRE(x.has_value());
    REQUIRE(z.has_value());

    // 规范 §4.4 点名了 u + u 与 u - u：和与差一般都不是单位向量，返回类型必须如实反映这一点，否则不变量会被静默破坏。
    STATIC_REQUIRE(std::is_same_v<decltype(*x + *z), Vector3>);
    STATIC_REQUIRE(std::is_same_v<decltype(*x - *z), Vector3>);

    CHECK(*x + *z == Vector3{1.0, 0.0, 1.0});
    CHECK(*x - *z == Vector3{1.0, 0.0, -1.0});
}

TEST_CASE("dot of two unit vectors is the cosine of the angle", "[linear][unitvector3]") {
    const auto a = Vector3{1.0, 0.0, 0.0}.Normalized();
    const auto b = Vector3{1.0, 1.0, 0.0}.Normalized();
    REQUIRE(a.has_value());
    REQUIRE(b.has_value());

    CHECK(a->Dot(*b) == Approx(0.7071067811865476));
    CHECK(a->Dot(*a) == Approx(1.0));
}

TEST_CASE("cross of two unit vectors is a plain Vector3", "[linear][unitvector3]") {
    const auto x = Vector3{1.0, 0.0, 0.0}.Normalized();
    const auto y = Vector3{0.0, 1.0, 0.0}.Normalized();
    REQUIRE(x.has_value());
    REQUIRE(y.has_value());

    // 数学上叉积仍是单位向量，但两向量接近平行时长度会退化到 0，无法维持不变量，故返回类型刻意是 Vector3。
    STATIC_REQUIRE(std::is_same_v<decltype(x->Cross(*y)), Vector3>);
    CHECK(x->Cross(*y).Z == Approx(1.0));

    const auto parallel = Vector3{1.0, 0.0, 0.0}.Normalized();
    REQUIRE(parallel.has_value());
    CHECK(x->Cross(*parallel).Length() == Approx(0.0));
}

TEST_CASE("normalize rejects non-finite input instead of returning a NaN unit vector", "[linear][unitvector3][degenerate]") {
    const double infinity = std::numeric_limits<double>::infinity();
    const double notANumber = std::numeric_limits<double>::quiet_NaN();

    // 一个 has_value() 为真、内容却是 NaN 的「单位向量」是本库最不该交出的返回值：调用者无从察觉，而 NaN 会一路污染 dot / cross 与所有容差判断。
    CHECK_FALSE(Vector3{infinity, 1.0, 1.0}.Normalized().has_value());
    CHECK_FALSE(Vector3{1.0, infinity, 1.0}.Normalized().has_value());
    CHECK_FALSE(Vector3{infinity, infinity, infinity}.Normalized().has_value());

    // NaN 输入比无穷更常见：任何上游的 0/0 或 inf - inf 都会落到这里
    CHECK_FALSE(Vector3{notANumber, 1.0, 1.0}.Normalized().has_value());
    CHECK_FALSE(Vector3{notANumber, notANumber, notANumber}.Normalized().has_value());
}

TEST_CASE("3D axis constants are the positive unit axes", "[linear][unitvector3]") {
    STATIC_REQUIRE(std::is_same_v<decltype(UnitVector3::XAxis), const UnitVector3>);
    STATIC_REQUIRE(std::is_same_v<decltype(UnitVector3::YAxis), const UnitVector3>);
    STATIC_REQUIRE(std::is_same_v<decltype(UnitVector3::ZAxis), const UnitVector3>);

    CHECK(UnitVector3::XAxis.AsVector() == Vector3{1.0, 0.0, 0.0});
    CHECK(UnitVector3::YAxis.AsVector() == Vector3{0.0, 1.0, 0.0});
    CHECK(UnitVector3::ZAxis.AsVector() == Vector3{0.0, 0.0, 1.0});

    // 右手系：X × Y = Z。某一轴取反会让叉积反向。
    CHECK(UnitVector3::XAxis.Cross(UnitVector3::YAxis) == UnitVector3::ZAxis.AsVector());

    CHECK(UnitVector3f::XAxis.X() == 1.0f);
    CHECK(UnitVector3f::YAxis.Y() == 1.0f);
    CHECK(UnitVector3f::ZAxis.Z() == 1.0f);
}

TEST_CASE("a 3D unit vector's perpendicular matches the frame completed from it", "[linear][unitvector3]") {
    const UnitVector3 axisX = UnitVector3::FromNormalizedUnchecked(Vector3{1.0, 0.0, 0.0});
    const UnitVector3 axisY = UnitVector3::FromNormalizedUnchecked(Vector3{0.0, 1.0, 0.0});
    const UnitVector3 axisZ = UnitVector3::FromNormalizedUnchecked(Vector3{0.0, 0.0, 1.0});

    // 三条坐标轴各走一个分支。期望值与 FromZAxis 的参考轴选择一致： +X → -Z，+Y → +Z，+Z → -Y。
    STATIC_REQUIRE(std::is_same_v<decltype(axisZ.Perpendicular()), UnitVector3>);
    CHECK(axisX.Perpendicular() == UnitVector3::FromNormalizedUnchecked(Vector3{0.0, 0.0, -1.0}));
    CHECK(axisY.Perpendicular() == UnitVector3::FromNormalizedUnchecked(Vector3{0.0, 0.0, 1.0}));
    CHECK(axisZ.Perpendicular() == UnitVector3::FromNormalizedUnchecked(Vector3{0.0, -1.0, 0.0}));
    CHECK(axisZ.Dot(axisZ.Perpendicular()) == 0.0);
    STATIC_REQUIRE(noexcept(axisZ.Perpendicular()));

    // |z| 最小的分支：z = (0.6, 0.8, 0) ⇒ 垂直向量 (-0.8, 0.6, 0)。
    const UnitVector3 flat = UnitVector3::FromNormalizedUnchecked(Vector3{0.6, 0.8, 0.0});
    CHECK(flat.Perpendicular().X() == Approx(-0.8).margin(1e-15));
    CHECK(flat.Perpendicular().Y() == Approx(0.6).margin(1e-15));
    CHECK(flat.Perpendicular().Z() == Approx(0.0).margin(1e-15));

    // 与 FromZAxis 补出的 X 轴是同一个向量。
    const auto z = Vector3{1.0, 1.0, 1.0}.Normalized();
    REQUIRE(z.has_value());
    const auto frame = Coordinate3::FromZAxis(Point3{}, *z);
    REQUIRE(frame.has_value());
    CHECK(frame->XAxis() == z->Perpendicular());
}

TEST_CASE("projecting onto a 3D unit vector keeps only the parallel part", "[linear][unitvector3]") {
    const UnitVector3 axisZ = UnitVector3::FromNormalizedUnchecked(Vector3{0.0, 0.0, 1.0});

    STATIC_REQUIRE(std::is_same_v<decltype(axisZ.Projected(Vector3{1.0, 2.0, 3.0})), Vector3>);
    CHECK(axisZ.Projected(Vector3{1.0, 2.0, 3.0}) == Vector3{0.0, 0.0, 3.0});
    CHECK(axisZ.Projected(Vector3{1.0, 2.0, -4.0}) == Vector3{0.0, 0.0, -4.0});
    CHECK(axisZ.Projected(Vector3{1.0, 2.0, 0.0}) == Vector3{0.0, 0.0, 0.0});

    const auto normal = Vector3{1.0, 2.0, 2.0}.Normalized();
    REQUIRE(normal.has_value());
    const Vector3 parallel = normal->Projected(Vector3{1.0, 2.0, 2.0});
    CHECK(parallel.X == Approx(1.0));
    CHECK(parallel.Y == Approx(2.0));
    CHECK(parallel.Z == Approx(2.0));
    CHECK(normal->Projected(normal->Perpendicular().AsVector()).Length() == Approx(0.0).margin(1e-12));

    constexpr UnitVector3 axis = UnitVector3::FromNormalizedUnchecked(Vector3{0.0, 1.0, 0.0});
    STATIC_REQUIRE(axis.Projected(Vector3{3.0, 4.0, 5.0}) == Vector3{0.0, 4.0, 0.0});
    STATIC_REQUIRE(noexcept(axis.Projected(Vector3{1.0, 2.0, 3.0})));
}

TEST_CASE("UnitVector3 AngleBetween returns acute or obtuse angle in [0, pi]", "[linear][unitvector3]") {
    using DragonGeo::Core::HALF_PI;

    const UnitVector3 x = UnitVector3::FromNormalizedUnchecked(Vector3{1.0, 0.0, 0.0});
    const UnitVector3 y = UnitVector3::FromNormalizedUnchecked(Vector3{0.0, 1.0, 0.0});

    CHECK(x.AngleBetween(y) == Approx(HALF_PI));
    CHECK(x.AngleBetween(-x) == Approx(HALF_PI + HALF_PI));
    CHECK(x.AngleBetween(x) == Approx(0.0));
}
