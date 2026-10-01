#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <limits>
#include <type_traits>

#include <DragonGeo/Core/Constants.hpp>
#include <DragonGeo/Linear/UnitVector2.hpp>

using Catch::Approx;

using DragonGeo::Linear::UnitVector2;
using DragonGeo::Linear::UnitVector2T;
using DragonGeo::Linear::UnitVector2f;
using DragonGeo::Linear::Vector2;

TEST_CASE("the float alias really is the float instantiation", "[linear][unitvector2]") {
    // 同 vector4Test.cpp：UnitVector2f 此前零命中，误绑定不会被断言发现。
    STATIC_REQUIRE(std::is_same_v<UnitVector2f, UnitVector2T<float>>);
}

TEST_CASE("normalize produces a unit-length 2D vector", "[linear][unitvector2]") {
    const auto result = Vector2{3.0, 4.0}.Normalized();

    REQUIRE(result.has_value());
    CHECK(result->X() == Approx(0.6));
    CHECK(result->Y() == Approx(0.8));
    CHECK(result->AsVector().Length() == Approx(1.0));
}

TEST_CASE("normalize rejects the 2D zero vector", "[linear][unitvector2][degenerate]") {
    CHECK_FALSE(Vector2{0.0, 0.0}.Normalized().has_value());
}

TEST_CASE("2D cross of unit vectors is the sine of the angle", "[linear][unitvector2]") {
    const auto x = Vector2{1.0, 0.0}.Normalized();
    const auto y = Vector2{0.0, 1.0}.Normalized();
    REQUIRE(x.has_value());
    REQUIRE(y.has_value());

    CHECK(x->Cross(*y) == Approx(1.0));
    CHECK(y->Cross(*x) == Approx(-1.0));
    CHECK(x->Cross(*x) == Approx(0.0));

    CHECK(x->Dot(*y) == Approx(0.0));
    CHECK(x->Dot(*x) == Approx(1.0));

    CHECK(-(*x) == UnitVector2::FromNormalizedUnchecked(Vector2{-1.0, 0.0}));
    CHECK(*x * 2.0 == Vector2{2.0, 0.0});
    CHECK(3.0 * *y == Vector2{0.0, 3.0});
    STATIC_REQUIRE(std::is_same_v<decltype(*x + *y), Vector2>);
    STATIC_REQUIRE(std::is_same_v<decltype(*x - *y), Vector2>);
    CHECK(*x + *y == Vector2{1.0, 1.0});
    CHECK(*x - *y == Vector2{1.0, -1.0});
}

TEST_CASE("normalize rejects non-finite input instead of returning a NaN unit vector", "[linear][unitvector2][degenerate]") {
    const double infinity = std::numeric_limits<double>::infinity();
    const double notANumber = std::numeric_limits<double>::quiet_NaN();

    // 理由同 UnitVector3T：交出一个内容为 NaN 的「单位向量」比返回 nullopt 危险得多。
    CHECK_FALSE(Vector2{infinity, 1.0}.Normalized().has_value());
    CHECK_FALSE(Vector2{1.0, infinity}.Normalized().has_value());
    CHECK_FALSE(Vector2{infinity, infinity}.Normalized().has_value());

    CHECK_FALSE(Vector2{notANumber, 1.0}.Normalized().has_value());
    CHECK_FALSE(Vector2{notANumber, notANumber}.Normalized().has_value());
}

TEST_CASE("2D axis constants are the positive unit axes", "[linear][unitvector2]") {
    STATIC_REQUIRE(std::is_same_v<decltype(UnitVector2::XAxis), const UnitVector2>);
    STATIC_REQUIRE(std::is_same_v<decltype(UnitVector2::YAxis), const UnitVector2>);

    CHECK(UnitVector2::XAxis.X() == 1.0);
    CHECK(UnitVector2::XAxis.Y() == 0.0);
    CHECK(UnitVector2::YAxis.X() == 0.0);
    CHECK(UnitVector2::YAxis.Y() == 1.0);

    // 正 X 的逆时针垂直是正 Y。两轴对调或 Y 取负都会失败。
    CHECK(UnitVector2::XAxis.Perpendicular() == UnitVector2::YAxis);
    CHECK(UnitVector2::XAxis.Cross(UnitVector2::YAxis) == 1.0);

    CHECK(UnitVector2f::XAxis.X() == 1.0f);
    CHECK(UnitVector2f::YAxis.Y() == 1.0f);
}

TEST_CASE("a 2D unit vector's perpendicular is a 90 degree counter-clockwise turn", "[linear][unitvector2]") {
    const UnitVector2 x = UnitVector2::FromNormalizedUnchecked(Vector2{1.0, 0.0});
    const UnitVector2 y = UnitVector2::FromNormalizedUnchecked(Vector2{0.0, 1.0});

    // 保住的是单位长度，所以返回 UnitVector2，而不是 Vector2。
    STATIC_REQUIRE(std::is_same_v<decltype(x.Perpendicular()), UnitVector2>);
    CHECK(x.Perpendicular() == y);
    CHECK(y.Perpendicular() == -x);
    CHECK(x.Dot(x.Perpendicular()) == 0.0);
    CHECK(x.Cross(x.Perpendicular()) == 1.0);

    // 非轴向：(0.6, 0.8) 的逆时针垂直是 (-0.8, 0.6)。顺时针会得到 (0.8, -0.6)。
    const UnitVector2 tilted = UnitVector2::FromNormalizedUnchecked(Vector2{0.6, 0.8});
    CHECK(tilted.Perpendicular() == UnitVector2::FromNormalizedUnchecked(Vector2{-0.8, 0.6}));

    constexpr UnitVector2 axis = UnitVector2::FromNormalizedUnchecked(Vector2{1.0, 0.0});
    STATIC_REQUIRE(axis.Perpendicular() == UnitVector2::FromNormalizedUnchecked(Vector2{0.0, 1.0}));
    STATIC_REQUIRE(noexcept(axis.Perpendicular()));
}

TEST_CASE("projecting onto a 2D unit vector keeps only the parallel part", "[linear][unitvector2]") {
    const UnitVector2 x = UnitVector2::FromNormalizedUnchecked(Vector2{1.0, 0.0});

    STATIC_REQUIRE(std::is_same_v<decltype(x.Projected(Vector2{3.0, 4.0})), Vector2>);
    CHECK(x.Projected(Vector2{3.0, 4.0}) == Vector2{3.0, 0.0});
    CHECK(x.Projected(Vector2{-2.0, 5.0}) == Vector2{-2.0, 0.0});
    CHECK(x.Projected(Vector2{0.0, 4.0}) == Vector2{0.0, 0.0});
    CHECK(x.Projected(Vector2{0.0, 0.0}) == Vector2{0.0, 0.0});

    // 斜方向：与法向垂直的向量投影为零，与法向平行的向量原样留下。
    const auto normal = Vector2{3.0, 4.0}.Normalized();
    REQUIRE(normal.has_value());
    CHECK(normal->Projected(Vector2{-4.0, 3.0}).Length() == Approx(0.0).margin(1e-12));
    const Vector2 parallel = normal->Projected(Vector2{3.0, 4.0});
    CHECK(parallel.X == Approx(3.0));
    CHECK(parallel.Y == Approx(4.0));

    constexpr UnitVector2 axis = UnitVector2::FromNormalizedUnchecked(Vector2{0.0, 1.0});
    STATIC_REQUIRE(axis.Projected(Vector2{3.0, 4.0}) == Vector2{0.0, 4.0});
    STATIC_REQUIRE(noexcept(axis.Projected(Vector2{1.0, 2.0})));
}

TEST_CASE("UnitVector2 AngleBetween and SignedAngle", "[linear][unitvector2]") {
    using DragonGeo::Core::HALF_PI;
    using DragonGeo::Core::QUARTER_PI;

    const UnitVector2 x = UnitVector2::FromNormalizedUnchecked(Vector2{1.0, 0.0});
    const UnitVector2 y = UnitVector2::FromNormalizedUnchecked(Vector2{0.0, 1.0});

    CHECK(x.AngleBetween(y) == Approx(HALF_PI));
    CHECK(x.AngleBetween(-x) == Approx(HALF_PI + HALF_PI));
    CHECK(x.AngleBetween(x) == Approx(0.0));

    CHECK(x.SignedAngle(y) == Approx(HALF_PI));
    CHECK(y.SignedAngle(x) == Approx(-HALF_PI));
    CHECK(x.SignedAngle(x) == Approx(0.0));

    const UnitVector2 diag = UnitVector2::FromNormalizedUnchecked(Vector2{1.0, 1.0});
    CHECK(x.SignedAngle(diag) == Approx(QUARTER_PI));
}
