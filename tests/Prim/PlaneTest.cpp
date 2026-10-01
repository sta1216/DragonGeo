#include <catch2/catch_test_macros.hpp>

#include <limits>
#include <type_traits>

#include <DragonGeo/Prim/Plane.hpp>

using DragonGeo::Linear::Point3;
using DragonGeo::Linear::UnitVector3;
using DragonGeo::Linear::Vector3;
using DragonGeo::Prim::Plane;
using DragonGeo::Prim::PlaneT;
using DragonGeo::Prim::Planef;

TEST_CASE("Plane is an aggregate of an origin and a normal", "[prim][plane]") {
    const auto normal = UnitVector3::FromNormalizedUnchecked(Vector3{0.0, 0.0, 1.0});
    const Plane plane{Point3{0.0, 1.0, 2.0}, normal};
    CHECK(plane.Origin == Point3{0.0, 1.0, 2.0});
    CHECK(plane.Normal == normal);
    CHECK(plane.IsValid());
    CHECK(plane == Plane{Point3{0.0, 1.0, 2.0}, normal});
    CHECK(plane != Plane{Point3{0.0, 1.0, 9.0}, normal});
    const auto otherNormal = UnitVector3::FromNormalizedUnchecked(Vector3{1.0, 0.0, 0.0});
    CHECK(plane != Plane{Point3{0.0, 1.0, 2.0}, otherNormal});
    STATIC_REQUIRE(std::is_aggregate_v<Plane>);
    STATIC_REQUIRE(std::is_same_v<Planef, PlaneT<float>>);
}

TEST_CASE("a Plane with a finite non-unit normal is valid", "[prim][plane]") {
    const auto normal = UnitVector3::FromNormalizedUnchecked(Vector3{0.0, 3.0, 0.0});
    const Plane plane{Point3{0.0, 0.0, 0.0}, normal};
    CHECK(plane.IsValid());
}

TEST_CASE("Plane with a non-finite coordinate is invalid", "[prim][plane]") {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const auto normal = UnitVector3::FromNormalizedUnchecked(Vector3{0.0, 0.0, 1.0});
    const Plane nanOrigin{Point3{0.0, 0.0, nan}, normal};
    CHECK_FALSE(nanOrigin.IsValid());

    const auto nanNormal = UnitVector3::FromNormalizedUnchecked(Vector3{nan, 0.0, 0.0});
    const Plane nanNormalPlane{Point3{0.0, 0.0, 0.0}, nanNormal};
    CHECK_FALSE(nanNormalPlane.IsValid());
}

TEST_CASE("Plane signed distance, distance, closest point, and flipped normal", "[prim][plane]") {
    const auto normal = UnitVector3::FromNormalizedUnchecked(Vector3{0.0, 0.0, 1.0});
    const Plane plane{Point3{0.0, 0.0, 0.0}, normal};

    CHECK(plane.SignedDistance(Point3{0.0, 0.0, 2.0}) == 2.0);
    CHECK(plane.Distance(Point3{0.0, 0.0, 2.0}) == 2.0);
    CHECK(plane.ClosestPoint(Point3{0.0, 0.0, 2.0}) == Point3{0.0, 0.0, 0.0});

    CHECK(plane.SignedDistance(Point3{4.0, -2.0, -3.0}) == -3.0);
    CHECK(plane.Distance(Point3{4.0, -2.0, -3.0}) == 3.0);
    CHECK(plane.ClosestPoint(Point3{4.0, -2.0, -3.0}) == Point3{4.0, -2.0, 0.0});

    CHECK(plane.SignedDistance(Point3{1.0, 2.0, 0.0}) == 0.0);
    CHECK(plane.Distance(Point3{1.0, 2.0, 0.0}) == 0.0);
    CHECK(plane.ClosestPoint(Point3{1.0, 2.0, 0.0}) == Point3{1.0, 2.0, 0.0});

    const Plane flipped = plane.Flipped();
    CHECK(flipped.Origin == plane.Origin);
    CHECK(flipped.Normal == -normal);
    CHECK(flipped != plane);
    CHECK(flipped.SignedDistance(Point3{0.0, 0.0, 2.0}) == -2.0);

    constexpr auto constantNormal = UnitVector3::FromNormalizedUnchecked(Vector3{0.0, 0.0, 1.0});
    constexpr Plane constantPlane{Point3{0.0, 0.0, 0.0}, constantNormal};
    STATIC_REQUIRE(constantPlane.SignedDistance(Point3{0.0, 0.0, 2.0}) == 2.0);
    STATIC_REQUIRE(constantPlane.Distance(Point3{4.0, -2.0, -3.0}) == 3.0);
    STATIC_REQUIRE(constantPlane.ClosestPoint(Point3{0.0, 0.0, 2.0}) == Point3{0.0, 0.0, 0.0});
    STATIC_REQUIRE(constantPlane.Flipped().Origin == constantPlane.Origin);
    STATIC_REQUIRE(constantPlane.Flipped().Normal == -constantNormal);
    STATIC_REQUIRE(constantPlane.Flipped() != constantPlane);
    STATIC_REQUIRE(noexcept(plane.SignedDistance(Point3{})));
    STATIC_REQUIRE(noexcept(plane.Distance(Point3{})));
    STATIC_REQUIRE(noexcept(plane.ClosestPoint(Point3{})));
    STATIC_REQUIRE(noexcept(plane.Flipped()));
}
