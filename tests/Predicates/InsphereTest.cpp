#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>

#include <DragonGeo/Predicates/Predicates.hpp>

using DragonGeo::Linear::Point3;
using DragonGeo::Predicates::Insphere;
using DragonGeo::Predicates::Orient3d;

namespace {

int InsphereInt(long long ax, long long ay, long long az,
                 long long bx, long long by, long long bz,
                 long long cx, long long cy, long long cz,
                 long long dx, long long dy, long long dz,
                 long long ex, long long ey, long long ez) {
    const auto comp = [](long long px, long long py, long long pz, long long qx, long long qy,
                          long long qz) {
        return std::array<long long, 3>{px - qx, py - qy, pz - qz};
    };
    const auto a = comp(ax, ay, az, ex, ey, ez);
    const auto b = comp(bx, by, bz, ex, ey, ez);
    const auto c = comp(cx, cy, cz, ex, ey, ez);
    const auto d = comp(dx, dy, dz, ex, ey, ez);
    const auto lift = [](const std::array<long long, 3>& v) {
        return v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
    };
    const auto triple = [](const std::array<long long, 3>& u, const std::array<long long, 3>& v,
                            const std::array<long long, 3>& w) {
        const long long cx = v[1] * w[2] - v[2] * w[1];
        const long long cy = v[2] * w[0] - v[0] * w[2];
        const long long cz = v[0] * w[1] - v[1] * w[0];
        return u[0] * cx + u[1] * cy + u[2] * cz;
    };
    const long long det = lift(a) * triple(b, c, d) - lift(b) * triple(a, c, d)
        + lift(c) * triple(a, b, d) - lift(d) * triple(a, b, c);
    if (det > 0) {
        return 1;
    }
    if (det < 0) {
        return -1;
    }
    return 0;
}

} // namespace

TEST_CASE("Insphere follows the orientation of the defining tetrahedron",
          "[predicates][insphere]") {
    const Point3 a{1.0, 0.0, 0.0};
    const Point3 b{0.0, 0.0, 1.0};
    const Point3 c{0.0, 1.0, 0.0};
    const Point3 d{-1.0, 0.0, 0.0};
    REQUIRE(Orient3d(a, b, c, d) == 1);
    CHECK(Insphere(a, b, c, d, Point3{0.0, 0.0, 0.0}) == 1);
    CHECK(Insphere(a, b, c, d, Point3{3.0, 3.0, 3.0}) == -1);
    CHECK(Insphere(a, b, c, d, Point3{0.0, -1.0, 0.0}) == 0);

    const Point3 nb{0.0, 1.0, 0.0};
    const Point3 nc{0.0, 0.0, 1.0};
    const Point3 nd{0.0, 0.0, -1.0};
    REQUIRE(Orient3d(a, nb, nc, nd) == -1);
    CHECK(Insphere(a, nb, nc, nd, Point3{0.0, 0.0, 0.0}) == -1);
}

TEST_CASE("Insphere of a one-ulp lift off the sphere follows the interior",
          "[predicates][insphere]") {
    const Point3 a{1.0, 0.0, 0.0};
    const Point3 b{0.0, 0.0, 1.0};
    const Point3 c{0.0, 1.0, 0.0};
    const Point3 d{-1.0, 0.0, 0.0};
    const double inward = std::nextafter(-1.0, 0.0);
    const double outward = std::nextafter(-1.0, -std::numeric_limits<double>::infinity());
    CHECK(Insphere(a, b, c, d, Point3{0.0, inward, 0.0}) == 1);
    CHECK(Insphere(a, b, c, d, Point3{0.0, outward, 0.0}) == -1);
}

TEST_CASE("Insphere matches the int64 determinant on small integer points",
          "[predicates][insphere]") {
    std::uint32_t state = 13;
    for (int sample = 0; sample < 40; ++sample) {
        long long coords[15];
        for (int i = 0; i < 15; ++i) {
            state = state * 1664525u + 1013904223u;
            coords[i] = static_cast<long long>(state % 401u) - 200;
        }
        const Point3 a{static_cast<double>(coords[0]), static_cast<double>(coords[1]),
                       static_cast<double>(coords[2])};
        const Point3 b{static_cast<double>(coords[3]), static_cast<double>(coords[4]),
                       static_cast<double>(coords[5])};
        const Point3 c{static_cast<double>(coords[6]), static_cast<double>(coords[7]),
                       static_cast<double>(coords[8])};
        const Point3 d{static_cast<double>(coords[9]), static_cast<double>(coords[10]),
                       static_cast<double>(coords[11])};
        const Point3 e{static_cast<double>(coords[12]), static_cast<double>(coords[13]),
                       static_cast<double>(coords[14])};
        CHECK(Insphere(a, b, c, d, e) == InsphereInt(
            coords[0], coords[1], coords[2], coords[3], coords[4], coords[5],
            coords[6], coords[7], coords[8], coords[9], coords[10], coords[11],
            coords[12], coords[13], coords[14]));
    }
}

TEST_CASE("Insphere is noexcept", "[predicates][insphere]") {
    STATIC_REQUIRE(noexcept(Insphere(Point3{}, Point3{}, Point3{}, Point3{}, Point3{})));
}
