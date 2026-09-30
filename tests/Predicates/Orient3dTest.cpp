#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdint>
#include <limits>

#include <DragonGeo/Predicates/Predicates.hpp>

using DragonGeo::Linear::Point3;
using DragonGeo::Predicates::Orient3d;

namespace {

int Orient3dInt(long long ax, long long ay, long long az,
                 long long bx, long long by, long long bz,
                 long long cx, long long cy, long long cz,
                 long long dx, long long dy, long long dz) {
    const long long abx = bx - ax;
    const long long aby = by - ay;
    const long long abz = bz - az;
    const long long acx = cx - ax;
    const long long acy = cy - ay;
    const long long acz = cz - az;
    const long long adx = dx - ax;
    const long long ady = dy - ay;
    const long long adz = dz - az;
    const long long det = adx * (aby * acz - abz * acy)
        + ady * (abz * acx - abx * acz)
        + adz * (abx * acy - aby * acx);
    if (det > 0) {
        return 1;
    }
    if (det < 0) {
        return -1;
    }
    return 0;
}

} // namespace

TEST_CASE("Orient3d is positive for a right-handed tetrahedron", "[predicates][orient3d]") {
    CHECK(Orient3d(Point3{0.0, 0.0, 0.0}, Point3{1.0, 0.0, 0.0}, Point3{0.0, 1.0, 0.0},
                   Point3{0.0, 0.0, 1.0}) == 1);
    CHECK(Orient3d(Point3{0.0, 0.0, 0.0}, Point3{1.0, 0.0, 0.0}, Point3{0.0, 1.0, 0.0},
                   Point3{0.0, 0.0, -1.0}) == -1);
}

TEST_CASE("Orient3d is zero for a coplanar or coincident point", "[predicates][orient3d]") {
    CHECK(Orient3d(Point3{0.0, 0.0, 0.0}, Point3{1.0, 0.0, 0.0}, Point3{0.0, 1.0, 0.0},
                   Point3{2.0, 3.0, 0.0}) == 0);
    CHECK(Orient3d(Point3{1.0, 2.0, 3.0}, Point3{1.0, 2.0, 3.0}, Point3{4.0, 5.0, 6.0},
                   Point3{7.0, 8.0, 9.0}) == 0);
}

TEST_CASE("swapping two Orient3d arguments negates the sign", "[predicates][orient3d]") {
    const Point3 a{0.0, 0.0, 0.0};
    const Point3 b{1.0, 0.0, 0.0};
    const Point3 c{0.0, 1.0, 0.0};
    const Point3 d{0.0, 0.0, 1.0};
    CHECK(Orient3d(a, b, c, d) == -Orient3d(a, c, b, d));
}

TEST_CASE("Orient3d matches the int64 determinant on small integer points",
          "[predicates][orient3d]") {
    std::uint32_t state = 7;
    for (int sample = 0; sample < 100; ++sample) {
        long long coords[12];
        for (int i = 0; i < 12; ++i) {
            state = state * 1664525u + 1013904223u;
            coords[i] = static_cast<long long>(state % 2001u) - 1000;
        }
        const Point3 a{static_cast<double>(coords[0]), static_cast<double>(coords[1]),
                       static_cast<double>(coords[2])};
        const Point3 b{static_cast<double>(coords[3]), static_cast<double>(coords[4]),
                       static_cast<double>(coords[5])};
        const Point3 c{static_cast<double>(coords[6]), static_cast<double>(coords[7]),
                       static_cast<double>(coords[8])};
        const Point3 d{static_cast<double>(coords[9]), static_cast<double>(coords[10]),
                       static_cast<double>(coords[11])};
        CHECK(Orient3d(a, b, c, d) == Orient3dInt(
            coords[0], coords[1], coords[2], coords[3], coords[4], coords[5],
            coords[6], coords[7], coords[8], coords[9], coords[10], coords[11]));
    }
}

TEST_CASE("Orient3d of a one-ulp lift off the plane is positive", "[predicates][orient3d]") {
    // 过滤会弃权。(0,0,0)、(1,1,0) 与 (0.5,0.5,0) 落在 z=0 平面上。
    // 把第三点的 Y 抬高一个 ulp 后，(b-a)×(c-a) 指向 +z，d 在正侧。
    const double lifted = std::nextafter(0.5, std::numeric_limits<double>::infinity());
    const double lowered = std::nextafter(0.5, -std::numeric_limits<double>::infinity());
    const Point3 a{0.0, 0.0, 0.0};
    const Point3 b{1.0, 1.0, 0.0};
    const Point3 d{0.0, 0.0, 1.0};
    CHECK(Orient3d(a, b, Point3{0.5, lifted, 0.0}, d) == 1);
    CHECK(Orient3d(a, b, Point3{0.5, lowered, 0.0}, d) == -1);
}

TEST_CASE("Orient3d keeps the sign of a one-ulp lift at magnitude 2^50",
          "[predicates][orient3d]") {
    const double scale = std::ldexp(1.0, 50);
    const double mid = std::ldexp(1.0, 49);
    const double lifted = std::nextafter(mid, std::numeric_limits<double>::infinity());
    CHECK(Orient3d(Point3{0.0, 0.0, 0.0}, Point3{scale, scale, 0.0}, Point3{mid, lifted, 0.0},
                   Point3{0.0, 0.0, 1.0}) == 1);
}

TEST_CASE("Orient3d is noexcept", "[predicates][orient3d]") {
    STATIC_REQUIRE(noexcept(Orient3d(Point3{}, Point3{}, Point3{}, Point3{})));
}
