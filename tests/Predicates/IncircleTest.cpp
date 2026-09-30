#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdint>
#include <limits>

#include <DragonGeo/Predicates/Predicates.hpp>

using DragonGeo::Linear::Point2;
using DragonGeo::Predicates::Incircle;

namespace {

int IncircleInt(long long ax, long long ay, long long bx, long long by,
                 long long cx, long long cy, long long dx, long long dy) {
    const long long adx = ax - dx;
    const long long ady = ay - dy;
    const long long bdx = bx - dx;
    const long long bdy = by - dy;
    const long long cdx = cx - dx;
    const long long cdy = cy - dy;
    const long long alift = adx * adx + ady * ady;
    const long long blift = bdx * bdx + bdy * bdy;
    const long long clift = cdx * cdx + cdy * cdy;
    const long long det = alift * (bdx * cdy - cdx * bdy)
        + blift * (cdx * ady - adx * cdy)
        + clift * (adx * bdy - bdx * ady);
    if (det > 0) {
        return 1;
    }
    if (det < 0) {
        return -1;
    }
    return 0;
}

} // namespace

TEST_CASE("Incircle of the unit circle follows the boundary orientation",
          "[predicates][incircle]") {
    const Point2 a{1.0, 0.0};
    const Point2 b{0.0, 1.0};
    const Point2 c{-1.0, 0.0};
    CHECK(Incircle(a, b, c, Point2{0.0, 0.0}) == 1);
    CHECK(Incircle(a, b, c, Point2{2.0, 0.0}) == -1);
    CHECK(Incircle(a, b, c, Point2{0.0, -1.0}) == 0);
    // 边界改成顺时针后，同一个内点必须变号。
    CHECK(Incircle(a, c, b, Point2{0.0, 0.0}) == -1);
}

TEST_CASE("Incircle is zero when the query point repeats a boundary point",
          "[predicates][incircle]") {
    const Point2 a{1.0, 0.0};
    const Point2 b{0.0, 1.0};
    const Point2 c{-1.0, 0.0};
    CHECK(Incircle(a, b, c, a) == 0);
}

TEST_CASE("Incircle matches the int64 determinant on small integer points",
          "[predicates][incircle]") {
    std::uint32_t state = 11;
    for (int sample = 0; sample < 100; ++sample) {
        long long coords[8];
        for (int i = 0; i < 8; ++i) {
            state = state * 1664525u + 1013904223u;
            coords[i] = static_cast<long long>(state % 2001u) - 1000;
        }
        const Point2 a{static_cast<double>(coords[0]), static_cast<double>(coords[1])};
        const Point2 b{static_cast<double>(coords[2]), static_cast<double>(coords[3])};
        const Point2 c{static_cast<double>(coords[4]), static_cast<double>(coords[5])};
        const Point2 d{static_cast<double>(coords[6]), static_cast<double>(coords[7])};
        CHECK(Incircle(a, b, c, d) == IncircleInt(coords[0], coords[1], coords[2], coords[3],
                                                   coords[4], coords[5], coords[6], coords[7]));
    }
}

TEST_CASE("Incircle of a one-ulp lift off the square follows inside and outside",
          "[predicates][incircle]") {
    // 过滤会弃权。(0,0)、(1,0)、(1,1)、(0,1) 共圆，前三点逆时针。
    // 把第四点的 Y 抬高一个 ulp 后它在圆外；降低一个 ulp 后在圆内。
    const Point2 a{0.0, 0.0};
    const Point2 b{1.0, 0.0};
    const Point2 c{1.0, 1.0};
    const double outward = std::nextafter(1.0, std::numeric_limits<double>::infinity());
    const double inward = std::nextafter(1.0, 0.0);
    CHECK(Incircle(a, b, c, Point2{0.0, outward}) == -1);
    CHECK(Incircle(a, b, c, Point2{0.0, inward}) == 1);
}

TEST_CASE("Incircle keeps the sign of a one-ulp lift at magnitude 2^50",
          "[predicates][incircle]") {
    const double scale = std::ldexp(1.0, 50);
    const double outward = std::nextafter(scale, std::numeric_limits<double>::infinity());
    CHECK(Incircle(Point2{0.0, 0.0}, Point2{scale, 0.0}, Point2{scale, scale},
                   Point2{0.0, outward}) == -1);
}

TEST_CASE("Incircle is noexcept", "[predicates][incircle]") {
    STATIC_REQUIRE(noexcept(Incircle(Point2{}, Point2{}, Point2{}, Point2{})));
}
