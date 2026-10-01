#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdint>
#include <limits>

#include <DragonGeo/Predicates/Predicates.hpp>

using DragonGeo::Linear::Point2;
using DragonGeo::Predicates::Orient2d;

namespace {

int Orient2dInt(long long ax, long long ay, long long bx, long long by, long long cx, long long cy) {
    const long long det = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
    if (det > 0) {
        return 1;
    }
    if (det < 0) {
        return -1;
    }
    return 0;
}

} // namespace

TEST_CASE("Orient2d is positive for a left turn", "[predicates][orient2d]") {
    CHECK(Orient2d(Point2{0.0, 0.0}, Point2{1.0, 0.0}, Point2{0.0, 1.0}) == 1);
    CHECK(Orient2d(Point2{0.0, 0.0}, Point2{0.0, 1.0}, Point2{1.0, 0.0}) == -1);
}

TEST_CASE("Orient2d is zero when the points are collinear or coincident", "[predicates][orient2d]") {
    CHECK(Orient2d(Point2{0.0, 0.0}, Point2{1.0, 1.0}, Point2{2.0, 2.0}) == 0);
    CHECK(Orient2d(Point2{0.0, 0.0}, Point2{1.0, 1.0}, Point2{0.5, 0.5}) == 0);
    CHECK(Orient2d(Point2{3.0, 4.0}, Point2{3.0, 4.0}, Point2{9.0, 1.0}) == 0);
    CHECK(Orient2d(Point2{-3.0, 4.0}, Point2{0.0, 0.0}, Point2{3.0, -4.0}) == 0);
}

TEST_CASE("Orient2d of a one-ulp lift off the diagonal is positive", "[predicates][orient2d]") {
    // 过滤会弃权。(0,0)、(1,1) 与 (0.5, 0.5) 共线；把第三点的 Y 抬高一个 ulp 之后，精确行列式为正。
    const double lifted = std::nextafter(0.5, std::numeric_limits<double>::infinity());
    const double lowered = std::nextafter(0.5, -std::numeric_limits<double>::infinity());
    CHECK(Orient2d(Point2{0.0, 0.0}, Point2{1.0, 1.0}, Point2{0.5, lifted}) == 1);
    CHECK(Orient2d(Point2{0.0, 0.0}, Point2{1.0, 1.0}, Point2{0.5, lowered}) == -1);
}

TEST_CASE("Orient2d keeps the sign of a one-ulp lift at magnitude 2^50", "[predicates][orient2d]") {
    const double scale = std::ldexp(1.0, 50);
    const double mid = std::ldexp(1.0, 49);
    const double lifted = std::nextafter(mid, std::numeric_limits<double>::infinity());
    CHECK(Orient2d(Point2{0.0, 0.0}, Point2{scale, scale}, Point2{mid, lifted}) == 1);
}

TEST_CASE("swapping two Orient2d arguments negates the sign", "[predicates][orient2d]") {
    const Point2 a{0.0, 0.0};
    const Point2 b{1.0, 1.0};
    const Point2 c{0.5, std::nextafter(0.5, std::numeric_limits<double>::infinity())};
    CHECK(Orient2d(a, b, c) == -Orient2d(a, c, b));
    CHECK(Orient2d(a, b, c) == -Orient2d(b, a, c));
}

TEST_CASE("integer translation and positive power-of-two scaling keep Orient2d", "[predicates][orient2d]") {
    const Point2 a{1.0, 2.0};
    const Point2 b{4.0, 2.0};
    const Point2 c{4.0, 6.0};
    const int sign = Orient2d(a, b, c);
    CHECK(Orient2d(Point2{a.X + 3.0, a.Y - 5.0}, Point2{b.X + 3.0, b.Y - 5.0}, Point2{c.X + 3.0, c.Y - 5.0}) == sign);
    CHECK(Orient2d(Point2{a.X * 4.0, a.Y * 4.0}, Point2{b.X * 4.0, b.Y * 4.0}, Point2{c.X * 4.0, c.Y * 4.0}) == sign);
}

TEST_CASE("Orient2d matches the int64 determinant on small integer points", "[predicates][orient2d]") {
    std::uint32_t state = 1;
    for (int sample = 0; sample < 200; ++sample) {
        long long coords[6];
        for (int i = 0; i < 6; ++i) {
            state = state * 1664525u + 1013904223u;
            coords[i] = static_cast<long long>(state % 2001u) - 1000;
        }
        const Point2 a{static_cast<double>(coords[0]), static_cast<double>(coords[1])};
        const Point2 b{static_cast<double>(coords[2]), static_cast<double>(coords[3])};
        const Point2 c{static_cast<double>(coords[4]), static_cast<double>(coords[5])};
        CHECK(Orient2d(a, b, c) == Orient2dInt(coords[0], coords[1], coords[2], coords[3], coords[4], coords[5]));
    }
}

TEST_CASE("Orient2d is noexcept", "[predicates][orient2d]") {
    STATIC_REQUIRE(noexcept(Orient2d(Point2{}, Point2{}, Point2{})));
}
