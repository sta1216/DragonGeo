#include <DragonGeo/Predicates/Predicates.hpp>

#include <algorithm>
#include <array>
#include <cmath>

#include "Expansion.hpp"

namespace DragonGeo::Predicates {

int Orient2d(Linear::Point2 a, Linear::Point2 b, Linear::Point2 c) noexcept {
    const double detLeft = (b.X - a.X) * (c.Y - a.Y);
    const double detRight = (b.Y - a.Y) * (c.X - a.X);
    const double det = detLeft - detRight;
    const double permanent = std::abs(detLeft) + std::abs(detRight);
    if (std::isfinite(det) && std::isfinite(permanent)
        && std::abs(det) >= Detail::ORIENT2D_ERROR_BOUND * permanent) {
        return Detail::SignOf(det);
    }

    const double maxAbs = std::max({std::abs(a.X), std::abs(a.Y), std::abs(b.X), std::abs(b.Y),
                                     std::abs(c.X), std::abs(c.Y)});
    const int shift = Detail::ScaleShift(maxAbs);
    const double ax = std::ldexp(a.X, shift);
    const double ay = std::ldexp(a.Y, shift);
    const double bx = std::ldexp(b.X, shift);
    const double by = std::ldexp(b.Y, shift);
    const double cx = std::ldexp(c.X, shift);
    const double cy = std::ldexp(c.Y, shift);

    const Detail::Expansion abx = Detail::DifferenceOfScalars(bx, ax);
    const Detail::Expansion aby = Detail::DifferenceOfScalars(by, ay);
    const Detail::Expansion acx = Detail::DifferenceOfScalars(cx, ax);
    const Detail::Expansion acy = Detail::DifferenceOfScalars(cy, ay);
    const Detail::Expansion left = Detail::Multiply(abx, acy);
    const Detail::Expansion right = Detail::Multiply(aby, acx);
    return Detail::Sign(Detail::Add(left, Detail::Negate(right)));
}

int Orient3d(Linear::Point3 a, Linear::Point3 b, Linear::Point3 c, Linear::Point3 d) noexcept {
    const double abx = b.X - a.X;
    const double aby = b.Y - a.Y;
    const double abz = b.Z - a.Z;
    const double acx = c.X - a.X;
    const double acy = c.Y - a.Y;
    const double acz = c.Z - a.Z;
    const double adx = d.X - a.X;
    const double ady = d.Y - a.Y;
    const double adz = d.Z - a.Z;
    const double mx1 = aby * acz;
    const double mx2 = abz * acy;
    const double my1 = abz * acx;
    const double my2 = abx * acz;
    const double mz1 = abx * acy;
    const double mz2 = aby * acx;
    const double det = adx * (mx1 - mx2) + ady * (my1 - my2) + adz * (mz1 - mz2);
    const double permanent = std::abs(adx) * (std::abs(mx1) + std::abs(mx2))
        + std::abs(ady) * (std::abs(my1) + std::abs(my2))
        + std::abs(adz) * (std::abs(mz1) + std::abs(mz2));
    if (std::isfinite(det) && std::isfinite(permanent)
        && std::abs(det) >= Detail::ORIENT3D_ERROR_BOUND * permanent) {
        return Detail::SignOf(det);
    }

    const double maxAbs = std::max({std::abs(a.X), std::abs(a.Y), std::abs(a.Z),
                                     std::abs(b.X), std::abs(b.Y), std::abs(b.Z),
                                     std::abs(c.X), std::abs(c.Y), std::abs(c.Z),
                                     std::abs(d.X), std::abs(d.Y), std::abs(d.Z)});
    const int shift = Detail::ScaleShift(maxAbs);
    const auto scaled = [shift](double value) { return std::ldexp(value, shift); };
    const Detail::Triple ab = Detail::Difference3(scaled(a.X), scaled(a.Y), scaled(a.Z),
                                                  scaled(b.X), scaled(b.Y), scaled(b.Z));
    const Detail::Triple ac = Detail::Difference3(scaled(a.X), scaled(a.Y), scaled(a.Z),
                                                  scaled(c.X), scaled(c.Y), scaled(c.Z));
    const Detail::Triple ad = Detail::Difference3(scaled(a.X), scaled(a.Y), scaled(a.Z),
                                                  scaled(d.X), scaled(d.Y), scaled(d.Z));
    const Detail::Expansion tx = Detail::CrossComponent(ab.Y, ab.Z, ac.Y, ac.Z);
    const Detail::Expansion ty = Detail::CrossComponent(ab.Z, ab.X, ac.Z, ac.X);
    const Detail::Expansion tz = Detail::CrossComponent(ab.X, ab.Y, ac.X, ac.Y);
    const Detail::Expansion detExpansion = Detail::Add(
        Detail::Add(Detail::Multiply(ad.X, tx), Detail::Multiply(ad.Y, ty)),
        Detail::Multiply(ad.Z, tz));
    return Detail::Sign(detExpansion);
}

int Incircle(Linear::Point2 a, Linear::Point2 b, Linear::Point2 c, Linear::Point2 d) noexcept {
    const double adx = a.X - d.X;
    const double ady = a.Y - d.Y;
    const double bdx = b.X - d.X;
    const double bdy = b.Y - d.Y;
    const double cdx = c.X - d.X;
    const double cdy = c.Y - d.Y;
    const double bdxcdy = bdx * cdy;
    const double cdxbdy = cdx * bdy;
    const double cdxady = cdx * ady;
    const double adxcdy = adx * cdy;
    const double adxbdy = adx * bdy;
    const double bdxady = bdx * ady;
    const double alift = adx * adx + ady * ady;
    const double blift = bdx * bdx + bdy * bdy;
    const double clift = cdx * cdx + cdy * cdy;
    const double det = alift * (bdxcdy - cdxbdy) + blift * (cdxady - adxcdy)
        + clift * (adxbdy - bdxady);
    const double permanent = (std::abs(bdxcdy) + std::abs(cdxbdy)) * alift
        + (std::abs(cdxady) + std::abs(adxcdy)) * blift
        + (std::abs(adxbdy) + std::abs(bdxady)) * clift;
    if (std::isfinite(det) && std::isfinite(permanent)
        && std::abs(det) >= Detail::INCIRCLE_ERROR_BOUND * permanent) {
        return Detail::SignOf(det);
    }

    const double maxAbs = std::max({std::abs(a.X), std::abs(a.Y), std::abs(b.X), std::abs(b.Y),
                                     std::abs(c.X), std::abs(c.Y), std::abs(d.X), std::abs(d.Y)});
    const int shift = Detail::ScaleShift(maxAbs);
    const auto scaled = [shift](double value) { return std::ldexp(value, shift); };
    const Detail::Expansion ax = Detail::DifferenceOfScalars(scaled(a.X), scaled(d.X));
    const Detail::Expansion ay = Detail::DifferenceOfScalars(scaled(a.Y), scaled(d.Y));
    const Detail::Expansion bx = Detail::DifferenceOfScalars(scaled(b.X), scaled(d.X));
    const Detail::Expansion by = Detail::DifferenceOfScalars(scaled(b.Y), scaled(d.Y));
    const Detail::Expansion cx = Detail::DifferenceOfScalars(scaled(c.X), scaled(d.X));
    const Detail::Expansion cy = Detail::DifferenceOfScalars(scaled(c.Y), scaled(d.Y));
    const auto lift = [](const Detail::Expansion& x, const Detail::Expansion& y) {
        return Detail::Add(Detail::Multiply(x, x), Detail::Multiply(y, y));
    };
    const auto cross = [](const Detail::Expansion& ux, const Detail::Expansion& uy,
                           const Detail::Expansion& vx, const Detail::Expansion& vy) {
        return Detail::Add(Detail::Multiply(ux, vy), Detail::Negate(Detail::Multiply(uy, vx)));
    };
    const Detail::Expansion detExpansion = Detail::Add(
        Detail::Add(Detail::Multiply(lift(ax, ay), cross(bx, by, cx, cy)),
                    Detail::Multiply(lift(bx, by), cross(cx, cy, ax, ay))),
        Detail::Multiply(lift(cx, cy), cross(ax, ay, bx, by)));
    return Detail::Sign(detExpansion);
}

int Insphere(Linear::Point3 a, Linear::Point3 b, Linear::Point3 c, Linear::Point3 d,
             Linear::Point3 e) noexcept {
    const auto sub = [](Linear::Point3 p, Linear::Point3 origin) {
        return std::array<double, 3>{p.X - origin.X, p.Y - origin.Y, p.Z - origin.Z};
    };
    const auto liftOf = [](const std::array<double, 3>& v) {
        return v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
    };
    const auto crossOf = [](const std::array<double, 3>& u, const std::array<double, 3>& v) {
        return std::array<double, 3>{u[1] * v[2] - u[2] * v[1],
                                      u[2] * v[0] - u[0] * v[2],
                                      u[0] * v[1] - u[1] * v[0]};
    };
    const auto dotOf = [](const std::array<double, 3>& u, const std::array<double, 3>& v) {
        return u[0] * v[0] + u[1] * v[1] + u[2] * v[2];
    };
    const std::array<double, 3> av = sub(a, e);
    const std::array<double, 3> bv = sub(b, e);
    const std::array<double, 3> cv = sub(c, e);
    const std::array<double, 3> dv = sub(d, e);
    const double alift = liftOf(av);
    const double blift = liftOf(bv);
    const double clift = liftOf(cv);
    const double dlift = liftOf(dv);
    const double bcd = dotOf(bv, crossOf(cv, dv));
    const double acd = dotOf(av, crossOf(cv, dv));
    const double abd = dotOf(av, crossOf(bv, dv));
    const double abc = dotOf(av, crossOf(bv, cv));
    const double det = alift * bcd - blift * acd + clift * abd - dlift * abc;
    // 永久量取混合积相减之前的各项，再乘 lift。用已经相消后的 |lift * triple|
    // 会在内部相消时把误差界缩得过小，过滤可能把舍入误差认证成符号。
    const auto minorPermanent = [](const std::array<double, 3>& u, const std::array<double, 3>& v,
                                    const std::array<double, 3>& w) {
        const double cx1 = v[1] * w[2];
        const double cx2 = v[2] * w[1];
        const double cy1 = v[2] * w[0];
        const double cy2 = v[0] * w[2];
        const double cz1 = v[0] * w[1];
        const double cz2 = v[1] * w[0];
        return std::abs(u[0]) * (std::abs(cx1) + std::abs(cx2))
            + std::abs(u[1]) * (std::abs(cy1) + std::abs(cy2))
            + std::abs(u[2]) * (std::abs(cz1) + std::abs(cz2));
    };
    const double permanent = std::abs(alift) * minorPermanent(bv, cv, dv)
        + std::abs(blift) * minorPermanent(av, cv, dv)
        + std::abs(clift) * minorPermanent(av, bv, dv)
        + std::abs(dlift) * minorPermanent(av, bv, cv);
    if (std::isfinite(det) && std::isfinite(permanent)
        && std::abs(det) >= Detail::INSPHERE_ERROR_BOUND * permanent) {
        return Detail::SignOf(det);
    }

    const double maxAbs = std::max({std::abs(a.X), std::abs(a.Y), std::abs(a.Z),
                                     std::abs(b.X), std::abs(b.Y), std::abs(b.Z),
                                     std::abs(c.X), std::abs(c.Y), std::abs(c.Z),
                                     std::abs(d.X), std::abs(d.Y), std::abs(d.Z),
                                     std::abs(e.X), std::abs(e.Y), std::abs(e.Z)});
    const int shift = Detail::ScaleShift(maxAbs);
    const auto coordinate = [shift](double value) { return std::ldexp(value, shift); };
    const auto vectorFrom = [&](Linear::Point3 p) {
        Detail::Triple result{Detail::DifferenceOfScalars(coordinate(p.X), coordinate(e.X)),
                              Detail::DifferenceOfScalars(coordinate(p.Y), coordinate(e.Y)),
                              Detail::DifferenceOfScalars(coordinate(p.Z), coordinate(e.Z))};
        return result;
    };
    const Detail::Triple ea = vectorFrom(a);
    const Detail::Triple eb = vectorFrom(b);
    const Detail::Triple ec = vectorFrom(c);
    const Detail::Triple ed = vectorFrom(d);
    const auto lift = [](const Detail::Triple& v) {
        return Detail::Add(Detail::Add(Detail::Multiply(v.X, v.X), Detail::Multiply(v.Y, v.Y)),
                           Detail::Multiply(v.Z, v.Z));
    };
    const auto triple = [](const Detail::Triple& u, const Detail::Triple& v, const Detail::Triple& w) {
        const Detail::Expansion cx = Detail::CrossComponent(v.Y, v.Z, w.Y, w.Z);
        const Detail::Expansion cy = Detail::CrossComponent(v.Z, v.X, w.Z, w.X);
        const Detail::Expansion cz = Detail::CrossComponent(v.X, v.Y, w.X, w.Y);
        return Detail::Add(Detail::Add(Detail::Multiply(u.X, cx), Detail::Multiply(u.Y, cy)),
                           Detail::Multiply(u.Z, cz));
    };
    const Detail::Expansion detExpansion = Detail::Add(
        Detail::Add(Detail::Multiply(lift(ea), triple(eb, ec, ed)),
                    Detail::Negate(Detail::Multiply(lift(eb), triple(ea, ec, ed)))),
        Detail::Add(Detail::Multiply(lift(ec), triple(ea, eb, ed)),
                    Detail::Negate(Detail::Multiply(lift(ed), triple(ea, eb, ec)))));
    return Detail::Sign(detExpansion);
}

} // namespace DragonGeo::Predicates
