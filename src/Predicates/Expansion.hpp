#pragma once

#include <cmath>
#include <cstddef>
#include <vector>

namespace DragonGeo::Predicates::Detail {

/// 本头不是稳定接口，也不随安装包发布。调用方只使用 Predicates.hpp 里的四个函数。
///
/// 无误差变换要求就近舍入，并且不能把乘法与相邻的加减合成一条乘加。
/// 中间结果经 volatile 写回 double，避免消费方打开浮点收缩时丢掉误差项。

inline constexpr double EPSILON = 0x1p-53;
inline constexpr double SPLITTER = 0x1p27 + 1.0;

inline constexpr double ORIENT2D_ERROR_BOUND = (3.0 + 16.0 * EPSILON) * EPSILON;
inline constexpr double ORIENT3D_ERROR_BOUND = (7.0 + 56.0 * EPSILON) * EPSILON;
inline constexpr double INCIRCLE_ERROR_BOUND = (10.0 + 96.0 * EPSILON) * EPSILON;
inline constexpr double INSPHERE_ERROR_BOUND = (16.0 + 224.0 * EPSILON) * EPSILON;

struct Two {
    double Hi;
    double Lo;
};

/// 把值写进 volatile double 再读出，强制按 IEEE double 舍入。
[[nodiscard]] inline double ForceRound(double value) noexcept {
    volatile double rounded = value;
    return rounded;
}

[[nodiscard]] inline Two TwoSum(double a, double b) noexcept {
    const double x = ForceRound(a + b);
    const double bVirtual = ForceRound(x - a);
    const double aVirtual = ForceRound(x - bVirtual);
    const double aError = ForceRound(a - aVirtual);
    const double bError = ForceRound(b - bVirtual);
    return {x, ForceRound(aError + bError)};
}

[[nodiscard]] inline Two Split(double value) noexcept {
    const double c = ForceRound(SPLITTER * value);
    const double aBig = ForceRound(c - value);
    const double hi = ForceRound(c - aBig);
    return {hi, ForceRound(value - hi)};
}

[[nodiscard]] inline Two TwoProduct(double a, double b) noexcept {
    const double x = ForceRound(a * b);
    const Two aParts = Split(a);
    const Two bParts = Split(b);
    const double hiHi = ForceRound(aParts.Hi * bParts.Hi);
    const double hiLo = ForceRound(aParts.Hi * bParts.Lo);
    const double loHi = ForceRound(aParts.Lo * bParts.Hi);
    const double loLo = ForceRound(aParts.Lo * bParts.Lo);
    double y = ForceRound(hiHi - x);
    y = ForceRound(y + hiLo);
    y = ForceRound(y + loHi);
    y = ForceRound(y + loLo);
    return {x, y};
}

using Expansion = std::vector<double>;

[[nodiscard]] inline int Sign(const Expansion& expansion) noexcept {
    for (auto it = expansion.rbegin(); it != expansion.rend(); ++it) {
        if (*it > 0.0) {
            return 1;
        }
        if (*it < 0.0) {
            return -1;
        }
    }
    return 0;
}

[[nodiscard]] inline int SignOf(double value) noexcept {
    if (value > 0.0) {
        return 1;
    }
    if (value < 0.0) {
        return -1;
    }
    return 0;
}

/// 把最大绝对值乘上 `2^返回值` 后落到 [1, 2)。零和非有限输入返回 0。
[[nodiscard]] inline int ScaleShift(double maxAbs) noexcept {
    if (!(maxAbs > 0.0) || !std::isfinite(maxAbs)) {
        return 0;
    }
    int exponent = 0;
    std::frexp(maxAbs, &exponent);
    return 1 - exponent;
}

[[nodiscard]] inline Expansion Grow(const Expansion& expansion, double value) {
    double q = value;
    Expansion result;
    result.reserve(expansion.size() + 1);
    for (const double component : expansion) {
        const Two sum = TwoSum(q, component);
        q = sum.Hi;
        if (sum.Lo != 0.0) {
            result.push_back(sum.Lo);
        }
    }
    if (q != 0.0 || result.empty()) {
        result.push_back(q);
    }
    return result;
}

[[nodiscard]] inline Expansion Add(const Expansion& left, const Expansion& right) {
    Expansion sum = left;
    for (const double component : right) {
        sum = Grow(sum, component);
    }
    return sum;
}

[[nodiscard]] inline Expansion ScaleBy(const Expansion& expansion, double factor) {
    if (expansion.empty()) {
        return {};
    }
    const auto productOf = [](double component, double scale) {
        return TwoProduct(component, scale);
    };
    const Two first = productOf(expansion[0], factor);
    Expansion result;
    if (first.Lo != 0.0) {
        result.push_back(first.Lo);
    }
    double q = first.Hi;
    for (std::size_t i = 1; i < expansion.size(); ++i) {
        const Two product = productOf(expansion[i], factor);
        const Two low = TwoSum(q, product.Lo);
        if (low.Lo != 0.0) {
            result.push_back(low.Lo);
        }
        const Two high = TwoSum(product.Hi, low.Hi);
        q = high.Hi;
        if (high.Lo != 0.0) {
            result.push_back(high.Lo);
        }
    }
    if (q != 0.0 || result.empty()) {
        result.push_back(q);
    }
    return result;
}

[[nodiscard]] inline Expansion Multiply(const Expansion& left, const Expansion& right) {
    Expansion product;
    for (const double component : right) {
        product = Add(product, ScaleBy(left, component));
    }
    return product;
}

[[nodiscard]] inline Expansion Negate(Expansion expansion) {
    for (double& component : expansion) {
        component = -component;
    }
    return expansion;
}

[[nodiscard]] inline Expansion DifferenceOfScalars(double left, double right) {
    const Two diff = TwoSum(left, -right);
    Expansion result;
    if (diff.Lo != 0.0) {
        result.push_back(diff.Lo);
    }
    if (diff.Hi != 0.0 || result.empty()) {
        result.push_back(diff.Hi);
    }
    return result;
}

struct Triple {
    Expansion X;
    Expansion Y;
    Expansion Z;
};

[[nodiscard]] inline Triple Difference3(double ax, double ay, double az,
                                        double bx, double by, double bz) {
    return {DifferenceOfScalars(bx, ax), DifferenceOfScalars(by, ay), DifferenceOfScalars(bz, az)};
}

[[nodiscard]] inline Expansion CrossComponent(const Expansion& uy, const Expansion& uz,
                                              const Expansion& vy, const Expansion& vz) {
    return Add(Multiply(uy, vz), Negate(Multiply(uz, vy)));
}

} // namespace DragonGeo::Predicates::Detail
