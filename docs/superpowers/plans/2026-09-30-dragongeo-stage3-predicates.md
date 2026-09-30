> **2026-09-30 更正：** 实现完成后，谓词改为编译进静态库，公共头只保留四个声明。下文里「保持 header-only、不改 `INTERFACE`」的句子是当时的计划，不再执行。

# DragonGeo 阶段 3：精确谓词 实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在 `DragonGeo::Predicates` 提供 `Orient2d`、`Orient3d`、`Incircle`、`Insphere` 四个精确谓词，过滤路径放在头文件里，不确定时用无误差变换展开给出 `-1` / `0` / `+1`。

**Architecture:** 库保持 header-only，不把 `DragonGeo` 改成要链接的静态库。公共头 `include/DragonGeo/Predicates/Predicates.hpp` 只放四个函数。展开运算放在 `include/DragonGeo/Predicates/Detail/Expansion.hpp`，命名空间 `DragonGeo::Predicates::Detail`，不是稳定接口。每个谓词先算浮点行列式；绝对值不小于 Shewchuk 1997 的 `errboundA` 乘永久量时直接返回符号，否则把坐标按同一个 2 的整数次幂缩放后再做精确展开。展开按分量绝对值从小到大排列，符号取最高位非零分量。

**Tech Stack:** C++20 · CMake ≥ 3.20 · 既有预设 `windows-vs-debug` · Catch2 v3.7.1 · header-only 目标 `DragonGeo::DragonGeo`

**Spec:** `docs/superpowers/specs/2026-09-29-dragongeo-design.md`（§3.3、§4.1、§4.5、§7.2、§8 阶段 3、§9 决策 8 与 18）

## Global Constraints

- C++20 必需。CMake ≥ 3.20。零运行时依赖。
- 文件名、类名、方法名、自由函数名：大驼峰。参数与局部变量：小驼峰。public 数据成员：大驼峰。非 public 数据成员：`m_` + 小驼峰。常量：全大写下划线分词。宏：`DRAGONGEO_` 前缀。注释用中文。
- 模块目录与命名空间大驼峰：`DragonGeo::Predicates`，目录 `include/DragonGeo/Predicates/`。
- 四个函数的签名以 spec §4.5 为准，返回 `-1` / `0` / `+1`，`noexcept`，不接受 `Tolerance`。
- 符号约定以 spec §4.5 为准：`Orient2d` 逆时针为 `+1`；`Orient3d` 的正侧由 `(b - a) × (c - a)` 决定；`Incircle` 在边界逆时针时圆内为 `+1`；`Insphere` 在 `Orient3d(a, b, c, d)` 为正时球内为 `+1`。
- 坐标必须有限。`NaN` 与无穷的返回值不作规定，测试不断言它们的符号。
- 不移植第三方谓词源码。误差界只用 Shewchuk 1997 的 `errboundA` 那一组常数。
- 阶段 3 不引入编译单元，不修改 `add_library(DragonGeo INTERFACE)`。
- 文中的 Commit 步骤只在用户明确要求提交时执行。计划本身不授权提交。

## Review Focus

- 点落在对角线上再抬高一个 ulp 时，浮点过滤会弃权，精确路径仍必须给出 `+1`。对应 `Orient2d` 的 nextafter 用例。
- 共线、共面、共圆、共球、点重合必须返回 `0`，不能把“很接近”收成非零。每个谓词任务里都有一条恰好退化的用例。
- `Incircle` 的圆内符号跟着边界三角形的方向走：同一内点，边界改成顺时针后结果必须变成 `-1`。
- `Insphere` 同样：前四个点负定向时，球心的结果必须是 `-1`。
- 坐标大到 `2^50` 且只偏离对角线一个 ulp 时，缩放后的精确路径必须仍是 `+1`，不能在中间积里把这个符号吃掉。

---

### Task 1: 无误差变换展开

**Files:**
- Create: `include/DragonGeo/Predicates/Detail/Expansion.hpp`
- Test: `tests/Predicates/ExpansionTest.cpp`

**Interfaces:**
- Consumes: 无。只依赖 `<cmath>`、`<vector>`、`<cstddef>`。
- Produces: `DragonGeo::Predicates::Detail` 中的 `Two`、`TwoSum`、`TwoProduct`、`DifferenceOfScalars`、`Add`、`Multiply`、`Negate`、`Sign`、`ScaleShift`、`ORIENT2D_ERROR_BOUND`、`ORIENT3D_ERROR_BOUND`、`INCIRCLE_ERROR_BOUND`、`INSPHERE_ERROR_BOUND`。`Expansion` 是 `std::vector<double>`，分量从小到大。`Sign` 取最高位非零分量的符号。

- [ ] **Step 1: 写会失败的展开测试**

```cpp
#include <catch2/catch_test_macros.hpp>

#include <cmath>

#include <DragonGeo/Predicates/Detail/Expansion.hpp>

using DragonGeo::Predicates::Detail::ScaleShift;
using DragonGeo::Predicates::Detail::Sign;
using DragonGeo::Predicates::Detail::TwoProduct;
using DragonGeo::Predicates::Detail::TwoSum;

TEST_CASE("TwoSum keeps the roundoff of a half-ulp add", "[predicates][expansion]") {
    const auto sum = TwoSum(1.0, std::ldexp(1.0, -53));
    CHECK(sum.Hi == 1.0);
    CHECK(sum.Lo == std::ldexp(1.0, -53));
}

TEST_CASE("TwoProduct keeps the square of one plus one ulp", "[predicates][expansion]") {
    const double u = 1.0 + std::ldexp(1.0, -52);
    const auto product = TwoProduct(u, u);
    CHECK(product.Hi == 1.0 + std::ldexp(1.0, -51));
    CHECK(product.Lo == std::ldexp(1.0, -104));
}

TEST_CASE("Sign reads the leading non-zero component", "[predicates][expansion]") {
    CHECK(Sign({0.0, -2.0}) == -1);
    CHECK(Sign({3.0, 0.0}) == 1);
    CHECK(Sign({0.0, 0.0}) == 0);
    CHECK(Sign({}) == 0);
}

TEST_CASE("ScaleShift is the power of two that brings the max into [1, 2)",
          "[predicates][expansion]") {
    CHECK(ScaleShift(0.0) == 0);
    CHECK(ScaleShift(1.0) == 0);
    CHECK(ScaleShift(3.0) == -1);
    CHECK(std::ldexp(3.0, ScaleShift(3.0)) == 1.5);
}
```

- [ ] **Step 2: 跑测试，确认失败**

Run: `cmake --build --preset windows-vs-debug --target DragonGeoTests`

Expected: 编译失败，找不到 `DragonGeo/Predicates/Detail/Expansion.hpp`。

- [ ] **Step 3: 写展开头**

`include/DragonGeo/Predicates/Detail/Expansion.hpp` 的全文：

```cpp
#pragma once

#include <cmath>
#include <cstddef>
#include <vector>

namespace DragonGeo::Predicates::Detail {

/// 本头不是稳定接口。调用方只使用 Predicates.hpp 里的四个函数。

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

[[nodiscard]] inline Two TwoSum(double a, double b) noexcept {
    const double x = a + b;
    const double bVirtual = x - a;
    const double aVirtual = x - bVirtual;
    return {x, (a - aVirtual) + (b - bVirtual)};
}

[[nodiscard]] inline Two Split(double value) noexcept {
    const double c = SPLITTER * value;
    const double aBig = c - value;
    const double hi = c - aBig;
    return {hi, value - hi};
}

[[nodiscard]] inline Two TwoProduct(double a, double b) noexcept {
    const double x = a * b;
    const Two aParts = Split(a);
    const Two bParts = Split(b);
    const double y = ((aParts.Hi * bParts.Hi - x) + aParts.Hi * bParts.Lo + aParts.Lo * bParts.Hi)
        + aParts.Lo * bParts.Lo;
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
    const Two factorParts = Split(factor);
    const auto productOf = [&](double component) {
        const double x = component * factor;
        const Two parts = Split(component);
        const double y = ((parts.Hi * factorParts.Hi - x) + parts.Hi * factorParts.Lo
            + parts.Lo * factorParts.Hi) + parts.Lo * factorParts.Lo;
        return Two{x, y};
    };
    const Two first = productOf(expansion[0]);
    Expansion result;
    if (first.Lo != 0.0) {
        result.push_back(first.Lo);
    }
    double q = first.Hi;
    for (std::size_t i = 1; i < expansion.size(); ++i) {
        const Two product = productOf(expansion[i]);
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

} // namespace DragonGeo::Predicates::Detail
```

- [ ] **Step 4: 跑测试，确认通过**

Run: `cmake --build --preset windows-vs-debug --target DragonGeoTests`，然后 `ctest --preset windows-vs-debug -R expansion`

Expected: 新增的 expansion 用例通过。

- [ ] **Step 5: Commit**

仅当用户要求提交时：

```bash
git add include/DragonGeo/Predicates/Detail/Expansion.hpp tests/Predicates/ExpansionTest.cpp
git commit -m "feat(predicates): add error-free expansion arithmetic"
```

---

### Task 2: Orient2d

**Files:**
- Create: `include/DragonGeo/Predicates/Predicates.hpp`
- Test: `tests/Predicates/Orient2dTest.cpp`

**Interfaces:**
- Consumes: Task 1 的 `DifferenceOfScalars`、`Multiply`、`Add`、`Negate`、`Sign`、`SignOf`、`ScaleShift`、`ORIENT2D_ERROR_BOUND`。`DragonGeo::Linear::Point2`。
- Produces: `int DragonGeo::Predicates::Orient2d(Point2 a, Point2 b, Point2 c) noexcept`。

- [ ] **Step 1: 写会失败的 Orient2d 测试**

```cpp
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdint>
#include <limits>

#include <DragonGeo/Predicates/Predicates.hpp>

using DragonGeo::Linear::Point2;
using DragonGeo::Predicates::Orient2d;

namespace {

int Orient2dInt(long long ax, long long ay, long long bx, long long by,
                 long long cx, long long cy) {
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

TEST_CASE("Orient2d is zero when the points are collinear or coincident",
          "[predicates][orient2d]") {
    CHECK(Orient2d(Point2{0.0, 0.0}, Point2{1.0, 1.0}, Point2{2.0, 2.0}) == 0);
    CHECK(Orient2d(Point2{0.0, 0.0}, Point2{1.0, 1.0}, Point2{0.5, 0.5}) == 0);
    CHECK(Orient2d(Point2{3.0, 4.0}, Point2{3.0, 4.0}, Point2{9.0, 1.0}) == 0);
    CHECK(Orient2d(Point2{-3.0, 4.0}, Point2{0.0, 0.0}, Point2{3.0, -4.0}) == 0);
}

TEST_CASE("Orient2d of a one-ulp lift off the diagonal is positive",
          "[predicates][orient2d]") {
    // 过滤会弃权。(0,0)、(1,1) 与 (0.5, 0.5) 共线；把第三点的 Y 抬高一个 ulp
    // 之后，精确行列式为正。
    const double lifted = std::nextafter(0.5, std::numeric_limits<double>::infinity());
    const double lowered = std::nextafter(0.5, -std::numeric_limits<double>::infinity());
    CHECK(Orient2d(Point2{0.0, 0.0}, Point2{1.0, 1.0}, Point2{0.5, lifted}) == 1);
    CHECK(Orient2d(Point2{0.0, 0.0}, Point2{1.0, 1.0}, Point2{0.5, lowered}) == -1);
}

TEST_CASE("Orient2d keeps the sign of a one-ulp lift at magnitude 2^50",
          "[predicates][orient2d]") {
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

TEST_CASE("integer translation and positive power-of-two scaling keep Orient2d",
          "[predicates][orient2d]") {
    const Point2 a{1.0, 2.0};
    const Point2 b{4.0, 2.0};
    const Point2 c{4.0, 6.0};
    const int sign = Orient2d(a, b, c);
    CHECK(Orient2d(Point2{a.X + 3.0, a.Y - 5.0}, Point2{b.X + 3.0, b.Y - 5.0},
                   Point2{c.X + 3.0, c.Y - 5.0}) == sign);
    CHECK(Orient2d(Point2{a.X * 4.0, a.Y * 4.0}, Point2{b.X * 4.0, b.Y * 4.0},
                   Point2{c.X * 4.0, c.Y * 4.0}) == sign);
}

TEST_CASE("Orient2d matches the int64 determinant on small integer points",
          "[predicates][orient2d]") {
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
        CHECK(Orient2d(a, b, c) == Orient2dInt(coords[0], coords[1], coords[2], coords[3],
                                                coords[4], coords[5]));
    }
}

TEST_CASE("Orient2d is noexcept", "[predicates][orient2d]") {
    STATIC_REQUIRE(noexcept(Orient2d(Point2{}, Point2{}, Point2{})));
}
```

- [ ] **Step 2: 跑测试，确认失败**

Run: `cmake --build --preset windows-vs-debug --target DragonGeoTests`

Expected: 编译失败，找不到 `Orient2d`。

- [ ] **Step 3: 写 Orient2d**

`include/DragonGeo/Predicates/Predicates.hpp`：

```cpp
#pragma once

#include <algorithm>
#include <cmath>

#include <DragonGeo/Linear/Point2.hpp>
#include <DragonGeo/Predicates/Detail/Expansion.hpp>

namespace DragonGeo::Predicates {

/// 三点定向。逆时针为 +1，顺时针为 -1，共线（含点重合）为 0。
///
/// 零表示代数上确实共线，不是“误差在阈值内”。函数不接受容差。
/// 坐标必须是有限数。NaN 或无穷时仍不抛异常，返回值不作规定。
/// 各坐标的指数跨度大到使中间积下溢到次正规数时，返回值不作规定。
///
/// 精确路径可能分配内存。内存耗尽时按 noexcept 的规则终止。
[[nodiscard]] inline int Orient2d(Linear::Point2 a, Linear::Point2 b, Linear::Point2 c) noexcept {
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

} // namespace DragonGeo::Predicates
```

- [ ] **Step 4: 跑测试，确认通过**

Run: `cmake --build --preset windows-vs-debug --target DragonGeoTests`，然后 `ctest --preset windows-vs-debug -R orient2d`

Expected: orient2d 用例通过。若 one-ulp 用例得到 `0`，说明过滤在不确定时返回了浮点符号，精确路径没有接上。

- [ ] **Step 5: Commit**

仅当用户要求提交时：

```bash
git add include/DragonGeo/Predicates/Predicates.hpp tests/Predicates/Orient2dTest.cpp
git commit -m "feat(predicates): add exact Orient2d"
```

---

### Task 3: Orient3d

**Files:**
- Modify: `include/DragonGeo/Predicates/Predicates.hpp`
- Test: `tests/Predicates/Orient3dTest.cpp`

**Interfaces:**
- Consumes: Task 1 的展开运算，以及 `ORIENT3D_ERROR_BOUND`。`Point3`。
- Produces: `int Orient3d(Point3 a, Point3 b, Point3 c, Point3 d) noexcept`。正侧是 `(b - a) × (c - a)` 的方向。

- [ ] **Step 1: 写会失败的 Orient3d 测试**

```cpp
#include <catch2/catch_test_macros.hpp>

#include <cstdint>

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

TEST_CASE("Orient3d is noexcept", "[predicates][orient3d]") {
    STATIC_REQUIRE(noexcept(Orient3d(Point3{}, Point3{}, Point3{}, Point3{})));
}
```

- [ ] **Step 2: 跑测试，确认失败**

Run: `cmake --build --preset windows-vs-debug --target DragonGeoTests`

Expected: 编译失败，`Orient3d` 未声明。

- [ ] **Step 3: 把 Orient3d 加进 Predicates.hpp**

`Predicates.hpp` 增加 `#include <DragonGeo/Linear/Point3.hpp>`。下面三个辅助类型放进 `Expansion.hpp` 的 `Detail` 命名空间、闭合括号之前。过滤用与精确式相同的混合积，永久量是相减之前各项绝对值之和。精确式是 `ad · (ab × ac)`。

```cpp
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
```

`Orient3d` 加在 `Orient2d` 之后：

```cpp

/// 四点定向。d 在平面 abc 的正侧为 +1，正侧由 (b - a) × (c - a) 决定，共面为 0。
///
/// 输入约定与 Orient2d 相同。
[[nodiscard]] inline int Orient3d(Linear::Point3 a, Linear::Point3 b, Linear::Point3 c,
                                  Linear::Point3 d) noexcept {
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
```

- [ ] **Step 4: 跑测试，确认通过**

Run: `cmake --build --preset windows-vs-debug --target DragonGeoTests`，然后 `ctest --preset windows-vs-debug -R "orient2d|orient3d"`

Expected: 两组都通过。Orient2d 不能回退。

- [ ] **Step 5: Commit**

仅当用户要求提交时：

```bash
git add include/DragonGeo/Predicates/Predicates.hpp include/DragonGeo/Predicates/Detail/Expansion.hpp tests/Predicates/Orient3dTest.cpp
git commit -m "feat(predicates): add exact Orient3d"
```

---

### Task 4: Incircle

**Files:**
- Modify: `include/DragonGeo/Predicates/Predicates.hpp`
- Test: `tests/Predicates/IncircleTest.cpp`

**Interfaces:**
- Consumes: Task 1 的展开运算，`INCIRCLE_ERROR_BOUND`。
- Produces: `int Incircle(Point2 a, Point2 b, Point2 c, Point2 d) noexcept`。`a, b, c` 逆时针时，`d` 在圆内为 `+1`。

- [ ] **Step 1: 写会失败的 Incircle 测试**

```cpp
#include <catch2/catch_test_macros.hpp>

#include <cstdint>

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

TEST_CASE("Incircle is noexcept", "[predicates][incircle]") {
    STATIC_REQUIRE(noexcept(Incircle(Point2{}, Point2{}, Point2{}, Point2{})));
}
```

- [ ] **Step 2: 跑测试，确认失败**

Run: `cmake --build --preset windows-vs-debug --target DragonGeoTests`

Expected: 编译失败，`Incircle` 未声明。

- [ ] **Step 3: 把 Incircle 加进 Predicates.hpp**

相对 `d` 平移。过滤的永久量用相减之前的两个乘积的绝对值，再乘上对应的 lift，三项相加。这与 `INCIRCLE_ERROR_BOUND` 配套。精确路径用同一多项式的展开。

```cpp
/// a、b、c 逆时针时，d 在圆内为 +1，圆外为 -1，圆上为 0。边界顺时针时符号相反。
///
/// 输入约定与 Orient2d 相同。
[[nodiscard]] inline int Incircle(Linear::Point2 a, Linear::Point2 b, Linear::Point2 c,
                                  Linear::Point2 d) noexcept {
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
```

- [ ] **Step 4: 跑测试，确认通过**

Run: `cmake --build --preset windows-vs-debug --target DragonGeoTests`，然后 `ctest --preset windows-vs-debug -R "orient2d|orient3d|incircle"`

Expected: 三组都通过。顺时针边界那条必须是 `-1`。若整数对拍失败，先核对永久量是不是用了相减之后的绝对值；不要把误差界改大去掩盖错误符号。

- [ ] **Step 5: Commit**

仅当用户要求提交时：

```bash
git add include/DragonGeo/Predicates/Predicates.hpp tests/Predicates/IncircleTest.cpp
git commit -m "feat(predicates): add exact Incircle"
```

---

### Task 5: Insphere

**Files:**
- Modify: `include/DragonGeo/Predicates/Predicates.hpp`
- Test: `tests/Predicates/InsphereTest.cpp`

**Interfaces:**
- Consumes: Task 1 与 Task 3 放进 `Detail` 的 `CrossComponent`。`INSPHERE_ERROR_BOUND`。
- Produces: `int Insphere(Point3 a, Point3 b, Point3 c, Point3 d, Point3 e) noexcept`。`Orient3d(a, b, c, d) > 0` 时，`e` 在球内为 `+1`。

- [ ] **Step 1: 写会失败的 Insphere 测试**

```cpp
#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cstdint>

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
```

整数范围用 `±200`：lift 与混合积的乘积仍放得进 `long long`。把范围放大到 `±10000` 会让对拍自己溢出。

- [ ] **Step 2: 跑测试，确认失败**

Run: `cmake --build --preset windows-vs-debug --target DragonGeoTests`

Expected: 编译失败，`Insphere` 未声明。

- [ ] **Step 3: 把 Insphere 加进 Predicates.hpp**

相对 `e` 平移。行列式是

`alift * (B · (C × D)) - blift * (A · (C × D)) + clift * (A · (B × D)) - dlift * (A · (B × C))`。

这个符号已经对上“正定向四面体的球内为 +1”。不要再取反。过滤的永久量是这四项绝对值之和。

```cpp
/// Orient3d(a, b, c, d) 为正时，e 在球内为 +1，球外为 -1，球上为 0。
/// 前四个点反向时符号相反。
///
/// 输入约定与 Orient2d 相同。
[[nodiscard]] inline int Insphere(Linear::Point3 a, Linear::Point3 b, Linear::Point3 c,
                                  Linear::Point3 d, Linear::Point3 e) noexcept {
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
    const double permanent = std::abs(alift * bcd) + std::abs(blift * acd)
        + std::abs(clift * abd) + std::abs(dlift * abc);
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
```

`Triple` 与 `CrossComponent` 已经在 Task 3 放进 `Detail`。这里继续用它们。`Predicates.hpp` 需要 `#include <array>`。

- [ ] **Step 4: 跑测试，确认通过**

Run: `cmake --build --preset windows-vs-debug --target DragonGeoTests`，然后 `ctest --preset windows-vs-debug -R predicates`

Expected: 全部谓词用例通过。负定向四面体的球心必须是 `-1`。若它是 `+1`，精确式多取了一次反，不要去改测试期望。

- [ ] **Step 5: Commit**

仅当用户要求提交时：

```bash
git add include/DragonGeo/Predicates/Predicates.hpp tests/Predicates/InsphereTest.cpp
git commit -m "feat(predicates): add exact Insphere"
```

---

### Task 6: 接到总头，并更新仍写着“谓词尚未实现”的文档

**Files:**
- Modify: `include/DragonGeo/DragonGeo.hpp`
- Modify: `src/CMakeLists.txt` 顶部注释
- Modify: `README.md` 里 “Predicates layer is planned, not present” 那一句
- Modify: `docs/superpowers/specs/2026-09-29-dragongeo-design.md` §8 状态列

**Interfaces:**
- Consumes: `Predicates.hpp`。
- Produces: `#include <DragonGeo/DragonGeo.hpp>` 之后可以直接调用四个谓词。库仍是 `INTERFACE`。

- [ ] **Step 1: 写一条走总头的用例**

在 `tests/Predicates/Orient2dTest.cpp` 增加：

```cpp
#include <DragonGeo/DragonGeo.hpp>

TEST_CASE("the umbrella header exports Orient2d", "[predicates][orient2d]") {
    CHECK(DragonGeo::Predicates::Orient2d(DragonGeo::Linear::Point2{0.0, 0.0},
                                          DragonGeo::Linear::Point2{1.0, 0.0},
                                          DragonGeo::Linear::Point2{0.0, 1.0}) == 1);
}
```

这一条在改总头之前，如果测试文件原来只包含 `Predicates.hpp`，它会通过；所以同时在一个只包含总头、不包含 `Predicates.hpp` 的新用例文件里验证。把上面的包含改成只包含 `DragonGeo.hpp`，并且这个测试文件不要再包含 `Predicates.hpp`。现有 `Orient2dTest.cpp` 继续直接包含谓词头。新建 `tests/Predicates/UmbrellaTest.cpp`，内容就是上面这一条，包含只用 `DragonGeo.hpp`。

- [ ] **Step 2: 跑测试，确认失败**

Run: `cmake --build --preset windows-vs-debug --target DragonGeoTests`

Expected: `UmbrellaTest.cpp` 编译失败，`DragonGeo::Predicates` 不存在。

- [ ] **Step 3: 接上总头并改文档**

`include/DragonGeo/DragonGeo.hpp` 在最后一个 Linear 包含之后加上：

```cpp
#include <DragonGeo/Predicates/Predicates.hpp>
```

`src/CMakeLists.txt` 顶部注释改成：阶段 3 的谓词放在头文件里。`Polygon`、`Mesh`、`bvh` 引入编译单元时，再把 `add_library(DragonGeo INTERFACE)` 改为 `STATIC` 并在此追加源文件。

`README.md` 把 “the `Predicates` layer is planned, not present” 改成谓词已经提供，定向、内接圆、内接球返回精确符号。

spec §8：阶段 2 去掉「← 本阶段」，阶段 3 标成「← 本阶段」。阶段标题不要重写。

- [ ] **Step 4: 跑全部测试**

Run: `cmake --build --preset windows-vs-debug --target DragonGeoTests`，然后 `ctest --preset windows-vs-debug`

Expected: 全部通过，包含原有 Linear 用例。

- [ ] **Step 5: Commit**

仅当用户要求提交时：

```bash
git add include/DragonGeo/DragonGeo.hpp src/CMakeLists.txt README.md docs/superpowers/specs/2026-09-29-dragongeo-design.md tests/Predicates/UmbrellaTest.cpp
git commit -m "feat(predicates): export exact predicates from the umbrella header"
```
