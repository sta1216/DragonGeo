# Reflection Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 给 `Transform2` / `Transform3` 加上过一点、沿单位法向的反射，以及过原点的坐标轴 / 坐标平面便捷工厂。

**Architecture:** 工厂是头文件里的 `constexpr` 静态函数。线性部分写 `I − 2 n nᵀ`，平移列写 `2 (n · p) n`。便捷工厂调用一般工厂。`Coordinate` 的实现不改，测试确认它拒绝反射。

**Tech Stack:** C++20，头文件 `Linear`，Catch2，MSVC 2022，CMake 预设 `windows-vs-debug`。

## Global Constraints

- 命名空间 `DragonGeo`，工厂名 PascalCase，参数小驼峰。缩写只允许 `BSpline` 与 `BVH`，工厂名用 `Reflection`。
- 头文件注释用中文。
- `Linear` 保持头文件，不新增 `.cpp`。
- 新工厂 `constexpr`、`noexcept`，不返回 `optional`，不接收容差。
- 法向的单位长度是前置条件，工厂不检查。
- 点走完整仿射，方向只走线性部分。
- 每个新工厂都有测试。double 别名覆盖行为，float 不单列一套。
- 构建：`cmake --build --preset windows-vs-debug --target DragonGeoTests`，再跑 `build\windows-vs\tests\Debug\DragonGeoTests.exe` 的对应标签。
- 不改 `Coordinate` 的接受条件，不加 `TransformNormal`，不加中心对称工厂，不给 `Box` / `OrientedBox` 加镜像成员。

**规格：** `docs/superpowers/specs/2026-09-30-dragongeo-reflection-design.md`

---

### Task 1: Transform2 反射

**Files:**
- Modify: `include/DragonGeo/Linear/Transform2.hpp`
- Test: `tests/Linear/Transform2Test.cpp`

**Interfaces:**
- Consumes: `Point2T`、`UnitVector2T`、`Vector2T`、`MatrixT<Scalar, 3>::Identity`
- Produces:
  - `static constexpr Transform2T Reflection(Point2T<Scalar> point, UnitVector2T<Scalar> normal) noexcept`
  - `static constexpr Transform2T ReflectionX() noexcept`
  - `static constexpr Transform2T ReflectionY() noexcept`

- [x] **Step 1: Write the failing test**

在 `tests/Linear/Transform2Test.cpp` 增加 `Coordinate2.hpp`、`UnitVector2.hpp` 的包含与 `using`，并追加规格里的用例：`ReflectionX` / `ReflectionY` 的坐标动作、与 `Scaling` 及一般工厂相等、`t * t == Identity()`、`constexpr`、法向变号、不通过原点的镜面、同一直线上两点、方向不受镜面平移影响、非轴对齐法向 `(1,1)` 把 `(1,0)` 送到 `(0,-1)`、`Inverse()` 与反射本身把点送到同一处、`Coordinate2::FromTransform(ReflectionX())` 为 `nullopt`。

- [x] **Step 2: Run test to verify it fails**

Run: `cmake --build --preset windows-vs-debug --target DragonGeoTests`

Expected: 编译失败，`Transform2` 没有 `Reflection`。

- [x] **Step 3: Write minimal implementation**

`Transform2.hpp` 包含 `UnitVector2.hpp`。在 `Rotation` 之后加入三个工厂。`ReflectionX` 的法向是 +Y，`ReflectionY` 的法向是 +X。一般工厂从单位矩阵出发，写入

- `Data[0][0] = 1 - 2 nx nx`，`Data[1][1] = 1 - 2 ny ny`
- `Data[0][1] = Data[1][0] = -2 nx ny`
- `Data[0][2] = 2 (n · point) nx`，`Data[1][2] = 2 (n · point) ny`

- [x] **Step 4: Run test to verify it passes**

Run: `build\windows-vs\tests\Debug\DragonGeoTests.exe "[transform2]"`

Expected: 全部通过。

### Task 2: Transform3 反射

**Files:**
- Modify: `include/DragonGeo/Linear/Transform3.hpp`
- Test: `tests/Linear/Transform3Test.cpp`

**Interfaces:**
- Consumes: Task 1 的同一公式，升到三维
- Produces:
  - `static constexpr Transform3T Reflection(Point3T<Scalar> point, UnitVector3T<Scalar> normal) noexcept`
  - `static constexpr Transform3T ReflectionYZ() noexcept`（法向 +X）
  - `static constexpr Transform3T ReflectionZX() noexcept`（法向 +Y）
  - `static constexpr Transform3T ReflectionXY() noexcept`（法向 +Z）

- [x] **Step 1: Write the failing test**

在 `tests/Linear/Transform3Test.cpp` 包含 `Coordinate3.hpp`，追加：三个便捷工厂的坐标动作、与 `Scaling` 及一般工厂相等、`t * t == Identity()`、`constexpr`、法向变号、平面 `z = 2` 上的点不动且 `(1,4,5)` 落到 `(1,4,-1)`、同一平面两点相等、方向与过原点的反射一致、法向 `(1,1,0)` 把 `(1,0,5)` 送到 `(0,-1,5)`、`Inverse()`、`Coordinate3::FromTransform(ReflectionXY())` 为 `nullopt`。

- [x] **Step 2: Run test to verify it fails**

Run: `cmake --build --preset windows-vs-debug --target DragonGeoTests`

Expected: 编译失败，`Transform3` 没有 `Reflection`。

- [x] **Step 3: Write minimal implementation**

在 `Rotation` 之后加入四个工厂。平移写入第 4 列 `Data[i][3]`。最后一行保持单位矩阵的 `(0,0,0,1)`。

- [x] **Step 4: Run test to verify it passes**

Run: `build\windows-vs\tests\Debug\DragonGeoTests.exe "[transform2]"` 与 `"[transform3]"`

Expected: 全部通过。
