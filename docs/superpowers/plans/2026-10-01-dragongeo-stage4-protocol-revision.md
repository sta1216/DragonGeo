# 阶段 4 曲线协议修订 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 把已经落地的第 1 步改成修订后的曲线协议：变换就地修改，折线存点并补齐曲线方法，平面能处理点和矢量，实现放进静态库。

**Architecture:** 头文件保留声明。`double` 与 `float` 在 `src/Prim`、`src/Query` 显式实例化。`Core` 与 `Linear` 仍是头文件。无虚基类。

**Tech Stack:** C++20 · CMake ≥ 3.20 · 预设 `windows-vs` · Catch2 v3 · 静态库 `DragonGeo`

**Spec:** `docs/superpowers/specs/2026-10-01-dragongeo-stage4-curve-protocol-revision.md` 与 `docs/superpowers/specs/2026-09-30-dragongeo-stage4-design.md` 第 4.1、5.1、5.3 节。

## Global Constraints

- 曲线方法：`Translate`、`Rotate`、`Mirror`、`Reverse` 返回 `void` 并改自己。`Transform` 返回 `bool`，失败时对象不变。`Clone` 仍按值返回副本。`Box()` 返回 `Box2` 或 `Box3`。
- `DragonGeo::Prim::Winding` 为 `CounterClockwise`、`Clockwise`、`Degenerate`。`Orientation()` 返回 `std::optional<Winding>`。不闭合则为空。
- 查询仍是 `double`、`noexcept`。用到 `Orient2d` / `Orient3d` 的方法只对 `double` 提供。
- 每个新公开方法或改过语义的公开方法都有直接调用它的测试。
- 构建：`cmake --build D:/personal/Geometry/build/windows-vs --config Debug --target DragonGeoTests`，再跑 `D:/personal/Geometry/build/windows-vs/tests/Debug/DragonGeoTests.exe "[prim]"`，最后 `RUN_TESTS`。
- 用户已授权本计划的每个任务提交。

## Review Focus

- `Transform` 失败后字段与调用前相同。
- 射线 `Reverse` 保持原点，只翻转方向。
- 三角形 `Reverse` 得到 `{A, C, B}`，接缝仍是 `A`。
- 折线不足两个点、或含非有限点时 `FromPoints` 为空。
- 只包含 `Segment2.hpp` 的翻译单元仍能实例化 `AsRay` / `AsLine`。

### Task 1: 就地变换与 Winding

**Files:**
- Create: `include/DragonGeo/Prim/Winding.hpp`
- Modify: `Segment2.hpp`、`Segment3.hpp`、`Ray2.hpp`、`Ray3.hpp`、`Line2.hpp`、`Line3.hpp`、`Triangle2.hpp`、`Triangle3.hpp`、`Polyline3.hpp`（只把 `Bounds` 改名为 `Box`）
- Test: `tests/Prim/` 里对应测试

**Interfaces:**
- Produces: `void Translate(Vector)`、`void Rotate(...)`、`void Mirror(point, unitNormal)`、`void Reverse()`、`bool Transform(const Transform&)`、`Box()`、`optional<Winding> Orientation()`。参数表与现有 `Translated` / `Rotated` / `Mirrored` 相同。

- [ ] **Step 1:** 增加 `Winding.hpp`，把上述方法改成就地修改。`Transform` 沿用现在的失败条件，失败时不写字段。
- [ ] **Step 2:** 改测试。拷贝后再调用；`Transform` 用 `CHECK` 返回值，并断言失败时对象仍等于调用前。`Orientation()` 与 `Winding` 比较。
- [ ] **Step 3:** 跑 `[prim]` 与 `RUN_TESTS`。
- [ ] **Step 4:** Commit `refactor(prim): mutate curve transforms in place`

### Task 2: 折线改存点并补齐曲线协议

**Files:**
- Modify: `include/DragonGeo/Prim/Polyline3.hpp`
- Test: `tests/Prim/Polyline3Test.cpp`
- 调用方: `Triangle3::Subcurve` 若直接构造 `Polyline3`，改为 `FromPoints`

**Interfaces:**
- Produces: `FromPoints(span<const Point3>)`。至少两个有限点，否则空。`Segment(index)` 由相邻点现拼。`Point(0) == Point(PointCount - 1)` 时闭合。实现第 4.1 节，方法名与 Task 1 相同。

- [ ] **Step 1:** 用点列重写存储，删掉 `FromSegments`。
- [ ] **Step 2:** 为每个新公开方法写直接调用。闭合三点折线 `A,B,C,A` 的 `Orientation` 与对应三角形一致，`Area` 不把重复末点再算一次。
- [ ] **Step 3:** 跑 `[prim]` 与 `RUN_TESTS`。
- [ ] **Step 4:** Commit `feat(prim): store polylines as points`

### Task 3: 平面上的点与矢量

**Files:**
- Modify: `include/DragonGeo/Prim/Plane.hpp`
- Test: `tests/Prim/PlaneTest.cpp`

**Interfaces:**
- Produces: `Project(Vector)`、`Project(UnitVector)`（平行法线时为空）、`Mirror(Point)`、`Mirror(Vector)`、`Contains(Point, tolerance = {})`、`void Offset(Scalar)`。`Flipped` 仍返回新平面。

- [ ] **Step 1:** 实现并直接调用。法线 `+Z`、原点在原点：`(0,0,2)` 的 `Contains` 在默认容差下为假；`Offset(2)` 后原点是 `(0,0,2)`。`Project` 把 `(1,2,3)` 投成 `(1,2,0)`。
- [ ] **Step 2:** 跑 `[prim]` 与 `RUN_TESTS`。
- [ ] **Step 3:** Commit `feat(prim): project and mirror through a plane`

### Task 4: 定义移入静态库

**Files:**
- Create: `src/Prim/*.cpp`、`src/Query/Intersection.cpp`
- Modify: 对应头文件改为声明。`src/CMakeLists.txt` 列入这些源文件。
- 头文件保留聚合字段和 `IsValid`。其余方法定义进 cpp，并对 `double` 与 `float` 显式实例化。`Query` 只有 `double`。
- `AsRay` / `AsLine` 的定义放进 `Segment2.cpp` / `Segment3.cpp`，去掉为打破包含环而加的宏。只包含线段头的测试仍然链接得过。

- [ ] **Step 1:** 移定义，去掉这些方法上的 `constexpr`。把测试里依赖 `constexpr` 求值的 `STATIC_REQUIRE` 改成运行期 `CHECK`。`noexcept` 的 `STATIC_REQUIRE` 保留。
- [ ] **Step 2:** 跑 `RUN_TESTS`。
- [ ] **Step 3:** Commit `refactor(prim): compile stage-4 methods into the static library`
