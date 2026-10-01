# DragonGeo 阶段 4 第 1 步 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 落地 `Segment`、`Ray`、`Line`、`Plane`、`Triangle` 和 `Polyline3` 的类型壳、它们做得到的曲线协议、直线族转换，以及需求合同第 6 节里只依赖这些类型和已有 `Box` / `OrientedBox` 的求交与距离。

**Architecture:** 无虚基类。一种类型一个头文件，方法同名。波 1 只放聚合字段、`IsValid` 和访问器；波 2 再加协议与 `Query`。解析查询 header-only，结果类型放在 `Query`。用到 `Orient2d` / `Orient3d` 的函数只对 `double` 提供。`float` 别名只存数据。重复的参数夹紧放在 `DragonGeo::Detail` 函数模板里，调用点仍是具体类型。

**Tech Stack:** C++20 · CMake ≥ 3.20 · 预设 `windows-vs-debug` · Catch2 v3.7.1 · header-only 目标 `DragonGeo::DragonGeo`

**Spec:** `docs/superpowers/specs/2026-09-30-dragongeo-stage4-design.md`（第 4、5.1、6.1、6.2 节）与 `docs/superpowers/specs/2026-10-01-dragongeo-stage4-implementation-design.md`（第 1 步）

**Out of scope:** 圆、椭圆、矩形、折线、多边形、NURBS、体元、GJK/EPA、`ThrowNotImplemented`。那些是后续计划。

## Global Constraints

- C++20。CMake ≥ 3.20。零运行时依赖。命名沿用总览：文件名与公开方法大驼峰，非公开数据成员 `m_` + 小驼峰，注释中文。
- 命名空间 `DragonGeo::Prim` 与 `DragonGeo::Query`。目录 `include/DragonGeo/Prim/` 与 `include/DragonGeo/Query/`。
- 聚合类型公开字段，不设工厂。`NaN` 允许存在。`IsValid` 要求坐标有限；射线、直线、平面的方向或法向必须是有限单位向量（长度平方与 1 的差用精确比较 `== 1` 不要求，单位性由 `UnitVector` 类型保证，`IsValid` 只拒绝非有限分量）。
- 相等是表示相等，逐字段精确比较。
- 线段参数 `t` 在 `[0, 1]`，`PointAt(t) = (1 - t) * A + t * B`。射线与直线是 `Origin + t * Direction`，`t` 是有符号距离。射线要求 `t ≥ 0`。
- 零长度线段、零面积三角形允许存在，构造时不拒绝。
- 查询碰到非有限输入时不抛异常，返回值不作规定，测试不断言那种输入。
- `Intersects` 按闭集回答。交点坐标是普通 `double` 运算。共线重叠、共面重叠用结果种类表示，不编造一个点。
- 布尔相交、点在三角形内走 `Orient2d` / `Orient3d`，不接收容差。这些函数不是模板，参数用 `double` 别名。
- 曲线协议里本步做得到的方法都要出现。做不到的不声明：直线没有 `StartPoint` 的值时返回空，而不是省略方法。无界曲线没有有限包围盒时 `Bounds` 返回规范空盒。`Length` 对射线和直线是 `+inf`。
- `Clone` 按值返回同类型副本。
- 绕点旋转调用 `Transform2::RotationAbout` / `Transform3::RotationAbout`。
- 点到 `Box` 的距离已经在 Linear 里，本步不改 `Box`。
- 测试文件放进 `tests/Prim/` 或 `tests/Query/`，CMake 用 `GLOB_RECURSE`，不必改 `tests/CMakeLists.txt`。
- 每个新公开方法都有直接调用它的用例。跑测试：`cmake --build D:/personal/Geometry/build/windows-vs --config Debug --target DragonGeoTests`，再跑 `D:/personal/Geometry/build/windows-vs/tests/Debug/DragonGeoTests.exe "<用例名>"`。
- 曲线协议的空值条件对所有曲线相同，见实现方案。本步没有椭圆和 NURBS。`Subcurve` 的成功类型仍按需求合同的表。`Length()` 一律返回 `Scalar`。
- 用户已授权执行本计划，每个任务的 Commit 步骤要执行。

## Review Focus

- 零长度线段 `AsRay` / `AsLine` 必须为空，`IsValid` 仍为真，`Length` 为 0。
- `Line2` 没有起点和中点：`StartPoint`、`EndPoint`、`MidPoint` 都是空。
- 线段与线段只共享端点时 `Kind` 是 `Point`；正长度重叠是 `Overlap`，重叠段方向沿第一个对象。
- 线段与平面共面且有重叠时 `Intersects` 为真，`Intersection` 为空。
- `Triangle3::Contains` 在不共面时为假；共面且在边界上为真。
- `Triangle3::Subcurve` 跨过顶点时的类型是 `Polyline3`，不是二维 `Polyline`。

---

### Task 1: Segment2 类型壳

**Files:**
- Create: `include/DragonGeo/Prim/Segment2.hpp`
- Create: `include/DragonGeo/Prim/Prim.hpp`
- Test: `tests/Prim/Segment2Test.cpp`

**Interfaces:**
- Consumes: `DragonGeo::Linear::Point2`、`Point2T<float>`。
- Produces: `DragonGeo::Prim::Segment2T<Scalar>`，字段 `Point2T<Scalar> A`、`B`。别名 `Segment2`、`Segment2f`。`[[nodiscard]] constexpr bool IsValid() const noexcept`：两个点的分量都有限时为真。`Prim.hpp` 只包含 `Segment2.hpp`。

- [ ] **Step 1: 写会失败的测试**

```cpp
#include <catch2/catch_test_macros.hpp>

#include <limits>

#include <DragonGeo/Prim/Segment2.hpp>

using DragonGeo::Prim::Segment2;
using DragonGeo::Prim::Segment2f;
using DragonGeo::Prim::Segment2T;
using DragonGeo::Linear::Point2;

TEST_CASE("Segment2 is an aggregate of two points", "[prim][segment2]") {
    Segment2 segment{Point2{0.0, 1.0}, Point2{2.0, 3.0}};
    CHECK(segment.A == Point2{0.0, 1.0});
    CHECK(segment.B == Point2{2.0, 3.0});
    CHECK(segment.IsValid());
    CHECK(segment == Segment2{Point2{0.0, 1.0}, Point2{2.0, 3.0}});
    CHECK(segment != Segment2{Point2{0.0, 1.0}, Point2{2.0, 4.0}});
    STATIC_REQUIRE(std::is_aggregate_v<Segment2>);
    STATIC_REQUIRE(std::is_same_v<Segment2f, Segment2T<float>>);
}

TEST_CASE("a zero-length Segment2 is valid", "[prim][segment2]") {
    const Segment2 segment{Point2{1.0, 1.0}, Point2{1.0, 1.0}};
    CHECK(segment.IsValid());
}

TEST_CASE("Segment2 with a non-finite coordinate is invalid", "[prim][segment2]") {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const Segment2 segment{Point2{nan, 0.0}, Point2{1.0, 0.0}};
    CHECK_FALSE(segment.IsValid());
}
```

- [ ] **Step 2: 运行测试，确认失败**

Run: `cmake --build D:/personal/Geometry/build/windows-vs --config Debug --target DragonGeoTests`

Expected: 编译失败，找不到 `Segment2.hpp`。

- [ ] **Step 3: 实现类型壳**

`Segment2.hpp` 包含 `Point2.hpp` 与 `Core/Numeric.hpp`。`IsValid` 对 `A.X`、`A.Y`、`B.X`、`B.Y` 调用 `Core::IsFinite`。`operator==` 默认生成。`Prim.hpp` 包含该头。

- [ ] **Step 4: 运行测试，确认通过**

Run: `D:/personal/Geometry/build/windows-vs/tests/Debug/DragonGeoTests.exe "Segment2"`

Expected: 3 个用例通过。

- [ ] **Step 5: Commit**

```bash
git add include/DragonGeo/Prim/Segment2.hpp include/DragonGeo/Prim/Prim.hpp tests/Prim/Segment2Test.cpp
git commit -m "feat(prim): add the Segment2 aggregate"
```

---

### Task 2: Segment3、Ray、Line、Plane、Triangle、Polyline3 的类型壳

**Files:**
- Create: `include/DragonGeo/Prim/Segment3.hpp`
- Create: `include/DragonGeo/Prim/Ray2.hpp`
- Create: `include/DragonGeo/Prim/Ray3.hpp`
- Create: `include/DragonGeo/Prim/Line2.hpp`
- Create: `include/DragonGeo/Prim/Line3.hpp`
- Create: `include/DragonGeo/Prim/Plane.hpp`
- Create: `include/DragonGeo/Prim/Triangle2.hpp`
- Create: `include/DragonGeo/Prim/Triangle3.hpp`
- Create: `include/DragonGeo/Prim/Polyline3.hpp`
- Modify: `include/DragonGeo/Prim/Prim.hpp`
- Test: `tests/Prim/Segment3Test.cpp`、`Ray2Test.cpp`、`Ray3Test.cpp`、`Line2Test.cpp`、`Line3Test.cpp`、`PlaneTest.cpp`、`Triangle2Test.cpp`、`Triangle3Test.cpp`、`Polyline3Test.cpp`

**Interfaces:**
- Consumes: Task 1 的头文件布局。`UnitVector2`、`UnitVector3`、`Point2`、`Point3`。
- Produces:
  - `Segment3T<Scalar>` 字段 `A`、`B`，别名 `Segment3`、`Segment3f`，`IsValid` 同 Segment2 的三维版本。
  - `Ray2T` / `Line2T` 字段 `Point2T<Scalar> Origin`、`UnitVector2T<Scalar> Direction`。别名 `Ray2`、`Ray2f`、`Line2`、`Line2f`。
  - `Ray3T` / `Line3T` 同样，点与单位向量是三维。别名 `Ray3`、`Ray3f`、`Line3`、`Line3f`。
  - `PlaneT` 字段 `Point3T<Scalar> Origin`、`UnitVector3T<Scalar> Normal`。别名 `Plane`、`Planef`。
  - `Triangle2T` 字段 `A`、`B`、`C`。别名 `Triangle2`、`Triangle2f`。
  - `Triangle3T` 字段 `A`、`B`、`C`。别名 `Triangle3`、`Triangle3f`。
  - `Polyline3T<Scalar>::FromSegments(std::span<const Segment3T<Scalar>>)` 返回 `std::optional<Polyline3T>`。至少一段，且 `segments[i].B == segments[i + 1].A`，否则空。方法 `SegmentCount`、`Segment(index)`、`PointCount`、`Point(index)`、`IsValid`。连续指端点精确相等。别名 `Polyline3`、`Polyline3f`。
  - 全部是聚合，除 `Polyline3`（私有段序列，工厂构造）。`IsValid`：点与单位向量的分量有限则为真。零面积三角形有效。

- [ ] **Step 1: 为每个类型写直接调用字段和 `IsValid` 的测试**

每个测试文件至少三条：`float` 别名与聚合性、正常值字段相等、含 `NaN` 时 `IsValid` 为假。`Polyline3` 另测：空 span 失败、端点不重合失败、两段相接成功且 `Point(0)` 是首段 `A`、`Point(2)` 是末段 `B`。零面积 `Triangle2{ {0,0}, {1,0}, {2,0} }` 的 `IsValid` 为真。

- [ ] **Step 2: 运行测试，确认编译失败**

Run: `cmake --build D:/personal/Geometry/build/windows-vs --config Debug --target DragonGeoTests`

Expected: 缺少对应头文件。

- [ ] **Step 3: 按上面的字段实现九个头，并让 `Prim.hpp` 按文件名包含它们**

`Polyline3` 的连续性用 `operator==` 比较点，不容差。

- [ ] **Step 4: 运行这些用例，确认通过**

Run: `D:/personal/Geometry/build/windows-vs/tests/Debug/DragonGeoTests.exe "prim"`

Expected: Task 1 与本任务的用例都通过。

- [ ] **Step 5: Commit**

```bash
git add include/DragonGeo/Prim tests/Prim
git commit -m "feat(prim): add the step-1 primitive shells"
```

---

### Task 3: 线段、射线、直线的参数与距离

**Files:**
- Create: `include/DragonGeo/Detail/CurveParameter.hpp`
- Modify: `include/DragonGeo/Prim/Segment2.hpp`
- Modify: `include/DragonGeo/Prim/Segment3.hpp`
- Modify: `include/DragonGeo/Prim/Ray2.hpp`
- Modify: `include/DragonGeo/Prim/Ray3.hpp`
- Modify: `include/DragonGeo/Prim/Line2.hpp`
- Modify: `include/DragonGeo/Prim/Line3.hpp`
- Test: 在对应的 `tests/Prim/*Test.cpp` 末尾追加用例

**Interfaces:**
- Consumes: Task 2 的类型。`Interval`、`Box2`、`Box3`、`UnitVector`。
- Produces: 每个直线族类型上的
  - `Domain() const noexcept` → `IntervalT<Scalar>`。线段 `[0,1]`。射线 `[0,+inf)`。直线是 `Interval::Empty()` 的无界形式：下端 `-inf`、上端 `+inf`。若 `Interval` 没有这个工厂，在 `CurveParameter.hpp` 里写 `Unbounded()` 返回 `{ -inf, +inf }`，不改 `Interval` 的空集语义。
  - `PointAt(Scalar t) const noexcept` → `std::optional<Point>`。`t` 不在 `Domain` 内时为空。直线接受任何有限 `t`。
  - `ClosestPoint(Point) const noexcept`、`DistanceSquared(Point) const noexcept`、`Distance(Point) const noexcept`。线段把参数夹在 `[0,1]`，射线夹在 `t ≥ 0`，直线不夹。
  - 仅线段：`Length() const noexcept`、`LengthSquared() const noexcept`、`Direction() const noexcept` → `std::optional<UnitVector>`，`A == B` 时为空。
  - `Bounds() const noexcept`。线段是 `Box::FromCorners(A, B)`。射线与直线返回 `Box::Empty()`。
  - `StartPoint`、`EndPoint`、`MidPoint`、`StartTangent`、`EndTangent`、`MidTangent` → `std::optional`。线段都有值；中点是 `PointAt(0.5)`。射线只有起点和起点切向，方向就是 `Direction`。直线六个都为空。
  - `TangentAt(Scalar t)`：参数在域内且方向非零时返回该单位方向；零长度线段为空。
  - `IsClosed()` 恒为 false。`Area()`、`Orientation()`、`Centroid()`、`Contains()` 恒为空或 false。
  - `ContainsPoint(Point, ToleranceT<Scalar> = {})`：点到对象的距离不超过 `tolerance.Resolve(尺度)`。尺度是线段长度；长度为零时用 `1`。射线和直线的尺度用 `1`。
  - `ParameterOf(Point, ToleranceT<Scalar> = {})`：先找最近点，距离超过 `ContainsPoint` 的同一容差时为空。否则返回该参数。线段上多个参数只可能在零长度时发生，返回 `0`。
  - `Translated(Vector)`、`Rotated(...)`、`Mirrored(point, unitNormal)`、`Reversed()`、`Clone()` 返回同类型。`Rotated` 二维是 `(Point center, Scalar radians)`，三维是 `(Point origin, UnitVector axis, Scalar radians)`，内部调用 `RotationAbout`。`Mirrored` 调用已有 `Transform::Reflection`。
  - `Transformed(const Transform&)` → `std::optional<同类型>`。方向变成零向量或结果非有限时为空。射线和直线变换后要重新单位化方向；单位化失败则为空。
  - `Subcurve(Interval)`：区间必须落在 `Domain` 内且长度大于 0，否则空。线段、射线、直线的有限子区间都返回 `std::optional<Segment>`。
  - `Detail::ClampParameter(t, domain)` 与 `Detail::ProjectParameter(origin, direction, point)` 只被这些方法调用。

- [ ] **Step 1: 写合同点名的直接用例**

`Segment2`：`PointAt(0)` 是 `A`，`PointAt(1)` 是 `B`，`PointAt(0.5)` 是中点，`PointAt(2)` 为空。`A=(0,0)`、`B=(3,0)` 时 `ClosestPoint((1,4))` 是 `(1,0)`，`DistanceSquared` 是 `16`。`A==B` 时 `Direction()` 为空，`Length()` 是 `0`，`IsValid()` 仍为真。`Reversed()` 交换 `A` 与 `B`。

`Line2`：`StartPoint()`、`MidPoint()` 为空。`ClosestPoint` 对 `Origin=(0,0)`、`Direction=+X`、点 `(1,2)` 得到 `(1,0)`。`Bounds()` 等于 `Box2::Empty()`。`Length()` 是 `+inf`。

`Ray2`：`PointAt(-1)` 为空。方向是 `-X`、查询点在原点右侧时，`ClosestPoint` 是原点。

每个新方法再各有一条直接调用：`Translated` 把两端点平移 `(1,0)`；`Mirrored` 关于过原点、法向 `+Y` 的直线把 `(0,1)` 变成 `(0,-1)`；`Rotated` 绕原点转 `HALF_PI` 后 `(1,0)` 落到 `(0,1)`；`Transformed(Identity)` 等于原对象；`ContainsPoint` 对线段上的中点为真、对偏离 1 的点在默认容差下为假；`ParameterOf` 对起点返回 `0`；`Clone` 等于原对象；`Subcurve` 取 `[0,0.5]` 得到半段，取 `[0,2]` 为空；`IsClosed` 为假；`Area` 与 `Centroid` 为空；`Contains` 为假。

三维各加一条：`Segment3` 的中点、`Line3` 没有起点、`Ray3` 拒绝负参数。

`Triangle2::Subcurve` 跨顶点需要 `Polyline`，而 `Polyline` 属于第 3 步。本计划只实现落在同一条边上的返回值。跨顶点的重载留在第 3 步计划，不在本步声明一个会抛异常的版本。

- [ ] **Step 2: 运行新用例，确认编译失败**

Expected: `PointAt` 等方法不存在。

- [ ] **Step 3: 实现 `CurveParameter.hpp` 和六个类型上的方法**

投影参数是 `((point - origin) · direction)`。线段夹紧到 `[0,1]`，射线夹紧到 `[0,+inf)`，直线不夹。`Distance` 用 `std::sqrt`。`Transformed` 对点用 `TransformPoint`，对方向用 `operator*(Vector)` 再 `Normalized`。

- [ ] **Step 4: 运行 `"Segment2"`、`"Line2"`、`"Ray2"`、`"Segment3"`、`"Line3"`、`"Ray3"`**

Expected: 通过。

- [ ] **Step 5: Commit**

```bash
git add include/DragonGeo/Detail/CurveParameter.hpp include/DragonGeo/Prim/Segment2.hpp include/DragonGeo/Prim/Segment3.hpp include/DragonGeo/Prim/Ray2.hpp include/DragonGeo/Prim/Ray3.hpp include/DragonGeo/Prim/Line2.hpp include/DragonGeo/Prim/Line3.hpp tests/Prim
git commit -m "feat(prim): add linear curve protocol and point distances"
```

---

### Task 4: 直线族转换

**Files:**
- Modify: Task 3 的六个头
- Test: 追加到对应测试文件

**Interfaces:**
- Consumes: Task 3 的类型。
- Produces:
  - `Segment::AsRay()`、`Segment::AsLine()` → `std::optional<Ray/Line>`。起点是 `A`，方向从 `A` 指向 `B`。`A == B` 时为空。
  - `Ray::AsLine()` 原点和方向原样带走，返回 `Line`，不包 optional。
  - `Line::AsRay()` 用这条直线自己的原点和方向，返回 `Ray`。
  - `Ray::AsSegment(Scalar length)`、`Line::AsSegment(Scalar parameterStart, Scalar parameterEnd)` → `std::optional<Segment>`。`length` 有限且 `> 0`。直线的两个参数有限且不相等，参数先后决定方向。

- [ ] **Step 1: 写用例**

零长度 `Segment2` 的 `AsRay` 与 `AsLine` 为空。`Segment2{(0,0),(3,0)}` 的 `AsRay` 方向是 `+X`，原点是 `(0,0)`。`Ray2::AsSegment(2)` 的终点是 `Origin + 2 * Direction`。`length == 0` 为空。`Line2::AsSegment(1, -1)` 的 `A` 是参数 1 的点，`B` 是参数 -1 的点。`AsSegment(1, 1)` 为空。

- [ ] **Step 2: 运行，确认失败**

Expected: `AsRay` 未声明。

- [ ] **Step 3: 实现转换**

方向用 `(B - A).Normalized()`。失败即空。不增加「指定另一个原点再变成射线」的重载。

- [ ] **Step 4: 运行 `"AsRay"`、`"AsSegment"`**

Expected: 通过。

- [ ] **Step 5: Commit**

```bash
git add include/DragonGeo/Prim tests/Prim
git commit -m "feat(prim): convert among segments, rays, and lines"
```

---

### Task 5: Plane 与 Triangle 的协议

**Files:**
- Modify: `include/DragonGeo/Prim/Plane.hpp`
- Modify: `include/DragonGeo/Prim/Triangle2.hpp`
- Modify: `include/DragonGeo/Prim/Triangle3.hpp`
- Modify: `include/DragonGeo/Prim/Polyline3.hpp`
- Test: `tests/Prim/PlaneTest.cpp`、`Triangle2Test.cpp`、`Triangle3Test.cpp`、`Polyline3Test.cpp`

**Interfaces:**
- Consumes: `Orient2d`、`Orient3d`、Task 2 的 `Polyline3`。
- Produces:
  - `Plane::SignedDistance(Point3)`、`Distance`、`ClosestPoint`、`Flipped()`。`Flipped` 法向取反，原点不变，与原平面 `!=`。
  - `Triangle2::SignedArea()` 逆时针为正。`Contains(Point2)` 仅 `double`，边界算内部。零面积时只包含精确落在退化边上的点。
  - `Triangle3::SignedArea()` 是叉积长度的一半，带符号由顶点顺序决定，零面积为 0。`Contains(Point3)` 仅 `double`：`Orient3d(A,B,C,point)==0` 且点在三条边围成的区域内（把三角形投到叉积绝对值最大的坐标平面后调用与 `Triangle2::Contains` 相同的二维判断）。
  - 三角形实现曲线协议里闭合多边形做得到的部分：`Domain` 是 `[0,3]`，参数中点在边界上。`Subcurve` 落在同一条边上返回 `Segment`，跨过顶点返回折线。二维折线类型本步还不存在，所以 `Triangle2::Subcurve` 在跨顶点时先不实现；只实现落在同一条边上的 `std::optional<Segment2>`。跨顶点的 `Polyline` 留到第 3 步计划，本步在 `Triangle2` 的注释里写明这个缺口。`Triangle3::Subcurve` 跨顶点时返回 `std::optional<std::variant<Segment3, Polyline3>>`。
  - `Polyline3` 补上第 4.1 节里支撑 `Subcurve` 所需的 `Domain`、`PointAt`、`IsValid`、`Bounds`、`Clone`。其余协议方法留到调用方真正需要的后续计划，本步不声明。

- [ ] **Step 1: 写用例**

`Plane` 过原点、法向 `+Z`：`(0,0,2)` 的 `SignedDistance` 是 `2`，`Distance` 是 `2`，`ClosestPoint` 是 `(0,0,0)`。`Flipped()` 的法向是 `-Z`，且不等于原平面。

`Triangle2{(0,0),(1,0),(0,1)}` 的 `SignedArea` 是 `0.5`。包含 `(0,0)` 与内部点 `(0.2,0.2)`，不包含 `(1,1)`。零面积三角形包含边上的点，不包含外侧点。

`Triangle3` 同一三角形放在 `z=0`：共面内点包含，`(0.2,0.2,1)` 不包含。`Subcurve` 取只覆盖第一条边的区间时是 `Segment3`；从第一条边中点到第二条边中点的区间是 `Polyline3`。

- [ ] **Step 2: 运行，确认失败**

Expected: `SignedDistance` 或 `Contains` 未声明。

- [ ] **Step 3: 实现**

`Triangle2::Contains` 用三次 `Orient2d`，符号全部 `≥ 0` 或全部 `≤ 0` 才算内部。零面积时改成「点到三条边之一的参数落在 `[0,1]` 且 `Orient2d` 为 0」。

- [ ] **Step 4: 运行 `"Plane"`、`"Triangle2"`、`"Triangle3"`**

Expected: 通过。

- [ ] **Step 5: Commit**

```bash
git add include/DragonGeo/Prim/Plane.hpp include/DragonGeo/Prim/Triangle2.hpp include/DragonGeo/Prim/Triangle3.hpp include/DragonGeo/Prim/Polyline3.hpp tests/Prim
git commit -m "feat(prim): add plane and triangle queries on the types"
```

---

### Task 6: 直线族求交

**Files:**
- Create: `include/DragonGeo/Query/CurveMeet.hpp`
- Create: `include/DragonGeo/Query/Intersection.hpp`
- Create: `include/DragonGeo/Query/Query.hpp`
- Test: `tests/Query/LinearIntersectionTest.cpp`

**Interfaces:**
- Consumes: `Segment2`、`Line2`、`Ray2`、`Segment3`、`Line3`、`Ray3`、`Plane`、`Triangle3`、`Box2`、`Box3`、`OrientedBox2`、`OrientedBox3`、`Orient2d`、`Orient3d`。
- Produces: 需求合同第 6 节的 `CurveMeet`、`CurveMeet2`、`CurveMeet3`、`ParameterPoint2`、`ParameterPoint3`、`ParameterInterval2`、`ParameterInterval3`。函数都是 `noexcept`：
  - `Intersects(Line2, Line2)`、`Intersection(Line2, Line2) -> CurveMeet2`
  - `Intersects(Segment2, Segment2)`、`Intersection(Segment2, Segment2) -> CurveMeet2`
  - `Intersects(Line2, Segment2)`、`Intersection(Line2, Segment2) -> CurveMeet2`
  - `Intersects(Ray3, Plane)`、`Intersects(Segment3, Plane)`、`Intersection(...) -> std::optional<ParameterPoint3>`
  - `Intersects(Line3, Plane)`、`Intersection(Line3, Plane) -> CurveMeet3`
  - `Intersects(Ray3, Triangle3)`、`Intersects(Segment3, Triangle3)`、`Intersects(Line3, Triangle3)`，`Intersection` 返回 `std::optional<ParameterPoint3>`
  - `Ray2` / `Segment2` / `Line2` 与 `Box2`，以及三维对 `Box3`，`Intersection` 返回 `std::optional<ParameterInterval>`
  - `Ray2` / `Segment2` 与 `OrientedBox2`，`Ray3` / `Segment3` 与 `OrientedBox3`。不对直线提供 OrientedBox 相交。
  - `Query::Distance` 与 `Query::DistanceSquared`：`Segment2×Segment2`、`Segment3×Segment3`、`Triangle3×Triangle3`。

- [ ] **Step 1: 写锁定种类的用例**

两条 `Line2` 十字相交：`Kind == Point`，交点是原点。平行分离：`Kind == None`。同一条直线：`Kind == Coincident`。

两条 `Segment2` 只在端点相接：`Kind == Point`。`[(0,0),(2,0)]` 与 `[(1,0),(3,0)]`：`Kind == Overlap`，重叠段是 `[(1,0),(2,0)]`。

`Segment3` 躺在 `Plane` 上：`Intersects` 为真，`Intersection` 为空。射线从 `z=-1` 指向 `+Z`、平面 `z=0`：交点参数是 `1`。

`Segment2` 穿过 `Box2{ (0,0), (1,1) }`：`Enter < Exit`，两端点在盒边界上。空盒不相交。

`Query::DistanceSquared` 对两条垂直且不相交的 `Segment3`（`(0,0,0)-(1,0,0)` 与 `(0,1,1)-(0,1,2)`）等于 `2`。

再补三条组合：`Line2` 穿过 `Box2{(0,0),(1,1)}` 时 `Enter < Exit`；`Ray3` 从盒外指向 `OrientedBox3` 的一个面时 `Intersection` 有值；`Line3` 与 `Triangle3` 共面重叠时 `Intersects` 为真且 `Intersection` 为空。

- [ ] **Step 2: 运行，确认失败**

Expected: `Query/Intersection.hpp` 不存在。

- [ ] **Step 3: 实现**

二维线段相交先用 `Orient2d` 判断端点分侧。异号才有严格内部交点，再用参数插值求点。共线时把第二段端点投到第一段参数上，求 `[0,1]` 的重叠区间；区间长度为零是 `Point`，大于零是 `Overlap`。

射线与轴对齐盒用 slab 法。`OrientedBox` 先把射线变到盒的局部坐标，再对以原点为中心、半轴为 `HalfExtent` 的轴对齐盒做 slab。

三角形相交：射线与三角形所在平面求点，再用 `Triangle3::Contains`。共面时 `Intersects` 看线段是否碰到三角形（端点包含或与三条边相交），`Intersection` 返回空。

`Triangle3` 距离：若 `Intersects` 为真则距离平方为 0；否则取两组「点到对方三角形」和「边到对方三条边」的最小值。

- [ ] **Step 4: 运行 `"LinearIntersection"` 与全量 `RUN_TESTS`**

Expected: 新用例通过，原有 343 项仍然通过。

- [ ] **Step 5: Commit**

```bash
git add include/DragonGeo/Query tests/Query
git commit -m "feat(query): intersect and measure the linear primitives"
```

---

### Task 7: 接到总头

**Files:**
- Modify: `include/DragonGeo/DragonGeo.hpp`
- Test: `tests/SmokeTest.cpp` 追加一条包含检查，若该文件已有总头用例则只加 `Prim.hpp` 与 `Query.hpp` 能被总头间接包含的断言

**Interfaces:**
- Consumes: `Prim.hpp`、`Query.hpp`。
- Produces: `DragonGeo.hpp` 包含这两份汇总头。

- [ ] **Step 1: 在 SmokeTest 里直接包含 `DragonGeo.hpp` 并声明一个 `Segment2` 与一次 `Intersection(Line2, Line2)`**

若现有用例已经包含总头，把这两次使用加进去，保证新头被实例化。

- [ ] **Step 2: 运行 SmokeTest，确认在修改总头之前失败或未覆盖**

若总头尚未包含新头，编译失败即为预期。

- [ ] **Step 3: 在 `DragonGeo.hpp` 末尾包含 `Prim/Prim.hpp` 与 `Query/Query.hpp`**

- [ ] **Step 4: 跑全量测试**

Run: `cmake --build D:/personal/Geometry/build/windows-vs --config Debug --target RUN_TESTS`

Expected: 全部通过。

- [ ] **Step 5: Commit**

```bash
git add include/DragonGeo/DragonGeo.hpp tests/SmokeTest.cpp
git commit -m "feat: export step-1 primitives from the umbrella header"
```
