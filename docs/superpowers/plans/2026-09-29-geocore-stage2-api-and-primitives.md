# GeoCore 阶段 2：API 成员化与基础类型 实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 把 `linear` 层的具名运算从自由函数改为成员函数，并补齐 `Point2/3`、`Interval`、`Box2/3`、`OrientedBox2/3`、`Coordinate2/3` 五组基础类型。

**Architecture:** 全部落在 `linear` 层，不新增层。`Point` 进入 `linear` 后，`Transform` 得以直接提供 `transform_point(Point) -> Point`，取代原先把位置塞进 `Vector` 的 `apply(t, Vector)` 分工。运算符保持自由函数（二元运算的对称性），具名运算一律成员。

**Tech Stack:** C++20 · CMake ≥ 3.20 · Catch2 v3（FetchContent）· 既有 `GeoCore::GeoCore` target。

**Spec:** `docs/superpowers/specs/2026-09-29-geocore-design.md`（§3 分层、§5.0 linear 清单、§9 决策 15–17）

## Global Constraints

- **C++20 必需。**
- **命名：** 类型 PascalCase 优先完整拼写；函数与变量 `snake_case`；私有数据成员尾随下划线；常量 `snake_case`；宏 `GEOCORE_UPPER_SNAKE`；顶层 namespace `GeoCore`，子模块小写。
- **容差显式传参**，默认值来自 `core::Tolerance`，不得在函数体内硬编码阈值。
- **`VectorNT`/`PointNT` 分量是数据成员（聚合）；`UnitVectorNT` 分量是访问器方法。** 两者会在同一段代码里并存。
- **`MatrixT` 是单成员聚合**（`Scalar data[N][N]`），字面量需要**两层**花括号。
- **具名运算一律成员函数**（`dot`/`cross`/`norm`/`determinant`/`transposed`/`inverse`/`normalized`/`rotate`/`to_matrix`/`apply`），**运算符保持自由函数**（`+`/`-`/`*`/`/`/`==`）。这是 spec §9 决策 17 的分界，不得混淆。
- **静态工厂用类名限定**：`Transform3T<Scalar>::translation(...)`、`QuaternionT<Scalar>::from_axis_angle(...)`、`MatrixT<Scalar, N>::identity()`。
- **测试文件与被测头文件目录结构镜像**，Catch2 `TEST_CASE` + 标签。
- **Catch2 v3 的 `Approx` 需要两件事**：包含 `<catch2/catch_approx.hpp>` **并且** `using Catch::Approx;`。
- **本机为 Windows + Visual Studio 2022，未安装 Ninja。** 本地命令用 `windows-vs` 预设；CI 用 `ninja-debug`。

## Review Focus

以下五类是本阶段特有的失败模式，每条都在对应任务里配了测试。

1. **`Point` 与 `Vector` 的运算规则** —— `p + p` 必须**编译失败**，`p - p` 必须得到 `Vector`。这是类型系统的承诺，写错了不会报错、只会静默给出错误语义。（Task 4）
2. **`Coordinate` 的正交性** —— 非正交的"坐标系"会让变换静默地拉伸几何。必须无法通过公开接口构造出来。（Task 7）
3. **`Box` 与 `OrientedBox` 的空盒语义** —— 空盒（`min > max`）的 `contains`/`intersects`/`merged` 必须有定义，且 `OrientedBox::to_axis_aligned()` 对已旋转的盒子必须真的变大而不是返回原盒。（Task 6、Task 8）
4. **成员化后的语义零漂移** —— `v.dot(w)` 必须与旧 `dot(v, w)` **逐位相同**（含 NaN/inf 传播），不允许在搬运过程中改变求值顺序。（Task 1–3）
5. **下标访问的越界与常量性** —— `operator[]` 必须提供 const 与非 const 两个重载，越界行为必须有文档（不静默返回垃圾）。（Task 1）

---

## File Structure

| 文件 | 职责 |
|---|---|
| `include/GeoCore/linear/Vector2.hpp` / `Vector3.hpp` / `Vector4.hpp` | 成员化：`dot`/`cross`/`normalized` + `operator[]` + `to_array` |
| `include/GeoCore/linear/UnitVector2.hpp` / `UnitVector3.hpp` | 成员化：`dot`/`cross`；承接 `VectorNT::normalized` 的定义 |
| `include/GeoCore/linear/Matrix.hpp` | 成员化：`determinant`/`transposed`/`inverse` + 静态 `identity` |
| `include/GeoCore/linear/Quaternion.hpp` | 成员化：`norm`/`conjugate`/`normalized`/`rotate`/`to_matrix` + 静态 `from_axis_angle`/`identity` |
| `include/GeoCore/linear/Transform2.hpp` / `Transform3.hpp` | 成员化：静态工厂 + `apply` 入类 + `transform_point` |
| `include/GeoCore/linear/Point2.hpp` / `Point3.hpp` | 新建：位置类型 |
| `include/GeoCore/linear/Interval.hpp` | 新建：一维区间 |
| `include/GeoCore/linear/Box2.hpp` / `Box3.hpp` | 新建：AABB |
| `include/GeoCore/linear/Coordinate2.hpp` / `Coordinate3.hpp` | 新建：坐标系（强不变量） |
| `include/GeoCore/linear/OrientedBox2.hpp` / `OrientedBox3.hpp` | 新建：有向包围盒 |
| `tests/linear/*_test.cpp` | 每个头文件一个测试文件，另加 `point_test.cpp`、`interval_test.cpp` 等 |

---

### Task 1: `Vector` / `UnitVector` 成员化与下标访问

`dot`/`cross` 从自由函数改为成员；`Vector` 增加下标访问与数组导出。**`normalized()` 的声明在 `Vector` 类内、定义在 `UnitVector` 头文件** —— 因为 `Vector3T::normalized()` 返回 `std::optional<UnitVector3T>`，而 `UnitVector3.hpp` 已经包含 `Vector3.hpp`，反向包含会成环。

**Files:**
- Modify: `include/GeoCore/linear/Vector2.hpp` / `Vector3.hpp` / `Vector4.hpp`
- Modify: `include/GeoCore/linear/UnitVector2.hpp` / `UnitVector3.hpp`
- Modify: `tests/linear/vector2_test.cpp` / `vector3_test.cpp` / `vector4_test.cpp` / `unit_vector2_test.cpp` / `unit_vector3_test.cpp`

**Interfaces:**
- Consumes: `core::Tolerance`、`core::absolute_value`
- Produces:
  - `Scalar VectorNT::dot(VectorNT) const noexcept`
  - `Scalar Vector2T::cross(Vector2T) const noexcept`；`Vector3T Vector3T::cross(Vector3T) const noexcept`
  - `std::optional<UnitVectorNT> VectorNT::normalized(core::Tolerance = {}) const noexcept`
  - `Scalar& VectorNT::operator[](int) noexcept` / `const Scalar& operator[](int) const noexcept`
  - `std::array<Scalar, N> VectorNT::to_array() const noexcept`
  - `Scalar UnitVectorNT::dot(UnitVectorNT) const noexcept`
  - `Scalar UnitVector2T::cross(UnitVector2T) const noexcept` / `Vector3T UnitVector3T::cross(UnitVector3T) const noexcept`

- [ ] **Step 1: 写失败测试**

在 `tests/linear/vector3_test.cpp` 追加：

```cpp
TEST_CASE("Vector3 members: dot, cross, subscript and array export",
          "[linear][vector3]") {
    const Vector3 a{1.0, 2.0, 3.0};
    const Vector3 b{4.0, 5.0, 6.0};

    // 成员形式的 dot/cross
    CHECK(a.dot(b) == 32.0);
    CHECK(a.cross(b) == Vector3{-3.0, 6.0, -3.0});

    // 下标访问：索引 0/1/2 对应 x/y/z
    CHECK(a[0] == 1.0);
    CHECK(a[1] == 2.0);
    CHECK(a[2] == 3.0);

    // 非 const 下标可写
    Vector3 mutable_v{};
    mutable_v[0] = 7.0;
    mutable_v[1] = 8.0;
    mutable_v[2] = 9.0;
    CHECK(mutable_v == Vector3{7.0, 8.0, 9.0});

    // 数组导出
    const std::array<double, 3> arr = a.to_array();
    CHECK(arr[0] == 1.0);
    CHECK(arr[1] == 2.0);
    CHECK(arr[2] == 3.0);
}

TEST_CASE("Vector3 normalized is a member returning optional",
          "[linear][vector3][degenerate]") {
    const auto unit = Vector3{3.0, 4.0, 0.0}.normalized();

    REQUIRE(unit.has_value());
    CHECK(unit->x() == Approx(0.6));
    CHECK(unit->y() == Approx(0.8));

    // 零向量仍然拒绝，且与旧自由函数 normalize 语义一致
    CHECK_FALSE(Vector3{0.0, 0.0, 0.0}.normalized().has_value());
}
```

在 `tests/linear/vector2_test.cpp` 追加：

```cpp
TEST_CASE("Vector2 members: dot, cross and subscript", "[linear][vector2]") {
    const Vector2 a{1.0, 2.0};
    const Vector2 b{3.0, 5.0};

    CHECK(a.dot(b) == 13.0);          // 1*3 + 2*5
    CHECK(a.cross(b) == -1.0);        // 1*5 - 2*3
    CHECK(a[0] == 1.0);
    CHECK(a[1] == 2.0);
}
```

- [ ] **Step 2: 运行测试，确认失败**

```bash
cmake --build --preset windows-vs-debug
```

Expected: 编译失败 —— `dot` / `cross` / `operator[]` / `to_array` / `normalized` 不是成员。

- [ ] **Step 3: 实现**

在 `Vector2T` 内加入（`<array>` 与 `<optional>` 需加进 include 区）：

```cpp
    /// 下标访问。索引 0/1 依次对应 x/y。
    ///
    /// 越界是未定义行为 —— 与 std::array 一致，不做边界检查。
    [[nodiscard]] constexpr Scalar& operator[](int index) noexcept {
        return index == 0 ? x : y;
    }

    [[nodiscard]] constexpr const Scalar& operator[](int index) const noexcept {
        return index == 0 ? x : y;
    }

    /// 导出为数组，便于与外部库互操作。
    [[nodiscard]] constexpr std::array<Scalar, 2> to_array() const noexcept {
        return {x, y};
    }

    /// 点积。
    [[nodiscard]] constexpr Scalar dot(Vector2T other) const noexcept {
        return x * other.x + y * other.y;
    }

    /// 二维叉积，返回标量（有向面积的两倍再取半，即 z 分量）。
    /// 正值表示 other 在 this 的逆时针一侧。
    [[nodiscard]] constexpr Scalar cross(Vector2T other) const noexcept {
        return x * other.y - y * other.x;
    }

    /// 归一化。零向量或退化向量返回 std::nullopt。
    ///
    /// 定义在 UnitVector2.hpp —— 返回类型 UnitVector2T 在那里才完整。
    [[nodiscard]] std::optional<UnitVector2T<Scalar>> normalized(
        core::Tolerance tolerance = {}) const noexcept;
```

并在 `Vector2.hpp` 顶部加前向声明（`Vector2T` 定义之前）：

```cpp
template <typename Scalar> class UnitVector2T;
```

同法处理 `Vector3T`（`operator[]` 用 `index == 0 ? x : (index == 1 ? y : z)`，`dot`/`cross` 为三维版本，`cross` 返回 `Vector3T`）与 `Vector4T`（无 `cross`）。

在 `UnitVector3.hpp` 末尾承接定义：

```cpp
template <typename Scalar>
[[nodiscard]] std::optional<UnitVector3T<Scalar>> Vector3T<Scalar>::normalized(
    core::Tolerance tolerance) const noexcept {
    const Scalar length = this->length();
    if (!core::is_finite(length) || tolerance.is_zero(static_cast<double>(length))) {
        return std::nullopt;
    }
    return UnitVector3T<Scalar>::from_normalized_unchecked(*this / length);
}
```

`UnitVector2T` / `UnitVector3T` 内加入 `dot` 与 `cross` 成员：

```cpp
    [[nodiscard]] constexpr Scalar dot(UnitVector3T other) const noexcept {
        return value_.dot(other.value_);
    }

    /// 叉积。结果不保证是单位向量（两向量平行时为零向量），故返回 Vector3T。
    [[nodiscard]] constexpr Vector3T<Scalar> cross(UnitVector3T other) const noexcept {
        return value_.cross(other.value_);
    }
```

**删除**原来的自由函数 `dot` / `cross` / `normalize`（三处 `Vector` 版本、两处 `UnitVector` 版本、以及 `UnitVector::normalize`），并同步更新 5 个测试文件的调用点。

- [ ] **Step 4: 运行测试，确认通过**

```bash
cmake --build --preset windows-vs-debug && ctest --preset windows-vs-debug
```

Expected: 全绿。**特别注意既有的退化用例必须逐位不变** —— 它们现在走成员路径，语义不得漂移。

- [ ] **Step 5: 提交**

```bash
git add include/GeoCore tests/
git commit -m "refactor(linear): make dot/cross/normalized members; add subscript access"
```

---

### Task 2: `Matrix` / `Quaternion` 成员化

**Files:**
- Modify: `include/GeoCore/linear/Matrix.hpp`、`Quaternion.hpp`
- Modify: `tests/linear/matrix_test.cpp`、`matrix_inverse_test.cpp`、`quaternion_test.cpp`

**Interfaces:**
- Produces:
  - `Scalar MatrixT::determinant() const noexcept`
  - `MatrixT MatrixT::transposed() const noexcept`
  - `std::optional<MatrixT> MatrixT::inverse(core::Tolerance = {}) const noexcept`
  - `static MatrixT MatrixT::identity() noexcept`
  - `Scalar QuaternionT::norm() const noexcept`
  - `QuaternionT QuaternionT::conjugate() const noexcept`
  - `std::optional<QuaternionT> QuaternionT::normalized(core::Tolerance = {}) const noexcept`
  - `Vector3T QuaternionT::rotate(Vector3T) const noexcept`
  - `MatrixT<Scalar,3> QuaternionT::to_matrix() const noexcept`
  - `static QuaternionT QuaternionT::from_axis_angle(UnitVector3T, Scalar) noexcept`
  - `static QuaternionT QuaternionT::identity() noexcept`

- [ ] **Step 1: 写失败测试**

在 `tests/linear/matrix_test.cpp` 追加：

```cpp
TEST_CASE("Matrix members: determinant, transposed, inverse, identity",
          "[linear][matrix]") {
    const Matrix2 m{{{1.0, 2.0}, {3.0, 4.0}}};

    CHECK(m.determinant() == -2.0);
    CHECK(m.transposed() == Matrix2{{{1.0, 3.0}, {2.0, 4.0}}});
    CHECK(Matrix2::identity() == Matrix2{{{1.0, 0.0}, {0.0, 1.0}}});

    const auto inv = m.inverse();
    REQUIRE(inv.has_value());
    CHECK(inv->determinant() == Approx(-0.5));
}
```

在 `tests/linear/quaternion_test.cpp` 追加：

```cpp
TEST_CASE("Quaternion members: norm, conjugate, rotate, to_matrix, factories",
          "[linear][quaternion]") {
    const Quaternion q = Quaternion::from_axis_angle(z_axis, half_pi);

    CHECK(q.norm() == Approx(1.0));
    CHECK(q.conjugate().rotate(Vector3{0.0, 1.0, 0.0}).x == Approx(0.0).margin(1e-15));
    CHECK(Quaternion::identity().rotate(Vector3{1.0, 2.0, 3.0}) == Vector3{1.0, 2.0, 3.0});

    const auto unit = Quaternion{2.0, 0.0, 0.0, 0.0}.normalized();
    REQUIRE(unit.has_value());
    CHECK(unit->w == Approx(1.0));

    const Matrix3 m = q.to_matrix();
    const Vector3 v{1.0, 2.0, 3.0};
    CHECK((m * v).x == Approx(q.rotate(v).x));
}
```

（`z_axis` 沿用该文件已有的匿名命名空间常量。）

- [ ] **Step 2: 运行，确认失败**

```bash
cmake --build --preset windows-vs-debug
```

Expected: 编译失败，成员不存在。

- [ ] **Step 3: 实现并删除旧自由函数**

把 `determinant(m)` / `transpose(m)` / `inverse(m)` / `identity<S,N>()` 的实现搬进 `MatrixT` 成为 `determinant()` / `transposed()` / `inverse()` / 静态 `identity()`；把 `norm(q)` / `conjugate(q)` / `rotate(q,v)` / `to_matrix(q)` / `from_axis_angle(a,θ)` / `identity_quaternion()` 搬进 `QuaternionT`。

**关键：搬运时保持求值顺序逐字不变** —— 这条路径上的 NaN/inf 传播行为已有测试钉住，任何"顺手整理"都可能改变它。`transposed()` 用过去式命名以区别于原地操作，与 `normalized()` 一致。

- [ ] **Step 4: 运行测试并提交**

```bash
cmake --build --preset windows-vs-debug && ctest --preset windows-vs-debug
git add include/GeoCore tests/
git commit -m "refactor(linear): make matrix and quaternion operations members"
```

---

### Task 3: `Transform` 成员化与静态工厂

**Files:**
- Modify: `include/GeoCore/linear/Transform2.hpp`、`Transform3.hpp`
- Modify: `tests/linear/transform2_test.cpp`、`transform3_test.cpp`、`examples/transform_pipeline.cpp`

**Interfaces:**
- Produces:
  - `static Transform3T Transform3T::identity() noexcept`
  - `static Transform3T Transform3T::translation(Vector3T) noexcept`
  - `static Transform3T Transform3T::scaling(Vector3T) noexcept` / `scaling(Scalar)`
  - `static Transform3T Transform3T::rotation(UnitVector3T, Scalar) noexcept`
  - `Vector3T Transform3T::apply(Vector3T) const noexcept`
  - `std::optional<Transform3T> Transform3T::inverse(core::Tolerance = {}) const noexcept`
  - `Transform2T` 同构（`identity` / `translation` / `scaling` / `rotation` / `apply` / `inverse`）

- [ ] **Step 1: 写失败测试**

在 `tests/linear/transform3_test.cpp` 追加：

```cpp
TEST_CASE("Transform3 members and static factories", "[linear][transform3]") {
    const Transform3 t = Transform3::translation(Vector3{10.0, 20.0, 30.0});

    CHECK(t.apply(Vector3{1.0, 2.0, 3.0}) == Vector3{11.0, 22.0, 33.0});
    CHECK(t * Vector3{1.0, 2.0, 3.0} == Vector3{1.0, 2.0, 3.0});

    const Transform3 unit = Transform3::identity();
    CHECK(unit.apply(Vector3{1.0, 2.0, 3.0}) == Vector3{1.0, 2.0, 3.0});

    const auto inv = t.inverse();
    REQUIRE(inv.has_value());
    CHECK(inv->apply(Vector3{11.0, 22.0, 33.0}) == Vector3{1.0, 2.0, 3.0});
}
```

- [ ] **Step 2: 运行，确认失败**

```bash
cmake --build --preset windows-vs-debug
```

Expected: 编译失败。

- [ ] **Step 3: 实现**

把 `identity_transform<S>()` / `translation_3d` / `scaling_3d` / `rotation_3d` / `apply(t,v)` / `inverse(t,tol)` 搬进 `TransformNT` 成为静态工厂或成员，删除旧自由函数，更新两个测试文件与 `examples/transform_pipeline.cpp` 的调用点。

- [ ] **Step 4: 运行全部并提交**

```bash
cmake --build --preset windows-vs-debug && ctest --preset windows-vs-debug
git add include/GeoCore tests/ examples/
git commit -m "refactor(linear): make transform factories and apply members"
```

---

### Task 4: `Point2` / `Point3`

`Point` 与 `Vector` 同底层存储、不同类型语义：点没有加法、没有零元，但有点差与点加向量。

**Files:**
- Create: `include/GeoCore/linear/Point2.hpp`、`Point3.hpp`
- Create: `tests/linear/point2_test.cpp`、`point3_test.cpp`
- Modify: `include/GeoCore/GeoCore.hpp`

**Interfaces:**
- Consumes: `Vector2T` / `Vector3T`
- Produces:
  - `struct Point2T { Scalar x{}, y{}; }`，别名 `Point2` / `Point2f`；`Point3T` 同构
  - `Scalar& operator[](int)` / `const Scalar&` / `std::array<Scalar, N> to_array()`
  - 自由运算符：`Point + Vector -> Point`、`Point - Vector -> Point`、`Point - Point -> Vector`、`operator==`
  - 成员 `Scalar distance_to(PointNT) const noexcept`
  - **不存在** `Point + Point`

- [ ] **Step 1: 写失败测试**

创建 `tests/linear/point3_test.cpp`：

```cpp
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <type_traits>

#include <GeoCore/linear/Point3.hpp>

using Catch::Approx;
using GeoCore::linear::Point3;
using GeoCore::linear::Vector3;

TEST_CASE("Point3 supports subscript and array export", "[linear][point3]") {
    const Point3 p{1.0, 2.0, 3.0};

    CHECK(p[0] == 1.0);
    CHECK(p[1] == 2.0);
    CHECK(p[2] == 3.0);

    const std::array<double, 3> arr = p.to_array();
    CHECK(arr[2] == 3.0);
}

TEST_CASE("point and vector arithmetic follows affine rules",
          "[linear][point3]") {
    const Point3 a{1.0, 2.0, 3.0};
    const Point3 b{4.0, 6.0, 8.0};
    const Vector3 v{10.0, 20.0, 30.0};

    // 点 + 向量 = 点
    STATIC_REQUIRE(std::is_same_v<decltype(a + v), Point3>);
    CHECK(a + v == Point3{11.0, 22.0, 33.0});

    // 点 - 向量 = 点
    STATIC_REQUIRE(std::is_same_v<decltype(a - v), Point3>);
    CHECK(a - v == Point3{-9.0, -18.0, -27.0});

    // 点 - 点 = 向量
    STATIC_REQUIRE(std::is_same_v<decltype(b - a), Vector3>);
    CHECK(b - a == Vector3{3.0, 4.0, 5.0});

    // 点 + 点不存在 —— 下面这行若能编译，说明类型约束失效
    // STATIC_REQUIRE(!requires(Point3 p, Point3 q) { p + q; });
}

TEST_CASE("Point3 distance_to", "[linear][point3]") {
    CHECK(Point3{0.0, 0.0, 0.0}.distance_to(Point3{3.0, 4.0, 0.0}) == Approx(5.0));
}
```

- [ ] **Step 2: 运行，确认失败**

```bash
cmake --build --preset windows-vs-debug
```

Expected: 找不到 `GeoCore/linear/Point3.hpp`。

- [ ] **Step 3: 实现**

`Point3.hpp` 的结构与 `Vector3T` 平行，但**只提供位置语义的运算**：

```cpp
template <typename Scalar>
struct Point3T {
    using scalar_type = Scalar;

    Scalar x{};
    Scalar y{};
    Scalar z{};

    [[nodiscard]] constexpr Scalar& operator[](int index) noexcept {
        return index == 0 ? x : (index == 1 ? y : z);
    }

    [[nodiscard]] constexpr const Scalar& operator[](int index) const noexcept {
        return index == 0 ? x : (index == 1 ? y : z);
    }

    [[nodiscard]] constexpr std::array<Scalar, 3> to_array() const noexcept {
        return {x, y, z};
    }

    [[nodiscard]] Scalar distance_to(Point3T other) const noexcept {
        return (*this - other).length();
    }
};
```

自由运算符（`Point + Point` **刻意不提供**）：

```cpp
template <typename Scalar>
[[nodiscard]] constexpr Point3T<Scalar> operator+(Point3T<Scalar> p, Vector3T<Scalar> v) noexcept;

template <typename Scalar>
[[nodiscard]] constexpr Point3T<Scalar> operator-(Point3T<Scalar> p, Vector3T<Scalar> v) noexcept;

/// 两点之差是位移向量，不是点。
template <typename Scalar>
[[nodiscard]] constexpr Vector3T<Scalar> operator-(Point3T<Scalar> a, Point3T<Scalar> b) noexcept;
```

在 `GeoCore.hpp` 的 `Vector2/3/4` 之后追加两个 include。

- [ ] **Step 4: 运行测试并提交**

```bash
cmake --build --preset windows-vs-debug && ctest --preset windows-vs-debug
git add include/GeoCore tests/linear/point2_test.cpp tests/linear/point3_test.cpp
git commit -m "feat(linear): add Point2T and Point3T"
```

---

### Task 5: `Interval`

一维区间。空区间用 `[+inf, -inf]` 表示（`min > max`），这是该表示的常规约定，使 `merged`/`intersects` 无需特判。

**Files:**
- Create: `include/GeoCore/linear/Interval.hpp`
- Create: `tests/linear/interval_test.cpp`
- Modify: `include/GeoCore/GeoCore.hpp`

**Interfaces:**
- Produces:
  - `struct IntervalT { Scalar min{}; Scalar max{}; }`，别名 `Interval` / `Intervalf`
  - `static constexpr IntervalT empty() noexcept` —— `[+inf, -inf]`
  - `static constexpr IntervalT unbounded() noexcept` —— `[-inf, +inf]`
  - `bool is_empty() const`、`bool contains(Scalar) const`、`bool intersects(IntervalT) const`
  - `Scalar length() const`、`Scalar center() const`
  - `IntervalT merged(IntervalT) const`、`IntervalT expanded(Scalar) const`、`IntervalT clipped(IntervalT) const`（交集）

- [ ] **Step 1: 写失败测试**

覆盖：构造、`empty`/`unbounded`、包含（含端点）、相交、合并（含与空区间合并）、长度/中心、膨胀/收缩、与空区间求交、退化区间（`min == max`）。关键用例：

```cpp
TEST_CASE("merging with the empty interval is the identity",
          "[linear][interval]") {
    const Interval a{1.0, 5.0};
    CHECK(a.merged(Interval::empty()) == a);
    CHECK(Interval::empty().merged(a) == a);
    CHECK(Interval::empty().merged(Interval::empty()).is_empty());
}

TEST_CASE("a degenerate interval contains exactly one point",
          "[linear][interval]") {
    const Interval point{3.0, 3.0};

    CHECK_FALSE(point.is_empty());
    CHECK(point.contains(3.0));
    CHECK_FALSE(point.contains(3.0 + 1e-15));
    CHECK(point.length() == 0.0);
}
```

- [ ] **Step 2-4: 实现、验证、提交**

`contains` 用闭区间；`intersects` 与 `clipped` 对空区间返回 `is_empty` / `empty()`；`expanded` 对空区间返回自身（不产生 `[-inf, +inf]`）。

```bash
cmake --build --preset windows-vs-debug && ctest --preset windows-vs-debug
git add include/GeoCore tests/linear/interval_test.cpp
git commit -m "feat(linear): add IntervalT"
```

---

### Task 6: `Box2` / `Box3`

轴对齐包围盒。空盒 = `min > max`（任一分量），与 `Interval` 的空表示一致。

**Files:**
- Create: `include/GeoCore/linear/Box2.hpp`、`Box3.hpp`
- Create: `tests/linear/box2_test.cpp`、`box3_test.cpp`
- Modify: `include/GeoCore/GeoCore.hpp`

**Interfaces:**
- Consumes: `Point2T` / `Point3T`
- Produces:
  - `struct Box3T { Point3T<Scalar> min{}; Point3T<Scalar> max{}; }`，别名 `Box3` / `Box3f`
  - `static constexpr Box3T empty() noexcept`；`static Box3T from_points(span<const Point3T>)`（★ 后续，本轮先给两点的 `from_corners`）
  - `bool is_empty() const`、`bool contains(Point3T) const`、`bool contains(Box3T) const`、`bool intersects(Box3T) const`
  - `Point3T center() const`、`Vector3T extent() const`、`Vector3T half_extent() const`
  - `Box3T merged(Box3T) const`、`Box3T expanded(Scalar) const`
  - `Point3T corner(int index) const` —— 8 个角，索引位含义文档化

- [ ] **Step 1: 写失败测试**

关键用例：

```cpp
TEST_CASE("an empty box contains nothing and merges as identity",
          "[linear][box3]") {
    const Box3 empty = Box3::empty();
    const Box3 box{Point3{0.0, 0.0, 0.0}, Point3{1.0, 1.0, 1.0}};

    CHECK(empty.is_empty());
    CHECK_FALSE(empty.contains(Point3{0.0, 0.0, 0.0}));
    CHECK_FALSE(empty.intersects(box));
    CHECK(empty.merged(box) == box);
    CHECK(box.merged(empty) == box);
}

TEST_CASE("a degenerate box (a point) is not empty", "[linear][box3]") {
    const Box3 point{Point3{1.0, 2.0, 3.0}, Point3{1.0, 2.0, 3.0}};

    CHECK_FALSE(point.is_empty());
    CHECK(point.contains(Point3{1.0, 2.0, 3.0}));
    CHECK(point.extent() == Vector3{0.0, 0.0, 0.0});
}

TEST_CASE("box corners are ordered and distinct", "[linear][box3]") {
    const Box3 box{Point3{0.0, 0.0, 0.0}, Point3{1.0, 2.0, 4.0}};

    // 索引的 bit0/bit1/bit2 依次选择 x/y/z 取 min 还是 max
    CHECK(box.corner(0) == Point3{0.0, 0.0, 0.0});
    CHECK(box.corner(7) == Point3{1.0, 2.0, 4.0});
    CHECK(box.center() == Point3{0.5, 1.0, 2.0});
    CHECK(box.half_extent() == Vector3{0.5, 1.0, 2.0});
}
```

- [ ] **Step 2: 实现**

**空盒的确切表示（Task 8 依赖它，不得改动）**：`min` 的每个分量为 `+inf`，`max` 的每个分量为 `-inf`。这个选择使 `merged` 与后续 Task 8 的「从空盒开始取 min/max」自然成立，无需对空盒特判：

```cpp
template <typename Scalar>
[[nodiscard]] constexpr Box3T<Scalar> Box3T<Scalar>::empty() noexcept {
    constexpr Scalar inf = std::numeric_limits<Scalar>::infinity();
    return Box3T<Scalar>{Point3T<Scalar>{inf, inf, inf}, Point3T<Scalar>{-inf, -inf, -inf}};
}

[[nodiscard]] constexpr bool is_empty() const noexcept {
    return min.x > max.x || min.y > max.y || min.z > max.z;
}

/// 合并。与空盒合并返回另一方 —— 这一点由上面的表示自动成立。
[[nodiscard]] constexpr Box3T merged(Box3T other) const noexcept {
    Box3T result{};
    result.min.x = min.x < other.min.x ? min.x : other.min.x;
    result.min.y = min.y < other.min.y ? min.y : other.min.y;
    result.min.z = min.z < other.min.z ? min.z : other.min.z;
    result.max.x = max.x > other.max.x ? max.x : other.max.x;
    result.max.y = max.y > other.max.y ? max.y : other.max.y;
    result.max.z = max.z > other.max.z ? max.z : other.max.z;
    return result;
}
```

`corner(int index)` 的索引约定：**bit 0 / bit 1 / bit 2 依次选择 x / y / z 取 `min` 还是 `max`** —— 置位取 `max`。该含义写进文档，因为 `corner(0)` 到 `corner(7)` 的遍历顺序在后续的网格与包围体代码里会被反复使用。

- [ ] **Step 3: 验证并提交**

```bash
cmake --build --preset windows-vs-debug && ctest --preset windows-vs-debug
git add include/GeoCore tests/linear/box2_test.cpp tests/linear/box3_test.cpp
git commit -m "feat(linear): add Box2T and Box3T"
```

---

### Task 7: `Coordinate2` / `Coordinate3`

坐标系：原点 + 一组**正交**单位轴。正交性是坐标系的定义性质 —— 非正交的"坐标系"会让经它表达的变换静默地拉伸几何，因此**必须无法通过公开接口构造出来**。做法与 `UnitVector` 相同：私有构造 + 工厂，工厂返回 `std::optional` 或保证成立。

**Files:**
- Create: `include/GeoCore/linear/Coordinate2.hpp`、`Coordinate3.hpp`
- Create: `tests/linear/coordinate2_test.cpp`、`coordinate3_test.cpp`
- Modify: `include/GeoCore/GeoCore.hpp`

**Interfaces:**
- Consumes: `Point2T` / `Point3T`、`UnitVector2T` / `UnitVector3T`、`Transform2T` / `Transform3T`、`QuaternionT`、`core::Tolerance`
- Produces:
  - `class Coordinate3T`，别名 `Coordinate3` / `Coordinate3f`
  - `static Coordinate3T identity() noexcept`
  - `static std::optional<Coordinate3T> from_axes(Point3T, UnitVector3T x, UnitVector3T y, UnitVector3T z, Tolerance = {})` —— 校验正交且右手
  - `static std::optional<Coordinate3T> from_z_axis(Point3T, UnitVector3T z, Tolerance = {})` —— 自动补全一组正交的 x/y
  - `static Coordinate3T from_transform(const Transform3T&) noexcept` —— 线性部分须为正交（由 `Transform` 的工厂保证）
  - `Point3T origin() const`、`UnitVector3T x_axis() const` / `y_axis()` / `z_axis()`
  - `Point3T to_parent(Point3T local) const`、`Point3T to_local(Point3T parent) const`
  - `Vector3T to_parent(Vector3T local) const`、`Vector3T to_local(Vector3T parent) const`

- [ ] **Step 1: 写失败测试**

```cpp
TEST_CASE("a coordinate frame cannot be built from non-orthogonal axes",
          "[linear][coordinate3][degenerate]") {
    const Point3 o{0.0, 0.0, 0.0};
    const auto x = UnitVector3::from_normalized_unchecked(Vector3{1.0, 0.0, 0.0});
    const auto y = UnitVector3::from_normalized_unchecked(Vector3{0.0, 1.0, 0.0});
    const auto z = UnitVector3::from_normalized_unchecked(Vector3{0.0, 0.0, 1.0});

    // 正交 —— 成功
    CHECK(Coordinate3::from_axes(o, x, y, z).has_value());

    // 斜交 —— 必须被拒绝，而不是构造出一个会拉伸几何的“坐标系”
    const auto skewed = UnitVector3::from_normalized_unchecked(
        Vector3{0.6, 0.8, 0.0});
    CHECK_FALSE(Coordinate3::from_axes(o, x, skewed, z).has_value());

    // 左手系 —— 同样拒绝
    CHECK_FALSE(Coordinate3::from_axes(o, x, y, -z).has_value());
}

TEST_CASE("coordinate frame round-trips a point", "[linear][coordinate3]") {
    const auto frame = Coordinate3::from_z_axis(
        Point3{10.0, 0.0, 0.0},
        UnitVector3::from_normalized_unchecked(Vector3{0.0, 0.0, 1.0}));
    REQUIRE(frame.has_value());

    const Point3 local{1.0, 2.0, 3.0};
    const Point3 round_trip = frame->to_local(frame->to_parent(local));

    CHECK(round_trip.x == Approx(local.x).margin(1e-12));
    CHECK(round_trip.y == Approx(local.y).margin(1e-12));
    CHECK(round_trip.z == Approx(local.z).margin(1e-12));
}
```

- [ ] **Step 2: 实现**

`from_z_axis` 的正交补全是最容易写错的一处 —— 参考向量选得不好，当 `z` 接近某个坐标轴时叉积会退化为零向量：

```cpp
/// 由 z 轴补全一组正交的 x/y。
///
/// 参考向量取「z 的绝对值最小的那个分量方向」—— 该方向与 z 的夹角必然
/// 不小于 45°，因此叉积不会退化成零向量。若固定用 (0,0,1) 作参考，z 接近
/// z 轴时就会失败。
template <typename Scalar>
[[nodiscard]] std::optional<Coordinate3T<Scalar>> Coordinate3T<Scalar>::from_z_axis(
    Point3T<Scalar> origin, UnitVector3T<Scalar> z, core::Tolerance tolerance) noexcept {
    const Scalar ax = core::absolute_value(z.x());
    const Scalar ay = core::absolute_value(z.y());
    const Scalar az = core::absolute_value(z.z());

    Vector3T<Scalar> reference{};
    if (ax <= ay && ax <= az) {
        reference = Vector3T<Scalar>{Scalar{1}, Scalar{0}, Scalar{0}};
    } else if (ay <= az) {
        reference = Vector3T<Scalar>{Scalar{0}, Scalar{1}, Scalar{0}};
    } else {
        reference = Vector3T<Scalar>{Scalar{0}, Scalar{0}, Scalar{1}};
    }

    const auto x = cross(reference, z.as_vector()).normalized(tolerance);
    if (!x.has_value()) {
        return std::nullopt;
    }
    const auto y = z.cross(*x).normalized(tolerance);
    if (!y.has_value()) {
        return std::nullopt;
    }
    return Coordinate3T<Scalar>::from_axes(origin, *x, *y, z, tolerance);
}
```

`from_axes` 的校验：三轴两两点积的绝对值须在容差内为零，且 `x × y` 与 `z` 同向（右手系）。

- [ ] **Step 3: 验证并提交**

```bash
cmake --build --preset windows-vs-debug && ctest --preset windows-vs-debug
git add include/GeoCore tests/linear/coordinate2_test.cpp tests/linear/coordinate3_test.cpp
git commit -m "feat(linear): add Coordinate2T and Coordinate3T"
```

---

### Task 8: `OrientedBox2` / `OrientedBox3`

**Files:**
- Create: `include/GeoCore/linear/OrientedBox2.hpp`、`OrientedBox3.hpp`
- Create: `tests/linear/oriented_box2_test.cpp`、`oriented_box3_test.cpp`
- Modify: `include/GeoCore/GeoCore.hpp`

**Interfaces:**
- Consumes: `Coordinate2T` / `Coordinate3T`、`Box2T` / `Box3T`
- Produces:
  - `struct OrientedBox3T { Coordinate3T<Scalar> frame; Vector3T<Scalar> half_extent; }`，别名 `OrientedBox3` / `OrientedBox3f`
  - `Point3T center() const`（即 `frame.origin()`）
  - `bool contains(Point3T, Tolerance = {}) const`（`to_local` 后逐轴比较）
  - `Point3T corner(int index) const` —— 8 个角
  - **`Box3T to_axis_aligned() const`** —— 紧致地包住旋转后的盒子
  - `OrientedBox3T expanded(Scalar) const`

- [ ] **Step 1: 写失败测试**

关键用例 —— **旋转后的紧包围盒必须真的变大**：

```cpp
TEST_CASE("rotating a box grows its axis-aligned bounding box",
          "[linear][orientedbox3]") {
    const auto frame = Coordinate3::from_z_axis(
        Point3{0.0, 0.0, 0.0},
        UnitVector3::from_normalized_unchecked(Vector3{0.0, 0.0, 1.0}));
    REQUIRE(frame.has_value());

    const OrientedBox3 box{*frame, Vector3{1.0, 1.0, 1.0}};   // 边长 2 的立方体
    const Box3 aabb = box.to_axis_aligned();

    // 未旋转时紧包围盒就是自身
    CHECK(aabb.min.x == Approx(-1.0));
    CHECK(aabb.max.x == Approx(1.0));

    // 绕 z 转 45° 后，x/y 方向的紧包围盒必须扩展到 √2
    const auto rotated_frame = Coordinate3::from_z_axis(
        Point3{0.0, 0.0, 0.0},
        UnitVector3::from_normalized_unchecked(Vector3{0.0, 0.0, 1.0}));
    const OrientedBox3 rotated{
        Coordinate3::from_axes(
            Point3{0.0, 0.0, 0.0},
            UnitVector3::from_normalized_unchecked(Vector3{0.7071067811865476, 0.7071067811865476, 0.0}),
            UnitVector3::from_normalized_unchecked(Vector3{-0.7071067811865476, 0.7071067811865476, 0.0}),
            UnitVector3::from_normalized_unchecked(Vector3{0.0, 0.0, 1.0}))
            .value(),
        Vector3{1.0, 1.0, 1.0}};
    const Box3 rotated_aabb = rotated.to_axis_aligned();

    CHECK(rotated_aabb.max.x == Approx(1.4142135623730951).margin(1e-12));
    CHECK(rotated_aabb.max.x > aabb.max.x);   // 真的变大了
    CHECK(rotated_aabb.max.z == Approx(1.0)); // z 方向不变
}
```

- [ ] **Step 2: 实现**

`to_axis_aligned` 必须用**八个角的实际 min/max**：

```cpp
/// 紧致的轴对齐包围盒。
///
/// 取八个角的 min/max，而不是「中心 ± 半轴长度之和」—— 后者把盒子当成球
/// 来处理，在旋转下系统地过松（绕 z 转 45° 的立方体，正确结果是 √2 倍，
/// 而用半轴之和会得到 2 倍）。
template <typename Scalar>
[[nodiscard]] Box3T<Scalar> OrientedBox3T<Scalar>::to_axis_aligned() const noexcept {
    Box3T<Scalar> result = Box3T<Scalar>::empty();
    for (int i = 0; i < 8; ++i) {
        const Point3T<Scalar> c = corner(i);
        result.min.x = c.x < result.min.x ? c.x : result.min.x;
        result.min.y = c.y < result.min.y ? c.y : result.min.y;
        result.min.z = c.z < result.min.z ? c.z : result.min.z;
        result.max.x = c.x > result.max.x ? c.x : result.max.x;
        result.max.y = c.y > result.max.y ? c.y : result.max.y;
        result.max.z = c.z > result.max.z ? c.z : result.max.z;
    }
    return result;
}
```

（起点用 `Box3T::empty()`，其 `min = +inf` / `max = -inf` 使首轮比较自然成立。）

- [ ] **Step 3: 验证并提交**

```bash
cmake --build --preset windows-vs-debug && ctest --preset windows-vs-debug
git add include/GeoCore tests/linear/oriented_box2_test.cpp tests/linear/oriented_box3_test.cpp
git commit -m "feat(linear): add OrientedBox2T and OrientedBox3T"
```

---

### Task 9: `Transform::transform_point` 与示例更新

`Point` 进了 `linear` 之后，`Transform` 终于可以直接变换点 —— 这消除了原先 `apply(t, Vector)` 把位置塞进向量的补偿性分工。

**Files:**
- Modify: `include/GeoCore/linear/Transform2.hpp`、`Transform3.hpp`
- Modify: `tests/linear/transform3_test.cpp`、`examples/transform_pipeline.cpp`

**Interfaces:**
- Produces:
  - `Point3T Transform3T::transform_point(Point3T) const noexcept`
  - `Point2T Transform2T::transform_point(Point2T) const noexcept`
  - 自由运算符 `Transform3T * Point3T -> Point3T`（与 `Transform * Vector3T` 并列）

- [ ] **Step 1: 写失败测试**

```cpp
TEST_CASE("a transform carries points, not just vectors",
          "[linear][transform3]") {
    const Transform3 t = Transform3::translation(Vector3{10.0, 0.0, 0.0});
    const Point3 p{1.0, 2.0, 3.0};

    // 点被平移
    STATIC_REQUIRE(std::is_same_v<decltype(t * p), Point3>);
    CHECK(t * p == Point3{11.0, 2.0, 3.0});
    CHECK(t.transform_point(p) == Point3{11.0, 2.0, 3.0});

    // 方向不被平移 —— 这是 operator* 与 transform_point 的分工
    CHECK(t * Vector3{1.0, 2.0, 3.0} == Vector3{1.0, 2.0, 3.0});
}
```

- [ ] **Step 2-3: 实现**

`transform_point` 与 `apply` 的数学内容相同，但接受 `Point3T` 并返回 `Point3T`。**保留 `apply(Vector3T)`** —— 它仍有用途（对以 `Vector3T` 承载的位置做变换），但文档须指明新代码应当用 `transform_point`。

更新 `examples/transform_pipeline.cpp` 改用 `Point3` + `transform_point`，让示例展示正确的用法；预期输出随之需要修正（把 `apply(model, position)` 换成 `model * position`）。

- [ ] **Step 4: 运行全部并提交**

```bash
cmake --build --preset windows-vs-debug && ctest --preset windows-vs-debug
git add include/GeoCore tests/ examples/
git commit -m "feat(linear): let Transform carry points"
```

---

### Task 10: 文档与收尾

**Files:**
- Modify: `README.md`
- Modify: `examples/vector_basics.cpp`（成员化后的调用点）

- [ ] **Step 1: 更新 README 与示例**

README 的 quick start 改用成员形式（`a.dot(b)`、`a.normalized()`），并在"What works today"表里加入本轮新增的五个类型。示例同样改用成员形式，重跑并核对输出。

- [ ] **Step 2: 全量验证**

```bash
cmake --preset windows-vs
cmake --build --preset windows-vs-debug
ctest --preset windows-vs-debug
./build/windows-vs/examples/Debug/vector_basics.exe
./build/windows-vs/examples/Debug/transform_pipeline.exe
```

Expected: 全绿，两个示例输出与 README/示例注释中的期望值一致。

- [ ] **Step 3: 提交**

```bash
git add README.md examples/
git commit -m "docs: document the member API and the new base types"
```

---

### Task 11: 性能基准

本阶段没有算法，但**有大量会被高频调用的运算** —— `dot`/`cross`/`length`/`normalized` 与矩阵乘法是上层每一处几何代码的原子操作，成员化与下标访问是否引入开销必须被测量，而不是假设"内联了就没差别"。

**Files:**
- Create: `benchmarks/CMakeLists.txt`
- Create: `benchmarks/linear_benchmarks.cpp`
- Modify: 顶层 `CMakeLists.txt`（加入 `GEOCORE_BUILD_BENCHMARKS` 选项与 `add_subdirectory`）

**Interfaces:**
- Consumes: 全部 `linear` 公开接口
- Produces: 可执行的基准程序 `GeoCoreBenchmarks`，打印每项操作的耗时与吞吐

- [ ] **Step 1: 接入 Google Benchmark**

在 `benchmarks/CMakeLists.txt` 里经 FetchContent 引入 Google Benchmark（与测试框架一样，属**开发期依赖**，不得进入库的公开接口）：

```cmake
include(FetchContent)

FetchContent_Declare(
    GoogleBenchmark
    GIT_REPOSITORY https://github.com/google/benchmark.git
    GIT_TAG        v1.9.1
    GIT_SHALLOW    TRUE
)
set(BENCHMARK_ENABLE_TESTING OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(GoogleBenchmark)

add_executable(GeoCoreBenchmarks linear_benchmarks.cpp)
target_link_libraries(GeoCoreBenchmarks PRIVATE GeoCore::GeoCore benchmark::benchmark)
```

顶层 `CMakeLists.txt` 加选项与环境变量 `GEOCORE_BENCHMARKS_INCLUDE_DIR`（复用 `GEOCORE_SOURCE_INCLUDE_DIR`）：

```cmake
option(GEOCORE_BUILD_BENCHMARKS "Build GeoCore benchmarks" OFF)

if(GEOCORE_BUILD_BENCHMARKS)
    add_subdirectory(benchmarks)
endif()
```

默认关闭 —— 基准是开发工具，不应拖慢使用者的配置时间。

- [ ] **Step 2: 写基准**

`benchmarks/linear_benchmarks.cpp` 覆盖**高频路径**，每项都要防止被编译器优化掉（用 `benchmark::DoNotOptimize`）：

```cpp
#include <benchmark/benchmark.h>

#include <GeoCore/GeoCore.hpp>

using namespace GeoCore::linear;

namespace {

void bench_dot(benchmark::State& state) {
    const Vector3 a{1.0, 2.0, 3.0};
    const Vector3 b{4.0, 5.0, 6.0};
    for (auto _ : state) {
        benchmark::DoNotOptimize(a.dot(b));
    }
}
BENCHMARK(bench_dot);

void bench_cross(benchmark::State& state) {
    const Vector3 a{1.0, 2.0, 3.0};
    const Vector3 b{4.0, 5.0, 6.0};
    for (auto _ : state) {
        benchmark::DoNotOptimize(a.cross(b));
    }
}
BENCHMARK(bench_cross);

void bench_length(benchmark::State& state) {
    const Vector3 v{3.0, 4.0, 12.0};
    for (auto _ : state) {
        benchmark::DoNotOptimize(v.length());
    }
}
BENCHMARK(bench_length);

void bench_normalized(benchmark::State& state) {
    const Vector3 v{3.0, 4.0, 12.0};
    for (auto _ : state) {
        benchmark::DoNotOptimize(v.normalized());
    }
}
BENCHMARK(bench_normalized);

void bench_subscript(benchmark::State& state) {
    const Vector3 v{3.0, 4.0, 12.0};
    for (auto _ : state) {
        benchmark::DoNotOptimize(v[0] + v[1] + v[2]);
    }
}
BENCHMARK(bench_subscript);

void bench_matrix_vector(benchmark::State& state) {
    const Matrix4 m{{{2.0, 0.0, 0.0, 1.0},
                     {0.0, 3.0, 0.0, 2.0},
                     {0.0, 0.0, 4.0, 3.0},
                     {0.0, 0.0, 0.0, 1.0}}};
    const Vector4 v{1.0, 2.0, 3.0, 4.0};
    for (auto _ : state) {
        benchmark::DoNotOptimize(m * v);
    }
}
BENCHMARK(bench_matrix_vector);

void bench_matrix_inverse(benchmark::State& state) {
    const Matrix3 m{{{1.0, 2.0, 3.0}, {0.0, 1.0, 4.0}, {5.0, 6.0, 0.0}}};
    for (auto _ : state) {
        benchmark::DoNotOptimize(m.inverse());
    }
}
BENCHMARK(bench_matrix_inverse);

void bench_quaternion_rotate(benchmark::State& state) {
    const auto axis = UnitVector3::from_normalized_unchecked(Vector3{0.0, 0.0, 1.0});
    const Quaternion q = Quaternion::from_axis_angle(axis, 0.7);
    const Vector3 v{1.0, 2.0, 3.0};
    for (auto _ : state) {
        benchmark::DoNotOptimize(q.rotate(v));
    }
}
BENCHMARK(bench_quaternion_rotate);

} // namespace
```

- [ ] **Step 3: 运行并记录基线**

```bash
cmake --preset windows-vs -DGEOCORE_BUILD_BENCHMARKS=ON
cmake --build --preset windows-vs-release --target GeoCoreBenchmarks
./build/windows-vs/benchmarks/Release/GeoCoreBenchmarks.exe --benchmark_min_time=0.5s
```

把结果写入 `benchmarks/BASELINE.md`，**标注工具链与构建配置**（MSVC 版本、Release、`/O2`）—— 基准数字脱离环境没有意义。

**判定标准**：`dot`/`cross`/`subscript` 的单项耗时应在**个位数纳秒**量级；若某一项达到数十纳秒，说明内联失败，需要检查 `operator[]` 的 `switch` 是否被优化为直接寻址。这是本任务要防的唯一一件事。

- [ ] **Step 4: 提交**

```bash
git add benchmarks/ CMakeLists.txt
git commit -m "perf: add benchmarks for the hot linear-algebra paths"
```

---

## 完成标准

1. 全库测试通过，且**既有断言逐位不变** —— 成员化是搬迁，不是修改语义。
2. `p + p` 无法编译，`p - p` 得到 `Vector`；`Coordinate` 无法从非正交轴构造；`OrientedBox::to_axis_aligned` 对旋转盒真的变大。
3. 自由函数形式的 `dot`/`cross`/`norm`/`determinant`/`transposed`/`inverse`/`normalized`/`rotate`/`to_matrix`/`apply` 与各静态工厂**已不存在**；运算符仍为自由函数。
4. 两个示例以成员形式编写并输出正确。
5. `benchmarks/BASELINE.md` 存有可复现的基线，且 `dot`/`cross`/`subscript` 处于个位数纳秒量级。

**不在本阶段**：曲线（登记于 spec §5.6）、JSON、SVG。SVG 依赖 2D 图形对象，须待曲线落地；JSON 已明确暂不实现。

**后续阶段的测试要求（来自本轮反馈，登记备查）**：性能测试须覆盖涉及算法或可能被高频调用的场景 —— 本阶段的基准只覆盖线性代数原子操作，`predicates`/`query`/`polygon` 阶段各自需要自己的基准（尤其布尔运算与 BVH 遍历）。精度测试方面，`predicates` 阶段必须对精确谓词做容差边界的穷举验证。
