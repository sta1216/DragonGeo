# GeoCore 阶段 2：API 成员化与基础类型 实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 把 `linear` 层的具名运算从自由函数改为成员函数，并补齐 `Point2/3`、`Interval`、`Box2/3`、`OrientedBox2/3`、`Coordinate2/3` 五组基础类型。

**Architecture:** 全部落在 `linear` 层，不新增层。`Point` 进入 `linear` 后，`Transform` 得以直接提供 `transform_point(Point) -> Point`，取代原先把位置塞进 `Vector` 的 `apply(t, Vector)` 分工。运算符保持自由函数（二元运算的对称性），具名运算一律成员。

**Tech Stack:** C++20 · CMake ≥ 3.20 · Catch2 v3（FetchContent）· 既有 `GeoCore::GeoCore` target。

**Spec:** `docs/superpowers/specs/2026-09-29-geocore-design.md`（§3 分层、§5.0 linear 清单、§9 决策 15–17）

> **给负责派发的人：每次改动本文件之后，都要重新生成当前任务的 brief，并核对生成的文本确实含这次改动。**
> brief 是从本文件抽出来的快照，`.superpowers/sdd/` 被 gitignore，**没有任何机制防止它漂移**。Task 4 的修复轮就踩过：计划已改，brief 仍是旧版，而派发时被口头告知「已重新生成」。实现者自己去比对才发现，若它照旧 brief 干活，补的断言会缺三条而无人察觉。
> 最省事的做法是**把「重新生成 brief」放在最后一次计划提交之后**，作为派发前的固定动作。**「我记得已经生成过」不算核对** —— 用这条命令实测，比 brief 旧就重新生成：
>
> ```bash
> # brief 比计划最后一次提交还旧 → 已经漂移，必须重新生成
> test "$(stat -c %Y .superpowers/sdd/<plan-slug>/task-<N>-brief.md)" -lt \
>      "$(git log -1 --format=%ct -- docs/superpowers/plans/<plan-file>.md)" \
>   && echo "STALE — regenerate" || echo "fresh"
> ```
>
> 更要紧的是：**不要让实现者去发现派发方的不实陈述** —— 那是把校验责任推给了被校验的一方。

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
6. **成员化改变了表达式的解析** —— 自由函数 `apply(t, v)` 改成 `t.apply(v)` 之后，`apply(a * b, v)` 的直译 `a * b.apply(v)` 不再等价：`operator*` 会先接住 `b.apply(v)`（`operator*(Transform, Vector)` 存在），于是算的是「`a` 的线性部分作用在 `b` 变换后的结果上」，而不是「先复合再施加完整仿射」。**两种写法都能编译**，编译器不会帮忙。必须写成 `(a * b).apply(v)`。Task 3 已实测到这一点，且其中一例因数值巧合而**假通过** —— 即测试套件对此无检出能力。（Task 3、Task 8、Task 9）

7. **测试通过 ≠ 实现正确 —— 新类型任务要做变异测试。** 本计划反复出现同一个模式：**测试的输入取自一个「错误无法显现」的子空间**。已经出现过的形态：单位阵乘法（掩盖转置）、只绕 z 轴（掩盖 x/y 分量）、纯平移（线性部分是恒等）、世界 z 轴补全（得到的恰好是标准基）、立方体（对轴置换对称）、Δz = 0（掩盖 z 分量）、所有期望值恰好与左操作数共享 x（掩盖 `==` 的 y/z 比较）。这类缺陷**骗得过任何次数的仔细阅读** —— 本计划有几处就是这样被两轮审查放过的。

   **因此：凡是新增或修改具名运算的任务（Task 4–11）都必须做一次廉价变异验证，并在报告里给出结果。** 这一条不限于新类型 —— 基准（Task 11）尤其适用：一个被常量折叠的基准和一个恒真的断言是同一个东西。方法：把仓库的头文件复制到仓库外，注入一个错误（`==` 只比较第一个分量、某个具名运算丢掉一个分量、`to_array()` 槽位置换、返回类型加上 `const`），用**真实的测试文件**编译链接后运行，看是否失败。**若变异体存活，说明测试没有检出能力，必须补断言。** Task 4 用这个方法接连找出三个洞（`==` 只比 x、`distance_to` 丢掉 z、`to_array()` 只查末槽），而**两轮仔细阅读一个都没发现**。

   **一条必须遵守的方法论前提：对照组通过并不能证明遮蔽生效。** 若 shadow 目录的层级放错（例如放了 `shadow/linear/` 而不是 `shadow/GeoCore/linear/`），`-I` 会**静默**回落到仓库里的真实头文件 —— 此时对照组照样全绿，而所有「变异体死亡」的结论都是假的，整个验证无声地失效。

   **默认用「金丝雀变异」自证，`/showIncludes` 作为加分项。** 金丝雀 = 一个必定死亡的变异（例如让某个被测函数恒返回 0）；它可移植（不依赖 MSVC 的开关），而且**同时验证了编译、链接、运行整条链路**，而 `/showIncludes` 只验证编译期解析。**在采信任何一个变异体的结果之前先让金丝雀死一次。**（Task 4–11）

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
  - `Scalar VectorNT::dot(VectorNT) const noexcept`（2D/3D/4D 都有）
  - `Scalar Vector2T::cross(Vector2T) const noexcept`；`Vector3T Vector3T::cross(Vector3T) const noexcept`
  - `std::optional<UnitVector2T> Vector2T::normalized(core::Tolerance = {}) const noexcept`
  - `std::optional<UnitVector3T> Vector3T::normalized(core::Tolerance = {}) const noexcept`
  - **`Vector4T` 不提供 `normalized()`** —— 库里不存在 `UnitVector4T`，而 `normalized` 的返回类型就是它。四维单位向量没有几何用途，不为此新增类型。该决定在 `Vector4.hpp` 里以注释记录。
  - `Scalar& VectorNT::operator[](int) noexcept` / `const Scalar& operator[](int) const noexcept`
  - `std::array<Scalar, N> VectorNT::to_array() const noexcept`
  - `Scalar UnitVectorNT::dot(UnitVectorNT) const noexcept`
  - `Scalar UnitVector2T::cross(UnitVector2T) const noexcept` / `Vector3T UnitVector3T::cross(UnitVector3T) const noexcept`

- [ ] **Step 1: 写失败测试**

在 `tests/linear/vector3_test.cpp` 追加（**该文件需要新增 `#include <GeoCore/linear/UnitVector3.hpp>`** —— `normalized()` 的返回类型在那里才完整；这是循环依赖处理方式的固有义务，见下）。

```cpp
#include <GeoCore/linear/UnitVector3.hpp>   // normalized() 的返回类型
```


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

在 `UnitVector3.hpp` 末尾承接定义（**注释必须保留** —— 它记的是"为什么拒绝非有限值"，是阶段 1 三轮裁定的结论，丢掉就要重新推导）：

```cpp
template <typename Scalar>
[[nodiscard]] std::optional<UnitVector3T<Scalar>> Vector3T<Scalar>::normalized(
    core::Tolerance tolerance) const noexcept {
    const Scalar length = this->length();
    // 非有限长度同样返回 nullopt。容差判断对 ±inf 与 NaN 一律返回 false
    // （`is_zero` 刻意不把溢出量静默归类为零），若就此放行，本函数会交出一个
    // has_value() 为真、内容却是 NaN 的「单位向量」：调用者无从察觉，而 NaN
    // 会一路污染 dot / cross 与每一个容差比较 —— 那些比较对 NaN 都返回 false，
    // 下游几何代码会静默走「否」分支。NaN 输入比无穷更常见：任何上游的
    // 0/0 或 inf - inf 都会落到这里。
    if (!core::is_finite(length) || tolerance.is_zero(static_cast<double>(length))) {
        return std::nullopt;
    }
    return UnitVector3T<Scalar>::from_normalized_unchecked(*this / length);
}
```

**顺带一条文档义务**：调用 `normalized()` 的使用者需要**额外包含** `<GeoCore/linear/UnitVector3.hpp>` 才能用到返回类型 —— 这是「声明在 Vector 头、定义在 UnitVector 头」这一循环依赖处理方式的固有代价。在 `Vector2.hpp` / `Vector3.hpp` 的 `normalized` 声明处注明这一点。

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
- Modify: `include/GeoCore/linear/Transform2.hpp`、`Transform3.hpp`（**仅**把内部对已删自由函数的调用改成成员形式；见 Step 3 的边界说明）
- Modify: `tests/linear/matrix_test.cpp`、`matrix_inverse_test.cpp`、`quaternion_test.cpp`
- **不要动**：`tests/linear/transform2_test.cpp`、`transform3_test.cpp`、`examples/transform_pipeline.cpp`

**Interfaces:**
- Produces:
  - `Scalar MatrixT::determinant() const noexcept`
  - `MatrixT MatrixT::transposed() const noexcept`
  - `std::optional<MatrixT> MatrixT::inverse(core::Tolerance = {}) const noexcept`
  - `static MatrixT MatrixT::identity() noexcept`
  - `Scalar QuaternionT::norm() const noexcept`
  - `Scalar QuaternionT::dot(QuaternionT) const noexcept`
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

    // 共轭把转过的角度原路转回。q 是绕 z 轴 +90°，所以 (0,1,0) 应落到 (1,0,0)。
    // 注意不要照抄文件末尾 x 轴用例里的 `.x == 0` —— 那条是绕 x 轴转，x 才恰好为 0；
    // 绕 z 轴转的 x 分量是 ±1。
    const Vector3 undone = q.conjugate().rotate(Vector3{0.0, 1.0, 0.0});
    CHECK(undone.x == Approx(1.0));
    CHECK(undone.y == Approx(0.0).margin(1e-15));

    CHECK(Quaternion::identity().rotate(Vector3{1.0, 2.0, 3.0}) == Vector3{1.0, 2.0, 3.0});

    // 四个分量都非零且互不相同 —— 只用 w 非零的输入，一个「只缩放 w」的
    // 实现也能通过。模长 sqrt(1+4+9+16) = sqrt(30)。
    const auto unit = Quaternion{1.0, 2.0, 3.0, 4.0}.normalized();
    REQUIRE(unit.has_value());
    CHECK(unit->w == Approx(1.0 / std::sqrt(30.0)));
    CHECK(unit->x == Approx(2.0 / std::sqrt(30.0)));
    CHECK(unit->y == Approx(3.0 / std::sqrt(30.0)));
    CHECK(unit->z == Approx(4.0 / std::sqrt(30.0)));

    // 手算值：绕 z 轴转 90° 把 (1,2,3) 送到 (-2,1,3)。这与「和 rotate 比」是
    // 两回事 —— 后者只能证明两者自洽，共同的符号约定错误照样通过。
    const Matrix3 m = q.to_matrix();
    const Vector3 v{1.0, 2.0, 3.0};
    const Vector3 by_matrix = m * v;
    CHECK(by_matrix.x == Approx(-2.0));
    CHECK(by_matrix.y == Approx(1.0));
    CHECK(by_matrix.z == Approx(3.0));

    const Vector3 by_quaternion = q.rotate(v);
    CHECK(by_matrix.x == Approx(by_quaternion.x));
    CHECK(by_matrix.y == Approx(by_quaternion.y));
    CHECK(by_matrix.z == Approx(by_quaternion.z));
}
```

（`z_axis` 沿用该文件已有的匿名命名空间常量；`<cmath>` 与 `using Catch::Approx;` 该文件已引入。）

- [ ] **Step 2: 运行，确认失败**

```bash
cmake --build --preset windows-vs-debug
```

Expected: 编译失败，成员不存在。

- [ ] **Step 3: 实现并删除旧自由函数**

把 `determinant(m)` / `transpose(m)` / `inverse(m)` / `identity<S,N>()` 的实现搬进 `MatrixT` 成为 `determinant()` / `transposed()` / `inverse()` / 静态 `identity()`；把 `norm(q)` / `conjugate(q)` / `rotate(q,v)` / `to_matrix(q)` / `from_axis_angle(a,θ)` / `identity_quaternion()` 搬进 `QuaternionT`（后者成为静态 `QuaternionT::identity()`）。

**关键：搬运时保持求值顺序逐字不变** —— 这条路径上的 NaN/inf 传播行为已有测试钉住，任何"顺手整理"都可能改变它。`transposed()` 用过去式命名以区别于原地操作，与 `normalized()` 一致。

**边界：哪些 `inverse` 不归本任务。** 库里有两个同名自由函数 —— Matrix 的 `inverse(m, tol)`（本任务删）与 Transform 的 `inverse(t, tol)`（`Transform2.hpp:92`、`Transform3.hpp:124`，**Task 3 才处理**）。后者要保持原样，本任务只改它**函数体内**对前者的调用。

删除自由函数会打断每一个调用点。本任务必须一并改掉的（已全库扫描确认）：

| 文件 | 位置 | 改法 |
|---|---|---|
| `Transform2.hpp` | `identity<Scalar, 3>()` | `MatrixT<Scalar, 3>::identity()` |
| `Transform2.hpp` | `inverse(t.matrix, tolerance)` | `t.matrix.inverse(tolerance)` |
| `Transform3.hpp` | `identity<Scalar, 4>()` | `MatrixT<Scalar, 4>::identity()` |
| `Transform3.hpp` | `from_axis_angle(axis, angle_radians)` | `QuaternionT<Scalar>::from_axis_angle(...)` |
| `Transform3.hpp` | `to_matrix(q)` | `q.to_matrix()` |
| `Transform3.hpp` | `inverse(t.matrix, tolerance)` | `t.matrix.inverse(tolerance)` |
| `tests/linear/matrix_test.cpp` | `identity<double, 3>` / `identity<double, 4>` / `transpose(m)` 共 6 处（第 132 行注释里也提到 `identity<4>()`） | 成员形式 |
| `tests/linear/matrix_inverse_test.cpp` | `determinant(...)` / `identity<double, N>()`，以及**矩阵实参**的 `inverse(...)` | 成员形式 |
| `tests/linear/quaternion_test.cpp` | `norm` / `conjugate` / `rotate` / `to_matrix` / `from_axis_angle` | 成员形式 |

**注意 `matrix_inverse_test.cpp` 里的 `inverse` 有两类，别一刀切。** 第 152/160/166/191 行的实参是 `scaling_3d(...)` / `translation_3d(...)` 的返回值，即 `Transform3T` —— 那调的是 **Transform 自己的**自由 `inverse`，返回 `std::optional<Transform3T>`（紧随其后的 `(*tiny).matrix(0, 0)` 就是证据）。这四处**保持自由调用**，该文件的 `using GeoCore::linear::inverse;` 也保留，于是它会同时出现 `x.inverse()` 与 `inverse(t)` 直到 Task 3 —— 这是预期状态，不是遗漏。

**`QuaternionT::dot` 也在本任务范围内。** 阶段 1 之后它是全库**唯一**残留的自由 `dot`（`Quaternion.hpp:149`），计划 line 20 的「具名运算一律成员函数」把 `dot` 列入其内，阶段末判据也要求自由 `dot` 不存在，而后续任务没有一个会碰 `Quaternion.hpp`。改成员后记得一并处理 `quaternion_test.cpp` 的 `using GeoCore::linear::dot;`（using 声明指向已不存在的名字是编译错误）与 `dot(q, q)` 调用点。

**改完请自己再 grep 一遍**，不要只信这张表 —— 阶段 1 的教训是调用点会藏在测试与示例里：

```bash
grep -rn "\bdeterminant(\|\btranspose(\|\bnorm(\|\bconjugate(\|\brotate(\|\bto_matrix(\|\bfrom_axis_angle(\|\bidentity_quaternion(\|\bidentity<" include/ examples/ tests/
```

预期：剩下的命中只有 Transform 自己的 `inverse` 定义与调用，以及注释/文档文字。

**顺带修一处过时引用**（阶段 1 复核留下的 Minor）：`Quaternion.hpp:81` 的注释写着「与 UnitVector::normalize / Matrix::inverse 同一条原则」，这两个自由函数都已不存在 —— 改为指向当前的成员形式 `UnitVector3T::normalized` / `MatrixT::inverse`。

- [ ] **Step 4: 运行测试并提交**

```bash
cmake --build --preset windows-vs-debug && ctest --preset windows-vs-debug
git add include/GeoCore tests/
git commit -m "refactor(linear): make matrix and quaternion operations members"
```

验收：既有断言的值一个都不许变（这是成员化，不是行为变更），用例数与断言数只增加 Step 1 新增的那些。

---

### Task 3: `Transform` 成员化与静态工厂

**Files:**
- Modify: `include/GeoCore/linear/Transform2.hpp`、`Transform3.hpp`
- Modify: `include/GeoCore/linear/Matrix.hpp`（**仅**三处注释里对 `translation_3d` / `scaling_3d` 的具名引用，见 Step 3）
- Modify: `tests/linear/transform2_test.cpp`、`transform3_test.cpp`、`matrix_inverse_test.cpp`、`examples/transform_pipeline.cpp`
- Modify: `ci/consumer/main.cpp`（CI 的 `consumer-smoke` 作业，见 Step 3c）

**Interfaces:**
- Produces:
  - `static Transform3T Transform3T::identity() noexcept`
  - `static Transform3T Transform3T::translation(Vector3T) noexcept`
  - `static Transform3T Transform3T::scaling(Vector3T) noexcept`
  - `template <std::floating_point Factor> static Transform3T Transform3T::scaling(Factor) noexcept` —— **不是** `scaling(Scalar)`。原自由函数 `scaling_3d` 的 `Scalar` 是**从实参推导**的模板参数并带 `floating_point` 约束，这正是「`scaling(2)` 传整数字面量必须编译失败」那条文档承诺的实现方式（`Transform3.hpp` 该函数的注释里写着）。写成取类标量的 `scaling(Scalar)` 会让 `Transform3::scaling(2)` 静默通过 —— 那是行为变更，不是成员化。类内不能复用 `Scalar` 这个名字，故模板参数改名 `Factor`。`Transform2T::scaling` 同理。
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

    // 逆走的是行列平衡 + 除法还原，这条路径不承诺精确舍入；本文件既有的
    // "inverse undoes the transform" 用的就是 margin(1e-12)。沿用同一强度 ——
    // 断言精确相等会把正常的浮点舍入当成缺陷，换个编译器/libm 就会碎。
    const auto inv = t.inverse();
    REQUIRE(inv.has_value());
    const Vector3 back = inv->apply(Vector3{11.0, 22.0, 33.0});
    CHECK(back.x == Approx(1.0).margin(1e-12));
    CHECK(back.y == Approx(2.0).margin(1e-12));
    CHECK(back.z == Approx(3.0).margin(1e-12));
}
```

（前三行用精确相等是对的：平移的分量相加、以及 `operator*` 只施加线性部分（平移的线性部分是恒等），都逐位精确。只有逆那一段要放 margin。）

- [ ] **Step 2: 运行，确认失败**

```bash
cmake --build --preset windows-vs-debug
```

Expected: 编译失败。

- [ ] **Step 3: 实现**

把 `identity_transform<S>()` / `translation_3d` / `scaling_3d` / `rotation_3d` / `apply(t,v)` / `inverse(t,tol)` 搬进 `TransformNT` 成为静态工厂或成员，删除旧自由函数。

**注意二维与三维不同构的地方：**

- `identity_transform<S>()` **只有三维有**。`Transform2T::identity()` 是**新建**的静态工厂，没有自由函数可搬（`Transform2.hpp` 目前用成员默认值 `MatrixT<Scalar, 3>::identity()`）。
- 旋转的签名本就不同：`rotation_3d(UnitVector3T, Scalar)` vs `rotation_2d(Scalar)`（二维没有转轴）。Interfaces 里的「同构」指的是命名，不是参数表。
- 两个带标量的重载（`scaling_3d(Scalar)`、`scaling_2d(Scalar)`）内部调用各自的向量版本，改写时是成员调用自家静态工厂。

**要一并改掉的调用点**（已全库扫描确认；阶段 2 已两次栽在漏掉调用点上，其中一次就是漏掉 examples）：

下表按「**每个文件用到哪些将被删除的自由名**」列出，不含计数 —— 计数会被 `using` 行污染，我前两次就是数错的。请以你自己 grep 的结果为准。

| 文件 | 用到的自由名 |
|---|---|
| `Transform2.hpp` | `scaling_2d`（`scaling_2d(Scalar)` 体内调 `scaling_2d(Vector2T)`） |
| `Transform3.hpp` | `scaling_3d`（`scaling_3d(Scalar)` 体内调 `scaling_3d(Vector3T)`） |
| `tests/linear/transform2_test.cpp` | `translation_2d`、`rotation_2d`、`scaling_2d`、`apply`、`inverse` |
| `tests/linear/transform3_test.cpp` | `identity_transform`、`translation_3d`、`rotation_3d`、`scaling_3d`、`apply`、`inverse` |
| `tests/linear/matrix_inverse_test.cpp` | `scaling_3d`、`translation_3d`、`apply`、`inverse` |
| `examples/transform_pipeline.cpp` | `translation_3d`、`rotation_3d`、`scaling_3d`、`apply`、`inverse` |
| `Matrix.hpp` | 第 119/124/127 行注释里对 `translation_3d(t)` / `scaling_3d(...)` 的具名引用 → 改指新静态工厂名 |

**别漏了 `inverse`。** 上面四个 `tests`/`examples` 文件里的 `inverse(...)` 调的是 **Transform 自己的**自由 `inverse`（`Transform2.hpp:92`、`Transform3.hpp:124`）—— 它**属于本任务**，要一并改成成员 `t.inverse(tol)`。这与 Task 2 的情况正好相反（那时它不归 Task 2 管），别把上一轮的边界记混。`matrix_inverse_test.cpp` 里 `inverse(scaling_3d(1e-4))` 这类会变成 `Transform3::scaling(1e-4).inverse()`。

**`matrix_inverse_test.cpp` 不在原 Files 列表里，但必须改** —— 它用 `scaling_3d`/`translation_3d` 构造变换来测 Matrix 的逆，这些自由函数一删它就编译不过。Task 2 的实现者已经踩过同一个坑。

**每个文件的 `using GeoCore::linear::<被删名字>;` 都要删掉** —— using 声明指向已不存在的名字是编译错误，不是警告。上一轮就有一处 `using GeoCore::linear::dot;` 属于这类。

改完自己跑这三条，不要只信这张表。**注意作用域是仓库根、不只是 `include/ examples/ tests/`** —— 我前两轮都把范围写小了，结果漏掉了 `ci/consumer/main.cpp`（见下）：

```bash
# 从仓库根扫，排除构建产物
grep -rn --exclude-dir=build --exclude-dir=.git --exclude-dir=docs \
  "\bidentity_transform\|\btranslation_3d\|\bscaling_3d\|\brotation_3d\|\btranslation_2d\|\bscaling_2d\|\brotation_2d" .
grep -rnE --exclude-dir=build --exclude-dir=.git --exclude-dir=docs \
  "(^|[^.[:alnum:]_:>])(apply|inverse)\(" .
grep -rn --exclude-dir=build --exclude-dir=.git --exclude-dir=docs \
  "using GeoCore::linear::" . | grep -E "apply|inverse|identity_transform|translation_|scaling_|rotation_"
```

预期：前两条只剩成员声明与注释文字（阶段 1 的历史计划文档在 `docs/` 里，已排除），第三条零命中。

- [ ] **Step 3b: 修正 `Transform3.hpp` 顶部那段已经失效的分层说明**

`Transform3.hpp:17-26` 的文档注释写着「Point3 属于 prim 层，linear 不得依赖它 …… 作用于 Point3 的运算符由 prim 层提供」。**本阶段已把 `Point` 移进 `linear`**（spec 决策 15–17），这段话现在两句都是错的。请改成：`operator*` 只施加线性部分（变换方向）、`apply` 施加完整仿射变换（把 `Vector` 读作位置）这个区分仍然成立，因此保留；但删去「Point3 属于 prim 层」与「由 prim 层提供」的说法，改为说明作用于 `Point3` 的成员（`transform_point`）在同一类型上一并提供，随 `Point3` 落地（Task 9）。

**不要**顺手把这段里关于「第 4 行假定为 (0,0,0,1)」的说明改掉 —— 那段仍然准确，且是 `apply`/`operator*` 不读第 4 行的依据。

- [ ] **Step 3c: 修 `ci/consumer/main.cpp`（Task 1 与 Task 3 两轮共同的欠账）**

`ci/consumer/` 是 `.github/workflows/ci.yml` 的 `consumer-smoke` 作业：install 到临时前缀后用 `find_package` 构建，专门验证 `/utf-8` 等 `INTERFACE_COMPILE_OPTIONS` 是否随安装包到达消费方（阶段 1 的 C1 就是这条防线抓到的）。**它现在编不过，该作业只能红**：

- 第 18 行 `dot(a, b)` — Task 1 删的自由函数
- 第 20 行 `normalize(Vector3{3.0, 4.0, 0.0})` — Task 1 删的
- 第 27 行 `translation_3d(...)` / `scaling_3d(2.0)` — 本任务删的
- 第 28 行 `apply(pipeline, ...)` — 本任务删的

改成成员/静态工厂形式：`a.dot(b)`、`Vector3{3.0, 4.0, 0.0}.normalized()`、`Transform3::translation(...)`、`Transform3::scaling(2.0)`、`pipeline.apply(...)`。

**三条硬约束，改的时候别踩：**

1. **不要给这个工程加任何编译选项，也不要改它的 CMakeLists。** 它刻意不设任何编码选项 —— 那正是它的测试内容。你若加了 `/utf-8`，这项测试就失去意义了。
2. **文件里的中文注释必须保留**（第 3–6 行）。它们同样是测试的一部分：能编过就证明 `/utf-8` 到了消费方。
3. 第 25 行的 `unit->x()` **不要改** —— `UnitVectorNT` 的分量本来就是访问器方法，那里是对的（与 `VectorNT` 的数据成员不同）。

本任务改完后，`include/`、`examples/`、`tests/`、`ci/` 里应再无对已删名字的引用。

> **已确认的防线缺口 —— 留待用户决策，不在本任务内改**
>
> **结论：`consumer-smoke` 目前守不住「`/utf-8` 随 `INTERFACE_COMPILE_OPTIONS` 导出」这条回归。**这不是推测，是端到端实测：重审者把安装好的 `GeoCoreTargets.cmake` 里 `INTERFACE_COMPILE_OPTIONS` 那一行**摘掉**（`/utf-8` 的唯一来源），再完整按 `ci.yml` 的流程跑 configure → build → run，**三步全 exit 0，输出 `dot = 32 / unit = (0.6, 0.8, 0)` 全部正确**。
>
> 机制（从 MSBuild 实际命令行读到）：`CL.exe ... /W1 /WX- ... /external:W0 /external:I "<prefix>/include" /utf-8`。`/external:I` 把安装前缀标为外部头、`/external:W0` 把外部头的警告压到 0 级（手工 `/W4` 编译时能看到的那 13 条头文件 `C4819` 在 CI 配置下根本不出现），`/WX-` 表示警告永不致失败，而作业只取退出码、不读警告。**这一点与 runner 的代码页无关** —— 即便 GitHub 的 cp1252 一个 `C4819` 都不报，结果也一样。
>
> 正对照（导出完好）零诊断，`vcxproj` 的 `AdditionalOptions` 里确实有 `/utf-8` —— 所以**被测对象本身是好的，坏的是这道检验的强度**。这正是阶段 1 C1 依赖的那道防线。
>
> **建议的补法（一行，实测有效）：给 `ci/consumer` 加 `/WX`。** 它不设置任何**编码**选项，因此「本工程刻意不设编码选项，诊断只能指向 GeoCore 的头文件」这个前提完好；而 `C4819` 会变成硬失败（实测 `/W1 /WX` 缺 `/utf-8` 时 exit 2）。**这不属于本计划任何任务，请先与用户确认再动。**

- [ ] **Step 3d: 补 `Transform2T::identity()` 的覆盖**

`Transform2T::identity()` 是本任务**新建**的工厂（二维原本没有 `identity_transform` 自由函数可搬），也是这批工厂里唯一零覆盖的一个。在 `tests/linear/transform2_test.cpp` 里补一条，照 3D 那条的写法：

```cpp
TEST_CASE("Transform2 identity leaves a position unchanged", "[linear][transform2]") {
    const Transform2 unit = Transform2::identity();

    CHECK(unit.apply(Vector2{1.0, 2.0}) == Vector2{1.0, 2.0});
}
```

**注意**：`apply` 作用在恒等变换上时「原样返回」与「真的做了一次矩阵乘法」数值相同，所以这一条**只**能证明工厂存在且不改变位置，钉不住实现细节 —— 这是可接受的（恒等阵的定义就是如此），不要为了"加强"它去编造别的期望值。

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

#include <array>
#include <type_traits>

#include <GeoCore/linear/Point3.hpp>

using Catch::Approx;
using GeoCore::linear::Point3;
using GeoCore::linear::Vector3;

namespace {
/// 「能否相加」这个判断必须包在概念里，见下面 static_assert 处的说明。
template <typename T>
concept addable = requires(T p, T q) { p + q; };
}

TEST_CASE("Point3 supports subscript and array export", "[linear][point3]") {
    const Point3 p{1.0, 2.0, 3.0};

    CHECK(p[0] == 1.0);
    CHECK(p[1] == 2.0);
    CHECK(p[2] == 3.0);

    // 三个槽位**逐个**查。只查末槽是不够的：`{y, x, z}`、`{0, 0, z}` 这类
    // 错位实现都能通过（已用变异测试实测存活）。to_array() 是公开的互操作
    // 接口（喂给外部库 / GPU buffer），槽位错位就是静默的坐标错乱。
    // 同目录的 vector3_test.cpp 正是三个下标全查的，这里与它对齐。
    const std::array<double, 3> arr = p.to_array();
    CHECK(arr[0] == 1.0);
    CHECK(arr[1] == 2.0);
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

    // 两个点只要有一个分量不同就必须不相等。**这一条不能省** ——
    // 上面所有期望值的 x 分量都恰好与左操作数相同（11、-9 都是从 a.x=1 算出来的），
    // 于是一个「只比较 x」的 operator== 会让本文件全部断言静默通过（已用变异测试
    // 实测：把 == 改成只比 x，21 条断言全过）。
    CHECK(Point3{1.0, 2.0, 3.0} != Point3{1.0, 9.0, 3.0});
    CHECK(Point3{1.0, 2.0, 3.0} != Point3{1.0, 2.0, 9.0});
    CHECK(Point3{1.0, 2.0, 3.0} == Point3{1.0, 2.0, 3.0});

    // 点 + 点不存在 —— 这是本任务的核心不变量（Review Focus 第 1 条）：
    // 写错了不会报错，只会静默给出错误语义。必须真的断言，不能只注释掉。
    //
    // 不能直接写 `static_assert(!requires(Point3 p, Point3 q) { p + q; });`
    // —— 那不是「求值为假」，而是**非良构的 C++**。requirements 里的非法表达式
    // 只有在「替换模板实参」的语境下才求值为 false；`Point3` 是具体类型，
    // `p + q` 不依赖任何模板参数，因此是硬错误（MSVC 报 C2676，编译器不会
    // 给你 false）。必须包一层概念，让失败发生在替换上下文里 —— 见文件顶部
    // 匿名命名空间的 `addable`。这一点与 Catch2 无关，也与用不用
    // STATIC_REQUIRE 无关。
    static_assert(!addable<Point3>, "Point + Point must not be well-formed");
}

TEST_CASE("Point3 distance_to", "[linear][point3]") {
    CHECK(Point3{0.0, 0.0, 0.0}.distance_to(Point3{3.0, 4.0, 0.0}) == Approx(5.0));

    // 上一条的 Δz 是 0，一个「只算 x/y」的实现照样通过（已用变异测试实测：
    // 丢掉 z 之后本文件 11 条断言全过）。补一条三个分量都非零的，
    // 并把 z 单独钉一次。
    CHECK(Point3{0.0, 0.0, 0.0}.distance_to(Point3{0.0, 0.0, 5.0}) == Approx(5.0));
    CHECK(Point3{1.0, 2.0, 3.0}.distance_to(Point3{2.0, 4.0, 5.0}) == Approx(3.0));
}
```

创建 `tests/linear/point2_test.cpp`。二维的结构与三维平行，**同一条不变量也必须在这里断言一次** —— 别因为"3D 测过了"就省掉，两个头文件是独立写的，一个写错另一个不会知道：

```cpp
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <array>
#include <type_traits>

#include <GeoCore/linear/Point2.hpp>

using Catch::Approx;
using GeoCore::linear::Point2;
using GeoCore::linear::Vector2;

namespace {
/// 同 point3_test.cpp：判断「能否相加」必须包在概念里。
template <typename T>
concept addable = requires(T p, T q) { p + q; };
}

TEST_CASE("Point2 supports subscript and array export", "[linear][point2]") {
    const Point2 p{1.0, 2.0};

    CHECK(p[0] == 1.0);
    CHECK(p[1] == 2.0);

    // 同 point3_test.cpp：两个槽位逐个查，只查末槽会让 `{0, y}` 这类
    // 错位实现静默通过。
    const std::array<double, 2> arr = p.to_array();
    CHECK(arr[0] == 1.0);
    CHECK(arr[1] == 2.0);
}

TEST_CASE("Point2 arithmetic follows the same affine rules as Point3",
          "[linear][point2]") {
    const Point2 a{1.0, 2.0};
    const Point2 b{4.0, 6.0};
    const Vector2 v{10.0, 20.0};

    STATIC_REQUIRE(std::is_same_v<decltype(a + v), Point2>);
    CHECK(a + v == Point2{11.0, 22.0});

    STATIC_REQUIRE(std::is_same_v<decltype(a - v), Point2>);
    CHECK(a - v == Point2{-9.0, -18.0});

    STATIC_REQUIRE(std::is_same_v<decltype(b - a), Vector2>);
    CHECK(b - a == Vector2{3.0, 4.0});

    // 同 point3_test.cpp：== 的 y 分量必须有独立证据，否则一个「只比较 x」
    // 的实现会让本文件全部断言静默通过。
    CHECK(Point2{1.0, 2.0} != Point2{1.0, 9.0});
    CHECK(Point2{1.0, 2.0} == Point2{1.0, 2.0});

    static_assert(!addable<Point2>, "Point + Point must not be well-formed");
}

TEST_CASE("Point2 distance_to", "[linear][point2]") {
    CHECK(Point2{0.0, 0.0}.distance_to(Point2{3.0, 4.0}) == Approx(5.0));
}
```

- [ ] **Step 2: 运行，确认失败**

```bash
cmake --build --preset windows-vs-debug
```

Expected: 找不到 `GeoCore/linear/Point2.hpp` 与 `Point3.hpp`。

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

/// 逐分量比较。与 Vector 的约定一致：`operator!=` 由 C++20 自动生成，不手写。
template <typename Scalar>
[[nodiscard]] constexpr bool operator==(Point3T<Scalar> a, Point3T<Scalar> b) noexcept;
```

`Point2T` 的三个运算符与 `operator==` 照写一遍（把 `3` 换成 `2`、`z` 去掉）。

**关于 `Point + Point`**：代码块里刻意没有它，`static_assert` 会守住这一点。**也不要顺手加 `Vector + Point`** —— spec 的运算符清单（**§4.4**，不是 §5.0；§5.0 只是能力清单）里没有它，计划不擅自扩 API。若你实现时觉得它显然应该存在，那是**设计问题不是实现问题**，请在报告里提出来，别默默加上。

（顺带说明：§4.4 的片段只列了 `Point3 + Vector3` 与 `Point3 - Point3` 两条，本计划的 Interfaces 多了 `Point - Vector` —— 那是计划有意加的，实现跟随 Interfaces。`Transform3T * Point3T` 将在 Task 9 加，届时应回填进 §4.4。）

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
  - `bool operator==(IntervalT, IntervalT) noexcept`（自由函数，与 `Vector`/`Point` 的约定一致；`!=` 由 C++20 生成）

**空区间的规范表示（本任务的关键约定）。** 空区间一律用 `[+inf, -inf]` 表示，且**凡是产出空区间的运算都必须给出这个规范形式**，不能给 `[2, 0]` 这种「倒置但不规范」的区间。理由是 spec 决策 17 那条原则的延伸：同一个数学量不该有两种语义 —— 若 `is_empty()` 接受两种表示，那么 `==`、`length()`、`center()` 都会变成「看情况」，调用者无从判断。`is_empty()` 仍然实现为 `min > max`（对两种表示都安全），但**生产者一律规范化**。

**两个哨兵区间的 `length()` 与 `center()` 必须定义**，否则它们会静默给出非有限值：

- `length()`：`is_empty() ? 0 : max - min`。空区间的原始差是 `-inf - (+inf) = -inf`，一个没有任何意义的「长度」，还会一路传播下去；空集的测度是 0。无界区间（含半无界）的原始差本来就是 `+inf`，正确。
- `center()`：用 `min * 0.5 + max * 0.5` 实现，**不要**用 `(min + max) * 0.5` 或 `min + (max - min) * 0.5` —— 前者在 `[1e308, 1e308]` 上溢出成 `+inf`，后者在 `[-1e308, 1e308]` 上溢出成 `+inf`，而 `min*0.5 + max*0.5` 两种都正确。任一端点无穷时它自然是 `NaN`（`(+inf) + (-inf)`），这正是「没有中点」的诚实答案：**文档里写明这是哨兵值、并配测试钉住**，不要为了「避免 NaN」而返回某个看似成功的数字。

- [ ] **Step 1: 写失败测试**

创建 `tests/linear/interval_test.cpp`（需要 `<cmath>` 与 `<limits>`）：

```cpp
TEST_CASE("merging with the empty interval is the identity",
          "[linear][interval]") {
    const Interval a{1.0, 5.0};
    CHECK(a.merged(Interval::empty()) == a);
    CHECK(Interval::empty().merged(a) == a);
    CHECK(Interval::empty().merged(Interval::empty()) == Interval::empty());
}

TEST_CASE("a degenerate interval contains exactly one point",
          "[linear][interval]") {
    const Interval point{3.0, 3.0};

    CHECK_FALSE(point.is_empty());
    CHECK(point.contains(3.0));
    CHECK_FALSE(point.contains(3.0 + 1e-15));
    CHECK(point.length() == 0.0);
}

TEST_CASE("contains is closed at both ends", "[linear][interval]") {
    const Interval a{1.0, 5.0};

    CHECK(a.contains(1.0));
    CHECK(a.contains(5.0));
    CHECK(a.contains(3.0));
    CHECK_FALSE(a.contains(0.999));
    CHECK_FALSE(a.contains(5.001));
}

TEST_CASE("length and center are defined on the sentinel intervals",
          "[linear][interval]") {
    // 空区间的测度是 0；max - min 会给出 -inf，那是没有意义的长度。
    CHECK(Interval::empty().length() == 0.0);

    // 无界区间的测度确实是 +inf。
    CHECK(Interval::unbounded().length() ==
          std::numeric_limits<double>::infinity());

    CHECK(Interval{1.0, 5.0}.length() == 4.0);
    CHECK(Interval{1.0, 5.0}.center() == 3.0);

    // center 用 min*0.5 + max*0.5：这两种极端值都能算对，
    // (min+max)*0.5 与 min+(max-min)*0.5 各会在其中一个上溢出。
    CHECK(Interval{1e308, 1e308}.center() == 1e308);
    CHECK(Interval{-1e308, 1e308}.center() == 0.0);

    // 端点无穷时没有「中点」：(+inf) + (-inf) 是 NaN。这是刻意的哨兵，
    // 不是漏判 —— 调用者应先 is_empty() 或判端点有限性。
    CHECK(std::isnan(Interval::empty().center()));
    CHECK(std::isnan(Interval::unbounded().center()));
}

TEST_CASE("intersects and clipped agree, and clipped canonicalises",
          "[linear][interval]") {
    const Interval a{1.0, 5.0};
    const Interval b{4.0, 8.0};
    const Interval disjoint{6.0, 9.0};

    CHECK(a.intersects(b));
    CHECK_FALSE(a.intersects(disjoint));
    CHECK(a.clipped(b) == Interval{4.0, 5.0});

    // 不相交时必须是规范空区间，不能是 [6.0, 5.0] 那种倒置形式 ——
    // 后者 is_empty() 也为真，但 min/max 携带的是错误信息。
    CHECK(a.clipped(disjoint) == Interval::empty());
    CHECK_FALSE(a.clipped(disjoint).intersects(a));
}

TEST_CASE("empty intervals stay empty under intersects and clipped",
          "[linear][interval]") {
    const Interval a{1.0, 5.0};

    CHECK_FALSE(Interval::empty().intersects(a));
    CHECK_FALSE(a.intersects(Interval::empty()));
    CHECK(Interval::empty().clipped(a) == Interval::empty());
    CHECK(a.clipped(Interval::empty()) == Interval::empty());
    CHECK(Interval::empty().clipped(Interval::empty()) == Interval::empty());
}

TEST_CASE("expanded grows both ends, and a negative amount shrinks",
          "[linear][interval]") {
    CHECK(Interval{1.0, 5.0}.expanded(2.0) == Interval{-1.0, 7.0});
    CHECK(Interval{1.0, 5.0}.expanded(-1.0) == Interval{2.0, 4.0});

    // 收缩过头得到空区间 —— 同样是规范形式，不是 [4.0, 2.0]。
    CHECK(Interval{1.0, 5.0}.expanded(-3.0) == Interval::empty());

    // 空区间膨胀后仍是空区间，不会变成 [-inf, +inf]。
    CHECK(Interval::empty().expanded(1.0) == Interval::empty());

    // 无界区间膨胀后仍然无界。
    CHECK(Interval::unbounded().expanded(1.0) == Interval::unbounded());
}
```

- [ ] **Step 2-4: 实现、验证、提交**

实现要点：

- `contains` 用**闭区间**（`value >= min && value <= max`），不引入容差 —— 区间包含是精确谓词，带容差的包含会让「这个点是否在区间内」随上下文变化。
- `is_empty()` 是 `min > max`；`empty()` 是 `{+inf, -inf}`，`unbounded()` 是 `{-inf, +inf}`。
- `merged` 是 `{min(a.min,b.min), max(a.max,b.max)}` —— 对空区间自然成立，无需特判（这正是选这个空表示的理由）。
- `intersects` 是 `max(min) <= min(max)`（闭区间，端点相接算相交），**返回 bool 而非区间**。空区间无需特判：`[+inf,-inf]` 会把 `max(min)` 顶到 `+inf`、把 `min(max)` 压到 `-inf`，比较必然为假 —— 这正是选这个空表示的理由。`clipped` 则不同，它必须显式判空并规范化。
- `clipped` 取交；结果若为空则**返回 `Interval::empty()`**。
- `expanded(k)` 是 `{min - k, max + k}`；结果若倒置则**返回 `Interval::empty()`**；空区间膨胀后仍是 `Interval::empty()`（**不要**退化成 `unbounded()`）。
- `length()` 与 `center()` 按上面 Interfaces 里的规则实现。

**别在实现里硬编码容差阈值** —— `Interval` 的谓词全部是精确比较，本类型不需要 `Tolerance` 参数（与 `Box` 不同，盒的构造可能需要容差来判退化，那在 Task 6 处理）。

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
  - `static constexpr Box3T empty() noexcept`；`static Box3T from_corners(Point3T, Point3T)`（★ `from_points(span<const Point3T>)` 留待后续阶段）
  - `bool is_empty() const`、`bool contains(Point3T) const`、`bool contains(Box3T) const`、`bool intersects(Box3T) const`
  - `Point3T center() const`、`Vector3T extent() const`、`Vector3T half_extent() const`
  - `Box3T merged(Box3T) const`、`Box3T expanded(Scalar) const`
  - `Point3T corner(int index) const` —— 8 个角，索引位含义文档化
  - `bool operator==(Box3T, Box3T) noexcept`（自由函数；`!=` 由 C++20 生成）

**空盒语义（与 Task 5 的 `Interval` 完全对齐，两处不得各行其是）：**

- **规范表示**：`min` 的每个分量 `+inf`，`max` 的每个分量 `-inf`。**凡是产出空盒的运算都必须给出这个规范形式**，不能给「只有 x 分量倒置」的盒 —— 理由同 `Interval`：一个量两种表示会让 `==`、`extent()`、`center()` 全部变成「看情况」。
- **`extent()`**：空盒返回 `Vector3{0,0,0}`。原始差是 `-inf - (+inf) = -inf`，那是没有意义的「尺寸」还会静默传播；空集的测度是 0。
- **`center()`**：用 `min * 0.5 + max * 0.5` 逐分量实现（不是 `(min+max)*0.5`，那是 Interval 那边已经裁定的溢出陷阱）。空盒与任一无穷端点都自然得到 NaN —— 这是「没有中心」的诚实答案，文档写明并配测试。
  **注意这里不能直写 `min * 0.5 + max * 0.5`** —— `min`/`max` 是 `Point3T`，而 Task 4 刻意**没有**给 `Point` 提供 `operator*(Point, Scalar)`，也**没有** `Point + Point`（`p * 2.0` 与 `-p` 都编译不过，已实测）。必须写成逐分量的标量运算：`min.x * 0.5 + max.x * 0.5` 等等。`extent()` 的 `max - min` 则可以直写（`Point - Point -> Vector` 已提供）。
- **`contains(Box3T)`：任一方为空盒时返回 `false`。** 数学上 `∅ ⊆ B` 是空真，但那会让 `if (a.contains(b))` 在 `b` 为空时通过，是每个调用者都会踩的坑；`intersects` 已经采用「空盒与任何盒都不相交」，`contains` 保持同向。**这是刻意的约定，必须在文档里写明理由**，否则下一个人会把它当 bug 改掉。
- **`contains(Point3T)` 与 `intersects` 用闭区间**（边界算在内），且都无需对空盒特判：`+inf <= x` 与 `x <= -inf` 必然为假，比较自然失败 —— 这正是选这个表示的理由。

- [ ] **Step 1: 写失败测试**

创建 `tests/linear/box3_test.cpp`。需要 `<catch2/catch_approx.hpp>`、`<catch2/catch_test_macros.hpp>`、`<cmath>`（`std::isnan`）。若你实现 `empty()` 时要用 `std::numeric_limits`，头文件里还得有 `<limits>`：

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
    CHECK(empty.merged(empty) == empty);
}

TEST_CASE("a degenerate box (a point) is not empty", "[linear][box3]") {
    const Box3 point{Point3{1.0, 2.0, 3.0}, Point3{1.0, 2.0, 3.0}};

    CHECK_FALSE(point.is_empty());
    CHECK(point.contains(Point3{1.0, 2.0, 3.0}));
    CHECK(point.extent() == Vector3{0.0, 0.0, 0.0});
}

TEST_CASE("box corners pin every index bit", "[linear][box3]") {
    const Box3 box{Point3{0.0, 0.0, 0.0}, Point3{1.0, 2.0, 4.0}};

    // 索引的 bit0/bit1/bit2 依次选择 x/y/z 取 min 还是 max，置位取 max。
    // 只测 0 与 7 是不够的：那两端恰好是「全 min」与「全 max」，三个轴的
    // 位若被两两互换，0 和 7 仍然都对得上。中间三个索引各只置一位，
    // 把每一位独立钉死。
    CHECK(box.corner(0) == Point3{0.0, 0.0, 0.0});
    CHECK(box.corner(1) == Point3{1.0, 0.0, 0.0});
    CHECK(box.corner(2) == Point3{0.0, 2.0, 0.0});
    CHECK(box.corner(4) == Point3{0.0, 0.0, 4.0});
    CHECK(box.corner(7) == Point3{1.0, 2.0, 4.0});

    CHECK(box.center() == Point3{0.5, 1.0, 2.0});
    CHECK(box.half_extent() == Vector3{0.5, 1.0, 2.0});
    CHECK(box.extent() == Vector3{1.0, 2.0, 4.0});
}

TEST_CASE("box overlap is closed and empty boxes intersect nothing",
          "[linear][box3]") {
    const Box3 a{Point3{0.0, 0.0, 0.0}, Point3{2.0, 2.0, 2.0}};
    const Box3 touching{Point3{2.0, 0.0, 0.0}, Point3{4.0, 2.0, 2.0}};
    const Box3 apart{Point3{3.0, 0.0, 0.0}, Point3{4.0, 2.0, 2.0}};

    CHECK(a.intersects(touching));            // 共享一个面，算相交
    CHECK_FALSE(a.intersects(apart));
    CHECK(a.contains(Box3{Point3{0.5, 0.5, 0.5}, Point3{1.5, 1.5, 1.5}}));
    CHECK_FALSE(a.contains(touching));

    // 空盒与任何盒都不相交，也不包含任何盒，且不被任何盒包含。
    CHECK_FALSE(Box3::empty().intersects(a));
    CHECK_FALSE(a.intersects(Box3::empty()));
    CHECK_FALSE(a.contains(Box3::empty()));
    CHECK_FALSE(Box3::empty().contains(a));
}

TEST_CASE("box extent and center are defined on the empty box",
          "[linear][box3]") {
    // 空盒的测度是 0，不是 -inf。
    CHECK(Box3::empty().extent() == Vector3{0.0, 0.0, 0.0});

    // 没有中心：(+inf) + (-inf) 逐分量为 NaN。刻意如此，不是漏判。
    CHECK(std::isnan(Box3::empty().center().x));
    CHECK(std::isnan(Box3::empty().center().y));
    CHECK(std::isnan(Box3::empty().center().z));

    // center 用 min*0.5 + max*0.5，这两种极端值都能算对。
    CHECK(Box3{Point3{1e308, 0.0, 0.0}, Point3{1e308, 0.0, 0.0}}.center().x == 1e308);
    CHECK(Box3{Point3{-1e308, 0.0, 0.0}, Point3{1e308, 0.0, 0.0}}.center().x == 0.0);
}

TEST_CASE("merged and expanded keep the canonical empty box",
          "[linear][box3]") {
    const Box3 a{Point3{0.0, 0.0, 0.0}, Point3{1.0, 1.0, 1.0}};
    const Box3 b{Point3{2.0, 2.0, 2.0}, Point3{3.0, 3.0, 3.0}};

    CHECK(a.merged(b) == Box3{Point3{0.0, 0.0, 0.0}, Point3{3.0, 3.0, 3.0}});

    CHECK(a.expanded(1.0) == Box3{Point3{-1.0, -1.0, -1.0}, Point3{2.0, 2.0, 2.0}});

    // 收缩过头得到规范空盒，不是「分量倒置但非规范」的形式。
    CHECK(a.expanded(-2.0) == Box3::empty());

    // 空盒膨胀后仍是空盒，不会变成整个空间。
    CHECK(Box3::empty().expanded(1.0) == Box3::empty());
}

TEST_CASE("from_corners accepts the two corners in either order",
          "[linear][box3]") {
    const Box3 forward = Box3::from_corners(Point3{0.0, 0.0, 0.0}, Point3{1.0, 2.0, 4.0});
    const Box3 reversed = Box3::from_corners(Point3{1.0, 2.0, 4.0}, Point3{0.0, 0.0, 0.0});

    CHECK(forward == reversed);
    CHECK(forward.min == Point3{0.0, 0.0, 0.0});
    CHECK(forward.max == Point3{1.0, 2.0, 4.0});
}
```

创建 `tests/linear/box2_test.cpp`。二维**不共享**三维的测试文件，同一条不变量在这里要独立断言一次 —— 两个头文件是分开写的，3D 对了不代表 2D 也对。把上面 `box3_test.cpp` 的用例按二维改写一遍：`Point2`/`Vector2`/`Box2`，`corner` 只有 4 个（索引的 bit0/bit1 选 x/y），去掉 z 分量，非均匀盒用 `(1, 2)` 这样的边长以便暴露轴位互换，`extent`/`center`/`expanded`/`from_corners`/空盒的规则完全照搬。**`center` 的溢出用例保留**（`{1e308,0}`–`{1e308,0}` 与 `{-1e308,0}`–`{1e308,0}`）。

- [ ] **Step 2: 实现**

**空盒的确切表示（Task 8 依赖它，不得改动）**：`min` 的每个分量为 `+inf`，`max` 的每个分量为 `-inf`。这个选择使 `merged` 与后续 Task 8 的「从空盒开始取 min/max」自然成立，无需对空盒特判：

```cpp
template <typename Scalar>
[[nodiscard]] constexpr Box3T<Scalar> Box3T<Scalar>::empty() noexcept {
    constexpr Scalar inf = std::numeric_limits<Scalar>::infinity();
    return Box3T<Scalar>{Point3T<Scalar>{inf, inf, inf}, Point3T<Scalar>{-inf, -inf, -inf}};
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

其余成员：

```cpp
/// 由两个角点构造，**接受任意顺序** —— 名字说的是「角点」不是「min/max」，
/// 调用方不该被迫先自己排一遍。逐分量取 min / max。
template <typename Scalar>
[[nodiscard]] constexpr Box3T<Scalar> from_corners(Point3T<Scalar> a, Point3T<Scalar> b) noexcept;

[[nodiscard]] constexpr bool is_empty() const noexcept {
    return min.x > max.x || min.y > max.y || min.z > max.z;
}

/// 空盒返回零向量：空集的测度是 0，而 max - min 会给出 -inf。
[[nodiscard]] constexpr Vector3T<Scalar> extent() const noexcept {
    if (is_empty()) {
        return Vector3T<Scalar>{};
    }
    return max - min;
}

/// 逐分量 min*0.5 + max*0.5 —— 不是 (min+max)*0.5（在 [1e308,1e308] 上溢出），
/// 也不是 min+(max-min)*0.5（在 [-1e308,1e308] 上溢出）。端点无穷时自然得到 NaN。
[[nodiscard]] constexpr Point3T<Scalar> center() const noexcept;

/// 向两侧各扩 k。收缩过头（结果倒置）时返回规范空盒，
/// 空盒膨胀后仍是空盒（**不要**退化成整个空间）。
[[nodiscard]] constexpr Box3T<Scalar> expanded(Scalar amount) const noexcept;
```

**`contains(Box3T)` 里「任一方为空盒返回 `false`」必须显式写出来**，不能指望朴素比较：

```cpp
[[nodiscard]] constexpr bool contains(Box3T<Scalar> other) const noexcept {
    if (is_empty() || other.is_empty()) {
        return false;   // 见 Interfaces 里的理由，这是刻意的约定
    }
    return min.x <= other.min.x && max.x >= other.max.x
        && min.y <= other.min.y && max.y >= other.max.y
        && min.z <= other.min.z && max.z >= other.max.z;
}
```

若不判空，`a.contains(Box3::empty())` 会因为 `min.x <= +inf` 与 `max.x >= -inf` 恒真而**返回 true** —— 那是空真，是每个调用者都会踩的坑。

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
  - `static std::optional<Coordinate3T> from_transform(const Transform3T&, Tolerance = {})` —— **不是 `noexcept` 返回裸值**，理由见下
  - `Point3T origin() const`、`UnitVector3T x_axis() const` / `y_axis()` / `z_axis()`
  - `Point3T to_parent(Point3T local) const`、`Point3T to_local(Point3T parent) const`
  - `Vector3T to_parent(Vector3T local) const`、`Vector3T to_local(Vector3T parent) const`
  - `Coordinate2T` 同构，但有两处必然不同：
    - `static std::optional<Coordinate2T> from_axes(Point2T, UnitVector2T x, UnitVector2T y, Tolerance = {})` —— 只有两轴，校验 `x·y ≈ 0` 且二维叉积（`x.x*y.y - x.y*y.x`）为正（右手/逆时针）
    - `static Coordinate2T from_x_axis(Point2T, UnitVector2T x) noexcept` —— **取代三维的 `from_z_axis`**。二维没有第三轴，x 是主轴，`y` 就是 `x` 逆时针转 90°：`(-x.y, x.x)`，无需参考向量、也不存在退化，因此可以 `noexcept` 返回裸值

**`from_transform` 必须是 `optional`。** 计划原先写的是 `noexcept` 返回裸值，理由是「线性部分须为正交（由 `Transform` 的工厂保证）」—— **这句话是假的**。`Transform3T` 的工厂里有 `scaling(Vector3{2,3,4})`，它显然不正交；而 `translation * rotation * scaling` 这样的复合更是把非正交直接喂进来。若 `from_transform` 不校验，它就是一条**公开的、能构造出非正交坐标系的路径** —— 正好是本任务开头那句「必须无法通过公开接口构造出来」要禁止的事，也正好是 Review Focus 第 2 条点名的失败模式（静默拉伸几何）。所以它和另外两个工厂一样返回 `optional`。

`from_transform` 的语义：原点取 `transform` 作用于局部原点；**线性部分必须本身是正交矩阵**（即变换是刚体变换），否则返回 `nullopt`。

**判据是「线性部分本身正交」，不是「三列各自归一化之后再判它们两两正交」。** 后者有个隐蔽的洞：`diag(2,3,4)` 与 `diag(2,2,2)` 归一化之后都变成标准基，看起来完全正交 —— 非均匀缩放会静默通过，而它恰恰是最该被拒绝的那种（会把几何拉伸）。实现上直接检查 `AᵀA ≈ I`（用容差），通过之后再取三列构造轴即可，此时三列本来就已经是单位向量。

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

TEST_CASE("from_z_axis produces a right-handed orthonormal frame",
          "[linear][coordinate3]") {
    // 上面那条往返用例的 z 是世界 z 轴，补全出来的恰好是标准基 ——
    // 于是「旋转矩阵转置了」「左手系」这两种实现都能通过它。这一条改用
    // 非轴向的 z，并把补全出来的轴逐分量钉死。
    //
    // 期望值是手算的：z = (1,1,1)/√3 时三个分量的绝对值并列，选择器取
    // x 方向作参考向量，得 x = (0,-1,1)/√2、y = (2,-1,-1)/√6。
    const auto z = Vector3{1.0, 1.0, 1.0}.normalized();
    REQUIRE(z.has_value());

    const auto frame = Coordinate3::from_z_axis(Point3{0.0, 0.0, 0.0}, *z);
    REQUIRE(frame.has_value());

    const double s2 = std::sqrt(2.0);
    const double s3 = std::sqrt(3.0);
    const double s6 = std::sqrt(6.0);

    CHECK(frame->x_axis().x() == Approx(0.0).margin(1e-15));
    CHECK(frame->x_axis().y() == Approx(-1.0 / s2));
    CHECK(frame->x_axis().z() == Approx(1.0 / s2));

    CHECK(frame->y_axis().x() == Approx(2.0 / s6));
    CHECK(frame->y_axis().y() == Approx(-1.0 / s6));
    CHECK(frame->y_axis().z() == Approx(-1.0 / s6));

    CHECK(frame->z_axis().x() == Approx(1.0 / s3));
    CHECK(frame->z_axis().y() == Approx(1.0 / s3));
    CHECK(frame->z_axis().z() == Approx(1.0 / s3));

    // 直接钉住定义性质，不依赖上面那组手算值。
    const Vector3 xy = frame->x_axis().as_vector().cross(frame->y_axis().as_vector());
    CHECK(xy.x == Approx(frame->z_axis().x()));
    CHECK(xy.y == Approx(frame->z_axis().y()));
    CHECK(xy.z == Approx(frame->z_axis().z()));
}

TEST_CASE("only a rigid transform is a coordinate frame",
          "[linear][coordinate3][degenerate]") {
    // from_transform 若返回裸值，这里就会静默得到一个会拉伸几何的「坐标系」。
    CHECK_FALSE(Coordinate3::from_transform(
                    Transform3::scaling(Vector3{2.0, 3.0, 4.0})).has_value());

    // 均匀缩放同样不是标架 —— 注意不能靠「先把三列归一化再校验」来判：
    // diag(2,3,4) 与 diag(2,2,2) 归一化之后都变成标准基，看起来完全正交，
    // 于是非均匀缩放会静默通过。判据必须是线性部分**本身**正交，而不是
    // 归一化之后的三个方向正交。
    CHECK_FALSE(Coordinate3::from_transform(Transform3::scaling(2.0)).has_value());

    // 刚体变换才是标架。
    const auto rigid = Coordinate3::from_transform(
        Transform3::translation(Vector3{1.0, 2.0, 3.0})
        * Transform3::rotation(z_axis, half_pi));
    REQUIRE(rigid.has_value());
    CHECK(rigid->origin() == Point3{1.0, 2.0, 3.0});

    CHECK(Coordinate3::from_transform(Transform3::identity()).has_value());
}
```

测试文件 `tests/linear/coordinate3_test.cpp` 需要 `<cmath>`（`std::sqrt`）、`<catch2/catch_approx.hpp` 与 `using Catch::Approx;`、`GeoCore/core/Constants.hpp`（`half_pi`），并照 `transform3_test.cpp` 的写法在匿名命名空间里放一个 `z_axis` 常量。

创建 `tests/linear/coordinate2_test.cpp`。二维**不共享**三维的测试文件 —— 两个头文件分开写，3D 对了不代表 2D 也对。按二维改写：`Point2`/`Vector2`/`Coordinate2`/`UnitVector2`；`from_axes` 只有两个轴（校验两轴正交且 `x × y` 的**标量叉积为正**，二维没有第三个轴可比）；`from_z_axis` 改为 `from_x_axis(origin, x)`（`y` 就是 `x` 逆时针转 90°）。**非正交拒绝、左手系拒绝、往返、非轴向 y 的期望轴值、`from_transform` 的四条判据**都要在二维各来一份。

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

    const auto x = reference.cross(z.as_vector()).normalized(tolerance);
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
  - `bool contains(Point3T, Tolerance = {}) const` —— 先把点 `to_local`，再逐轴比较 `|local.i| <= half_extent.i`，**闭区间**（边界算在内），容差加在比较的右侧
  - `Point3T corner(int index) const` —— 8 个角，索引约定与 `Box3T::corner` **完全一致**（bit0/bit1/bit2 依次选 x/y/z）
  - **`Box3T to_axis_aligned() const`** —— 紧致地包住旋转后的盒子
  - `OrientedBox3T expanded(Scalar) const` —— **逐分量把半轴夹到非负**：`max(0, half_extent.i + amount)`。负的半轴不是「有向盒」的合法状态，放它过去会让 `to_axis_aligned()` 给出一个倒置的、悄悄错的盒子
  - `bool operator==(OrientedBox3T, OrientedBox3T) noexcept`（自由函数；`!=` 由 C++20 生成）
  - `OrientedBox2T` 同构，`corner` 有 4 个，二维的 `from_axes` 取两轴

- [ ] **Step 1: 写失败测试**

创建 `tests/linear/oriented_box3_test.cpp`（需要 `<cmath>` 取 `std::sqrt`、`<catch2/catch_approx.hpp>` + `using Catch::Approx;`，并照其它测试放一个匿名命名空间的 `z_axis`）。

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
    const auto x45 = Vector3{1.0, 1.0, 0.0}.normalized();
    const auto y45 = Vector3{-1.0, 1.0, 0.0}.normalized();
    REQUIRE(x45.has_value());
    REQUIRE(y45.has_value());

    const auto rotated_frame = Coordinate3::from_axes(
        Point3{0.0, 0.0, 0.0}, *x45, *y45, z_axis);
    REQUIRE(rotated_frame.has_value());

    const OrientedBox3 rotated{*rotated_frame, Vector3{1.0, 1.0, 1.0}};
    const Box3 rotated_aabb = rotated.to_axis_aligned();

    CHECK(rotated_aabb.max.x == Approx(std::sqrt(2.0)).margin(1e-12));
    CHECK(rotated_aabb.min.x == Approx(-std::sqrt(2.0)).margin(1e-12));
    CHECK(rotated_aabb.max.x > aabb.max.x);   // 真的变大了
    CHECK(rotated_aabb.max.z == Approx(1.0)); // z 方向不变
}

TEST_CASE("a non-cubic oriented box pins every axis separately",
          "[linear][orientedbox3]") {
    // 上面的用例是立方体 —— 半轴三个分量相等，于是任何把 x/y/z 弄混、
    // 或把 corner 的索引位弄反的实现都看不出来。半轴取 (1,2,3) 之后，
    // 每个轴各自可辨。
    const auto frame = Coordinate3::from_z_axis(
        Point3{0.0, 0.0, 0.0},
        UnitVector3::from_normalized_unchecked(Vector3{0.0, 0.0, 1.0}));
    REQUIRE(frame.has_value());

    const OrientedBox3 box{*frame, Vector3{1.0, 2.0, 3.0}};
    const Box3 aabb = box.to_axis_aligned();

    CHECK(aabb.min == Point3{-1.0, -2.0, -3.0});
    CHECK(aabb.max == Point3{1.0, 2.0, 3.0});

    // corner 的 bit0/bit1/bit2 依次选 x/y/z，置位取 max。
    CHECK(box.corner(0) == Point3{-1.0, -2.0, -3.0});
    CHECK(box.corner(1) == Point3{1.0, -2.0, -3.0});
    CHECK(box.corner(2) == Point3{-1.0, 2.0, -3.0});
    CHECK(box.corner(4) == Point3{-1.0, -2.0, 3.0});
    CHECK(box.corner(7) == Point3{1.0, 2.0, 3.0});

    CHECK(box.center() == Point3{0.0, 0.0, 0.0});
}

TEST_CASE("oriented box containment follows the frame", "[linear][orientedbox3]") {
    const auto x45 = Vector3{1.0, 1.0, 0.0}.normalized();
    const auto y45 = Vector3{-1.0, 1.0, 0.0}.normalized();
    REQUIRE(x45.has_value());
    REQUIRE(y45.has_value());

    const auto frame = Coordinate3::from_axes(Point3{1.0, 1.0, 0.0}, *x45, *y45, z_axis);
    REQUIRE(frame.has_value());

    const OrientedBox3 box{*frame, Vector3{1.0, 1.0, 1.0}};

    CHECK(box.contains(Point3{1.0, 1.0, 0.0}));          // 中心
    CHECK(box.contains(Point3{1.0, 1.0, 1.0}));          // 局部 z 的面上
    CHECK_FALSE(box.contains(Point3{1.0, 1.0, 1.5}));

    // 角落方向：局部 (1,1,1) 在世界里是 rotated_frame 作用后的点。
    // 先确认「局部 (2,0,0)」这个明显在外面的点被拒绝，再确认边界点被接受。
    CHECK_FALSE(box.contains(Point3{1.0 + 2.0 * x45->x(), 1.0 + 2.0 * x45->y(), 0.0}));

    // expanded 之后同一个点应当被接受。
    CHECK(box.expanded(1.5).contains(
        Point3{1.0 + 2.0 * x45->x(), 1.0 + 2.0 * x45->y(), 0.0}));
}

TEST_CASE("expanded never produces a negative half extent",
          "[linear][orientedbox3]") {
    const auto frame = Coordinate3::from_z_axis(
        Point3{0.0, 0.0, 0.0},
        UnitVector3::from_normalized_unchecked(Vector3{0.0, 0.0, 1.0}));
    REQUIRE(frame.has_value());

    const OrientedBox3 box{*frame, Vector3{1.0, 2.0, 3.0}};

    CHECK(box.expanded(1.0).half_extent == Vector3{2.0, 3.0, 4.0});
    CHECK(box.expanded(-0.5).half_extent == Vector3{0.5, 1.5, 2.5});

    // 收缩过头：逐分量夹到 0，而不是留下负的半轴 —— 负半轴会让
    // to_axis_aligned() 给出一个倒置的、悄悄错的盒子。
    CHECK(box.expanded(-10.0).half_extent == Vector3{0.0, 0.0, 0.0});

    // 三维都为 0 时，紧包围盒退化成一个点（即中心），不是空盒。
    const Box3 degenerate = box.expanded(-10.0).to_axis_aligned();
    CHECK_FALSE(degenerate.is_empty());
    CHECK(degenerate.min == Point3{0.0, 0.0, 0.0});
    CHECK(degenerate.max == Point3{0.0, 0.0, 0.0});
}
```

创建 `tests/linear/oriented_box2_test.cpp`。二维**不共享**三维的测试文件，上面四类用例按二维各写一份（`OrientedBox2`/`Box2`/`Point2`/`Vector2`，`corner` 只有 4 个，`from_axes` 两轴，绕 z 旋转改为绕原点旋转 45°）。**非立方体那一份尤其不能省** —— 它是唯一能暴露轴位互换的用例。

- [ ] **Step 2: 实现**

`to_axis_aligned` 必须用**八个角的实际 min/max**：

```cpp
/// 紧致的轴对齐包围盒。
///
/// 取八个角在世界坐标下的实际 min/max。两种常见替代写法都是错的，
/// 且**错的方向相反**，测试对两者都要有检出能力：
///   - 用「包围球半径」（半轴长度之和，这里 = √3 ≈ 1.732）：旋转后过松；
///   - 只把中心变换过去、半轴沿用局部值（这里 = 1）：**过紧** —— 盒子
///     框不住自己的角点，是更危险的那种错。
/// 绕 z 转 45° 的边长 2 立方体，正确结果是 √2 ≈ 1.41421356。
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

`corner(int index)` 先在**局部**坐标里取角（`min = -half_extent`、`max = +half_extent`，索引位的含义与 `Box3T::corner` 相同），再用 `frame.to_parent` 送到世界坐标。`contains` 则是反向的：先 `frame.to_local` 再逐轴比。两个方向都要有测试。

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
- Modify: `tests/linear/transform3_test.cpp`、`transform2_test.cpp`
- Modify: `examples/transform_pipeline.cpp`

**Interfaces:**
- Produces:
  - `constexpr Point3T Transform3T::transform_point(Point3T) const noexcept` —— **`constexpr`，与 `apply` 同口径**（`apply` 是 `constexpr`，两者数学内容相同，没有理由一个能用于常量表达式另一个不能）
  - `constexpr Point2T Transform2T::transform_point(Point2T) const noexcept`
  - 自由运算符 `Transform3T * Point3T -> Point3T`（与 `Transform * Vector3T` 并列）及 `Transform2T * Point2T`

- [ ] **Step 1: 写失败测试**

`transform3_test.cpp` 目前**没有** `Point3` 的 using，也**没有** `<type_traits>`（它现有的 `STATIC_REQUIRE` 都用在别处）。请先补上：

```cpp
#include <type_traits>          // 本任务新用 STATIC_REQUIRE(std::is_same_v<...>)

using GeoCore::linear::Point3;  // 同段的 using 一起补
```

然后追加：

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

**二维同样要一份**（`transform2_test.cpp`）：`Transform2T::transform_point` 在 Interfaces 里，但原计划只在 Files 里列了三维测试文件 —— 那会让这个成员**零覆盖**。照着上面改写：`Transform2::translation(Vector2{10.0, 0.0})`、`Point2{1.0, 2.0}`、期望 `Point2{11.0, 2.0}`，并保留「方向不被平移」那一行（`t * Vector2{1.0, 2.0} == Vector2{1.0, 2.0}`）。

- [ ] **Step 2-3: 实现**

`transform_point` 与 `apply` 的数学内容相同，但接受 `PointNT` 并返回 `PointNT`。**保留 `apply(VectorNT)`** —— 它仍有用途（对以 `VectorNT` 承载的位置做变换），但文档须指明新代码应当用 `transform_point`。

**`Transform3.hpp` 顶部那段注释必须一起改。** Task 3 的 Step 3b 已经把那句失效的「Point3 属于 prim 层」删掉，换成了指向未来的表述：「作用于 Point3 的成员（transform_point）在同一类型上一并提供，**随 Point3 落地（Task 9）**」。Task 9 落地之后这句话就成了**永久的将来时** —— 必须改成现在时，并把这个新的第三种「施加」语义写进那张分工表：

```
/// 三种「施加」语义不要混用 ——
///   operator*(Point3T)    施加完整仿射变换，输入按**位置**解读
///   operator*(Vector3T)   只施加线性部分，输入按**方向**解读（平移不生效）
///   apply(Vector3T)       施加完整仿射变换，把 Vector 读作位置（历史用法，新代码请用 transform_point）
/// transform_point(Point3T) 与 operator*(Point3T) 等价，名字更直白。
```

**「第 4 行假定为 (0,0,0,1)」那一段仍然不要动。**

更新 `examples/transform_pipeline.cpp`：

- `position` 改为 `Point3`，`apply(model, position)` 改为 `model * position`（或 `model.transform_point(position)`），`round_trip` 同理。
- **`print` 辅助函数只接受 `const Vector3&`**，而 `Point3` **不能**转换成 `Vector3`（Task 4 已实测两个方向都不可转换）。所以要么加一个 `print(const char*, const Point3&)` 重载，要么直接打印三个分量。**这一点计划原稿没提，不加就编不过。**
- 文件顶部的注释第 2 条现在写的是「apply() 变换位置，operator\* 变换方向（平移不作用于方向）」—— 改成描述 `operator*` 对 Point 与 Vector 的两种含义。
- **预期输出应当逐字节不变。** 数据没变、数学没变，只是类型从 `Vector3` 变成 `Point3`，打印出来的数字完全相同。**这就是本次示例改动的验收标准**：若输出有任何一位数字变了，说明改错了 —— 不要把它当成「预期输出需要重新生成」。第 44–47 行那段关于 `-4.44e-16` 的注释仍然适用，**不要美化**。

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
- Modify: `include/GeoCore/linear/Vector2.hpp`（**仅**一处已失效的注释，见 Step 1b）

- [ ] **Step 1: 更新 README 与示例**

README 的 quick start 改用成员形式（`a.dot(b)`、`a.normalized()`），并在"What works today"表里加入本轮新增的五个类型。示例同样改用成员形式，重跑并核对输出。

**顺带修**（Task 1 的复核留下的）：`examples/vector_basics.cpp:39` 打印的字符串 `"normalize(zero) correctly returned nullopt"` 里那个函数名已不存在，改成 `normalized`。

- [ ] **Step 1b: 修 `Vector2.hpp` 里那句已经失效的分层说明**

`Vector2.hpp:18` 的文档注释写着「（Point2 属于 prim 层。）」—— **本阶段已把 `Point` 移进 `linear`**（spec 决策 15–17），这句话现在是错的，而且它正是 `Vector` 与 `Point` 类型分离的理由所在，读者会照着它去理解分层。

改法：保留前半句「与 `Point2` 的区别是语义而非存储 —— 两个点相加没有意义，因此类型系统不允许它」—— 这仍然准确且是这个类型存在的理由；只把括注里的分层归属改对（`Point2` 与本类型同处 `linear`）。

Task 3 的 Step 3b 修掉了 `Transform3.hpp` 顶部的同类说法（那段讲的是「Point3 属于 prim 层，linear 不得依赖它」）。**全库只剩这一处**，改完请自己再 grep 一次确认：

```bash
grep -rn "prim 层\|prim层" include/
```

预期零命中（`docs/` 里的历史计划与 spec 记录不算 —— 那是决策的留痕，不要改）。

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
set(BENCHMARK_ENABLE_INSTALL OFF CACHE BOOL "" FORCE)   # 别把第三方的 install 规则带进我们的安装包
FetchContent_MakeAvailable(GoogleBenchmark)

add_executable(GeoCoreBenchmarks linear_benchmarks.cpp)
target_link_libraries(GeoCoreBenchmarks PRIVATE GeoCore::GeoCore benchmark::benchmark)
```

顶层 `CMakeLists.txt` 加选项：

```cmake
option(GEOCORE_BUILD_BENCHMARKS "Build GeoCore benchmarks" OFF)

if(GEOCORE_BUILD_BENCHMARKS)
    add_subdirectory(benchmarks)
endif()
```

默认关闭 —— 基准是开发工具，不应拖慢使用者的配置时间。

- [ ] **Step 2: 写基准**

`benchmarks/linear_benchmarks.cpp` 覆盖**高频路径**，每项都要防止被编译器优化掉（用 `benchmark::DoNotOptimize`）：

**先说清楚这一节最容易搞砸的地方：输入必须是运行期不可知的。**

把输入写成 `const Vector3 a{1.0, 2.0, 3.0};` 这样的字面量常量，编译器会把 `a.dot(b)`
**整个折叠成一个常数**（这里是 32）。`DoNotOptimize` 只保证**结果**不被丢弃，它挡不住
**输入**被常量传播。于是循环体退化成「把一个常量放进寄存器」，测出来是零点几纳秒 ——
而那种数字看上去很漂亮。

所以每个基准都：**输入用非 const 局部变量，并在循环内 `DoNotOptimize` 它们**，
让编译器无法假设它们的值。

```cpp
#include <benchmark/benchmark.h>

#include <GeoCore/GeoCore.hpp>

using namespace GeoCore::linear;

namespace {

// 空循环基准 —— 它是这一节的「零点」，用来发现空转：若某个基准的耗时与它
// 相差无几，那个基准就是被折叠掉了，不是「快」。
void bench_empty(benchmark::State& state) {
    double sink = 0.0;
    for (auto _ : state) {
        benchmark::DoNotOptimize(sink);
    }
}
BENCHMARK(bench_empty);

void bench_dot(benchmark::State& state) {
    Vector3 a{1.0, 2.0, 3.0};
    Vector3 b{4.0, 5.0, 6.0};
    for (auto _ : state) {
        benchmark::DoNotOptimize(a);
        benchmark::DoNotOptimize(b);
        benchmark::DoNotOptimize(a.dot(b));
    }
}
BENCHMARK(bench_dot);

void bench_cross(benchmark::State& state) {
    Vector3 a{1.0, 2.0, 3.0};
    Vector3 b{4.0, 5.0, 6.0};
    for (auto _ : state) {
        benchmark::DoNotOptimize(a);
        benchmark::DoNotOptimize(b);
        benchmark::DoNotOptimize(a.cross(b));
    }
}
BENCHMARK(bench_cross);

void bench_length(benchmark::State& state) {
    Vector3 v{3.0, 4.0, 12.0};
    for (auto _ : state) {
        benchmark::DoNotOptimize(v);
        benchmark::DoNotOptimize(v.length());
    }
}
BENCHMARK(bench_length);

void bench_normalized(benchmark::State& state) {
    Vector3 v{3.0, 4.0, 12.0};
    for (auto _ : state) {
        benchmark::DoNotOptimize(v);
        benchmark::DoNotOptimize(v.normalized());
    }
}
BENCHMARK(bench_normalized);

void bench_subscript(benchmark::State& state) {
    Vector3 v{3.0, 4.0, 12.0};
    for (auto _ : state) {
        benchmark::DoNotOptimize(v);
        benchmark::DoNotOptimize(v[0] + v[1] + v[2]);
    }
}
BENCHMARK(bench_subscript);

void bench_matrix_vector(benchmark::State& state) {
    Matrix4 m{{{2.0, 0.0, 0.0, 1.0},
               {0.0, 3.0, 0.0, 2.0},
               {0.0, 0.0, 4.0, 3.0},
               {0.0, 0.0, 0.0, 1.0}}};
    Vector4 v{1.0, 2.0, 3.0, 4.0};
    for (auto _ : state) {
        benchmark::DoNotOptimize(m);
        benchmark::DoNotOptimize(v);
        benchmark::DoNotOptimize(m * v);
    }
}
BENCHMARK(bench_matrix_vector);

void bench_matrix_inverse(benchmark::State& state) {
    Matrix3 m{{{1.0, 2.0, 3.0}, {0.0, 1.0, 4.0}, {5.0, 6.0, 0.0}}};
    for (auto _ : state) {
        benchmark::DoNotOptimize(m);
        benchmark::DoNotOptimize(m.inverse());
    }
}
BENCHMARK(bench_matrix_inverse);

void bench_quaternion_rotate(benchmark::State& state) {
    const auto axis = UnitVector3::from_normalized_unchecked(Vector3{0.0, 0.0, 1.0});
    Quaternion q = Quaternion::from_axis_angle(axis, 0.7);
    Vector3 v{1.0, 2.0, 3.0};
    for (auto _ : state) {
        benchmark::DoNotOptimize(q);
        benchmark::DoNotOptimize(v);
        benchmark::DoNotOptimize(q.rotate(v));
    }
}
BENCHMARK(bench_quaternion_rotate);

// 本阶段新增的类型也要覆盖 —— 上层会在紧循环里反复调用它们
// （包围盒剔除、坐标系往返变换），它们才是接下来最可能成为热点的地方。
void bench_box_contains(benchmark::State& state) {
    Box3 box{Point3{0.0, 0.0, 0.0}, Point3{1.0, 2.0, 3.0}};
    Point3 p{0.5, 1.0, 1.5};
    for (auto _ : state) {
        benchmark::DoNotOptimize(box);
        benchmark::DoNotOptimize(p);
        benchmark::DoNotOptimize(box.contains(p));
    }
}
BENCHMARK(bench_box_contains);

void bench_coordinate_to_parent(benchmark::State& state) {
    const auto frame = Coordinate3::from_z_axis(
        Point3{1.0, 0.0, 0.0},
        UnitVector3::from_normalized_unchecked(Vector3{0.0, 0.0, 1.0}));
    if (!frame.has_value()) {
        state.SkipWithError("frame construction failed");
        return;
    }
    Coordinate3 c = *frame;
    Point3 p{1.0, 2.0, 3.0};
    for (auto _ : state) {
        benchmark::DoNotOptimize(c);
        benchmark::DoNotOptimize(p);
        benchmark::DoNotOptimize(c.to_parent(p));
    }
}
BENCHMARK(bench_coordinate_to_parent);

} // namespace
```

- [ ] **Step 3: 运行并记录基线**

```bash
cmake --preset windows-vs -DGEOCORE_BUILD_BENCHMARKS=ON
cmake --build --preset windows-vs-release --target GeoCoreBenchmarks
./build/windows-vs/benchmarks/Release/GeoCoreBenchmarks.exe --benchmark_min_time=0.5s
```

把结果写入 `benchmarks/BASELINE.md`，**标注工具链与构建配置**（MSVC 版本、Release、`/O2`）—— 基准数字脱离环境没有意义。

**先做反空转验证，再看数字。** 一个被常量折叠掉的基准会给出**零点几纳秒**的漂亮结果，而「越快越好」的直觉会把它当成好消息。所以：

1. 看 `bench_empty` 的耗时（这是零点）。
2. **每一个实用基准都必须明显慢于 `bench_empty`。** 若某项与之相差无几，那个基准测的就是空气 —— 回头检查它的输入是否被编译器当成了常量。
3. 想更确定的话，做一个直接的反证：把某个基准的循环体临时改成 `benchmark::DoNotOptimize(a.x)`（一个平凡表达式），重跑。若耗时**几乎不变**，原基准就是空转的。验完撤回。

**数字的判读**：`dot`/`cross`/`subscript` 大致在个位数纳秒量级属正常。**但要注意方向** —— 这一节要防的是**假快**，不是假慢。某一项若显示为数十纳秒，先怀疑 `DoNotOptimize` 的屏障开销盖过了运算本身（尤其 `dot` 这种三乘两加的操作）；某一项若显示为零点几纳秒，则先怀疑它被折叠了。**两种异常都要在 BASELINE.md 里如实记录并说明判断依据，不要把数字调成"看起来对"的样子。**

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
5. `benchmarks/BASELINE.md` 存有可复现的基线，且**每个基准都明显慢于空循环基准**（这一条比具体数字重要 —— 详见 Task 11 Step 3 的反空转验证）。

**不在本阶段**：曲线（登记于 spec §5.6）、JSON、SVG。SVG 依赖 2D 图形对象，须待曲线落地；JSON 已明确暂不实现。

**后续阶段的测试要求（来自本轮反馈，登记备查）**：性能测试须覆盖涉及算法或可能被高频调用的场景 —— 本阶段的基准只覆盖线性代数原子操作，`predicates`/`query`/`polygon` 阶段各自需要自己的基准（尤其布尔运算与 BVH 遍历）。精度测试方面，`predicates` 阶段必须对精确谓词做容差边界的穷举验证。
