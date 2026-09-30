# 命名约定落地 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [x]`) syntax for tracking.

**Goal:** 把现有 DragonGeo 源码改成下面这套命名，并把规则写进设计文档，供后续阶段遵循。

**Architecture:** 只改标识符与源文件名，不改语义、分层和算法。头文件已是大驼峰，测试 / 示例 / 基准的源文件名从 `snake_case` 改为大驼峰。自由函数与成员函数同一套大小写。历史实施计划保持原文，活文档以 `docs/superpowers/specs/2026-09-29-dragongeo-design.md` 的命名一节为准。

**Tech Stack:** C++20 · CMake ≥ 3.20 · 既有 `windows-vs` 预设。

## Global Constraints

用户给出的七条，执行时按下列精确定义理解（歧义已在此钉死，不再另作解释）：

- **文件名、类名、方法名、自由函数名、类型别名：大驼峰。** 例：`Vector3Test.cpp`、`DistanceTo`、`ScalarType`、`FromZAxis`。运算符保持 `operator+` 这种写法。
- **函数参数、局部变量、非常量对象：小驼峰。** 例：`angleRadians`、`scaledX`、`halfPi` 不算变量（它是常量）。函数内的 `const` 局部量仍是变量，用小驼峰，不升格为全大写。
- **public 数据成员：大驼峰。** 例：`X`、`Y`、`Z`、`W`、`Min`、`Max`、`Abs`、`Rel`、`Data`、`Frame`、`Matrix`、`HalfExtent`。
- **非 public 数据成员：`m_` + 小驼峰。** 例：`value_` → `m_value`，`origin_` → `m_origin`，`x_` → `m_x`。
- **指针：`p` + 大驼峰。** 裸指针 `pNode`。智能指针允许前缀变化：`unique_ptr` 为 `up`，`shared_ptr` 为 `sp`，`weak_ptr` 为 `wp`，后面仍是大驼峰（`upNode`、`spNode`、`wpNode`）。当前库没有指针，本条只立规则。
- **常量：全大写，单词用下划线。** 命名空间作用域的 `constexpr` / `const`、类作用域的编译期常量、枚举量、宏。例：`PI`、`HALF_PI`、`DIMENSION`；宏保持 `DRAGONGEO_` 前缀。
- **注释用中文。** 公式、命令、标识符可以留在中文句子里。
- **模块名（namespace 与对应目录）：大驼峰。** 例：`DragonGeo::Core`、`DragonGeo::Linear`，目录 `include/DragonGeo/Core/`、`include/DragonGeo/Linear/`。同名类型写在模块内，如 `DragonGeo::Polygon::Polygon`。
- **工具链保留名不改：** `CMakeLists.txt`，以及程序入口 `main.cpp`。

---

### Task 1: 把规则写进设计文档

**Files:**
- Modify: `docs/superpowers/specs/2026-09-29-dragongeo-design.md`（§3.4，以及 §4.5 谓词签名）
- Modify: `README.md`（快速开始里的调用改为新名字，并指向命名一节）

- [x] **Step 1:** 用上面的 Global Constraints 替换 spec §3.4 的 snake_case 条文。
- [x] **Step 2:** 把 §4.5 的 `orient2d` 等自由函数改成大驼峰，避免下一阶段照旧名字实现。
- [x] **Step 3:** 更新 README 快速开始示例。

### Task 2: 标识符

**Files:**
- Modify: `include/DragonGeo/**/*.hpp`、`tests/**/*.cpp`、`examples/*.cpp`、`benchmarks/linear_benchmarks.cpp`、`ci/consumer/main.cpp`

对照：

| 旧 | 新 | 原因 |
|---|---|---|
| `dot(` / `length(` / `normalized(` 等成员与自由函数 | `Dot(` / `Length(` / `Normalized(` | 方法名、函数名大驼峰 |
| `length_squared`、`to_array`、`is_empty`、`transform_point` 等 | `LengthSquared`、`ToArray`、`IsEmpty`、`TransformPoint` | 同上 |
| `x_axis()` 方法 | `XAxis()` | 方法名大驼峰 |
| 变量 `z_axis` | `zAxis` | 变量小驼峰 |
| `scalar_type` | `ScalarType` | 类型别名大驼峰 |
| `.x` `.y` `.z` `.w` `.min` `.max` `.abs` `.rel` `.data` `.frame` `.matrix` | `.X` `.Y` `.Z` `.W` `.Min` `.Max` `.Abs` `.Rel` `.Data` `.Frame` `.Matrix` | public 成员大驼峰 |
| `half_extent` | `HalfExtent` | public 成员，且同名方法也是大驼峰 |
| `value_` `origin_` `x_` `y_` `z_` | `m_value` `m_origin` `m_x` `m_y` `m_z` | 非 public |
| `pi` `half_pi` `two_pi` `quarter_pi` `sqrt_two` | `PI` `HALF_PI` `TWO_PI` `QUARTER_PI` `SQRT_TWO` | 常量 |
| `MatrixT::dimension` | `DIMENSION` | 类作用域常量 |
| `scaled_x`、`angle_radians`、`round_trip` 等 | `scaledX`、`angleRadians`、`roundTrip` | 变量小驼峰 |
| `std::` / Catch2 / `static_assert` / `has_value` / `numeric_limits` / `type_traits` | 不改 | 第三方与标准库 |

标准库里的 `std::numbers::pi` 保持原样。

- [x] **Step 1:** 按上表改全部自有源码。
- [x] **Step 2:** 全库检索，确认自有标识符不再是 `snake_case`，私有成员不再是尾随下划线。

### Task 3: 文件名

**Files:**
- Rename: `tests/**/*_test.cpp`、`examples/*.cpp`、`benchmarks/linear_benchmarks.cpp`
- Modify: `examples/CMakeLists.txt`、`benchmarks/CMakeLists.txt`、`benchmarks/BASELINE.md`

头文件已经是大驼峰，不动。`ci/consumer/main.cpp` 保持 `main.cpp`。

- [x] **Step 1:** 源文件改为大驼峰，CMake 与基线文档里的路径跟着改。示例目标名与文件名一致：`HelloDragonGeo`、`VectorBasics`、`TransformPipeline`。

### Task 4: 注释

**Files:**
- Modify: 仍含英文叙述的自有注释（已知 `tests/core/numeric_test.cpp` 有一段英文）

- [x] **Step 1:** 英文叙述改为中文。命令行、公式、标识符保留。

### Task 5: 验证

- [x] **Step 1:** `cmake --build --preset windows-vs-debug --target DragonGeoTests` 通过。
- [x] **Step 2:** `ctest --preset windows-vs-debug` 全部通过。
