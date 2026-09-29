# GeoCore 阶段 1：工程地基与数值核心 实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 建立 GeoCore 的构建 / 测试 / CI 地基，并完整实现 `core` 与 `linear` 两层 —— 使使用者能用上向量、单位向量、矩阵、四元数与仿射变换。

**Architecture:** 严格单向分层，本计划只覆盖最底两层。`core` 提供常量、数值工具与显式容差模型；`linear` 提供纯代数类型，不含任何几何语义。全部类型 header-only；算法层（predicates / polygon / mesh / solid）留待后续计划，本计划不引入任何编译单元。

**Tech Stack:** C++20 · CMake ≥ 3.20 · Catch2 v3（经 FetchContent 引入，仅开发期依赖）· GitHub Actions · Visual Studio 2022 / GCC / Clang / AppleClang。

**Spec:** `docs/superpowers/specs/2026-09-29-geocore-design.md`

## Global Constraints

- **C++20 必需。** 使用 concepts 约束模板、`std::span` 表达接口、`<=>` 简化比较。
- **CMake ≥ 3.20**，且必须提供 `CMakePresets.json`。
- **零运行时依赖。** Catch2 是开发期依赖，不得出现在任何公开头文件中。
- **对外只暴露 `GeoCore::GeoCore` 一个 target。**
- **命名：** 类型 PascalCase 且优先完整拼写（全库仅 `BSpline`、`BVH` 两个缩写例外）；函数与变量 `snake_case`；私有数据成员尾随下划线；常量 `snake_case`；宏 `GEOCORE_UPPER_SNAKE`；顶层 namespace `GeoCore`，子模块 namespace 小写。
- **容差显式传参**，默认值来自 `GeoCore::core::Tolerance`，不得在任何函数体内硬编码阈值。
- **Point / Vector / UnitVector 类型分离**（Point 属于 `prim` 层，不在本计划范围内）。
- **分量访问的形式取决于类型，且两者会在同一段代码里并存：** `VectorNT` 是聚合，其 `x`/`y`/`z`/`w` 是**数据成员**；`UnitVectorNT` 是不变量类型，其 `x()`/`y()`/`z()` 是**访问器方法**。凡接收集合函数返回值的表达式（如 `cross(u, v)`、`dot` 的两参返回值路径），其分量一律用数据成员形式 —— 写成 `cross(...).z()` 会得到 `error C2064`。
- **未实现的方法一律抛 `std::logic_error`**，消息含函数名与原因。
- **测试文件与被测头文件目录结构镜像**，用 Catch2 的 `TEST_CASE` + 标签。
- **Catch2 v3 的 `Approx` 需要两件事，缺一不可：** 包含 `<catch2/catch_approx.hpp>`，**并且** `using Catch::Approx;`。v3 把 `Approx` 放在 `Catch` 命名空间内，且 Catch2 自身不提供任何 `using Catch::Approx;` —— 只包含头文件会让调用处报 `error C3861: "Approx": 找不到标识符`。断言宏（`TEST_CASE`、`CHECK`、`REQUIRE`、`STATIC_REQUIRE`、`SUCCEED`）来自 `<catch2/catch_test_macros.hpp>`，它们是宏，不受命名空间影响，无需 using。
- **本机为 Windows + Visual Studio 2022，未安装 Ninja。** 本地命令一律用 `windows-vs` 预设；CI 在 Linux/macOS 上用 `ninja-debug`。

## Review Focus

以下五类输入在 spec 中没有逐条规定行为，但最可能咬到使用者。每一条都在对应任务里配了固定它的测试。

1. **零向量归一化** —— `normalize(Vector3{0,0,0})` 必须返回 `std::nullopt`，不得返回含 NaN 的单位向量，不得崩溃。（Task 5）
2. **极大分量导致的中间溢出** —— `Vector3{1e200, 1e200, 0}.length()` 的朴素实现会先算出 `inf`。期望得到有限的 `1.414e200`。（Task 4）
3. **极小组件导致的中间下溢** —— `Vector3{1e-200, 1e-200, 0}.length()` 的朴素实现会下溢到 0。期望得到 `1.414e-200`。（Task 4）
4. **奇异矩阵求逆** —— `inverse()` 遇到行列式为 0 的矩阵必须返回 `std::nullopt`，不得产出含 `inf` / `NaN` 的矩阵。（Task 7）
5. **非有限输入** —— `NaN` / `inf` 进入运算后不得崩溃或死循环。传播语义不作保证，但"不崩溃"必须被测试固定住。（Task 4、Task 7）

---

## File Structure

| 文件 | 职责 |
|---|---|
| `CMakeLists.txt` | 顶层：项目定义、选项、版本头生成、安装与导出 |
| `CMakePresets.json` | 开箱即用的 configure / build / test 预设 |
| `cmake/GeoCoreConfig.cmake.in` | `find_package(GeoCore)` 包配置模板 |
| `cmake/Version.hpp.in` | 版本宏模板 |
| `src/CMakeLists.txt` | 库 target 定义（本阶段为 INTERFACE 库） |
| `.github/workflows/ci.yml` | 跨平台 CI |
| `README.md` | 快速开始 |
| `include/GeoCore/GeoCore.hpp` | 总头，聚合公开接口 |
| `include/GeoCore/core/Constants.hpp` | 数学常量 |
| `include/GeoCore/core/Numeric.hpp` | 数值工具 |
| `include/GeoCore/core/Tolerance.hpp` | 容差模型 |
| `include/GeoCore/linear/Vector2.hpp` | `Vector2T` |
| `include/GeoCore/linear/Vector3.hpp` | `Vector3T` |
| `include/GeoCore/linear/Vector4.hpp` | `Vector4T` |
| `include/GeoCore/linear/UnitVector2.hpp` | `UnitVector2T` + `normalize` |
| `include/GeoCore/linear/UnitVector3.hpp` | `UnitVector3T` + `normalize` |
| `include/GeoCore/linear/Matrix.hpp` | `MatrixT` 家族、行列式、求逆 |
| `include/GeoCore/linear/Quaternion.hpp` | `QuaternionT` |
| `include/GeoCore/linear/Transform2.hpp` | `Transform2T` |
| `include/GeoCore/linear/Transform3.hpp` | `Transform3T` |
| `tests/core/*.cpp` | `core` 层测试 |
| `tests/linear/*.cpp` | `linear` 层测试 |
| `examples/vector_basics.cpp` | 可独立运行的使用示例 |

---

### Task 1: 工程骨架

建立可配置、可构建、可测试、可安装的工程骨架，并以一个冒烟测试证明整条链路通畅。本任务结束后 `cmake --preset windows-vs` → 构建 → `ctest` 必须全绿。

**Files:**
- Create: `CMakeLists.txt`
- Create: `CMakePresets.json`
- Create: `cmake/GeoCoreConfig.cmake.in`
- Create: `cmake/Version.hpp.in`
- Create: `src/CMakeLists.txt`
- Create: `tests/CMakeLists.txt`
- Create: `tests/smoke_test.cpp`
- Create: `examples/CMakeLists.txt`
- Create: `examples/hello_geocore.cpp`
- Create: `include/GeoCore/GeoCore.hpp`
- Create: `.github/workflows/ci.yml`
- Create: `README.md`
- Create: `LICENSE`

**Interfaces:**
- Consumes: 无（本计划第一个任务）
- Produces:
  - CMake target `GeoCore::GeoCore`
  - 生成头 `GeoCore/core/Version.hpp`，宏 `GEOCORE_VERSION_MAJOR/MINOR/PATCH/STRING`
  - 预设名：`windows-vs`、`ninja-debug`、`ninja-release`，对应构建预设 `windows-vs-debug`、`ninja-debug`，测试预设同名
  - 后续所有任务的头文件都放在 `include/GeoCore/`，测试都放在 `tests/` 且由 `tests/CMakeLists.txt` 中的 GLOB 自动纳入

- [ ] **Step 1: 写失败测试 —— 冒烟测试**

创建 `tests/smoke_test.cpp`：

```cpp
#include <catch2/catch_test_macros.hpp>

#include <GeoCore/GeoCore.hpp>

TEST_CASE("GeoCore headers can be included and version macros are defined", "[smoke]") {
    STATIC_REQUIRE(GEOCORE_VERSION_MAJOR >= 0);
    STATIC_REQUIRE(GEOCORE_VERSION_MINOR >= 1);
    SUCCEED("GeoCore.hpp included successfully");
}
```

创建 `include/GeoCore/GeoCore.hpp`（本阶段先聚合 `core`，后续任务逐步补充）：

```cpp
#pragma once

/// GeoCore —— 一个稳健、高效、易用的 C++ 几何库。
///
/// 本头文件聚合全部公开接口。也可以只包含需要的子头以减少编译时间。

#include <GeoCore/core/Version.hpp>
```

创建 `tests/CMakeLists.txt`：

```cmake
include(FetchContent)

FetchContent_Declare(
    Catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG        v3.7.1
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(Catch2)

add_executable(GeoCoreTests)

# 递归收集测试源文件，新增测试文件无需改动本文件。
file(GLOB_RECURSE GEOCORE_TEST_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")

target_sources(GeoCoreTests PRIVATE ${GEOCORE_TEST_SOURCES})
target_link_libraries(GeoCoreTests PRIVATE GeoCore::GeoCore Catch2::Catch2WithMain)

include(Catch)
catch_discover_tests(GeoCoreTests)
```

- [ ] **Step 2: 写库与项目的构建脚本**

创建 `cmake/Version.hpp.in`：

```cpp
#pragma once

#define GEOCORE_VERSION_MAJOR @PROJECT_VERSION_MAJOR@
#define GEOCORE_VERSION_MINOR @PROJECT_VERSION_MINOR@
#define GEOCORE_VERSION_PATCH @PROJECT_VERSION_PATCH@
#define GEOCORE_VERSION_STRING "@PROJECT_VERSION@"
```

创建 `src/CMakeLists.txt`：

```cmake
# 本阶段全部为核心类型，header-only。
# 后续计划引入 predicates / polygon / mesh / solid 的编译单元时，
# 把 add_library(GeoCore INTERFACE) 改为 STATIC 并在此追加源文件。
add_library(GeoCore INTERFACE)
add_library(GeoCore::GeoCore ALIAS GeoCore)

target_include_directories(GeoCore INTERFACE
    $<BUILD_INTERFACE:${GEOCORE_SOURCE_INCLUDE_DIR}>
    $<BUILD_INTERFACE:${GEOCORE_GENERATED_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

target_compile_features(GeoCore INTERFACE cxx_std_20)
```

创建 `CMakeLists.txt`：

```cmake
cmake_minimum_required(VERSION 3.20)

project(GeoCore
    VERSION 0.1.0
    DESCRIPTION "A robust, efficient, and easy-to-use C++ geometry library"
    LANGUAGES CXX)

# 供 src/ 引用的路径变量
set(GEOCORE_SOURCE_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include")
set(GEOCORE_GENERATED_INCLUDE_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated/include")

option(GEOCORE_BUILD_TESTS "Build GeoCore tests" ON)
option(GEOCORE_BUILD_EXAMPLES "Build GeoCore examples" ON)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 库自身的编译选项（只作用于 GeoCore 的编译单元，不泄漏给使用者）
if(MSVC)
    # /utf-8: 源码是 UTF-8 且不带 BOM。不显式声明编码时 MSVC 会按本地代码页
    # 解码，中文注释会产生 C4819；更糟的是注释后若跟字符串字面量，字节会被
    # 真正按错误编码解读 —— 那是静默的行为错误，不只是警告。
    add_compile_options(/W4 /permissive- /utf-8)
else()
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 生成版本头
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Version.hpp.in"
    "${GEOCORE_GENERATED_INCLUDE_DIR}/GeoCore/core/Version.hpp"
    @ONLY)

add_subdirectory(src)

if(GEOCORE_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

if(GEOCORE_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

# ---- 安装与导出 ----
include(CMakePackageConfigHelpers)
include(GNUInstallDirs)

install(TARGETS GeoCore
    EXPORT GeoCoreTargets
    INCLUDES DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}")

install(DIRECTORY "${GEOCORE_SOURCE_INCLUDE_DIR}/GeoCore"
    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}")
install(DIRECTORY "${GEOCORE_GENERATED_INCLUDE_DIR}/GeoCore"
    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}")

install(EXPORT GeoCoreTargets
    FILE GeoCoreTargets.cmake
    NAMESPACE GeoCore::
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/GeoCore")

configure_package_config_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/GeoCoreConfig.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/GeoCoreConfig.cmake"
    INSTALL_DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/GeoCore")

write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/GeoCoreConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion)

install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/GeoCoreConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/GeoCoreConfigVersion.cmake"
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/GeoCore")
```

创建 `cmake/GeoCoreConfig.cmake.in`：

```cmake
@PACKAGE_INIT@

include("${CMAKE_CURRENT_LIST_DIR}/GeoCoreTargets.cmake")

check_required_components(GeoCore)
```

创建 `examples/CMakeLists.txt`：

```cmake
add_executable(hello_geocore hello_geocore.cpp)
target_link_libraries(hello_geocore PRIVATE GeoCore::GeoCore)
```

创建 `examples/hello_geocore.cpp`：

```cpp
#include <iostream>

#include <GeoCore/GeoCore.hpp>

int main() {
    std::cout << "GeoCore " << GEOCORE_VERSION_STRING << '\n';
    return 0;
}
```

- [ ] **Step 3: 配置并构建，确认冒烟测试通过**

本机为 Windows + Visual Studio 2022 且未安装 Ninja，故用 Visual Studio generator：

```bash
cmake -S . -B build/windows-vs -G "Visual Studio 17 2022" -A x64
```

Expected: 配置成功。

```bash
cmake --build build/windows-vs --config Debug
```

Expected: 构建成功，无警告（`/W4` 下）。

```bash
ctest --test-dir build/windows-vs -C Debug --output-on-failure
```

Expected:
```
1/1 Test #1: ...  Passed
100% tests passed, 0 tests failed out of 1
```

- [ ] **Step 4: 写预设、CI 与 README**

创建 `CMakePresets.json`：

> **注意 `version` 的取值：** 必须与项目声明的 CMake 底线一致。schema v2 对应 CMake 3.20，恰好是本计划的底线。切勿写成 v6 —— 那是 CMake 3.25 才引入的 schema，而 presets 文件在任何 `CMakeLists.txt` 之前解析，使用者在 3.20–3.24 上会得到一个预设解析硬错误，且错误信息与项目代码毫无关联。本文件只用到 v2 已有的能力（三类预设列表与 `output.outputOnFailure`）。

```json
{
  "version": 2,
  "cmakeMinimumRequired": { "major": 3, "minor": 20, "patch": 0 },
  "configurePresets": [
    {
      "name": "windows-vs",
      "displayName": "Windows (Visual Studio 2022)",
      "generator": "Visual Studio 17 2022",
      "architecture": { "value": "x64", "strategy": "set" },
      "binaryDir": "${sourceDir}/build/windows-vs",
      "cacheVariables": { "CMAKE_EXPORT_COMPILE_COMMANDS": "ON" }
    },
    {
      "name": "ninja-debug",
      "displayName": "Ninja Debug",
      "generator": "Ninja",
      "binaryDir": "${sourceDir}/build/ninja-debug",
      "cacheVariables": {
        "CMAKE_BUILD_TYPE": "Debug",
        "CMAKE_EXPORT_COMPILE_COMMANDS": "ON"
      }
    },
    {
      "name": "ninja-release",
      "displayName": "Ninja Release",
      "generator": "Ninja",
      "binaryDir": "${sourceDir}/build/ninja-release",
      "cacheVariables": {
        "CMAKE_BUILD_TYPE": "Release",
        "CMAKE_EXPORT_COMPILE_COMMANDS": "ON"
      }
    }
  ],
  "buildPresets": [
    { "name": "windows-vs-debug",   "configurePreset": "windows-vs",  "configuration": "Debug" },
    { "name": "windows-vs-release", "configurePreset": "windows-vs",  "configuration": "Release" },
    { "name": "ninja-debug",        "configurePreset": "ninja-debug" },
    { "name": "ninja-release",      "configurePreset": "ninja-release" }
  ],
  "testPresets": [
    {
      "name": "windows-vs-debug",
      "configurePreset": "windows-vs",
      "configuration": "Debug",
      "output": { "outputOnFailure": true }
    },
    {
      "name": "ninja-debug",
      "configurePreset": "ninja-debug",
      "output": { "outputOnFailure": true }
    }
  ]
}
```

创建 `.github/workflows/ci.yml`：

```yaml
name: CI

on:
  push:
    branches: [main]
  pull_request:

jobs:
  build-and-test:
    name: ${{ matrix.name }}
    runs-on: ${{ matrix.os }}
    strategy:
      fail-fast: false
      matrix:
        include:
          - name: Linux GCC
            os: ubuntu-latest
            configure: ninja-debug
            build: ninja-debug
            test: ninja-debug
            cxx: g++
          - name: Linux Clang
            os: ubuntu-latest
            configure: ninja-debug
            build: ninja-debug
            test: ninja-debug
            cxx: clang++
          - name: macOS AppleClang
            os: macos-latest
            configure: ninja-debug
            build: ninja-debug
            test: ninja-debug
            cxx: clang++
          - name: Windows MSVC
            os: windows-latest
            configure: windows-vs
            build: windows-vs-debug
            test: windows-vs-debug
            cxx: ""

    steps:
      - uses: actions/checkout@v4

      - name: Configure
        run: cmake --preset ${{ matrix.configure }}
        env:
          CXX: ${{ matrix.cxx }}

      - name: Build
        run: cmake --build --preset ${{ matrix.build }}

      - name: Test
        run: ctest --preset ${{ matrix.test }}
```

创建 `README.md`：

```markdown
# GeoCore

A robust, efficient, and easy-to-use C++ geometry library.

GeoCore targets three capability domains — a general geometry toolkit, 2D
computational geometry, and 3D meshes and graphics — plus a B-rep interface
skeleton for exact solid modelling.

**Status:** early development. The API surface is still `unstable`.

## Requirements

- A C++20 compiler (MSVC 19.30+, GCC 11+, Clang 14+, AppleClang 14+)
- CMake 3.20 or newer

## Building

```bash
cmake --preset windows-vs      # or ninja-debug on Linux/macOS
cmake --build --preset windows-vs-debug
ctest --preset windows-vs-debug
```

## Design principles

- **Robust predicates over epsilon.** Orientation and in-circle decisions are
  exact, so degenerate input gets a correct answer rather than a plausible one.
- **Tolerance is an explicit parameter.** There is no global epsilon constant.
- **Types carry invariants.** `Point`, `Vector`, and `UnitVector` are distinct
  types, so a whole class of bugs is rejected at compile time.

See `docs/superpowers/specs/` for the full design.
```

创建 `LICENSE`（MIT，对应 spec 7.6 的默认选择）：

```
MIT License

Copyright (c) 2026 GeoCore contributors

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

- [ ] **Step 5: 用预设重新验证并提交**

```bash
cmake --preset windows-vs
cmake --build --preset windows-vs-debug
ctest --preset windows-vs-debug
```

Expected: 1 个测试通过。

```bash
git add CMakeLists.txt CMakePresets.json cmake/ src/ tests/ examples/ include/ .github/ README.md LICENSE
git commit -m "build: add project skeleton with CMake, Catch2 and CI"
```

---

### Task 2: core —— 常量、数值工具与容差模型

实现最底层的 `core`。容差模型（`Tolerance`）是全库的公共契约，后续每一个需要精度的函数都会用到它，因此它的语义必须在本任务中被测试固定死。

**Files:**
- Create: `include/GeoCore/core/Constants.hpp`
- Create: `include/GeoCore/core/Numeric.hpp`
- Create: `include/GeoCore/core/Tolerance.hpp`
- Create: `tests/core/numeric_test.cpp`
- Create: `tests/core/tolerance_test.cpp`
- Modify: `include/GeoCore/GeoCore.hpp`（追加 include）

**Interfaces:**
- Consumes: Task 1 的 `GeoCore::GeoCore` target 与版本头
- Produces:
  - `GeoCore::core::pi`、`half_pi`、`two_pi`、`quarter_pi`、`sqrt_two` —— 均为 `inline constexpr double`
  - `template <std::floating_point S> constexpr S absolute_value(S) noexcept`
  - `template <std::floating_point S> constexpr S clamp(S value, S low, S high) noexcept`
  - `template <std::floating_point S> S safe_sqrt(S) noexcept` —— 负输入返回 0
  - `struct Tolerance { double abs{1e-12}; double rel{1e-9}; }`，含 `resolve(double)`、`equal(double,double)`、`is_zero(double)`

- [ ] **Step 1: 写失败测试 —— 数值工具**

创建 `tests/core/numeric_test.cpp`：

```cpp
#include <catch2/catch_test_macros.hpp>

#include <limits>

#include <GeoCore/core/Numeric.hpp>

using GeoCore::core::absolute_value;
using GeoCore::core::clamp;
using GeoCore::core::safe_sqrt;

TEST_CASE("absolute_value returns the magnitude", "[core][numeric]") {
    CHECK(absolute_value(-3.0) == 3.0);
    CHECK(absolute_value(3.0) == 3.0);
    CHECK(absolute_value(0.0) == 0.0);
}

TEST_CASE("absolute_value propagates NaN rather than returning garbage",
          "[core][numeric][degenerate]") {
    CHECK(std::isnan(absolute_value(std::numeric_limits<double>::quiet_NaN())));
}

TEST_CASE("clamp bounds the value on both sides", "[core][numeric]") {
    CHECK(clamp(-1.0, 0.0, 1.0) == 0.0);
    CHECK(clamp(0.5, 0.0, 1.0) == 0.5);
    CHECK(clamp(2.0, 0.0, 1.0) == 1.0);
}

TEST_CASE("safe_sqrt maps negative input to zero", "[core][numeric][degenerate]") {
    CHECK(safe_sqrt(4.0) == 2.0);
    CHECK(safe_sqrt(0.0) == 0.0);
    // Rounding can push a squared length a hair below zero; that must not
    // produce NaN.
    CHECK(safe_sqrt(-1e-18) == 0.0);
}
```

- [ ] **Step 2: 运行测试，确认失败**

```bash
cmake --build build/windows-vs --config Debug
```

Expected: 编译失败 —— `Cannot open include file: 'GeoCore/core/Numeric.hpp'`。

- [ ] **Step 3: 实现常量与数值工具**

创建 `include/GeoCore/core/Constants.hpp`：

```cpp
#pragma once

#include <numbers>

namespace GeoCore::core {

/// 圆周率。
inline constexpr double pi = std::numbers::pi;

/// 半圆周率，π/2。
inline constexpr double half_pi = pi / 2.0;

/// 整圆周，2π。
inline constexpr double two_pi = 2.0 * pi;

/// 四分之一圆周，π/4。
inline constexpr double quarter_pi = pi / 4.0;

/// √2。
inline constexpr double sqrt_two = std::numbers::sqrt2;

} // namespace GeoCore::core
```

创建 `include/GeoCore/core/Numeric.hpp`：

```cpp
#pragma once

#include <concepts>
#include <cmath>
#include <limits>

namespace GeoCore::core {

/// 绝对值。与 std::abs 的差别：本函数是 constexpr（C++20 的 std::abs
/// 对标量浮点尚不是），且对 -0.0 返回 -0.0 —— 由于 -0.0 == 0.0，
/// 这不影响任何比较结果。
template <std::floating_point Scalar>
[[nodiscard]] constexpr Scalar absolute_value(Scalar value) noexcept {
    return value < Scalar{0} ? -value : value;
}

/// 是否为有限值（既非 ±inf 也非 NaN）。
///
/// 手写而非使用 std::isfinite：后者在 C++20 尚不是 constexpr，而本层的
/// 工具函数需要能在常量表达式中求值。Tolerance 依赖它来拒绝非有限输入 ——
/// 没有这个判断，`|inf - 5| <= resolve(inf)` 会退化成 `inf <= inf`，
/// 把无穷大判成「与任何有限值相等」。
template <std::floating_point Scalar>
[[nodiscard]] constexpr bool is_finite(Scalar value) noexcept {
    return value == value
        && value != std::numeric_limits<Scalar>::infinity()
        && value != -std::numeric_limits<Scalar>::infinity();
}

/// 把 value 限制到 [low, high]。要求 low <= high。
template <std::floating_point Scalar>
[[nodiscard]] constexpr Scalar clamp(Scalar value, Scalar low, Scalar high) noexcept {
    if (value < low) {
        return low;
    }
    if (value > high) {
        return high;
    }
    return value;
}

/// 平方根，负输入返回 0。
///
/// 存在意义：浮点舍入可能让本应为零的平方和变为极小的负数，
/// 此时 std::sqrt 会返回 NaN 并污染整条计算链。
///
/// 注意：-0.0 与恰好为 0 的输入返回 0；NaN 输入返回 NaN。
template <std::floating_point Scalar>
[[nodiscard]] inline Scalar safe_sqrt(Scalar value) noexcept {
    if (value <= Scalar{0}) {
        return Scalar{0};
    }
    return std::sqrt(value);
}

} // namespace GeoCore::core
```

修改 `include/GeoCore/GeoCore.hpp`：

```cpp
#pragma once

/// GeoCore —— 一个稳健、高效、易用的 C++ 几何库。
///
/// 本头文件聚合全部公开接口。也可以只包含需要的子头以减少编译时间。

#include <GeoCore/core/Constants.hpp>
#include <GeoCore/core/Numeric.hpp>
#include <GeoCore/core/Tolerance.hpp>
#include <GeoCore/core/Version.hpp>
```

（`Tolerance.hpp` 在下一步创建；若此时编译失败属预期，下一步补齐。）

- [ ] **Step 4: 写失败测试 —— 容差模型**

创建 `tests/core/tolerance_test.cpp`：

```cpp
#include <catch2/catch_test_macros.hpp>

#include <limits>

#include <GeoCore/core/Tolerance.hpp>

using GeoCore::core::Tolerance;

TEST_CASE("Tolerance::resolve grows with magnitude", "[core][tolerance]") {
    const Tolerance tolerance{1e-12, 1e-9};

    // 量级远小于绝对项时，有效容差就是绝对项
    CHECK(tolerance.resolve(0.0) == 1e-12);

    // 量级足够大时，相对项主导
    CHECK(tolerance.resolve(1.0) == 1e-12 + 1e-9);
    CHECK(tolerance.resolve(1e6) == 1e-12 + 1e-9 * 1e6);
}

TEST_CASE("Tolerance::equal respects scale", "[core][tolerance]") {
    const Tolerance tolerance{1e-12, 1e-9};

    CHECK(tolerance.equal(1.0, 1.0));
    CHECK(tolerance.equal(1.0, 1.0 + 1e-12));
    CHECK_FALSE(tolerance.equal(1.0, 1.0 + 1e-6));

    // 大尺度下，1e-3 的差异在相对容差之内
    CHECK(tolerance.equal(1e7, 1e7 + 1e-3));
    // 小尺度下，同样的绝对差异过大
    CHECK_FALSE(tolerance.equal(1e-6, 1e-6 + 1e-3));
}

TEST_CASE("Tolerance::is_zero rejects only genuinely tiny values",
          "[core][tolerance]") {
    const Tolerance tolerance{1e-12, 1e-9};

    CHECK(tolerance.is_zero(0.0));
    CHECK(tolerance.is_zero(1e-15));
    CHECK_FALSE(tolerance.is_zero(1e-6));
    CHECK_FALSE(tolerance.is_zero(-1.0));
}

TEST_CASE("a zero tolerance rejects only exact zero", "[core][tolerance]") {
    const Tolerance exact{0.0, 0.0};

    CHECK(exact.is_zero(0.0));
    CHECK_FALSE(exact.is_zero(1e-300));
}

TEST_CASE("Tolerance::resolve is symmetric in the sign of the magnitude",
          "[core][tolerance]") {
    const Tolerance tolerance{1e-12, 1e-9};

    // 量级一律取绝对值，符号不应有任何影响。
    //
    // 这条断言专门盯住一个不写绝对值就完全无法察觉的回归：若 resolve 误写成
    // `abs + rel * magnitude`，正量级的用例会全部照过（文件里其余量级都是正数），
    // 只有负量级会得到一个更小的容差。这是该公式最可能的一次写错。
    CHECK(tolerance.resolve(-1e6) == tolerance.resolve(1e6));
    CHECK(tolerance.resolve(-1.0) == tolerance.resolve(1.0));
    CHECK(tolerance.resolve(-1e6) > tolerance.resolve(0.0));
}

TEST_CASE("non-finite values are neither zero nor approximately equal",
          "[core][tolerance][degenerate]") {
    const Tolerance tolerance{1e-12, 1e-9};
    const double infinity = std::numeric_limits<double>::infinity();
    const double not_a_number = std::numeric_limits<double>::quiet_NaN();

    // 溢出成无穷大的量绝不能被判为零 —— 它恰恰是调用者最该被示警的情形
    CHECK_FALSE(tolerance.is_zero(infinity));
    CHECK_FALSE(tolerance.is_zero(-infinity));

    // 无穷大不等于任何有限值
    CHECK_FALSE(tolerance.equal(infinity, 5.0));
    CHECK_FALSE(tolerance.equal(infinity, 1e300));
    CHECK_FALSE(tolerance.equal(-infinity, 1e300));

    // 但「完全相等」仍是相等，无穷大也不例外
    CHECK(tolerance.equal(infinity, infinity));

    // NaN 不等于任何东西，包括它自己
    CHECK_FALSE(tolerance.equal(not_a_number, not_a_number));
    CHECK_FALSE(tolerance.is_zero(not_a_number));
}
```

- [ ] **Step 5: 运行测试，确认失败**

```bash
cmake --build build/windows-vs --config Debug
```

Expected: 编译失败 —— 找不到 `GeoCore/core/Tolerance.hpp`。

- [ ] **Step 6: 实现容差模型**

创建 `include/GeoCore/core/Tolerance.hpp`：

```cpp
#pragma once

#include <GeoCore/core/Numeric.hpp>

namespace GeoCore::core {

/// 显式的容差模型。
///
/// GeoCore 刻意不提供全局 epsilon 常量：单一阈值在坐标尺度相差数个
/// 数量级的输入上必然失效。容差作为参数显式传递，调用者始终知道
/// 自己在何种精度下工作。
///
/// 有效容差 = abs + rel * |magnitude|，即「绝对项」与「随量级增长的
/// 相对项」之和。
struct Tolerance {
    /// 绝对项，与量级无关的下限。
    double abs = 1e-12;

    /// 相对项，随参考量级线性增长。
    double rel = 1e-9;

    /// 给出参考量级对应的有效容差。
    [[nodiscard]] constexpr double resolve(double magnitude) const noexcept {
        return abs + rel * absolute_value(magnitude);
    }

    /// 判定两个标量在该容差下是否可视为相等。
    /// 参考量级取二者绝对值中的较大者。
    ///
    /// 非有限输入另行处理：完全相等（含 ±inf 与自身）返回 true，其余任何
    /// 涉及 ±inf 或 NaN 的组合一律返回 false。若不特判，`|inf - 5| <=
    /// resolve(inf)` 会退化为 `inf <= inf` 而返回 true —— 无穷大被判成
    /// 「与任何有限值相等」，这与本类型的存在目的恰好相反。
    [[nodiscard]] constexpr bool equal(double a, double b) const noexcept {
        if (a == b) {
            return true;
        }
        if (!is_finite(a) || !is_finite(b)) {
            return false;
        }
        const double scale = absolute_value(a) > absolute_value(b)
                                 ? absolute_value(a)
                                 : absolute_value(b);
        return absolute_value(a - b) <= resolve(scale);
    }

    /// 判定标量在该容差下是否可视为零。
    ///
    /// 注意参考量级取的是 x 自身，故判定条件等价于
    /// |x| <= abs / (1 - rel)，在 rel 远小于 1 时约等于 abs。
    ///
    /// 非有限值一律不视为零：把溢出成 ±inf 或变成 NaN 的量静默归类为
    /// 「可忽略」，正是调用者最需要被示警时却得到放行的情形。
    [[nodiscard]] constexpr bool is_zero(double x) const noexcept {
        if (!is_finite(x)) {
            return false;
        }
        return absolute_value(x) <= resolve(x);
    }
};

} // namespace GeoCore::core
```

- [ ] **Step 7: 运行全部测试并提交**

```bash
cmake --build build/windows-vs --config Debug
ctest --test-dir build/windows-vs -C Debug --output-on-failure
```

Expected: 全部测试通过（冒烟 1 个 + numeric 4 个 + tolerance 6 个，共 11 个）。

```bash
git add include/GeoCore tests/core
git commit -m "feat(core): add constants, numeric utilities and the tolerance model"
```

---

### Task 3: linear —— Vector2

`Vector2T` 是第一个代数类型，也确立了本层其余类型的写法：简单聚合 + 自由函数运算符 + 稳健几何量计算。

`length()` 刻意不写成朴素的 `std::sqrt(x*x + y*y)`。朴素实现在分量超过约 1e154 时中间量上溢为 `inf`，在分量小于约 1e-154 时下溢为 0。本实现先按最大分量缩放，两个方向的退化都被消除 —— 这正是 Review Focus 第 2、3 条。

**Files:**
- Create: `include/GeoCore/linear/Vector2.hpp`
- Create: `tests/linear/vector2_test.cpp`
- Modify: `include/GeoCore/GeoCore.hpp`（追加 include）

**Interfaces:**
- Consumes: `GeoCore::core::absolute_value`（Task 2）
- Produces:
  - `template <typename Scalar> struct Vector2T { Scalar x{}; Scalar y{}; }`，聚合初始化 `Vector2T<S>{x, y}`
  - 别名 `Vector2`（`double`）、`Vector2f`（`float`）
  - 成员：`length_squared() const noexcept` → `Scalar`；`length() const noexcept` → `Scalar`
  - 自由函数：`dot(Vector2T<S>, Vector2T<S>) -> S`；`cross(Vector2T<S>, Vector2T<S>) -> S`（二维叉积为标量）
  - 运算符：`+`、一元 `-`、二元 `-`、`*`（向量×标量、标量×向量）、`/`（向量÷标量）、`==`

- [ ] **Step 1: 写失败测试**

创建 `tests/linear/vector2_test.cpp`：

```cpp
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <limits>

#include <GeoCore/linear/Vector2.hpp>

using Catch::Approx;

using GeoCore::linear::Vector2;
using GeoCore::linear::Vector2f;

TEST_CASE("Vector2 supports aggregate initialization", "[linear][vector2]") {
    const Vector2 v{3.0, 4.0};
    CHECK(v.x == 3.0);
    CHECK(v.y == 4.0);
}

TEST_CASE("default-constructed Vector2 is the zero vector", "[linear][vector2]") {
    const Vector2 v{};
    CHECK(v.x == 0.0);
    CHECK(v.y == 0.0);
    CHECK(v.length() == 0.0);
}

TEST_CASE("Vector2 arithmetic is component-wise", "[linear][vector2]") {
    const Vector2 a{1.0, 2.0};
    const Vector2 b{3.0, 5.0};

    CHECK(a + b == Vector2{4.0, 7.0});
    CHECK(b - a == Vector2{2.0, 3.0});
    CHECK(-a == Vector2{-1.0, -2.0});
    CHECK(a * 2.0 == Vector2{2.0, 4.0});
    CHECK(2.0 * a == Vector2{2.0, 4.0});
    CHECK(b / 2.0 == Vector2{1.5, 2.5});
}

TEST_CASE("scalar multiplication accepts integer factors", "[linear][vector2]") {
    const Vector2 a{1.0, 2.0};
    CHECK(a * 3 == Vector2{3.0, 6.0});
}

TEST_CASE("dot and cross match their definitions", "[linear][vector2]") {
    const Vector2 a{1.0, 0.0};
    const Vector2 b{0.0, 1.0};

    CHECK(dot(a, b) == 0.0);
    CHECK(dot(a, a) == 1.0);
    CHECK(cross(a, b) == 1.0);
    CHECK(cross(b, a) == -1.0);
    CHECK(cross(a, a) == 0.0);
}

TEST_CASE("length of the 3-4-5 triangle is exact", "[linear][vector2]") {
    const Vector2 v{3.0, 4.0};
    CHECK(v.length_squared() == 25.0);
    CHECK(v.length() == 5.0);
}

TEST_CASE("length survives components near the overflow threshold",
          "[linear][vector2][degenerate]") {
    // 朴素实现会先算出 1e200 * 1e200 == inf
    const Vector2 v{1e200, 1e200};
    const double length = v.length();

    CHECK(std::isfinite(length));
    CHECK(length == Approx(1.4142135623730951e200).epsilon(1e-12));
}

TEST_CASE("length survives components near the underflow threshold",
          "[linear][vector2][degenerate]") {
    // 朴素实现会先算出 1e-200 * 1e-200 == 0
    const Vector2 v{1e-200, 1e-200};
    const double length = v.length();

    CHECK(length > 0.0);
    CHECK(length == Approx(1.4142135623730951e-200).epsilon(1e-12));
}

TEST_CASE("length of a vector containing NaN does not crash",
          "[linear][vector2][degenerate]") {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const Vector2 v{nan, 1.0};
    CHECK(std::isnan(v.length()));
}

TEST_CASE("Vector2T is usable with float", "[linear][vector2]") {
    const Vector2f v{3.0f, 4.0f};
    CHECK(v.length() == 5.0f);
    CHECK(v.x == 3.0f);
}

TEST_CASE("length of a vector containing infinity is infinity, not NaN",
          "[linear][vector2][degenerate]") {
    const double infinity = std::numeric_limits<double>::infinity();

    // 缩放写法若不特判，inf / inf 会算出 NaN 并污染整条计算链 ——
    // 缩放本是为消除溢出而引入，不能反而在无穷输入上退化。
    CHECK(Vector2{infinity, 1.0}.length() == infinity);
    CHECK(Vector2{1.0, infinity}.length() == infinity);
    CHECK(Vector2{infinity, infinity}.length() == infinity);
}
```

- [ ] **Step 2: 运行测试，确认失败**

```bash
cmake --build build/windows-vs --config Debug
```

Expected: 编译失败 —— 找不到 `GeoCore/linear/Vector2.hpp`。

- [ ] **Step 3: 实现 Vector2**

创建 `include/GeoCore/linear/Vector2.hpp`：

```cpp
#pragma once

#include <concepts>
#include <cmath>

#include <GeoCore/core/Numeric.hpp>

namespace GeoCore::linear {

/// 二维向量：纯代数载体，不含任何几何语义。
///
/// 与 Point2 的区别是语义而非存储 —— 两个点相加没有意义，因此
/// 类型系统不允许它。（Point2 属于 prim 层。）
template <typename Scalar>
struct Vector2T {
    using scalar_type = Scalar;

    // 刻意不声明任何构造函数。C++20 起「用户声明的构造函数」——哪怕只是
    // `= default` —— 都会让类型不再是聚合，进而使 `Vector2{3.0, 4.0}` 这类
    // 聚合初始化失效。默认成员初始化器已经提供了零初始化，无需额外构造函数。
    Scalar x{};
    Scalar y{};

    [[nodiscard]] constexpr Scalar length_squared() const noexcept {
        return x * x + y * y;
    }

    /// 欧几里得长度。
    ///
    /// 先按最大分量缩放再求平方根，因此分量在 1e-200 或 1e200 这类
    /// 极端量级上都不会因中间量下溢/上溢而丢失精度。代价是两次除法，
    /// 热路径上若只需要比较长度请改用 length_squared()。
    [[nodiscard]] Scalar length() const noexcept {
        const Scalar abs_x = core::absolute_value(x);
        const Scalar abs_y = core::absolute_value(y);
        const Scalar scale = abs_x > abs_y ? abs_x : abs_y;
        if (scale == Scalar{0}) {
            return Scalar{0};
        }
        if (!core::is_finite(scale)) {
            // 含 ±inf（或全为 NaN）分量时，下面的 inf / inf 会算出 NaN 并污染
            // 结果 —— 缩放本是为消除溢出而引入，不能反而在无穷输入上退化。
            // 此时真实长度本就是 ±inf，直接返回它。
            return scale;
        }
        const Scalar scaled_x = x / scale;
        const Scalar scaled_y = y / scale;
        return scale * std::sqrt(scaled_x * scaled_x + scaled_y * scaled_y);
    }
};

using Vector2 = Vector2T<double>;
using Vector2f = Vector2T<float>;

// ---- 运算符 ----

template <typename Scalar>
[[nodiscard]] constexpr Vector2T<Scalar> operator+(Vector2T<Scalar> a, Vector2T<Scalar> b) noexcept {
    return Vector2T<Scalar>{a.x + b.x, a.y + b.y};
}

template <typename Scalar>
[[nodiscard]] constexpr Vector2T<Scalar> operator-(Vector2T<Scalar> v) noexcept {
    return Vector2T<Scalar>{-v.x, -v.y};
}

template <typename Scalar>
[[nodiscard]] constexpr Vector2T<Scalar> operator-(Vector2T<Scalar> a, Vector2T<Scalar> b) noexcept {
    return Vector2T<Scalar>{a.x - b.x, a.y - b.y};
}

/// 向量 × 标量。因子接受任意可转换为 Scalar 的算术类型，
/// 因此 `v * 3` 与 `v * 3.0` 都成立；传向量进去会被约束拒绝。
template <typename Scalar, typename Factor>
    requires std::convertible_to<Factor, Scalar>
[[nodiscard]] constexpr Vector2T<Scalar> operator*(Vector2T<Scalar> v, Factor factor) noexcept {
    const auto scale = static_cast<Scalar>(factor);
    return Vector2T<Scalar>{v.x * scale, v.y * scale};
}

template <typename Scalar, typename Factor>
    requires std::convertible_to<Factor, Scalar>
[[nodiscard]] constexpr Vector2T<Scalar> operator*(Factor factor, Vector2T<Scalar> v) noexcept {
    return v * factor;
}

template <typename Scalar, typename Divisor>
    requires std::convertible_to<Divisor, Scalar>
[[nodiscard]] constexpr Vector2T<Scalar> operator/(Vector2T<Scalar> v, Divisor divisor) noexcept {
    const auto scale = static_cast<Scalar>(divisor);
    return Vector2T<Scalar>{v.x / scale, v.y / scale};
}

template <typename Scalar>
[[nodiscard]] constexpr bool operator==(Vector2T<Scalar> a, Vector2T<Scalar> b) noexcept {
    return a.x == b.x && a.y == b.y;
}
// C++20 由 operator== 自动生成 operator!=，无需手写。

// ---- 几何量 ----

/// 点积。
template <typename Scalar>
[[nodiscard]] constexpr Scalar dot(Vector2T<Scalar> a, Vector2T<Scalar> b) noexcept {
    return a.x * b.x + a.y * b.y;
}

/// 二维叉积，返回标量（有向面积的两倍再取半，即 z 分量）。
/// 正值表示 b 在 a 的逆时针一侧。
template <typename Scalar>
[[nodiscard]] constexpr Scalar cross(Vector2T<Scalar> a, Vector2T<Scalar> b) noexcept {
    return a.x * b.y - a.y * b.x;
}

} // namespace GeoCore::linear
```

修改 `include/GeoCore/GeoCore.hpp`，在 `core` 的 include 之后追加：

```cpp
#include <GeoCore/linear/Vector2.hpp>
```

- [ ] **Step 4: 运行测试，确认通过**

```bash
cmake --build build/windows-vs --config Debug
ctest --test-dir build/windows-vs -C Debug --output-on-failure
```

Expected: 全部通过，其中包含两条 `[degenerate]` 的重缩放测试。

- [ ] **Step 5: 提交**

```bash
git add include/GeoCore/linear/Vector2.hpp include/GeoCore/GeoCore.hpp tests/linear/vector2_test.cpp
git commit -m "feat(linear): add Vector2T with overflow-safe length"
```

---

### Task 4: linear —— Vector3 与 Vector4

`Vector3T` 比 `Vector2T` 多一个真正返回向量的叉积；`Vector4T` 没有叉积，但需要为 Task 6 的矩阵乘法提供齐次坐标上的支持。两者的 `length()` 沿用 Task 3 的缩放写法。

**Files:**
- Create: `include/GeoCore/linear/Vector3.hpp`
- Create: `include/GeoCore/linear/Vector4.hpp`
- Create: `tests/linear/vector3_test.cpp`
- Create: `tests/linear/vector4_test.cpp`
- Modify: `include/GeoCore/GeoCore.hpp`（追加 include）

**Interfaces:**
- Consumes: `GeoCore::core::absolute_value`（Task 2）
- Produces:
  - `template <typename Scalar> struct Vector3T { Scalar x{}; Scalar y{}; Scalar z{}; }`
  - `template <typename Scalar> struct Vector4T { Scalar x{}; Scalar y{}; Scalar z{}; Scalar w{}; }`
  - 别名 `Vector3`、`Vector3f`、`Vector4`、`Vector4f`
  - 成员 `length_squared()` / `length()`，语义同 Task 3
  - 自由函数 `dot`、`cross`（仅三维：`cross(Vector3T<S>, Vector3T<S>) -> Vector3T<S>`）
  - 运算符与 `Vector2T` 完全一致：`+`、一元 `-`、二元 `-`、`*`、`/`、`==`

- [ ] **Step 1: 写失败测试**

创建 `tests/linear/vector3_test.cpp`：

```cpp
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <limits>

#include <GeoCore/linear/Vector3.hpp>

using Catch::Approx;

using GeoCore::linear::Vector3;
using GeoCore::linear::Vector3f;

TEST_CASE("Vector3 arithmetic is component-wise", "[linear][vector3]") {
    const Vector3 a{1.0, 2.0, 3.0};
    const Vector3 b{4.0, 5.0, 6.0};

    CHECK(a + b == Vector3{5.0, 7.0, 9.0});
    CHECK(b - a == Vector3{3.0, 3.0, 3.0});
    CHECK(-a == Vector3{-1.0, -2.0, -3.0});
    CHECK(a * 2.0 == Vector3{2.0, 4.0, 6.0});
    CHECK(b / 2.0 == Vector3{2.0, 2.5, 3.0});
}

TEST_CASE("dot is symmetric and matches the definition", "[linear][vector3]") {
    const Vector3 a{1.0, 2.0, 3.0};
    const Vector3 b{4.0, -5.0, 6.0};

    CHECK(dot(a, b) == 1.0 * 4.0 + 2.0 * -5.0 + 3.0 * 6.0);
    CHECK(dot(a, b) == dot(b, a));
}

TEST_CASE("cross is anticommutative and right-handed", "[linear][vector3]") {
    const Vector3 x_axis{1.0, 0.0, 0.0};
    const Vector3 y_axis{0.0, 1.0, 0.0};
    const Vector3 z_axis{0.0, 0.0, 1.0};

    CHECK(cross(x_axis, y_axis) == z_axis);
    CHECK(cross(y_axis, x_axis) == -z_axis);
    CHECK(cross(x_axis, x_axis) == Vector3{0.0, 0.0, 0.0});

    const Vector3 a{1.0, 2.0, 3.0};
    const Vector3 b{4.0, 5.0, 6.0};
    CHECK(cross(a, b) == -cross(b, a));

    // 上面这些断言全是输出的线性性质，任何固定线性映射都能满足它们 ——
    // 反交换性对任意 S 都成立（S(b×a) = -S(a×b)），零结果对线性映射也
    // 仍是零。因此形如 diag(s1, s2, 1)·(a×b) 的实现能通过全部断言，
    // 包括 s1 = s2 = 0（x、y 分量恒为零）。下面用三个轴的循环和一个
    // 一般对，把每个分量各自钉死。
    CHECK(cross(y_axis, z_axis) == x_axis);
    CHECK(cross(z_axis, x_axis) == y_axis);
    CHECK(cross(a, b) == Vector3{-3.0, 6.0, -3.0});
}

TEST_CASE("cross of parallel vectors is the zero vector", "[linear][vector3]") {
    const Vector3 a{1.0, 2.0, 3.0};
    const Vector3 b{2.0, 4.0, 6.0};
    CHECK(cross(a, b) == Vector3{0.0, 0.0, 0.0});
}

TEST_CASE("length survives extreme magnitudes", "[linear][vector3][degenerate]") {
    const Vector3 huge{1e200, 1e200, 0.0};
    CHECK(std::isfinite(huge.length()));
    CHECK(huge.length() == Approx(1.4142135623730951e200).epsilon(1e-12));

    const Vector3 tiny{1e-200, 1e-200, 0.0};
    CHECK(tiny.length() > 0.0);
    CHECK(tiny.length() == Approx(1.4142135623730951e-200).epsilon(1e-12));
}

TEST_CASE("length of the 1-2-2 vector is 3", "[linear][vector3]") {
    const Vector3 v{1.0, 2.0, 2.0};
    CHECK(v.length_squared() == 9.0);
    CHECK(v.length() == 3.0);
}

TEST_CASE("Vector3T is usable with float", "[linear][vector3]") {
    const Vector3f v{1.0f, 2.0f, 2.0f};
    CHECK(v.length() == 3.0f);
}

TEST_CASE("length of a vector containing infinity is infinity, not NaN",
          "[linear][vector3][degenerate]") {
    const double infinity = std::numeric_limits<double>::infinity();

    // 同 Vector2T：缩放写法若不特判，inf / inf 会算出 NaN。
    CHECK(Vector3{infinity, 1.0, 1.0}.length() == infinity);
    CHECK(Vector3{1.0, infinity, 1.0}.length() == infinity);
    CHECK(Vector3{infinity, infinity, infinity}.length() == infinity);
}
```

创建 `tests/linear/vector4_test.cpp`：

```cpp
#include <catch2/catch_test_macros.hpp>

#include <limits>

#include <GeoCore/linear/Vector4.hpp>

using GeoCore::linear::Vector4;

TEST_CASE("Vector4 arithmetic is component-wise", "[linear][vector4]") {
    const Vector4 a{1.0, 2.0, 3.0, 4.0};
    const Vector4 b{5.0, 6.0, 7.0, 8.0};

    CHECK(a + b == Vector4{6.0, 8.0, 10.0, 12.0});
    CHECK(b - a == Vector4{4.0, 4.0, 4.0, 4.0});
    CHECK(a * 2.0 == Vector4{2.0, 4.0, 6.0, 8.0});
}

TEST_CASE("Vector4 dot includes the w component", "[linear][vector4]") {
    const Vector4 a{1.0, 2.0, 3.0, 4.0};
    const Vector4 b{1.0, 1.0, 1.0, 1.0};
    CHECK(dot(a, b) == 10.0);
}

TEST_CASE("Vector4 length includes the w component", "[linear][vector4]") {
    const Vector4 v{1.0, 2.0, 2.0, 4.0};
    CHECK(v.length_squared() == 25.0);
    CHECK(v.length() == 5.0);
}

TEST_CASE("default-constructed Vector4 is zero", "[linear][vector4]") {
    const Vector4 v{};
    CHECK(v.w == 0.0);
    CHECK(v.length() == 0.0);
}

TEST_CASE("length of a vector containing infinity is infinity, not NaN",
          "[linear][vector4][degenerate]") {
    const double infinity = std::numeric_limits<double>::infinity();

    // 同 Vector2T：缩放写法若不特判，inf / inf 会算出 NaN。
    CHECK(Vector4{infinity, 1.0, 1.0, 1.0}.length() == infinity);
    CHECK(Vector4{1.0, 1.0, 1.0, infinity}.length() == infinity);
    CHECK(Vector4{infinity, infinity, infinity, infinity}.length() == infinity);
}
```

- [ ] **Step 2: 运行测试，确认失败**

```bash
cmake --build build/windows-vs --config Debug
```

Expected: 编译失败 —— 找不到 `GeoCore/linear/Vector3.hpp`。

- [ ] **Step 3: 实现 Vector3 与 Vector4**

创建 `include/GeoCore/linear/Vector3.hpp`：

```cpp
#pragma once

#include <concepts>
#include <cmath>

#include <GeoCore/core/Numeric.hpp>

namespace GeoCore::linear {

/// 三维向量：纯代数载体，不含任何几何语义。
template <typename Scalar>
struct Vector3T {
    using scalar_type = Scalar;

    // 同 Vector2T：不声明任何构造函数，以保持聚合性。
    Scalar x{};
    Scalar y{};
    Scalar z{};

    [[nodiscard]] constexpr Scalar length_squared() const noexcept {
        return x * x + y * y + z * z;
    }

    /// 欧几里得长度。先按最大分量缩放，避免中间量上溢或下溢。
    [[nodiscard]] Scalar length() const noexcept {
        const Scalar abs_x = core::absolute_value(x);
        const Scalar abs_y = core::absolute_value(y);
        const Scalar abs_z = core::absolute_value(z);
        const Scalar scale = abs_x > abs_y ? (abs_x > abs_z ? abs_x : abs_z)
                                           : (abs_y > abs_z ? abs_y : abs_z);
        if (scale == Scalar{0}) {
            return Scalar{0};
        }
        if (!core::is_finite(scale)) {
            // 同 Vector2T::length：含 ±inf（或全为 NaN）分量时，inf / inf 会
            // 算出 NaN；真实长度本就是 ±inf，直接返回它。
            return scale;
        }
        const Scalar scaled_x = x / scale;
        const Scalar scaled_y = y / scale;
        const Scalar scaled_z = z / scale;
        return scale * std::sqrt(scaled_x * scaled_x + scaled_y * scaled_y + scaled_z * scaled_z);
    }
};

using Vector3 = Vector3T<double>;
using Vector3f = Vector3T<float>;

// ---- 运算符 ----

template <typename Scalar>
[[nodiscard]] constexpr Vector3T<Scalar> operator+(Vector3T<Scalar> a, Vector3T<Scalar> b) noexcept {
    return Vector3T<Scalar>{a.x + b.x, a.y + b.y, a.z + b.z};
}

template <typename Scalar>
[[nodiscard]] constexpr Vector3T<Scalar> operator-(Vector3T<Scalar> v) noexcept {
    return Vector3T<Scalar>{-v.x, -v.y, -v.z};
}

template <typename Scalar>
[[nodiscard]] constexpr Vector3T<Scalar> operator-(Vector3T<Scalar> a, Vector3T<Scalar> b) noexcept {
    return Vector3T<Scalar>{a.x - b.x, a.y - b.y, a.z - b.z};
}

template <typename Scalar, typename Factor>
    requires std::convertible_to<Factor, Scalar>
[[nodiscard]] constexpr Vector3T<Scalar> operator*(Vector3T<Scalar> v, Factor factor) noexcept {
    const auto scale = static_cast<Scalar>(factor);
    return Vector3T<Scalar>{v.x * scale, v.y * scale, v.z * scale};
}

template <typename Scalar, typename Factor>
    requires std::convertible_to<Factor, Scalar>
[[nodiscard]] constexpr Vector3T<Scalar> operator*(Factor factor, Vector3T<Scalar> v) noexcept {
    return v * factor;
}

template <typename Scalar, typename Divisor>
    requires std::convertible_to<Divisor, Scalar>
[[nodiscard]] constexpr Vector3T<Scalar> operator/(Vector3T<Scalar> v, Divisor divisor) noexcept {
    const auto scale = static_cast<Scalar>(divisor);
    return Vector3T<Scalar>{v.x / scale, v.y / scale, v.z / scale};
}

template <typename Scalar>
[[nodiscard]] constexpr bool operator==(Vector3T<Scalar> a, Vector3T<Scalar> b) noexcept {
    return a.x == b.x && a.y == b.y && a.z == b.z;
}

// ---- 几何量 ----

template <typename Scalar>
[[nodiscard]] constexpr Scalar dot(Vector3T<Scalar> a, Vector3T<Scalar> b) noexcept {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

/// 三维叉积。结果垂直于两个输入，方向遵循右手定则。
/// 两向量平行（含任一为零向量）时结果为零向量。
template <typename Scalar>
[[nodiscard]] constexpr Vector3T<Scalar> cross(Vector3T<Scalar> a, Vector3T<Scalar> b) noexcept {
    return Vector3T<Scalar>{
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x,
    };
}

} // namespace GeoCore::linear
```

创建 `include/GeoCore/linear/Vector4.hpp`：

```cpp
#pragma once

#include <concepts>
#include <cmath>

#include <GeoCore/core/Numeric.hpp>

namespace GeoCore::linear {

/// 四维向量：纯代数载体。齐次坐标与四元数的底层表示都用它，
/// 因此本层刻意不赋予它任何几何语义。
template <typename Scalar>
struct Vector4T {
    using scalar_type = Scalar;

    // 同 Vector2T：不声明任何构造函数，以保持聚合性。
    Scalar x{};
    Scalar y{};
    Scalar z{};
    Scalar w{};

    [[nodiscard]] constexpr Scalar length_squared() const noexcept {
        return x * x + y * y + z * z + w * w;
    }

    /// 欧几里得长度。先按最大分量缩放，避免中间量上溢或下溢。
    [[nodiscard]] Scalar length() const noexcept {
        const Scalar abs_x = core::absolute_value(x);
        const Scalar abs_y = core::absolute_value(y);
        const Scalar abs_z = core::absolute_value(z);
        const Scalar abs_w = core::absolute_value(w);

        Scalar scale = abs_x;
        if (abs_y > scale) { scale = abs_y; }
        if (abs_z > scale) { scale = abs_z; }
        if (abs_w > scale) { scale = abs_w; }

        if (scale == Scalar{0}) {
            return Scalar{0};
        }
        if (!core::is_finite(scale)) {
            // 同 Vector2T::length：含 ±inf（或全为 NaN）分量时，inf / inf 会
            // 算出 NaN；真实长度本就是 ±inf，直接返回它。
            return scale;
        }
        const Scalar scaled_x = x / scale;
        const Scalar scaled_y = y / scale;
        const Scalar scaled_z = z / scale;
        const Scalar scaled_w = w / scale;
        return scale * std::sqrt(scaled_x * scaled_x + scaled_y * scaled_y
                                 + scaled_z * scaled_z + scaled_w * scaled_w);
    }
};

using Vector4 = Vector4T<double>;
using Vector4f = Vector4T<float>;

// ---- 运算符 ----

template <typename Scalar>
[[nodiscard]] constexpr Vector4T<Scalar> operator+(Vector4T<Scalar> a, Vector4T<Scalar> b) noexcept {
    return Vector4T<Scalar>{a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w};
}

template <typename Scalar>
[[nodiscard]] constexpr Vector4T<Scalar> operator-(Vector4T<Scalar> v) noexcept {
    return Vector4T<Scalar>{-v.x, -v.y, -v.z, -v.w};
}

template <typename Scalar>
[[nodiscard]] constexpr Vector4T<Scalar> operator-(Vector4T<Scalar> a, Vector4T<Scalar> b) noexcept {
    return Vector4T<Scalar>{a.x - b.x, a.y - b.y, a.z - b.z, a.w - b.w};
}

template <typename Scalar, typename Factor>
    requires std::convertible_to<Factor, Scalar>
[[nodiscard]] constexpr Vector4T<Scalar> operator*(Vector4T<Scalar> v, Factor factor) noexcept {
    const auto scale = static_cast<Scalar>(factor);
    return Vector4T<Scalar>{v.x * scale, v.y * scale, v.z * scale, v.w * scale};
}

template <typename Scalar, typename Factor>
    requires std::convertible_to<Factor, Scalar>
[[nodiscard]] constexpr Vector4T<Scalar> operator*(Factor factor, Vector4T<Scalar> v) noexcept {
    return v * factor;
}

template <typename Scalar, typename Divisor>
    requires std::convertible_to<Divisor, Scalar>
[[nodiscard]] constexpr Vector4T<Scalar> operator/(Vector4T<Scalar> v, Divisor divisor) noexcept {
    const auto scale = static_cast<Scalar>(divisor);
    return Vector4T<Scalar>{v.x / scale, v.y / scale, v.z / scale, v.w / scale};
}

template <typename Scalar>
[[nodiscard]] constexpr bool operator==(Vector4T<Scalar> a, Vector4T<Scalar> b) noexcept {
    return a.x == b.x && a.y == b.y && a.z == b.z && a.w == b.w;
}

// ---- 几何量 ----

template <typename Scalar>
[[nodiscard]] constexpr Scalar dot(Vector4T<Scalar> a, Vector4T<Scalar> b) noexcept {
    return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
}

} // namespace GeoCore::linear
```

修改 `include/GeoCore/GeoCore.hpp`，在 `Vector2.hpp` 之后追加：

```cpp
#include <GeoCore/linear/Vector3.hpp>
#include <GeoCore/linear/Vector4.hpp>
```

- [ ] **Step 4: 运行测试并提交**

```bash
cmake --build build/windows-vs --config Debug
ctest --test-dir build/windows-vs -C Debug --output-on-failure
```

Expected: 全部通过。

```bash
git add include/GeoCore/linear/Vector3.hpp include/GeoCore/linear/Vector4.hpp include/GeoCore/GeoCore.hpp tests/linear/vector3_test.cpp tests/linear/vector4_test.cpp
git commit -m "feat(linear): add Vector3T and Vector4T"
```

---

### Task 5: linear —— UnitVector2 与 UnitVector3

这是全库"不变量由类型承载"原则的第一次落地，也是 `Tolerance` 的第一个真实用例。核心设计约束：**运算的返回类型必须如实反映该运算是否保持单位长度**。

`UnitVector3T * 2.0` 返回 `Vector3T`（不再是单位向量），而 `-u` 返回 `UnitVector3T`（仍是单位向量）。类型系统在这里替使用者记住哪些运算安全。

**Files:**
- Create: `include/GeoCore/linear/UnitVector2.hpp`
- Create: `include/GeoCore/linear/UnitVector3.hpp`
- Create: `tests/linear/unit_vector2_test.cpp`
- Create: `tests/linear/unit_vector3_test.cpp`
- Modify: `include/GeoCore/GeoCore.hpp`（追加 include）

**Interfaces:**
- Consumes: `Vector2T` / `Vector3T`（Task 3、4）、`GeoCore::core::Tolerance`（Task 2）
- Produces:
  - `template <typename Scalar> class UnitVector2T` / `UnitVector3T`，别名 `UnitVector2`、`UnitVector2f`、`UnitVector3`、`UnitVector3f`
  - 静态工厂 `from_normalized_unchecked(VectorNT<S>) -> UnitVectorNT<S>`（前置条件：已归一化）
  - 成员 `as_vector()`、`x()`、`y()`、`z()`
  - 自由函数 `normalize(VectorNT<S>, core::Tolerance = {}) -> std::optional<UnitVectorNT<S>>`
  - 运算符：一元 `-`（保持单位）、`UnitVectorNT * scalar -> VectorNT`、`scalar * UnitVectorNT -> VectorNT`、`==`
  - `dot(UnitVectorNT, UnitVectorNT) -> Scalar`；`cross(UnitVector2T, UnitVector2T) -> Scalar`、`cross(UnitVector3T, UnitVector3T) -> Vector3T`

- [ ] **Step 1: 写失败测试**

创建 `tests/linear/unit_vector3_test.cpp`：

```cpp
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <limits>
#include <optional>
#include <type_traits>

#include <GeoCore/linear/UnitVector3.hpp>

using Catch::Approx;

using GeoCore::core::Tolerance;
using GeoCore::linear::normalize;
using GeoCore::linear::UnitVector3;
using GeoCore::linear::Vector3;

TEST_CASE("normalize produces a unit-length vector", "[linear][unitvector3]") {
    const auto result = normalize(Vector3{3.0, 4.0, 0.0});

    REQUIRE(result.has_value());
    CHECK(result->x() == Approx(0.6));
    CHECK(result->y() == Approx(0.8));
    CHECK(result->z() == Approx(0.0));
    CHECK(result->as_vector().length() == Approx(1.0));
}

TEST_CASE("normalize rejects the zero vector", "[linear][unitvector3][degenerate]") {
    const auto result = normalize(Vector3{0.0, 0.0, 0.0});

    // 必须返回 nullopt，而不是含 NaN 的单位向量
    CHECK_FALSE(result.has_value());
}

TEST_CASE("normalize rejects vectors below the tolerance",
          "[linear][unitvector3][degenerate]") {
    CHECK_FALSE(normalize(Vector3{1e-15, 0.0, 0.0}).has_value());
}

TEST_CASE("a zero tolerance rejects only the exact zero vector",
          "[linear][unitvector3][degenerate]") {
    const Tolerance exact{0.0, 0.0};

    CHECK_FALSE(normalize(Vector3{0.0, 0.0, 0.0}, exact).has_value());
    CHECK(normalize(Vector3{1e-300, 0.0, 0.0}, exact).has_value());
}

TEST_CASE("normalize rejects vectors below the absolute tolerance",
          "[linear][unitvector3][degenerate]") {
    // 1e-200 的长度远小于默认绝对容差 1e-12，因此按「视为零」处理。
    // 需要在这个量级上工作时，必须显式传入更小的容差 —— 这正是
    // 容差显式传参原则的预期后果。
    CHECK_FALSE(normalize(Vector3{1e-200, 0.0, 0.0}).has_value());
}

TEST_CASE("normalize scales correctly at extreme magnitudes",
          "[linear][unitvector3][degenerate]") {
    // 关闭容差判断，单独考察重缩放后的计算本身是否上溢或下溢
    const Tolerance exact{0.0, 0.0};

    const auto huge = normalize(Vector3{1e200, 1e200, 0.0}, exact);
    REQUIRE(huge.has_value());
    CHECK(huge->x() == Approx(0.7071067811865476));
    CHECK(huge->y() == Approx(0.7071067811865476));

    const auto tiny = normalize(Vector3{1e-200, 1e-200, 0.0}, exact);
    REQUIRE(tiny.has_value());
    CHECK(tiny->x() == Approx(0.7071067811865476));
    CHECK(tiny->y() == Approx(0.7071067811865476));
}

TEST_CASE("negation preserves the unit invariant", "[linear][unitvector3]") {
    const auto u = normalize(Vector3{1.0, 2.0, 2.0});
    REQUIRE(u.has_value());

    const UnitVector3 negated = -*u;
    CHECK(negated.x() == Approx(-1.0 / 3.0));
    CHECK(negated.y() == Approx(-2.0 / 3.0));
    CHECK(negated.z() == Approx(-2.0 / 3.0));
}

TEST_CASE("scaling a unit vector yields a plain Vector3",
          "[linear][unitvector3]") {
    const auto u = normalize(Vector3{0.0, 0.0, 1.0});
    REQUIRE(u.has_value());

    // 关键的类型契约：缩放后不再是单位向量，返回类型必须改变
    STATIC_REQUIRE(std::is_same_v<decltype(*u * 2.0), Vector3>);
    CHECK(*u * 2.0 == Vector3{0.0, 0.0, 2.0});
    CHECK(2.0 * *u == Vector3{0.0, 0.0, 2.0});
}

TEST_CASE("dot of two unit vectors is the cosine of the angle",
          "[linear][unitvector3]") {
    const auto a = normalize(Vector3{1.0, 0.0, 0.0});
    const auto b = normalize(Vector3{1.0, 1.0, 0.0});
    REQUIRE(a.has_value());
    REQUIRE(b.has_value());

    CHECK(dot(*a, *b) == Approx(0.7071067811865476));
    CHECK(dot(*a, *a) == Approx(1.0));
}

TEST_CASE("cross of two unit vectors is a plain Vector3",
          "[linear][unitvector3]") {
    const auto x = normalize(Vector3{1.0, 0.0, 0.0});
    const auto y = normalize(Vector3{0.0, 1.0, 0.0});
    REQUIRE(x.has_value());
    REQUIRE(y.has_value());

    // 数学上叉积仍是单位向量，但两向量接近平行时长度会退化到 0，
    // 无法维持不变量，故返回类型刻意是 Vector3。
    STATIC_REQUIRE(std::is_same_v<decltype(cross(*x, *y)), Vector3>);
    CHECK(cross(*x, *y).z == Approx(1.0));

    const auto parallel = normalize(Vector3{1.0, 0.0, 0.0});
    REQUIRE(parallel.has_value());
    CHECK(cross(*x, *parallel).length() == Approx(0.0));
}

TEST_CASE("normalize rejects non-finite input instead of returning a NaN unit vector",
          "[linear][unitvector3][degenerate]") {
    const double infinity = std::numeric_limits<double>::infinity();
    const double not_a_number = std::numeric_limits<double>::quiet_NaN();

    // 一个 has_value() 为真、内容却是 NaN 的「单位向量」是本库最不该交出的
    // 返回值：调用者无从察觉，而 NaN 会一路污染 dot / cross 与所有容差判断。
    CHECK_FALSE(normalize(Vector3{infinity, 1.0, 1.0}).has_value());
    CHECK_FALSE(normalize(Vector3{1.0, infinity, 1.0}).has_value());
    CHECK_FALSE(normalize(Vector3{infinity, infinity, infinity}).has_value());

    // NaN 输入比无穷更常见：任何上游的 0/0 或 inf - inf 都会落到这里
    CHECK_FALSE(normalize(Vector3{not_a_number, 1.0, 1.0}).has_value());
    CHECK_FALSE(normalize(Vector3{not_a_number, not_a_number, not_a_number}).has_value());
}
```

创建 `tests/linear/unit_vector2_test.cpp`：

```cpp
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <limits>

#include <GeoCore/linear/UnitVector2.hpp>

using Catch::Approx;

using GeoCore::linear::normalize;
using GeoCore::linear::Vector2;

TEST_CASE("normalize produces a unit-length 2D vector", "[linear][unitvector2]") {
    const auto result = normalize(Vector2{3.0, 4.0});

    REQUIRE(result.has_value());
    CHECK(result->x() == Approx(0.6));
    CHECK(result->y() == Approx(0.8));
    CHECK(result->as_vector().length() == Approx(1.0));
}

TEST_CASE("normalize rejects the 2D zero vector", "[linear][unitvector2][degenerate]") {
    CHECK_FALSE(normalize(Vector2{0.0, 0.0}).has_value());
}

TEST_CASE("2D cross of unit vectors is the sine of the angle",
          "[linear][unitvector2]") {
    const auto x = normalize(Vector2{1.0, 0.0});
    const auto y = normalize(Vector2{0.0, 1.0});
    REQUIRE(x.has_value());
    REQUIRE(y.has_value());

    CHECK(cross(*x, *y) == Approx(1.0));
    CHECK(cross(*y, *x) == Approx(-1.0));
    CHECK(cross(*x, *x) == Approx(0.0));
}

TEST_CASE("normalize rejects non-finite input instead of returning a NaN unit vector",
          "[linear][unitvector2][degenerate]") {
    const double infinity = std::numeric_limits<double>::infinity();
    const double not_a_number = std::numeric_limits<double>::quiet_NaN();

    // 理由同 UnitVector3T：交出一个内容为 NaN 的「单位向量」比返回 nullopt 危险得多。
    CHECK_FALSE(normalize(Vector2{infinity, 1.0}).has_value());
    CHECK_FALSE(normalize(Vector2{1.0, infinity}).has_value());
    CHECK_FALSE(normalize(Vector2{infinity, infinity}).has_value());

    CHECK_FALSE(normalize(Vector2{not_a_number, 1.0}).has_value());
    CHECK_FALSE(normalize(Vector2{not_a_number, not_a_number}).has_value());
}
```

- [ ] **Step 2: 运行测试，确认失败**

```bash
cmake --build build/windows-vs --config Debug
```

Expected: 编译失败 —— 找不到 `GeoCore/linear/UnitVector3.hpp`。

- [ ] **Step 3: 实现 UnitVector3**

创建 `include/GeoCore/linear/UnitVector3.hpp`：

```cpp
#pragma once

#include <optional>

#include <GeoCore/core/Tolerance.hpp>
#include <GeoCore/linear/Vector3.hpp>

namespace GeoCore::linear {

/// 单位向量的三维类型。
///
/// 不变量是**弱不变量**：|v| 在浮点舍入误差内等于 1，而不是精确等于 1。
/// 它保证的是「已归一化过一次」，因此不能用作依赖精确单位长度的判定。
///
/// 运算的返回类型如实反映是否保持该不变量：
///   -u               -> UnitVector3T   （保持）
///   u * scalar       -> Vector3T       （缩放后不再是单位向量）
///   u + u            -> Vector3T       （和一般不是单位向量）
///   cross(u, u)      -> Vector3T       （平行时退化为零向量）
template <typename Scalar>
class UnitVector3T {
public:
    using scalar_type = Scalar;

    /// 前置条件：normalized 已是单位向量。命名即警告。
    ///
    /// 违反前置条件不会立即出错，但会让本类型的不变量永久失效，
    /// 后续依赖该不变量的代码将得到错误结果。请优先使用 normalize()。
    [[nodiscard]] static constexpr UnitVector3T from_normalized_unchecked(
        Vector3T<Scalar> normalized) noexcept {
        return UnitVector3T{normalized};
    }

    [[nodiscard]] constexpr Vector3T<Scalar> as_vector() const noexcept {
        return value_;
    }

    [[nodiscard]] constexpr Scalar x() const noexcept { return value_.x; }
    [[nodiscard]] constexpr Scalar y() const noexcept { return value_.y; }
    [[nodiscard]] constexpr Scalar z() const noexcept { return value_.z; }

    [[nodiscard]] constexpr bool operator==(const UnitVector3T&) const noexcept = default;

private:
    explicit constexpr UnitVector3T(Vector3T<Scalar> value) noexcept : value_(value) {}

    Vector3T<Scalar> value_;
};

using UnitVector3 = UnitVector3T<double>;
using UnitVector3f = UnitVector3T<float>;

/// 归一化。向量长度在给定容差下可视为零、或本身不是有限值时返回
/// std::nullopt，因此调用者无法得到含 NaN 的单位向量。
template <typename Scalar>
[[nodiscard]] std::optional<UnitVector3T<Scalar>> normalize(
    Vector3T<Scalar> v, core::Tolerance tolerance = {}) noexcept {
    const Scalar length = v.length();
    // 非有限长度同样返回 nullopt。容差判断对 ±inf 与 NaN 一律返回 false
    // （`is_zero` 刻意不把溢出量静默归类为零），若就此放行，本函数会交出一个
    // has_value() 为真、内容却是 NaN 的「单位向量」：调用者无从察觉，而 NaN
    // 会一路污染 dot / cross 与每一个容差比较 —— 那些比较对 NaN 都返回 false，
    // 下游几何代码会静默走「否」分支。NaN 输入比无穷更常见：任何上游的
    // 0/0 或 inf - inf 都会落到这里。
    if (!core::is_finite(length) || tolerance.is_zero(static_cast<double>(length))) {
        return std::nullopt;
    }
    return UnitVector3T<Scalar>::from_normalized_unchecked(v / length);
}

// ---- 保持不变量的运算 ----

template <typename Scalar>
[[nodiscard]] constexpr UnitVector3T<Scalar> operator-(UnitVector3T<Scalar> u) noexcept {
    return UnitVector3T<Scalar>::from_normalized_unchecked(-u.as_vector());
}

// ---- 不保持不变量的运算：返回类型相应改变 ----

template <typename Scalar, typename Factor>
    requires std::convertible_to<Factor, Scalar>
[[nodiscard]] constexpr Vector3T<Scalar> operator*(UnitVector3T<Scalar> u, Factor factor) noexcept {
    return u.as_vector() * factor;
}

template <typename Scalar, typename Factor>
    requires std::convertible_to<Factor, Scalar>
[[nodiscard]] constexpr Vector3T<Scalar> operator*(Factor factor, UnitVector3T<Scalar> u) noexcept {
    return u.as_vector() * factor;
}

template <typename Scalar>
[[nodiscard]] constexpr Vector3T<Scalar> operator+(
    UnitVector3T<Scalar> a, UnitVector3T<Scalar> b) noexcept {
    return a.as_vector() + b.as_vector();
}

template <typename Scalar>
[[nodiscard]] constexpr Vector3T<Scalar> operator-(
    UnitVector3T<Scalar> a, UnitVector3T<Scalar> b) noexcept {
    return a.as_vector() - b.as_vector();
}

template <typename Scalar>
[[nodiscard]] constexpr Scalar dot(UnitVector3T<Scalar> a, UnitVector3T<Scalar> b) noexcept {
    return dot(a.as_vector(), b.as_vector());
}

/// 叉积。结果不保证是单位向量（两向量平行时为零向量），故返回 Vector3T。
template <typename Scalar>
[[nodiscard]] constexpr Vector3T<Scalar> cross(UnitVector3T<Scalar> a, UnitVector3T<Scalar> b) noexcept {
    return cross(a.as_vector(), b.as_vector());
}

} // namespace GeoCore::linear
```

创建 `include/GeoCore/linear/UnitVector2.hpp`：

```cpp
#pragma once

#include <optional>

#include <GeoCore/core/Tolerance.hpp>
#include <GeoCore/linear/Vector2.hpp>

namespace GeoCore::linear {

/// 单位向量的二维类型。语义与 UnitVector3T 完全一致，
/// 差异仅在维度与二维叉积返回标量。
template <typename Scalar>
class UnitVector2T {
public:
    using scalar_type = Scalar;

    /// 前置条件：normalized 已是单位向量。命名即警告。
    [[nodiscard]] static constexpr UnitVector2T from_normalized_unchecked(
        Vector2T<Scalar> normalized) noexcept {
        return UnitVector2T{normalized};
    }

    [[nodiscard]] constexpr Vector2T<Scalar> as_vector() const noexcept {
        return value_;
    }

    [[nodiscard]] constexpr Scalar x() const noexcept { return value_.x; }
    [[nodiscard]] constexpr Scalar y() const noexcept { return value_.y; }

    [[nodiscard]] constexpr bool operator==(const UnitVector2T&) const noexcept = default;

private:
    explicit constexpr UnitVector2T(Vector2T<Scalar> value) noexcept : value_(value) {}

    Vector2T<Scalar> value_;
};

using UnitVector2 = UnitVector2T<double>;
using UnitVector2f = UnitVector2T<float>;

/// 归一化。语义与 UnitVector3T 的同名函数一致：长度在给定容差下可视为零、
/// 或本身不是有限值时返回 std::nullopt。
template <typename Scalar>
[[nodiscard]] std::optional<UnitVector2T<Scalar>> normalize(
    Vector2T<Scalar> v, core::Tolerance tolerance = {}) noexcept {
    const Scalar length = v.length();
    // 非有限长度同样返回 nullopt，理由见 UnitVector3T 的同名函数。
    if (!core::is_finite(length) || tolerance.is_zero(static_cast<double>(length))) {
        return std::nullopt;
    }
    return UnitVector2T<Scalar>::from_normalized_unchecked(v / length);
}

template <typename Scalar>
[[nodiscard]] constexpr UnitVector2T<Scalar> operator-(UnitVector2T<Scalar> u) noexcept {
    return UnitVector2T<Scalar>::from_normalized_unchecked(-u.as_vector());
}

template <typename Scalar, typename Factor>
    requires std::convertible_to<Factor, Scalar>
[[nodiscard]] constexpr Vector2T<Scalar> operator*(UnitVector2T<Scalar> u, Factor factor) noexcept {
    return u.as_vector() * factor;
}

template <typename Scalar, typename Factor>
    requires std::convertible_to<Factor, Scalar>
[[nodiscard]] constexpr Vector2T<Scalar> operator*(Factor factor, UnitVector2T<Scalar> u) noexcept {
    return u.as_vector() * factor;
}

template <typename Scalar>
[[nodiscard]] constexpr Vector2T<Scalar> operator+(
    UnitVector2T<Scalar> a, UnitVector2T<Scalar> b) noexcept {
    return a.as_vector() + b.as_vector();
}

template <typename Scalar>
[[nodiscard]] constexpr Vector2T<Scalar> operator-(
    UnitVector2T<Scalar> a, UnitVector2T<Scalar> b) noexcept {
    return a.as_vector() - b.as_vector();
}

template <typename Scalar>
[[nodiscard]] constexpr Scalar dot(UnitVector2T<Scalar> a, UnitVector2T<Scalar> b) noexcept {
    return dot(a.as_vector(), b.as_vector());
}

/// 二维叉积，即两单位向量夹角的正弦。
template <typename Scalar>
[[nodiscard]] constexpr Scalar cross(UnitVector2T<Scalar> a, UnitVector2T<Scalar> b) noexcept {
    return cross(a.as_vector(), b.as_vector());
}

} // namespace GeoCore::linear
```

修改 `include/GeoCore/GeoCore.hpp`，在 `Vector4.hpp` 之后追加：

```cpp
#include <GeoCore/linear/UnitVector2.hpp>
#include <GeoCore/linear/UnitVector3.hpp>
```

- [ ] **Step 4: 运行测试并提交**

```bash
cmake --build build/windows-vs --config Debug
ctest --test-dir build/windows-vs -C Debug --output-on-failure
```

Expected: 全部通过。

```bash
git add include/GeoCore/linear/UnitVector2.hpp include/GeoCore/linear/UnitVector3.hpp include/GeoCore/GeoCore.hpp tests/linear/unit_vector2_test.cpp tests/linear/unit_vector3_test.cpp
git commit -m "feat(linear): add UnitVector2T and UnitVector3T with invariant-preserving operations"
```

---

### Task 6: linear —— Matrix 家族基础

`MatrixT<Scalar, N>` 一个模板覆盖 2×2 / 3×3 / 4×4，行主序存储。行列式与求逆因为需要按尺寸写出闭式解，放在 Task 7 单独处理。

**Files:**
- Create: `include/GeoCore/linear/Matrix.hpp`
- Create: `tests/linear/matrix_test.cpp`
- Modify: `include/GeoCore/GeoCore.hpp`（追加 include）

**Interfaces:**
- Consumes: `Vector2T` / `Vector3T` / `Vector4T`（Task 3、4）
- Produces:
  - `template <typename Scalar, int N> struct MatrixT`，成员 `Scalar data[N][N]{}`，`operator()(row, column)`，常量 `dimension`
  - 别名 `Matrix2`、`Matrix2f`、`Matrix3`、`Matrix3f`、`Matrix4`、`Matrix4f`
  - 自由函数 `identity<Scalar, N>() -> MatrixT`、`transpose(MatrixT) -> MatrixT`
  - 运算符 `+`、`-`、`*`（矩阵×矩阵、矩阵×标量）、`==`
  - 矩阵 × 向量：`MatrixT<S,2> * Vector2T<S>`、`MatrixT<S,3> * Vector3T<S>`、`MatrixT<S,4> * Vector4T<S>`

- [ ] **Step 1: 写失败测试**

创建 `tests/linear/matrix_test.cpp`：

```cpp
#include <catch2/catch_test_macros.hpp>

#include <GeoCore/linear/Matrix.hpp>

using GeoCore::linear::identity;
using GeoCore::linear::Matrix2;
using GeoCore::linear::Matrix3;
using GeoCore::linear::Matrix4;
using GeoCore::linear::transpose;
using GeoCore::linear::Vector2;
using GeoCore::linear::Vector3;

TEST_CASE("MatrixT is zero-initialized and supports aggregate init",
          "[linear][matrix]") {
    const Matrix2 zero{};
    CHECK(zero(0, 0) == 0.0);
    CHECK(zero(1, 1) == 0.0);

    const Matrix2 m{{1.0, 2.0}, {3.0, 4.0}};
    CHECK(m(0, 0) == 1.0);
    CHECK(m(0, 1) == 2.0);
    CHECK(m(1, 0) == 3.0);
    CHECK(m(1, 1) == 4.0);
}

TEST_CASE("identity is the multiplicative unit", "[linear][matrix]") {
    const Matrix3 unit = identity<double, 3>();

    const Matrix3 m{{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}, {7.0, 8.0, 9.0}};
    CHECK(unit * m == m);
    CHECK(m * unit == m);
}

TEST_CASE("matrix multiplication follows the row-column rule",
          "[linear][matrix]") {
    const Matrix2 a{{1.0, 2.0}, {3.0, 4.0}};
    const Matrix2 b{{5.0, 6.0}, {7.0, 8.0}};

    // [1 2] [5 6]   [19 22]
    // [3 4] [7 8] = [43 50]
    CHECK(a * b == Matrix2{{19.0, 22.0}, {43.0, 50.0}});
}

TEST_CASE("matrix multiplication is not commutative", "[linear][matrix]") {
    const Matrix2 a{{1.0, 2.0}, {3.0, 4.0}};
    const Matrix2 b{{5.0, 6.0}, {7.0, 8.0}};

    CHECK_FALSE(a * b == b * a);
}

TEST_CASE("transpose swaps rows and columns", "[linear][matrix]") {
    const Matrix2 m{{1.0, 2.0}, {3.0, 4.0}};
    CHECK(transpose(m) == Matrix2{{1.0, 3.0}, {2.0, 4.0}});

    // 转置两次回到自身
    CHECK(transpose(transpose(m)) == m);

    // 转置与乘法反交换
    const Matrix2 n{{5.0, 6.0}, {7.0, 8.0}};
    CHECK(transpose(m * n) == transpose(n) * transpose(m));
}

TEST_CASE("matrix addition and subtraction are component-wise",
          "[linear][matrix]") {
    const Matrix2 a{{1.0, 2.0}, {3.0, 4.0}};
    const Matrix2 b{{5.0, 6.0}, {7.0, 8.0}};

    CHECK(a + b == Matrix2{{6.0, 8.0}, {10.0, 12.0}});
    CHECK(b - a == Matrix2{{4.0, 4.0}, {4.0, 4.0}});
}

TEST_CASE("matrix times vector applies the row-column rule",
          "[linear][matrix]") {
    const Matrix2 m{{1.0, 2.0}, {3.0, 4.0}};
    const Vector2 v{5.0, 6.0};

    // [1 2] [5]   [17]
    // [3 4] [6] = [39]
    CHECK(m * v == Vector2{17.0, 39.0});
}

TEST_CASE("4x4 matrix times 4D vector", "[linear][matrix]") {
    const Matrix4 unit = identity<double, 4>();
    const GeoCore::linear::Vector4 v{1.0, 2.0, 3.0, 4.0};

    CHECK(unit * v == v);
}

TEST_CASE("3x3 rotation-like matrix transforms a vector", "[linear][matrix]") {
    // 绕 z 轴旋转 90°： (x, y) -> (-y, x)
    const Matrix3 rotate{{0.0, -1.0, 0.0}, {1.0, 0.0, 0.0}, {0.0, 0.0, 1.0}};
    const Vector3 v{1.0, 0.0, 0.0};

    const Vector3 rotated = rotate * v;
    CHECK(rotated.x == 0.0);
    CHECK(rotated.y == 1.0);
    CHECK(rotated.z == 0.0);
}
```

- [ ] **Step 2: 运行测试，确认失败**

```bash
cmake --build build/windows-vs --config Debug
```

Expected: 编译失败 —— 找不到 `GeoCore/linear/Matrix.hpp`。

- [ ] **Step 3: 实现 Matrix**

创建 `include/GeoCore/linear/Matrix.hpp`：

```cpp
#pragma once

#include <GeoCore/linear/Vector2.hpp>
#include <GeoCore/linear/Vector3.hpp>
#include <GeoCore/linear/Vector4.hpp>

namespace GeoCore::linear {

/// N×N 方阵，行主序存储。
///
/// 用二维数组而非命名字段：4×4 有 16 个元素，逐一命名反而降低可读性。
/// 通过 operator()(row, column) 访问以保持「先行后列」的一致读法。
template <typename Scalar, int N>
struct MatrixT {
    using scalar_type = Scalar;
    static constexpr int dimension = N;

    Scalar data[N][N]{};

    [[nodiscard]] constexpr Scalar& operator()(int row, int column) noexcept {
        return data[row][column];
    }

    [[nodiscard]] constexpr Scalar operator()(int row, int column) const noexcept {
        return data[row][column];
    }
};

using Matrix2 = MatrixT<double, 2>;
using Matrix2f = MatrixT<float, 2>;
using Matrix3 = MatrixT<double, 3>;
using Matrix3f = MatrixT<float, 3>;
using Matrix4 = MatrixT<double, 4>;
using Matrix4f = MatrixT<float, 4>;

// ---- 构造与变换 ----

template <typename Scalar, int N>
[[nodiscard]] constexpr MatrixT<Scalar, N> identity() noexcept {
    MatrixT<Scalar, N> result{};
    for (int i = 0; i < N; ++i) {
        result.data[i][i] = Scalar{1};
    }
    return result;
}

template <typename Scalar, int N>
[[nodiscard]] constexpr MatrixT<Scalar, N> transpose(const MatrixT<Scalar, N>& m) noexcept {
    MatrixT<Scalar, N> result{};
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) {
            result.data[i][j] = m.data[j][i];
        }
    }
    return result;
}

// ---- 运算符 ----

template <typename Scalar, int N>
[[nodiscard]] constexpr MatrixT<Scalar, N> operator+(
    const MatrixT<Scalar, N>& a, const MatrixT<Scalar, N>& b) noexcept {
    MatrixT<Scalar, N> result{};
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) {
            result.data[i][j] = a.data[i][j] + b.data[i][j];
        }
    }
    return result;
}

template <typename Scalar, int N>
[[nodiscard]] constexpr MatrixT<Scalar, N> operator-(
    const MatrixT<Scalar, N>& a, const MatrixT<Scalar, N>& b) noexcept {
    MatrixT<Scalar, N> result{};
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) {
            result.data[i][j] = a.data[i][j] - b.data[i][j];
        }
    }
    return result;
}

template <typename Scalar, int N>
[[nodiscard]] constexpr MatrixT<Scalar, N> operator*(
    const MatrixT<Scalar, N>& a, const MatrixT<Scalar, N>& b) noexcept {
    MatrixT<Scalar, N> result{};
    for (int i = 0; i < N; ++i) {
        for (int k = 0; k < N; ++k) {
            const Scalar factor = a.data[i][k];
            for (int j = 0; j < N; ++j) {
                result.data[i][j] += factor * b.data[k][j];
            }
        }
    }
    return result;
}

template <typename Scalar, int N, typename Factor>
    requires std::convertible_to<Factor, Scalar>
[[nodiscard]] constexpr MatrixT<Scalar, N> operator*(
    const MatrixT<Scalar, N>& m, Factor factor) noexcept {
    const auto scale = static_cast<Scalar>(factor);
    MatrixT<Scalar, N> result{};
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) {
            result.data[i][j] = m.data[i][j] * scale;
        }
    }
    return result;
}

template <typename Scalar, int N>
[[nodiscard]] constexpr bool operator==(
    const MatrixT<Scalar, N>& a, const MatrixT<Scalar, N>& b) noexcept {
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) {
            if (a.data[i][j] != b.data[i][j]) {
                return false;
            }
        }
    }
    return true;
}

// ---- 矩阵 × 向量 ----
//
// 三个维度各写一个重载，而不是写一个泛型版本：泛型版本需要把向量
// 也参数化，会立刻把调用点拖进模板推导的泥潭，得不偿失。

template <typename Scalar>
[[nodiscard]] constexpr Vector2T<Scalar> operator*(
    const MatrixT<Scalar, 2>& m, Vector2T<Scalar> v) noexcept {
    return Vector2T<Scalar>{
        m.data[0][0] * v.x + m.data[0][1] * v.y,
        m.data[1][0] * v.x + m.data[1][1] * v.y,
    };
}

template <typename Scalar>
[[nodiscard]] constexpr Vector3T<Scalar> operator*(
    const MatrixT<Scalar, 3>& m, Vector3T<Scalar> v) noexcept {
    return Vector3T<Scalar>{
        m.data[0][0] * v.x + m.data[0][1] * v.y + m.data[0][2] * v.z,
        m.data[1][0] * v.x + m.data[1][1] * v.y + m.data[1][2] * v.z,
        m.data[2][0] * v.x + m.data[2][1] * v.y + m.data[2][2] * v.z,
    };
}

template <typename Scalar>
[[nodiscard]] constexpr Vector4T<Scalar> operator*(
    const MatrixT<Scalar, 4>& m, Vector4T<Scalar> v) noexcept {
    return Vector4T<Scalar>{
        m.data[0][0] * v.x + m.data[0][1] * v.y + m.data[0][2] * v.z + m.data[0][3] * v.w,
        m.data[1][0] * v.x + m.data[1][1] * v.y + m.data[1][2] * v.z + m.data[1][3] * v.w,
        m.data[2][0] * v.x + m.data[2][1] * v.y + m.data[2][2] * v.z + m.data[2][3] * v.w,
        m.data[3][0] * v.x + m.data[3][1] * v.y + m.data[3][2] * v.z + m.data[3][3] * v.w,
    };
}

} // namespace GeoCore::linear
```

修改 `include/GeoCore/GeoCore.hpp`，在 `UnitVector3.hpp` 之后追加：

```cpp
#include <GeoCore/linear/Matrix.hpp>
```

- [ ] **Step 4: 运行测试并提交**

```bash
cmake --build build/windows-vs --config Debug
ctest --test-dir build/windows-vs -C Debug --output-on-failure
```

Expected: 全部通过。

```bash
git add include/GeoCore/linear/Matrix.hpp include/GeoCore/GeoCore.hpp tests/linear/matrix_test.cpp
git commit -m "feat(linear): add MatrixT with multiplication, transpose and matrix-vector product"
```

---

### Task 7: linear —— 行列式与求逆

`determinant()` 返回标量，`inverse()` 返回 `std::optional` —— 奇异矩阵无法求逆是数学事实，不是异常情况，因此用返回值表达而不是抛异常或产出含 `inf` 的矩阵。这正是 Review Focus 第 4 条。

**Files:**
- Modify: `include/GeoCore/linear/Matrix.hpp`（追加行列式与求逆）
- Create: `tests/linear/matrix_inverse_test.cpp`

**Interfaces:**
- Consumes: Task 6 的 `MatrixT`
- Produces:
  - `determinant(const MatrixT<S, N>&) -> S`，支持 `N` = 2、3、4
  - `inverse(const MatrixT<S, N>&, core::Tolerance = {}) -> std::optional<MatrixT<S, N>>`，支持 `N` = 2、3、4；行列式在容差下可视为零时返回 `std::nullopt`

- [ ] **Step 1: 写失败测试**

创建 `tests/linear/matrix_inverse_test.cpp`：

```cpp
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <limits>

#include <GeoCore/linear/Matrix.hpp>

using Catch::Approx;

using GeoCore::core::Tolerance;
using GeoCore::linear::determinant;
using GeoCore::linear::identity;
using GeoCore::linear::inverse;
using GeoCore::linear::Matrix2;
using GeoCore::linear::Matrix3;
using GeoCore::linear::Matrix4;

TEST_CASE("determinant of a 2x2 matrix", "[linear][matrix][determinant]") {
    CHECK(determinant(Matrix2{{1.0, 2.0}, {3.0, 4.0}}) == Approx(-2.0));
    CHECK(determinant(identity<double, 2>()) == Approx(1.0));
    CHECK(determinant(Matrix2{{1.0, 2.0}, {2.0, 4.0}}) == Approx(0.0));
}

TEST_CASE("determinant of a 3x3 matrix", "[linear][matrix][determinant]") {
    const Matrix3 m{{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}, {7.0, 8.0, 10.0}};
    CHECK(determinant(m) == Approx(-3.0));

    CHECK(determinant(identity<double, 3>()) == Approx(1.0));
}

TEST_CASE("determinant of a singular 3x3 matrix is zero",
          "[linear][matrix][determinant][degenerate]") {
    // 第三行是前两行之和
    const Matrix3 m{{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}, {5.0, 7.0, 9.0}};
    CHECK(determinant(m) == Approx(0.0));
}

TEST_CASE("determinant of a 4x4 matrix", "[linear][matrix][determinant]") {
    CHECK(determinant(identity<double, 4>()) == Approx(1.0));

    // 下三角矩阵的行列式是对角线之积
    const Matrix4 lower{
        {2.0, 0.0, 0.0, 0.0},
        {3.0, 4.0, 0.0, 0.0},
        {5.0, 6.0, 7.0, 0.0},
        {8.0, 9.0, 10.0, 11.0},
    };
    CHECK(determinant(lower) == Approx(2.0 * 4.0 * 7.0 * 11.0));
}

TEST_CASE("inverse times original is the identity", "[linear][matrix][inverse]") {
    const Matrix2 m{{4.0, 7.0}, {2.0, 6.0}};
    const auto inv = inverse(m);

    REQUIRE(inv.has_value());
    const Matrix2 product = m * *inv;

    CHECK(product(0, 0) == Approx(1.0));
    CHECK(product(0, 1) == Approx(0.0));
    CHECK(product(1, 0) == Approx(0.0));
    CHECK(product(1, 1) == Approx(1.0));
}

TEST_CASE("3x3 inverse round-trips", "[linear][matrix][inverse]") {
    const Matrix3 m{{1.0, 2.0, 3.0}, {0.0, 1.0, 4.0}, {5.0, 6.0, 0.0}};
    const auto inv = inverse(m);

    REQUIRE(inv.has_value());
    const Matrix3 product = m * *inv;
    const Matrix3 unit = identity<double, 3>();

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            CHECK(product(i, j) == Approx(unit(i, j)).margin(1e-12));
        }
    }
}

TEST_CASE("inverse of a singular matrix is nullopt",
          "[linear][matrix][inverse][degenerate]") {
    // 第二行是第一行的两倍
    const Matrix2 singular{{1.0, 2.0}, {2.0, 4.0}};

    const auto inv = inverse(singular);

    // 必须是 nullopt，而不是含 inf / NaN 的矩阵
    CHECK_FALSE(inv.has_value());

    const Matrix3 singular3{
        {1.0, 2.0, 3.0},
        {2.0, 4.0, 6.0},
        {1.0, 1.0, 1.0},
    };
    CHECK_FALSE(inverse(singular3).has_value());
}

TEST_CASE("inverse of a matrix containing NaN does not crash",
          "[linear][matrix][inverse][degenerate]") {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const Matrix2 bad{{nan, 1.0}, {2.0, 3.0}};

    // 结果不作保证，但不得崩溃
    const auto inv = inverse(bad);
    (void)inv;
    SUCCEED("inverse of a NaN matrix returned without crashing");
}

TEST_CASE("an exact tolerance rejects only a genuinely singular matrix",
          "[linear][matrix][inverse][degenerate]") {
    const Tolerance exact{0.0, 0.0};

    CHECK_FALSE(inverse(Matrix2{{1.0, 2.0}, {2.0, 4.0}}, exact).has_value());
    CHECK(inverse(Matrix2{{1.0, 2.0}, {2.0, 4.0000000001}}, exact).has_value());
}
```

- [ ] **Step 2: 运行测试，确认失败**

```bash
cmake --build build/windows-vs --config Debug
```

Expected: 编译失败 —— `determinant` / `inverse` 未声明。

- [ ] **Step 3: 实现行列式与求逆**

在 `include/GeoCore/linear/Matrix.hpp` 中，`// ---- 矩阵 × 向量 ----` 注释块**之前**插入以下内容，并在文件顶部的 include 区追加 `#include <optional>` 与 `#include <GeoCore/core/Tolerance.hpp>`：

```cpp
// ---- 行列式 ----

/// 2×2 / 3×3 / 4×4 的行列式。
///
/// 用 if constexpr 写成单一实现，而不是为每个尺寸提供一个重载：单一
/// 实现能借 static_assert 给出「只支持 2/3/4」这样明确的编译期诊断，
/// 也免去维护三份几乎相同的签名。
///
/// 展开为闭式解（4×4 用第一行余子式展开）而非通用 LU 分解：固定尺寸的
/// 展开式没有循环开销，编译器也能完全内联。
template <typename Scalar, int N>
[[nodiscard]] constexpr Scalar determinant(const MatrixT<Scalar, N>& m) noexcept {
    static_assert(N == 2 || N == 3 || N == 4,
                  "determinant is implemented for 2x2, 3x3 and 4x4 matrices only");

    if constexpr (N == 2) {
        return m.data[0][0] * m.data[1][1] - m.data[0][1] * m.data[1][0];
    } else if constexpr (N == 3) {
        return m.data[0][0] * (m.data[1][1] * m.data[2][2] - m.data[1][2] * m.data[2][1])
             - m.data[0][1] * (m.data[1][0] * m.data[2][2] - m.data[1][2] * m.data[2][0])
             + m.data[0][2] * (m.data[1][0] * m.data[2][1] - m.data[1][1] * m.data[2][0]);
    } else {
        const Scalar sub_00 = m.data[1][1] * (m.data[2][2] * m.data[3][3] - m.data[2][3] * m.data[3][2])
                            - m.data[1][2] * (m.data[2][1] * m.data[3][3] - m.data[2][3] * m.data[3][1])
                            + m.data[1][3] * (m.data[2][1] * m.data[3][2] - m.data[2][2] * m.data[3][1]);
        const Scalar sub_01 = m.data[1][0] * (m.data[2][2] * m.data[3][3] - m.data[2][3] * m.data[3][2])
                            - m.data[1][2] * (m.data[2][0] * m.data[3][3] - m.data[2][3] * m.data[3][0])
                            + m.data[1][3] * (m.data[2][0] * m.data[3][2] - m.data[2][2] * m.data[3][0]);
        const Scalar sub_02 = m.data[1][0] * (m.data[2][1] * m.data[3][3] - m.data[2][3] * m.data[3][1])
                            - m.data[1][1] * (m.data[2][0] * m.data[3][3] - m.data[2][3] * m.data[3][0])
                            + m.data[1][3] * (m.data[2][0] * m.data[3][1] - m.data[2][1] * m.data[3][0]);
        const Scalar sub_03 = m.data[1][0] * (m.data[2][1] * m.data[3][2] - m.data[2][2] * m.data[3][1])
                            - m.data[1][1] * (m.data[2][0] * m.data[3][2] - m.data[2][2] * m.data[3][0])
                            + m.data[1][2] * (m.data[2][0] * m.data[3][1] - m.data[2][1] * m.data[3][0]);

        return m.data[0][0] * sub_00 - m.data[0][1] * sub_01
             + m.data[0][2] * sub_02 - m.data[0][3] * sub_03;
    }
}

// ---- 求逆 ----

/// 逆矩阵。矩阵在给定容差下行列式可视为零（即奇异）时返回 std::nullopt。
///
/// 用 optional 而非抛出异常：奇异矩阵是数学事实，不是程序错误，调用者
/// 有责任处理这个分支。返回 std::nullopt 也让调用者不可能拿到一个含
/// inf / NaN 的矩阵。
///
/// 与 determinant 一样用 if constexpr 写成单一实现。
template <typename Scalar, int N>
[[nodiscard]] constexpr std::optional<MatrixT<Scalar, N>> inverse(
    const MatrixT<Scalar, N>& m, core::Tolerance tolerance = {}) noexcept {
    static_assert(N == 2 || N == 3 || N == 4,
                  "inverse is implemented for 2x2, 3x3 and 4x4 matrices only");

    const Scalar det = determinant(m);
    if (tolerance.is_zero(static_cast<double>(det))) {
        return std::nullopt;
    }
    const Scalar inv_det = Scalar{1} / det;

    if constexpr (N == 2) {
        MatrixT<Scalar, 2> result{};
        result.data[0][0] =  m.data[1][1] * inv_det;
        result.data[0][1] = -m.data[0][1] * inv_det;
        result.data[1][0] = -m.data[1][0] * inv_det;
        result.data[1][1] =  m.data[0][0] * inv_det;
        return result;
    } else if constexpr (N == 3) {
        MatrixT<Scalar, 3> result{};
        result.data[0][0] = (m.data[1][1] * m.data[2][2] - m.data[1][2] * m.data[2][1]) * inv_det;
        result.data[0][1] = (m.data[0][2] * m.data[2][1] - m.data[0][1] * m.data[2][2]) * inv_det;
        result.data[0][2] = (m.data[0][1] * m.data[1][2] - m.data[0][2] * m.data[1][1]) * inv_det;
        result.data[1][0] = (m.data[1][2] * m.data[2][0] - m.data[1][0] * m.data[2][2]) * inv_det;
        result.data[1][1] = (m.data[0][0] * m.data[2][2] - m.data[0][2] * m.data[2][0]) * inv_det;
        result.data[1][2] = (m.data[0][2] * m.data[1][0] - m.data[0][0] * m.data[1][2]) * inv_det;
        result.data[2][0] = (m.data[1][0] * m.data[2][1] - m.data[1][1] * m.data[2][0]) * inv_det;
        result.data[2][1] = (m.data[0][1] * m.data[2][0] - m.data[0][0] * m.data[2][1]) * inv_det;
        result.data[2][2] = (m.data[0][0] * m.data[1][1] - m.data[0][1] * m.data[1][0]) * inv_det;
        return result;
    } else {
        // 4×4 按伴随矩阵求逆：result(i, j) = cofactor(j, i) / det。
        // 先用 2×2 子式算出每个 3×3 余子式，再按符号填入转置位置。
        const auto minor3 = [&m](int skip_row, int skip_column) noexcept -> Scalar {
            Scalar block[3][3];
            int r = 0;
            for (int i = 0; i < 4; ++i) {
                if (i == skip_row) { continue; }
                int c = 0;
                for (int j = 0; j < 4; ++j) {
                    if (j == skip_column) { continue; }
                    block[r][c] = m.data[i][j];
                    ++c;
                }
                ++r;
            }
            return block[0][0] * (block[1][1] * block[2][2] - block[1][2] * block[2][1])
                 - block[0][1] * (block[1][0] * block[2][2] - block[1][2] * block[2][0])
                 + block[0][2] * (block[1][0] * block[2][1] - block[1][1] * block[2][0]);
        };

        MatrixT<Scalar, 4> result{};
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                const Scalar cofactor = minor3(j, i);
                const Scalar sign = ((i + j) % 2 == 0) ? Scalar{1} : Scalar{-1};
                result.data[i][j] = sign * cofactor * inv_det;
            }
        }
        return result;
    }
}
```

- [ ] **Step 4: 运行测试并提交**

```bash
cmake --build build/windows-vs --config Debug
ctest --test-dir build/windows-vs -C Debug --output-on-failure
```

Expected: 全部通过。

```bash
git add include/GeoCore/linear/Matrix.hpp tests/linear/matrix_inverse_test.cpp
git commit -m "feat(linear): add determinant and optional-returning inverse"
```

---

### Task 8: linear —— Quaternion

四元数用 `w, x, y, z` 成员顺序（标量部分在前），默认构造为**单位四元数**而非零 —— 四元数在这里表示旋转，默认值应当表示恒等旋转，否则默认构造的对象会静默地成为一个非法的零旋转。

**Files:**
- Create: `include/GeoCore/linear/Quaternion.hpp`
- Create: `tests/linear/quaternion_test.cpp`
- Modify: `include/GeoCore/GeoCore.hpp`（追加 include）

**Interfaces:**
- Consumes: `Vector3T`（Task 4）、`MatrixT`（Task 6）、`GeoCore::core::pi`（Task 2）
- Produces:
  - `template <typename Scalar> struct QuaternionT { Scalar w{1}; Scalar x{}; Scalar y{}; Scalar z{}; }`
  - 别名 `Quaternion`、`Quaternionf`
  - 自由函数 `identity_quaternion<Scalar>()`、`conjugate(QuaternionT) -> QuaternionT`、`norm(QuaternionT) -> Scalar`、`normalize(QuaternionT, core::Tolerance) -> std::optional<QuaternionT>`、`dot(QuaternionT, QuaternionT) -> Scalar`
  - 构造：`from_axis_angle(UnitVector3T<S>, S) -> QuaternionT<S>`
  - 应用：`rotate(QuaternionT<S>, Vector3T<S>) -> Vector3T<S>`
  - 转换：`to_matrix(QuaternionT<S>) -> MatrixT<S, 3>`
  - 运算符：`*`（四元数乘四元数、四元数乘标量）、`==`

- [ ] **Step 1: 写失败测试**

创建 `tests/linear/quaternion_test.cpp`：

```cpp
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>

#include <GeoCore/core/Constants.hpp>
#include <GeoCore/linear/Quaternion.hpp>

using Catch::Approx;

using GeoCore::core::half_pi;
using GeoCore::linear::conjugate;
using GeoCore::linear::dot;
using GeoCore::linear::from_axis_angle;
using GeoCore::linear::normalize;
using GeoCore::linear::Quaternion;
using GeoCore::linear::rotate;
using GeoCore::linear::to_matrix;
using GeoCore::linear::UnitVector3;
using GeoCore::linear::Vector3;

namespace {
const UnitVector3 z_axis = UnitVector3::from_normalized_unchecked(Vector3{0.0, 0.0, 1.0});
}

TEST_CASE("default-constructed quaternion is the identity rotation",
          "[linear][quaternion]") {
    const Quaternion q{};

    CHECK(q.w == 1.0);
    CHECK(q.x == 0.0);
    CHECK(q.y == 0.0);
    CHECK(q.z == 0.0);

    // 单位旋转不改变任何向量
    CHECK(rotate(q, Vector3{1.0, 2.0, 3.0}) == Vector3{1.0, 2.0, 3.0});
}

TEST_CASE("axis-angle construction produces a unit quaternion",
          "[linear][quaternion]") {
    const Quaternion q = from_axis_angle(z_axis, half_pi);

    CHECK(q.w == Approx(std::cos(half_pi / 2.0)));
    CHECK(q.z == Approx(std::sin(half_pi / 2.0)));
    CHECK(norm(q) == Approx(1.0));
}

TEST_CASE("rotation about z by 90 degrees maps x to y",
          "[linear][quaternion]") {
    const Quaternion q = from_axis_angle(z_axis, half_pi);

    const Vector3 rotated = rotate(q, Vector3{1.0, 0.0, 0.0});
    CHECK(rotated.x == Approx(0.0).margin(1e-15));
    CHECK(rotated.y == Approx(1.0));
    CHECK(rotated.z == Approx(0.0).margin(1e-15));
}

TEST_CASE("rotation preserves length", "[linear][quaternion]") {
    const Quaternion q = from_axis_angle(z_axis, 0.7);
    const Vector3 v{3.0, 4.0, 0.0};

    CHECK(rotate(q, v).length() == Approx(v.length()));
}

TEST_CASE("conjugate undoes the rotation", "[linear][quaternion]") {
    const Quaternion q = from_axis_angle(z_axis, 0.7);
    const Vector3 v{1.0, 2.0, 3.0};

    const Vector3 there_and_back = rotate(conjugate(q), rotate(q, v));
    CHECK(there_and_back.x == Approx(v.x));
    CHECK(there_and_back.y == Approx(v.y));
    CHECK(there_and_back.z == Approx(v.z));
}

TEST_CASE("quaternion multiplication composes rotations",
          "[linear][quaternion]") {
    const Quaternion half = from_axis_angle(z_axis, half_pi / 2.0);
    const Quaternion full = half * half;

    const Vector3 rotated = rotate(full, Vector3{1.0, 0.0, 0.0});
    CHECK(rotated.x == Approx(0.0).margin(1e-15));
    CHECK(rotated.y == Approx(1.0));
}

TEST_CASE("dot of a unit quaternion with itself is 1", "[linear][quaternion]") {
    const Quaternion q = from_axis_angle(z_axis, 0.7);
    CHECK(dot(q, q) == Approx(1.0));
}

TEST_CASE("normalize rejects the zero quaternion",
          "[linear][quaternion][degenerate]") {
    const Quaternion zero{0.0, 0.0, 0.0, 0.0};

    CHECK_FALSE(normalize(zero).has_value());
    CHECK(normalize(Quaternion{}).has_value());
}

TEST_CASE("to_matrix agrees with rotate", "[linear][quaternion]") {
    const Quaternion q = from_axis_angle(z_axis, half_pi);
    const GeoCore::linear::Matrix3 m = to_matrix(q);

    const Vector3 v{1.0, 2.0, 3.0};
    const Vector3 by_matrix = m * v;
    const Vector3 by_quaternion = rotate(q, v);

    CHECK(by_matrix.x == Approx(by_quaternion.x));
    CHECK(by_matrix.y == Approx(by_quaternion.y));
    CHECK(by_matrix.z == Approx(by_quaternion.z));
}
```

- [ ] **Step 2: 运行测试，确认失败**

```bash
cmake --build build/windows-vs --config Debug
```

Expected: 编译失败 —— 找不到 `GeoCore/linear/Quaternion.hpp`。

- [ ] **Step 3: 实现 Quaternion**

创建 `include/GeoCore/linear/Quaternion.hpp`：

```cpp
#pragma once

#include <optional>

#include <GeoCore/core/Constants.hpp>
#include <GeoCore/core/Tolerance.hpp>
#include <GeoCore/linear/Matrix.hpp>
#include <GeoCore/linear/UnitVector3.hpp>
#include <GeoCore/linear/Vector3.hpp>

namespace GeoCore::linear {

/// 四元数，表示三维旋转。
///
/// 成员顺序为标量部分在前（w, x, y, z）。默认构造得到**单位四元数**
/// 而非全零 —— 本类型表示旋转，默认值必须表示恒等旋转，否则默认构造
/// 的对象会静默地成为一个不可用的零旋转。
template <typename Scalar>
struct QuaternionT {
    using scalar_type = Scalar;

    Scalar w{1};
    Scalar x{};
    Scalar y{};
    Scalar z{};

    [[nodiscard]] constexpr bool operator==(const QuaternionT&) const noexcept = default;
};

using Quaternion = QuaternionT<double>;
using Quaternionf = QuaternionT<float>;

/// 单位四元数，即恒等旋转。
template <typename Scalar>
[[nodiscard]] constexpr QuaternionT<Scalar> identity_quaternion() noexcept {
    return QuaternionT<Scalar>{};
}

/// 共轭。对单位四元数而言即为逆旋转。
template <typename Scalar>
[[nodiscard]] constexpr QuaternionT<Scalar> conjugate(QuaternionT<Scalar> q) noexcept {
    return QuaternionT<Scalar>{q.w, -q.x, -q.y, -q.z};
}

/// 四元数内积。
template <typename Scalar>
[[nodiscard]] constexpr Scalar dot(QuaternionT<Scalar> a, QuaternionT<Scalar> b) noexcept {
    return a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
}

/// 模长。
template <typename Scalar>
[[nodiscard]] Scalar norm(QuaternionT<Scalar> q) noexcept {
    // 先按最大分量缩放，避免中间量上溢或下溢。
    Scalar scale = core::absolute_value(q.w);
    if (core::absolute_value(q.x) > scale) { scale = core::absolute_value(q.x); }
    if (core::absolute_value(q.y) > scale) { scale = core::absolute_value(q.y); }
    if (core::absolute_value(q.z) > scale) { scale = core::absolute_value(q.z); }

    if (scale == Scalar{0}) {
        return Scalar{0};
    }
    const Scalar sw = q.w / scale;
    const Scalar sx = q.x / scale;
    const Scalar sy = q.y / scale;
    const Scalar sz = q.z / scale;
    return scale * std::sqrt(sw * sw + sx * sx + sy * sy + sz * sz);
}

/// 归一化。模长在给定容差下可视为零时返回 std::nullopt。
template <typename Scalar>
[[nodiscard]] std::optional<QuaternionT<Scalar>> normalize(
    QuaternionT<Scalar> q, core::Tolerance tolerance = {}) noexcept {
    const Scalar magnitude = norm(q);
    if (tolerance.is_zero(static_cast<double>(magnitude))) {
        return std::nullopt;
    }
    const Scalar inverse_magnitude = Scalar{1} / magnitude;
    return QuaternionT<Scalar>{
        q.w * inverse_magnitude,
        q.x * inverse_magnitude,
        q.y * inverse_magnitude,
        q.z * inverse_magnitude,
    };
}

/// 四元数乘法，对应旋转的复合。`a * b` 表示先施加 b 再施加 a。
template <typename Scalar>
[[nodiscard]] constexpr QuaternionT<Scalar> operator*(
    QuaternionT<Scalar> a, QuaternionT<Scalar> b) noexcept {
    return QuaternionT<Scalar>{
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
    };
}

template <typename Scalar, typename Factor>
    requires std::convertible_to<Factor, Scalar>
[[nodiscard]] constexpr QuaternionT<Scalar> operator*(
    QuaternionT<Scalar> q, Factor factor) noexcept {
    const auto scale = static_cast<Scalar>(factor);
    return QuaternionT<Scalar>{q.w * scale, q.x * scale, q.y * scale, q.z * scale};
}

/// 由单位轴与弧度角构造旋转。
///
/// 轴参数刻意接受 UnitVector3T 而非 Vector3T：非单位轴会让结果不再是
/// 单位四元数，这个前置条件由类型系统表达比写在文档里更可靠。
template <typename Scalar>
[[nodiscard]] QuaternionT<Scalar> from_axis_angle(
    UnitVector3T<Scalar> axis, Scalar angle_radians) noexcept {
    const Scalar half_angle = angle_radians / Scalar{2};
    const Scalar sine = std::sin(half_angle);
    return QuaternionT<Scalar>{
        std::cos(half_angle),
        axis.x() * sine,
        axis.y() * sine,
        axis.z() * sine,
    };
}

/// 用四元数旋转向量。要求旋转是单位四元数。
template <typename Scalar>
[[nodiscard]] constexpr Vector3T<Scalar> rotate(
    QuaternionT<Scalar> q, Vector3T<Scalar> v) noexcept {
    // v' = v + 2 * q_vec × (q_vec × v + w * v)
    // 比 q * (0,v) * conj(q) 少了两次四元数乘法，且不必构造纯四元数。
    const Vector3T<Scalar> q_vector{q.x, q.y, q.z};
    const Vector3T<Scalar> t = cross(q_vector, v) + v * q.w;
    return v + cross(q_vector, t) * Scalar{2};
}

/// 转换为等价的旋转矩阵。
template <typename Scalar>
[[nodiscard]] constexpr MatrixT<Scalar, 3> to_matrix(QuaternionT<Scalar> q) noexcept {
    const Scalar xx = q.x * q.x;
    const Scalar yy = q.y * q.y;
    const Scalar zz = q.z * q.z;
    const Scalar xy = q.x * q.y;
    const Scalar xz = q.x * q.z;
    const Scalar yz = q.y * q.z;
    const Scalar wx = q.w * q.x;
    const Scalar wy = q.w * q.y;
    const Scalar wz = q.w * q.z;

    MatrixT<Scalar, 3> result{};
    result.data[0][0] = Scalar{1} - Scalar{2} * (yy + zz);
    result.data[0][1] = Scalar{2} * (xy - wz);
    result.data[0][2] = Scalar{2} * (xz + wy);
    result.data[1][0] = Scalar{2} * (xy + wz);
    result.data[1][1] = Scalar{1} - Scalar{2} * (xx + zz);
    result.data[1][2] = Scalar{2} * (yz - wx);
    result.data[2][0] = Scalar{2} * (xz - wy);
    result.data[2][1] = Scalar{2} * (yz + wx);
    result.data[2][2] = Scalar{1} - Scalar{2} * (xx + yy);
    return result;
}

} // namespace GeoCore::linear
```

`Quaternion.hpp` 需要 `<cmath>` 提供 `std::sin` / `std::cos` / `std::sqrt` —— 在 include 区加上 `#include <cmath>`。

修改 `include/GeoCore/GeoCore.hpp`，在 `Matrix.hpp` 之后追加：

```cpp
#include <GeoCore/linear/Quaternion.hpp>
```

- [ ] **Step 4: 运行测试并提交**

```bash
cmake --build build/windows-vs --config Debug
ctest --test-dir build/windows-vs -C Debug --output-on-failure
```

Expected: 全部通过。若 `rotate` 的测试在末位出现约 1e-16 的偏差，属预期，测试已用 `margin(1e-15)` 容忍。

```bash
git add include/GeoCore/linear/Quaternion.hpp include/GeoCore/GeoCore.hpp tests/linear/quaternion_test.cpp
git commit -m "feat(linear): add QuaternionT with axis-angle construction and rotation"
```

---

### Task 9: linear —— Transform2 与 Transform3

仿射变换。这里有一个由分层约束逼出来的设计：`Point2` / `Point3` 属于 `prim` 层，而 `linear` **不允许**依赖 `prim` —— 否则依赖方向就反了。因此 `Transform3` 在本层只提供作用于 `Vector3T` 的两个操作：

- `operator*(Vector3T)` —— 只施加线性部分，用于变换**方向**（法线、速度等）
- `transform_position(Vector3T)` —— 施加完整仿射变换，参数被当作**位置**解读

作用于 `Point3` 的运算符由 `prim` 层在后续计划中提供。这个分工不是权宜之计，而是架构约束在具体 API 上的正确投影。

**Files:**
- Create: `include/GeoCore/linear/Transform2.hpp`
- Create: `include/GeoCore/linear/Transform3.hpp`
- Create: `tests/linear/transform2_test.cpp`
- Create: `tests/linear/transform3_test.cpp`
- Modify: `include/GeoCore/GeoCore.hpp`（追加 include）

**Interfaces:**
- Consumes: `MatrixT`（Task 6）、`QuaternionT`（Task 8）、`UnitVector3T`（Task 5）
- Produces:
  - `template <typename Scalar> struct Transform2T { MatrixT<Scalar, 3> matrix; }`，别名 `Transform2`、`Transform2f`
  - `template <typename Scalar> struct Transform3T { MatrixT<Scalar, 4> matrix; }`，别名 `Transform3`、`Transform3f`
  - `identity_transform<Scalar, N>()`，或按类型命名的 `Transform2T<S>::identity()`
  - 工厂：`translation_2d(Vector2T<S>)`、`scaling_2d(S)`、`rotation_2d(S)`；`translation_3d(Vector3T<S>)`、`scaling_3d(S)`、`scaling_3d(Vector3T<S>)`、`rotation_3d(UnitVector3T<S>, S)`
  - 应用：`apply(const TransformNT<S>&, VectorNT<S>) -> VectorNT<S>`（完整仿射）、`operator*(TransformNT<S>, VectorNT<S>) -> VectorNT<S>`（仅线性部分）
  - 复合：`operator*(TransformNT<S>, TransformNT<S>) -> TransformNT<S>`
  - 求逆：`inverse(const TransformNT<S>&, core::Tolerance = {}) -> std::optional<TransformNT<S>>`
  - `operator==`

- [ ] **Step 1: 写失败测试**

创建 `tests/linear/transform3_test.cpp`：

```cpp
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>

#include <GeoCore/core/Constants.hpp>
#include <GeoCore/linear/Transform3.hpp>

using Catch::Approx;

using GeoCore::core::half_pi;
using GeoCore::linear::apply;
using GeoCore::linear::inverse;
using GeoCore::linear::identity_transform;
using GeoCore::linear::rotation_3d;
using GeoCore::linear::scaling_3d;
using GeoCore::linear::Transform3;
using GeoCore::linear::translation_3d;
using GeoCore::linear::UnitVector3;
using GeoCore::linear::Vector3;

namespace {
const UnitVector3 z_axis = UnitVector3::from_normalized_unchecked(Vector3{0.0, 0.0, 1.0});
}

TEST_CASE("identity transform leaves a position unchanged", "[linear][transform3]") {
    const Transform3 t = identity_transform<double>();

    CHECK(apply(t, Vector3{1.0, 2.0, 3.0}) == Vector3{1.0, 2.0, 3.0});
}

TEST_CASE("translation moves a position but not a direction",
          "[linear][transform3]") {
    const Transform3 t = translation_3d(Vector3{10.0, 20.0, 30.0});

    // 位置被平移
    CHECK(apply(t, Vector3{1.0, 2.0, 3.0}) == Vector3{11.0, 22.0, 33.0});

    // 方向不受平移影响 —— 这正是 operator* 与 apply 的分工
    CHECK(t * Vector3{1.0, 2.0, 3.0} == Vector3{1.0, 2.0, 3.0});
}

TEST_CASE("uniform scaling scales both positions and directions",
          "[linear][transform3]") {
    const Transform3 t = scaling_3d(2.0);

    CHECK(apply(t, Vector3{1.0, 2.0, 3.0}) == Vector3{2.0, 4.0, 6.0});
    CHECK(t * Vector3{1.0, 2.0, 3.0} == Vector3{2.0, 4.0, 6.0});
}

TEST_CASE("non-uniform scaling scales each axis independently",
          "[linear][transform3]") {
    const Transform3 t = scaling_3d(Vector3{2.0, 3.0, 4.0});

    CHECK(apply(t, Vector3{1.0, 1.0, 1.0}) == Vector3{2.0, 3.0, 4.0});
}

TEST_CASE("rotation about z by 90 degrees", "[linear][transform3]") {
    const Transform3 t = rotation_3d(z_axis, half_pi);

    const Vector3 rotated = apply(t, Vector3{1.0, 0.0, 0.0});
    CHECK(rotated.x == Approx(0.0).margin(1e-15));
    CHECK(rotated.y == Approx(1.0));
    CHECK(rotated.z == Approx(0.0).margin(1e-15));
}

TEST_CASE("composition applies the right operand first", "[linear][transform3]") {
    const Transform3 move = translation_3d(Vector3{1.0, 0.0, 0.0});
    const Transform3 spin = rotation_3d(z_axis, half_pi);

    // 绕 z 的 90° 旋转在浮点下含约 1e-16 的误差，故用 margin 而非精确相等
    // 先平移再旋转： (1,0,0) -> (2,0,0) -> (0,2,0)
    const Vector3 rotate_after = apply(spin * move, Vector3{1.0, 0.0, 0.0});
    CHECK(rotate_after.x == Approx(0.0).margin(1e-15));
    CHECK(rotate_after.y == Approx(2.0));
    CHECK(rotate_after.z == Approx(0.0).margin(1e-15));

    // 先旋转再平移： (1,0,0) -> (0,1,0) -> (1,1,0)
    const Vector3 translate_after = apply(move * spin, Vector3{1.0, 0.0, 0.0});
    CHECK(translate_after.x == Approx(1.0));
    CHECK(translate_after.y == Approx(1.0));
    CHECK(translate_after.z == Approx(0.0).margin(1e-15));
}

TEST_CASE("inverse undoes the transform", "[linear][transform3]") {
    const Transform3 t = translation_3d(Vector3{5.0, -3.0, 2.0})
                       * rotation_3d(z_axis, 0.7)
                       * scaling_3d(2.0);

    const auto inv = inverse(t);

    REQUIRE(inv.has_value());
    const Vector3 original{1.0, 2.0, 3.0};
    const Vector3 round_trip = apply(*inv, apply(t, original));

    CHECK(round_trip.x == Approx(original.x).margin(1e-12));
    CHECK(round_trip.y == Approx(original.y).margin(1e-12));
    CHECK(round_trip.z == Approx(original.z).margin(1e-12));
}

TEST_CASE("inverse of a degenerate transform is nullopt",
          "[linear][transform3][degenerate]") {
    const Transform3 flatten = scaling_3d(Vector3{1.0, 0.0, 1.0});

    CHECK_FALSE(inverse(flatten).has_value());
}
```

创建 `tests/linear/transform2_test.cpp`：

```cpp
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <GeoCore/core/Constants.hpp>
#include <GeoCore/linear/Transform2.hpp>

using Catch::Approx;

using GeoCore::core::half_pi;
using GeoCore::linear::apply;
using GeoCore::linear::rotation_2d;
using GeoCore::linear::Transform2;
using GeoCore::linear::translation_2d;
using GeoCore::linear::Vector2;

TEST_CASE("2D translation moves positions only", "[linear][transform2]") {
    const Transform2 t = translation_2d(Vector2{5.0, 7.0});

    CHECK(apply(t, Vector2{1.0, 2.0}) == Vector2{6.0, 9.0});
    CHECK(t * Vector2{1.0, 2.0} == Vector2{1.0, 2.0});
}

TEST_CASE("2D rotation by 90 degrees maps x to y", "[linear][transform2]") {
    const Transform2 t = rotation_2d(half_pi);

    const Vector2 rotated = apply(t, Vector2{1.0, 0.0});
    CHECK(rotated.x == Approx(0.0).margin(1e-15));
    CHECK(rotated.y == Approx(1.0));
}
```

- [ ] **Step 2: 运行测试，确认失败**

```bash
cmake --build build/windows-vs --config Debug
```

Expected: 编译失败 —— 找不到 `GeoCore/linear/Transform3.hpp`。

- [ ] **Step 3: 实现 Transform3**

创建 `include/GeoCore/linear/Transform3.hpp`：

```cpp
#pragma once

#include <concepts>
#include <optional>

#include <GeoCore/core/Tolerance.hpp>
#include <GeoCore/linear/Matrix.hpp>
#include <GeoCore/linear/Quaternion.hpp>
#include <GeoCore/linear/UnitVector3.hpp>
#include <GeoCore/linear/Vector3.hpp>

namespace GeoCore::linear {

/// 三维仿射变换，内部为 4×4 齐次矩阵（行主序，列向量约定 `M * v`）。
/// 平移量位于第 4 列。
///
/// 注意分层约束：Point3 属于 prim 层，linear 不得依赖它。因此本类型
/// 只提供作用于 Vector3T 的运算 ——
///   operator*(v)          只施加线性部分，用于变换方向
///   apply(t, v)           施加完整仿射变换，v 被解读为位置
/// 作用于 Point3 的运算符由 prim 层提供。
template <typename Scalar>
struct Transform3T {
    using scalar_type = Scalar;

    MatrixT<Scalar, 4> matrix = identity<Scalar, 4>();

    [[nodiscard]] constexpr bool operator==(const Transform3T&) const noexcept = default;
};

using Transform3 = Transform3T<double>;
using Transform3f = Transform3T<float>;

// ---- 工厂 ----

template <typename Scalar>
[[nodiscard]] constexpr Transform3T<Scalar> identity_transform() noexcept {
    return Transform3T<Scalar>{};
}

template <typename Scalar>
[[nodiscard]] constexpr Transform3T<Scalar> translation_3d(Vector3T<Scalar> offset) noexcept {
    Transform3T<Scalar> result{};
    result.matrix.data[0][3] = offset.x;
    result.matrix.data[1][3] = offset.y;
    result.matrix.data[2][3] = offset.z;
    return result;
}

/// 各轴独立的缩放。
template <typename Scalar>
[[nodiscard]] constexpr Transform3T<Scalar> scaling_3d(Vector3T<Scalar> factors) noexcept {
    Transform3T<Scalar> result{};
    result.matrix.data[0][0] = factors.x;
    result.matrix.data[1][1] = factors.y;
    result.matrix.data[2][2] = factors.z;
    return result;
}

/// 各轴相同的缩放。
///
/// 参数类型即 Scalar，因此调用处写 `scaling_3d(2.0)` 即可。传整数字面量
/// 会因不满足 floating_point 约束而编译失败 —— 这是刻意的：缩放因子作用
/// 在浮点变换上，应当是浮点数。若确实要写 `scaling_3d(2)`，请改为 `2.0`。
template <std::floating_point Scalar>
[[nodiscard]] constexpr Transform3T<Scalar> scaling_3d(Scalar factor) noexcept {
    return scaling_3d(Vector3T<Scalar>{factor, factor, factor});
}

/// 绕给定单位轴旋转 angle_radians 弧度。
template <typename Scalar>
[[nodiscard]] Transform3T<Scalar> rotation_3d(
    UnitVector3T<Scalar> axis, Scalar angle_radians) noexcept {
    const QuaternionT<Scalar> q = from_axis_angle(axis, angle_radians);
    const MatrixT<Scalar, 3> rotation = to_matrix(q);

    Transform3T<Scalar> result{};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            result.matrix.data[i][j] = rotation.data[i][j];
        }
    }
    return result;
}

// ---- 应用 ----

/// 施加完整仿射变换，输入按**位置**解读（平移生效）。
template <typename Scalar>
[[nodiscard]] constexpr Vector3T<Scalar> apply(
    const Transform3T<Scalar>& t, Vector3T<Scalar> position) noexcept {
    const MatrixT<Scalar, 4>& m = t.matrix;
    return Vector3T<Scalar>{
        m.data[0][0] * position.x + m.data[0][1] * position.y + m.data[0][2] * position.z + m.data[0][3],
        m.data[1][0] * position.x + m.data[1][1] * position.y + m.data[1][2] * position.z + m.data[1][3],
        m.data[2][0] * position.x + m.data[2][1] * position.y + m.data[2][2] * position.z + m.data[2][3],
    };
}

/// 只施加线性部分，输入按**方向**解读（平移不生效）。
template <typename Scalar>
[[nodiscard]] constexpr Vector3T<Scalar> operator*(
    const Transform3T<Scalar>& t, Vector3T<Scalar> direction) noexcept {
    const MatrixT<Scalar, 4>& m = t.matrix;
    return Vector3T<Scalar>{
        m.data[0][0] * direction.x + m.data[0][1] * direction.y + m.data[0][2] * direction.z,
        m.data[1][0] * direction.x + m.data[1][1] * direction.y + m.data[1][2] * direction.z,
        m.data[2][0] * direction.x + m.data[2][1] * direction.y + m.data[2][2] * direction.z,
    };
}

/// 复合。`a * b` 表示先施加 b 再施加 a。
template <typename Scalar>
[[nodiscard]] constexpr Transform3T<Scalar> operator*(
    const Transform3T<Scalar>& a, const Transform3T<Scalar>& b) noexcept {
    return Transform3T<Scalar>{a.matrix * b.matrix};
}

/// 逆变换。线性部分奇异时返回 std::nullopt。
template <typename Scalar>
[[nodiscard]] std::optional<Transform3T<Scalar>> inverse(
    const Transform3T<Scalar>& t, core::Tolerance tolerance = {}) noexcept {
    const auto inverse_matrix = inverse(t.matrix, tolerance);
    if (!inverse_matrix.has_value()) {
        return std::nullopt;
    }
    return Transform3T<Scalar>{*inverse_matrix};
}

} // namespace GeoCore::linear
```

创建 `include/GeoCore/linear/Transform2.hpp`：

```cpp
#pragma once

#include <concepts>
#include <optional>

#include <GeoCore/core/Tolerance.hpp>
#include <GeoCore/linear/Matrix.hpp>
#include <GeoCore/linear/Vector2.hpp>

namespace GeoCore::linear {

/// 二维仿射变换，内部为 3×3 齐次矩阵（行主序）。
/// 分层约束与 Transform3T 相同，详见该类型的文档。
template <typename Scalar>
struct Transform2T {
    using scalar_type = Scalar;

    MatrixT<Scalar, 3> matrix = identity<Scalar, 3>();

    [[nodiscard]] constexpr bool operator==(const Transform2T&) const noexcept = default;
};

using Transform2 = Transform2T<double>;
using Transform2f = Transform2T<float>;

template <typename Scalar>
[[nodiscard]] constexpr Transform2T<Scalar> translation_2d(Vector2T<Scalar> offset) noexcept {
    Transform2T<Scalar> result{};
    result.matrix.data[0][2] = offset.x;
    result.matrix.data[1][2] = offset.y;
    return result;
}

template <typename Scalar>
[[nodiscard]] constexpr Transform2T<Scalar> scaling_2d(Vector2T<Scalar> factors) noexcept {
    Transform2T<Scalar> result{};
    result.matrix.data[0][0] = factors.x;
    result.matrix.data[1][1] = factors.y;
    return result;
}

template <std::floating_point Scalar>
[[nodiscard]] constexpr Transform2T<Scalar> scaling_2d(Scalar factor) noexcept {
    return scaling_2d(Vector2T<Scalar>{factor, factor});
}

/// 逆时针旋转 angle_radians 弧度。
template <typename Scalar>
[[nodiscard]] Transform2T<Scalar> rotation_2d(Scalar angle_radians) noexcept {
    const Scalar cosine = std::cos(angle_radians);
    const Scalar sine = std::sin(angle_radians);

    Transform2T<Scalar> result{};
    result.matrix.data[0][0] = cosine;
    result.matrix.data[0][1] = -sine;
    result.matrix.data[1][0] = sine;
    result.matrix.data[1][1] = cosine;
    return result;
}

/// 施加完整仿射变换，输入按**位置**解读。
template <typename Scalar>
[[nodiscard]] constexpr Vector2T<Scalar> apply(
    const Transform2T<Scalar>& t, Vector2T<Scalar> position) noexcept {
    const MatrixT<Scalar, 3>& m = t.matrix;
    return Vector2T<Scalar>{
        m.data[0][0] * position.x + m.data[0][1] * position.y + m.data[0][2],
        m.data[1][0] * position.x + m.data[1][1] * position.y + m.data[1][2],
    };
}

/// 只施加线性部分，输入按**方向**解读。
template <typename Scalar>
[[nodiscard]] constexpr Vector2T<Scalar> operator*(
    const Transform2T<Scalar>& t, Vector2T<Scalar> direction) noexcept {
    const MatrixT<Scalar, 3>& m = t.matrix;
    return Vector2T<Scalar>{
        m.data[0][0] * direction.x + m.data[0][1] * direction.y,
        m.data[1][0] * direction.x + m.data[1][1] * direction.y,
    };
}

/// 复合。`a * b` 表示先施加 b 再施加 a。
template <typename Scalar>
[[nodiscard]] constexpr Transform2T<Scalar> operator*(
    const Transform2T<Scalar>& a, const Transform2T<Scalar>& b) noexcept {
    return Transform2T<Scalar>{a.matrix * b.matrix};
}

template <typename Scalar>
[[nodiscard]] std::optional<Transform2T<Scalar>> inverse(
    const Transform2T<Scalar>& t, core::Tolerance tolerance = {}) noexcept {
    const auto inverse_matrix = inverse(t.matrix, tolerance);
    if (!inverse_matrix.has_value()) {
        return std::nullopt;
    }
    return Transform2T<Scalar>{*inverse_matrix};
}

} // namespace GeoCore::linear
```

`Transform2.hpp` 需要 `<cmath>` 提供 `std::sin` / `std::cos` —— 在 include 区加上 `#include <cmath>`。

修改 `include/GeoCore/GeoCore.hpp`，在 `Quaternion.hpp` 之后追加：

```cpp
#include <GeoCore/linear/Transform2.hpp>
#include <GeoCore/linear/Transform3.hpp>
```

- [ ] **Step 4: 运行测试并提交**

```bash
cmake --build build/windows-vs --config Debug
ctest --test-dir build/windows-vs -C Debug --output-on-failure
```

Expected: 全部通过。

```bash
git add include/GeoCore/linear/Transform2.hpp include/GeoCore/linear/Transform3.hpp include/GeoCore/GeoCore.hpp tests/linear/transform2_test.cpp tests/linear/transform3_test.cpp
git commit -m "feat(linear): add Transform2T and Transform3T"
```

---

### Task 10: 示例与文档收尾

示例是对外发布的库最被实际阅读的部分 —— 使用者复制粘贴的就是它。本任务把 `examples/` 从占位变成真正演示 `core` 与 `linear` 的可用代码，并让 README 反映阶段 1 的实际能力。

**Files:**
- Create: `examples/vector_basics.cpp`
- Create: `examples/transform_pipeline.cpp`
- Create: `docs/Doxyfile`
- Modify: `examples/CMakeLists.txt`
- Modify: `README.md`

**Interfaces:**
- Consumes: 全部 `GeoCore` / `GeoCore::linear` 公开接口
- Produces: 无（终端任务）

- [ ] **Step 1: 写向量基础示例**

创建 `examples/vector_basics.cpp`：

```cpp
// GeoCore 基础示例：向量与单位向量。
//
// 构建后可直接运行：
//   cmake --build build/windows-vs --config Debug --target vector_basics
//   ./build/windows-vs/examples/Debug/vector_basics.exe

#include <cassert>
#include <cmath>
#include <iostream>

#include <GeoCore/GeoCore.hpp>

using GeoCore::linear::cross;
using GeoCore::linear::dot;
using GeoCore::linear::normalize;
using GeoCore::linear::Vector3;

int main() {
    const Vector3 a{1.0, 2.0, 3.0};
    const Vector3 b{4.0, 5.0, 6.0};

    std::cout << "a          = (" << a.x << ", " << a.y << ", " << a.z << ")\n";
    std::cout << "b          = (" << b.x << ", " << b.y << ", " << b.z << ")\n";
    std::cout << "a + b      = (" << (a + b).x << ", " << (a + b).y << ", " << (a + b).z << ")\n";
    std::cout << "a . b      = " << dot(a, b) << '\n';

    const Vector3 perpendicular = cross(a, b);
    std::cout << "a x b      = (" << perpendicular.x << ", "
              << perpendicular.y << ", " << perpendicular.z << ")\n";

    // 叉积的结果垂直于两个输入 —— 点积应当为零
    std::cout << "a . (a x b) = " << dot(a, perpendicular) << '\n';

    // 归一化返回 optional：零向量无法归一化，这是编译期就不会被忽略的分支
    if (const auto unit = normalize(a)) {
        std::cout << "a / |a|    = (" << unit->x() << ", "
                  << unit->y() << ", " << unit->z() << ")\n";
        std::cout << "|a / |a||  = " << unit->as_vector().length() << '\n';
    }

    if (!normalize(Vector3{0.0, 0.0, 0.0})) {
        std::cout << "normalize(zero) correctly returned nullopt\n";
    }

    return 0;
}
```

- [ ] **Step 2: 写变换管线的示例**

创建 `examples/transform_pipeline.cpp`：

```cpp
// GeoCore 示例：仿射变换的复合与求逆。
//
// 演示三件事：
//   1. 变换可以像数值一样相乘来复合
//   2. apply() 变换位置，operator* 变换方向（平移不作用于方向）
//   3. inverse() 用 optional 表达「不可逆」这一数学事实

#include <iostream>

#include <GeoCore/GeoCore.hpp>

using GeoCore::core::half_pi;
using GeoCore::linear::apply;
using GeoCore::linear::inverse;
using GeoCore::linear::rotation_3d;
using GeoCore::linear::scaling_3d;
using GeoCore::linear::Transform3;
using GeoCore::linear::translation_3d;
using GeoCore::linear::UnitVector3;
using GeoCore::linear::Vector3;

namespace {

void print(const char* label, const Vector3& v) {
    std::cout << label << " = (" << v.x << ", " << v.y << ", " << v.z << ")\n";
}

} // namespace

int main() {
    // 绕 z 轴的单位向量。unsafe 构造在此是安全的：常量显然是单位长度。
    const UnitVector3 z_axis = UnitVector3::from_normalized_unchecked(Vector3{0.0, 0.0, 1.0});

    // 先缩放 2 倍，再绕 z 轴转 90°，最后平移。
    const Transform3 model = translation_3d(Vector3{10.0, 0.0, 0.0})
                           * rotation_3d(z_axis, half_pi)
                           * scaling_3d(2.0);

    const Vector3 position{1.0, 0.0, 0.0};
    print("position          ", position);
    print("transformed       ", apply(model, position));

    // 方向不受平移影响。
    const Vector3 direction{1.0, 0.0, 0.0};
    print("direction         ", direction);
    print("transformed dir   ", model * direction);

    // 求逆并回代。
    if (const auto undo = inverse(model)) {
        const Vector3 round_trip = apply(*undo, apply(model, position));
        print("round trip        ", round_trip);
    }

    // 压扁平面的变换不可逆，inverse 返回 nullopt 而不是产出 inf。
    const Transform3 flatten = scaling_3d(Vector3{1.0, 0.0, 1.0});
    std::cout << "inverse(flatten) is "
              << (inverse(flatten) ? "available" : "nullopt (as expected)")
              << '\n';

    return 0;
}
```

- [ ] **Step 3: 注册示例并运行**

把 `examples/CMakeLists.txt` 改为：

```cmake
add_executable(hello_geocore hello_geocore.cpp)
target_link_libraries(hello_geocore PRIVATE GeoCore::GeoCore)

add_executable(vector_basics vector_basics.cpp)
target_link_libraries(vector_basics PRIVATE GeoCore::GeoCore)

add_executable(transform_pipeline transform_pipeline.cpp)
target_link_libraries(transform_pipeline PRIVATE GeoCore::GeoCore)
```

构建并运行：

```bash
cmake --preset windows-vs
cmake --build --preset windows-vs-debug
./build/windows-vs/examples/Debug/vector_basics.exe
```

Expected:
```
a          = (1, 2, 3)
b          = (4, 5, 6)
a + b      = (5, 7, 9)
a . b      = 32
a x b      = (-3, 6, -3)
a . (a x b) = 0
a / |a|    = (0.267261, 0.534522, 0.801784)
|a / |a||  = 1
normalize(zero) correctly returned nullopt
```

再运行第二个：

```bash
./build/windows-vs/examples/Debug/transform_pipeline.exe
```

Expected: 输出的 `transformed` 为 `(10, 2, 0)`，`transformed dir` 为 `(0, 2, 0)`，`round trip` 回到 `(1, 0, 0)`，末行为 `inverse(flatten) is nullopt (as expected)`。

- [ ] **Step 4: 完善 README 与 Doxygen 配置**

把 `README.md` 的 `## Design principles` 一节之后追加：

```markdown
## Quick start

```cpp
#include <GeoCore/GeoCore.hpp>
#include <iostream>

int main() {
    using namespace GeoCore::linear;

    const Vector3 a{1.0, 2.0, 3.0};
    const Vector3 b{4.0, 5.0, 6.0};

    std::cout << dot(a, b) << '\n';   // 32

    // 归一化返回 optional —— 零向量无解这一事实由类型表达，
    // 调用者无法忽略这个分支。
    if (const auto unit = normalize(a)) {
        std::cout << unit->as_vector().length() << '\n';   // 1
    }
}
```

更多示例见 `examples/`。

## What works today

| Area | Status |
|---|---|
| `core` — 常量、数值工具、容差模型 | 完成 |
| `linear` — `Vector2/3/4`、`UnitVector2/3`、`Matrix2/3/4`、`Quaternion`、`Transform2/3` | 完成 |
| `predicates` — 精确方向 / 内切圆判定 | 计划 2 |
| `prim` / `query` — 几何原语与求交查询 | 计划 3 |
| `polygon` / `mesh` / `solid` 接口骨架 | 计划 4 |

尚未实现的能力以接口形式存在时会抛出 `std::logic_error`，绝不静默返回错误结果。
```

创建 `docs/Doxyfile`：

```
PROJECT_NAME           = GeoCore
PROJECT_BRIEF          = "A robust, efficient, and easy-to-use C++ geometry library"
OUTPUT_DIRECTORY       = api
INPUT                  = ../include ../README.md
RECURSIVE              = YES
FILE_PATTERNS          = *.hpp *.md
USE_MDFILE_AS_MAINPAGE = ../README.md
EXTRACT_ALL            = YES
JAVADOC_AUTOBRIEF      = YES
GENERATE_HTML          = YES
GENERATE_LATEX         = NO
QUIET                  = YES
WARN_IF_UNDOCUMENTED   = NO
```

用 `doxygen docs/Doxyfile` 生成到 `docs/api/`（该目录已被 `.gitignore` 中的规则覆盖需自行确认；若未覆盖，请把 `docs/api/` 追加进 `.gitignore`）。

- [ ] **Step 5: 全量验证并提交**

```bash
cmake --preset windows-vs
cmake --build --preset windows-vs-debug
ctest --preset windows-vs-debug
```

Expected: 全部测试通过，无失败。

```bash
git add examples/ README.md docs/Doxyfile
git commit -m "docs: add runnable examples and document stage 1 capabilities"
```

---

## 完成标准

本计划完成后应满足：

1. `cmake --preset windows-vs && cmake --build --preset windows-vs-debug && ctest --preset windows-vs-debug` 全绿。
2. `examples/` 下三个示例都能构建并以预期输出运行。
3. 五个 Review Focus 场景全部有通过的测试固定：零向量归一化、极大分量上溢、极小分量下溢、奇异矩阵求逆、NaN 输入不崩溃。
4. 无第三方运行时依赖；`GeoCore::GeoCore` 是唯一的公开 target。
5. 库可在 `/W4` 或 `-Wall -Wextra -Wpedantic` 下无警告构建。

阶段 2（`predicates`）的计划在本计划完成、且 `linear` 的接口经受住实际使用之后另行动笔。
