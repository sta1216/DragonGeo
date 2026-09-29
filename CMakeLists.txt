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
