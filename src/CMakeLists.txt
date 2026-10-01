# 阶段 3 的谓词编译进这个静态库。Core 与 Linear 仍是头文件。
# Polygon、Mesh、bvh 继续往这个目标追加源文件。
add_library(DragonGeo STATIC
    Predicates/Predicates.cpp
    Prim/Line2.cpp
    Prim/Line3.cpp
    Prim/Plane.cpp
    Prim/Polyline3.cpp
    Prim/Ray2.cpp
    Prim/Ray3.cpp
    Prim/Segment2.cpp
    Prim/Segment3.cpp
    Prim/Triangle2.cpp
    Prim/Triangle3.cpp
    Query/Intersection.cpp)
add_library(DragonGeo::DragonGeo ALIAS DragonGeo)

target_include_directories(DragonGeo PUBLIC
    $<BUILD_INTERFACE:${DRAGONGEO_SOURCE_INCLUDE_DIR}>
    $<BUILD_INTERFACE:${DRAGONGEO_GENERATED_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

target_compile_features(DragonGeo PUBLIC cxx_std_20)

# /utf-8 必须随目标导出。顶层 add_compile_options 只作用于本目录树下编译的
# 目标（本库的编译单元、tests/ 与 examples/ 因此够用），它不会进入
# DragonGeoTargets.cmake —— 而公共头是 UTF-8 无 BOM，消费方在代码页非 UTF-8
# 的 MSVC 主机上会按本地代码页解码头文件里的中文注释，直接编译失败（cp936
# 上表现为 C2737/C2146，指向头文件的注释行）。编码是接口的一部分，只能由
# 导出选项携带。PUBLIC 同时覆盖本库自己的编译单元。
target_compile_options(DragonGeo PUBLIC
    $<$<CXX_COMPILER_ID:MSVC>:/utf-8>)
