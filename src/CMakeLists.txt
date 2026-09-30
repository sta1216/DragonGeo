# 本阶段全部为核心类型，header-only。
# 后续计划引入 predicates / polygon / mesh / solid 的编译单元时，
# 把 add_library(DragonGeo INTERFACE) 改为 STATIC 并在此追加源文件。
add_library(DragonGeo INTERFACE)
add_library(DragonGeo::DragonGeo ALIAS DragonGeo)

target_include_directories(DragonGeo INTERFACE
    $<BUILD_INTERFACE:${DRAGONGEO_SOURCE_INCLUDE_DIR}>
    $<BUILD_INTERFACE:${DRAGONGEO_GENERATED_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

target_compile_features(DragonGeo INTERFACE cxx_std_20)

# /utf-8 必须随目标导出。顶层 add_compile_options 只作用于本目录树下编译的
# 目标（tests/ 与 examples/ 因此够用），它不会进入 DragonGeoTargets.cmake ——
# 而公共头是 UTF-8 无 BOM，消费方在代码页非 UTF-8 的 MSVC 主机上会按本地
# 代码页解码头文件里的中文注释，直接编译失败（cp936 上表现为 C2737/C2146，
# 指向头文件的注释行）。编码是接口的一部分，只能由 INTERFACE 选项携带。
target_compile_options(DragonGeo INTERFACE
    $<$<CXX_COMPILER_ID:MSVC>:/utf-8>)
