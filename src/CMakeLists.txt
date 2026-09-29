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
