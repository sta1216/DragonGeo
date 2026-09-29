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
