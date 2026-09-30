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

add_executable(DragonGeoBenchmarks LinearBenchmarks.cpp)
target_link_libraries(DragonGeoBenchmarks PRIVATE DragonGeo::DragonGeo benchmark::benchmark)
