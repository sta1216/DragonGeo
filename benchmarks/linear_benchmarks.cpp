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

BENCHMARK_MAIN();
