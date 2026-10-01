#include <benchmark/benchmark.h>

#include <DragonGeo/DragonGeo.hpp>

using namespace DragonGeo::Linear;

namespace {

// 空循环基准 —— 它是这一节的「零点」，用来发现空转：若某个基准的耗时与它相差无几，那个基准就是被折叠掉了，不是「快」。
void BenchEmpty(benchmark::State& state) {
    double sink = 0.0;
    for (auto _ : state) {
        benchmark::DoNotOptimize(sink);
    }
}
BENCHMARK(BenchEmpty);

void BenchDot(benchmark::State& state) {
    Vector3 a{1.0, 2.0, 3.0};
    Vector3 b{4.0, 5.0, 6.0};
    for (auto _ : state) {
        benchmark::DoNotOptimize(a);
        benchmark::DoNotOptimize(b);
        benchmark::DoNotOptimize(a.Dot(b));
    }
}
BENCHMARK(BenchDot);

void BenchCross(benchmark::State& state) {
    Vector3 a{1.0, 2.0, 3.0};
    Vector3 b{4.0, 5.0, 6.0};
    for (auto _ : state) {
        benchmark::DoNotOptimize(a);
        benchmark::DoNotOptimize(b);
        benchmark::DoNotOptimize(a.Cross(b));
    }
}
BENCHMARK(BenchCross);

void BenchLength(benchmark::State& state) {
    Vector3 v{3.0, 4.0, 12.0};
    for (auto _ : state) {
        benchmark::DoNotOptimize(v);
        benchmark::DoNotOptimize(v.Length());
    }
}
BENCHMARK(BenchLength);

void BenchNormalized(benchmark::State& state) {
    Vector3 v{3.0, 4.0, 12.0};
    for (auto _ : state) {
        benchmark::DoNotOptimize(v);
        benchmark::DoNotOptimize(v.Normalized());
    }
}
BENCHMARK(BenchNormalized);

void BenchSubscript(benchmark::State& state) {
    Vector3 v{3.0, 4.0, 12.0};
    for (auto _ : state) {
        benchmark::DoNotOptimize(v);
        benchmark::DoNotOptimize(v[0] + v[1] + v[2]);
    }
}
BENCHMARK(BenchSubscript);

void BenchMatrixVector(benchmark::State& state) {
    Matrix4 m{{{2.0, 0.0, 0.0, 1.0}, {0.0, 3.0, 0.0, 2.0}, {0.0, 0.0, 4.0, 3.0}, {0.0, 0.0, 0.0, 1.0}}};
    Vector4 v{1.0, 2.0, 3.0, 4.0};
    for (auto _ : state) {
        benchmark::DoNotOptimize(m);
        benchmark::DoNotOptimize(v);
        benchmark::DoNotOptimize(m * v);
    }
}
BENCHMARK(BenchMatrixVector);

void BenchMatrixInverse(benchmark::State& state) {
    Matrix3 m{{{1.0, 2.0, 3.0}, {0.0, 1.0, 4.0}, {5.0, 6.0, 0.0}}};
    for (auto _ : state) {
        benchmark::DoNotOptimize(m);
        benchmark::DoNotOptimize(m.Inverse());
    }
}
BENCHMARK(BenchMatrixInverse);

void BenchQuaternionRotate(benchmark::State& state) {
    const auto axis = UnitVector3::FromNormalizedUnchecked(Vector3{0.0, 0.0, 1.0});
    Quaternion q = Quaternion::FromAxisAngle(axis, 0.7);
    Vector3 v{1.0, 2.0, 3.0};
    for (auto _ : state) {
        benchmark::DoNotOptimize(q);
        benchmark::DoNotOptimize(v);
        benchmark::DoNotOptimize(q.Rotate(v));
    }
}
BENCHMARK(BenchQuaternionRotate);

// 本阶段新增的类型也要覆盖 —— 上层会在紧循环里反复调用它们 （包围盒剔除、坐标系往返变换），它们才是接下来最可能成为热点的地方。
void BenchBoxContains(benchmark::State& state) {
    Box3 box{Point3{0.0, 0.0, 0.0}, Point3{1.0, 2.0, 3.0}};
    Point3 p{0.5, 1.0, 1.5};
    for (auto _ : state) {
        benchmark::DoNotOptimize(box);
        benchmark::DoNotOptimize(p);
        benchmark::DoNotOptimize(box.Contains(p));
    }
}
BENCHMARK(BenchBoxContains);

void BenchCoordinateToParent(benchmark::State& state) {
    const auto frame = Coordinate3::FromZAxis(Point3{1.0, 0.0, 0.0}, UnitVector3::FromNormalizedUnchecked(Vector3{0.0, 0.0, 1.0}));
    if (!frame.has_value()) {
        state.SkipWithError("frame construction failed");
        return;
    }
    Coordinate3 c = *frame;
    Point3 p{1.0, 2.0, 3.0};
    for (auto _ : state) {
        benchmark::DoNotOptimize(c);
        benchmark::DoNotOptimize(p);
        benchmark::DoNotOptimize(c.ToParent(p));
    }
}
BENCHMARK(BenchCoordinateToParent);

} // namespace

BENCHMARK_MAIN();
