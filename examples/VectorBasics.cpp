// DragonGeo 基础示例：向量与单位向量。
//
// 构建后可直接运行：
//   cmake --build build/windows-vs --config Debug --target VectorBasics
//   ./build/windows-vs/examples/Debug/VectorBasics.exe

#include <cassert>
#include <cmath>
#include <iostream>

#include <DragonGeo/DragonGeo.hpp>

using DragonGeo::Linear::Vector3;

int main() {
    const Vector3 a{1.0, 2.0, 3.0};
    const Vector3 b{4.0, 5.0, 6.0};

    std::cout << "a          = (" << a.X << ", " << a.Y << ", " << a.Z << ")\n";
    std::cout << "b          = (" << b.X << ", " << b.Y << ", " << b.Z << ")\n";
    std::cout << "a + b      = (" << (a + b).X << ", " << (a + b).Y << ", " << (a + b).Z << ")\n";
    std::cout << "a . b      = " << a.Dot(b) << '\n';

    const Vector3 perpendicular = a.Cross(b);
    std::cout << "a x b      = (" << perpendicular.X << ", "
              << perpendicular.Y << ", " << perpendicular.Z << ")\n";

    // 叉积的结果垂直于两个输入 —— 点积应当为零
    std::cout << "a . (a x b) = " << a.Dot(perpendicular) << '\n';

    // 归一化返回 optional：零向量无法归一化，这是编译期就不会被忽略的分支
    if (const auto unit = a.Normalized()) {
        std::cout << "a / |a|    = (" << unit->X() << ", "
                  << unit->Y() << ", " << unit->Z() << ")\n";
        std::cout << "|a / |a||  = " << unit->AsVector().Length() << '\n';
    }

    if (!Vector3{0.0, 0.0, 0.0}.Normalized()) {
        std::cout << "Normalized(zero) correctly returned nullopt\n";
    }

    return 0;
}
