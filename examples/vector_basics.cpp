// GeoCore 基础示例：向量与单位向量。
//
// 构建后可直接运行：
//   cmake --build build/windows-vs --config Debug --target vector_basics
//   ./build/windows-vs/examples/Debug/vector_basics.exe

#include <cassert>
#include <cmath>
#include <iostream>

#include <GeoCore/GeoCore.hpp>

using GeoCore::linear::Vector3;

int main() {
    const Vector3 a{1.0, 2.0, 3.0};
    const Vector3 b{4.0, 5.0, 6.0};

    std::cout << "a          = (" << a.x << ", " << a.y << ", " << a.z << ")\n";
    std::cout << "b          = (" << b.x << ", " << b.y << ", " << b.z << ")\n";
    std::cout << "a + b      = (" << (a + b).x << ", " << (a + b).y << ", " << (a + b).z << ")\n";
    std::cout << "a . b      = " << a.dot(b) << '\n';

    const Vector3 perpendicular = a.cross(b);
    std::cout << "a x b      = (" << perpendicular.x << ", "
              << perpendicular.y << ", " << perpendicular.z << ")\n";

    // 叉积的结果垂直于两个输入 —— 点积应当为零
    std::cout << "a . (a x b) = " << a.dot(perpendicular) << '\n';

    // 归一化返回 optional：零向量无法归一化，这是编译期就不会被忽略的分支
    if (const auto unit = a.normalized()) {
        std::cout << "a / |a|    = (" << unit->x() << ", "
                  << unit->y() << ", " << unit->z() << ")\n";
        std::cout << "|a / |a||  = " << unit->as_vector().length() << '\n';
    }

    if (!Vector3{0.0, 0.0, 0.0}.normalized()) {
        std::cout << "normalized(zero) correctly returned nullopt\n";
    }

    return 0;
}
