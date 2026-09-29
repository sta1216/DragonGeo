// GeoCore 安装包的消费方冒烟测试。
//
// 本文件刻意只用 ASCII：一旦编译失败，错误必然指向 GeoCore 的头文件，
// 而不是消费方自己的源码编码。

#include <GeoCore/GeoCore.hpp>

#include <iostream>

int main() {
    using namespace GeoCore::linear;

    const Vector3 a{1.0, 2.0, 3.0};
    const Vector3 b{4.0, 5.0, 6.0};

    std::cout << "dot = " << dot(a, b) << '\n';

    const auto unit = normalize(Vector3{3.0, 4.0, 0.0});
    if (!unit.has_value()) {
        std::cout << "normalize returned nullopt\n";
        return 1;
    }
    std::cout << "unit = (" << unit->x() << ", " << unit->y() << ", " << unit->z() << ")\n";

    const Transform3 pipeline = translation_3d(Vector3{1.0, 2.0, 3.0}) * scaling_3d(2.0);
    const Vector3 moved = apply(pipeline, Vector3{1.0, 1.0, 1.0});
    std::cout << "moved = (" << moved.x << ", " << moved.y << ", " << moved.z << ")\n";

    return moved == Vector3{3.0, 4.0, 5.0} ? 0 : 2;
}
