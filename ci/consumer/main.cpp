// DragonGeo 安装包的消费方冒烟测试。
//
// 本工程刻意不设置任何编码选项，也不依赖仓库内的构建：一旦编译失败，诊断
// 只会指向 DragonGeo 随包安装的头文件，或本文件同样按 UTF-8 写的中文注释 ——
// 两者都要求 /utf-8 随 DragonGeo::DragonGeo 一起到达消费方。可执行代码本身
// 是纯 ASCII，因此行号上的错误不可能来自这里的代码。

#include <DragonGeo/DragonGeo.hpp>

#include <iostream>

int main() {
    using namespace DragonGeo::Linear;

    const Vector3 a{1.0, 2.0, 3.0};
    const Vector3 b{4.0, 5.0, 6.0};

    std::cout << "dot = " << a.Dot(b) << '\n';

    const auto unit = Vector3{3.0, 4.0, 0.0}.Normalized();
    if (!unit.has_value()) {
        std::cout << "normalized returned nullopt\n";
        return 1;
    }
    std::cout << "unit = (" << unit->X() << ", " << unit->Y() << ", " << unit->Z() << ")\n";

    const Transform3 pipeline = Transform3::Translation(Vector3{1.0, 2.0, 3.0}) * Transform3::Scaling(2.0);
    const Vector3 moved = pipeline.Apply(Vector3{1.0, 1.0, 1.0});
    std::cout << "moved = (" << moved.X << ", " << moved.Y << ", " << moved.Z << ")\n";

    return moved == Vector3{3.0, 4.0, 5.0} ? 0 : 2;
}
