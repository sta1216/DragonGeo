// DragonGeo 示例：仿射变换的复合与求逆。
//
// 演示三件事： 1. 变换可以像数值一样相乘来复合 2. operator* 对 Point 施加完整仿射变换（输入按位置解读），对 Vector 只施加线性部分（输入按方向解读）—— 平移只作用于前者 3. Inverse() 用 optional 表达「不可逆」这一数学事实

#include <iostream>

#include <DragonGeo/DragonGeo.hpp>

using DragonGeo::Core::HALF_PI;
using DragonGeo::Linear::Point3;
using DragonGeo::Linear::Transform3;
using DragonGeo::Linear::UnitVector3;
using DragonGeo::Linear::Vector3;

namespace {

void print(const char* label, const Vector3& v) {
    std::cout << label << " = (" << v.X << ", " << v.Y << ", " << v.Z << ")\n";
}

void print(const char* label, const Point3& p) {
    std::cout << label << " = (" << p.X << ", " << p.Y << ", " << p.Z << ")\n";
}

} // namespace

int main() {
    // 绕 z 轴的单位向量。unsafe 构造在此是安全的：常量显然是单位长度。
    const UnitVector3 zAxis = UnitVector3::FromNormalizedUnchecked(Vector3{0.0, 0.0, 1.0});

    // 先缩放 2 倍，再绕 z 轴转 90°，最后平移。
    const Transform3 model = Transform3::Translation(Vector3{10.0, 0.0, 0.0}) * Transform3::Rotation(zAxis, HALF_PI) * Transform3::Scaling(2.0);

    const Point3 position{1.0, 0.0, 0.0};
    print("position          ", position);
    print("transformed       ", model * position);

    // 方向不受平移影响。
    //
    // 注意随后两行输出里会出现 -4.44e-16、-8.88e-16 这样的极小分量：绕 z 轴旋转 90° 在浮点下并不精确，cos(π/2) 不是精确的 0。这是 IEEE-754 的固有行为，不是缺陷，示例照实打印而不做美化 —— 使用者本就应该预期到它。
    const Vector3 direction{1.0, 0.0, 0.0};
    print("direction         ", direction);
    print("transformed dir   ", model * direction);

    // 求逆并回代。
    if (const auto undo = model.Inverse()) {
        const Point3 roundTrip = undo->TransformPoint(model * position);
        print("round trip        ", roundTrip);
    }

    // 压扁平面的变换不可逆，inverse 返回 nullopt 而不是产出 inf。
    const Transform3 flatten = Transform3::Scaling(Vector3{1.0, 0.0, 1.0});
    std::cout << "Inverse(flatten) is " << (flatten.Inverse() ? "available" : "nullopt (as expected)") << '\n';

    return 0;
}
