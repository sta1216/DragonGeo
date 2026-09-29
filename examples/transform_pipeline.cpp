// GeoCore 示例：仿射变换的复合与求逆。
//
// 演示三件事：
//   1. 变换可以像数值一样相乘来复合
//   2. apply() 变换位置，operator* 变换方向（平移不作用于方向）
//   3. inverse() 用 optional 表达「不可逆」这一数学事实

#include <iostream>

#include <GeoCore/GeoCore.hpp>

using GeoCore::core::half_pi;
using GeoCore::linear::Transform3;
using GeoCore::linear::UnitVector3;
using GeoCore::linear::Vector3;

namespace {

void print(const char* label, const Vector3& v) {
    std::cout << label << " = (" << v.x << ", " << v.y << ", " << v.z << ")\n";
}

} // namespace

int main() {
    // 绕 z 轴的单位向量。unsafe 构造在此是安全的：常量显然是单位长度。
    const UnitVector3 z_axis = UnitVector3::from_normalized_unchecked(Vector3{0.0, 0.0, 1.0});

    // 先缩放 2 倍，再绕 z 轴转 90°，最后平移。
    const Transform3 model = Transform3::translation(Vector3{10.0, 0.0, 0.0})
                           * Transform3::rotation(z_axis, half_pi)
                           * Transform3::scaling(2.0);

    const Vector3 position{1.0, 0.0, 0.0};
    print("position          ", position);
    print("transformed       ", model.apply(position));

    // 方向不受平移影响。
    //
    // 注意随后两行输出里会出现 -4.44e-16、-8.88e-16 这样的极小分量：绕 z 轴
    // 旋转 90° 在浮点下并不精确，cos(π/2) 不是精确的 0。这是 IEEE-754 的固有
    // 行为，不是缺陷，示例照实打印而不做美化 —— 使用者本就应该预期到它。
    const Vector3 direction{1.0, 0.0, 0.0};
    print("direction         ", direction);
    print("transformed dir   ", model * direction);

    // 求逆并回代。
    if (const auto undo = model.inverse()) {
        const Vector3 round_trip = undo->apply(model.apply(position));
        print("round trip        ", round_trip);
    }

    // 压扁平面的变换不可逆，inverse 返回 nullopt 而不是产出 inf。
    const Transform3 flatten = Transform3::scaling(Vector3{1.0, 0.0, 1.0});
    std::cout << "inverse(flatten) is "
              << (flatten.inverse() ? "available" : "nullopt (as expected)")
              << '\n';

    return 0;
}
