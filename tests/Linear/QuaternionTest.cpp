#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <limits>
#include <type_traits>

#include <DragonGeo/Core/Constants.hpp>
#include <DragonGeo/Linear/Quaternion.hpp>

using Catch::Approx;

using DragonGeo::Core::HALF_PI;
using DragonGeo::Core::Tolerance;
using DragonGeo::Linear::Matrix3;
using DragonGeo::Linear::Quaternion;
using DragonGeo::Linear::QuaternionT;
using DragonGeo::Linear::Quaternionf;
using DragonGeo::Linear::UnitVector3;
using DragonGeo::Linear::Vector3;

namespace {
const UnitVector3 zAxis = UnitVector3::FromNormalizedUnchecked(Vector3{0.0, 0.0, 1.0});
}

TEST_CASE("the float alias really is the float instantiation", "[linear][quaternion]") {
    // 同 vector4Test.cpp：Quaternionf 此前零命中，误绑定不会被断言发现。
    STATIC_REQUIRE(std::is_same_v<Quaternionf, QuaternionT<float>>);
}

TEST_CASE("default-constructed quaternion is the identity rotation", "[linear][quaternion]") {
    const Quaternion q{};

    CHECK(q.W == 1.0);
    CHECK(q.X == 0.0);
    CHECK(q.Y == 0.0);
    CHECK(q.Z == 0.0);

    // 单位旋转不改变任何向量
    CHECK(q.Rotate(Vector3{1.0, 2.0, 3.0}) == Vector3{1.0, 2.0, 3.0});
}

TEST_CASE("axis-angle construction produces a unit quaternion", "[linear][quaternion]") {
    const Quaternion q = Quaternion::FromAxisAngle(zAxis, HALF_PI);

    CHECK(q.W == Approx(std::cos(HALF_PI / 2.0)));
    CHECK(q.Z == Approx(std::sin(HALF_PI / 2.0)));
    CHECK(q.Norm() == Approx(1.0));
}

TEST_CASE("rotation about z by 90 degrees maps x to y", "[linear][quaternion]") {
    const Quaternion q = Quaternion::FromAxisAngle(zAxis, HALF_PI);

    const Vector3 rotated = q.Rotate(Vector3{1.0, 0.0, 0.0});
    CHECK(rotated.X == Approx(0.0).margin(1e-15));
    CHECK(rotated.Y == Approx(1.0));
    CHECK(rotated.Z == Approx(0.0).margin(1e-15));
}

TEST_CASE("rotation preserves length", "[linear][quaternion]") {
    const Quaternion q = Quaternion::FromAxisAngle(zAxis, 0.7);
    const Vector3 v{3.0, 4.0, 0.0};

    CHECK(q.Rotate(v).Length() == Approx(v.Length()));
}

TEST_CASE("conjugate undoes the rotation", "[linear][quaternion]") {
    const Quaternion q = Quaternion::FromAxisAngle(zAxis, 0.7);
    const Vector3 v{1.0, 2.0, 3.0};

    const Vector3 thereAndBack = q.Conjugate().Rotate(q.Rotate(v));
    CHECK(thereAndBack.X == Approx(v.X));
    CHECK(thereAndBack.Y == Approx(v.Y));
    CHECK(thereAndBack.Z == Approx(v.Z));
}

TEST_CASE("quaternion multiplication composes rotations", "[linear][quaternion]") {
    const Quaternion half = Quaternion::FromAxisAngle(zAxis, HALF_PI / 2.0);
    const Quaternion full = half * half;

    const Vector3 rotated = full.Rotate(Vector3{1.0, 0.0, 0.0});
    CHECK(rotated.X == Approx(0.0).margin(1e-15));
    CHECK(rotated.Y == Approx(1.0));
}

TEST_CASE("dot of a unit quaternion with itself is 1", "[linear][quaternion]") {
    const Quaternion q = Quaternion::FromAxisAngle(zAxis, 0.7);
    CHECK(q.Dot(q) == Approx(1.0));
}

TEST_CASE("normalize rejects the zero quaternion", "[linear][quaternion][degenerate]") {
    const Quaternion zero{0.0, 0.0, 0.0, 0.0};

    CHECK_FALSE(zero.Normalized().has_value());
    CHECK(Quaternion{}.Normalized().has_value());
}

TEST_CASE("ToMatrix agrees with rotate", "[linear][quaternion]") {
    const Quaternion q = Quaternion::FromAxisAngle(zAxis, HALF_PI);
    const DragonGeo::Linear::Matrix3 m = q.ToMatrix();

    const Vector3 v{1.0, 2.0, 3.0};
    const Vector3 byMatrix = m * v;
    const Vector3 byQuaternion = q.Rotate(v);

    CHECK(byMatrix.X == Approx(byQuaternion.X));
    CHECK(byMatrix.Y == Approx(byQuaternion.Y));
    CHECK(byMatrix.Z == Approx(byQuaternion.Z));
}

TEST_CASE("norm and normalize handle non-finite input consistently", "[linear][quaternion][degenerate]") {
    const double infinity = std::numeric_limits<double>::infinity();
    const double notANumber = std::numeric_limits<double>::quiet_NaN();

    // Norm 与 Vector3T::Length 语义一致：无穷输入的模长就是无穷，不是 NaN
    CHECK(Quaternion{infinity, 0.0, 0.0, 0.0}.Norm() == infinity);
    CHECK(Quaternion{0.0, 0.0, infinity, 0.0}.Norm() == infinity);

    // normalize 与 UnitVector3T::normalized / MatrixT::inverse 同一条原则：绝不交出一个 has_value() 为真、内容却是 NaN 的结果。
    CHECK_FALSE(Quaternion{infinity, 0.0, 0.0, 0.0}.Normalized().has_value());
    CHECK_FALSE(Quaternion{0.0, infinity, 0.0, 0.0}.Normalized().has_value());
    CHECK_FALSE(Quaternion{notANumber, 0.0, 0.0, 0.0}.Normalized().has_value());
    CHECK_FALSE(Quaternion{notANumber, notANumber, notANumber, notANumber}.Normalized().has_value());
}

TEST_CASE("norm gives a slot-independent answer on non-finite input", "[linear][quaternion][degenerate]") {
    const double infinity = std::numeric_limits<double>::infinity();
    const double nan = std::numeric_limits<double>::quiet_NaN();

    // Norm 与 VectorNT::Length 共用 Core::MaxAbsOf，规则相同：任一无穷分量 ⇒ +inf；否则含 NaN ⇒ NaN。槽位不影响答案。
    CHECK(std::isnan(Quaternion{5.0, nan, 0.0, 0.0}.Norm()));
    CHECK(std::isnan(Quaternion{nan, 5.0, 0.0, 0.0}.Norm()));
    CHECK(std::isnan(Quaternion{0.0, 0.0, nan, 5.0}.Norm()));
    CHECK(std::isnan(Quaternion{5.0, 0.0, 0.0, nan}.Norm()));

    CHECK(Quaternion{infinity, nan, 0.0, 0.0}.Norm() == infinity);
    CHECK(Quaternion{nan, infinity, 0.0, 0.0}.Norm() == infinity);
    CHECK(Quaternion{1.0, 1.0, nan, infinity}.Norm() == infinity);

    // 无 NaN 的输入不受影响
    CHECK(Quaternion{5.0, 0.0, 0.0, 0.0}.Norm() == 5.0);
}

TEST_CASE("normalize rejects input whose reciprocal magnitude overflows", "[linear][quaternion][degenerate]") {
    const Tolerance exact{0.0, 0.0};
    const double smallest = std::numeric_limits<double>::denorm_min();   // 5e-324

    // 模长有限并不保证倒数有限：最小的正次正规数作模长时 1/|q| 直接溢出成 inf，分量乘上去即得 inf。用精确容差排除「模长视为零」这条分支，单独考察溢出这一条 —— 曾经这里会返回 has_value() 为真、w == inf 的四元数。
    const auto result = Quaternion{smallest, 0.0, 0.0, 0.0}.Normalized(exact);

    CHECK_FALSE(result.has_value());
}

TEST_CASE("rotation about x by 90 degrees maps y to z", "[linear][quaternion]") {
    // 上面所有用例都绕 z 轴，因此四元数的 x、y 分量恒为零 —— 那些分量上的符号或系数错误会在 0 = 0 里消失。换一个轴把它们逼出来。
    const UnitVector3 xAxis = UnitVector3::FromNormalizedUnchecked(Vector3{1.0, 0.0, 0.0});
    const Quaternion q = Quaternion::FromAxisAngle(xAxis, HALF_PI);

    const Vector3 rotated = q.Rotate(Vector3{0.0, 1.0, 0.0});
    CHECK(rotated.X == Approx(0.0).margin(1e-15));
    CHECK(rotated.Y == Approx(0.0).margin(1e-15));
    CHECK(rotated.Z == Approx(1.0));
}

TEST_CASE("ToMatrix and rotate agree on a general axis", "[linear][quaternion]") {
    // 绕 z 轴时 xz = wy = yz = wx = 0，于是 m02/m20/m12/m21 恒为零，任何局限于这两块的符号错误都不可见。用一个一般轴让它们全部非零。
    const UnitVector3 axis = UnitVector3::FromNormalizedUnchecked(Vector3{1.0, 2.0, 3.0} / std::sqrt(14.0));
    const Quaternion q = Quaternion::FromAxisAngle(axis, 0.7);
    const DragonGeo::Linear::Matrix3 m = q.ToMatrix();

    // 先证明这次确实覆盖了那四个条目
    CHECK(m(0, 2) != 0.0);
    CHECK(m(2, 0) != 0.0);
    CHECK(m(1, 2) != 0.0);
    CHECK(m(2, 1) != 0.0);

    // 两条独立推导路径必须给出同一答案
    const Vector3 v{0.3, -0.7, 1.1};
    const Vector3 byMatrix = m * v;
    const Vector3 byQuaternion = q.Rotate(v);

    CHECK(byMatrix.X == Approx(byQuaternion.X));
    CHECK(byMatrix.Y == Approx(byQuaternion.Y));
    CHECK(byMatrix.Z == Approx(byQuaternion.Z));
}

TEST_CASE("scalar multiplication and negation are available", "[linear][quaternion]") {
    const Quaternion q{1.0, 2.0, 3.0, 4.0};

    // 与 Vector / Matrix 的接口保持一致：两种写法都要成立
    CHECK(q * 2.0 == Quaternion{2.0, 4.0, 6.0, 8.0});
    CHECK(2.0 * q == Quaternion{2.0, 4.0, 6.0, 8.0});

    CHECK(-q == Quaternion{-1.0, -2.0, -3.0, -4.0});

    // q 与 -q 是同一个旋转：对同一个向量作用的结果必须一致
    const Quaternion unit = Quaternion::FromAxisAngle(zAxis, HALF_PI);
    const Vector3 v{1.0, 2.0, 3.0};
    const Vector3 byQ = unit.Rotate(v);
    const Vector3 byNegated = (-unit).Rotate(v);

    CHECK(byNegated.X == Approx(byQ.X));
    CHECK(byNegated.Y == Approx(byQ.Y));
    CHECK(byNegated.Z == Approx(byQ.Z));
}

TEST_CASE("normalize scales a non-unit quaternion", "[linear][quaternion]") {
    // 原用例只用 has_value() 检查 normalize，因此一个「对非零模长原样返回」的实现也能全部通过。这里数值验证缩放路径本身。
    const auto axisAligned = Quaternion{2.0, 0.0, 0.0, 0.0}.Normalized();
    REQUIRE(axisAligned.has_value());
    CHECK(axisAligned->W == Approx(1.0));

    const auto general = Quaternion{1.0, 2.0, 3.0, 4.0}.Normalized();
    REQUIRE(general.has_value());
    CHECK(general->Norm() == Approx(1.0));
    CHECK(general->W == Approx(1.0 / std::sqrt(30.0)));
    CHECK(general->X == Approx(2.0 / std::sqrt(30.0)));
    CHECK(general->Y == Approx(3.0 / std::sqrt(30.0)));
    CHECK(general->Z == Approx(4.0 / std::sqrt(30.0)));
}

TEST_CASE("quaternion product composes rotations in the documented order", "[linear][quaternion]") {
    // 全库只有一处调用 operator*（half * half），而那是自乘积、操作数又是 z 轴四元数（x = y = 0），因此组合顺序从未被区分过 —— 自乘积对顺序不敏感。这里乘两个不同的半转，乘积不可交换，且下面同时验证它与逐次旋转一致。
    //
    // 注意：这两个操作数是轴对齐的，各有零分量，因而 16 个乘积项里只有 4 项是活的。钉住全部项的是紧随其后的那个一般轴用例。
    const UnitVector3 xAxis = UnitVector3::FromNormalizedUnchecked(Vector3{1.0, 0.0, 0.0});
    const UnitVector3 yAxis = UnitVector3::FromNormalizedUnchecked(Vector3{0.0, 1.0, 0.0});

    const Quaternion a = Quaternion::FromAxisAngle(xAxis, HALF_PI);   // 绕 x 轴 90°
    const Quaternion b = Quaternion::FromAxisAngle(yAxis, HALF_PI);   // 绕 y 轴 90°

    // 手算：a = (√2/2, √2/2, 0, 0)，b = (√2/2, 0, √2/2, 0)
    const Quaternion ab = a * b;
    CHECK(ab.W == Approx(0.5));
    CHECK(ab.X == Approx(0.5));
    CHECK(ab.Y == Approx(0.5));
    CHECK(ab.Z == Approx(0.5));

    // 交换相乘后 z 分量变号 —— 顺序确实有区别，且这个区别被钉住
    const Quaternion ba = b * a;
    CHECK(ba.W == Approx(0.5));
    CHECK(ba.X == Approx(0.5));
    CHECK(ba.Y == Approx(0.5));
    CHECK(ba.Z == Approx(-0.5));

    // 且必须与逐次旋转一致：a * b 表示先施加 b 再施加 a
    const Vector3 v{1.0, 2.0, 3.0};
    const Vector3 composed = ab.Rotate(v);
    const Vector3 sequential = a.Rotate(b.Rotate(v));
    CHECK(composed.X == Approx(sequential.X));
    CHECK(composed.Y == Approx(sequential.Y));
    CHECK(composed.Z == Approx(sequential.Z));
}

TEST_CASE("quaternion product with general operands pins all sixteen terms", "[linear][quaternion]") {
    // 上一个用例的两个操作数轴对齐、各有零分量，16 个乘积项里只有 4 项是活的： a.Z*b.Y、a.Z*b.X、a.X*b.X 都退化成 0·□，那些项上的符号错误依然看不见。这里两个操作数都取一般轴，四个分量全部非零，十六项全部参与运算。
    const UnitVector3 axisA = UnitVector3::FromNormalizedUnchecked(Vector3{1.0, 2.0, 3.0} / std::sqrt(14.0));
    const UnitVector3 axisB = UnitVector3::FromNormalizedUnchecked(Vector3{-2.0, 1.0, 0.5} / std::sqrt(4.0 + 1.0 + 0.25));

    const Quaternion a = Quaternion::FromAxisAngle(axisA, 0.7);
    const Quaternion b = Quaternion::FromAxisAngle(axisB, 1.1);

    // 先证明这次确实没有零分量可供退化
    CHECK(a.X != 0.0);
    CHECK(a.Y != 0.0);
    CHECK(a.Z != 0.0);
    CHECK(b.X != 0.0);
    CHECK(b.Y != 0.0);
    CHECK(b.Z != 0.0);

    // 期望值由参考实现（逐项按 Hamilton 规则）独立算出
    const Quaternion ab = a * b;
    CHECK(ab.W == Approx(0.7694798520425258));
    CHECK(ab.X == Approx(-0.39226136832113895));
    CHECK(ab.Y == Approx(0.23465896735876354));
    CHECK(ab.Z == Approx(0.446057109865496));

    // 不可交换：w 相同，其余三个分量各不相同
    const Quaternion ba = b * a;
    CHECK(ba.W == Approx(0.7694798520425258));
    CHECK(ba.X == Approx(-0.30863891228533297));
    CHECK(ba.Y == Approx(0.5064319494751331));
    CHECK(ba.Z == Approx(0.23700096977598095));
}

TEST_CASE("Quaternion members: norm, conjugate, rotate, ToMatrix, factories", "[linear][quaternion]") {
    const Quaternion q = Quaternion::FromAxisAngle(zAxis, HALF_PI);

    CHECK(q.Norm() == Approx(1.0));

    // 共轭把转过的角度原路转回。q 是绕 z 轴 +90°，所以 (0,1,0) 应落到 (1,0,0)。注意不要照抄文件末尾 x 轴用例里的 `.X == 0` —— 那条是绕 x 轴转，x 才恰好为 0；绕 z 轴转的 x 分量是 ±1。
    const Vector3 undone = q.Conjugate().Rotate(Vector3{0.0, 1.0, 0.0});
    CHECK(undone.X == Approx(1.0));
    CHECK(undone.Y == Approx(0.0).margin(1e-15));

    CHECK(Quaternion::Identity().Rotate(Vector3{1.0, 2.0, 3.0}) == Vector3{1.0, 2.0, 3.0});

    // 四个分量都非零且互不相同 —— 只用 w 非零的输入，一个「只缩放 w」的实现也能通过。模长 sqrt(1+4+9+16) = sqrt(30)。
    const auto unit = Quaternion{1.0, 2.0, 3.0, 4.0}.Normalized();
    REQUIRE(unit.has_value());
    CHECK(unit->W == Approx(1.0 / std::sqrt(30.0)));
    CHECK(unit->X == Approx(2.0 / std::sqrt(30.0)));
    CHECK(unit->Y == Approx(3.0 / std::sqrt(30.0)));
    CHECK(unit->Z == Approx(4.0 / std::sqrt(30.0)));

    // 手算值：绕 z 轴转 90° 把 (1,2,3) 送到 (-2,1,3)。这与「和 rotate 比」是两回事 —— 后者只能证明两者自洽，共同的符号约定错误照样通过。
    const Matrix3 m = q.ToMatrix();
    const Vector3 v{1.0, 2.0, 3.0};
    const Vector3 byMatrix = m * v;
    CHECK(byMatrix.X == Approx(-2.0));
    CHECK(byMatrix.Y == Approx(1.0));
    CHECK(byMatrix.Z == Approx(3.0));

    const Vector3 byQuaternion = q.Rotate(v);
    CHECK(byMatrix.X == Approx(byQuaternion.X));
    CHECK(byMatrix.Y == Approx(byQuaternion.Y));
    CHECK(byMatrix.Z == Approx(byQuaternion.Z));
}

TEST_CASE("Quaternion FromRotationMatrix round-trips axis-angle rotations", "[linear][quaternion]") {
    const Quaternion original = Quaternion::FromAxisAngle(zAxis, HALF_PI);
    const Matrix3 rotation = original.ToMatrix();
    const auto recovered = Quaternion::FromRotationMatrix(rotation);
    REQUIRE(recovered.has_value());

    const Vector3 probe{1.0, 2.0, 3.0};
    const Vector3 byOriginal = original.Rotate(probe);
    const Vector3 byRecovered = recovered->Rotate(probe);
    CHECK(byRecovered.X == Approx(byOriginal.X));
    CHECK(byRecovered.Y == Approx(byOriginal.Y));
    CHECK(byRecovered.Z == Approx(byOriginal.Z));

    Matrix3 reflection = Matrix3::Identity();
    reflection.Data[0][0] = -1.0;
    CHECK_FALSE(Quaternion::FromRotationMatrix(reflection).has_value());
}
