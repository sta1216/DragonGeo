#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <type_traits>

#include <DragonGeo/Linear/Matrix.hpp>

using Catch::Approx;

using DragonGeo::Linear::Matrix2;
using DragonGeo::Linear::Matrix2f;
using DragonGeo::Linear::Matrix3;
using DragonGeo::Linear::Matrix3f;
using DragonGeo::Linear::Matrix4;
using DragonGeo::Linear::Matrix4f;
using DragonGeo::Linear::MatrixT;
using DragonGeo::Linear::Vector2;
using DragonGeo::Linear::Vector3;

TEST_CASE("the float aliases really are the float instantiations", "[linear][matrix]") {
    // 同 vector4Test.cpp：三个别名此前零命中，误绑定不会被任何断言发现。模板实参要写全 —— MatrixT 是两参数模板（宿主列数 N 在别名里写死）。
    STATIC_REQUIRE(std::is_same_v<Matrix2f, MatrixT<float, 2>>);
    STATIC_REQUIRE(std::is_same_v<Matrix3f, MatrixT<float, 3>>);
    STATIC_REQUIRE(std::is_same_v<Matrix4f, MatrixT<float, 4>>);
}

TEST_CASE("MatrixT is zero-initialized and supports aggregate init", "[linear][matrix]") {
    const Matrix2 zero{};
    CHECK(zero(0, 0) == 0.0);
    CHECK(zero(1, 1) == 0.0);

    const Matrix2 m{{{1.0, 2.0}, {3.0, 4.0}}};
    CHECK(m(0, 0) == 1.0);
    CHECK(m(0, 1) == 2.0);
    CHECK(m(1, 0) == 3.0);
    CHECK(m(1, 1) == 4.0);
}

TEST_CASE("identity is the multiplicative unit", "[linear][matrix]") {
    const Matrix3 unit = Matrix3::Identity();

    const Matrix3 m{{{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}, {7.0, 8.0, 9.0}}};
    CHECK(unit * m == m);
    CHECK(m * unit == m);
}

TEST_CASE("scalar multiplication is available in both orders", "[linear][matrix]") {
    const Matrix2 m{{{1.0, 2.0}, {3.0, 4.0}}};

    // 与 Vector / UnitVector 的接口保持一致：两种写法都要成立
    CHECK(m * 2.0 == Matrix2{{{2.0, 4.0}, {6.0, 8.0}}});
    CHECK(2.0 * m == Matrix2{{{2.0, 4.0}, {6.0, 8.0}}});
    CHECK(m * 2 == Matrix2{{{2.0, 4.0}, {6.0, 8.0}}});
}

TEST_CASE("unary minus negates every element", "[linear][matrix]") {
    const Matrix2 m{{{1.0, -2.0}, {3.0, 4.0}}};

    CHECK(-m == Matrix2{{{-1.0, 2.0}, {-3.0, -4.0}}});
    CHECK(m + (-m) == Matrix2{});
}

TEST_CASE("matrix multiplication follows the row-column rule", "[linear][matrix]") {
    const Matrix2 a{{{1.0, 2.0}, {3.0, 4.0}}};
    const Matrix2 b{{{5.0, 6.0}, {7.0, 8.0}}};

    // [1 2] [5 6]   [19 22] [3 4] [7 8] = [43 50]
    CHECK(a * b == Matrix2{{{19.0, 22.0}, {43.0, 50.0}}});
}

TEST_CASE("matrix multiplication is not commutative", "[linear][matrix]") {
    const Matrix2 a{{{1.0, 2.0}, {3.0, 4.0}}};
    const Matrix2 b{{{5.0, 6.0}, {7.0, 8.0}}};

    CHECK_FALSE(a * b == b * a);
}

TEST_CASE("transpose swaps rows and columns", "[linear][matrix]") {
    const Matrix2 m{{{1.0, 2.0}, {3.0, 4.0}}};
    CHECK(m.Transposed() == Matrix2{{{1.0, 3.0}, {2.0, 4.0}}});

    // 转置两次回到自身
    CHECK(m.Transposed().Transposed() == m);

    // 转置与乘法反交换
    const Matrix2 n{{{5.0, 6.0}, {7.0, 8.0}}};
    CHECK((m * n).Transposed() == n.Transposed() * m.Transposed());
}

TEST_CASE("matrix addition and subtraction are component-wise", "[linear][matrix]") {
    const Matrix2 a{{{1.0, 2.0}, {3.0, 4.0}}};
    const Matrix2 b{{{5.0, 6.0}, {7.0, 8.0}}};

    CHECK(a + b == Matrix2{{{6.0, 8.0}, {10.0, 12.0}}});
    CHECK(b - a == Matrix2{{{4.0, 4.0}, {4.0, 4.0}}});
}

TEST_CASE("matrix times vector applies the row-column rule", "[linear][matrix]") {
    const Matrix2 m{{{1.0, 2.0}, {3.0, 4.0}}};
    const Vector2 v{5.0, 6.0};

    // [1 2] [5]   [17] [3 4] [6] = [39]
    CHECK(m * v == Vector2{17.0, 39.0});
}

TEST_CASE("4x4 matrix times 4D vector", "[linear][matrix]") {
    const Matrix4 unit = Matrix4::Identity();
    const DragonGeo::Linear::Vector4 v{1.0, 2.0, 3.0, 4.0};

    CHECK(unit * v == v);
}

TEST_CASE("3x3 rotation-like matrix transforms a vector", "[linear][matrix]") {
    // 绕 z 轴旋转 90°： (x, y) -> (-y, x)
    const Matrix3 rotate{{{0.0, -1.0, 0.0}, {1.0, 0.0, 0.0}, {0.0, 0.0, 1.0}}};
    const Vector3 v{1.0, 0.0, 0.0};

    const Vector3 rotated = rotate * v;
    CHECK(rotated.X == 0.0);
    CHECK(rotated.Y == 1.0);
    CHECK(rotated.Z == 0.0);
}

TEST_CASE("3x3 matrix times vector pins every term", "[linear][matrix]") {
    // 上一条用 v = (1,0,0)：两个零分量消灭了九项中的六项，三个期望值里又有两个是 0，因此丢掉某一行里的 z 项、或交换任意 y/z 系数，都仍能通过。这里改用非对称矩阵乘以全非零向量，把每一项各自钉死。
    const Matrix3 m{{{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}, {7.0, 8.0, 10.0}}};
    const Vector3 v{1.0, 2.0, 3.0};

    // C[0] = 1 + 4  + 9  = 14 C[1] = 4 + 10 + 18 = 32 C[2] = 7 + 16 + 30 = 53
    CHECK(m * v == Vector3{14.0, 32.0, 53.0});
}

TEST_CASE("4x4 matrix times vector pins every term", "[linear][matrix]") {
    // 上一条用 Matrix4::Identity()：单位阵对称，一个完全转置的 4×4 实现也能通过，且非对角项全为零使系数错位无从暴露。这里同样逐项钉死。
    const Matrix4 m{{{1.0, 2.0, 3.0, 4.0}, {5.0, 6.0, 7.0, 8.0}, {9.0, 10.0, 11.0, 12.0}, {13.0, 14.0, 15.0, 17.0}}};
    const DragonGeo::Linear::Vector4 v{1.0, 2.0, 3.0, 4.0};

    // C[0] = 1 + 4  + 9  + 16 = 30 C[1] = 5 + 12 + 21 + 32 = 70 C[2] = 9 + 20 + 33 + 48 = 110 C[3] = 13 + 28 + 45 + 68 = 154
    CHECK(m * v == DragonGeo::Linear::Vector4{30.0, 70.0, 110.0, 154.0});
}

TEST_CASE("Matrix members: determinant, transposed, inverse, identity", "[linear][matrix]") {
    const Matrix2 m{{{1.0, 2.0}, {3.0, 4.0}}};

    CHECK(m.Determinant() == -2.0);
    CHECK(m.Transposed() == Matrix2{{{1.0, 3.0}, {2.0, 4.0}}});
    CHECK(Matrix2::Identity() == Matrix2{{{1.0, 0.0}, {0.0, 1.0}}});

    const auto inv = m.Inverse();
    REQUIRE(inv.has_value());
    CHECK(inv->Determinant() == Approx(-0.5));
}
