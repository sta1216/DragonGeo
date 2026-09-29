#include <catch2/catch_test_macros.hpp>

#include <GeoCore/linear/Matrix.hpp>

using GeoCore::linear::identity;
using GeoCore::linear::Matrix2;
using GeoCore::linear::Matrix3;
using GeoCore::linear::Matrix4;
using GeoCore::linear::transpose;
using GeoCore::linear::Vector2;
using GeoCore::linear::Vector3;

TEST_CASE("MatrixT is zero-initialized and supports aggregate init",
          "[linear][matrix]") {
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
    const Matrix3 unit = identity<double, 3>();

    const Matrix3 m{{{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}, {7.0, 8.0, 9.0}}};
    CHECK(unit * m == m);
    CHECK(m * unit == m);
}

TEST_CASE("matrix multiplication follows the row-column rule",
          "[linear][matrix]") {
    const Matrix2 a{{{1.0, 2.0}, {3.0, 4.0}}};
    const Matrix2 b{{{5.0, 6.0}, {7.0, 8.0}}};

    // [1 2] [5 6]   [19 22]
    // [3 4] [7 8] = [43 50]
    CHECK(a * b == Matrix2{{{19.0, 22.0}, {43.0, 50.0}}});
}

TEST_CASE("matrix multiplication is not commutative", "[linear][matrix]") {
    const Matrix2 a{{{1.0, 2.0}, {3.0, 4.0}}};
    const Matrix2 b{{{5.0, 6.0}, {7.0, 8.0}}};

    CHECK_FALSE(a * b == b * a);
}

TEST_CASE("transpose swaps rows and columns", "[linear][matrix]") {
    const Matrix2 m{{{1.0, 2.0}, {3.0, 4.0}}};
    CHECK(transpose(m) == Matrix2{{{1.0, 3.0}, {2.0, 4.0}}});

    // 转置两次回到自身
    CHECK(transpose(transpose(m)) == m);

    // 转置与乘法反交换
    const Matrix2 n{{{5.0, 6.0}, {7.0, 8.0}}};
    CHECK(transpose(m * n) == transpose(n) * transpose(m));
}

TEST_CASE("matrix addition and subtraction are component-wise",
          "[linear][matrix]") {
    const Matrix2 a{{{1.0, 2.0}, {3.0, 4.0}}};
    const Matrix2 b{{{5.0, 6.0}, {7.0, 8.0}}};

    CHECK(a + b == Matrix2{{{6.0, 8.0}, {10.0, 12.0}}});
    CHECK(b - a == Matrix2{{{4.0, 4.0}, {4.0, 4.0}}});
}

TEST_CASE("matrix times vector applies the row-column rule",
          "[linear][matrix]") {
    const Matrix2 m{{{1.0, 2.0}, {3.0, 4.0}}};
    const Vector2 v{5.0, 6.0};

    // [1 2] [5]   [17]
    // [3 4] [6] = [39]
    CHECK(m * v == Vector2{17.0, 39.0});
}

TEST_CASE("4x4 matrix times 4D vector", "[linear][matrix]") {
    const Matrix4 unit = identity<double, 4>();
    const GeoCore::linear::Vector4 v{1.0, 2.0, 3.0, 4.0};

    CHECK(unit * v == v);
}

TEST_CASE("3x3 rotation-like matrix transforms a vector", "[linear][matrix]") {
    // 绕 z 轴旋转 90°： (x, y) -> (-y, x)
    const Matrix3 rotate{{{0.0, -1.0, 0.0}, {1.0, 0.0, 0.0}, {0.0, 0.0, 1.0}}};
    const Vector3 v{1.0, 0.0, 0.0};

    const Vector3 rotated = rotate * v;
    CHECK(rotated.x == 0.0);
    CHECK(rotated.y == 1.0);
    CHECK(rotated.z == 0.0);
}
