#pragma once

#include <GeoCore/linear/Vector2.hpp>
#include <GeoCore/linear/Vector3.hpp>
#include <GeoCore/linear/Vector4.hpp>

namespace GeoCore::linear {

/// N×N 方阵，行主序存储。
///
/// 用二维数组而非命名字段：4×4 有 16 个元素，逐一命名反而降低可读性。
/// 通过 operator()(row, column) 访问以保持「先行后列」的一致读法。
template <typename Scalar, int N>
struct MatrixT {
    using scalar_type = Scalar;
    static constexpr int dimension = N;

    Scalar data[N][N]{};

    [[nodiscard]] constexpr Scalar& operator()(int row, int column) noexcept {
        return data[row][column];
    }

    [[nodiscard]] constexpr Scalar operator()(int row, int column) const noexcept {
        return data[row][column];
    }
};

using Matrix2 = MatrixT<double, 2>;
using Matrix2f = MatrixT<float, 2>;
using Matrix3 = MatrixT<double, 3>;
using Matrix3f = MatrixT<float, 3>;
using Matrix4 = MatrixT<double, 4>;
using Matrix4f = MatrixT<float, 4>;

// ---- 构造与变换 ----

template <typename Scalar, int N>
[[nodiscard]] constexpr MatrixT<Scalar, N> identity() noexcept {
    MatrixT<Scalar, N> result{};
    for (int i = 0; i < N; ++i) {
        result.data[i][i] = Scalar{1};
    }
    return result;
}

template <typename Scalar, int N>
[[nodiscard]] constexpr MatrixT<Scalar, N> transpose(const MatrixT<Scalar, N>& m) noexcept {
    MatrixT<Scalar, N> result{};
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) {
            result.data[i][j] = m.data[j][i];
        }
    }
    return result;
}

// ---- 运算符 ----

template <typename Scalar, int N>
[[nodiscard]] constexpr MatrixT<Scalar, N> operator+(
    const MatrixT<Scalar, N>& a, const MatrixT<Scalar, N>& b) noexcept {
    MatrixT<Scalar, N> result{};
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) {
            result.data[i][j] = a.data[i][j] + b.data[i][j];
        }
    }
    return result;
}

template <typename Scalar, int N>
[[nodiscard]] constexpr MatrixT<Scalar, N> operator-(
    const MatrixT<Scalar, N>& a, const MatrixT<Scalar, N>& b) noexcept {
    MatrixT<Scalar, N> result{};
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) {
            result.data[i][j] = a.data[i][j] - b.data[i][j];
        }
    }
    return result;
}

template <typename Scalar, int N>
[[nodiscard]] constexpr MatrixT<Scalar, N> operator*(
    const MatrixT<Scalar, N>& a, const MatrixT<Scalar, N>& b) noexcept {
    MatrixT<Scalar, N> result{};
    for (int i = 0; i < N; ++i) {
        for (int k = 0; k < N; ++k) {
            const Scalar factor = a.data[i][k];
            for (int j = 0; j < N; ++j) {
                result.data[i][j] += factor * b.data[k][j];
            }
        }
    }
    return result;
}

template <typename Scalar, int N, typename Factor>
    requires std::convertible_to<Factor, Scalar>
[[nodiscard]] constexpr MatrixT<Scalar, N> operator*(
    const MatrixT<Scalar, N>& m, Factor factor) noexcept {
    const auto scale = static_cast<Scalar>(factor);
    MatrixT<Scalar, N> result{};
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) {
            result.data[i][j] = m.data[i][j] * scale;
        }
    }
    return result;
}

template <typename Scalar, int N>
[[nodiscard]] constexpr bool operator==(
    const MatrixT<Scalar, N>& a, const MatrixT<Scalar, N>& b) noexcept {
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) {
            if (a.data[i][j] != b.data[i][j]) {
                return false;
            }
        }
    }
    return true;
}

// ---- 矩阵 × 向量 ----
//
// 三个维度各写一个重载，而不是写一个泛型版本：泛型版本需要把向量
// 也参数化，会立刻把调用点拖进模板推导的泥潭，得不偿失。

template <typename Scalar>
[[nodiscard]] constexpr Vector2T<Scalar> operator*(
    const MatrixT<Scalar, 2>& m, Vector2T<Scalar> v) noexcept {
    return Vector2T<Scalar>{
        m.data[0][0] * v.x + m.data[0][1] * v.y,
        m.data[1][0] * v.x + m.data[1][1] * v.y,
    };
}

template <typename Scalar>
[[nodiscard]] constexpr Vector3T<Scalar> operator*(
    const MatrixT<Scalar, 3>& m, Vector3T<Scalar> v) noexcept {
    return Vector3T<Scalar>{
        m.data[0][0] * v.x + m.data[0][1] * v.y + m.data[0][2] * v.z,
        m.data[1][0] * v.x + m.data[1][1] * v.y + m.data[1][2] * v.z,
        m.data[2][0] * v.x + m.data[2][1] * v.y + m.data[2][2] * v.z,
    };
}

template <typename Scalar>
[[nodiscard]] constexpr Vector4T<Scalar> operator*(
    const MatrixT<Scalar, 4>& m, Vector4T<Scalar> v) noexcept {
    return Vector4T<Scalar>{
        m.data[0][0] * v.x + m.data[0][1] * v.y + m.data[0][2] * v.z + m.data[0][3] * v.w,
        m.data[1][0] * v.x + m.data[1][1] * v.y + m.data[1][2] * v.z + m.data[1][3] * v.w,
        m.data[2][0] * v.x + m.data[2][1] * v.y + m.data[2][2] * v.z + m.data[2][3] * v.w,
        m.data[3][0] * v.x + m.data[3][1] * v.y + m.data[3][2] * v.z + m.data[3][3] * v.w,
    };
}

} // namespace GeoCore::linear
