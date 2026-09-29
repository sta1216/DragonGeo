#pragma once

#include <concepts>
#include <optional>

#include <GeoCore/core/Numeric.hpp>
#include <GeoCore/core/Tolerance.hpp>
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

    /// 单位阵。
    [[nodiscard]] static constexpr MatrixT identity() noexcept {
        MatrixT<Scalar, N> result{};
        for (int i = 0; i < N; ++i) {
            result.data[i][i] = Scalar{1};
        }
        return result;
    }

    /// 转置：交换行与列。
    ///
    /// 命名为过去分词以区别于原地操作，与 normalized() 一致。
    [[nodiscard]] constexpr MatrixT transposed() const noexcept {
        MatrixT<Scalar, N> result{};
        for (int i = 0; i < N; ++i) {
            for (int j = 0; j < N; ++j) {
                result.data[i][j] = data[j][i];
            }
        }
        return result;
    }

    /// 2×2 / 3×3 / 4×4 的行列式。
    ///
    /// 用 if constexpr 写成单一实现，而不是为每个尺寸提供一个重载：单一
    /// 实现能借 static_assert 给出「只支持 2/3/4」这样明确的编译期诊断，
    /// 也免去维护三份几乎相同的签名。
    ///
    /// 展开为闭式解（4×4 用第一行余子式展开）而非通用 LU 分解：固定尺寸的
    /// 展开式没有循环开销，编译器也能完全内联。
    [[nodiscard]] constexpr Scalar determinant() const noexcept {
        static_assert(N == 2 || N == 3 || N == 4,
                      "determinant is implemented for 2x2, 3x3 and 4x4 matrices only");

        if constexpr (N == 2) {
            return data[0][0] * data[1][1] - data[0][1] * data[1][0];
        } else if constexpr (N == 3) {
            return data[0][0] * (data[1][1] * data[2][2] - data[1][2] * data[2][1])
                 - data[0][1] * (data[1][0] * data[2][2] - data[1][2] * data[2][0])
                 + data[0][2] * (data[1][0] * data[2][1] - data[1][1] * data[2][0]);
        } else {
            const Scalar sub_00 = data[1][1] * (data[2][2] * data[3][3] - data[2][3] * data[3][2])
                                - data[1][2] * (data[2][1] * data[3][3] - data[2][3] * data[3][1])
                                + data[1][3] * (data[2][1] * data[3][2] - data[2][2] * data[3][1]);
            const Scalar sub_01 = data[1][0] * (data[2][2] * data[3][3] - data[2][3] * data[3][2])
                                - data[1][2] * (data[2][0] * data[3][3] - data[2][3] * data[3][0])
                                + data[1][3] * (data[2][0] * data[3][2] - data[2][2] * data[3][0]);
            const Scalar sub_02 = data[1][0] * (data[2][1] * data[3][3] - data[2][3] * data[3][1])
                                - data[1][1] * (data[2][0] * data[3][3] - data[2][3] * data[3][0])
                                + data[1][3] * (data[2][0] * data[3][1] - data[2][1] * data[3][0]);
            const Scalar sub_03 = data[1][0] * (data[2][1] * data[3][2] - data[2][2] * data[3][1])
                                - data[1][1] * (data[2][0] * data[3][2] - data[2][2] * data[3][0])
                                + data[1][2] * (data[2][0] * data[3][1] - data[2][1] * data[3][0]);

            return data[0][0] * sub_00 - data[0][1] * sub_01
                 + data[0][2] * sub_02 - data[0][3] * sub_03;
        }
    }

    /// 逆矩阵。行列式在给定容差下可视为零（即奇异）时返回 std::nullopt ——
    /// 参与比较的是双侧平衡后的行列式，理由见函数体首段注释。
    ///
    /// 用 optional 而非抛出异常：奇异矩阵是数学事实，不是程序错误，调用者
    /// 有责任处理这个分支。返回 std::nullopt 也让调用者不可能拿到一个含
    /// inf / NaN 的矩阵。
    ///
    /// 与 determinant 一样用 if constexpr 写成单一实现。
    [[nodiscard]] constexpr std::optional<MatrixT> inverse(
        core::Tolerance tolerance = {}) const noexcept {
        static_assert(N == 2 || N == 3 || N == 4,
                      "inverse is implemented for 2x2, 3x3 and 4x4 matrices only");

        // 行列式是 N 阶量，直接与长度容差比较是量纲错误：s = 1e-4 时 det = 1e-12
        // 会被默认容差的绝对项判为奇异，而 s = 1e150 时 det 会先溢出成 ±inf。
        //
        // 双侧平衡：先逐行、再逐列，各除以本行 / 本列的最大绝对元素。
        //
        // 记 R = diag(row_scale)、C = diag(column_scale)，则
        //     B = R⁻¹·A        （第 i 行除以 row_scale[i]）
        //     N = B·C⁻¹        （B 的第 j 列除以 column_scale[j]）
        // 于是
        //     A = R·N·C        且        A⁻¹ = C⁻¹·N⁻¹·R⁻¹
        // 即还原时元素 (i, j) 要除以 column_scale[i] 与 row_scale[j] —— **行号取
        // 列尺度、列号取行尺度**。两个对角因子一左一右，下标极易写反，而对称矩阵
        // 上的测试看不出这个错误。
        //
        // 为什么必须两侧都做：只做逐行平衡时，translation_3d(t) 的第 0 行 (1,0,0,t)
        // 变成 (1/t,0,0,1)，行列式恰为 1/t，t ≥ 1e12 就被默认容差的绝对项（1e-12）
        // 判成奇异 —— 可它的真逆是精确平移 -t，既存在又精确可表示。逐列平衡把第 0
        // 列重新放大回 1，行列式回到 1，与 t 无关。这正是本函数该有的语义：可逆性
        // 由条件数（即平衡后行列式与 0 的距离）决定，而不是由矩阵的整体尺度决定 ——
        // 否则 scaling_3d(1e150)（条件数 1e150）可逆，而 translation_3d(1e12)
        // （条件数 1e12）反倒「奇异」，自相矛盾。
        //
        // 「按整个矩阵的最大元素归一化」同样不成立 —— scaling_3d 返回的是齐次矩阵，
        // 其第 4 行恒为 (0,0,0,1)，最大元素永远是 1，于是小尺度缩放的 det 依旧是
        // 1e-12。逐行 / 逐列平衡不假设各行、各列元素同量级，正合此处；平衡后的
        // 行列式落在由 N 决定的小常数之内，既不会溢出，也不会因量纲而与一阶容差
        // 不可比。
        //
        // 折叠用滚动比较而非 max_abs_of：这里与 length() 不同，非有限分量不需要
        // 在折叠里显式保留 —— 它要么被跳过（与 row_max 的比较恒为 false），要么
        // 让 row_max 变成 ±inf，两种情形都会在 balanced 里留下 NaN，交由下面的
        // 行列式守卫处置。
        Scalar row_scale[N];
        for (int i = 0; i < N; ++i) {
            Scalar row_max = Scalar{0};
            for (int j = 0; j < N; ++j) {
                const Scalar magnitude = core::absolute_value(data[i][j]);
                if (magnitude > row_max) {
                    row_max = magnitude;
                }
            }
            if (row_max == Scalar{0}) {
                return std::nullopt;   // 整行为零 ⇒ 必然奇异
            }
            row_scale[i] = row_max;
        }

        // 列平衡作用在**已完成行平衡的** B 上，而不是原始矩阵：两级平衡必须依次
        // 施加，后一级的量尺要在前一级的结果上量。若在原始元素上取列尺度，行与行
        // 的量级差会重新混进列里 —— 此时 N 的列最大元素是 1/r_i（而非 1），行、列
        // 都没有归一化到 1，行列式「落在由 N 决定的小常数之内」这条保证随之失效。
        MatrixT<Scalar, N> balanced{};
        for (int i = 0; i < N; ++i) {
            for (int j = 0; j < N; ++j) {
                balanced.data[i][j] = data[i][j] / row_scale[i];
            }
        }

        Scalar column_scale[N];
        for (int j = 0; j < N; ++j) {
            Scalar column_max = Scalar{0};
            for (int i = 0; i < N; ++i) {
                const Scalar magnitude = core::absolute_value(balanced.data[i][j]);
                if (magnitude > column_max) {
                    column_max = magnitude;
                }
            }
            // 行尺度非零，故 B 的整列为零当且仅当 A 的该列整列为零 ⇒ 必然奇异。
            if (column_max == Scalar{0}) {
                return std::nullopt;
            }
            column_scale[j] = column_max;
        }

        for (int i = 0; i < N; ++i) {
            for (int j = 0; j < N; ++j) {
                balanced.data[i][j] /= column_scale[j];
            }
        }

        const Scalar det = balanced.determinant();
        // 行列式非有限时同样返回 nullopt。若只依赖容差判断，含 NaN 或 ±inf 的
        // 矩阵会得到一个 has_value() 为真、内容全是 NaN 的逆矩阵 —— 调用者无从
        // 察觉，而 NaN 会污染后续全部计算。这与 Vector3T::normalized 的裁定是同一条原则。
        if (!core::is_finite(static_cast<double>(det))
            || tolerance.is_zero(static_cast<double>(det))) {
            return std::nullopt;
        }
        const Scalar inv_det = Scalar{1} / det;

        MatrixT<Scalar, N> result{};
        if constexpr (N == 2) {
            result.data[0][0] =  balanced.data[1][1] * inv_det;
            result.data[0][1] = -balanced.data[0][1] * inv_det;
            result.data[1][0] = -balanced.data[1][0] * inv_det;
            result.data[1][1] =  balanced.data[0][0] * inv_det;
        } else if constexpr (N == 3) {
            result.data[0][0] = (balanced.data[1][1] * balanced.data[2][2] - balanced.data[1][2] * balanced.data[2][1]) * inv_det;
            result.data[0][1] = (balanced.data[0][2] * balanced.data[2][1] - balanced.data[0][1] * balanced.data[2][2]) * inv_det;
            result.data[0][2] = (balanced.data[0][1] * balanced.data[1][2] - balanced.data[0][2] * balanced.data[1][1]) * inv_det;
            result.data[1][0] = (balanced.data[1][2] * balanced.data[2][0] - balanced.data[1][0] * balanced.data[2][2]) * inv_det;
            result.data[1][1] = (balanced.data[0][0] * balanced.data[2][2] - balanced.data[0][2] * balanced.data[2][0]) * inv_det;
            result.data[1][2] = (balanced.data[0][2] * balanced.data[1][0] - balanced.data[0][0] * balanced.data[1][2]) * inv_det;
            result.data[2][0] = (balanced.data[1][0] * balanced.data[2][1] - balanced.data[1][1] * balanced.data[2][0]) * inv_det;
            result.data[2][1] = (balanced.data[0][1] * balanced.data[2][0] - balanced.data[0][0] * balanced.data[2][1]) * inv_det;
            result.data[2][2] = (balanced.data[0][0] * balanced.data[1][1] - balanced.data[0][1] * balanced.data[1][0]) * inv_det;
        } else {
            // 4×4 按伴随矩阵求逆：result(i, j) = cofactor(j, i) / det。
            // 先用 2×2 子式算出每个 3×3 余子式，再按符号填入转置位置。
            const auto minor3 = [&balanced](int skip_row, int skip_column) noexcept -> Scalar {
                Scalar block[3][3];
                int r = 0;
                for (int i = 0; i < 4; ++i) {
                    if (i == skip_row) { continue; }
                    int c = 0;
                    for (int j = 0; j < 4; ++j) {
                        if (j == skip_column) { continue; }
                        block[r][c] = balanced.data[i][j];
                        ++c;
                    }
                    ++r;
                }
                return block[0][0] * (block[1][1] * block[2][2] - block[1][2] * block[2][1])
                     - block[0][1] * (block[1][0] * block[2][2] - block[1][2] * block[2][0])
                     + block[0][2] * (block[1][0] * block[2][1] - block[1][1] * block[2][0]);
            };

            for (int i = 0; i < 4; ++i) {
                for (int j = 0; j < 4; ++j) {
                    const Scalar cofactor = minor3(j, i);
                    const Scalar sign = ((i + j) % 2 == 0) ? Scalar{1} : Scalar{-1};
                    result.data[i][j] = sign * cofactor * inv_det;
                }
            }
        }

        // A = R·N·C ⇒ A^-1 = C^-1·N^-1·R^-1，即元素 (i, j) 除以 column_scale[i] 与
        // row_scale[j]：**行号配列尺度、列号配行尺度**，与直觉相反。result 里装的
        // 是 N^-1，两个尺度一左一右夹着它；写成 row_scale[i] / column_scale[j] 会
        // 让非对角元出错，而对称矩阵上的测试未必看得出来。
        //
        // 两次除法分开写，而不是先乘出 1/(column_scale[i] · row_scale[j])：尺度量级
        // 相差悬殊时，这个中间乘积是唯一会脱离 double 范围的量（一侧可以极小、
        // 另一侧可以极大），先算出它会在结果本身完全可表示时就提前上溢或下溢成
        // 0 / ±inf。分两步除，是否溢出只取决于最终元素，那才是真实逆矩阵的固有性质。
        //
        // 还原尺度后元素可能重新溢出（真实逆确实可能巨大），必须再检查一次，否则
        // 又会交出一个 has_value() 为真、内容却是 inf 的矩阵 —— 那正是先前裁定
        // 禁止的形态。
        for (int i = 0; i < N; ++i) {
            for (int j = 0; j < N; ++j) {
                result.data[i][j] /= column_scale[i];
                result.data[i][j] /= row_scale[j];
                if (!core::is_finite(static_cast<double>(result.data[i][j]))) {
                    return std::nullopt;
                }
            }
        }
        return result;
    }
};

using Matrix2 = MatrixT<double, 2>;
using Matrix2f = MatrixT<float, 2>;
using Matrix3 = MatrixT<double, 3>;
using Matrix3f = MatrixT<float, 3>;
using Matrix4 = MatrixT<double, 4>;
using Matrix4f = MatrixT<float, 4>;

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

/// 标量 × 矩阵。与 operator*(matrix, factor) 对称 —— 缺了它 `2.0 * m`
/// 会编译失败，而 Vector 与 UnitVector 都提供两种写法。
template <typename Scalar, int N, typename Factor>
    requires std::convertible_to<Factor, Scalar>
[[nodiscard]] constexpr MatrixT<Scalar, N> operator*(
    Factor factor, const MatrixT<Scalar, N>& m) noexcept {
    return m * factor;
}

/// 逐元素取负。
template <typename Scalar, int N>
[[nodiscard]] constexpr MatrixT<Scalar, N> operator-(const MatrixT<Scalar, N>& m) noexcept {
    MatrixT<Scalar, N> result{};
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) {
            result.data[i][j] = -m.data[i][j];
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
