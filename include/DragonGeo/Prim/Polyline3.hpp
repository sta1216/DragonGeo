#pragma once

#include <cstddef>
#include <optional>
#include <span>
#include <vector>

#include <DragonGeo/Prim/Segment3.hpp>

namespace DragonGeo::Prim {

/// 三维折线：至少一段，且相邻段首尾精确相接。
///
/// 本步只是类型壳。段序列是私有的，只能经 `FromSegments` 构造，因此本类型不是聚合。
/// 相接判定使用点的 `operator==`，不容差。曲线协议方法在后续任务添加。
template <typename Scalar>
class Polyline3T {
public:
    using ScalarType = Scalar;

    /// 空序列，或任一相邻段不满足 `segments[i].B == segments[i + 1].A` 时返回空。
    [[nodiscard]] static std::optional<Polyline3T> FromSegments(
        std::span<const Segment3T<Scalar>> segments) {
        if (segments.empty()) {
            return std::nullopt;
        }
        for (std::size_t index = 0; index + 1 < segments.size(); ++index) {
            if (!(segments[index].B == segments[index + 1].A)) {
                return std::nullopt;
            }
        }
        return std::optional<Polyline3T>(Polyline3T{
            std::vector<Segment3T<Scalar>>(segments.begin(), segments.end())});
    }

    [[nodiscard]] constexpr std::size_t SegmentCount() const noexcept {
        return m_segments.size();
    }

    /// `index` 必须小于 `SegmentCount()`。越界是未定义行为。
    [[nodiscard]] constexpr Segment3T<Scalar> Segment(std::size_t index) const {
        return m_segments[index];
    }

    [[nodiscard]] constexpr std::size_t PointCount() const noexcept {
        return SegmentCount() + 1;
    }

    /// `index == 0` 是首段的 `A`，`index == SegmentCount()` 是末段的 `B`。
    /// 越界是未定义行为。
    [[nodiscard]] constexpr Linear::Point3T<Scalar> Point(std::size_t index) const {
        if (index == m_segments.size()) {
            return m_segments.back().B;
        }
        return m_segments[index].A;
    }

    /// 存储的每一段都有效时为真。
    [[nodiscard]] constexpr bool IsValid() const noexcept {
        for (const Segment3T<Scalar>& segment : m_segments) {
            if (!segment.IsValid()) {
                return false;
            }
        }
        return true;
    }

private:
    explicit Polyline3T(std::vector<Segment3T<Scalar>> segments)
        : m_segments(std::move(segments)) {}

    std::vector<Segment3T<Scalar>> m_segments;
};

/// 逐段比较存储的每一段。按 const 引用传递，避免复制整段序列。
/// `operator!=` 由 C++20 自动生成，不手写。
template <typename Scalar>
[[nodiscard]] constexpr bool operator==(
    const Polyline3T<Scalar>& a, const Polyline3T<Scalar>& b) noexcept {
    if (a.SegmentCount() != b.SegmentCount()) {
        return false;
    }
    for (std::size_t index = 0; index < a.SegmentCount(); ++index) {
        if (!(a.Segment(index) == b.Segment(index))) {
            return false;
        }
    }
    return true;
}

using Polyline3 = Polyline3T<double>;
using Polyline3f = Polyline3T<float>;

} // namespace DragonGeo::Prim
