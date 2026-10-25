#pragma once

#include "core/Settings.hpp"

#include <cstddef>
#include <span>
#include <vector>

namespace ogun {

/// Binned distribution of a defect metric, laid out worst first (bin 0 is the worst end).
/// Zero, negative (log scale) and non-finite values always fall into the worst bin.
struct MetricHistogram {
    bool logScale = false;
    bool lowIsWorst = true;
    double minValue = 0.0; ///< Smallest binned value (smallest positive value on a log scale)
    double maxValue = 0.0; ///< Largest binned value
    std::vector<std::size_t> counts;

    [[nodiscard]] bool empty() const noexcept {
        return counts.empty();
    }

    [[nodiscard]] std::size_t numBins() const noexcept {
        return counts.size();
    }

    [[nodiscard]] std::size_t maxCount() const noexcept;

    /// Display bin (worst first) that `value` falls into.
    [[nodiscard]] std::size_t binOf(double value) const noexcept;

    /// Value range [lower, upper] covered by display bin `bin` (lower <= upper).
    [[nodiscard]] double binLower(std::size_t bin) const noexcept;
    [[nodiscard]] double binUpper(std::size_t bin) const noexcept;
};

/// Bins `values` into `numBins` equal steps (logarithmic steps when `logScale`).
[[nodiscard]] MetricHistogram buildHistogram(
    std::span<const double> values,
    std::size_t numBins = settings::kHistogramBins,
    bool logScale = false,
    bool lowIsWorst = true);

} // namespace ogun
