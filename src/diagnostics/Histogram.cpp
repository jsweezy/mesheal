#include "diagnostics/Histogram.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace mesheal {

namespace {

[[nodiscard]] bool isBinnable(double value, bool logScale) noexcept {
    return std::isfinite(value) && (!logScale || value > 0.0);
}

} // namespace

std::size_t MetricHistogram::maxCount() const noexcept {
    return counts.empty() ? 0 : *std::max_element(counts.begin(), counts.end());
}

std::size_t MetricHistogram::binOf(double value) const noexcept {
    if (counts.empty()) {
        return 0;
    }
    const std::size_t n = counts.size();
    if (!isBinnable(value, logScale)) {
        return 0;
    }
    const double lo = logScale ? std::log10(minValue) : minValue;
    const double hi = logScale ? std::log10(maxValue) : maxValue;
    const double v = logScale ? std::log10(value) : value;
    std::size_t ascending = 0;
    if (hi > lo) {
        const double t = std::clamp((v - lo) / (hi - lo), 0.0, 1.0);
        ascending = std::min(n - 1, static_cast<std::size_t>(t * static_cast<double>(n)));
    }
    return lowIsWorst ? ascending : n - 1 - ascending;
}

double MetricHistogram::binLower(std::size_t bin) const noexcept {
    if (counts.empty()) {
        return 0.0;
    }
    const std::size_t n = counts.size();
    const std::size_t ascending = lowIsWorst ? bin : n - 1 - bin;
    const double t = static_cast<double>(ascending) / static_cast<double>(n);
    if (logScale) {
        return std::pow(10.0, std::log10(minValue) + t * (std::log10(maxValue) - std::log10(minValue)));
    }
    return minValue + t * (maxValue - minValue);
}

double MetricHistogram::binUpper(std::size_t bin) const noexcept {
    if (counts.empty()) {
        return 0.0;
    }
    const std::size_t n = counts.size();
    const std::size_t ascending = lowIsWorst ? bin : n - 1 - bin;
    const double t = static_cast<double>(ascending + 1) / static_cast<double>(n);
    if (logScale) {
        return std::pow(10.0, std::log10(minValue) + t * (std::log10(maxValue) - std::log10(minValue)));
    }
    return minValue + t * (maxValue - minValue);
}

MetricHistogram buildHistogram(std::span<const double> values, std::size_t numBins, bool logScale, bool lowIsWorst) {
    MetricHistogram histogram;
    histogram.logScale = logScale;
    histogram.lowIsWorst = lowIsWorst;
    if (values.empty() || numBins == 0) {
        return histogram;
    }

    double lo = std::numeric_limits<double>::infinity();
    double hi = -std::numeric_limits<double>::infinity();
    for (const double v : values) {
        if (isBinnable(v, logScale)) {
            lo = std::min(lo, v);
            hi = std::max(hi, v);
        }
    }
    if (!(lo <= hi)) {
        lo = hi = logScale ? 1.0 : 0.0;
    }
    histogram.minValue = lo;
    histogram.maxValue = hi;
    histogram.counts.assign(numBins, 0);
    for (const double v : values) {
        ++histogram.counts[histogram.binOf(v)];
    }
    return histogram;
}

} // namespace mesheal
