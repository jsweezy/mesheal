#include "diagnostics/Histogram.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <limits>
#include <numeric>
#include <vector>

TEST_CASE("Linear histogram bins values low-is-worst from the left", "[diagnostics][histogram]") {
    const std::vector<double> values = {0.0, 1.0, 2.0, 3.0, 4.0, 10.0};
    const ogun::MetricHistogram h = ogun::buildHistogram(values, 10, false, true);
    REQUIRE(h.numBins() == 10);
    REQUIRE(std::accumulate(h.counts.begin(), h.counts.end(), std::size_t{0}) == values.size());
    REQUIRE(h.binOf(0.0) == 0);
    REQUIRE(h.binOf(10.0) == 9);
    REQUIRE(h.binOf(1.0) == 1);
    REQUIRE(h.binLower(0) == Catch::Approx(0.0));
    REQUIRE(h.binUpper(9) == Catch::Approx(10.0));
    REQUIRE(h.maxCount() == 1);
}

TEST_CASE("High-is-worst histograms mirror the bins", "[diagnostics][histogram]") {
    const std::vector<double> values = {1.0, 2.0, 3.0, 9.0};
    const ogun::MetricHistogram h = ogun::buildHistogram(values, 4, false, false);
    REQUIRE(h.binOf(9.0) == 0);
    REQUIRE(h.binOf(1.0) == 3);
    REQUIRE(h.binLower(0) > h.binLower(3));
    REQUIRE(h.binLower(0) < h.binUpper(0));
}

TEST_CASE("Log histograms put zeros and non-finite values in the worst bin", "[diagnostics][histogram]") {
    const std::vector<double> values = {0.0, 1.0e-8, 1.0e-6, 1.0e-4, std::numeric_limits<double>::infinity()};
    const ogun::MetricHistogram h = ogun::buildHistogram(values, 4, true, true);
    REQUIRE(h.minValue == Catch::Approx(1.0e-8));
    REQUIRE(h.maxValue == Catch::Approx(1.0e-4));
    REQUIRE(h.binOf(0.0) == 0);
    REQUIRE(h.binOf(std::numeric_limits<double>::infinity()) == 0);
    REQUIRE(h.binOf(1.0e-8) == 0);
    REQUIRE(h.binOf(1.0e-6) == 2);
    REQUIRE(h.binOf(1.0e-4) == 3);
    REQUIRE(h.counts[0] == 3);
    REQUIRE(h.binUpper(1) == Catch::Approx(1.0e-6));
}

TEST_CASE("Histograms of empty or constant data are well defined", "[diagnostics][histogram]") {
    REQUIRE(ogun::buildHistogram({}, 24).empty());
    const std::vector<double> same = {2.0, 2.0, 2.0};
    const ogun::MetricHistogram h = ogun::buildHistogram(same, 24);
    REQUIRE(h.counts[0] == 3);
}
