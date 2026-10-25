#include "app/DefectBrowser.hpp"

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <vector>

namespace {

/// Thin-face style result: element i has metric 10^-(n-i) (worst first) and subtype i % 2.
ogun::CategoryResult makeResult(std::size_t n) {
    ogun::CategoryResult result;
    result.category = ogun::DefectCategory::ThinFaces;
    for (std::size_t i = 0; i < n; ++i) {
        ogun::DefectElement e;
        e.index = static_cast<Eigen::Index>(100 + i);
        e.metric = std::pow(10.0, -static_cast<double>(n - i));
        e.columns = {e.metric, static_cast<double>(n - i), 0.0};
        e.subtype = static_cast<int>(i % 2);
        result.elements.push_back(e);
        result.faces.push_back(e.index);
        result.faceOffsets.push_back(static_cast<std::uint32_t>(result.faces.size()));
    }
    return result;
}

} // namespace

TEST_CASE("DefectBrowser click, Ctrl-toggle and Shift-range selection", "[app][browser]") {
    const ogun::CategoryResult result = makeResult(6);
    ogun::DefectBrowser browser;
    browser.setResult(&result);
    REQUIRE(browser.visibleRows().size() == 6);
    REQUIRE_FALSE(browser.hasFocus());

    browser.clickRow(1, false, false);
    REQUIRE(browser.selectedCount() == 1);
    REQUIRE(browser.focusedElement() == 1);

    browser.clickRow(4, false, true); // Shift: 1..4
    REQUIRE(browser.selectedCount() == 4);
    REQUIRE(browser.focusedElement() == 4);

    browser.clickRow(2, true, false); // Ctrl: deselect 2
    REQUIRE(browser.selectedCount() == 3);
    REQUIRE_FALSE(browser.isSelected(2));
    REQUIRE(browser.focusedElement() == 4);

    browser.clickRow(4, true, false); // Ctrl on the focus: deselect and drop the focus
    REQUIRE_FALSE(browser.hasFocus());
    REQUIRE(browser.selectedElements() == std::vector<std::size_t>{1, 3});

    browser.invertSelection();
    REQUIRE(browser.selectedElements() == std::vector<std::size_t>{0, 2, 4, 5});
    browser.selectAllVisible();
    REQUIRE(browser.selectedCount() == 6);
    browser.selectNone();
    REQUIRE(browser.selectedCount() == 0);
}

TEST_CASE("DefectBrowser filters prune hidden selection and focus (FB-13)", "[app][browser]") {
    const ogun::CategoryResult result = makeResult(6);
    ogun::DefectBrowser browser;
    browser.setResult(&result);

    browser.clickRow(0, false, false);
    browser.clickRow(3, true, false); // Ctrl-click: element 3 (subtype 1) becomes the focus
    REQUIRE(browser.selectedCount() == 2);
    REQUIRE(browser.focusedElement() == 3);

    browser.setSubtypeFilter(0); // only even elements stay visible
    REQUIRE(browser.visibleRows() == std::vector<std::size_t>{0, 2, 4});
    REQUIRE_FALSE(browser.hasFocus());
    REQUIRE(browser.selectedElements() == std::vector<std::size_t>{0});
    REQUIRE_FALSE(browser.isVisible(3));
    REQUIRE(browser.visiblePositionOf(4) == 2);

    // The histogram covers only the sub-type-filtered rows; a bin filter hides the rest
    REQUIRE(browser.histogram().numBins() == ogun::settings::kHistogramBins);
    browser.focusElement(4);
    browser.selectBin(0, false);
    REQUIRE(browser.binFilter().has_value());
    REQUIRE(browser.visibleRows() == std::vector<std::size_t>{0});
    REQUIRE_FALSE(browser.hasFocus());
    REQUIRE(browser.countInBins(*browser.binFilter()) == 1);

    browser.selectBin(ogun::settings::kHistogramBins - 1, true); // Shift: grow the range to every bin
    REQUIRE(browser.visibleRows().size() == 3);
    browser.clearBinFilter();
    browser.setSubtypeFilter(-1);
    REQUIRE(browser.visibleRows().size() == 6);
}

TEST_CASE("DefectBrowser steps the focus worst first within the filter and wraps", "[app][browser]") {
    const ogun::CategoryResult result = makeResult(5);
    ogun::DefectBrowser browser;
    browser.setResult(&result);

    REQUIRE(browser.focusStep(+1));
    REQUIRE(browser.focusedElement() == 0);
    REQUIRE(browser.selectedCount() == 1);
    REQUIRE(browser.focusStep(+1));
    REQUIRE(browser.focusedElement() == 1);
    REQUIRE(browser.focusStep(-1));
    REQUIRE(browser.focusStep(-1));
    REQUIRE(browser.focusedElement() == 4);

    browser.setSubtypeFilter(1); // elements 1, 3
    REQUIRE(browser.focusStep(+1));
    REQUIRE(browser.focusedElement() == 1);
    REQUIRE(browser.focusStep(+1));
    REQUIRE(browser.focusedElement() == 3);
    REQUIRE(browser.focusStep(+1));
    REQUIRE(browser.focusedElement() == 1);

    REQUIRE(browser.findVisibleElementWithFace(103) == 3);
    REQUIRE(browser.findVisibleElementWithFace(102) == ogun::DefectBrowser::kNone);
}

TEST_CASE("DefectBrowser list cap, bin stepping, sorting and reset", "[app][browser]") {
    const ogun::CategoryResult result = makeResult(8);
    ogun::DefectBrowser browser;
    browser.setResult(&result);
    REQUIRE(browser.showList(8));
    REQUIRE_FALSE(browser.showList(7));

    browser.stepBin(+1); // first non-empty bin from the worst end
    REQUIRE(browser.binFilter()->first == 0);
    REQUIRE(browser.visibleRows() == std::vector<std::size_t>{0});
    browser.stepBin(+1);
    REQUIRE(browser.visibleRows().size() == 1);
    REQUIRE(browser.visibleRows()[0] == 1);
    REQUIRE(browser.showList(1));
    browser.clearBinFilter();

    browser.setSort(1, false); // column 1 = n - i, ascending reverses the order
    REQUIRE(browser.visibleRows().front() == 7);
    browser.setSort(-1, false);
    REQUIRE(browser.visibleRows().front() == 0);

    browser.setResult(nullptr);
    REQUIRE(browser.visibleRows().empty());
    REQUIRE(browser.histogram().empty());
    REQUIRE_FALSE(browser.focusStep(+1));
}
