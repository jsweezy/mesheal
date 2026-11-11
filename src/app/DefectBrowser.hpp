#pragma once

#include "app/Revision.hpp"
#include "diagnostics/Histogram.hpp"
#include "diagnostics/MeshHealth.hpp"

#include <Eigen/Core>

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace mesheal {

/// Inclusive range of histogram display bins used as a filter.
struct BinRange {
    std::size_t first = 0;
    std::size_t last = 0;

    [[nodiscard]] bool contains(std::size_t bin) const noexcept {
        return bin >= first && bin <= last;
    }
};

/// Headless browsing state of the selected Mesh Health row: filters, visible rows, multi-selection,
/// focus and the list-cap decision (.docs/UI_DESIGN.md §6, §8). Rows are element indices into a
/// `CategoryResult`; "visible position" is a position in `visibleRows()`.
class DefectBrowser {
public:
    static constexpr std::size_t kNone = static_cast<std::size_t>(-1);

    /// Points the browser at `result` (or nothing). Clears selection, focus, filters and sorting.
    void setResult(const CategoryResult* result);

    [[nodiscard]] const CategoryResult* result() const noexcept {
        return result_;
    }

    // ---- Filters (changing one drops hidden rows from the selection and focus: FB-13) ----

    /// Shell scope (.docs/UI_DESIGN.md §16): element i is listed only when `mask[i]` is non-zero. An empty
    /// mask lists every element. Clears the bin filter, since the histogram changes.
    void setScopeMask(std::vector<std::uint8_t> mask);

    [[nodiscard]] bool inScope(std::size_t element) const noexcept {
        return scopeMask_.empty() || (element < scopeMask_.size() && scopeMask_[element] != 0);
    }

    /// In-scope elements per sub-type (index matches `CategoryInfo::subtypes`).
    [[nodiscard]] std::array<std::size_t, 4> subtypeCounts() const noexcept {
        return subtypeCounts_;
    }

    /// In-scope elements.
    [[nodiscard]] std::size_t scopedCount() const noexcept {
        return scopedCount_;
    }

    /// Sub-type filter (-1: all).
    [[nodiscard]] int subtypeFilter() const noexcept {
        return subtypeFilter_;
    }
    void setSubtypeFilter(int subtype);

    /// Histogram of the sub-type-filtered rows (empty when the category has no metric).
    [[nodiscard]] const MetricHistogram& histogram() const noexcept {
        return histogram_;
    }

    [[nodiscard]] const std::optional<BinRange>& binFilter() const noexcept {
        return binFilter_;
    }

    /// Selects bin `bin`; with `extend`, grows the active range to include it.
    void selectBin(std::size_t bin, bool extend);
    /// Moves the bin filter one non-empty bin left (-1) or right (+1); starts at the worst bin.
    void stepBin(int direction);
    void clearBinFilter();

    /// Rows passing the filters, in display order.
    [[nodiscard]] const std::vector<std::size_t>& visibleRows() const noexcept {
        return visibleRows_;
    }
    [[nodiscard]] bool isVisible(std::size_t element) const noexcept;
    [[nodiscard]] std::size_t visiblePositionOf(std::size_t element) const noexcept;

    /// Elements per display bin after the sub-type filter (for chip counts).
    [[nodiscard]] std::size_t countInBins(const BinRange& range) const noexcept;

    // ---- Sorting (-1 = worst first, the result order) ----
    [[nodiscard]] int sortColumn() const noexcept {
        return sortColumn_;
    }
    [[nodiscard]] bool sortDescending() const noexcept {
        return sortDescending_;
    }
    void setSort(int column, bool descending);

    // ---- Selection & focus ----

    /// Click on visible position `pos`: plain selects only it, `toggle` (Ctrl/Cmd) flips it, `range`
    /// (Shift) selects from the anchor. The clicked row becomes the focus when it ends up selected.
    void clickRow(std::size_t pos, bool toggle, bool range);
    void selectAllVisible();
    void selectNone();
    void invertSelection();

    [[nodiscard]] bool isSelected(std::size_t element) const noexcept;
    [[nodiscard]] std::size_t selectedCount() const noexcept {
        return selectedCount_;
    }
    /// Selected elements in display order.
    [[nodiscard]] std::vector<std::size_t> selectedElements() const;

    [[nodiscard]] std::size_t focusedElement() const noexcept {
        return focused_;
    }
    [[nodiscard]] bool hasFocus() const noexcept {
        return focused_ != kNone;
    }
    /// Focuses and selects only `element` (no-op if it is hidden by the filters).
    void focusElement(std::size_t element);
    void clearFocus() noexcept {
        revision_ = nextRevision();
        focused_ = kNone;
    }
    /// Steps the focus through the visible rows (wrapping). Returns false if nothing is visible.
    bool focusStep(int direction);

    /// First visible element whose faces include `face` (kNone if none).
    [[nodiscard]] std::size_t findVisibleElementWithFace(Eigen::Index face) const noexcept;

    /// True when the visible rows fit under the list cap (otherwise the Tool shows a summary).
    [[nodiscard]] bool showList(std::size_t listCap) const noexcept {
        return visibleRows_.size() <= listCap;
    }

    /// Changes with every change of the result, filters, sorting, selection or focus (see nextRevision).
    [[nodiscard]] std::uint64_t revision() const noexcept {
        return revision_;
    }

private:
    /// Recounts `subtypeCounts_` and `scopedCount_` (the Tool panel reads them every frame).
    void recountScope() noexcept;
    void rebuildHistogram();
    void rebuildVisible();
    void pruneHidden();
    /// In scope and of the filtered sub-type.
    [[nodiscard]] bool passesSubtype(std::size_t element) const noexcept;
    [[nodiscard]] std::size_t binOfElement(std::size_t element) const noexcept;
    void setSelected(std::size_t element, bool selected) noexcept;

    const CategoryResult* result_ = nullptr;
    std::vector<std::uint8_t> scopeMask_;
    int subtypeFilter_ = -1;
    std::optional<BinRange> binFilter_;
    MetricHistogram histogram_;
    int sortColumn_ = -1;
    bool sortDescending_ = false;

    std::vector<std::size_t> visibleRows_;
    std::vector<std::size_t> visiblePos_; ///< element -> visible position (kNone if hidden)
    std::vector<std::uint8_t> selected_;
    std::size_t selectedCount_ = 0;
    std::size_t anchorPos_ = kNone;
    std::size_t focused_ = kNone;
    std::array<std::size_t, 4> subtypeCounts_{};
    std::size_t scopedCount_ = 0;
    std::uint64_t revision_ = nextRevision();
};

} // namespace mesheal
