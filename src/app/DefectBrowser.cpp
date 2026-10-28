#include "app/DefectBrowser.hpp"

#include <algorithm>
#include <numeric>
#include <utility>

namespace ogun {

void DefectBrowser::setResult(const CategoryResult* result) {
    revision_ = nextRevision();
    result_ = result;
    scopeMask_.clear();
    subtypeFilter_ = -1;
    binFilter_.reset();
    sortColumn_ = -1;
    sortDescending_ = false;
    const std::size_t n = result_ != nullptr ? result_->size() : 0;
    selected_.assign(n, 0);
    selectedCount_ = 0;
    anchorPos_ = kNone;
    focused_ = kNone;
    recountScope();
    rebuildHistogram();
    rebuildVisible();
}

bool DefectBrowser::passesSubtype(std::size_t element) const noexcept {
    return inScope(element) && (subtypeFilter_ < 0 || result_->elements[element].subtype == subtypeFilter_);
}

void DefectBrowser::setScopeMask(std::vector<std::uint8_t> mask) {
    if (mask == scopeMask_) {
        return;
    }
    revision_ = nextRevision();
    scopeMask_ = std::move(mask);
    recountScope();
    binFilter_.reset();
    rebuildHistogram();
    rebuildVisible();
    pruneHidden();
}

void DefectBrowser::recountScope() noexcept {
    subtypeCounts_.fill(0);
    scopedCount_ = 0;
    if (result_ == nullptr) {
        return;
    }
    for (std::size_t i = 0; i < result_->size(); ++i) {
        if (!inScope(i)) {
            continue;
        }
        ++scopedCount_;
        const int subtype = result_->elements[i].subtype;
        if (subtype >= 0 && subtype < static_cast<int>(subtypeCounts_.size())) {
            ++subtypeCounts_[static_cast<std::size_t>(subtype)];
        }
    }
}

std::size_t DefectBrowser::binOfElement(std::size_t element) const noexcept {
    return histogram_.binOf(result_->elements[element].metric);
}

void DefectBrowser::rebuildHistogram() {
    histogram_ = MetricHistogram{};
    if (result_ == nullptr) {
        return;
    }
    const CategoryInfo& info = categoryInfo(result_->category);
    if (info.metricName == nullptr) {
        return;
    }
    std::vector<double> values;
    values.reserve(result_->size());
    for (std::size_t i = 0; i < result_->size(); ++i) {
        if (passesSubtype(i)) {
            values.push_back(result_->elements[i].metric);
        }
    }
    if (!values.empty()) {
        histogram_ = buildHistogram(values, settings::kHistogramBins, info.metricLogScale, info.metricLowIsWorst);
    }
}

void DefectBrowser::rebuildVisible() {
    visibleRows_.clear();
    const std::size_t n = result_ != nullptr ? result_->size() : 0;
    visiblePos_.assign(n, kNone);
    if (n == 0) {
        return;
    }
    visibleRows_.reserve(n);
    for (std::size_t i = 0; i < n; ++i) {
        if (passesSubtype(i) && (!binFilter_ || binFilter_->contains(binOfElement(i)))) {
            visibleRows_.push_back(i);
        }
    }
    if (sortColumn_ >= 0 && sortColumn_ < 3) {
        const auto col = static_cast<std::size_t>(sortColumn_);
        std::stable_sort(visibleRows_.begin(), visibleRows_.end(), [&](std::size_t a, std::size_t b) {
            const double va = result_->elements[a].columns[col];
            const double vb = result_->elements[b].columns[col];
            return sortDescending_ ? va > vb : va < vb;
        });
    } else if (sortColumn_ == 3) {
        // Column 3 = element index (face / vertex / shell number)
        std::stable_sort(visibleRows_.begin(), visibleRows_.end(), [&](std::size_t a, std::size_t b) {
            const Eigen::Index ia = result_->elements[a].index;
            const Eigen::Index ib = result_->elements[b].index;
            return sortDescending_ ? ia > ib : ia < ib;
        });
    } else if (sortDescending_) {
        std::reverse(visibleRows_.begin(), visibleRows_.end());
    }
    for (std::size_t pos = 0; pos < visibleRows_.size(); ++pos) {
        visiblePos_[visibleRows_[pos]] = pos;
    }
}

void DefectBrowser::pruneHidden() {
    for (std::size_t i = 0; i < selected_.size(); ++i) {
        if (selected_[i] != 0 && visiblePos_[i] == kNone) {
            setSelected(i, false);
        }
    }
    if (focused_ != kNone && (focused_ >= visiblePos_.size() || visiblePos_[focused_] == kNone)) {
        focused_ = kNone;
    }
    anchorPos_ = kNone;
}

void DefectBrowser::setSubtypeFilter(int subtype) {
    revision_ = nextRevision();
    if (subtype == subtypeFilter_ || result_ == nullptr) {
        return;
    }
    subtypeFilter_ = subtype;
    binFilter_.reset();
    rebuildHistogram();
    rebuildVisible();
    pruneHidden();
}

void DefectBrowser::selectBin(std::size_t bin, bool extend) {
    revision_ = nextRevision();
    if (histogram_.empty() || bin >= histogram_.numBins()) {
        return;
    }
    if (extend && binFilter_) {
        binFilter_ = BinRange{std::min(binFilter_->first, bin), std::max(binFilter_->last, bin)};
    } else {
        binFilter_ = BinRange{bin, bin};
    }
    rebuildVisible();
    pruneHidden();
}

void DefectBrowser::stepBin(int direction) {
    revision_ = nextRevision();
    if (histogram_.empty()) {
        return;
    }
    const auto n = static_cast<long>(histogram_.numBins());
    long bin = binFilter_ ? static_cast<long>(direction < 0 ? binFilter_->first : binFilter_->last) + direction
                          : (direction < 0 ? n - 1 : 0);
    while (bin >= 0 && bin < n && histogram_.counts[static_cast<std::size_t>(bin)] == 0) {
        bin += direction < 0 ? -1 : 1;
    }
    if (bin >= 0 && bin < n) {
        selectBin(static_cast<std::size_t>(bin), false);
    }
}

void DefectBrowser::clearBinFilter() {
    revision_ = nextRevision();
    if (!binFilter_) {
        return;
    }
    binFilter_.reset();
    rebuildVisible();
    pruneHidden();
}

std::size_t DefectBrowser::countInBins(const BinRange& range) const noexcept {
    std::size_t count = 0;
    for (std::size_t b = range.first; b <= range.last && b < histogram_.numBins(); ++b) {
        count += histogram_.counts[b];
    }
    return count;
}

bool DefectBrowser::isVisible(std::size_t element) const noexcept {
    return element < visiblePos_.size() && visiblePos_[element] != kNone;
}

std::size_t DefectBrowser::visiblePositionOf(std::size_t element) const noexcept {
    return element < visiblePos_.size() ? visiblePos_[element] : kNone;
}

void DefectBrowser::setSort(int column, bool descending) {
    revision_ = nextRevision();
    if (column == sortColumn_ && descending == sortDescending_) {
        return;
    }
    sortColumn_ = column;
    sortDescending_ = descending;
    rebuildVisible();
    anchorPos_ = kNone;
}

void DefectBrowser::setSelected(std::size_t element, bool selected) noexcept {
    const bool was = selected_[element] != 0;
    if (was != selected) {
        selected_[element] = selected ? 1 : 0;
        selectedCount_ = selected ? selectedCount_ + 1 : selectedCount_ - 1;
    }
}

void DefectBrowser::clickRow(std::size_t pos, bool toggle, bool range) {
    revision_ = nextRevision();
    if (pos >= visibleRows_.size()) {
        return;
    }
    const std::size_t element = visibleRows_[pos];
    if (range && anchorPos_ != kNone && anchorPos_ < visibleRows_.size()) {
        if (!toggle) {
            std::fill(selected_.begin(), selected_.end(), std::uint8_t{0});
            selectedCount_ = 0;
        }
        for (std::size_t p = std::min(anchorPos_, pos); p <= std::max(anchorPos_, pos); ++p) {
            setSelected(visibleRows_[p], true);
        }
        focused_ = element;
        return;
    }
    if (toggle) {
        setSelected(element, !isSelected(element));
        if (isSelected(element)) {
            focused_ = element;
        } else if (focused_ == element) {
            focused_ = kNone;
        }
    } else {
        std::fill(selected_.begin(), selected_.end(), std::uint8_t{0});
        selectedCount_ = 0;
        setSelected(element, true);
        focused_ = element;
    }
    anchorPos_ = pos;
}

void DefectBrowser::selectAllVisible() {
    revision_ = nextRevision();
    for (const std::size_t element : visibleRows_) {
        setSelected(element, true);
    }
}

void DefectBrowser::selectNone() {
    revision_ = nextRevision();
    std::fill(selected_.begin(), selected_.end(), std::uint8_t{0});
    selectedCount_ = 0;
    focused_ = kNone;
    anchorPos_ = kNone;
}

void DefectBrowser::invertSelection() {
    revision_ = nextRevision();
    for (const std::size_t element : visibleRows_) {
        setSelected(element, !isSelected(element));
    }
    if (focused_ != kNone && !isSelected(focused_)) {
        focused_ = kNone;
    }
}

bool DefectBrowser::isSelected(std::size_t element) const noexcept {
    return element < selected_.size() && selected_[element] != 0;
}

std::vector<std::size_t> DefectBrowser::selectedElements() const {
    std::vector<std::size_t> out;
    out.reserve(selectedCount_);
    for (const std::size_t element : visibleRows_) {
        if (selected_[element] != 0) {
            out.push_back(element);
        }
    }
    return out;
}

void DefectBrowser::focusElement(std::size_t element) {
    revision_ = nextRevision();
    if (!isVisible(element)) {
        return;
    }
    std::fill(selected_.begin(), selected_.end(), std::uint8_t{0});
    selectedCount_ = 0;
    setSelected(element, true);
    focused_ = element;
    anchorPos_ = visiblePos_[element];
}

bool DefectBrowser::focusStep(int direction) {
    revision_ = nextRevision();
    if (visibleRows_.empty()) {
        return false;
    }
    const std::size_t n = visibleRows_.size();
    std::size_t pos = 0;
    if (focused_ != kNone && isVisible(focused_)) {
        const std::size_t current = visiblePos_[focused_];
        pos = direction < 0 ? (current + n - 1) % n : (current + 1) % n;
    } else if (direction < 0) {
        pos = n - 1;
    }
    focusElement(visibleRows_[pos]);
    return true;
}

std::size_t DefectBrowser::findVisibleElementWithFace(Eigen::Index face) const noexcept {
    if (result_ == nullptr) {
        return kNone;
    }
    for (const std::size_t element : visibleRows_) {
        for (const Eigen::Index f : result_->facesOf(element)) {
            if (f == face) {
                return element;
            }
        }
    }
    return kNone;
}

} // namespace ogun
