#include "app/AppController.hpp"
#include "core/Constants.hpp"
#include "core/Settings.hpp"
#include "diagnostics/ShellNesting.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <set>
#include <system_error>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace ogun {

const char* previewDisplayModeLabel(PreviewDisplayMode mode) noexcept {
    switch (mode) {
    case PreviewDisplayMode::TranslucentBase:
        return "Translucent Base";
    case PreviewDisplayMode::TranslucentPatch:
        return "Translucent Patch";
    case PreviewDisplayMode::CutoutOriginalEdges:
        return "Cutout + Original Edges";
    }
    return "Unknown";
}

const char* healthStatusLabel(HealthStatus status) noexcept {
    switch (status) {
    case HealthStatus::NotRun:
        return "Not checked yet";
    case HealthStatus::Running:
        return "Checking...";
    case HealthStatus::Ok:
        return "OK";
    case HealthStatus::Defects:
        return "Problems found";
    case HealthStatus::Stale:
        return "Stale: the mesh changed since this check ran";
    }
    return "Unknown";
}

const char* backFaceModeLabel(BackFaceMode mode) noexcept {
    switch (mode) {
    case BackFaceMode::Identical:
        return "Identical";
    case BackFaceMode::Darker:
        return "Darker";
    case BackFaceMode::Custom:
        return "Custom";
    case BackFaceMode::Cull:
        return "Hidden (cull)";
    }
    return "Unknown";
}

std::size_t CategoryHealth::displayCount(DefectCategory category) const noexcept {
    if (!result) {
        return 0;
    }
    return categoryInfo(category).infoOnly ? result->infoCount : result->size();
}

AppController::AppController()
    : thresholds_(ThinFaceThresholds::defaultsForUnit(Unit::Millimeter)) {
    mesh_.fileUnit = Unit::Millimeter;
    for (const DefectCategory category : kAllDefectCategories) {
        autoCheckDefaults_[categoryIndex(category)] = categoryInfo(category).autoDefault;
    }
    resetHealthReport();
}

std::filesystem::path AppController::resolveApplicationIconPath() noexcept {
    const std::array<std::filesystem::path, 4> candidates = {
        std::filesystem::path("assets/icons/ogun.png"),
        std::filesystem::path("assets/icons/ogun.jpeg"),
#ifdef OGUN_SOURCE_ICON_PNG_PATH
        std::filesystem::path(OGUN_SOURCE_ICON_PNG_PATH),
#else
        std::filesystem::path{},
#endif
#ifdef OGUN_SOURCE_ICON_JPEG_PATH
        std::filesystem::path(OGUN_SOURCE_ICON_JPEG_PATH),
#else
        std::filesystem::path{},
#endif
    };

    std::error_code ec;
    for (const auto& candidate : candidates) {
        if (!candidate.empty() && std::filesystem::exists(candidate, ec) && !ec) {
            return candidate;
        }
    }
    return {};
}

std::filesystem::path AppController::resolveExamplePath(std::string_view fileName) {
    const std::array<std::filesystem::path, 2> candidates = {
        std::filesystem::path("examples") / fileName,
#ifdef OGUN_SOURCE_EXAMPLES_DIR
        std::filesystem::path(OGUN_SOURCE_EXAMPLES_DIR) / fileName,
#else
        std::filesystem::path{},
#endif
    };
    std::error_code ec;
    for (const auto& candidate : candidates) {
        if (!candidate.empty() && std::filesystem::exists(candidate, ec) && !ec) {
            return candidate;
        }
    }
    return {};
}

bool AppController::loadStlFile(const std::filesystem::path& filepath) {
    Mesh loadedMesh;
    std::string err;
    if (!ogun::loadStl(filepath, loadedMesh, mesh_.fileUnit, toleranceConfig_, &err)) {
        lastErrorMessage_ = std::move(err);
        postStatusMessage(lastErrorMessage_, true);
        return false;
    }

    lastErrorMessage_.clear();
    const std::string label = filepath.filename().empty()
        ? filepath.string()
        : filepath.filename().string();
    setMesh(std::move(loadedMesh), label);
    currentFilePath_ = filepath;
    postStatusMessage("Loaded " + label);
    return true;
}

void AppController::loadSampleDefectiveMesh() {
    lastErrorMessage_.clear();
    setMesh(SampleMeshes::createSampleDefectiveMesh(mesh_.fileUnit), "Sample Defective Mesh");
}

bool AppController::loadSampleFile(std::string_view fileName) {
    const std::filesystem::path path = resolveExamplePath(fileName);
    if (path.empty()) {
        lastErrorMessage_ = "Sample file examples/" + std::string(fileName) + " was not found.";
        postStatusMessage(lastErrorMessage_, true);
        return false;
    }
    if (!loadStlFile(path)) {
        return false;
    }
    currentFilePath_.clear(); // samples are never overwritten by Save
    return true;
}

void AppController::setMesh(Mesh mesh, std::string sourceLabel) {
    loadedSourceLabel_ = std::move(sourceLabel);
    currentFilePath_.clear();
    modified_ = false;
    undoStack_.clear();
    lastRepairStatusMessage_.clear();
    checkRunner_.cancel();
    selectedCategory_.reset();
    activeTool_ = ToolSlot::None;
    shells_.clear();
    replaceMeshPreservingHistory(std::move(mesh));
    resetHealthReport();
    syncBrowserResult();
    if (checkMeshAfterLoad_ && !mesh_.empty()) {
        checkMesh();
    }
}

void AppController::replaceMeshPreservingHistory(Mesh mesh, std::optional<ShellBrowser> shells) {
    mesh_ = std::move(mesh);
    ++meshGeneration_;
    rebuildCgalMesh();
    if (shells) {
        shells_ = std::move(*shells);
    } else {
        shells_.rebuild(mesh_);
    }
    for (std::vector<std::uint8_t>& mask : scopeMasks_) {
        mask.clear();
    }
    scopedCounts_.fill(0);
    shellHealth_.clear();
    pickedFaceIndex_ = -1;
    browser_.setResult(nullptr);
    selectedFaceIndex_ = -1;
    lastFocusedFaceIndex_ = -1;
    lastFocusedNormalSign_ = 0;
    resetSingleFaceRepairState();
    orbitPivot_ = computeSelectionCentroid();
}

void AppController::rebuildCgalMesh() {
    cgalMeshValid_ = toCgalSurfaceMesh(mesh_, cgalMesh_);
}

bool AppController::saveStlFile(const std::filesystem::path& filepath, StlFormat format) {
    std::string err;
    if (!ogun::saveStl(filepath, mesh_, format, &err)) {
        lastErrorMessage_ = err.empty() ? "Failed to save STL file." : std::move(err);
        postStatusMessage(lastErrorMessage_, true);
        return false;
    }
    lastErrorMessage_.clear();
    currentFilePath_ = filepath;
    modified_ = false;
    postStatusMessage("Saved " + filepath.filename().string());
    return true;
}

void AppController::postStatusMessage(std::string message, bool isError) {
    statusMessage_ = std::move(message);
    statusMessageIsError_ = isError;
    ++statusMessageSerial_;
}

void AppController::setFileUnit(Unit newUnit) noexcept {
    const Unit oldUnit = mesh_.fileUnit;
    if (oldUnit == newUnit) {
        return;
    }

    ogun::setFileUnit(mesh_, newUnit);
    thresholds_ = thresholds_.convertedToUnit(oldUnit, newUnit);
    // Same numbers, new unit: lengths in results stay valid, but the thin thresholds changed meaning
    onCheckParamsChanged(DefectCategory::ThinFaces);
}

void AppController::scaleMeshCoordinatesToUnit(Unit targetUnit) {
    const Unit oldUnit = mesh_.fileUnit;
    if (oldUnit == targetUnit || mesh_.empty()) {
        return;
    }

    pushUndoEntry(
        "Scale Mesh Coordinates (" + std::string(unitSymbol(oldUnit)) + " -> " +
        std::string(unitSymbol(targetUnit)) + ")");
    Mesh scaled = mesh_;
    ogun::scaleMeshToUnit(scaled, targetUnit);
    thresholds_ = thresholds_.convertedToUnit(oldUnit, targetUnit);
    finishMeshEdit(std::move(scaled));
    postStatusMessage("Scaled coordinates from " + std::string(unitSymbol(oldUnit)) + " to " +
                      std::string(unitSymbol(targetUnit)) + ".");
}

// -----------------------------------------------------------------------------
// Mesh Health
// -----------------------------------------------------------------------------

CheckParams AppController::checkParams() const noexcept {
    CheckParams params;
    params.thinThresholds = thresholds_;
    params.degenerateAreaEpsilon = degenerateAreaEpsilon_;
    params.intersectionTolerance = intersectionTolerance_;
    return params;
}

void AppController::resetHealthReport() {
    for (const DefectCategory category : kAllDefectCategories) {
        CategoryHealth& row = health_[category];
        row.status = HealthStatus::NotRun;
        row.autoCheck = autoCheckDefaults_[categoryIndex(category)];
        row.result.reset();
    }
}

void AppController::setThresholds(const ThinFaceThresholds& thresholds) {
    const bool changed = thresholds.minAltitude != thresholds_.minAltitude ||
                         thresholds.minArea != thresholds_.minArea ||
                         thresholds.maxAspectRatio != thresholds_.maxAspectRatio;
    thresholds_ = thresholds;
    if (changed) {
        onCheckParamsChanged(DefectCategory::ThinFaces);
    }
}

void AppController::setDegenerateAreaEpsilon(double epsilon) {
    if (!std::isfinite(epsilon) || epsilon < 0.0 || epsilon == degenerateAreaEpsilon_) {
        return;
    }
    degenerateAreaEpsilon_ = epsilon;
    onCheckParamsChanged(DefectCategory::Degenerate);
}

void AppController::setIntersectionTolerance(double tolerance) {
    if (!std::isfinite(tolerance) || tolerance < 0.0 || tolerance == intersectionTolerance_) {
        return;
    }
    intersectionTolerance_ = tolerance;
    onCheckParamsChanged(DefectCategory::Intersections);
}

void AppController::markRunning(DefectCategory category) noexcept {
    health_[category].status = HealthStatus::Running;
}

void AppController::checkMesh() {
    if (mesh_.empty()) {
        return;
    }
    for (const DefectCategory category : kAllDefectCategories) {
        markRunning(category);
    }
    syncBrowserResult();
    launchQueuedChecks();
}

void AppController::recheckCategory(DefectCategory category) {
    if (mesh_.empty()) {
        return;
    }
    markRunning(category);
    syncBrowserResult();
    launchQueuedChecks();
}

void AppController::setAutoCheck(DefectCategory category, bool enabled) {
    CategoryHealth& row = health_[category];
    row.autoCheck = enabled;
    if (enabled && row.status == HealthStatus::Stale) {
        recheckCategory(category);
    }
}

void AppController::onCheckParamsChanged(DefectCategory category) {
    CategoryHealth& row = health_[category];
    if (row.status == HealthStatus::NotRun || mesh_.empty()) {
        return;
    }
    if (row.autoCheck || row.status == HealthStatus::Running) {
        recheckCategory(category);
    } else {
        row.status = HealthStatus::Stale;
        syncBrowserResult();
    }
}

void AppController::onMeshEdited() {
    checkRunner_.cancel();
    for (const DefectCategory category : kAllDefectCategories) {
        CategoryHealth& row = health_[category];
        if (row.status == HealthStatus::NotRun) {
            continue;
        }
        if (row.autoCheck || row.status == HealthStatus::Running) {
            row.status = HealthStatus::Running;
        } else {
            row.status = HealthStatus::Stale;
        }
    }
    syncBrowserResult();
    launchQueuedChecks();
}

void AppController::cancelChecks() {
    checkRunner_.cancel();
    for (CategoryHealth& row : health_.rows) {
        if (row.status == HealthStatus::Running) {
            row.status = row.result ? HealthStatus::Stale : HealthStatus::NotRun;
        }
    }
    syncBrowserResult();
}

void AppController::setBackgroundChecks(bool enabled) {
    if (enabled == backgroundChecks_) {
        return; // re-enabling would restart a running job
    }
    if (!enabled) {
        checkRunner_.cancel();
    }
    backgroundChecks_ = enabled;
    launchQueuedChecks();
}

void AppController::launchQueuedChecks() {
    std::vector<DefectCategory> queued;
    for (const DefectCategory category : kAllDefectCategories) {
        if (health_[category].status == HealthStatus::Running) {
            queued.push_back(category);
        }
    }
    if (queued.empty() || mesh_.empty()) {
        return;
    }
    if (backgroundChecks_) {
        checkRunner_.start(CheckRequest{meshGeneration_, std::make_shared<const Mesh>(mesh_), queued, checkParams()});
        return;
    }
    const CheckParams params = checkParams();
    for (const DefectCategory category : queued) {
        CategoryResult result = runCategoryCheck(category, mesh_, params);
        result.meshGeneration = meshGeneration_;
        applyCheckResult(std::move(result));
    }
}

void AppController::applyCheckResult(CategoryResult result) {
    CategoryHealth& row = health_[result.category];
    if (result.cancelled || result.meshGeneration != meshGeneration_ || row.status != HealthStatus::Running) {
        return;
    }
    const DefectCategory category = result.category;
    const bool infoOnly = categoryInfo(category).infoOnly;
    row.status = (infoOnly || result.empty()) ? HealthStatus::Ok : HealthStatus::Defects;
    row.result = std::move(result);
    computeScope(category);
    refreshShellHealth();
    if (category == DefectCategory::Intersections && shells_.multiShell()) {
        // A shell inside another that it does not cross is nested (e.g. a hidden internal part)
        std::set<std::pair<int, int>> crossing;
        for (std::size_t s = 0; s < shellHealth_.size(); ++s) {
            for (const int other : shellHealth_[s].crosses) {
                const auto own = static_cast<int>(s);
                crossing.insert({std::min(own, other), std::max(own, other)});
            }
        }
        std::vector<std::vector<Eigen::Index>> shellFaces;
        shellFaces.reserve(shells_.size());
        for (const ShellInfo& shell : shells_.shells()) {
            shellFaces.push_back(shell.faces);
        }
        shells_.setNestedIn(findNestedShells(mesh_, shellFaces, crossing));
    }
    if (selectedCategory_ == category) {
        syncBrowserResult();
    }
}

bool AppController::pollBackgroundChecks() {
    const bool wasBusy = checkRunner_.busy();
    std::vector<CategoryResult> finished = checkRunner_.takeFinished();
    const bool changed = !finished.empty();
    for (CategoryResult& result : finished) {
        applyCheckResult(std::move(result));
    }
    // A job that ended without covering every queued row (e.g. restarted mid-way) is relaunched
    if (backgroundChecks_ && !wasBusy && !checkRunner_.busy()) {
        for (const CategoryHealth& row : health_.rows) {
            if (row.status == HealthStatus::Running) {
                launchQueuedChecks();
                break;
            }
        }
    }
    return changed;
}

void AppController::waitForChecks() {
    checkRunner_.waitIdle();
    for (CategoryResult& result : checkRunner_.takeFinished()) {
        applyCheckResult(std::move(result));
    }
}

bool AppController::checksRunning() const noexcept {
    for (const CategoryHealth& row : health_.rows) {
        if (row.status == HealthStatus::Running) {
            return true;
        }
    }
    return false;
}

std::size_t AppController::totalProblemCount() const noexcept {
    std::size_t total = 0;
    for (const DefectCategory category : kAllDefectCategories) {
        if (health_[category].status == HealthStatus::Defects && !categoryInfo(category).infoOnly) {
            total += rowCount(category);
        }
    }
    return total;
}

std::size_t AppController::rowCount(DefectCategory category) const noexcept {
    const CategoryHealth& row = health_[category];
    if (categoryInfo(category).infoOnly || currentResult(category) == nullptr) {
        return row.displayCount(category);
    }
    return scopedCounts_[categoryIndex(category)];
}

HealthStatus AppController::rowStatus(DefectCategory category) const noexcept {
    const HealthStatus status = health_[category].status;
    return status == HealthStatus::Defects && rowCount(category) == 0 ? HealthStatus::Ok : status;
}

bool AppController::elementInScope(DefectCategory category, std::size_t element) const noexcept {
    const std::vector<std::uint8_t>& mask = scopeMasks_[categoryIndex(category)];
    return mask.empty() || (element < mask.size() && mask[element] != 0);
}

const CategoryResult* AppController::currentResult(DefectCategory category) const noexcept {
    const CategoryHealth& row = health_[category];
    return row.result && row.result->meshGeneration == meshGeneration_ ? &*row.result : nullptr;
}

void AppController::computeScope(DefectCategory category) {
    std::vector<std::uint8_t>& mask = scopeMasks_[categoryIndex(category)];
    mask.clear();
    const CategoryResult* result = currentResult(category);
    if (result == nullptr) {
        scopedCounts_[categoryIndex(category)] = 0;
        return;
    }
    if (!shells_.scoped() || categoryInfo(category).infoOnly) {
        scopedCounts_[categoryIndex(category)] = result->size();
        return;
    }
    const auto inScope = [this](Eigen::Index f) { return shells_.inScope(shells_.shellOfFace(f)); };
    mask.assign(result->size(), 0);
    std::size_t count = 0;
    for (std::size_t e = 0; e < result->size(); ++e) {
        const auto faces = result->facesOf(e);
        bool in = false;
        if (category == DefectCategory::Intersections && !faces.empty()) {
            // FB-22: a crossing counts when its own shell and the shell it crosses are both in scope
            const auto related = result->relatedOf(e);
            in = inScope(faces.front()) &&
                 (related.empty() || std::any_of(related.begin(), related.end(), inScope));
        } else {
            in = faces.empty() || std::any_of(faces.begin(), faces.end(), inScope);
        }
        mask[e] = in ? 1 : 0;
        count += in ? 1 : 0;
    }
    scopedCounts_[categoryIndex(category)] = count;
}

void AppController::onShellsChanged() {
    for (const DefectCategory category : kAllDefectCategories) {
        computeScope(category);
    }
    applyScopeToBrowser();
    orbitPivot_ = computeSelectionCentroid();
}

void AppController::applyScopeToBrowser() {
    if (browser_.result() == nullptr) {
        return;
    }
    const std::size_t focused = browser_.focusedElement();
    browser_.setScopeMask(scopeMasks_[categoryIndex(browser_.result()->category)]);
    if (browser_.focusedElement() != focused) {
        onFocusChanged();
    }
}

void AppController::refreshShellHealth() {
    shellHealth_.clear();
    if (!shells_.multiShell()) {
        return;
    }
    shellHealth_.resize(shells_.size());
    for (const DefectCategory category : kAllDefectCategories) {
        const CategoryResult* result = currentResult(category);
        if (result == nullptr || categoryInfo(category).infoOnly) {
            continue;
        }
        for (std::size_t e = 0; e < result->size(); ++e) {
            const auto faces = result->facesOf(e);
            const int own = faces.empty() ? -1 : shells_.shellOfFace(faces.front());
            if (own < 0) {
                continue;
            }
            ShellHealth& health = shellHealth_[static_cast<std::size_t>(own)];
            const auto related = result->relatedOf(e);
            const bool betweenOnly = !related.empty() && std::none_of(related.begin(), related.end(), [&](Eigen::Index p) {
                return shells_.shellOfFace(p) == own;
            });
            if (betweenOnly) {
                ++health.between;
            } else {
                ++health.counts[categoryIndex(category)];
                ++health.total;
            }
            for (const Eigen::Index p : related) {
                const int other = shells_.shellOfFace(p);
                if (other >= 0 && other != own) {
                    health.crosses.push_back(other);
                }
            }
        }
    }
    for (ShellHealth& health : shellHealth_) {
        std::sort(health.crosses.begin(), health.crosses.end());
        health.crosses.erase(std::unique(health.crosses.begin(), health.crosses.end()), health.crosses.end());
    }
}

bool AppController::shellHealthKnown() const noexcept {
    for (const DefectCategory category : kAllDefectCategories) {
        if (!categoryInfo(category).infoOnly && currentResult(category) != nullptr) {
            return true;
        }
    }
    return false;
}

WatertightState AppController::watertightState() const noexcept {
    const CategoryHealth& open = health_[DefectCategory::OpenEdges];
    const CategoryHealth& nonManifold = health_[DefectCategory::NonManifoldEdges];
    if (open.status == HealthStatus::Defects || nonManifold.status == HealthStatus::Defects) {
        return WatertightState::NotWatertight;
    }
    if (open.status == HealthStatus::Ok && nonManifold.status == HealthStatus::Ok) {
        return WatertightState::Watertight;
    }
    return WatertightState::Unknown;
}

// -----------------------------------------------------------------------------
// Selected Row, Filters, Selection & Focus
// -----------------------------------------------------------------------------

void AppController::syncBrowserResult() {
    const CategoryResult* wanted = nullptr;
    if (selectedCategory_) {
        const CategoryHealth& row = health_[*selectedCategory_];
        if (row.result && (row.status == HealthStatus::Ok || row.status == HealthStatus::Defects)) {
            wanted = &*row.result;
        }
    }
    if (browser_.result() != wanted) {
        browser_.setResult(wanted);
        applyScopeToBrowser();
        onFocusChanged();
    }
}

void AppController::selectCategory(std::optional<DefectCategory> category) {
    pickedFaceIndex_ = -1;
    selectedCategory_ = category;
    activeTool_ = category ? ToolSlot::DefectCategory : ToolSlot::None;
    browser_.setResult(nullptr);
    syncBrowserResult();
    onFocusChanged();
}

void AppController::setSubtypeFilter(int subtype) {
    browser_.setSubtypeFilter(subtype);
    onFocusChanged();
}

void AppController::selectHistogramBin(std::size_t bin, bool extend) {
    browser_.selectBin(bin, extend);
    browser_.selectAllVisible();
    onFocusChanged();
}

void AppController::stepHistogramBin(int direction) {
    browser_.stepBin(direction);
    browser_.selectAllVisible();
    onFocusChanged();
}

void AppController::clearHistogramBinFilter() {
    browser_.clearBinFilter();
    onFocusChanged();
}

void AppController::setDefectSort(int column, bool descending) {
    browser_.setSort(column, descending);
}

void AppController::clickDefectRow(std::size_t visiblePosition, bool toggle, bool range) {
    browser_.clickRow(visiblePosition, toggle, range);
    onFocusChanged();
}

void AppController::selectAllVisibleDefects() {
    browser_.selectAllVisible();
    onFocusChanged();
}

void AppController::selectNoDefects() {
    browser_.selectNone();
    onFocusChanged();
}

void AppController::invertDefectSelection() {
    browser_.invertSelection();
    onFocusChanged();
}

void AppController::focusDefect(std::size_t element) {
    browser_.focusElement(element);
    onFocusChanged();
}

std::span<const Eigen::Index> AppController::focusedRelatedFaces() const noexcept {
    const CategoryResult* result = browser_.result();
    if (result == nullptr || !browser_.hasFocus()) {
        return {};
    }
    return result->relatedOf(browser_.focusedElement());
}

bool AppController::focusRelatedFace(Eigen::Index faceIndex) {
    if (browser_.result() == nullptr) {
        return false;
    }
    const std::size_t element = browser_.findVisibleElementWithFace(faceIndex);
    if (element == DefectBrowser::kNone) {
        postStatusMessage("Face #" + std::to_string(faceIndex) + " is hidden by the current filter.");
        return false;
    }
    focusDefect(element);
    return true;
}

bool AppController::focusNextDefect(int direction) {
    if (!browser_.focusStep(direction)) {
        return false;
    }
    onFocusChanged();
    return true;
}

PickOutcome AppController::pickFace(Eigen::Index faceIndex) {
    if (mesh_.empty() || faceIndex < 0 || faceIndex >= mesh_.numFaces()) {
        return PickOutcome::None;
    }

    if (selectedCategory_ && browsableResult()) {
        const std::size_t element = browser_.findVisibleElementWithFace(faceIndex);
        if (element != DefectBrowser::kNone) {
            focusDefect(element);
            return PickOutcome::FocusedDefect;
        }
        postStatusMessage("Face #" + std::to_string(faceIndex) + " is not one of the " +
                          categoryInfo(*selectedCategory_).name + " shown.");
        return PickOutcome::NotInSelectedRow;
    }

    // No row selected: open the first checked row containing the face (thin faces first: repairable)
    std::array<DefectCategory, kDefectCategoryCount> order = kAllDefectCategories;
    std::stable_partition(order.begin(), order.end(), [](DefectCategory c) { return c == DefectCategory::ThinFaces; });
    for (const DefectCategory category : order) {
        const CategoryHealth& row = health_[category];
        if (categoryInfo(category).infoOnly || row.status != HealthStatus::Defects || !row.result) {
            continue;
        }
        for (std::size_t element = 0; element < row.result->size(); ++element) {
            if (!elementInScope(category, element)) {
                continue;
            }
            for (const Eigen::Index f : row.result->facesOf(element)) {
                if (f == faceIndex) {
                    selectCategory(category);
                    focusDefect(element);
                    return PickOutcome::SelectedRow;
                }
            }
        }
    }

    selectCategory(std::nullopt);
    pickedFaceIndex_ = faceIndex;
    lastFocusedFaceIndex_ = -1;
    lastFocusedNormalSign_ = 0;
    orbitPivot_ = computeSelectionCentroid();
    postStatusMessage("Face #" + std::to_string(faceIndex) + ": no defects found on this face.");
    return PickOutcome::PickedFace;
}

bool AppController::escape() {
    if (browser_.selectedCount() > 0 || browser_.hasFocus()) {
        selectNoDefects();
        return true;
    }
    if (pickedFaceIndex_ >= 0 || selectedCategory_ || activeTool_ != ToolSlot::None) {
        clearSelection();
        return true;
    }
    if (shells_.selectedCount() > 0) {
        shells_.selectNone();
        onShellsChanged();
        return true;
    }
    return false;
}

void AppController::clearSelection() noexcept {
    pickedFaceIndex_ = -1;
    selectedCategory_.reset();
    activeTool_ = ToolSlot::None;
    browser_.setResult(nullptr);
    onFocusChanged();
}

void AppController::openUnitsAndScale() {
    clearSelection();
    activeTool_ = ToolSlot::UnitsAndScale;
}

void AppController::closeTool() {
    clearSelection();
}

Eigen::Index AppController::focusedSingleFace() const noexcept {
    if (pickedFaceIndex_ >= 0) {
        return pickedFaceIndex_;
    }
    const CategoryResult* result = browser_.result();
    if (result != nullptr && browser_.hasFocus()) {
        const auto faces = result->facesOf(browser_.focusedElement());
        if (faces.size() == 1 && categoryInfo(result->category).kind == ElementKind::Face) {
            return faces.front();
        }
    }
    return -1;
}

Eigen::Index AppController::focusedFaceIndex() const noexcept {
    return focusedSingleFace();
}

void AppController::onFocusChanged() {
    const Eigen::Index face = focusedSingleFace();
    const bool thinFocus = face >= 0 && pickedFaceIndex_ < 0 && browser_.result() != nullptr &&
                           browser_.result()->category == DefectCategory::ThinFaces;
    if (face != lastFocusedFaceIndex_) {
        lastFocusedFaceIndex_ = -1;
        lastFocusedNormalSign_ = 0;
    }
    if (thinFocus) {
        analyzeFocusedFace(face);
    } else {
        selectedFaceIndex_ = -1;
        resetSingleFaceRepairState();
    }
    orbitPivot_ = computeSelectionCentroid();
}

void AppController::analyzeFocusedFace(Eigen::Index faceIndex) {
    const bool newSelection = selectedFaceIndex_ != faceIndex;
    selectedFaceIndex_ = faceIndex;
    selectedFaceAnalysis_ = cgalMeshValid_
        ? SliverHealer::analyzeFace(
              cgalMesh_,
              CgalFaceIndex(static_cast<CgalSurfaceMesh::size_type>(faceIndex)),
              thresholds_)
        : SliverAnalysis{};
    if (newSelection) {
        selectedHealAction_ = selectedFaceAnalysis_.recommendedAction;
    }
    refreshRepairPreview();
}

std::vector<Eigen::Index> AppController::highlightedFaceIndices() const {
    std::vector<Eigen::Index> faces;
    if (pickedFaceIndex_ >= 0) {
        faces.push_back(pickedFaceIndex_);
        return faces;
    }
    const CategoryResult* result = browser_.result();
    if (result == nullptr) {
        return faces;
    }
    for (const std::size_t element : browser_.selectedElements()) {
        const auto ef = result->facesOf(element);
        faces.insert(faces.end(), ef.begin(), ef.end());
    }
    if (browser_.hasFocus() && !browser_.isSelected(browser_.focusedElement())) {
        const auto ef = result->facesOf(browser_.focusedElement());
        faces.insert(faces.end(), ef.begin(), ef.end());
    }
    // Intersections: also show the faces the focused face crosses
    const auto related = focusedRelatedFaces();
    faces.insert(faces.end(), related.begin(), related.end());
    std::sort(faces.begin(), faces.end());
    faces.erase(std::unique(faces.begin(), faces.end()), faces.end());
    return faces;
}

std::vector<Eigen::Index> AppController::activeFrameFaces() const {
    std::vector<Eigen::Index> faces = highlightedFaceIndices();
    const CategoryResult* result = browser_.result();
    // Selected shells are framed by their boxes (computeActiveSelectionOrMeshBounds)
    if (!faces.empty() || result == nullptr || shells_.selectedCount() > 0) {
        return faces;
    }
    for (const std::size_t element : browser_.visibleRows()) {
        const auto ef = result->facesOf(element);
        faces.insert(faces.end(), ef.begin(), ef.end());
    }
    return faces;
}

// -----------------------------------------------------------------------------
// Single-Face Repair Inspector
// -----------------------------------------------------------------------------

void AppController::resetSingleFaceRepairState() noexcept {
    selectedFaceAnalysis_ = SliverAnalysis{};
    selectedHealAction_ = HealAction::None;
    repairPreview_ = SingleFaceRepairPreview{};
}

void AppController::refreshRepairPreview() {
    if (selectedFaceIndex_ < 0 || !cgalMeshValid_) {
        repairPreview_ = SingleFaceRepairPreview{};
        return;
    }
    repairPreview_ = SliverHealer::previewSingleFaceRepair(
        cgalMesh_,
        CgalFaceIndex(static_cast<CgalSurfaceMesh::size_type>(selectedFaceIndex_)),
        selectedHealAction_,
        mesh_.fileUnit,
        thresholds_);
}

void AppController::setPreviewBaseOpacity(double opacity) noexcept {
    if (std::isfinite(opacity)) {
        previewBaseOpacity_ = std::clamp(opacity, settings::kMinPreviewOpacity, 1.0);
    }
}

void AppController::setPreviewPatchOpacity(double opacity) noexcept {
    if (std::isfinite(opacity)) {
        previewPatchOpacity_ = std::clamp(opacity, settings::kMinPreviewOpacity, 1.0);
    }
}

bool AppController::previewUsesCutout() const noexcept {
    return repairPreviewShown() &&
           !repairPreview_.sourceFaceIndices.empty() &&
           previewDisplayMode_ == PreviewDisplayMode::CutoutOriginalEdges;
}

HighlightOverlayGeometry AppController::buildRepairPreviewOriginalEdges() const {
    HighlightOverlayGeometry overlay;
    if (previewUsesCutout()) {
        appendFaceGeometry(repairPreview_.sourceFaceIndices, overlay);
    }
    return overlay;
}

BaseMeshDisplay AppController::buildBaseMeshDisplay() const {
    BaseMeshDisplay display;
    const bool cutPreview = previewUsesCutout();
    const std::vector<Eigen::Index> highlighted = highlightedFaceIndices();
    const bool shellsHidden = shells_.visibleCount() < shells_.size();
    if (mesh_.empty() || (highlighted.empty() && !cutPreview && !shellsHidden)) {
        return display;
    }

    std::vector<bool> hidden(static_cast<std::size_t>(mesh_.numFaces()), false);
    const auto hide = [&](const std::vector<Eigen::Index>& faces) {
        for (const Eigen::Index f : faces) {
            if (f >= 0 && f < mesh_.numFaces()) {
                hidden[static_cast<std::size_t>(f)] = true;
            }
        }
    };
    hide(highlighted);
    if (cutPreview) {
        hide(repairPreview_.sourceFaceIndices);
    }
    for (const ShellInfo& shell : shells_.shells()) {
        if (!shell.visible) {
            hide(shell.faces);
        }
    }

    display.faceIndices.reserve(hidden.size());
    for (Eigen::Index f = 0; f < mesh_.numFaces(); ++f) {
        if (!hidden[static_cast<std::size_t>(f)]) {
            display.faceIndices.push_back(f);
        }
    }
    display.F.resize(static_cast<Eigen::Index>(display.faceIndices.size()), 3);
    for (Eigen::Index row = 0; row < display.F.rows(); ++row) {
        display.F.row(row) = mesh_.F.row(display.faceIndices[static_cast<std::size_t>(row)]);
    }
    display.isCut = true;
    return display;
}

void AppController::setSelectedHealAction(HealAction action) {
    if (selectedFaceIndex_ < 0) {
        return;
    }
    selectedHealAction_ = action;
    refreshRepairPreview();
}

// -----------------------------------------------------------------------------
// Mesh edits (all undoable)
// -----------------------------------------------------------------------------

void AppController::finishMeshEdit(Mesh mesh) {
    replaceMeshPreservingHistory(std::move(mesh));
    modified_ = true;
    onMeshEdited();
}

bool AppController::commitSelectedFaceRepair() {
    if (selectedFaceIndex_ < 0 || selectedHealAction_ == HealAction::None) {
        lastRepairStatusMessage_ = "Select a face and a repair strategy first.";
        return false;
    }

    const Eigen::Index faceIndex = selectedFaceIndex_;
    const HealAction action = selectedHealAction_;
    const std::string faceLabel = "Face #" + std::to_string(faceIndex);
    RepairResults results = SliverHealer::healSingleFace(mesh_, faceIndex, action, thresholds_);
    if (!results.success) {
        lastRepairStatusMessage_ = faceLabel + ": " + results.statusMessage;
        postStatusMessage(lastRepairStatusMessage_, true);
        return false;
    }

    pushUndoEntry("Single-Face Repair (" + faceLabel + ")");
    finishMeshEdit(std::move(results.mesh));
    lastRepairStatusMessage_ = std::string(healActionLabel(action)) + " committed on " + faceLabel + " (" +
                               std::to_string(results.remainingThinFaces) + " thin faces remaining).";
    postStatusMessage(lastRepairStatusMessage_);
    return true;
}

bool AppController::fixAllThinFaces() {
    if (mesh_.empty()) {
        lastRepairStatusMessage_ = "No mesh loaded.";
        return false;
    }

    std::string label = "Fix All Thin / Sliver Faces";
    RepairResults results;
    if (shells_.scoped()) {
        // Only the shells in scope are healed; hidden or unselected shells are never edited
        CategoryResult checked;
        const CategoryResult* thin = currentResult(DefectCategory::ThinFaces);
        if (thin == nullptr) {
            checked = runCategoryCheck(DefectCategory::ThinFaces, mesh_, checkParams());
            thin = &checked;
        }
        std::vector<Eigen::Index> faces;
        for (std::size_t e = 0; e < thin->size(); ++e) {
            const Eigen::Index f = thin->elements[e].index;
            if (shells_.inScope(shells_.shellOfFace(f))) {
                faces.push_back(f);
            }
        }
        if (faces.empty()) {
            lastRepairStatusMessage_ = "No thin / sliver faces in the shells in scope.";
            postStatusMessage(lastRepairStatusMessage_);
            return false;
        }
        results = SliverHealer::healFaces(mesh_, faces, thresholds_);
        label += " (" + std::to_string(shells_.scopeCount()) + " shells)";
    } else {
        results = SliverHealer::healAllThinFaces(mesh_, thresholds_);
        if (results.initialThinFaces == 0) {
            lastRepairStatusMessage_ = "No thin / sliver faces to heal.";
            postStatusMessage(lastRepairStatusMessage_);
            return false;
        }
    }
    if (!results.success) {
        lastRepairStatusMessage_ = results.statusMessage;
        postStatusMessage(lastRepairStatusMessage_, true);
        return false;
    }

    pushUndoEntry(std::move(label));
    finishMeshEdit(std::move(results.mesh));
    lastRepairStatusMessage_ = std::move(results.statusMessage);
    postStatusMessage(lastRepairStatusMessage_);
    return true;
}

bool AppController::fixSelectedThinFaces() {
    const CategoryResult* result = browser_.result();
    if (result == nullptr || result->category != DefectCategory::ThinFaces || browser_.selectedCount() == 0) {
        lastRepairStatusMessage_ = "Select thin / sliver faces first.";
        return false;
    }
    std::vector<Eigen::Index> faces;
    faces.reserve(browser_.selectedCount());
    for (const std::size_t element : browser_.selectedElements()) {
        faces.push_back(result->elements[element].index);
    }

    RepairResults results = SliverHealer::healFaces(mesh_, faces, thresholds_);
    if (!results.success) {
        lastRepairStatusMessage_ = results.statusMessage;
        postStatusMessage(lastRepairStatusMessage_, true);
        return false;
    }
    pushUndoEntry("Fix Selected Thin Faces (" + std::to_string(faces.size()) + ")");
    finishMeshEdit(std::move(results.mesh));
    lastRepairStatusMessage_ = std::move(results.statusMessage);
    postStatusMessage(lastRepairStatusMessage_);
    return true;
}

void AppController::pushUndoEntry(std::string actionLabel) {
    undoStack_.push_back(UndoEntry{
        mesh_,
        thresholds_,
        health_,
        selectedCategory_,
        selectedFaceIndex_,
        selectedHealAction_,
        modified_,
        shells_,
        std::move(actionLabel)});
    if (undoStack_.size() > settings::kMaxUndoHistoryDepth) {
        undoStack_.erase(undoStack_.begin());
    }
}

std::string AppController::lastUndoActionLabel() const {
    return undoStack_.empty() ? std::string{} : undoStack_.back().actionLabel;
}

bool AppController::undoLastModification() {
    if (undoStack_.empty()) {
        return false;
    }
    UndoEntry entry = std::move(undoStack_.back());
    undoStack_.pop_back();

    checkRunner_.cancel();
    replaceMeshPreservingHistory(std::move(entry.mesh), std::move(entry.shells));
    thresholds_ = entry.thresholds;
    modified_ = entry.modified;

    // The restored mesh is identical to the snapshot, so its results are valid again
    health_ = std::move(entry.health);
    for (CategoryHealth& row : health_.rows) {
        if (row.result) {
            row.result->meshGeneration = meshGeneration_;
        }
    }
    for (const DefectCategory category : kAllDefectCategories) {
        computeScope(category);
    }
    refreshShellHealth();
    selectedCategory_ = entry.selectedCategory;
    if (selectedCategory_) {
        activeTool_ = ToolSlot::DefectCategory;
    } else if (activeTool_ == ToolSlot::DefectCategory) {
        activeTool_ = ToolSlot::None;
    }
    syncBrowserResult();

    if (entry.focusedFaceIndex >= 0 && browser_.result() != nullptr) {
        const std::size_t element = browser_.findVisibleElementWithFace(entry.focusedFaceIndex);
        if (element != DefectBrowser::kNone) {
            focusDefect(element);
            if (entry.selectedHealAction != HealAction::None && entry.selectedHealAction != selectedHealAction_) {
                setSelectedHealAction(entry.selectedHealAction);
            }
        }
    }
    launchQueuedChecks();

    lastRepairStatusMessage_ = "Undid: " + entry.actionLabel;
    postStatusMessage(lastRepairStatusMessage_);
    return true;
}

// -----------------------------------------------------------------------------
// Shells
// -----------------------------------------------------------------------------

void AppController::clickShell(std::size_t shell, bool toggle, bool range, std::span<const std::size_t> order) {
    shells_.click(shell, toggle, range, order);
    onShellsChanged();
}

void AppController::selectShellsWhere(const std::function<bool(std::size_t)>& predicate) {
    shells_.selectWhere(predicate);
    onShellsChanged();
}

void AppController::setShellVisible(std::size_t shell, bool visible) {
    shells_.setVisible(shell, visible);
    onShellsChanged();
}

void AppController::showAllShells() {
    shells_.showAll();
    onShellsChanged();
}

void AppController::hideAllShells() {
    shells_.hideAll();
    onShellsChanged();
    postStatusMessage("All shells hidden. Tick Vis in the Shells panel to show the ones to work on.");
}

void AppController::hideSelectedShells() {
    shells_.hideSelected();
    onShellsChanged();
}

bool AppController::toggleIsolateShells() {
    const bool wasIsolated = shells_.isolated();
    if (!shells_.toggleIsolate()) {
        postStatusMessage("Select one or more shells to isolate (click a row, or Alt+click in 3D).");
        return false;
    }
    onShellsChanged();
    if (wasIsolated && !shells_.isolated()) {
        postStatusMessage("Isolate off: the other shells are back.");
    } else {
        postStatusMessage("Isolated " + std::to_string(shells_.selectedCount()) +
                          " shell(s): Mesh Health covers only them. Press I to go back.");
    }
    return true;
}

bool AppController::pickShell(Eigen::Index faceIndex, bool toggle) {
    const int shell = shells_.shellOfFace(faceIndex);
    if (shell < 0 || !shells_.multiShell()) {
        return false;
    }
    shells_.click(static_cast<std::size_t>(shell), toggle, false, {});
    onShellsChanged();
    postStatusMessage("Shell #" + std::to_string(shell + 1) + " (" +
                      std::to_string(shells_.shell(static_cast<std::size_t>(shell)).faces.size()) + " faces)" +
                      (shells_.shell(static_cast<std::size_t>(shell)).selected ? " selected." : " deselected."));
    return true;
}

bool AppController::deleteSelectedShells() {
    const std::vector<std::size_t> selected = shells_.selectedShells();
    if (selected.empty()) {
        postStatusMessage("Select the shells to delete first.");
        return false;
    }
    if (selected.size() == shells_.size()) {
        postStatusMessage("Every shell is selected: deleting them all would leave no mesh.", true);
        return false;
    }
    std::vector<bool> drop(static_cast<std::size_t>(mesh_.numFaces()), false);
    for (const Eigen::Index f : shells_.facesOf(selected)) {
        drop[static_cast<std::size_t>(f)] = true;
    }
    std::vector<Eigen::Index> keep;
    keep.reserve(drop.size());
    for (std::size_t f = 0; f < drop.size(); ++f) {
        if (!drop[f]) {
            keep.push_back(static_cast<Eigen::Index>(f));
        }
    }
    const std::string label = "Delete " + std::to_string(selected.size()) + " Shell(s)";
    pushUndoEntry(label);
    shells_.selectNone(); // the deleted shells were the selection
    finishMeshEdit(extractFaces(mesh_, keep));
    postStatusMessage(label + ": " + std::to_string(drop.size() - keep.size()) + " faces removed.");
    return true;
}

bool AppController::saveSelectedShells(const std::filesystem::path& path, StlFormat format) {
    const std::vector<std::size_t> selected = shells_.selectedShells();
    if (selected.empty()) {
        postStatusMessage("Select the shells to save first.");
        return false;
    }
    std::string err;
    if (!ogun::saveStl(path, extractFaces(mesh_, shells_.facesOf(selected)), format, &err)) {
        lastErrorMessage_ = err.empty() ? "Failed to save STL file." : std::move(err);
        postStatusMessage(lastErrorMessage_, true);
        return false;
    }
    lastErrorMessage_.clear();
    postStatusMessage("Saved " + std::to_string(selected.size()) + " shell(s) to " + path.filename().string());
    return true;
}

std::string AppController::shellFileStem() const {
    // Samples and custom meshes have a label with no file extension
    const std::filesystem::path source = currentFilePath_.empty() ? std::filesystem::path(loadedSourceLabel_) : currentFilePath_;
    const std::string stem = source.stem().string();
    return stem.empty() || !source.has_extension() ? std::string("mesh") : stem;
}

std::size_t AppController::saveEachSelectedShell(const std::filesystem::path& directory, StlFormat format) {
    const std::vector<std::size_t> selected = shells_.selectedShells();
    if (selected.empty()) {
        postStatusMessage("Select the shells to save first.");
        return 0;
    }
    const int width = std::max(2, static_cast<int>(std::to_string(shells_.size()).size()));
    const std::string stem = shellFileStem();
    std::size_t written = 0;
    for (const std::size_t s : selected) {
        std::array<char, 32> number{};
        std::snprintf(number.data(), number.size(), "%0*zu", width, s + 1);
        const std::filesystem::path path = directory / (stem + "_shell_" + number.data() + ".stl");
        const std::array<std::size_t, 1> one{s};
        std::string err;
        if (!ogun::saveStl(path, extractFaces(mesh_, shells_.facesOf(one)), format, &err)) {
            lastErrorMessage_ = err.empty() ? "Failed to save " + path.string() : std::move(err);
            postStatusMessage(lastErrorMessage_, true);
            return written;
        }
        ++written;
    }
    lastErrorMessage_.clear();
    postStatusMessage("Saved " + std::to_string(written) + " shell file(s) to " + directory.string());
    return written;
}

Eigen::Vector3d AppController::shellColor(std::size_t shell) noexcept {
    // Golden-ratio hue steps inside the green..purple band: red, orange and yellow mean defects
    const double t = std::fmod(static_cast<double>(shell) * 0.618033988749895, 1.0);
    const double h = 6.0 * (settings::kShellHueMin + settings::kShellHueSpan * t);
    const double v = settings::kShellColorValues[shell % 2];
    const double sat = settings::kShellColorSaturation;
    const double f = h - std::floor(h);
    const double p = v * (1.0 - sat);
    const double q = v * (1.0 - sat * f);
    const double u = v * (1.0 - sat * (1.0 - f));
    switch (static_cast<int>(h) % 6) {
    case 0: return {v, u, p};
    case 1: return {q, v, p};
    case 2: return {p, v, u};
    case 3: return {p, q, v};
    case 4: return {u, p, v};
    default: return {v, p, q};
    }
}

// -----------------------------------------------------------------------------
// 3D highlight overlays
// -----------------------------------------------------------------------------

HighlightOverlayGeometry AppController::buildHighlightOverlay() const {
    HighlightOverlayGeometry overlay;
    if (mesh_.empty()) {
        return overlay;
    }

    const auto numFaces = static_cast<std::size_t>(mesh_.numFaces());
    overlay.faceColors.assign(numFaces, overlayAppearance_.baseMesh.color);
    // Layers: shell color, then the selected-shell tint, then the defect tints
    if (shells_.multiShell()) {
        const Eigen::Vector3d tint = rgbFromArray(settings::kDefaultShellSelectedColor);
        constexpr double w = settings::kShellSelectedTintWeight;
        for (std::size_t s = 0; s < shells_.size(); ++s) {
            const ShellInfo& shell = shells_.shell(s);
            if (!colorByShell_ && !shell.selected) {
                continue;
            }
            Eigen::Vector3d color = colorByShell_ ? shellColor(s) : overlayAppearance_.baseMesh.color;
            if (shell.selected) {
                color = (1.0 - w) * color + w * tint;
            }
            for (const Eigen::Index f : shell.faces) {
                overlay.faceColors[static_cast<std::size_t>(f)] = color;
            }
        }
    }
    const auto paint = [&](Eigen::Index f, const Eigen::Vector3d& color) {
        if (f >= 0 && f < mesh_.numFaces()) {
            overlay.faceColors[static_cast<std::size_t>(f)] = color;
        }
    };

    const std::vector<Eigen::Index> highlighted = highlightedFaceIndices();
    const CategoryResult* result = browser_.result();
    std::unordered_map<int, std::size_t> nodeOf;
    if (result != nullptr) {
        // Tint every visible defect; tubes / spheres for all of them only under the overlay cap
        std::vector<Eigen::Index> visibleFaces;
        for (const std::size_t element : browser_.visibleRows()) {
            for (const Eigen::Index f : result->facesOf(element)) {
                paint(f, overlayAppearance_.categoryTintColor);
                visibleFaces.push_back(f);
            }
        }
        const ElementKind kind = categoryInfo(result->category).kind;
        const bool underCap = visibleFaces.size() <= overlayCap_;
        if (kind == ElementKind::Face && underCap) {
            appendFaceGeometry(visibleFaces, overlay, false, &nodeOf);
        }
        if (kind == ElementKind::Edge || kind == ElementKind::Vertex) {
            const auto node = [&](int v) {
                const auto [it, inserted] = nodeOf.try_emplace(v, overlay.overlayPoints.size());
                if (inserted) {
                    overlay.overlayPoints.emplace_back(mesh_.V.row(v).transpose());
                }
                return it->second;
            };
            const bool emphasizeOnly = !underCap;
            for (const std::size_t element : browser_.visibleRows()) {
                const bool emphasized = browser_.isSelected(element) || browser_.focusedElement() == element;
                if (emphasizeOnly && !emphasized) {
                    continue;
                }
                const DefectElement& e = result->elements[element];
                if (kind == ElementKind::Edge) {
                    overlay.overlayEdges.push_back({node(e.edge[0]), node(e.edge[1])});
                } else {
                    node(static_cast<int>(e.index));
                }
            }
        }
    }

    for (const Eigen::Index f : highlighted) {
        paint(f, overlayAppearance_.selectedFaces.color);
    }
    appendFaceGeometry(highlighted, overlay, true, &nodeOf);
    return overlay;
}

void AppController::appendFaceGeometry(
    const std::vector<Eigen::Index>& faces,
    HighlightOverlayGeometry& overlay,
    bool addTriangles,
    std::unordered_map<int, std::size_t>* nodeOf) const {
    // Points and edges already in `overlay` (from earlier calls sharing `nodeOf`) are reused
    std::unordered_map<int, std::size_t> localNodeOf;
    std::unordered_map<int, std::size_t>& globalVertToOverlayNode = nodeOf != nullptr ? *nodeOf : localNodeOf;
    globalVertToOverlayNode.reserve(overlay.overlayPoints.size() + faces.size() * 3);
    std::unordered_set<std::uint64_t> seenUndirectedEdges;
    seenUndirectedEdges.reserve(overlay.overlayEdges.size() + faces.size() * 3);
    const auto packEdge = [](std::size_t u, std::size_t v) {
        const auto [lo, hi] = std::minmax(u, v);
        return (static_cast<std::uint64_t>(lo) << 32) | static_cast<std::uint64_t>(hi);
    };
    for (const auto& edge : overlay.overlayEdges) {
        seenUndirectedEdges.insert(packEdge(edge[0], edge[1]));
    }

    for (const Eigen::Index faceIdx : faces) {
        if (faceIdx < 0 || faceIdx >= mesh_.numFaces()) {
            continue;
        }

        std::array<std::size_t, 3> localNodes{};
        for (int c = 0; c < 3; ++c) {
            const int vIdx = mesh_.F(faceIdx, c);
            auto [it, inserted] = globalVertToOverlayNode.try_emplace(vIdx, overlay.overlayPoints.size());
            if (inserted) {
                overlay.overlayPoints.emplace_back(mesh_.V.row(vIdx).transpose());
            }
            localNodes[static_cast<std::size_t>(c)] = it->second;
        }
        if (addTriangles) {
            overlay.overlayFaces.push_back(localNodes);
            overlay.overlayFaceIndices.push_back(faceIdx);
        }

        for (std::size_t c = 0; c < 3; ++c) {
            const std::size_t u = localNodes[c];
            const std::size_t v = localNodes[(c + 1) % 3];
            if (seenUndirectedEdges.insert(packEdge(u, v)).second) {
                overlay.overlayEdges.push_back({u, v});
            }
        }
    }
}

void AppController::setGhostMeshOpacity(double opacity) noexcept {
    if (std::isfinite(opacity)) {
        ghostMeshOpacity_ = std::clamp(opacity, settings::kMinGhostOpacity, 1.0);
    }
}

void AppController::setBaseMeshTransparency(double opacity) noexcept {
    if (!std::isfinite(opacity)) {
        return;
    }
    if (opacity >= 1.0) {
        ghostMeshEnabled_ = false;
        return;
    }
    setGhostMeshOpacity(opacity);
    ghostMeshEnabled_ = true;
}

bool SurfaceStyle::adoptPolyscopeEdgeWidth(double polyscopeWidth) noexcept {
    if (!(polyscopeWidth > 0.0)) {
        const bool changed = showEdges;
        showEdges = false;
        return changed;
    }
    if (!showEdges) {
        // Polyscope's "Edges" checkbox re-enables edges at exactly 1.0; keep the remembered width instead
        showEdges = true;
        if (polyscopeWidth != 1.0) {
            edgeWidthPx = polyscopeWidth;
        }
        return true;
    }
    if (std::abs(polyscopeWidth - edgeWidthPx) > 1.0e-4) {
        edgeWidthPx = polyscopeWidth;
        return true;
    }
    return false;
}

void AppController::setOverlayAppearance(const OverlayAppearance& appearance) noexcept {
    const auto clampColor = [](const Eigen::Vector3d& c, const Eigen::Vector3d& fallback) {
        return c.allFinite() ? c.cwiseMax(0.0).cwiseMin(1.0).eval() : fallback;
    };
    const auto clampWidth = [](double width, double fallback) {
        return std::isfinite(width)
            ? std::clamp(width, settings::kMinOverlayEdgeWidthPx, settings::kMaxOverlayEdgeWidthPx)
            : fallback;
    };
    const auto clampStyle = [&](const SurfaceStyle& in, SurfaceStyle& out) {
        out.color = clampColor(in.color, out.color);
        out.edgeColor = clampColor(in.edgeColor, out.edgeColor);
        out.edgeWidthPx = clampWidth(in.edgeWidthPx, out.edgeWidthPx);
        out.showEdges = in.showEdges;
    };
    clampStyle(appearance.selectedFaces, overlayAppearance_.selectedFaces);
    clampStyle(appearance.previewPatch, overlayAppearance_.previewPatch);
    overlayAppearance_.originalEdgeColor = clampColor(appearance.originalEdgeColor, overlayAppearance_.originalEdgeColor);
    overlayAppearance_.originalEdgeWidthPx = clampWidth(appearance.originalEdgeWidthPx, overlayAppearance_.originalEdgeWidthPx);
    if (std::isfinite(appearance.originalEdgeOpacity)) {
        overlayAppearance_.originalEdgeOpacity = std::clamp(appearance.originalEdgeOpacity, settings::kMinPreviewOpacity, 1.0);
    }
    overlayAppearance_.pointColor = clampColor(appearance.pointColor, overlayAppearance_.pointColor);
    if (std::isfinite(appearance.pointRadiusPx)) {
        overlayAppearance_.pointRadiusPx = std::clamp(
            appearance.pointRadiusPx,
            settings::kMinHighlightPointRadiusPx,
            settings::kMaxHighlightPointRadiusPx);
    }
}

double AppController::worldLengthPerPixel(
    double viewDistance,
    double fovDegrees,
    double viewportHeightPx) noexcept {
    if (!std::isfinite(viewDistance) || viewDistance <= 0.0 || !(viewportHeightPx > 0.0)) {
        return 0.0;
    }
    const double clampedFovDeg = std::clamp(
        fovDegrees,
        settings::kMinCameraFovDegrees,
        settings::kMaxCameraFovDegrees);
    const double viewHeight = 2.0 * viewDistance * std::tan((clampedFovDeg * 0.5) * kDegToRad);
    return viewHeight / viewportHeightPx;
}

double AppController::worldLengthAtSelection(
    double pixels,
    const Eigen::Vector3d& cameraPosition,
    double fovDegrees,
    double viewportHeightPx) const noexcept {
    if (mesh_.empty()) {
        return 0.0;
    }
    const double dist = (cameraPosition - computeSelectionCentroid()).norm();
    return pixels * worldLengthPerPixel(dist, fovDegrees, viewportHeightPx);
}

double AppController::referenceFullMeshViewDistance() const noexcept {
    if (mesh_.empty()) {
        return 1.0;
    }
    const double meshDiag = mesh_.boundingBox().diagonal();
    return std::max(1.0e-9, kReferenceFullMeshDistanceFactor * std::max(meshDiag, 1.0e-9));
}

double AppController::computeHighlightZoomScale(
    const Eigen::Vector3d& cameraPosition,
    double fovDegrees) const noexcept {
    if (mesh_.empty()) {
        return 1.0;
    }

    const Eigen::Vector3d targetCenter = computeSelectionCentroid();
    const double camDist = (cameraPosition - targetCenter).norm();
    const double refDist = referenceFullMeshViewDistance();
    if (!std::isfinite(camDist) || refDist <= 0.0) {
        return 1.0;
    }

    constexpr double kDefaultFovRadHalf = (settings::kDefaultCameraFovDegrees * 0.5) * kDegToRad;
    const double clampedFovDeg = std::clamp(
        fovDegrees,
        settings::kMinCameraFovDegrees,
        settings::kMaxCameraFovDegrees);
    const double fovRadHalf = (clampedFovDeg * 0.5) * kDegToRad;
    const double fovFactor = std::tan(fovRadHalf) / std::tan(kDefaultFovRadHalf);

    const double rawScale = (camDist / refDist) * fovFactor;
    return std::clamp(rawScale, kMinHighlightZoomScale, 1.0);
}


Eigen::Vector3d AppController::computeFaceCentroid(Eigen::Index faceIndex) const noexcept {
    if (mesh_.empty() || faceIndex < 0 || faceIndex >= mesh_.numFaces()) {
        return Eigen::Vector3d::Zero();
    }
    const int i0 = mesh_.F(faceIndex, 0);
    const int i1 = mesh_.F(faceIndex, 1);
    const int i2 = mesh_.F(faceIndex, 2);
    if (i0 < 0 || i0 >= mesh_.numVertices() ||
        i1 < 0 || i1 >= mesh_.numVertices() ||
        i2 < 0 || i2 >= mesh_.numVertices()) {
        return Eigen::Vector3d::Zero();
    }
    const Eigen::Vector3d v0 = mesh_.V.row(i0).transpose();
    const Eigen::Vector3d v1 = mesh_.V.row(i1).transpose();
    const Eigen::Vector3d v2 = mesh_.V.row(i2).transpose();
    return (v0 + v1 + v2) / 3.0;
}

Eigen::Vector3d AppController::computeSelectionCentroid() const noexcept {
    if (mesh_.empty()) {
        return Eigen::Vector3d::Zero();
    }
    const Eigen::Index face = focusedSingleFace();
    if (face >= 0 && face < mesh_.numFaces()) {
        return computeFaceCentroid(face);
    }
    return computeActiveSelectionOrMeshBounds().center();
}

Eigen::Vector3d AppController::computeFaceInspectionNormal(Eigen::Index faceIndex) const noexcept {
    if (mesh_.empty() || faceIndex < 0 || faceIndex >= mesh_.numFaces()) {
        return Eigen::Vector3d(0.0, 0.0, 1.0);
    }

    if (mesh_.N.rows() == mesh_.numFaces() && mesh_.N.cols() >= 3) {
        const Eigen::Vector3d cachedN = mesh_.N.row(faceIndex).transpose();
        const double cachedNorm = cachedN.norm();
        if (cachedNorm > 1.0e-12 && std::isfinite(cachedNorm)) {
            return cachedN / cachedNorm;
        }
    }

    const int i0 = mesh_.F(faceIndex, 0);
    const int i1 = mesh_.F(faceIndex, 1);
    const int i2 = mesh_.F(faceIndex, 2);
    if (i0 >= 0 && i0 < mesh_.numVertices() &&
        i1 >= 0 && i1 < mesh_.numVertices() &&
        i2 >= 0 && i2 < mesh_.numVertices()) {
        const Eigen::Vector3d v0 = mesh_.V.row(i0).transpose();
        const Eigen::Vector3d v1 = mesh_.V.row(i1).transpose();
        const Eigen::Vector3d v2 = mesh_.V.row(i2).transpose();

        const Eigen::Vector3d cross = (v1 - v0).cross(v2 - v0);
        const double crossNorm = cross.norm();
        if (crossNorm > 1.0e-15 && std::isfinite(crossNorm)) {
            return cross / crossNorm;
        }

        // Zero-area collinear triangle fallback: pick a unit vector perpendicular to the longest edge
        const Eigen::Vector3d e0 = v1 - v0;
        const Eigen::Vector3d e1 = v2 - v1;
        const Eigen::Vector3d e2 = v0 - v2;
        Eigen::Vector3d longestEdge = e0;
        if (e1.squaredNorm() > longestEdge.squaredNorm()) {
            longestEdge = e1;
        }
        if (e2.squaredNorm() > longestEdge.squaredNorm()) {
            longestEdge = e2;
        }
        const double edgeLen = longestEdge.norm();
        if (edgeLen > 1.0e-15 && std::isfinite(edgeLen)) {
            const Eigen::Vector3d edgeDir = longestEdge / edgeLen;
            const Eigen::Vector3d refAxis = (std::abs(edgeDir.z()) < 0.9)
                ? Eigen::Vector3d(0.0, 0.0, 1.0)
                : Eigen::Vector3d(0.0, 1.0, 0.0);
            const Eigen::Vector3d perp = edgeDir.cross(refAxis);
            const double perpNorm = perp.norm();
            if (perpNorm > 1.0e-12) {
                return perp / perpNorm;
            }
        }
    }

    return Eigen::Vector3d(0.0, 0.0, 1.0);
}

void AppController::setOrbitPivot(const Eigen::Vector3d& pivot) noexcept {
    if (pivot.allFinite()) {
        orbitPivot_ = pivot;
    }
}

void AppController::translateOrbitPivot(const Eigen::Vector3d& worldDelta) noexcept {
    if (worldDelta.allFinite()) {
        orbitPivot_ += worldDelta;
    }
}

void AppController::resetOrbitPivotToSelection() noexcept {
    orbitPivot_ = computeSelectionCentroid();
}

void AppController::resetOrbitPivotToMeshCenter() noexcept {
    lastFocusedFaceIndex_ = -1;
    lastFocusedNormalSign_ = 0;
    if (mesh_.empty()) {
        orbitPivot_ = Eigen::Vector3d::Zero();
    } else {
        orbitPivot_ = mesh_.boundingBox().center();
    }
}

Eigen::Vector3d AppController::computeOrthogonalUpVector(
    const Eigen::Vector3d& lookDirection,
    const Eigen::Vector3d& preferredUp) noexcept {
    Eigen::Vector3d lookDir = lookDirection;
    if (lookDir.norm() <= 1.0e-12 || !lookDir.allFinite()) {
        lookDir = Eigen::Vector3d(0.0, 0.0, -1.0);
    } else {
        lookDir.normalize();
    }

    Eigen::Vector3d up = preferredUp;
    if (up.norm() <= 1.0e-12 || !up.allFinite()) {
        up = Eigen::Vector3d(0.0, 1.0, 0.0);
    } else {
        up.normalize();
    }

    Eigen::Vector3d projUp = up - up.dot(lookDir) * lookDir;
    if (projUp.norm() > 1.0e-3) {
        return projUp.normalized();
    }

    // `preferredUp` is nearly collinear with `lookDir`: pick a stable orthogonal fallback axis
    const Eigen::Vector3d fallbackAxis = (std::abs(lookDir.z()) < 0.9)
        ? Eigen::Vector3d(0.0, 0.0, 1.0)
        : Eigen::Vector3d(1.0, 0.0, 0.0);
    projUp = fallbackAxis - fallbackAxis.dot(lookDir) * lookDir;
    return projUp.normalized();
}

BoundingBox AppController::computeActiveSelectionOrMeshBounds() const noexcept {
    bool initializedBounds = false;
    BoundingBox bounds;
    const auto addBox = [&](const BoundingBox& box) {
        bounds = initializedBounds ? BoundingBox{bounds.min.cwiseMin(box.min), bounds.max.cwiseMax(box.max)} : box;
        initializedBounds = true;
    };

    for (const Eigen::Index faceIdx : activeFrameFaces()) {
        if (faceIdx < 0 || faceIdx >= mesh_.numFaces()) {
            continue;
        }
        for (int c = 0; c < 3; ++c) {
            const int vIdx = mesh_.F(faceIdx, c);
            if (vIdx < 0 || vIdx >= mesh_.numVertices()) {
                continue;
            }
            const Eigen::Vector3d p = mesh_.V.row(vIdx).transpose();
            if (!initializedBounds) {
                bounds.min = p;
                bounds.max = p;
                initializedBounds = true;
            } else {
                bounds.min = bounds.min.cwiseMin(p);
                bounds.max = bounds.max.cwiseMax(p);
            }
        }
    }

    // No defect to frame: the selected shells, else the visible ones when some are hidden
    if (!initializedBounds && shells_.multiShell()) {
        const bool selected = shells_.selectedCount() > 0;
        if (selected || shells_.visibleCount() < shells_.size()) {
            for (const ShellInfo& shell : shells_.shells()) {
                if (selected ? shell.selected : shell.visible) {
                    addBox(shell.bbox);
                }
            }
        }
    }
    if (!initializedBounds && !mesh_.empty()) {
        bounds = mesh_.boundingBox();
    }
    return bounds;
}

CameraFocusTarget AppController::computeCameraFocusTarget(
    const Eigen::Vector3d& currentViewDirection,
    FocusNormalSide normalSide,
    const Eigen::Vector3d& preferredUpDirection) const {
    CameraFocusTarget focus;
    if (mesh_.empty()) {
        return focus;
    }

    const BoundingBox bounds = computeActiveSelectionOrMeshBounds();
    focus.selectionBounds = bounds;
    focus.targetCenter = computeSelectionCentroid();

    const double selDiag = bounds.diagonal();
    const double meshDiag = std::max(mesh_.boundingBox().diagonal(), 1.0e-6);
    const double minRadius = 1.0e-3 * meshDiag;
    const double rawRadius = (selDiag > 0.0)
        ? (1.75 * selDiag)
        : (0.1 * meshDiag);
    focus.focusRadius = std::max(rawRadius, minRadius);

    Eigen::Vector3d viewDir = currentViewDirection;
    int resolvedNormalSign = 0;

    const Eigen::Index focusedFace = focusedSingleFace();
    if (focusedFace >= 0 &&
        focusedFace < mesh_.numFaces() &&
        normalSide != FocusNormalSide::PreserveViewDirection) {
        if (normalSide == FocusNormalSide::NegativeNormal) {
            resolvedNormalSign = -1;
        } else if (normalSide == FocusNormalSide::PositiveNormal) {
            resolvedNormalSign = +1;
        } else if (normalSide == FocusNormalSide::AutoToggleNormal) {
            resolvedNormalSign = (lastFocusedFaceIndex_ == focusedFace && lastFocusedNormalSign_ == +1)
                ? -1
                : +1;
        }

        const Eigen::Vector3d faceNormal = computeFaceInspectionNormal(focusedFace);
        // Looking at the face from the +N side means the camera sits at +N and looks along -N
        viewDir = -static_cast<double>(resolvedNormalSign) * faceNormal;
    }

    if (viewDir.norm() <= 1.0e-12 || !viewDir.allFinite()) {
        viewDir = Eigen::Vector3d(-1.0, -1.0, -1.0);
    }
    viewDir.normalize();

    focus.normalSideSign = resolvedNormalSign;
    focus.cameraPosition = focus.targetCenter - viewDir * focus.focusRadius;
    focus.cameraUpDirection = computeOrthogonalUpVector(viewDir, preferredUpDirection);
    focus.valid = true;
    return focus;
}

CameraFocusTarget AppController::focusOnSelection(
    FocusNormalSide normalSide,
    const Eigen::Vector3d& currentViewDirection,
    const Eigen::Vector3d& preferredUpDirection) {
    const CameraFocusTarget focus = computeCameraFocusTarget(
        currentViewDirection,
        normalSide,
        preferredUpDirection);
    if (!focus.valid) {
        return focus;
    }

    orbitPivot_ = focus.targetCenter;
    if (focus.normalSideSign != 0) {
        lastFocusedFaceIndex_ = focusedSingleFace();
        lastFocusedNormalSign_ = focus.normalSideSign;
    } else {
        lastFocusedFaceIndex_ = -1;
        lastFocusedNormalSign_ = 0;
    }
    return focus;
}

} // namespace ogun
