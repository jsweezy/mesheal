#include "app/ShellBrowser.hpp"
#include "core/Settings.hpp"
#include "diagnostics/MeshChecks.hpp"

#include <algorithm>
#include <limits>
#include <numeric>
#include <unordered_map>
#include <utility>

namespace ogun {

namespace {

constexpr std::size_t kNoAnchor = static_cast<std::size_t>(-1);

[[nodiscard]] BoundingBox boxOfFaces(const Mesh& mesh, const std::vector<Eigen::Index>& faces) {
    constexpr double kInf = std::numeric_limits<double>::infinity();
    BoundingBox box{Eigen::Vector3d::Constant(kInf), Eigen::Vector3d::Constant(-kInf)};
    for (const Eigen::Index f : faces) {
        for (int c = 0; c < 3; ++c) {
            const Eigen::Vector3d p = mesh.V.row(mesh.F(f, c)).transpose();
            box.min = box.min.cwiseMin(p);
            box.max = box.max.cwiseMax(p);
        }
    }
    return box;
}

/// Overlap of two boxes padded by `pad` (intersection / union volume), so flat boxes still compare.
[[nodiscard]] double boxOverlap(const BoundingBox& a, const BoundingBox& b, double pad) {
    const Eigen::Array3d lo = a.min.cwiseMax(b.min).array() - pad;
    const Eigen::Array3d hi = a.max.cwiseMin(b.max).array() + pad;
    if ((hi <= lo).any()) {
        return 0.0;
    }
    const double inter = (hi - lo).prod();
    const double va = (a.extents().array() + 2.0 * pad).prod();
    const double vb = (b.extents().array() + 2.0 * pad).prod();
    return inter / (va + vb - inter);
}

[[nodiscard]] bool vertexValid(const Mesh& mesh, Eigen::Index f) noexcept {
    for (int c = 0; c < 3; ++c) {
        if (mesh.F(f, c) < 0 || mesh.F(f, c) >= mesh.numVertices()) {
            return false;
        }
    }
    return true;
}

} // namespace

void ShellBrowser::clear() {
    revision_ = nextRevision();
    shells_.clear();
    faceShell_.clear();
    isolateRestore_.reset();
    selectedCount_ = 0;
    anchor_ = kNoAnchor;
}

void ShellBrowser::rebuild(const Mesh& mesh) {
    revision_ = nextRevision();
    std::vector<ShellInfo> old = std::move(shells_);
    std::optional<std::vector<bool>> oldRestore = std::move(isolateRestore_);
    clear();
    if (mesh.empty()) {
        return;
    }

    // checkShells lists the smallest first; the panel numbers the largest #1
    const CategoryResult result = checkShells(mesh);
    std::vector<std::size_t> order(result.size());
    std::iota(order.begin(), order.end(), std::size_t{0});
    std::sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) {
        const std::size_t na = result.facesOf(a).size();
        const std::size_t nb = result.facesOf(b).size();
        return na != nb ? na > nb : result.facesOf(a).front() < result.facesOf(b).front();
    });
    const auto nf = static_cast<std::size_t>(mesh.numFaces());
    faceShell_.assign(nf, -1);
    shells_.resize(order.size());
    std::vector<int> vertexShell(static_cast<std::size_t>(mesh.numVertices()), -1);
    for (std::size_t s = 0; s < order.size(); ++s) {
        const auto faces = result.facesOf(order[s]);
        shells_[s].faces.assign(faces.begin(), faces.end());
        for (const Eigen::Index f : faces) {
            faceShell_[static_cast<std::size_t>(f)] = static_cast<int>(s);
            for (int c = 0; c < 3; ++c) {
                vertexShell[static_cast<std::size_t>(mesh.F(f, c))] = static_cast<int>(s);
            }
        }
    }
    // Faces checkShells skips (a repeated vertex) join the shell of one of their vertices
    bool attached = false;
    for (std::size_t f = 0; f < nf; ++f) {
        const auto fi = static_cast<Eigen::Index>(f);
        if (faceShell_[f] >= 0 || !vertexValid(mesh, fi)) {
            continue;
        }
        for (int c = 0; c < 3 && faceShell_[f] < 0; ++c) {
            faceShell_[f] = vertexShell[static_cast<std::size_t>(mesh.F(fi, c))];
        }
        if (faceShell_[f] >= 0) {
            shells_[static_cast<std::size_t>(faceShell_[f])].faces.push_back(fi);
            attached = true;
        }
    }
    for (ShellInfo& shell : shells_) {
        if (attached) {
            std::sort(shell.faces.begin(), shell.faces.end());
        }
        shell.bbox = boxOfFaces(mesh, shell.faces);
    }

    // Carry the view state over: exact match (face count and box) first, then the best box overlap
    const double tol = settings::kShellMatchBoxTolerance * std::max(mesh.boundingBox().diagonal(), 1.0e-12);
    std::vector<std::size_t> match(shells_.size(), kNoAnchor);
    std::vector<bool> used(old.size(), false);
    std::unordered_map<std::size_t, std::vector<std::size_t>> oldBySize;
    for (std::size_t o = 0; o < old.size(); ++o) {
        oldBySize[old[o].faces.size()].push_back(o);
    }
    for (std::size_t s = 0; s < shells_.size(); ++s) {
        const auto it = oldBySize.find(shells_[s].faces.size());
        if (it == oldBySize.end()) {
            continue;
        }
        for (const std::size_t o : it->second) {
            if (!used[o] && (old[o].bbox.min - shells_[s].bbox.min).cwiseAbs().maxCoeff() <= tol &&
                (old[o].bbox.max - shells_[s].bbox.max).cwiseAbs().maxCoeff() <= tol) {
                match[s] = o;
                used[o] = true;
                break;
            }
        }
    }
    for (std::size_t s = 0; s < shells_.size(); ++s) {
        if (match[s] != kNoAnchor) {
            continue;
        }
        double best = settings::kShellMatchMinOverlap;
        for (std::size_t o = 0; o < old.size(); ++o) {
            const double overlap = boxOverlap(old[o].bbox, shells_[s].bbox, tol);
            if (overlap >= best) {
                best = overlap;
                match[s] = o;
            }
        }
    }
    if (oldRestore && !old.empty()) {
        isolateRestore_.emplace(shells_.size(), true);
    }
    std::vector<int> newOf(old.size(), -1);
    for (std::size_t s = 0; s < shells_.size(); ++s) {
        if (match[s] != kNoAnchor) {
            newOf[match[s]] = static_cast<int>(s);
        }
    }
    for (std::size_t s = 0; s < shells_.size(); ++s) {
        if (match[s] == kNoAnchor) {
            continue;
        }
        const ShellInfo& was = old[match[s]];
        shells_[s].visible = was.visible;
        shells_[s].selected = was.selected;
        // Kept until the next intersection check recomputes it
        shells_[s].nestedIn = was.nestedIn >= 0 ? newOf[static_cast<std::size_t>(was.nestedIn)] : -1;
        if (isolateRestore_ && match[s] < oldRestore->size()) {
            (*isolateRestore_)[s] = (*oldRestore)[match[s]];
        }
    }
    recountSelection();
}

void ShellBrowser::updateBoxes(const Mesh& mesh) {
    revision_ = nextRevision();
    for (ShellInfo& shell : shells_) {
        shell.bbox = boxOfFaces(mesh, shell.faces);
    }
}

int ShellBrowser::shellOfFace(Eigen::Index f) const noexcept {
    return f >= 0 && static_cast<std::size_t>(f) < faceShell_.size() ? faceShell_[static_cast<std::size_t>(f)] : -1;
}

void ShellBrowser::setNestedIn(std::span<const int> nestedIn) {
    revision_ = nextRevision();
    for (std::size_t s = 0; s < shells_.size(); ++s) {
        shells_[s].nestedIn = s < nestedIn.size() ? nestedIn[s] : -1;
    }
}

void ShellBrowser::recountSelection() noexcept {
    selectedCount_ = static_cast<std::size_t>(
        std::count_if(shells_.begin(), shells_.end(), [](const ShellInfo& s) { return s.selected; }));
}

void ShellBrowser::setVisible(std::size_t s, bool visible) {
    revision_ = nextRevision();
    if (s < shells_.size()) {
        shells_[s].visible = visible;
    }
}

void ShellBrowser::showAll() {
    revision_ = nextRevision();
    for (ShellInfo& shell : shells_) {
        shell.visible = true;
    }
    isolateRestore_.reset();
}

void ShellBrowser::hideAll() {
    revision_ = nextRevision();
    for (ShellInfo& shell : shells_) {
        shell.visible = false;
        shell.selected = false;
    }
    isolateRestore_.reset();
    selectedCount_ = 0;
}

void ShellBrowser::hideSelected() {
    revision_ = nextRevision();
    for (ShellInfo& shell : shells_) {
        if (shell.selected) {
            shell.visible = false;
            shell.selected = false;
        }
    }
    selectedCount_ = 0;
}

bool ShellBrowser::toggleIsolate() {
    revision_ = nextRevision();
    if (isolateRestore_ && (selectedCount_ == 0 || selectionMatchesVisible())) {
        for (std::size_t s = 0; s < shells_.size(); ++s) {
            shells_[s].visible = (*isolateRestore_)[s];
        }
        isolateRestore_.reset();
        return true;
    }
    if (selectedCount_ == 0) {
        return false;
    }
    // Keep the view from before the first Isolate, so un-isolating always returns there
    if (!isolateRestore_) {
        std::vector<bool> restore(shells_.size());
        for (std::size_t s = 0; s < shells_.size(); ++s) {
            restore[s] = shells_[s].visible;
        }
        isolateRestore_ = std::move(restore);
    }
    for (ShellInfo& shell : shells_) {
        shell.visible = shell.selected;
    }
    return true;
}

std::size_t ShellBrowser::visibleCount() const noexcept {
    return static_cast<std::size_t>(
        std::count_if(shells_.begin(), shells_.end(), [](const ShellInfo& s) { return s.visible; }));
}

bool ShellBrowser::faceVisible(Eigen::Index f) const noexcept {
    const int s = shellOfFace(f);
    return s < 0 || shells_[static_cast<std::size_t>(s)].visible;
}

void ShellBrowser::click(std::size_t s, bool toggle, bool range, std::span<const std::size_t> order) {
    revision_ = nextRevision();
    if (s >= shells_.size()) {
        return;
    }
    if (range && anchor_ != kNoAnchor) {
        const auto a = std::find(order.begin(), order.end(), anchor_);
        const auto b = std::find(order.begin(), order.end(), s);
        if (a != order.end() && b != order.end()) {
            if (!toggle) {
                for (ShellInfo& shell : shells_) {
                    shell.selected = false;
                }
            }
            for (auto it = std::min(a, b); it <= std::max(a, b); ++it) {
                shells_[*it].selected = true;
            }
            recountSelection();
            return;
        }
    }
    if (toggle) {
        shells_[s].selected = !shells_[s].selected;
    } else {
        for (ShellInfo& shell : shells_) {
            shell.selected = false;
        }
        shells_[s].selected = true;
    }
    anchor_ = s;
    recountSelection();
}

void ShellBrowser::selectWhere(const std::function<bool(std::size_t)>& predicate) {
    revision_ = nextRevision();
    for (std::size_t s = 0; s < shells_.size(); ++s) {
        shells_[s].selected = predicate(s);
    }
    anchor_ = kNoAnchor;
    recountSelection();
}

void ShellBrowser::selectAll() {
    revision_ = nextRevision();
    selectWhere([](std::size_t) { return true; });
}

void ShellBrowser::selectNone() {
    revision_ = nextRevision();
    selectWhere([](std::size_t) { return false; });
}

void ShellBrowser::invertSelection() {
    revision_ = nextRevision();
    selectWhere([this](std::size_t s) { return !shells_[s].selected; });
}

std::vector<std::size_t> ShellBrowser::selectedShells() const {
    std::vector<std::size_t> out;
    out.reserve(selectedCount_);
    for (std::size_t s = 0; s < shells_.size(); ++s) {
        if (shells_[s].selected) {
            out.push_back(s);
        }
    }
    return out;
}

bool ShellBrowser::selectionMatchesVisible() const noexcept {
    return std::all_of(shells_.begin(), shells_.end(), [](const ShellInfo& s) { return s.selected == s.visible; });
}

std::vector<Eigen::Index> ShellBrowser::facesOf(std::span<const std::size_t> shells) const {
    std::vector<Eigen::Index> faces;
    for (const std::size_t s : shells) {
        if (s < shells_.size()) {
            faces.insert(faces.end(), shells_[s].faces.begin(), shells_[s].faces.end());
        }
    }
    std::sort(faces.begin(), faces.end());
    return faces;
}

bool ShellBrowser::scoped() const noexcept {
    return multiShell() && (selectedCount_ > 0 || visibleCount() < shells_.size());
}

bool ShellBrowser::inScope(int s) const noexcept {
    if (s < 0 || static_cast<std::size_t>(s) >= shells_.size()) {
        return true;
    }
    const ShellInfo& shell = shells_[static_cast<std::size_t>(s)];
    return selectedCount_ > 0 ? shell.selected : shell.visible;
}

std::size_t ShellBrowser::scopeCount() const noexcept {
    return selectedCount_ > 0 ? selectedCount_ : visibleCount();
}

} // namespace ogun
