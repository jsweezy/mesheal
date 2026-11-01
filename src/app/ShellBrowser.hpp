#pragma once

#include "app/Revision.hpp"
#include "mesh/Mesh.hpp"

#include <Eigen/Core>

#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <span>
#include <vector>

namespace ogun {

/// One edge-connected piece of the mesh with its view state (UI_DESIGN.md §16).
struct ShellInfo {
    std::vector<Eigen::Index> faces;
    BoundingBox bbox;
    int nestedIn = -1; ///< Shell this one lies inside without crossing it (-1: none or not known)
    bool visible = true;
    bool selected = false;
};

/// Headless state of the Shells panel: the shells of the current mesh (largest first, shown as #1..),
/// their visibility, selection and Isolate, and the scope that Mesh Health covers (FB-22): the
/// selected shells, or the visible shells when none is selected.
class ShellBrowser {
public:
    /// Recomputes the shells of `mesh`. When more than one shell remains, visibility, selection and the
    /// nested tag carry over from the previous shells that match (same face count and box, otherwise the
    /// most overlapping box), so they survive edits; a single-shell mesh is always visible and unselected.
    void rebuild(const Mesh& mesh);

    /// Forgets every shell and its state (a new mesh was loaded).
    void clear();

    /// Recomputes the shell boxes after the vertices moved but the faces did not (a rigid transform):
    /// the shells, their numbering and their view state stay as they are.
    void updateBoxes(const Mesh& mesh);

    [[nodiscard]] std::size_t size() const noexcept {
        return shells_.size();
    }

    /// True when the mesh has more than one shell (the Shells panel is shown).
    [[nodiscard]] bool multiShell() const noexcept {
        return shells_.size() > 1;
    }

    [[nodiscard]] const ShellInfo& shell(std::size_t s) const noexcept {
        return shells_[s];
    }

    [[nodiscard]] const std::vector<ShellInfo>& shells() const noexcept {
        return shells_;
    }

    /// Shell of face `f` (-1 for a face in no shell, e.g. one with invalid vertex indices).
    [[nodiscard]] int shellOfFace(Eigen::Index f) const noexcept;

    void setNestedIn(std::span<const int> nestedIn);

    // ---- Visibility (view only: no undo step) ----

    void setVisible(std::size_t s, bool visible);
    void showAll();
    /// Hides every shell and clears the selection.
    void hideAll();
    /// Hides and deselects the selected shells.
    void hideSelected();
    /// Isolates the selected shells (only they stay visible). Pressed again with the same selection,
    /// or with none, it restores the view from before the first Isolate; a different selection
    /// re-isolates onto it. Returns false when there was nothing to isolate or restore.
    bool toggleIsolate();

    [[nodiscard]] bool isolated() const noexcept {
        return isolateRestore_.has_value();
    }

    [[nodiscard]] std::size_t visibleCount() const noexcept;
    [[nodiscard]] bool faceVisible(Eigen::Index f) const noexcept;

    // ---- Selection ----

    /// Click on shell `s`: plain selects only it, `toggle` (Ctrl/Cmd) flips it, `range` (Shift)
    /// selects from the anchor through `s` in `order` (the table's display order).
    void click(std::size_t s, bool toggle, bool range, std::span<const std::size_t> order);
    void selectWhere(const std::function<bool(std::size_t)>& predicate);
    void selectAll();
    void selectNone();
    void invertSelection();

    [[nodiscard]] std::size_t selectedCount() const noexcept {
        return selectedCount_;
    }
    [[nodiscard]] std::vector<std::size_t> selectedShells() const;
    /// True when exactly the visible shells are selected.
    [[nodiscard]] bool selectionMatchesVisible() const noexcept;
    /// Faces of the given shells, in ascending face order.
    [[nodiscard]] std::vector<Eigen::Index> facesOf(std::span<const std::size_t> shells) const;

    // ---- Scope ----

    /// True when Mesh Health covers only part of the mesh (several shells, some hidden or selected).
    [[nodiscard]] bool scoped() const noexcept;
    /// Shell `s` is in the scope (-1, a face in no shell, always is).
    [[nodiscard]] bool inScope(int s) const noexcept;
    /// Number of shells in the scope.
    [[nodiscard]] std::size_t scopeCount() const noexcept;

    /// Changes with every change of the shells or their visibility / selection (see nextRevision).
    [[nodiscard]] std::uint64_t revision() const noexcept {
        return revision_;
    }

private:
    void recountSelection() noexcept;

    std::vector<ShellInfo> shells_;
    std::vector<int> faceShell_;
    std::optional<std::vector<bool>> isolateRestore_; ///< Visibility to restore when un-isolating
    std::size_t selectedCount_ = 0;
    std::size_t anchor_ = static_cast<std::size_t>(-1);
    std::uint64_t revision_ = nextRevision();
};

} // namespace ogun
