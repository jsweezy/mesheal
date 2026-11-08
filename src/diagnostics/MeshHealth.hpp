#pragma once

#include "core/Settings.hpp"
#include "diagnostics/ThinFaceChecker.hpp"
#include "mesh/Mesh.hpp"

#include <Eigen/Core>

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <span>
#include <stop_token>
#include <string>
#include <vector>

namespace mesheal {

/// One row of the Mesh Health panel (UI_DESIGN.md §5.1).
enum class DefectCategory : std::uint8_t {
    OpenEdges,
    NonManifoldEdges,
    NonManifoldVertices,
    FlippedNormals,
    Shells,
    Intersections,
    ThinFaces,
    Degenerate,
    Duplicates,
    InvertedShells ///< Last in the enum (indices of saved arrays stay put); shown after Flipped normals
};

inline constexpr std::size_t kDefectCategoryCount = 10;

/// Every row in panel order.
inline constexpr std::array<DefectCategory, kDefectCategoryCount> kAllDefectCategories = {
    DefectCategory::Shells,
    DefectCategory::OpenEdges,
    DefectCategory::NonManifoldEdges,
    DefectCategory::NonManifoldVertices,
    DefectCategory::FlippedNormals,
    DefectCategory::InvertedShells,
    DefectCategory::Intersections,
    DefectCategory::ThinFaces,
    DefectCategory::Degenerate,
    DefectCategory::Duplicates,
};

[[nodiscard]] constexpr std::size_t categoryIndex(DefectCategory category) noexcept {
    return static_cast<std::size_t>(category);
}

/// What one defect element of a category refers to.
enum class ElementKind { Face, Edge, Vertex, Shell, Hole };

enum class CategoryGroup { Topology, Geometry };

/// Physical dimension of a list column or metric, used to append the unit symbol.
enum class ValueDimension { None, Length, Area, Volume };

struct CategoryColumn {
    const char* name = nullptr; ///< nullptr: column unused
    ValueDimension dimension = ValueDimension::None;
};

/// Static description of a Mesh Health row: labels, tooltips, list columns and histogram metric.
struct CategoryInfo {
    const char* name;           ///< Row label, e.g. "Holes (open edges)"
    const char* menuName;       ///< Repair menu label, e.g. "Holes (open edges)"
    CategoryGroup group;
    const char* description;    ///< One-line explanation (tooltip and Tool panel description)
    ElementKind kind;
    const char* elementNoun;    ///< Plural noun for counts, e.g. "edges"
    bool autoDefault;           ///< Default of the row's Auto checkbox
    bool infoOnly;              ///< Information row: never orange (e.g. shells)
    bool hasFixAll;             ///< True once a Fix All repair exists for this row
    std::array<CategoryColumn, 3> columns;
    const char* metricName;     ///< Histogram metric (nullptr: no histogram)
    ValueDimension metricDimension;
    bool metricLogScale;
    bool metricLowIsWorst;
    std::array<const char*, 4> subtypes; ///< Sub-type filter labels (nullptr-terminated)
};

[[nodiscard]] const CategoryInfo& categoryInfo(DefectCategory category) noexcept;

/// Number of non-null entries in `CategoryInfo::subtypes`.
[[nodiscard]] std::size_t categorySubtypeCount(DefectCategory category) noexcept;

/// One defect found by a check (a face, edge, vertex or shell).
struct DefectElement {
    Eigen::Index index = -1;           ///< Face, vertex, shell or hole index (edges: the first vertex)
    std::array<int, 2> edge{-1, -1};   ///< Edge kinds: the two vertex indices (sorted); holes: a rim edge
    double metric = 0.0;               ///< Sort / histogram value
    std::array<double, 3> columns{};   ///< Values of `CategoryInfo::columns`
    int subtype = -1;                  ///< Index into `CategoryInfo::subtypes` (-1: none)
};

/// Which flip buttons a warning line offers next to *Show*.
enum class WarningKind : std::uint8_t {
    Note,         ///< *Show* only
    StoppedGroup, ///< An inward body holding shells, outer shell first: *Flip all* and *Flip outer only*
    Unclassified  ///< One inward shell rule O will not judge: *Flip it*
};

/// A line a check shows in its Tool panel without counting it (Inverted shells: shells it will not judge).
struct CategoryWarning {
    std::string text;        ///< Names shells in the Shells panel's numbering ("#1 and its inner shell #2 ...")
    std::vector<int> shells; ///< The shells it names (0-based, Shells-panel order): *Show* selects them
    WarningKind kind = WarningKind::Note;
};

/// Result of one Mesh Health check. Elements are sorted worst first.
struct CategoryResult {
    DefectCategory category = DefectCategory::ThinFaces;
    std::uint64_t meshGeneration = 0;
    std::vector<DefectElement> elements;
    /// Faces of each element in CSR form: element i owns `faces[faceOffsets[i] .. faceOffsets[i + 1])`.
    std::vector<std::uint32_t> faceOffsets{0};
    std::vector<Eigen::Index> faces;
    /// Related faces of each element in CSR form (intersections: the faces it crosses). Empty for
    /// categories without related faces.
    std::vector<std::uint32_t> relatedOffsets;
    std::vector<Eigen::Index> related;
    /// Edges of each element in CSR form (holes: the rim edges). Empty for other categories.
    std::vector<std::uint32_t> edgeOffsets;
    std::vector<std::array<int, 2>> edges;
    std::vector<CategoryWarning> warnings; ///< Not in the count (Inverted shells)
    std::size_t infoCount = 0; ///< Open edges (holes) or shell count (shells)
    double elapsedMs = 0.0;
    bool cancelled = false;

    [[nodiscard]] std::size_t size() const noexcept {
        return elements.size();
    }

    [[nodiscard]] bool empty() const noexcept {
        return elements.empty();
    }

    [[nodiscard]] std::span<const Eigen::Index> facesOf(std::size_t element) const noexcept {
        return {faces.data() + faceOffsets[element], faces.data() + faceOffsets[element + 1]};
    }

    [[nodiscard]] std::span<const Eigen::Index> relatedOf(std::size_t element) const noexcept {
        if (element + 1 >= relatedOffsets.size()) {
            return {};
        }
        return {related.data() + relatedOffsets[element], related.data() + relatedOffsets[element + 1]};
    }

    [[nodiscard]] std::span<const std::array<int, 2>> edgesOf(std::size_t element) const noexcept {
        if (element + 1 >= edgeOffsets.size()) {
            return {};
        }
        return {edges.data() + edgeOffsets[element], edges.data() + edgeOffsets[element + 1]};
    }

    /// Element count per sub-type (index matches `CategoryInfo::subtypes`).
    [[nodiscard]] std::array<std::size_t, 4> subtypeCounts() const noexcept;
};

/// Thresholds and tolerances used by the checks.
struct CheckParams {
    ThinFaceThresholds thinThresholds{};
    double needleEdgeRatio = settings::kDefaultNeedleEdgeRatio;
    double capAngleDeg = settings::kDefaultCapAngleDeg;
    double intersectionTolerance = settings::kDefaultIntersectionTolerance; ///< Fraction of the bbox diagonal
};

/// Runs one check on `mesh`. Pure (no GUI); polls `stopToken` and reports 0..1 through `progress`.
/// A stopped check returns a result with `cancelled == true`.
[[nodiscard]] CategoryResult runCategoryCheck(
    DefectCategory category,
    const Mesh& mesh,
    const CheckParams& params,
    std::stop_token stopToken = {},
    std::atomic<float>* progress = nullptr);

} // namespace mesheal
