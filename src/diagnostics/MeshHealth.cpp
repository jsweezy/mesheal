#include "diagnostics/MeshHealth.hpp"
#include "diagnostics/MeshChecks.hpp"
#include "diagnostics/SelfIntersections.hpp"
#include "diagnostics/ShellRoles.hpp"

#include <chrono>

namespace ogun {

namespace {

constexpr CategoryColumn kNoColumn{};

constexpr std::array<CategoryInfo, kDefectCategoryCount> kCategoryInfos = {{
    {"Holes (open edges)", "Holes (open edges)", CategoryGroup::Topology,
     "Holes: loops of edges used by only one triangle, so the mesh is not watertight.",
     ElementKind::Hole, "holes", true, false, true,
     {{{"Edges", ValueDimension::None}, {"Size", ValueDimension::Length}, {"Gap width", ValueDimension::Length}}},
     nullptr, ValueDimension::None, false, false,
     {"Flat", "Curved", "Crack", "Open chain"}},
    {"Non-manifold edges", "Non-manifold Edges", CategoryGroup::Topology,
     "Edges shared by three or more triangles (solids touching along an edge, fins, internal walls).",
     ElementKind::Edge, "edges", true, false, true,
     {{{"Faces", ValueDimension::None}, {"Length", ValueDimension::Length}, kNoColumn}},
     nullptr, ValueDimension::None, false, false,
     {"Touching", "Fin / other", nullptr, nullptr}},
    {"Non-manifold vertices", "Non-manifold Vertices", CategoryGroup::Topology,
     "Two or more fans of triangles that meet at a single vertex (bow-ties).",
     ElementKind::Vertex, "vertices", true, false, true,
     {{{"Fans", ValueDimension::None}, {"Faces", ValueDimension::None}, kNoColumn}},
     nullptr, ValueDimension::None, false, false,
     {nullptr, nullptr, nullptr, nullptr}},
    {"Flipped normals", "Flipped Normals", CategoryGroup::Topology,
     "Triangles wound opposite to their neighbours (inconsistent orientation).",
     ElementKind::Face, "faces", true, false, true,
     {{{"Shell", ValueDimension::None}, kNoColumn, kNoColumn}},
     nullptr, ValueDimension::None, false, false,
     {nullptr, nullptr, nullptr, nullptr}},
    {"Shells", "Shells", CategoryGroup::Topology,
     "Separate connected pieces. Several can be intended, so this row is information only.",
     ElementKind::Shell, "shells", true, true, false,
     {{{"Faces", ValueDimension::None}, {"Area", ValueDimension::Area}, kNoColumn}},
     nullptr, ValueDimension::None, false, true,
     {nullptr, nullptr, nullptr, nullptr}},
    {"Intersecting faces", "Intersecting Faces", CategoryGroup::Geometry,
     "Triangles that pass through other triangles (or overlap them while lying in the same plane).",
     ElementKind::Face, "faces", false, false, true,
     {{{"Crosses", ValueDimension::None}, {"First partner", ValueDimension::None}, kNoColumn}},
     "Faces crossed", ValueDimension::None, false, false,
     {"Crossing", "Coplanar overlap", nullptr, nullptr}},
    {"Thin / sliver faces", "Thin / Sliver Faces", CategoryGroup::Geometry,
     "Triangles stretched more than the aspect limit (default 100 : 1), or with no reliable normal.",
     ElementKind::Face, "faces", true, false, true,
     {{{"h_min", ValueDimension::Length}, {"Area", ValueDimension::Area}, {"Aspect", ValueDimension::None}}},
     "h_min", ValueDimension::Length, true, true,
     {"Zero-area", "Needle", "Cap", "Small"}},
    {"Degenerate faces", "Degenerate Faces", CategoryGroup::Geometry,
     "Triangles with a repeated vertex index or no reliable normal (flat, or below rounding at their position).",
     ElementKind::Face, "faces", true, false, true,
     {{{"Area", ValueDimension::Area}, {"Longest edge", ValueDimension::Length}, kNoColumn}},
     nullptr, ValueDimension::None, false, true,
     {"Repeated vertex", "Zero area", nullptr, nullptr}},
    {"Duplicate faces", "Duplicate Faces", CategoryGroup::Geometry,
     "Triangles with the same three vertices as another triangle.",
     ElementKind::Face, "faces", true, false, true,
     {{{"Copy of face", ValueDimension::None}, kNoColumn, kNoColumn}},
     nullptr, ValueDimension::None, false, true,
     {"Same winding", "Opposite winding", nullptr, nullptr}},
    {"Inverted shells", "Inverted Shells", CategoryGroup::Topology,
     "Closed shells wound inside out: facing inward with nothing around them and nothing inside them. "
     "Cavities (inward shells inside a solid) are correct and not counted.",
     ElementKind::Shell, "shells", true, false, true,
     {{{"Faces", ValueDimension::None}, {"Volume", ValueDimension::Volume}, kNoColumn}},
     nullptr, ValueDimension::None, false, false,
     {nullptr, nullptr, nullptr, nullptr}},
}};

} // namespace

const CategoryInfo& categoryInfo(DefectCategory category) noexcept {
    return kCategoryInfos[categoryIndex(category)];
}

std::size_t categorySubtypeCount(DefectCategory category) noexcept {
    std::size_t count = 0;
    for (const char* label : categoryInfo(category).subtypes) {
        if (label == nullptr) {
            break;
        }
        ++count;
    }
    return count;
}

std::array<std::size_t, 4> CategoryResult::subtypeCounts() const noexcept {
    std::array<std::size_t, 4> counts{};
    for (const DefectElement& element : elements) {
        if (element.subtype >= 0 && element.subtype < 4) {
            ++counts[static_cast<std::size_t>(element.subtype)];
        }
    }
    return counts;
}

CategoryResult runCategoryCheck(
    DefectCategory category,
    const Mesh& mesh,
    const CheckParams& params,
    std::stop_token stopToken,
    std::atomic<float>* progress) {
    if (progress != nullptr) {
        progress->store(0.0f);
    }
    const auto start = std::chrono::steady_clock::now();

    CategoryResult result;
    switch (category) {
    case DefectCategory::OpenEdges:
        result = checkOpenEdges(mesh, stopToken);
        break;
    case DefectCategory::NonManifoldEdges:
        result = checkNonManifoldEdges(mesh, stopToken);
        break;
    case DefectCategory::NonManifoldVertices:
        result = checkNonManifoldVertices(mesh, stopToken);
        break;
    case DefectCategory::FlippedNormals:
        result = checkFlippedNormals(mesh, stopToken);
        break;
    case DefectCategory::Shells:
        result = checkShells(mesh, stopToken);
        break;
    case DefectCategory::Intersections:
        result = checkSelfIntersections(mesh, params, stopToken, progress);
        break;
    case DefectCategory::ThinFaces:
        result = checkThinFaces(mesh, params, stopToken);
        break;
    case DefectCategory::Degenerate:
        result = checkDegenerateFaces(mesh, stopToken);
        break;
    case DefectCategory::Duplicates:
        result = checkDuplicateFaces(mesh, stopToken);
        break;
    case DefectCategory::InvertedShells:
        result = checkInvertedShells(mesh, params, stopToken, progress);
        break;
    }

    result.category = category;
    result.elapsedMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    if (progress != nullptr) {
        progress->store(1.0f);
    }
    return result;
}

} // namespace ogun
