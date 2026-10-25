#include "diagnostics/MeshHealth.hpp"
#include "diagnostics/MeshChecks.hpp"

#include <chrono>

namespace ogun {

namespace {

constexpr CategoryColumn kNoColumn{};

constexpr std::array<CategoryInfo, kDefectCategoryCount> kCategoryInfos = {{
    {"Open edges (holes)", "Open Edges (holes)", CategoryGroup::Topology,
     "Edges used by only one triangle: the rims of holes, so the mesh is not watertight.",
     ElementKind::Edge, "edges", true, false, false,
     {{{"Hole", ValueDimension::None}, {"Hole edges", ValueDimension::None}, {"Length", ValueDimension::Length}}},
     nullptr, ValueDimension::None, false, false,
     {nullptr, nullptr, nullptr, nullptr}},
    {"Non-manifold edges", "Non-manifold Edges", CategoryGroup::Topology,
     "Edges shared by three or more triangles (fins, internal walls, T-junctions).",
     ElementKind::Edge, "edges", true, false, false,
     {{{"Faces", ValueDimension::None}, {"Length", ValueDimension::Length}, kNoColumn}},
     nullptr, ValueDimension::None, false, false,
     {nullptr, nullptr, nullptr, nullptr}},
    {"Non-manifold vertices", "Non-manifold Vertices", CategoryGroup::Topology,
     "Two or more fans of triangles that meet at a single vertex (bow-ties).",
     ElementKind::Vertex, "vertices", true, false, false,
     {{{"Fans", ValueDimension::None}, {"Faces", ValueDimension::None}, kNoColumn}},
     nullptr, ValueDimension::None, false, false,
     {nullptr, nullptr, nullptr, nullptr}},
    {"Flipped normals", "Flipped Normals", CategoryGroup::Topology,
     "Triangles wound opposite to their neighbours (inconsistent orientation).",
     ElementKind::Face, "faces", true, false, false,
     {{{"Shell", ValueDimension::None}, kNoColumn, kNoColumn}},
     nullptr, ValueDimension::None, false, false,
     {nullptr, nullptr, nullptr, nullptr}},
    {"Shells", "Shells", CategoryGroup::Topology,
     "Separate connected pieces. Several can be intended, so this row is information only.",
     ElementKind::Shell, "shells", true, true, false,
     {{{"Faces", ValueDimension::None}, {"Area", ValueDimension::Area}, kNoColumn}},
     nullptr, ValueDimension::None, false, true,
     {nullptr, nullptr, nullptr, nullptr}},
    {"Thin / sliver faces", "Thin / Sliver Faces", CategoryGroup::Geometry,
     "Triangles with a tiny altitude, area, or a huge aspect ratio.",
     ElementKind::Face, "faces", true, false, true,
     {{{"h_min", ValueDimension::Length}, {"Area", ValueDimension::Area}, {"Aspect", ValueDimension::None}}},
     "h_min", ValueDimension::Length, true, true,
     {"Zero-area", "Needle", "Cap", "Small"}},
    {"Degenerate faces", "Degenerate Faces", CategoryGroup::Geometry,
     "Triangles with exactly zero area or a repeated vertex index.",
     ElementKind::Face, "faces", true, false, false,
     {{{"Area", ValueDimension::Area}, {"Longest edge", ValueDimension::Length}, kNoColumn}},
     nullptr, ValueDimension::None, false, true,
     {"Repeated vertex", "Zero area", nullptr, nullptr}},
    {"Duplicate faces", "Duplicate Faces", CategoryGroup::Geometry,
     "Triangles with the same three vertices as another triangle.",
     ElementKind::Face, "faces", true, false, false,
     {{{"Copy of face", ValueDimension::None}, kNoColumn, kNoColumn}},
     nullptr, ValueDimension::None, false, true,
     {"Same winding", "Opposite winding", nullptr, nullptr}},
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
    case DefectCategory::ThinFaces:
        result = checkThinFaces(mesh, params, stopToken);
        break;
    case DefectCategory::Degenerate:
        result = checkDegenerateFaces(mesh, params, stopToken);
        break;
    case DefectCategory::Duplicates:
        result = checkDuplicateFaces(mesh, stopToken);
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
