#include "repair/SurfaceRebuild.hpp"

#include "core/Constants.hpp"
#include "core/Settings.hpp"
#include "diagnostics/MeshChecks.hpp"
#include "diagnostics/SelfIntersections.hpp"
#include "diagnostics/ShellRelations.hpp"
#include "diagnostics/ThinFaceChecker.hpp"
#include "mesh/CgalMesh.hpp"
#include "repair/IntersectionHealer.hpp"
#include "repair/RepairMetrics.hpp"
#include "repair/SliverHealer.hpp"
#include "repair/TopologyCleanup.hpp"

#include <Eigen/Geometry>

#include <CGAL/Labeled_mesh_domain_3.h>
#include <CGAL/Mesh_complex_3_in_triangulation_3.h>
#include <CGAL/Mesh_criteria_3.h>
#include <CGAL/Mesh_triangulation_3.h>
#include <CGAL/Polygon_mesh_processing/autorefinement.h>
#include <CGAL/Polygon_mesh_processing/orientation.h>
#include <CGAL/Polygon_mesh_processing/remesh.h>
#include <CGAL/Polygon_mesh_processing/repair_polygon_soup.h>
#include <CGAL/Polygon_mesh_processing/tangential_relaxation.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Bounded_normal_change_filter.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Face_count_stop_predicate.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/GarlandHeckbert_policies.h>
#include <CGAL/Surface_mesh_simplification/edge_collapse.h>
#include <CGAL/alpha_wrap_3.h>
#include <CGAL/facets_in_complex_3_to_triangle_mesh.h>
#include <CGAL/make_mesh_3.h>

#include <igl/fast_winding_number.h>
#include <igl/marching_cubes.h>
#include <igl/voxel_grid.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <exception>
#include <functional>
#include <numeric>
#include <unordered_map>
#include <utility>
#include <vector>

namespace ogun {

namespace PMP = CGAL::Polygon_mesh_processing;

namespace {

using Soup = std::vector<std::array<std::size_t, 3>>;

void setStatus(std::string* statusOut, std::string message) {
    if (statusOut != nullptr) {
        *statusOut = std::move(message);
    }
}

std::vector<CgalPoint3> toPoints(const Mesh& mesh) {
    std::vector<CgalPoint3> points;
    points.reserve(static_cast<std::size_t>(mesh.numVertices()));
    for (Eigen::Index i = 0; i < mesh.numVertices(); ++i) {
        points.emplace_back(mesh.V(i, 0), mesh.V(i, 1), mesh.V(i, 2));
    }
    return points;
}

Soup toSoup(const Mesh& mesh) {
    Soup soup;
    soup.reserve(static_cast<std::size_t>(mesh.numFaces()));
    for (Eigen::Index f = 0; f < mesh.numFaces(); ++f) {
        soup.push_back({static_cast<std::size_t>(mesh.F(f, 0)),
                        static_cast<std::size_t>(mesh.F(f, 1)),
                        static_cast<std::size_t>(mesh.F(f, 2))});
    }
    return soup;
}

Mesh fromSoup(const std::vector<CgalPoint3>& points, const Soup& soup, Unit unit) {
    Mesh mesh;
    mesh.fileUnit = unit;
    mesh.V.resize(static_cast<Eigen::Index>(points.size()), 3);
    for (std::size_t i = 0; i < points.size(); ++i) {
        mesh.V.row(static_cast<Eigen::Index>(i)) << points[i].x(), points[i].y(), points[i].z();
    }
    mesh.F.resize(static_cast<Eigen::Index>(soup.size()), 3);
    for (std::size_t t = 0; t < soup.size(); ++t) {
        for (std::size_t c = 0; c < 3; ++c) {
            mesh.F(static_cast<Eigen::Index>(t), static_cast<Eigen::Index>(c)) = static_cast<int>(soup[t][c]);
        }
    }
    mesh.recomputeNormals();
    return mesh;
}

/// Drops vertices no face uses (the rebuild steps leave some behind).
Mesh compacted(const Mesh& mesh) {
    std::vector<Eigen::Index> all(static_cast<std::size_t>(mesh.numFaces()));
    std::iota(all.begin(), all.end(), Eigen::Index{0});
    return extractFaces(mesh, all);
}

/// Groups the triangles of an autorefined soup into patches: triangles sharing an edge that only
/// they use. Crossing curves (edges used by 3+ triangles) and open edges bound the patches.
std::vector<std::size_t> soupPatches(const Soup& soup, std::size_t& numPatches) {
    std::vector<std::size_t> parent(soup.size());
    std::iota(parent.begin(), parent.end(), std::size_t{0});
    const auto find = [&](std::size_t x) {
        while (parent[x] != x) {
            parent[x] = parent[parent[x]];
            x = parent[x];
        }
        return x;
    };

    std::unordered_map<std::uint64_t, std::vector<std::size_t>> edgeFaces;
    edgeFaces.reserve(soup.size() * 2);
    for (std::size_t t = 0; t < soup.size(); ++t) {
        for (std::size_t c = 0; c < 3; ++c) {
            const std::size_t a = soup[t][c];
            const std::size_t b = soup[t][(c + 1) % 3];
            const std::uint64_t key = (static_cast<std::uint64_t>(std::min(a, b)) << 32) | std::max(a, b);
            edgeFaces[key].push_back(t);
        }
    }
    for (const auto& [key, faces] : edgeFaces) {
        if (faces.size() == 2) {
            parent[find(faces[0])] = find(faces[1]);
        }
    }

    std::vector<std::size_t> patch(soup.size());
    std::unordered_map<std::size_t, std::size_t> rootToPatch;
    for (std::size_t t = 0; t < soup.size(); ++t) {
        const auto [it, inserted] = rootToPatch.try_emplace(find(t), rootToPatch.size());
        patch[t] = it->second;
    }
    numPatches = rootToPatch.size();
    return patch;
}

/// Lets a stop token end an alpha wrap: CGAL's wrapper asks its visitor `go_further()` before every step
/// of its flood fill (`CGAL/Alpha_wrap_3/internal/Alpha_wrap_3.h`, `alpha_flood_fill`). After a stop
/// the wrapper still makes its partial wrap manifold and extracts it (cheap); the caller drops it.
struct AlphaWrapStopVisitor : CGAL::Alpha_wraps_3::internal::Wrapping_default_visitor {
    std::stop_token stopToken;
    const std::function<void(std::size_t)>* onStep = nullptr;
    std::size_t steps = 0;

    template <typename Wrapper>
    bool go_further(const Wrapper&) {
        if (onStep != nullptr && *onStep) {
            (*onStep)(steps);
        }
        ++steps;
        return !stopToken.stop_requested();
    }
};

/// Applies `passes` iterations of Taubin (\lambda / \mu) umbrella Laplacian smoothing in-place on `mesh`.
/// Running this on the Marching Cubes mesh before converting to `CgalSurfaceMesh` ensures that
/// `PMP::isotropic_remeshing`'s internal AABB surface projection projects onto the de-terraced
/// surface rather than snapping vertices back onto voxel stair-steps.
bool applyTaubinSmoothing(Mesh& mesh, int passes, std::stop_token stopToken) {
    if (passes <= 0 || mesh.empty()) {
        return true;
    }
    const Eigen::Index numV = mesh.numVertices();
    const Eigen::Index numF = mesh.numFaces();
    std::vector<std::array<Eigen::Index, 2>> edges;
    edges.reserve(static_cast<std::size_t>(numF) * 3);
    for (Eigen::Index f = 0; f < numF; ++f) {
        for (Eigen::Index c = 0; c < 3; ++c) {
            const Eigen::Index a = mesh.F(f, c);
            const Eigen::Index b = mesh.F(f, (c + 1) % 3);
            if (a != b) {
                edges.push_back({std::min(a, b), std::max(a, b)});
            }
        }
    }
    std::sort(edges.begin(), edges.end());
    edges.erase(std::unique(edges.begin(), edges.end()), edges.end());

    std::vector<int> degree(static_cast<std::size_t>(numV), 0);
    for (const auto& [u, v] : edges) {
        ++degree[static_cast<std::size_t>(u)];
        ++degree[static_cast<std::size_t>(v)];
    }

    Eigen::MatrixXd delta(numV, 3);
    const auto laplacianStep = [&](double weight) {
        delta.setZero();
        for (const auto& [u, v] : edges) {
            const Eigen::RowVector3d diff = mesh.V.row(v) - mesh.V.row(u);
            delta.row(u) += diff;
            delta.row(v) -= diff;
        }
        for (Eigen::Index i = 0; i < numV; ++i) {
            const int deg = degree[static_cast<std::size_t>(i)];
            if (deg > 0) {
                mesh.V.row(i) += (weight / static_cast<double>(deg)) * delta.row(i);
            }
        }
    };

    for (int pass = 0; pass < passes; ++pass) {
        if (stopToken.stop_requested()) {
            return false;
        }
        laplacianStep(settings::kDefaultTaubinLambda);
        laplacianStep(settings::kDefaultTaubinMu);
    }
    return !stopToken.stop_requested();
}

/// Post-rebuild cleanup: removes degenerate/duplicate faces, strips microscopic noise shells,
/// and heals any residual thin/sliver faces or local self-intersection islands.
Mesh polishRebuiltMesh(Mesh result, std::stop_token stopToken = {}) {
    if (result.empty() || stopToken.stop_requested()) {
        return result;
    }
    result = removeRepeatedVertexFaces(result).mesh;
    result = removeDuplicateFaces(result).mesh;
    if (result.empty() || stopToken.stop_requested()) {
        return result;
    }

    const std::vector<std::vector<Eigen::Index>> shells = shellsLargestFirst(result);
    if (shells.size() > 1) {
        const std::vector<ShellFacts> facts = describeShells(result, shells);
        const double modelDiag = result.boundingBox().diagonal();
        std::vector<Eigen::Index> keepFaces;
        keepFaces.reserve(static_cast<std::size_t>(result.numFaces()));
        bool droppedAny = false;
        for (std::size_t s = 0; s < shells.size(); ++s) {
            const bool isTinyNoise = s > 0 &&
                (shells[s].size() < 12 ||
                 facts[s].box.diagonal() < settings::kDefaultFixAllTinyShellFraction * modelDiag);
            if (isTinyNoise) {
                droppedAny = true;
            } else {
                keepFaces.insert(keepFaces.end(), shells[s].begin(), shells[s].end());
            }
        }
        if (droppedAny && !keepFaces.empty()) {
            std::sort(keepFaces.begin(), keepFaces.end());
            result = compacted(extractFaces(result, keepFaces));
        }
    }
    if (stopToken.stop_requested()) {
        return result;
    }

    const auto isClosedManifold = [](const Mesh& m) {
        return !m.empty() && checkOpenEdges(m).empty() && checkNonManifoldEdges(m).empty();
    };

    if (!ThinFaceChecker::check(result).defects.empty()) {
        SliverHealOptions opts;
        opts.stopToken = stopToken;
        RepairResults healed = SliverHealer::healAllThinFaces(result, {}, {}, opts);
        if (healed.success && !healed.cancelled && isClosedManifold(healed.mesh)) {
            result = std::move(healed.mesh);
        }
    }
    if (stopToken.stop_requested()) {
        return result;
    }

    const std::size_t xingsBefore = checkSelfIntersections(result, CheckParams{}, stopToken).size();
    if (xingsBefore > 0 && !stopToken.stop_requested()) {
        IslandBatchResult islands = healIntersectionIslands(result, CheckParams{}, {}, {}, {}, stopToken);
        if (islands.success && islands.islandsAfter < islands.islandsBefore && isClosedManifold(islands.mesh)) {
            result = std::move(islands.mesh);
        }
        if (!stopToken.stop_requested() && !checkSelfIntersections(result, CheckParams{}, stopToken).empty()) {
            Mesh united = selfUnion(result);
            if (isClosedManifold(united) && checkSelfIntersections(united, CheckParams{}, stopToken).empty()) {
                result = std::move(united);
            }
        }
        if (!stopToken.stop_requested() && !ThinFaceChecker::check(result).defects.empty()) {
            SliverHealOptions opts;
            opts.stopToken = stopToken;
            RepairResults healed = SliverHealer::healAllThinFaces(result, {}, {}, opts);
            if (healed.success && !healed.cancelled && isClosedManifold(healed.mesh) &&
                checkSelfIntersections(healed.mesh, CheckParams{}, stopToken).empty()) {
                result = std::move(healed.mesh);
            }
        }
    }

    result.recomputeNormals();
    return result;
}

} // namespace

Mesh selfUnion(const Mesh& mesh, std::string* statusOut) {
    if (mesh.empty()) {
        setStatus(statusOut, "The mesh is empty.");
        return {};
    }
    std::vector<CgalPoint3> points = toPoints(mesh);
    Soup soup = toSoup(mesh);
    bool clean = false;
    try {
        clean = PMP::autorefine_triangle_soup(points, soup, CGAL::parameters::apply_iterative_snap_rounding(true));
    } catch (const std::exception& e) {
        setStatus(statusOut, std::string("Autorefinement failed: ") + e.what());
        return {};
    }

    // Normalize to a unit box around the origin: the fast winding number works in float
    const Eigen::RowVector3d center = 0.5 * (mesh.V.colwise().minCoeff() + mesh.V.colwise().maxCoeff());
    const double diag = mesh.boundingBox().diagonal();
    const double scale = diag > 0.0 ? 1.0 / diag : 1.0;
    const Eigen::MatrixXf V = ((mesh.V.rowwise() - center) * scale).cast<float>();
    igl::FastWindingNumberBVH bvh;
    igl::fast_winding_number(V, mesh.F, 2, bvh);

    // Classify each patch by its largest triangle: query just in front of and behind it
    std::size_t numPatches = 0;
    const std::vector<std::size_t> patch = soupPatches(soup, numPatches);
    std::vector<std::size_t> representative(numPatches, soup.size());
    std::vector<double> bestArea(numPatches, -1.0);
    std::vector<Eigen::Vector3d> normal(soup.size());
    std::vector<double> inradius(soup.size());
    for (std::size_t t = 0; t < soup.size(); ++t) {
        const auto p = [&](std::size_t c) {
            const CgalPoint3& q = points[soup[t][c]];
            return Eigen::Vector3d((q.x() - center.x()) * scale, (q.y() - center.y()) * scale, (q.z() - center.z()) * scale);
        };
        const Eigen::Vector3d a = p(0), b = p(1), c = p(2);
        const Eigen::Vector3d n = (b - a).cross(c - a);
        const double area2 = n.norm();
        normal[t] = area2 > 0.0 ? Eigen::Vector3d(n / area2) : Eigen::Vector3d::Zero();
        const double perimeter = (b - a).norm() + (c - b).norm() + (a - c).norm();
        inradius[t] = perimeter > 0.0 ? area2 / perimeter : 0.0;
        if (area2 > bestArea[patch[t]]) {
            bestArea[patch[t]] = area2;
            representative[patch[t]] = t;
        }
    }

    constexpr double kMaxProbe = 1.0e-3;  // probe distance cap, fraction of the diagonal
    Eigen::MatrixXf queries(static_cast<Eigen::Index>(2 * numPatches), 3);
    for (std::size_t pi = 0; pi < numPatches; ++pi) {
        const std::size_t t = representative[pi];
        Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
        for (std::size_t c = 0; c < 3; ++c) {
            const CgalPoint3& q = points[soup[t][c]];
            centroid += Eigen::Vector3d((q.x() - center.x()) * scale, (q.y() - center.y()) * scale, (q.z() - center.z()) * scale);
        }
        centroid /= 3.0;
        const double h = std::min(kMaxProbe, 0.5 * inradius[t]);
        queries.row(static_cast<Eigen::Index>(2 * pi)) = (centroid + h * normal[t]).cast<float>().transpose();
        queries.row(static_cast<Eigen::Index>(2 * pi + 1)) = (centroid - h * normal[t]).cast<float>().transpose();
    }
    Eigen::VectorXf w;
    igl::fast_winding_number(bvh, 2.0f, queries, w);

    // Keep a patch when it separates inside (w >= 0.5) from outside; orient it outward
    Soup kept;
    kept.reserve(soup.size());
    for (std::size_t t = 0; t < soup.size(); ++t) {
        const auto pi = static_cast<Eigen::Index>(patch[t]);
        const bool frontInside = w(2 * pi) >= 0.5f;
        const bool backInside = w(2 * pi + 1) >= 0.5f;
        if (backInside && !frontInside) {
            kept.push_back(soup[t]);
        } else if (frontInside && !backInside) {
            kept.push_back({soup[t][0], soup[t][2], soup[t][1]});
        }
    }
    PMP::repair_polygon_soup(points, kept);
    Mesh result = fromSoup(points, kept, mesh.fileUnit);
    setStatus(statusOut, std::string(clean ? "" : "Snap rounding left some crossings. ") + "Kept " +
                             std::to_string(kept.size()) + " of " + std::to_string(soup.size()) + " refined triangles.");
    return result;
}

Mesh alphaWrap(const Mesh& mesh, double alpha, double offset, std::string* statusOut, std::stop_token stopToken,
               const std::function<void(std::size_t)>& onStep) {
    if (mesh.empty() || !(alpha > 0.0) || !(offset > 0.0)) {
        setStatus(statusOut, "Alpha wrap needs a mesh and positive alpha / offset.");
        return {};
    }
    const std::vector<CgalPoint3> points = toPoints(mesh);
    const Soup soup = toSoup(mesh);
    CgalSurfaceMesh wrap;
    AlphaWrapStopVisitor visitor;
    visitor.stopToken = stopToken;
    visitor.onStep = &onStep;
    try {
        CGAL::alpha_wrap_3(points, soup, alpha, offset, wrap, CGAL::parameters::visitor(std::ref(visitor)));
    } catch (const std::exception& e) {
        setStatus(statusOut, std::string("Alpha wrap failed: ") + e.what());
        return {};
    }
    if (stopToken.stop_requested()) {
        setStatus(statusOut, "Cancelled.");
        return {};
    }
    std::array<char, 96> text{};
    std::snprintf(text.data(), text.size(), "Wrapped with alpha %.3g and offset %.3g.", alpha, offset);
    setStatus(statusOut, text.data());
    return fromCgalSurfaceMesh(wrap, mesh.fileUnit);
}

Mesh windingNumberRebuild(const Mesh& mesh, int resolution, std::string* statusOut) {
    if (mesh.empty() || resolution < 4) {
        setStatus(statusOut, "Winding-number rebuild needs a mesh and a grid of at least 4 cells.");
        return {};
    }
    // Same normalization as selfUnion: float winding numbers
    const Eigen::RowVector3d center = 0.5 * (mesh.V.colwise().minCoeff() + mesh.V.colwise().maxCoeff());
    const double diag = mesh.boundingBox().diagonal();
    const double scale = diag > 0.0 ? 1.0 / diag : 1.0;
    const Eigen::MatrixXd Vn = (mesh.V.rowwise() - center) * scale;

    Eigen::MatrixXd gridPoints;
    Eigen::RowVector3i side;
    const Eigen::AlignedBox3d box(Vn.colwise().minCoeff().transpose(), Vn.colwise().maxCoeff().transpose());
    igl::voxel_grid(box, resolution, 2, gridPoints, side);
    // Shift the samples off-lattice: CAD planes on grid nodes give w = 0.5 exactly and zero-area triangles
    const double cell = (gridPoints.row(1) - gridPoints.row(0)).norm();
    gridPoints.rowwise() += cell * Eigen::RowVector3d(0.3183, 0.2718, 0.1414);

    igl::FastWindingNumberBVH bvh;
    const Eigen::MatrixXf Vf = Vn.cast<float>();
    igl::fast_winding_number(Vf, mesh.F, 2, bvh);
    Eigen::VectorXf w;
    igl::fast_winding_number(bvh, 2.0f, gridPoints.cast<float>(), w);

    Mesh result;
    result.fileUnit = mesh.fileUnit;
    const Eigen::VectorXd field = w.cast<double>();
    igl::marching_cubes(field, gridPoints, static_cast<unsigned>(side(0)), static_cast<unsigned>(side(1)),
                        static_cast<unsigned>(side(2)), 0.5, result.V, result.F);
    if (result.empty()) {
        setStatus(statusOut, "The winding-number field has no inside at this resolution.");
        return {};
    }
    result.V = (result.V / scale).rowwise() + center;
    // Marching cubes winds the other way round for a field that is high inside
    if (signedVolume(result) < 0.0) {
        result.F.col(1).swap(result.F.col(2));
    }
    result.recomputeNormals();
    setStatus(statusOut, "Grid " + std::to_string(side(0)) + " x " + std::to_string(side(1)) + " x " +
                             std::to_string(side(2)) + ".");
    return compacted(result);
}

std::array<int, 3> estimateVoxelGridDimensions(const BoundingBox& bbox, double targetEdgeLength) noexcept {
    const Eigen::Vector3d span = bbox.extents();
    Eigen::Index maxAxis = 0;
    const double maxSpan = span.maxCoeff(&maxAxis);
    if (!(maxSpan > 0.0) || !(targetEdgeLength > 0.0)) {
        return {settings::kDefaultRemeshGridCells,
                settings::kDefaultRemeshGridCells,
                settings::kDefaultRemeshGridCells};
    }
    constexpr int kPadCount = 2;
    const int inS = std::clamp(static_cast<int>(std::ceil(maxSpan / targetEdgeLength)),
                               settings::kMinRemeshGridCells,
                               settings::kMaxRemeshGridCells);
    const double innerCells = static_cast<double>(inS - 2 * kPadCount);
    std::array<int, 3> side{};
    for (int axis = 0; axis < 3; ++axis) {
        if (axis == static_cast<int>(maxAxis)) {
            side[static_cast<std::size_t>(axis)] = inS;
        } else {
            side[static_cast<std::size_t>(axis)] =
                static_cast<int>(std::ceil(innerCells * span(axis) / maxSpan)) + 2 * kPadCount;
        }
    }
    return side;
}

Mesh voxelHealAndRemesh(const Mesh& mesh, double targetEdgeLength, int smoothingPasses,
                        std::string* statusOut, std::stop_token stopToken) {
    if (mesh.empty() || !(targetEdgeLength > 0.0)) {
        setStatus(statusOut, "Voxel heal & remesh needs a mesh and a positive target mesh size.");
        return {};
    }
    const BoundingBox bbox = mesh.boundingBox();
    const double maxSpan = bbox.extents().maxCoeff();
    if (!(maxSpan > 0.0)) {
        setStatus(statusOut, "The mesh bounding box has zero span.");
        return {};
    }
    const int clampedPasses = std::clamp(smoothingPasses,
                                         settings::kMinRemeshSmoothingPasses,
                                         settings::kMaxRemeshSmoothingPasses);
    const int resolution = std::clamp(static_cast<int>(std::ceil(maxSpan / targetEdgeLength)),
                                      settings::kMinRemeshGridCells,
                                      settings::kMaxRemeshGridCells);

    // Stage 1: Normalized fast winding number on an off-lattice voxel grid + Marching Cubes
    const Eigen::RowVector3d center = 0.5 * (mesh.V.colwise().minCoeff() + mesh.V.colwise().maxCoeff());
    const double diag = bbox.diagonal();
    const double scale = diag > 0.0 ? 1.0 / diag : 1.0;
    const Eigen::MatrixXd Vn = (mesh.V.rowwise() - center) * scale;

    Eigen::MatrixXd gridPoints;
    Eigen::RowVector3i side;
    const Eigen::AlignedBox3d box(Vn.colwise().minCoeff().transpose(), Vn.colwise().maxCoeff().transpose());
    igl::voxel_grid(box, resolution, 2, gridPoints, side);
    const double cell = (gridPoints.row(1) - gridPoints.row(0)).norm();
    gridPoints.rowwise() += cell * Eigen::RowVector3d(0.3183, 0.2718, 0.1414);

    if (stopToken.stop_requested()) {
        setStatus(statusOut, "Cancelled.");
        return {};
    }

    igl::FastWindingNumberBVH bvh;
    const Eigen::MatrixXf Vf = Vn.cast<float>();
    igl::fast_winding_number(Vf, mesh.F, 2, bvh);
    if (stopToken.stop_requested()) {
        setStatus(statusOut, "Cancelled.");
        return {};
    }

    Eigen::VectorXf w;
    igl::fast_winding_number(bvh, 2.0f, gridPoints.cast<float>(), w);
    if (stopToken.stop_requested()) {
        setStatus(statusOut, "Cancelled.");
        return {};
    }

    Mesh mc;
    mc.fileUnit = mesh.fileUnit;
    const Eigen::VectorXd field = w.cast<double>();
    igl::marching_cubes(field, gridPoints, static_cast<unsigned>(side(0)), static_cast<unsigned>(side(1)),
                        static_cast<unsigned>(side(2)), 0.5, mc.V, mc.F);
    if (mc.empty()) {
        setStatus(statusOut, "The winding-number field has no enclosed volume at this resolution.");
        return {};
    }
    mc.V = (mc.V / scale).rowwise() + center;
    if (signedVolume(mc) < 0.0) {
        mc.F.col(1).swap(mc.F.col(2));
    }
    mc = compacted(mc);

    // Stage 2: Volume-preserving Taubin (\lambda/\mu) de-terracing prior to CGAL AABB projection
    if (!applyTaubinSmoothing(mc, clampedPasses, stopToken)) {
        setStatus(statusOut, "Cancelled.");
        return {};
    }

    // Stage 3: CGAL isotropic remeshing at the target edge length
    std::vector<CgalPoint3> points = toPoints(mc);
    Soup soup = toSoup(mc);
    PMP::repair_polygon_soup(points, soup);
    Mesh repairedMc = fromSoup(points, soup, mesh.fileUnit);

    CgalSurfaceMesh surface;
    if (!toCgalSurfaceMesh(repairedMc, surface)) {
        setStatus(statusOut, "Failed to convert Marching Cubes surface for isotropic remeshing.");
        return {};
    }
    if (stopToken.stop_requested()) {
        setStatus(statusOut, "Cancelled.");
        return {};
    }

    const double voxelCellSize = maxSpan / static_cast<double>(std::max(1, resolution - 4));
    const double effectiveEdgeLength = std::max(targetEdgeLength, 0.5 * voxelCellSize);
    const unsigned int remeshIters = static_cast<unsigned int>(std::clamp(clampedPasses, 1, 3));
    const unsigned int relaxSteps = clampedPasses > 0 ? 1u : 0u;
    try {
        PMP::isotropic_remeshing(
            faces(surface),
            effectiveEdgeLength,
            surface,
            CGAL::parameters::number_of_iterations(remeshIters)
                .number_of_relaxation_steps(relaxSteps)
                .protect_constraints(false));
    } catch (const std::exception& e) {
        setStatus(statusOut, std::string("Isotropic remeshing failed: ") + e.what());
        return {};
    }
    if (stopToken.stop_requested()) {
        setStatus(statusOut, "Cancelled.");
        return {};
    }

    if (CGAL::is_closed(surface)) {
        PMP::orient_to_bound_a_volume(surface);
    }
    Mesh result = fromCgalSurfaceMesh(surface, mesh.fileUnit);
    if (signedVolume(result) < 0.0) {
        result.F.col(1).swap(result.F.col(2));
        result.recomputeNormals();
    }
    result = polishRebuiltMesh(std::move(result), stopToken);
    if (stopToken.stop_requested()) {
        setStatus(statusOut, "Cancelled.");
        return {};
    }
    std::array<char, 160> text{};
    std::snprintf(text.data(), text.size(),
                  "Grid %d x %d x %d, Taubin %d pass%s, remeshed at target %.3g.",
                  side(0), side(1), side(2), clampedPasses, clampedPasses == 1 ? "" : "es", effectiveEdgeLength);
    setStatus(statusOut, text.data());
    return result;
}

Mesh precisionIsosurfaceRemesh(const Mesh& mesh, double targetEdgeLength, double chordRatio,
                               int smoothingPasses, std::string* statusOut, std::stop_token stopToken) {
    if (mesh.empty() || !(targetEdgeLength > 0.0)) {
        setStatus(statusOut, "Precision isosurface needs a mesh and a positive target mesh size.");
        return {};
    }
    const BoundingBox bbox = mesh.boundingBox();
    const Eigen::Vector3d extents = bbox.extents();
    const double maxSpan = extents.maxCoeff();
    const double diag = bbox.diagonal();
    if (!(maxSpan > 0.0) || !(diag > 0.0)) {
        setStatus(statusOut, "The mesh bounding box has zero span.");
        return {};
    }
    const double clampedChordRatio = std::clamp(chordRatio,
                                                settings::kMinIsosurfaceChordRatio,
                                                settings::kMaxIsosurfaceChordRatio);
    const int clampedPasses = std::clamp(smoothingPasses,
                                         settings::kMinRemeshSmoothingPasses,
                                         settings::kMaxRemeshSmoothingPasses);
    const double minAllowedEdgeLength = maxSpan / static_cast<double>(settings::kMaxRemeshGridCells);
    const double effectiveEdgeLength = std::max(targetEdgeLength, minAllowedEdgeLength);
    const double chordDistance = std::max(effectiveEdgeLength * clampedChordRatio, 0.25 * minAllowedEdgeLength);
    const double facetRadius = effectiveEdgeLength / kSqrt3;

    // Normalize input to a unit diagonal centered at the origin for float BVH evaluation
    const Eigen::RowVector3d center = 0.5 * (mesh.V.colwise().minCoeff() + mesh.V.colwise().maxCoeff());
    const double scale = 1.0 / diag;
    const Eigen::MatrixXf Vf = ((mesh.V.rowwise() - center) * scale).cast<float>();

    if (stopToken.stop_requested()) {
        setStatus(statusOut, "Cancelled.");
        return {};
    }

    igl::FastWindingNumberBVH bvh;
    igl::fast_winding_number(Vf, mesh.F, 2, bvh);
    if (stopToken.stop_requested()) {
        setStatus(statusOut, "Cancelled.");
        return {};
    }

    // Pre-probe across sampled input triangles and a coarse interior lattice to:
    // 1. Detect open sheets / zero-volume inputs where w < 0.5 everywhere (avoiding CGAL's infinite seed loop)
    // 2. Collect bracket segments crossing w = 0.5 to seed CGAL::make_mesh_3 deterministically
    constexpr std::size_t kMaxTriangleProbes = 64;
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> triBrackets;
    triBrackets.reserve(kMaxTriangleProbes);
    const std::size_t numFaces = static_cast<std::size_t>(mesh.numFaces());
    const std::size_t stride = std::max<std::size_t>(1, numFaces / kMaxTriangleProbes);
    for (std::size_t f = 0; f < numFaces && triBrackets.size() < kMaxTriangleProbes; f += stride) {
        const Eigen::Vector3d a = mesh.V.row(mesh.F(static_cast<Eigen::Index>(f), 0)).transpose();
        const Eigen::Vector3d b = mesh.V.row(mesh.F(static_cast<Eigen::Index>(f), 1)).transpose();
        const Eigen::Vector3d c = mesh.V.row(mesh.F(static_cast<Eigen::Index>(f), 2)).transpose();
        const Eigen::Vector3d n = (b - a).cross(c - a);
        const double area2 = n.norm();
        const double perim = (b - a).norm() + (c - b).norm() + (a - c).norm();
        if (area2 > 0.0 && perim > 0.0) {
            const Eigen::Vector3d unitN = n / area2;
            const double inrad = area2 / perim;
            const double h = std::clamp(0.25 * inrad, 1.0e-4 * diag, 1.0e-2 * diag);
            const Eigen::Vector3d centroid = (a + b + c) / 3.0;
            triBrackets.emplace_back(centroid - h * unitN, centroid + h * unitN);
        }
    }

    constexpr int kProbeGrid = 10;
    const std::size_t numGridPts = static_cast<std::size_t>(kProbeGrid * kProbeGrid * kProbeGrid);
    std::vector<Eigen::Vector3d> gridWorldPts;
    gridWorldPts.reserve(numGridPts);
    for (int iz = 0; iz < kProbeGrid; ++iz) {
        for (int iy = 0; iy < kProbeGrid; ++iy) {
            for (int ix = 0; ix < kProbeGrid; ++ix) {
                const Eigen::Vector3d t(
                    (static_cast<double>(ix) + 0.37) / static_cast<double>(kProbeGrid),
                    (static_cast<double>(iy) + 0.41) / static_cast<double>(kProbeGrid),
                    (static_cast<double>(iz) + 0.29) / static_cast<double>(kProbeGrid));
                gridWorldPts.emplace_back(bbox.min.array() + t.array() * extents.array());
            }
        }
    }

    const Eigen::Index totalProbes = static_cast<Eigen::Index>(2 * triBrackets.size() + gridWorldPts.size());
    Eigen::MatrixXf probeQueries(totalProbes, 3);
    for (std::size_t i = 0; i < triBrackets.size(); ++i) {
        probeQueries.row(static_cast<Eigen::Index>(2 * i)) =
            ((triBrackets[i].first.transpose() - center) * scale).cast<float>();
        probeQueries.row(static_cast<Eigen::Index>(2 * i + 1)) =
            ((triBrackets[i].second.transpose() - center) * scale).cast<float>();
    }
    const Eigen::Index gridOffset = static_cast<Eigen::Index>(2 * triBrackets.size());
    for (std::size_t i = 0; i < gridWorldPts.size(); ++i) {
        probeQueries.row(gridOffset + static_cast<Eigen::Index>(i)) =
            ((gridWorldPts[i].transpose() - center) * scale).cast<float>();
    }

    Eigen::VectorXf probeW;
    igl::fast_winding_number(bvh, 2.0f, probeQueries, probeW);
    if (stopToken.stop_requested()) {
        setStatus(statusOut, "Cancelled.");
        return {};
    }

    std::vector<std::pair<CgalPoint3, CgalPoint3>> crossingSegments;
    crossingSegments.reserve(32);
    for (std::size_t i = 0; i < triBrackets.size(); ++i) {
        const bool inA = probeW(static_cast<Eigen::Index>(2 * i)) >= 0.5f;
        const bool inB = probeW(static_cast<Eigen::Index>(2 * i + 1)) >= 0.5f;
        if (inA != inB) {
            const Eigen::Vector3d& pIn = inA ? triBrackets[i].first : triBrackets[i].second;
            const Eigen::Vector3d& pOut = inA ? triBrackets[i].second : triBrackets[i].first;
            crossingSegments.emplace_back(CgalPoint3(pIn.x(), pIn.y(), pIn.z()),
                                          CgalPoint3(pOut.x(), pOut.y(), pOut.z()));
        }
    }

    // Also check lattice points against an exterior corner of the expanded bounding box
    const Eigen::Vector3d exteriorPt = center.transpose() + 1.25 * (bbox.max - center.transpose());
    const CgalPoint3 cgalExterior(exteriorPt.x(), exteriorPt.y(), exteriorPt.z());
    for (std::size_t i = 0; i < gridWorldPts.size() && crossingSegments.size() < 32; ++i) {
        if (probeW(gridOffset + static_cast<Eigen::Index>(i)) >= 0.5f) {
            const Eigen::Vector3d& pIn = gridWorldPts[i];
            crossingSegments.emplace_back(CgalPoint3(pIn.x(), pIn.y(), pIn.z()), cgalExterior);
        }
    }

    if (crossingSegments.empty()) {
        setStatus(statusOut, "The winding-number field has no enclosed volume (w = 0.5 isosurface).");
        return {};
    }

    using MeshDomain = CGAL::Labeled_mesh_domain_3<CgalKernel>;
    using MeshTr = CGAL::Mesh_triangulation_3<MeshDomain, CGAL::Default, CGAL::Parallel_if_available_tag>::type;
    using C3t3 = CGAL::Mesh_complex_3_in_triangulation_3<MeshTr>;
    using MeshCriteria = CGAL::Mesh_criteria_3<MeshTr>;

    std::atomic<bool> stopFlag{false};
    const auto windingOracle = [&](const CgalPoint3& p) -> double {
        if (stopToken.stop_requested()) {
            stopFlag.store(true, std::memory_order_relaxed);
            return 1.0;
        }
        const Eigen::RowVector3f q(
            static_cast<float>((p.x() - center.x()) * scale),
            static_cast<float>((p.y() - center.y()) * scale),
            static_cast<float>((p.z() - center.z()) * scale));
        const float wVal = igl::fast_winding_number(bvh, 2.0f, q);
        return 0.5 - static_cast<double>(wVal);
    };

    const CgalPoint3 sphereCenter(center.x(), center.y(), center.z());
    const double sphereRadius = 0.75 * diag;
    const CgalKernel::Sphere_3 boundingSphere(sphereCenter, sphereRadius * sphereRadius);

    CgalSurfaceMesh surface;
    CgalSurfaceMesh rawSurface;
    try {
        MeshDomain domain = MeshDomain::create_implicit_mesh_domain(
            windingOracle,
            boundingSphere,
            CGAL::parameters::relative_error_bound(settings::kDefaultIsosurfaceRelativeErrorBound));

        const auto doIntersect = domain.do_intersect_surface_object();
        const auto constructIntersection = domain.construct_intersection_object();
        std::vector<std::pair<CgalPoint3, MeshDomain::Index>> initialPoints;
        initialPoints.reserve(24);
        for (const auto& [pA, pB] : crossingSegments) {
            if (stopToken.stop_requested()) {
                setStatus(statusOut, "Cancelled.");
                return {};
            }
            if (initialPoints.size() >= 24) {
                break;
            }
            const CgalKernel::Segment_3 seg(pA, pB);
            if (const auto patch = doIntersect(seg)) {
                const CgalPoint3 hit = std::get<0>(constructIntersection(seg));
                initialPoints.emplace_back(hit, domain.index_from_surface_patch_index(*patch));
            }
        }
        if (initialPoints.empty()) {
            setStatus(statusOut, "The winding-number field has no enclosed volume (w = 0.5 isosurface).");
            return {};
        }

        const MeshCriteria criteria(
            CGAL::parameters::facet_angle(settings::kDefaultIsosurfaceMinFacetAngleDeg)
                .facet_size(facetRadius)
                .facet_distance(chordDistance)
                .facet_topology(CGAL::FACET_VERTICES_ON_SURFACE));

        const C3t3 c3t3 = CGAL::make_mesh_3<C3t3>(
            domain,
            criteria,
            CGAL::parameters::initial_points(initialPoints)
                .surface_only()
                .manifold()
                .no_exude()
                .no_perturb()
                .mesh_3_options(CGAL::parameters::pointer_to_stop_atomic_boolean(&stopFlag)));

        if (stopToken.stop_requested() || stopFlag.load(std::memory_order_relaxed)) {
            setStatus(statusOut, "Cancelled.");
            return {};
        }

        CGAL::facets_in_complex_3_to_triangle_mesh(c3t3, surface);
        if (surface.number_of_faces() == 0) {
            setStatus(statusOut, "Isosurface extraction produced no triangles.");
            return {};
        }
        if (CGAL::is_closed(surface)) {
            PMP::orient_to_bound_a_volume(surface);
        }

        if (clampedPasses > 0) {
            rawSurface = surface;
            PMP::tangential_relaxation(
                surface,
                CGAL::parameters::number_of_iterations(static_cast<unsigned int>(clampedPasses)));
        }
    } catch (const std::exception& e) {
        setStatus(statusOut, std::string("Precision isosurface failed: ") + e.what());
        return {};
    }

    if (stopToken.stop_requested()) {
        setStatus(statusOut, "Cancelled.");
        return {};
    }

    Mesh result = fromCgalSurfaceMesh(surface, mesh.fileUnit);
    if (signedVolume(result) < 0.0) {
        result.F.col(1).swap(result.F.col(2));
        result.recomputeNormals();
    }
    result = polishRebuiltMesh(std::move(result), stopToken);
    if (clampedPasses > 0 && !stopToken.stop_requested() &&
        (!checkSelfIntersections(result, CheckParams{}, stopToken).empty() ||
         !ThinFaceChecker::check(result).defects.empty())) {
        Mesh rawResult = fromCgalSurfaceMesh(rawSurface, mesh.fileUnit);
        if (signedVolume(rawResult) < 0.0) {
            rawResult.F.col(1).swap(rawResult.F.col(2));
            rawResult.recomputeNormals();
        }
        rawResult = polishRebuiltMesh(std::move(rawResult), stopToken);
        const std::size_t relDefects =
            checkSelfIntersections(result, CheckParams{}, stopToken).size() + ThinFaceChecker::check(result).defects.size();
        const std::size_t rawDefects =
            checkSelfIntersections(rawResult, CheckParams{}, stopToken).size() + ThinFaceChecker::check(rawResult).defects.size();
        if (rawDefects < relDefects) {
            result = std::move(rawResult);
        }
    }
    if (stopToken.stop_requested()) {
        setStatus(statusOut, "Cancelled.");
        return {};
    }
    std::array<char, 160> text{};
    std::snprintf(text.data(), text.size(),
                  "Isosurface extracted with target %.3g, chord %.3g (%d%%), %d relaxation pass%s.",
                  effectiveEdgeLength, chordDistance,
                  static_cast<int>(std::round(clampedChordRatio * 100.0)),
                  clampedPasses, clampedPasses == 1 ? "" : "es");
    setStatus(statusOut, text.data());
    return result;
}

Mesh simplifyToFaceCount(const Mesh& mesh, std::size_t targetFaces) {
    namespace SMS = CGAL::Surface_mesh_simplification;
    if (static_cast<std::size_t>(mesh.numFaces()) <= targetFaces || targetFaces == 0) {
        return mesh;
    }
    CgalSurfaceMesh surface;
    if (!toCgalSurfaceMesh(mesh, surface)) {
        return mesh;
    }
    using Policies = SMS::GarlandHeckbert_plane_policies<CgalSurfaceMesh, CgalKernel>;
    Policies policies(surface);
    const SMS::Face_count_stop_predicate<CgalSurfaceMesh> stop(targetFaces);
    // The normal-change filter rejects collapses that flip faces (a common source of new crossings)
    const SMS::Bounded_normal_change_filter<> filter;
    SMS::edge_collapse(surface, stop,
                       CGAL::parameters::get_cost(policies.get_cost())
                           .get_placement(policies.get_placement())
                           .filter(filter));
    surface.collect_garbage();
    return polishRebuiltMesh(fromCgalSurfaceMesh(surface, mesh.fileUnit));
}

} // namespace ogun
