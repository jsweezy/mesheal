#include "repair/SurfaceRebuild.hpp"

#include "mesh/CgalMesh.hpp"
#include "repair/RepairMetrics.hpp"

#include <Eigen/Geometry>

#include <CGAL/Polygon_mesh_processing/autorefinement.h>
#include <CGAL/Polygon_mesh_processing/repair_polygon_soup.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Bounded_normal_change_filter.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Face_count_stop_predicate.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/GarlandHeckbert_policies.h>
#include <CGAL/Surface_mesh_simplification/edge_collapse.h>
#include <CGAL/alpha_wrap_3.h>

#include <igl/fast_winding_number.h>
#include <igl/marching_cubes.h>
#include <igl/voxel_grid.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <exception>
#include <functional>
#include <numeric>
#include <unordered_map>
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
    return fromCgalSurfaceMesh(surface, mesh.fileUnit);
}

} // namespace ogun
