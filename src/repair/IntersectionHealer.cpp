#include "repair/IntersectionHealer.hpp"

#include "core/Constants.hpp"
#include "diagnostics/MeshChecks.hpp"
#include "diagnostics/SelfIntersections.hpp"
#include "mesh/CgalMesh.hpp"
#include "repair/SurfaceRebuild.hpp"

#include <Eigen/Geometry>

#include <CGAL/Polygon_mesh_processing/repair_self_intersections.h>
#include <CGAL/Polygon_mesh_processing/triangulate_hole.h>
#include <CGAL/boost/graph/Euler_operations.h>
#include <CGAL/boost/graph/iterator.h>
#include <CGAL/boost/graph/selection.h>

#include <algorithm>
#include <cmath>
#include <exception>
#include <numeric>
#include <unordered_map>
#include <unordered_set>

namespace mesheal {

namespace PMP = CGAL::Polygon_mesh_processing;

namespace {

/// Faces around each vertex in CSR form.
struct VertexFaces {
    std::vector<std::uint32_t> offsets;
    std::vector<Eigen::Index> faces;

    explicit VertexFaces(const Mesh& mesh) : offsets(static_cast<std::size_t>(mesh.numVertices()) + 1, 0) {
        for (Eigen::Index f = 0; f < mesh.numFaces(); ++f) {
            for (int c = 0; c < 3; ++c) {
                ++offsets[static_cast<std::size_t>(mesh.F(f, c)) + 1];
            }
        }
        std::partial_sum(offsets.begin(), offsets.end(), offsets.begin());
        faces.resize(offsets.back());
        std::vector<std::uint32_t> fill(offsets.begin(), offsets.end() - 1);
        for (Eigen::Index f = 0; f < mesh.numFaces(); ++f) {
            for (int c = 0; c < 3; ++c) {
                faces[fill[static_cast<std::size_t>(mesh.F(f, c))]++] = f;
            }
        }
    }

    [[nodiscard]] std::span<const Eigen::Index> of(int v) const noexcept {
        const auto i = static_cast<std::size_t>(v);
        return {faces.data() + offsets[i], faces.data() + offsets[i + 1]};
    }
};

struct UnionFind {
    std::vector<std::size_t> parent;
    explicit UnionFind(std::size_t n) : parent(n) {
        std::iota(parent.begin(), parent.end(), std::size_t{0});
    }
    std::size_t find(std::size_t x) {
        while (parent[x] != x) {
            parent[x] = parent[parent[x]];
            x = parent[x];
        }
        return x;
    }
    void unite(std::size_t a, std::size_t b) {
        parent[find(a)] = find(b);
    }
};

Eigen::Vector3d faceNormal(const Mesh& mesh, Eigen::Index f) {
    const Eigen::Vector3d a = mesh.V.row(mesh.F(f, 0));
    const Eigen::Vector3d b = mesh.V.row(mesh.F(f, 1));
    const Eigen::Vector3d c = mesh.V.row(mesh.F(f, 2));
    const Eigen::Vector3d n = (b - a).cross(c - a);
    const double len = n.norm();
    return len > 0.0 ? Eigen::Vector3d(n / len) : Eigen::Vector3d::Zero();
}

bool shareEdge(const Mesh& mesh, Eigen::Index f, Eigen::Index g) {
    int shared = 0;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            shared += mesh.F(f, i) == mesh.F(g, j) ? 1 : 0;
        }
    }
    return shared == 2;
}

/// Island faces plus every face sharing a vertex with them (sorted).
std::vector<Eigen::Index> dilateByVertex(const Mesh& mesh, const VertexFaces& vf, std::span<const Eigen::Index> faces) {
    std::vector<Eigen::Index> out(faces.begin(), faces.end());
    for (const Eigen::Index f : faces) {
        for (int c = 0; c < 3; ++c) {
            const auto around = vf.of(mesh.F(f, c));
            out.insert(out.end(), around.begin(), around.end());
        }
    }
    std::sort(out.begin(), out.end());
    out.erase(std::unique(out.begin(), out.end()), out.end());
    return out;
}

/// True if the faces form one edge-connected patch.
bool edgeConnected(const Mesh& mesh, const VertexFaces& vf, const std::vector<Eigen::Index>& faces) {
    if (faces.empty()) {
        return true;
    }
    std::unordered_set<Eigen::Index> inSet(faces.begin(), faces.end());
    std::unordered_set<Eigen::Index> visited{faces.front()};
    std::vector<Eigen::Index> stack{faces.front()};
    while (!stack.empty()) {
        const Eigen::Index f = stack.back();
        stack.pop_back();
        for (int c = 0; c < 3; ++c) {
            for (const Eigen::Index g : vf.of(mesh.F(f, c))) {
                if (inSet.count(g) != 0 && visited.count(g) == 0 && shareEdge(mesh, f, g)) {
                    visited.insert(g);
                    stack.push_back(g);
                }
            }
        }
    }
    return visited.size() == faces.size();
}

IslandAnalysis analyze(const Mesh& mesh, const VertexFaces& vf, const IntersectionIsland& island,
                       const IntersectionRepairSettings& settings) {
    IslandAnalysis a;
    a.local = island.local;
    const std::vector<Eigen::Index> dilated = dilateByVertex(mesh, vf, island.faces);
    std::vector<Eigen::Index> collar;
    std::set_difference(dilated.begin(), dilated.end(), island.faces.begin(), island.faces.end(), std::back_inserter(collar));

    // Dihedral angles between collar faces that share an edge (via vertex adjacency to avoid O(n^2) collar scans)
    std::unordered_set<Eigen::Index> collarSet(collar.begin(), collar.end());
    double sum = 0.0;
    std::size_t count = 0;
    for (const Eigen::Index fi : collar) {
        const Eigen::Vector3d ni = faceNormal(mesh, fi);
        for (int c = 0; c < 3; ++c) {
            const int u = mesh.F(fi, c);
            const int v = mesh.F(fi, (c + 1) % 3);
            for (const Eigen::Index fj : vf.of(u)) {
                if (fj <= fi || collarSet.count(fj) == 0) {
                    continue;
                }
                // Must hold v too, and share exactly that edge: a duplicate or flipped copy of fi shares all three
                // corners and is no neighbour (the old pairwise scan skipped it the same way)
                if ((mesh.F(fj, 0) != v && mesh.F(fj, 1) != v && mesh.F(fj, 2) != v) || !shareEdge(mesh, fi, fj)) {
                    continue;
                }
                const double dot = std::clamp(ni.dot(faceNormal(mesh, fj)), -1.0, 1.0);
                const double angle = std::acos(dot) * kRadToDeg;
                sum += angle;
                ++count;
                a.collarMaxDihedralDeg = std::max(a.collarMaxDihedralDeg, angle);
            }
        }
    }
    a.collarMeanDihedralDeg = count > 0 ? sum / static_cast<double>(count) : 0.0;

    if (!a.local) {
        a.recommended = IntersectionRepairMethod::SelfUnion;
    } else if (a.collarMeanDihedralDeg >= settings.smoothCollarMinAngleDeg &&
               a.collarMaxDihedralDeg < settings.creaseAngleDeg) {
        a.recommended = IntersectionRepairMethod::FillSmooth;
    } else {
        a.recommended = IntersectionRepairMethod::FillFlat;
    }
    return a;
}

struct IslandJob {
    std::vector<Eigen::Index> faces;  ///< Faces of the shell mesh
    IntersectionRepairMethod method = IntersectionRepairMethod::None;
};

bool isCgalJob(IntersectionRepairMethod method) {
    return method != IntersectionRepairMethod::SelfUnion && method != IntersectionRepairMethod::None;
}

FaceGeometryKey sortedCorners(std::array<Eigen::Vector3d, 3> p) {
    std::sort(p.begin(), p.end(), [](const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
        return std::lexicographical_compare(a.data(), a.data() + 3, b.data(), b.data() + 3);
    });
    FaceGeometryKey key{};
    for (std::size_t c = 0; c < 3; ++c) {
        std::copy(p[c].data(), p[c].data() + 3, key.begin() + static_cast<std::ptrdiff_t>(3 * c));
    }
    return key;
}

FaceGeometryKey meshFaceKey(const Mesh& mesh, Eigen::Index f) {
    return sortedCorners({mesh.V.row(mesh.F(f, 0)).transpose(), mesh.V.row(mesh.F(f, 1)).transpose(),
                          mesh.V.row(mesh.F(f, 2)).transpose()});
}

FaceGeometryKey cgalFaceKey(const CgalSurfaceMesh& sm, CgalFaceIndex f) {
    std::array<Eigen::Vector3d, 3> p;
    std::size_t c = 0;
    for (const CgalVertexIndex v : CGAL::vertices_around_face(sm.halfedge(f), sm)) {
        const CgalPoint3& q = sm.point(v);
        p[c++] = Eigen::Vector3d(q.x(), q.y(), q.z());
        if (c == 3) {
            break;
        }
    }
    return sortedCorners(p);
}

/// One job's faces in the halfedge mesh, with the corners each had when mapped. A fill frees face slots that
/// `Surface_mesh` hands to the next patch it builds, so a mapped index may later name a neighbour's new face.
struct MappedFaces {
    std::vector<CgalFaceIndex> faces;
    std::vector<FaceGeometryKey> keys;
};

/// The mapped faces still in `sm` as they were mapped: not removed, and the slot not reused by a patch.
std::vector<CgalFaceIndex> liveFaces(const CgalSurfaceMesh& sm, const MappedFaces& mapped) {
    std::vector<CgalFaceIndex> out;
    out.reserve(mapped.faces.size());
    for (std::size_t i = 0; i < mapped.faces.size(); ++i) {
        const CgalFaceIndex f = mapped.faces[i];
        if (!sm.is_removed(f) && cgalFaceKey(sm, f) == mapped.keys[i]) {
            out.push_back(f);
        }
    }
    return out;
}

/// Maps the faces of every CGAL job to the halfedge mesh built from `mesh`, in one pass. `toCgalSurfaceMesh`
/// keeps indices for manifold input; otherwise faces are matched by their corner positions (any winding).
std::vector<MappedFaces> mapJobsToCgal(const Mesh& mesh, const CgalSurfaceMesh& sm, const std::vector<IslandJob>& jobs) {
    std::vector<MappedFaces> mapped(jobs.size());
    bool direct = static_cast<Eigen::Index>(sm.number_of_faces()) == mesh.numFaces();
    for (std::size_t j = 0; j < jobs.size() && direct; ++j) {
        if (!isCgalJob(jobs[j].method)) {
            continue;
        }
        for (const Eigen::Index f : jobs[j].faces) {
            const CgalFaceIndex cf(static_cast<CgalSurfaceMesh::size_type>(f));
            const FaceGeometryKey key = meshFaceKey(mesh, f);
            if (cgalFaceKey(sm, cf) != key) {
                direct = false;
                break;
            }
            mapped[j].faces.push_back(cf);
            mapped[j].keys.push_back(key);
        }
    }
    if (direct) {
        return mapped;
    }

    std::unordered_map<FaceGeometryKey, CgalFaceIndex, FaceGeometryKeyHash> byKey;
    for (std::size_t j = 0; j < jobs.size(); ++j) {
        mapped[j] = {};
        if (!isCgalJob(jobs[j].method)) {
            continue;
        }
        for (const Eigen::Index f : jobs[j].faces) {
            mapped[j].keys.push_back(meshFaceKey(mesh, f));
            byKey.emplace(mapped[j].keys.back(), CgalSurfaceMesh::null_face());
        }
    }
    std::size_t unmatched = byKey.size();
    for (const CgalFaceIndex f : sm.faces()) {
        if (unmatched == 0) {
            break;
        }
        if (const auto it = byKey.find(cgalFaceKey(sm, f)); it != byKey.end() && it->second == CgalSurfaceMesh::null_face()) {
            it->second = f;
            --unmatched;
        }
    }
    for (MappedFaces& m : mapped) {
        std::vector<FaceGeometryKey> keys;
        keys.swap(m.keys);
        for (const FaceGeometryKey& key : keys) {
            if (const CgalFaceIndex cf = byKey.at(key); cf != CgalSurfaceMesh::null_face()) {
                m.faces.push_back(cf);
                m.keys.push_back(key);
            }
        }
    }
    return mapped;
}

/// Selects the island faces plus `rings` rings, grown so that removing them leaves a manifold hole.
std::vector<CgalFaceIndex> selectCut(CgalSurfaceMesh& sm, const std::vector<CgalFaceIndex>& faces, int rings,
                                     CgalSurfaceMesh::Property_map<CgalFaceIndex, bool>& selected) {
    std::vector<CgalFaceIndex> cut = faces;
    for (const CgalFaceIndex f : cut) {
        selected[f] = true;
    }
    if (rings > 0) {
        CGAL::expand_face_selection(faces, sm, static_cast<unsigned>(rings), selected, std::back_inserter(cut));
    }
    CGAL::expand_face_selection_for_removal(std::vector<CgalFaceIndex>(cut), sm, selected);
    // Collect what the removal expansion added: selected faces reachable through shared vertices
    std::unordered_set<std::uint32_t> inCut;
    for (const CgalFaceIndex f : cut) {
        inCut.insert(f.idx());
    }
    for (std::size_t i = 0; i < cut.size(); ++i) {
        for (const CgalHalfedgeIndex h : CGAL::halfedges_around_face(sm.halfedge(cut[i]), sm)) {
            for (const CgalFaceIndex g : CGAL::faces_around_target(h, sm)) {
                if (g != CgalSurfaceMesh::null_face() && selected[g] && inCut.insert(g.idx()).second) {
                    cut.push_back(g);
                }
            }
        }
    }
    return cut;
}

bool touchesBorder(const CgalSurfaceMesh& sm, const std::vector<CgalFaceIndex>& faces) {
    for (const CgalFaceIndex f : faces) {
        for (const CgalVertexIndex v : CGAL::vertices_around_face(sm.halfedge(f), sm)) {
            if (sm.is_border(v)) {
                return true;
            }
        }
    }
    return false;
}

enum class FillOutcome {
    NotCut,   ///< Refused before removing anything; the mesh is unchanged
    Filled,   ///< Cut and every new hole filled
    LeftHole, ///< Cut, but a new hole could not be filled: the mesh now has an opening the input did not
};

/// Cuts the island (+ rings) out, grown so the hole boundary stays manifold, and fills each new hole.
/// A cut that would reach an open border of the input is retried without rings, then refused: the
/// merged hole would include the original border and filling it would cover the whole opening.
FillOutcome fillIsland(CgalSurfaceMesh& sm, const std::vector<CgalFaceIndex>& live, bool smooth, int rings,
                       CgalSurfaceMesh::Property_map<CgalFaceIndex, bool>& selected, std::string& status) {
    if (live.empty()) {
        status = "The island was already removed by a neighbouring repair.";
        return FillOutcome::NotCut;
    }
    std::vector<CgalFaceIndex> cut;
    const auto clearSelected = [&]() {
        for (const CgalFaceIndex f : cut) {
            selected[f] = false;
        }
    };
    for (int r = std::max(rings, 0);; r = 0) {
        clearSelected();
        cut = selectCut(sm, live, r, selected);
        if (!touchesBorder(sm, cut) || r == 0) {
            break;
        }
    }
    if (touchesBorder(sm, cut)) {
        clearSelected();
        status = "The island reaches an open border: cutting it out would merge with that hole. "
                 "Try CGAL Local Repair or Cut & Merge Walls.";
        return FillOutcome::NotCut;
    }

    // Halfedges that turn into hole borders: the cut side of edges between cut and kept faces
    std::vector<CgalHalfedgeIndex> newBorder;
    for (const CgalFaceIndex f : cut) {
        for (const CgalHalfedgeIndex h : CGAL::halfedges_around_face(sm.halfedge(f), sm)) {
            const CgalFaceIndex other = sm.face(sm.opposite(h));
            if (other != CgalSurfaceMesh::null_face() && !selected[other]) {
                newBorder.push_back(h);
            }
        }
    }
    clearSelected();
    if (newBorder.empty()) {
        status = "The cut would remove the whole shell, leaving nothing to fill. Try another method or fewer rings.";
        return FillOutcome::NotCut;
    }
    for (const CgalFaceIndex f : cut) {
        if (!sm.is_removed(f)) {
            CGAL::Euler::remove_face(sm.halfedge(f), sm);
        }
    }

    // Fill each new hole once (original holes, whose border has no new halfedge, stay open)
    std::unordered_set<std::uint32_t> visited;
    int filled = 0;
    int fairFailed = 0;
    for (const CgalHalfedgeIndex h : newBorder) {
        if (sm.is_removed(h) || !sm.is_border(h) || visited.count(h.idx()) != 0) {
            continue;
        }
        std::size_t holeEdges = 0;
        for (const CgalHalfedgeIndex b : CGAL::halfedges_around_face(h, sm)) {
            visited.insert(b.idx());
            ++holeEdges;
        }
        // Same search as Fill Hole (HoleFiller): 2D CDT first; no O(n^3) fallback on big holes
        std::vector<CgalFaceIndex> patch;
        const auto np = CGAL::parameters::use_2d_constrained_delaunay_triangulation(true)
                            .do_not_use_cubic_algorithm(holeEdges > settings::kMaxCubicHoleFillEdges)
                            .face_output_iterator(std::back_inserter(patch));
        if (smooth) {
            fairFailed += std::get<0>(PMP::triangulate_refine_and_fair_hole(sm, h, np)) ? 0 : 1;
        } else {
            PMP::triangulate_hole(sm, h, np);
        }
        if (patch.empty()) {
            status = "A " + std::to_string(holeEdges) + "-edge hole left by the cut could not be triangulated.";
            return FillOutcome::LeftHole;
        }
        ++filled;
    }
    status = "Cut " + std::to_string(cut.size()) + " faces, filled " + std::to_string(filled) + " hole" +
             (filled == 1 ? "" : "s") + (fairFailed > 0 ? " (fairing failed; refined patch kept)" : "") + ".";
    return filled > 0 ? FillOutcome::Filled : FillOutcome::LeftHole;
}

/// Grows `faces` (marked in `inSet`) by whole vertex rings, `rings` times or until it holds more than `maxFaces`.
/// Returns false if it outgrew `maxFaces`.
bool growByVertexRings(const CgalSurfaceMesh& sm, std::vector<CgalFaceIndex>& faces,
                       std::unordered_set<std::uint32_t>& inSet, int rings, std::size_t maxFaces) {
    std::size_t ringStart = 0;
    for (int r = 0; r < rings && ringStart < faces.size(); ++r) {
        const std::size_t ringEnd = faces.size();
        for (std::size_t i = ringStart; i < ringEnd; ++i) {
            for (const CgalVertexIndex v : CGAL::vertices_around_face(sm.halfedge(faces[i]), sm)) {
                for (const CgalFaceIndex g : CGAL::faces_around_target(sm.halfedge(v), sm)) {
                    if (g != CgalSurfaceMesh::null_face() && inSet.insert(g.idx()).second) {
                        faces.push_back(g);
                        if (faces.size() > maxFaces) {
                            return false;
                        }
                    }
                }
            }
        }
        ringStart = ringEnd;
    }
    return true;
}

std::size_t countSetBoundaryEdges(const CgalSurfaceMesh& sm, const std::vector<CgalFaceIndex>& faces,
                                  const std::unordered_set<std::uint32_t>& faceSet) {
    std::size_t boundaryEdges = 0;
    for (const CgalFaceIndex g : faces) {
        for (const CgalHalfedgeIndex h : CGAL::halfedges_around_face(sm.halfedge(g), sm)) {
            const CgalFaceIndex opp = sm.face(sm.opposite(h));
            if (opp == CgalSurfaceMesh::null_face() || faceSet.count(opp.idx()) == 0) {
                ++boundaryEdges;
            }
        }
    }
    return boundaryEdges;
}

/// What a batch CGAL local repair may take on. `remove_self_intersections` grows each crossing patch by up to
/// `maxSteps - 1` rings, merges patches that touch, and refills each with a hole search that costs O(n^3) in the
/// rim length when its Delaunay search fails. Patches are predicted the same way and taken smallest first: any
/// hole over `kMaxCgalLocalRepairHoleEdges` rim edges is left out, and the rest until their summed n^3 reaches
/// the budget. A batch on an ordinary mesh takes everything; one on a mesh with 300k crossings stays bounded.
struct LocalRepairPlan {
    std::vector<CgalFaceIndex> faces; ///< Crossing faces to hand to CGAL
    std::size_t patches = 0;          ///< Predicted holes taken
    std::size_t skippedLarge = 0;     ///< Predicted holes left out for size
    std::size_t skippedBudget = 0;    ///< Predicted holes left out for the time budget
};

LocalRepairPlan planLocalRepair(const CgalSurfaceMesh& sm, const std::vector<CgalFaceIndex>& live, int maxSteps) {
    struct Patch {
        std::vector<CgalFaceIndex> seeds;
        std::vector<CgalFaceIndex> grown;
    };
    LocalRepairPlan plan;
    std::unordered_set<std::uint32_t> unvisited;
    for (const CgalFaceIndex f : live) {
        unvisited.insert(f.idx());
    }
    std::vector<Patch> patches;
    for (const CgalFaceIndex seed : live) {
        if (unvisited.erase(seed.idx()) == 0) {
            continue;
        }
        Patch p;
        p.seeds.push_back(seed);
        for (std::size_t q = 0; q < p.seeds.size(); ++q) {
            for (const CgalVertexIndex v : CGAL::vertices_around_face(sm.halfedge(p.seeds[q]), sm)) {
                for (const CgalFaceIndex g : CGAL::faces_around_target(sm.halfedge(v), sm)) {
                    if (g != CgalSurfaceMesh::null_face() && unvisited.erase(g.idx()) != 0) {
                        p.seeds.push_back(g);
                    }
                }
            }
        }
        p.grown = p.seeds;
        std::unordered_set<std::uint32_t> grownSet;
        for (const CgalFaceIndex f : p.grown) {
            grownSet.insert(f.idx());
        }
        if (p.grown.size() > settings::kMaxCgalLocalRepairPatchFaces ||
            !growByVertexRings(sm, p.grown, grownSet, std::max(maxSteps - 1, 0), settings::kMaxCgalLocalRepairPatchFaces)) {
            ++plan.skippedLarge;
            continue;
        }
        patches.push_back(std::move(p));
    }

    // Grown patches sharing a vertex become one hole in CGAL
    UnionFind uf(patches.size());
    std::unordered_map<std::uint32_t, std::size_t> vertexOwner;
    for (std::size_t i = 0; i < patches.size(); ++i) {
        for (const CgalFaceIndex f : patches[i].grown) {
            for (const CgalVertexIndex v : CGAL::vertices_around_face(sm.halfedge(f), sm)) {
                if (const auto [it, inserted] = vertexOwner.try_emplace(v.idx(), i); !inserted) {
                    uf.unite(i, it->second);
                }
            }
        }
    }
    std::unordered_map<std::size_t, std::vector<std::size_t>> byRoot;
    for (std::size_t i = 0; i < patches.size(); ++i) {
        byRoot[uf.find(i)].push_back(i);
    }
    struct Hole {
        std::size_t rimEdges;
        std::vector<std::size_t> members;
    };
    std::vector<Hole> holes;
    for (auto& [root, members] : byRoot) {
        std::vector<CgalFaceIndex> faces;
        std::unordered_set<std::uint32_t> faceSet;
        for (const std::size_t i : members) {
            for (const CgalFaceIndex f : patches[i].grown) {
                if (faceSet.insert(f.idx()).second) {
                    faces.push_back(f);
                }
            }
        }
        holes.push_back({countSetBoundaryEdges(sm, faces, faceSet), std::move(members)});
    }
    std::sort(holes.begin(), holes.end(), [](const Hole& a, const Hole& b) {
        return a.rimEdges != b.rimEdges ? a.rimEdges < b.rimEdges : a.members.front() < b.members.front();
    });

    double work = 0.0;
    for (const Hole& hole : holes) {
        const double cost = std::pow(static_cast<double>(hole.rimEdges), 3.0);
        if (hole.rimEdges > settings::kMaxCgalLocalRepairHoleEdges) {
            ++plan.skippedLarge;
        } else if (work + cost > settings::kMaxCgalLocalRepairCubicWork) {
            ++plan.skippedBudget;
        } else {
            work += cost;
            ++plan.patches;
            for (const std::size_t i : hole.members) {
                plan.faces.insert(plan.faces.end(), patches[i].seeds.begin(), patches[i].seeds.end());
            }
        }
    }
    return plan;
}

/// Runs CGAL's local self-intersection repair on `faces`. A batch (`limitSize`) plans the patches first and
/// leaves out any whose refill would stall (see `LocalRepairPlan`); a repair the user picked for one island
/// is always attempted in full.
bool cgalLocalRepair(CgalSurfaceMesh& sm, const std::vector<CgalFaceIndex>& faces, int maxSteps, bool limitSize,
                     std::string& status) {
    std::vector<CgalFaceIndex> live;
    std::unordered_set<std::uint32_t> seen;
    for (const CgalFaceIndex f : faces) {
        if (!sm.is_removed(f) && seen.insert(f.idx()).second) {
            live.push_back(f);
        }
    }
    if (live.empty()) {
        status = "The island was already removed by a neighbouring repair.";
        return false;
    }

    std::size_t patches = 1;
    std::string skippedNote;
    if (limitSize) {
        LocalRepairPlan plan = planLocalRepair(sm, live, maxSteps);
        const std::size_t skipped = plan.skippedLarge + plan.skippedBudget;
        if (skipped > 0) {
            skippedNote = " Skipped " + std::to_string(skipped) + " of " + std::to_string(skipped + plan.patches) +
                          " crossing patches:";
            if (plan.skippedLarge > 0) {
                skippedNote += " " + std::to_string(plan.skippedLarge) + " would leave a hole over " +
                               std::to_string(settings::kMaxCgalLocalRepairHoleEdges) + " edges or " +
                               std::to_string(settings::kMaxCgalLocalRepairPatchFaces) + " faces";
            }
            if (plan.skippedBudget > 0) {
                skippedNote += std::string(plan.skippedLarge > 0 ? "," : "") + " " + std::to_string(plan.skippedBudget) +
                               " did not fit the time budget";
            }
            skippedNote += ".";
        }
        if (plan.faces.empty()) {
            status = "Skipped CGAL local repair:" + skippedNote.substr(skippedNote.find(':') + 1);
            return false;
        }
        live = std::move(plan.faces);
        patches = plan.patches;
    }

    // CGAL's smoothing copies each patch through a Face_filtered_graph that scans every halfedge of `sm`:
    // keep it while the scans stay cheap (always on ordinary meshes, a few patches on multi-million-face ones)
    const bool useSmoothing =
        static_cast<double>(patches) * static_cast<double>(sm.number_of_halfedges()) <= settings::kMaxCgalLocalSmoothingScans;
    bool solved = false;
    try {
        solved = PMP::experimental::remove_self_intersections(
            live, sm, CGAL::parameters::preserve_genus(false).use_smoothing(useSmoothing).number_of_iterations(maxSteps));
    } catch (const std::exception& e) {
        status = std::string("CGAL local repair failed: ") + e.what();
        return false;
    }
    status = std::string(solved ? "CGAL local repair removed the crossings it was given."
                                : "CGAL local repair could not remove every crossing.") +
             (useSmoothing ? "" : " (Smoothing skipped on this large mesh.)") + skippedNote;
    return true;
}

/// Applies every job to `shell`: cuts / CGAL repairs in place on one halfedge mesh, then one self-union if any
/// job asked for it. Returns an empty mesh if nothing could be applied. A fill that cuts but cannot close its
/// hole spoils the halfedge mesh, so the shell is rebuilt and the jobs rerun without it (a few times at most).
/// `batch` limits CGAL Local Repair to patches it can refill in bounded time.
Mesh repairShell(const Mesh& shell, const std::vector<IslandJob>& jobs, const IntersectionRepairSettings& settings,
                 bool batch, std::string& status) {
    bool wantsUnion = false;
    bool wantsCgal = false;
    for (const IslandJob& job : jobs) {
        wantsUnion = wantsUnion || job.method == IntersectionRepairMethod::SelfUnion;
        wantsCgal = wantsCgal || isCgalJob(job.method);
    }

    Mesh current = shell;
    bool changed = false;
    std::vector<std::string> notes;
    std::vector<bool> dropped(jobs.size(), false);
    std::size_t droppedCount = 0;
    for (int attempt = 0; wantsCgal && attempt < settings::kMaxShellRepairAttempts; ++attempt) {
        CgalSurfaceMesh sm;
        if (!toCgalSurfaceMesh(shell, sm)) {
            status = "The shell could not be converted to a halfedge mesh.";
            return {};
        }
        const std::vector<MappedFaces> mapped = mapJobsToCgal(shell, sm, jobs);
        auto selected = sm.add_property_map<CgalFaceIndex, bool>("f:mesheal_cut", false).first;
        std::vector<std::size_t> leftHole;
        std::vector<std::size_t> localJobs;
        std::string firstFailure;
        std::string lastSuccess;
        changed = false;
        for (std::size_t j = 0; j < jobs.size(); ++j) {
            if (dropped[j]) {
                continue;
            }
            if (jobs[j].method == IntersectionRepairMethod::CgalLocalRepair) {
                localJobs.push_back(j); // one CGAL call for all of them, after the fills
                continue;
            }
            if (jobs[j].method != IntersectionRepairMethod::FillFlat && jobs[j].method != IntersectionRepairMethod::FillSmooth) {
                continue;
            }
            std::string jobStatus;
            const FillOutcome outcome = fillIsland(sm, liveFaces(sm, mapped[j]),
                                                   jobs[j].method == IntersectionRepairMethod::FillSmooth,
                                                   settings.dilationRings, selected, jobStatus);
            if (outcome == FillOutcome::LeftHole) {
                leftHole.push_back(j);
            }
            if (outcome == FillOutcome::Filled) {
                changed = true;
                lastSuccess = jobStatus;
            } else if (firstFailure.empty()) {
                firstFailure = jobStatus;
            }
        }
        sm.remove_property_map(selected);
        if (!leftHole.empty()) {
            for (const std::size_t j : leftHole) {
                dropped[j] = true;
            }
            droppedCount += leftHole.size();
            notes = {firstFailure};
            changed = false;
            bool anyLeft = false;
            for (std::size_t j = 0; j < jobs.size(); ++j) {
                anyLeft = anyLeft || (!dropped[j] && isCgalJob(jobs[j].method));
            }
            if (!anyLeft) {
                break; // nothing left to rerun
            }
            continue; // `sm` has an open cut: rebuild it and rerun the other jobs
        }
        notes.clear();
        if (droppedCount > 0) {
            notes.push_back(std::to_string(droppedCount) + (droppedCount == 1 ? " island was" : " islands were") +
                            " left as is: cutting it out left a hole that could not be filled.");
        }
        for (const std::string* s : {&lastSuccess, &firstFailure}) {
            if (!s->empty()) {
                notes.push_back(*s);
            }
        }
        if (!localJobs.empty()) {
            std::vector<CgalFaceIndex> faces;
            for (const std::size_t j : localJobs) {
                const std::vector<CgalFaceIndex> live = liveFaces(sm, mapped[j]);
                faces.insert(faces.end(), live.begin(), live.end());
            }
            std::string localStatus;
            changed = cgalLocalRepair(sm, faces, settings.cgalMaxSteps, batch, localStatus) || changed;
            notes.push_back(localStatus);
        }
        if (changed) {
            sm.collect_garbage();
            current = fromCgalSurfaceMesh(sm, shell.fileUnit);
        }
        break;
    }
    if (wantsUnion) {
        std::string unionStatus;
        Mesh united = selfUnion(current, &unionStatus);
        if (united.empty()) {
            notes.push_back(unionStatus);
        } else {
            current = std::move(united);
            changed = true;
            notes.push_back("Self-union: " + unionStatus);
        }
    }
    status.clear();
    for (const std::string& note : notes) {
        status += (status.empty() ? "" : " ") + note;
    }
    return changed ? current : Mesh{};
}

/// False when a round's result `after` puts new surface farther from the shell `before` than the limit. A cut
/// along the crossings (self-union) only splits old faces and drops what ends up inside, so its new faces lie on
/// the old surface; a fill that bulges over a CAD wall to undo a small crossing does not. Faces removed are not
/// measured: an inner part a union drops (a post sealed inside a cup) is far from the result but no damage.
bool keepsShape(const Mesh& before, const Mesh& after, const IntersectionRepairSettings& settings) {
    if (settings.maxDeviationFraction <= 0.0) {
        return true;
    }
    const double limit = settings.maxDeviationFraction * before.boundingBox().diagonal();
    return maxSurfaceDistance(extractFaces(after, faceDiff(before, after).added), before) <= limit;
}

double minAngleDeg(const Mesh& mesh, std::span<const Eigen::Index> faces) {
    double best = 180.0;
    for (const Eigen::Index f : faces) {
        for (int c = 0; c < 3; ++c) {
            const Eigen::Vector3d p = mesh.V.row(mesh.F(f, c));
            const Eigen::Vector3d u = Eigen::Vector3d(mesh.V.row(mesh.F(f, (c + 1) % 3))) - p;
            const Eigen::Vector3d v = Eigen::Vector3d(mesh.V.row(mesh.F(f, (c + 2) % 3))) - p;
            const double denom = u.norm() * v.norm();
            const double angle = denom > 0.0 ? std::acos(std::clamp(u.dot(v) / denom, -1.0, 1.0)) * kRadToDeg : 0.0;
            best = std::min(best, angle);
        }
    }
    return best;
}

/// Displaces each vertex along its area-weighted normal (keeps the preview off the base mesh).
void offsetAlongNormals(Mesh& mesh, double offset) {
    Eigen::MatrixXd normals = Eigen::MatrixXd::Zero(mesh.numVertices(), 3);
    for (Eigen::Index f = 0; f < mesh.numFaces(); ++f) {
        const Eigen::Vector3d a = mesh.V.row(mesh.F(f, 0));
        const Eigen::Vector3d n = (Eigen::Vector3d(mesh.V.row(mesh.F(f, 1))) - a).cross(Eigen::Vector3d(mesh.V.row(mesh.F(f, 2))) - a);
        for (int c = 0; c < 3; ++c) {
            normals.row(mesh.F(f, c)) += n.transpose();
        }
    }
    for (Eigen::Index v = 0; v < mesh.numVertices(); ++v) {
        const double len = normals.row(v).norm();
        if (len > 0.0) {
            mesh.V.row(v) += offset * normals.row(v) / len;
        }
    }
    mesh.recomputeNormals();
}

std::vector<Eigen::Index> facesWithShell(const std::vector<int>& faceShell, int shell) {
    std::vector<Eigen::Index> faces;
    for (std::size_t f = 0; f < faceShell.size(); ++f) {
        if (faceShell[f] == shell) {
            faces.push_back(static_cast<Eigen::Index>(f));
        }
    }
    return faces;
}

std::vector<Eigen::Index> facesWithoutShells(const std::vector<int>& faceShell, const std::unordered_set<int>& shells) {
    std::vector<Eigen::Index> faces;
    for (std::size_t f = 0; f < faceShell.size(); ++f) {
        if (shells.count(faceShell[f]) == 0) {
            faces.push_back(static_cast<Eigen::Index>(f));
        }
    }
    return faces;
}

/// Global face index -> index inside `faces` (-1 if absent).
std::vector<Eigen::Index> localIndex(Eigen::Index numFaces, const std::vector<Eigen::Index>& faces) {
    std::vector<Eigen::Index> local(static_cast<std::size_t>(numFaces), -1);
    for (std::size_t i = 0; i < faces.size(); ++i) {
        local[static_cast<std::size_t>(faces[i])] = static_cast<Eigen::Index>(i);
    }
    return local;
}

} // namespace

const char* intersectionRepairMethodLabel(IntersectionRepairMethod method) noexcept {
    switch (method) {
    case IntersectionRepairMethod::FillFlat:
        return "Remove & Fill Flat";
    case IntersectionRepairMethod::FillSmooth:
        return "Remove & Fill Smooth";
    case IntersectionRepairMethod::CgalLocalRepair:
        return "CGAL Local Repair";
    case IntersectionRepairMethod::SelfUnion:
        return "Cut & Merge Walls (self-union)";
    case IntersectionRepairMethod::None:
        break;
    }
    return "None";
}

std::vector<int> shellIdsOfFaces(const Mesh& mesh) {
    std::vector<int> faceShell(static_cast<std::size_t>(mesh.numFaces()), -1);
    const CategoryResult shells = checkShells(mesh);
    for (std::size_t s = 0; s < shells.size(); ++s) {
        for (const Eigen::Index f : shells.facesOf(s)) {
            faceShell[static_cast<std::size_t>(f)] = static_cast<int>(s);
        }
    }
    return faceShell;
}

std::vector<IntersectionIsland> clusterIntersectionIslands(
    const Mesh& mesh,
    const CategoryResult& intersections,
    std::span<const int> faceShell,
    std::span<const std::uint8_t> faceInScope) {
    const auto shellOf = [&](Eigen::Index f) {
        return faceShell.empty() ? 0 : faceShell[static_cast<std::size_t>(f)];
    };
    const auto inScope = [&](Eigen::Index f) {
        return faceInScope.empty() || faceInScope[static_cast<std::size_t>(f)];
    };

    // Faces with at least one in-scope partner of the same shell
    std::unordered_map<Eigen::Index, std::size_t> slot;
    std::vector<Eigen::Index> faces;
    std::vector<std::pair<Eigen::Index, Eigen::Index>> pairs;
    for (std::size_t e = 0; e < intersections.size(); ++e) {
        const auto own = intersections.facesOf(e);
        if (own.empty() || !inScope(own[0])) {
            continue;
        }
        const Eigen::Index f = own[0];
        for (const Eigen::Index p : intersections.relatedOf(e)) {
            if (inScope(p) && shellOf(p) == shellOf(f) && shellOf(f) >= 0) {
                pairs.emplace_back(f, p);
            }
        }
    }
    const auto slotOf = [&](Eigen::Index f) {
        const auto [it, inserted] = slot.try_emplace(f, faces.size());
        if (inserted) {
            faces.push_back(f);
        }
        return it->second;
    };
    for (const auto& [a, b] : pairs) {
        slotOf(a);
        slotOf(b);
    }
    if (faces.empty()) {
        return {};
    }

    UnionFind sets(faces.size());
    for (const auto& [a, b] : pairs) {
        sets.unite(slot[a], slot[b]);
    }
    std::unordered_map<int, std::size_t> vertexOwner;
    for (std::size_t i = 0; i < faces.size(); ++i) {
        for (int c = 0; c < 3; ++c) {
            const auto [it, inserted] = vertexOwner.try_emplace(mesh.F(faces[i], c), i);
            if (!inserted) {
                sets.unite(i, it->second);
            }
        }
    }

    std::unordered_map<std::size_t, std::size_t> rootToIsland;
    std::vector<IntersectionIsland> islands;
    for (std::size_t i = 0; i < faces.size(); ++i) {
        const auto [it, inserted] = rootToIsland.try_emplace(sets.find(i), islands.size());
        if (inserted) {
            islands.emplace_back();
            islands.back().shell = shellOf(faces[i]);
        }
        islands[it->second].faces.push_back(faces[i]);
    }

    const VertexFaces vf(mesh);
    for (IntersectionIsland& island : islands) {
        std::sort(island.faces.begin(), island.faces.end());
        island.local = edgeConnected(mesh, vf, dilateByVertex(mesh, vf, island.faces));
    }
    std::stable_sort(islands.begin(), islands.end(), [](const IntersectionIsland& a, const IntersectionIsland& b) {
        return a.faces.size() > b.faces.size();
    });
    return islands;
}

IslandAnalysis analyzeIsland(const Mesh& mesh, const IntersectionIsland& island, const IntersectionRepairSettings& settings) {
    return analyze(mesh, VertexFaces(mesh), island, settings);
}

std::array<IntersectionRepairMethod, 4> islandFallbackOrder(IntersectionRepairMethod recommended, bool local) noexcept {
    using M = IntersectionRepairMethod;
    if (!local || recommended == M::SelfUnion) {
        return {M::SelfUnion, M::CgalLocalRepair, M::FillSmooth, M::FillFlat};
    }
    if (recommended == M::FillSmooth) {
        return {M::FillSmooth, M::CgalLocalRepair, M::FillFlat, M::SelfUnion};
    }
    if (recommended == M::CgalLocalRepair) {
        return {M::CgalLocalRepair, M::FillFlat, M::FillSmooth, M::SelfUnion};
    }
    return {M::FillFlat, M::CgalLocalRepair, M::FillSmooth, M::SelfUnion};
}

IslandRepairResult repairIsland(
    const Mesh& mesh,
    const IntersectionIsland& island,
    IntersectionRepairMethod method,
    const CheckParams& params,
    const IntersectionRepairSettings& settings,
    std::span<const int> knownFaceShell) {
    IslandRepairResult result;
    result.mesh = mesh;
    if (island.faces.empty() || method == IntersectionRepairMethod::None) {
        result.statusMessage = "Pick an island and a repair method first.";
        return result;
    }
    const std::vector<int> faceShell = knownFaceShell.size() == static_cast<std::size_t>(mesh.numFaces())
        ? std::vector<int>(knownFaceShell.begin(), knownFaceShell.end())
        : shellIdsOfFaces(mesh);
    const int shell = faceShell[static_cast<std::size_t>(island.faces.front())];
    const std::vector<Eigen::Index> shellFaces = facesWithShell(faceShell, shell);
    const std::vector<Eigen::Index> restFaces = facesWithoutShells(faceShell, {shell});
    const std::vector<Eigen::Index> local = localIndex(mesh.numFaces(), shellFaces);

    const Mesh shellMesh = extractFaces(mesh, shellFaces);
    IslandJob job{{}, method};
    for (const Eigen::Index f : island.faces) {
        if (local[static_cast<std::size_t>(f)] >= 0) {
            job.faces.push_back(local[static_cast<std::size_t>(f)]);
        }
    }
    result.crossingsBefore = checkSelfIntersections(shellMesh, params).size();

    const Mesh repaired = repairShell(shellMesh, {job}, settings, false, result.statusMessage);
    if (repaired.empty()) {
        if (result.statusMessage.empty()) {
            result.statusMessage = std::string(intersectionRepairMethodLabel(method)) + " changed nothing.";
        }
        return result;
    }

    const FaceDiff diff = faceDiff(shellMesh, repaired);
    Mesh assembled = extractFaces(mesh, restFaces);
    const Eigen::Index offset = assembled.numFaces();
    appendMesh(assembled, repaired);
    result.mesh = std::move(assembled);
    for (const Eigen::Index f : diff.removed) {
        result.removedFaces.push_back(shellFaces[static_cast<std::size_t>(f)]);
    }
    std::unordered_set<Eigen::Index> added;
    for (const Eigen::Index f : diff.added) {
        result.addedFaces.push_back(offset + f);
        added.insert(f);
    }

    const CategoryResult after = checkSelfIntersections(repaired, params);
    result.crossingsAfter = after.size();
    for (std::size_t e = 0; e < after.size(); ++e) {
        const auto own = after.facesOf(e);
        result.newCrossings += !own.empty() && added.count(own[0]) != 0 ? 1 : 0;
    }
    result.volumeChange = signedVolume(repaired) - signedVolume(shellMesh);
    result.minNewAngleDeg = minAngleDeg(repaired, diff.added);
    result.success = !diff.removed.empty() || !diff.added.empty();
    if (!result.success) {
        result.statusMessage = std::string(intersectionRepairMethodLabel(method)) + " changed nothing.";
    }
    return result;
}

IslandRepairPreview previewIslandRepair(
    const Mesh& mesh,
    const IntersectionIsland& island,
    IntersectionRepairMethod method,
    const CheckParams& params,
    const IntersectionRepairSettings& settings,
    std::span<const int> faceShell) {
    IslandRepairPreview preview;
    preview.method = method;
    IslandRepairResult r = repairIsland(mesh, island, method, params, settings, faceShell);
    preview.statusMessage = r.statusMessage;
    if (!r.success) {
        return preview;
    }
    preview.facesRemoved = r.removedFaces.size();
    preview.facesAdded = r.addedFaces.size();
    preview.crossingsAfter = r.crossingsAfter;
    preview.newCrossings = r.newCrossings;
    preview.volumeChange = r.volumeChange;
    preview.minNewAngleDeg = r.minNewAngleDeg;
    preview.sourceFaceIndices = std::move(r.removedFaces);
    if (!r.addedFaces.empty()) {
        preview.previewPatchMesh = extractFaces(r.mesh, r.addedFaces);
        offsetAlongNormals(preview.previewPatchMesh,
                           settings.previewOffsetFactor * preview.previewPatchMesh.boundingBox().diagonal());
    }
    preview.valid = true;
    return preview;
}

IslandBatchResult healIntersectionIslands(
    const Mesh& mesh,
    const CheckParams& params,
    std::span<const std::uint8_t> faceInScope,
    std::span<const Eigen::Index> seedFaces,
    const IntersectionRepairSettings& settings,
    std::stop_token stopToken,
    std::atomic<float>* progress) {
    IslandBatchResult result;
    result.mesh = mesh;
    const std::vector<int> faceShell = shellIdsOfFaces(mesh);
    const CategoryResult check = checkSelfIntersections(mesh, params, stopToken);
    std::vector<IntersectionIsland> islands = clusterIntersectionIslands(mesh, check, faceShell, faceInScope);
    if (!seedFaces.empty()) {
        const std::unordered_set<Eigen::Index> seeds(seedFaces.begin(), seedFaces.end());
        std::erase_if(islands, [&](const IntersectionIsland& island) {
            return std::none_of(island.faces.begin(), island.faces.end(), [&](Eigen::Index f) { return seeds.count(f) != 0; });
        });
    }
    result.islandsBefore = islands.size();
    if (islands.empty()) {
        result.statusMessage = seedFaces.empty() ? "No crossings within a shell to repair." : "The selected faces are in no repairable island.";
        return result;
    }

    // Shell by shell, in rounds of recommended method then fallbacks
    std::unordered_map<int, std::vector<IntersectionIsland>> byShell;
    for (IntersectionIsland& island : islands) {
        byShell[island.shell].push_back(std::move(island));
    }
    std::unordered_set<int> touched;
    for (const auto& [shell, list] : byShell) {
        touched.insert(shell);
    }
    Mesh assembled = extractFaces(mesh, facesWithoutShells(faceShell, touched));

    std::size_t done = 0;
    std::size_t islandsLeft = 0;
    for (auto& [shell, shellIslands] : byShell) {
        Mesh shellMesh = extractFaces(mesh, facesWithShell(faceShell, shell));
        // Seeds and later islands are tracked by geometry: indices change after every round
        std::unordered_set<FaceGeometryKey, FaceGeometryKeyHash> targetKeys;
        const std::vector<Eigen::Index> local = localIndex(mesh.numFaces(), facesWithShell(faceShell, shell));
        for (IntersectionIsland& island : shellIslands) {
            for (Eigen::Index& f : island.faces) {
                f = local[static_cast<std::size_t>(f)];
                if (!seedFaces.empty()) {
                    targetKeys.insert(faceGeometryKey(shellMesh, f));
                }
            }
        }
        std::size_t crossings = checkSelfIntersections(shellMesh, params).size();

        for (int round = 0; round < settings::kMaxIslandRepairRounds && !shellIslands.empty(); ++round) {
            if (stopToken.stop_requested()) {
                result.statusMessage = "Cancelled.";
                return result;
            }
            const VertexFaces vf(shellMesh);
            std::vector<IslandJob> jobs;
            jobs.reserve(shellIslands.size());
            // Self-union refines the whole shell: on one with this many crossings it runs for many minutes
            const bool allowSelfUnion = crossings <= settings::kMaxIslandSelfUnionCrossings;
            bool hasWork = false;
            for (const IntersectionIsland& island : shellIslands) {
                const IslandAnalysis a = analyze(shellMesh, vf, island, settings);
                auto m = islandFallbackOrder(a.recommended, island.local)[static_cast<std::size_t>(round)];
                if (m == IntersectionRepairMethod::SelfUnion && !allowSelfUnion) {
                    m = IntersectionRepairMethod::None;
                }
                hasWork = hasWork || m != IntersectionRepairMethod::None;
                jobs.push_back({island.faces, m});
            }
            if (!hasWork) {
                continue;
            }
            std::string status;
            Mesh candidate = repairShell(shellMesh, jobs, settings, true, status);
            ++result.rounds;
            if (candidate.empty()) {
                continue;
            }
            const CategoryResult after = checkSelfIntersections(candidate, params);
            if (after.size() >= crossings && crossings > 0) {
                continue; // this round made nothing better: undo it, next fallback
            }
            if (!keepsShape(shellMesh, candidate, settings)) {
                ++result.roundsRefusedForShape;
                continue; // it would remove or move real surface: undo it, next fallback
            }
            // New faces become targets too (a patch that crosses must be repaired again)
            if (!seedFaces.empty()) {
                for (const Eigen::Index f : faceDiff(shellMesh, candidate).added) {
                    targetKeys.insert(faceGeometryKey(candidate, f));
                }
            }
            shellMesh = std::move(candidate);
            crossings = after.size();
            const std::vector<int> candidateShells = shellIdsOfFaces(shellMesh);
            shellIslands = clusterIntersectionIslands(shellMesh, after, candidateShells);
            if (!seedFaces.empty()) {
                std::erase_if(shellIslands, [&](const IntersectionIsland& island) {
                    return std::none_of(island.faces.begin(), island.faces.end(), [&](Eigen::Index f) {
                        return targetKeys.count(faceGeometryKey(shellMesh, f)) != 0;
                    });
                });
            }
        }
        islandsLeft += shellIslands.size();
        appendMesh(assembled, shellMesh);
        if (progress != nullptr) {
            progress->store(static_cast<float>(++done) / static_cast<float>(byShell.size()));
        }
    }

    result.mesh = std::move(assembled);
    result.islandsAfter = islandsLeft;
    result.success = faceDiff(mesh, result.mesh).removed.size() > 0;
    result.statusMessage = "Repaired " + std::to_string(result.islandsBefore - std::min(result.islandsBefore, islandsLeft)) +
                           " of " + std::to_string(result.islandsBefore) + " islands" +
                           (islandsLeft > 0 ? " (" + std::to_string(islandsLeft) + " left)" : "") + "." +
                           (result.roundsRefusedForShape > 0
                                ? " " + std::to_string(result.roundsRefusedForShape) +
                                      (result.roundsRefusedForShape == 1 ? " repair round was undone because it"
                                                                         : " repair rounds were undone because they") +
                                      " would have changed the shape."
                                : "");
    if (!result.success) {
        result.statusMessage = "No island could be repaired.";
    }
    return result;
}

} // namespace mesheal
