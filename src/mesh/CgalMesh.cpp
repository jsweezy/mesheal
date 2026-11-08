#include "mesh/CgalMesh.hpp"

#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <CGAL/Polygon_mesh_processing/orient_polygon_soup.h>
#include <CGAL/Polygon_mesh_processing/polygon_soup_to_polygon_mesh.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace mesheal {

namespace PMP = CGAL::Polygon_mesh_processing;

namespace {

std::array<CgalVertexIndex, 3> faceVertices(const CgalSurfaceMesh& mesh, CgalFaceIndex f) {
    const CgalHalfedgeIndex h0 = mesh.halfedge(f);
    const CgalHalfedgeIndex h1 = mesh.next(h0);
    const CgalHalfedgeIndex h2 = mesh.next(h1);
    return {mesh.target(h0), mesh.target(h1), mesh.target(h2)};
}

bool buildDirect(const Mesh& mesh, CgalSurfaceMesh& out) {
    std::vector<CgalVertexIndex> verts;
    verts.reserve(static_cast<std::size_t>(mesh.numVertices()));
    for (Eigen::Index i = 0; i < mesh.numVertices(); ++i) {
        verts.push_back(out.add_vertex(CgalPoint3(mesh.V(i, 0), mesh.V(i, 1), mesh.V(i, 2))));
    }
    for (Eigen::Index f = 0; f < mesh.numFaces(); ++f) {
        const auto a = verts[static_cast<std::size_t>(mesh.F(f, 0))];
        const auto b = verts[static_cast<std::size_t>(mesh.F(f, 1))];
        const auto c = verts[static_cast<std::size_t>(mesh.F(f, 2))];
        if (out.add_face(a, b, c) == CgalSurfaceMesh::null_face()) {
            return false;
        }
    }
    return true;
}

} // namespace

bool toCgalSurfaceMesh(const Mesh& mesh, CgalSurfaceMesh& outSurfaceMesh, bool* usedSoupFallback) {
    outSurfaceMesh.clear();
    if (usedSoupFallback != nullptr) {
        *usedSoupFallback = false;
    }
    if (mesh.empty()) {
        return false;
    }
    if (mesh.F.minCoeff() < 0 || mesh.F.maxCoeff() >= mesh.numVertices()) {
        return false;
    }

    if (buildDirect(mesh, outSurfaceMesh)) {
        return true;
    }

    // Non-manifold or inconsistently wound input: repair as a polygon soup.
    outSurfaceMesh.clear();
    if (usedSoupFallback != nullptr) {
        *usedSoupFallback = true;
    }
    std::vector<CgalPoint3> points;
    points.reserve(static_cast<std::size_t>(mesh.numVertices()));
    for (Eigen::Index i = 0; i < mesh.numVertices(); ++i) {
        points.emplace_back(mesh.V(i, 0), mesh.V(i, 1), mesh.V(i, 2));
    }
    std::vector<std::vector<std::size_t>> polygons;
    polygons.reserve(static_cast<std::size_t>(mesh.numFaces()));
    for (Eigen::Index f = 0; f < mesh.numFaces(); ++f) {
        polygons.push_back({static_cast<std::size_t>(mesh.F(f, 0)),
                            static_cast<std::size_t>(mesh.F(f, 1)),
                            static_cast<std::size_t>(mesh.F(f, 2))});
    }
    PMP::orient_polygon_soup(points, polygons);
    // A face that repeats a vertex cannot be a polygon-mesh face (CGAL's precondition)
    if (!PMP::is_polygon_soup_a_polygon_mesh(polygons)) {
        return false;
    }
    PMP::polygon_soup_to_polygon_mesh(points, polygons, outSurfaceMesh);
    return outSurfaceMesh.number_of_faces() > 0;
}

Mesh fromCgalSurfaceMesh(const CgalSurfaceMesh& surfaceMesh, Unit unit, double normalOffset) {
    Mesh result;
    result.fileUnit = unit;

    // Surviving vertices referenced by a face keep their relative order in the compact index space.
    std::vector<std::array<CgalVertexIndex, 3>> triangles;
    triangles.reserve(surfaceMesh.number_of_faces());
    std::vector<bool> referenced(surfaceMesh.num_vertices() + surfaceMesh.number_of_removed_vertices(), false);
    for (const CgalFaceIndex f : surfaceMesh.faces()) {
        const auto verts = faceVertices(surfaceMesh, f);
        triangles.push_back(verts);
        for (const CgalVertexIndex v : verts) {
            referenced[static_cast<std::size_t>(v)] = true;
        }
    }
    std::vector<int> remap(referenced.size(), -1);
    std::vector<CgalVertexIndex> usedVertices;
    usedVertices.reserve(surfaceMesh.number_of_vertices());
    for (const CgalVertexIndex v : surfaceMesh.vertices()) {
        if (referenced[static_cast<std::size_t>(v)]) {
            remap[static_cast<std::size_t>(v)] = static_cast<int>(usedVertices.size());
            usedVertices.push_back(v);
        }
    }
    std::vector<std::array<int, 3>> faces;
    faces.reserve(triangles.size());
    for (const auto& tri : triangles) {
        faces.push_back({remap[static_cast<std::size_t>(tri[0])],
                         remap[static_cast<std::size_t>(tri[1])],
                         remap[static_cast<std::size_t>(tri[2])]});
    }

    result.V.resize(static_cast<Eigen::Index>(usedVertices.size()), 3);
    for (std::size_t i = 0; i < usedVertices.size(); ++i) {
        const CgalPoint3& p = surfaceMesh.point(usedVertices[i]);
        Eigen::Vector3d pos(CGAL::to_double(p.x()), CGAL::to_double(p.y()), CGAL::to_double(p.z()));
        if (normalOffset != 0.0) {
            const CgalVector3 n = PMP::compute_vertex_normal(usedVertices[i], surfaceMesh);
            const Eigen::Vector3d nd(CGAL::to_double(n.x()), CGAL::to_double(n.y()), CGAL::to_double(n.z()));
            if (std::isfinite(nd.norm()) && nd.norm() > 0.0) {
                pos += normalOffset * nd.normalized();
            }
        }
        result.V.row(static_cast<Eigen::Index>(i)) = pos.transpose();
    }

    result.F.resize(static_cast<Eigen::Index>(faces.size()), 3);
    for (std::size_t f = 0; f < faces.size(); ++f) {
        for (int k = 0; k < 3; ++k) {
            result.F(static_cast<Eigen::Index>(f), k) = faces[f][static_cast<std::size_t>(k)];
        }
    }
    result.recomputeNormals();
    return result;
}

CgalFaceIndex extractLocalPatch(
    const CgalSurfaceMesh& baseMesh,
    CgalFaceIndex centerFace,
    int numRings,
    CgalSurfaceMesh& outPatch,
    std::vector<CgalFaceIndex>* sourceFacesOut) {
    outPatch.clear();
    if (sourceFacesOut) {
        sourceFacesOut->clear();
    }
    if (centerFace == CgalSurfaceMesh::null_face() || !baseMesh.is_valid(centerFace) ||
        baseMesh.is_removed(centerFace)) {
        return CgalSurfaceMesh::null_face();
    }

    // Grow the face set ring by ring: each ring adds every face touching a vertex of the set.
    std::unordered_set<CgalSurfaceMesh::size_type> faceSet{static_cast<CgalSurfaceMesh::size_type>(centerFace)};
    std::vector<CgalFaceIndex> faceOrder{centerFace};
    std::size_t frontier = 0;
    for (int ring = 0; ring < numRings; ++ring) {
        const std::size_t end = faceOrder.size();
        for (std::size_t i = frontier; i < end; ++i) {
            for (const CgalVertexIndex v : faceVertices(baseMesh, faceOrder[i])) {
                for (const CgalFaceIndex nf : CGAL::faces_around_target(baseMesh.halfedge(v), baseMesh)) {
                    if (nf != CgalSurfaceMesh::null_face() &&
                        faceSet.insert(static_cast<CgalSurfaceMesh::size_type>(nf)).second) {
                        faceOrder.push_back(nf);
                    }
                }
            }
        }
        frontier = end;
    }

    std::unordered_map<CgalSurfaceMesh::size_type, CgalVertexIndex> vmap;
    CgalFaceIndex mappedCenter = CgalSurfaceMesh::null_face();
    for (const CgalFaceIndex f : faceOrder) {
        std::array<CgalVertexIndex, 3> tri;
        const auto verts = faceVertices(baseMesh, f);
        for (std::size_t k = 0; k < 3; ++k) {
            const CgalVertexIndex v = verts[k];
            auto it = vmap.find(static_cast<CgalSurfaceMesh::size_type>(v));
            if (it == vmap.end()) {
                it = vmap.emplace(static_cast<CgalSurfaceMesh::size_type>(v), outPatch.add_vertex(baseMesh.point(v))).first;
            }
            tri[k] = it->second;
        }
        const CgalFaceIndex added = outPatch.add_face(tri);
        if (sourceFacesOut && added != CgalSurfaceMesh::null_face()) {
            sourceFacesOut->push_back(f);
        }
        if (f == centerFace) {
            mappedCenter = added;
        }
    }
    return mappedCenter;
}

} // namespace mesheal
