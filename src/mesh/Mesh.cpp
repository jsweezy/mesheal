#include "mesh/Mesh.hpp"

#include <Eigen/Geometry>

#include <cstddef>
#include <cstdint>
#include <vector>

namespace ogun {

namespace {

template <typename Precision, typename IndexType>
manifold::MeshGLP<Precision, IndexType> convertToMeshGLP(const Mesh& mesh) {
    manifold::MeshGLP<Precision, IndexType> out;
    out.numProp = 3;

    const auto nVerts = static_cast<std::size_t>(mesh.V.rows());
    out.vertProperties.resize(nVerts * 3);
    for (std::size_t i = 0; i < nVerts; ++i) {
        const auto row = static_cast<Eigen::Index>(i);
        out.vertProperties[3 * i + 0] = static_cast<Precision>(mesh.V(row, 0));
        out.vertProperties[3 * i + 1] = static_cast<Precision>(mesh.V(row, 1));
        out.vertProperties[3 * i + 2] = static_cast<Precision>(mesh.V(row, 2));
    }

    const auto nTris = static_cast<std::size_t>(mesh.F.rows());
    out.triVerts.resize(nTris * 3);
    for (std::size_t t = 0; t < nTris; ++t) {
        const auto row = static_cast<Eigen::Index>(t);
        out.triVerts[3 * t + 0] = static_cast<IndexType>(mesh.F(row, 0));
        out.triVerts[3 * t + 1] = static_cast<IndexType>(mesh.F(row, 1));
        out.triVerts[3 * t + 2] = static_cast<IndexType>(mesh.F(row, 2));
    }

    return out;
}

template <typename Precision, typename IndexType>
Mesh convertFromMeshGLP(
    const manifold::MeshGLP<Precision, IndexType>& meshGl,
    Unit unit) {
    Mesh mesh;
    mesh.fileUnit = unit;

    const std::size_t numProp = meshGl.numProp >= 3 ? static_cast<std::size_t>(meshGl.numProp) : 3;
    const std::size_t nVerts = meshGl.vertProperties.size() / numProp;
    const std::size_t nTris = meshGl.triVerts.size() / 3;

    mesh.V.resize(static_cast<Eigen::Index>(nVerts), 3);
    for (std::size_t i = 0; i < nVerts; ++i) {
        const auto row = static_cast<Eigen::Index>(i);
        mesh.V(row, 0) = static_cast<double>(meshGl.vertProperties[i * numProp + 0]);
        mesh.V(row, 1) = static_cast<double>(meshGl.vertProperties[i * numProp + 1]);
        mesh.V(row, 2) = static_cast<double>(meshGl.vertProperties[i * numProp + 2]);
    }

    mesh.F.resize(static_cast<Eigen::Index>(nTris), 3);
    for (std::size_t t = 0; t < nTris; ++t) {
        const auto row = static_cast<Eigen::Index>(t);
        mesh.F(row, 0) = static_cast<int>(meshGl.triVerts[3 * t + 0]);
        mesh.F(row, 1) = static_cast<int>(meshGl.triVerts[3 * t + 1]);
        mesh.F(row, 2) = static_cast<int>(meshGl.triVerts[3 * t + 2]);
    }

    mesh.recomputeNormals();
    return mesh;
}

} // namespace

BoundingBox Mesh::boundingBox() const {
    BoundingBox bbox;
    if (V.rows() == 0 || V.cols() < 3) {
        return bbox;
    }
    bbox.min = V.colwise().minCoeff().transpose();
    bbox.max = V.colwise().maxCoeff().transpose();
    return bbox;
}

void Mesh::recomputeNormals() {
    const Eigen::Index nFaces = F.rows();
    N.resize(nFaces, 3);
    if (nFaces == 0 || V.rows() == 0) {
        return;
    }

    for (Eigen::Index f = 0; f < nFaces; ++f) {
        const Eigen::Vector3d v0 = V.row(F(f, 0)).transpose();
        const Eigen::Vector3d v1 = V.row(F(f, 1)).transpose();
        const Eigen::Vector3d v2 = V.row(F(f, 2)).transpose();

        const Eigen::Vector3d cross = (v1 - v0).cross(v2 - v0);
        const double norm = cross.norm();
        if (norm > 0.0) {
            N.row(f) = (cross / norm).transpose();
        } else {
            N.row(f).setZero();
        }
    }
}

manifold::MeshGL Mesh::toMeshGL() const {
    return convertToMeshGLP<float, uint32_t>(*this);
}

manifold::MeshGL64 Mesh::toMeshGL64() const {
    return convertToMeshGLP<double, uint64_t>(*this);
}

Mesh Mesh::fromMeshGL(const manifold::MeshGL& meshGl, Unit unit) {
    return convertFromMeshGLP(meshGl, unit);
}

Mesh Mesh::fromMeshGL64(const manifold::MeshGL64& meshGl64, Unit unit) {
    return convertFromMeshGLP(meshGl64, unit);
}

Mesh extractFaces(const Mesh& mesh, std::span<const Eigen::Index> faces) {
    Mesh out;
    out.fileUnit = mesh.fileUnit;
    out.F.resize(static_cast<Eigen::Index>(faces.size()), 3);
    std::vector<int> remap(static_cast<std::size_t>(mesh.numVertices()), -1);
    std::vector<Eigen::Index> used;
    used.reserve(faces.size());
    for (std::size_t row = 0; row < faces.size(); ++row) {
        for (int corner = 0; corner < 3; ++corner) {
            const auto v = static_cast<std::size_t>(mesh.F(faces[row], corner));
            if (remap[v] < 0) {
                remap[v] = static_cast<int>(used.size());
                used.push_back(static_cast<Eigen::Index>(v));
            }
            out.F(static_cast<Eigen::Index>(row), corner) = remap[v];
        }
    }
    out.V.resize(static_cast<Eigen::Index>(used.size()), 3);
    for (std::size_t i = 0; i < used.size(); ++i) {
        out.V.row(static_cast<Eigen::Index>(i)) = mesh.V.row(used[i]);
    }
    out.recomputeNormals();
    return out;
}

void appendMesh(Mesh& mesh, const Mesh& part) {
    if (part.empty()) {
        return;
    }
    const Eigen::Index v0 = mesh.numVertices();
    const Eigen::Index f0 = mesh.numFaces();
    mesh.V.conservativeResize(v0 + part.numVertices(), 3);
    mesh.V.bottomRows(part.numVertices()) = part.V;
    mesh.F.conservativeResize(f0 + part.numFaces(), 3);
    mesh.F.bottomRows(part.numFaces()) = part.F.array() + static_cast<int>(v0);
    mesh.recomputeNormals();
}

manifold::MeshGL toMeshGL(const Mesh& mesh) {
    return mesh.toMeshGL();
}

Mesh fromMeshGL(const manifold::MeshGL& meshGl, Unit unit) {
    return Mesh::fromMeshGL(meshGl, unit);
}

} // namespace ogun
