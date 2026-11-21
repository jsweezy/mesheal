// The exact self-union lives in its own file: libigl's copyleft CGAL booleans are header-only and
// slow to compile, so only this translation unit pays for them.

#include "repair/ExactUnion.hpp"

#include "diagnostics/ShellRelations.hpp"
#include "repair/RepairMetrics.hpp"

// libigl 2.5 still includes CGAL headers that CGAL 6 marks deprecated; they work, so only the warning goes
#ifndef CGAL_NO_DEPRECATION_WARNINGS
#define CGAL_NO_DEPRECATION_WARNINGS
#endif
#include <igl/copyleft/cgal/mesh_boolean.h>
#include <igl/piecewise_constant_winding_number.h>

#include <algorithm>
#include <exception>
#include <string>

namespace mesheal {

ExactUnionResult exactSelfUnion(const Mesh& mesh, std::size_t maxFaces, std::stop_token stopToken) {
    ExactUnionResult result;
    if (stopToken.stop_requested()) {
        result.status = ExactUnionStatus::Cancelled;
        result.message = "Cancelled.";
        return result;
    }
    if (static_cast<std::size_t>(mesh.numFaces()) > maxFaces) {
        result.status = ExactUnionStatus::TooLarge;
        result.message = "it has more than " + std::to_string(maxFaces) + " faces";
        return result;
    }
    if (mesh.numFaces() == 0 || !igl::piecewise_constant_winding_number(mesh.F)) {
        result.status = ExactUnionStatus::NotClosed;
        result.message = "it is not closed and consistently wound";
        return result;
    }
    const Eigen::MatrixXd noVertices(0, 3);
    const Eigen::MatrixXi noFaces(0, 3);
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    Eigen::VectorXi birthFace;
    try {
        if (!igl::copyleft::cgal::mesh_boolean(mesh.V, mesh.F, noVertices, noFaces, igl::MESH_BOOLEAN_TYPE_UNION, V, F,
                                               birthFace)) {
            result.message = "the exact union reported a failure";
            return result;
        }
    } catch (const std::exception& e) {
        result.message = std::string("the exact union failed: ") + e.what();
        return result;
    }
    result.status = ExactUnionStatus::United;
    result.mesh.V = std::move(V);
    result.mesh.F = std::move(F);
    result.mesh.fileUnit = mesh.fileUnit;
    result.mesh.recomputeNormals();
    return result;
}

std::size_t fillEnclosedPockets(Mesh& mesh, double* volumeOut) {
    std::size_t pockets = 0;
    double pocketVolume = 0.0;
    std::vector<Eigen::Index> outer;
    for (const std::vector<Eigen::Index>& piece : shellsLargestFirst(mesh)) {
        const double volume = signedVolume(extractFaces(mesh, piece));
        if (volume < 0.0) {
            ++pockets;
            pocketVolume -= volume;
        } else {
            outer.insert(outer.end(), piece.begin(), piece.end());
        }
    }
    if (pockets > 0) {
        std::sort(outer.begin(), outer.end());
        mesh = extractFaces(mesh, outer);
        mesh.recomputeNormals();
    }
    if (volumeOut != nullptr) {
        *volumeOut = pocketVolume;
    }
    return pockets;
}

} // namespace mesheal
