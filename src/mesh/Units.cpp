#include "mesh/Units.hpp"
#include "mesh/Mesh.hpp"

namespace ogun {

void setFileUnit(Mesh& mesh, Unit newUnit) noexcept {
    mesh.fileUnit = newUnit;
}

void scaleMeshToUnit(Mesh& mesh, Unit targetUnit) {
    if (mesh.fileUnit == targetUnit) {
        return;
    }
    const double factor = unitConversionFactor(mesh.fileUnit, targetUnit);
    if (mesh.V.size() > 0) {
        mesh.V *= factor;
    }
    mesh.fileUnit = targetUnit;
}

} // namespace ogun
