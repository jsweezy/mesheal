#include "repair/GlobalRepair.hpp"

#include <nanobind/nanobind.h>

namespace nb = nanobind;
using namespace mesheal;

void bindParams(nb::module_& m);

void bindParams(nb::module_& m) {
    nb::class_<CheckParams>(m, "CheckParams")
        .def(nb::init<>())
        .def_rw("intersection_tolerance", &CheckParams::intersectionTolerance)
        .def_prop_rw(
            "thin_max_aspect", [](const CheckParams& p) { return p.thinThresholds.maxAspectRatio; },
            [](CheckParams& p, double v) { p.thinThresholds.maxAspectRatio = v; })
        .def_rw("needle_edge_ratio", &CheckParams::needleEdgeRatio)
        .def_rw("cap_angle_deg", &CheckParams::capAngleDeg);

    nb::class_<FixAllParams>(m, "FixAllParams")
        .def(nb::init<>())
        .def_rw("passes", &FixAllParams::passes)
        .def_rw("time_budget", &FixAllParams::timeBudgetSeconds)
        .def_rw("weld", &FixAllParams::weld)
        .def_prop_rw(
            "weld_tolerance", [](const FixAllParams& p) { return p.weldTolerance.relTol; },
            [](FixAllParams& p, double v) {
                p.weldTolerance.absFloor = v;
                p.weldTolerance.relTol = v;
            })
        .def_rw("remove_tiny_shells", &FixAllParams::removeTinyShells)
        .def_rw("tiny_shell_fraction", &FixAllParams::tinyShellFraction)
        .def_rw("remove_fins", &FixAllParams::removeFins)
        .def_rw("close_cracks", &FixAllParams::closeCracks)
        .def_rw("heal_thin_faces", &FixAllParams::healThinFaces)
        .def_rw("merge_shells", &FixAllParams::mergeShells);

    nb::class_<GlobalRepairParams>(m, "RepairParams")
        .def(nb::init<>())
        .def_rw("alpha_wrap_detail", &GlobalRepairParams::alphaWrapDetail)
        .def_rw("alpha_wrap_offset_ratio", &GlobalRepairParams::alphaWrapOffsetRatio)
        .def_rw("decimate_ratio", &GlobalRepairParams::decimateRatio)
        .def_rw("merge_distance", &GlobalRepairParams::mergeDistance)
        .def_rw("merge_keep_surfaces_apart", &GlobalRepairParams::mergeKeepSurfacesApart)
        .def_rw("merge_preserve_open_edges", &GlobalRepairParams::mergePreserveOpenEdges)
        .def_rw("remesh_target_size", &GlobalRepairParams::remeshTargetSize)
        .def_rw("voxel_smoothing_passes", &GlobalRepairParams::voxelSmoothingPasses)
        .def_rw("isosurface_smoothing_passes", &GlobalRepairParams::isosurfaceSmoothingPasses)
        .def_rw("isosurface_chord_ratio", &GlobalRepairParams::isosurfaceChordRatio)
        .def_rw("simplify_rebuild", &GlobalRepairParams::simplifyRebuild)
        .def_prop_rw(
            "check", [](GlobalRepairParams& p) -> CheckParams& { return p.checkParams; },
            [](GlobalRepairParams& p, const CheckParams& v) { p.checkParams = v; }, nb::rv_policy::reference_internal)
        .def_prop_rw(
            "fix_all", [](GlobalRepairParams& p) -> FixAllParams& { return p.fixAll; },
            [](GlobalRepairParams& p, const FixAllParams& v) { p.fixAll = v; }, nb::rv_policy::reference_internal);
}
