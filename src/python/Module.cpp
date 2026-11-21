#include "Convert.hpp"
#include "Job.hpp"

#include "diagnostics/MeshState.hpp"
#include "mesheal/Version.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/unique_ptr.h>
#include <nanobind/stl/vector.h>

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <variant>

namespace nb = nanobind;
using namespace nb::literals;
using namespace mesheal;
using namespace mesheal::py;

void bindParams(nb::module_& m); // Params.cpp

namespace {

nb::list checkList(const CheckOutput& out) {
    nb::list list;
    for (const CategoryOutput& c : out.categories) {
        list.append(nb::cast(c));
    }
    return list;
}

nb::list syncCheck(const VertexArray& vertices, const FaceArray& faces, const CheckParams& params,
                   const std::optional<std::vector<DefectCategory>>& categories) {
    MeshInput in = makeInput(vertices, faces, std::nullopt);
    const std::vector<DefectCategory> cats = categories ? *categories : allCategories();
    CheckOutput out;
    {
        nb::gil_scoped_release release;
        out = runCheckWorker(in.mesh, params, cats, {});
    }
    return checkList(out);
}

RepairOutput syncRepair(const VertexArray& vertices, const FaceArray& faces, Method method,
                        const GlobalRepairParams& params, const std::optional<MaskArray>& mask) {
    MeshInput in = makeInput(vertices, faces, mask);
    RepairOutput out;
    {
        nb::gil_scoped_release release;
        out = runRepairWorker(in, method, params, {});
    }
    return out;
}

nb::dict kindCounts(const std::array<std::size_t, kCrossingKindCount>& counts) {
    nb::dict d;
    for (std::size_t k = 0; k < kCrossingKindCount; ++k) {
        d[nb::str(std::string(crossingKindName(static_cast<CrossingKind>(k))).c_str())] = counts[k];
    }
    return d;
}

MeshState syncMeasure(const VertexArray& vertices, const FaceArray& faces, const CheckParams& params) {
    MeshInput in = makeInput(vertices, faces, std::nullopt);
    nb::gil_scoped_release release;
    return measureMeshState(in.mesh, params);
}

/// Crossing pairs as numpy columns: a, b (face indices), kind (index into CROSSING_KINDS), shell_a, shell_b.
nb::dict syncCrossingPairs(const VertexArray& vertices, const FaceArray& faces, const CheckParams& params) {
    MeshInput in = makeInput(vertices, faces, std::nullopt);
    std::optional<std::vector<CrossingPair>> pairs;
    {
        nb::gil_scoped_release release;
        pairs = classifyCrossings(in.mesh, params);
    }
    std::vector<std::int64_t> a, b, kind, shellA, shellB;
    if (pairs) {
        for (const CrossingPair& p : *pairs) {
            a.push_back(p.a);
            b.push_back(p.b);
            kind.push_back(static_cast<std::int64_t>(p.kind));
            shellA.push_back(p.shellA);
            shellB.push_back(p.shellB);
        }
    }
    nb::dict d;
    d["a"] = makeIndexArray(a);
    d["b"] = makeIndexArray(b);
    d["kind"] = makeIndexArray(kind);
    d["shell_a"] = makeIndexArray(shellA);
    d["shell_b"] = makeIndexArray(shellB);
    return d;
}

nb::object syncShellIds(const VertexArray& vertices, const FaceArray& faces) {
    MeshInput in = makeInput(vertices, faces, std::nullopt);
    std::vector<int> ids;
    {
        nb::gil_scoped_release release;
        ids = shellIndexOfFaces(in.mesh);
    }
    return makeIndexArray(std::vector<std::int64_t>(ids.begin(), ids.end()));
}

} // namespace

NB_MODULE(_mesheal, m) {
    m.doc() = "Mesheal mesh checks and repairs";
    m.def("version", [] { return std::string(mesheal::kVersionString); });
    // True when CGAL's parallel algorithms run on oneTBB (the Blender 5.1+ build uses Blender's TBB)
#ifdef CGAL_LINKED_WITH_TBB
    m.attr("tbb") = true;
#else
    m.attr("tbb") = false;
#endif

    nb::enum_<DefectCategory>(m, "Category")
        .value("SHELLS", DefectCategory::Shells)
        .value("OPEN_EDGES", DefectCategory::OpenEdges)
        .value("NON_MANIFOLD_EDGES", DefectCategory::NonManifoldEdges)
        .value("NON_MANIFOLD_VERTICES", DefectCategory::NonManifoldVertices)
        .value("FLIPPED_NORMALS", DefectCategory::FlippedNormals)
        .value("INVERTED_SHELLS", DefectCategory::InvertedShells)
        .value("INTERSECTIONS", DefectCategory::Intersections)
        .value("THIN_FACES", DefectCategory::ThinFaces)
        .value("DEGENERATE", DefectCategory::Degenerate)
        .value("DUPLICATES", DefectCategory::Duplicates);

    nb::enum_<Method>(m, "Method")
        .value("FIX_ALL", Method::FixAll)
        .value("VOXEL", Method::Voxel)
        .value("ISOSURFACE", Method::Isosurface)
        .value("ALPHA_WRAP", Method::AlphaWrap)
        .value("DECIMATE", Method::Decimate)
        .value("MERGE", Method::Merge);

    bindParams(m);

    nb::class_<CategoryOutput>(m, "CategoryResult")
        .def_ro("category", &CategoryOutput::category)
        .def_ro("name", &CategoryOutput::name)
        .def_ro("count", &CategoryOutput::count)
        .def_ro("info_count", &CategoryOutput::infoCount)
        .def_prop_ro("faces", [](const CategoryOutput& c) { return makeIndexArray(c.faces); })
        .def_ro("warnings", &CategoryOutput::warnings)
        .def_ro("cancelled", &CategoryOutput::cancelled)
        .def_ro("elapsed_ms", &CategoryOutput::elapsedMs);

    nb::list kinds;
    for (std::size_t k = 0; k < kCrossingKindCount; ++k) {
        kinds.append(nb::str(std::string(crossingKindName(static_cast<CrossingKind>(k))).c_str()));
    }
    m.attr("CROSSING_KINDS") = kinds;

    nb::class_<MeshState>(m, "MeshState")
        .def_ro("faces", &MeshState::faces)
        .def_ro("vertices", &MeshState::vertices)
        .def_ro("shells", &MeshState::shells)
        .def_ro("volume", &MeshState::volume)
        .def_ro("open_edges", &MeshState::openEdges)
        .def_ro("non_manifold_edges", &MeshState::nonManifoldEdges)
        .def_ro("non_manifold_vertices", &MeshState::nonManifoldVertices)
        .def_ro("flipped_faces", &MeshState::flippedFaces)
        .def_ro("inverted_shells", &MeshState::invertedShells)
        .def_ro("degenerate_faces", &MeshState::degenerateFaces)
        .def_ro("duplicate_faces", &MeshState::duplicateFaces)
        .def_ro("thin_faces", &MeshState::thinFaces)
        .def_ro("crossing_faces", &MeshState::crossingFaces)
        .def_prop_ro("pairs_within_shell", [](const MeshState& s) { return kindCounts(s.pairsWithinShell); })
        .def_prop_ro("pairs_between_shells", [](const MeshState& s) { return kindCounts(s.pairsBetweenShells); })
        .def_ro("complete", &MeshState::complete);

    nb::class_<StepOutput>(m, "FixAllStep")
        .def_ro("pass_number", &StepOutput::pass)
        .def_ro("phase", &StepOutput::phase)
        .def_ro("stage", &StepOutput::stage)
        .def_ro("status", &StepOutput::status)
        .def_ro("detail", &StepOutput::detail)
        .def_ro("reason", &StepOutput::reason)
        .def_ro("count_before", &StepOutput::countBefore)
        .def_ro("count_after", &StepOutput::countAfter)
        .def_ro("faces_added", &StepOutput::facesAdded)
        .def_ro("faces_removed", &StepOutput::facesRemoved)
        .def_ro("elapsed_ms", &StepOutput::elapsedMs)
        .def_ro("state", &StepOutput::state)
        .def_ro("refused_state", &StepOutput::refusedState);

    nb::class_<PassOutput>(m, "FixAllPass")
        .def_ro("pass_number", &PassOutput::pass)
        .def_ro("closed_manifold", &PassOutput::closedManifold)
        .def_ro("oriented", &PassOutput::oriented)
        .def_ro("intersecting", &PassOutput::intersecting)
        .def_ro("junk", &PassOutput::junk)
        .def_ro("thin", &PassOutput::thin)
        .def_ro("new_best", &PassOutput::newBest)
        .def_ro("repeated", &PassOutput::repeated)
        .def_ro("elapsed_ms", &PassOutput::elapsedMs);

    nb::class_<RepairOutput>(m, "RepairResult")
        .def_ro("success", &RepairOutput::success)
        .def_ro("cancelled", &RepairOutput::cancelled)
        .def_ro("status", &RepairOutput::status)
        .def_prop_ro("vertices", [](const RepairOutput& r) { return makeVertexArray(r.vertices); })
        .def_prop_ro("faces", [](const RepairOutput& r) { return makeFaceArray(r.faces); })
        .def_prop_ro("changed_faces", [](const RepairOutput& r) { return makeIndexArray(r.changedFaces); })
        .def_prop_ro("removed_faces", [](const RepairOutput& r) { return makeIndexArray(r.removedFaces); })
        .def_ro("report", &RepairOutput::report)
        .def_ro("steps", &RepairOutput::steps)
        .def_ro("passes", &RepairOutput::passes)
        .def_ro("best_pass", &RepairOutput::bestPass)
        .def_ro("state_before", &RepairOutput::stateBefore);

    nb::class_<Job>(m, "Job")
        .def_prop_ro("progress", &Job::progress)
        .def_prop_ro("status_text", &Job::statusText)
        .def_prop_ro("done", &Job::done)
        .def("cancel", &Job::cancel)
        .def("wait", &Job::wait, "timeout"_a = -1.0, nb::call_guard<nb::gil_scoped_release>())
        .def("result", [](Job& job) -> nb::object {
            const auto& r = job.result();
            if (const auto* c = std::get_if<CheckOutput>(&r)) {
                return checkList(*c);
            }
            if (const auto* rep = std::get_if<RepairOutput>(&r)) {
                return nb::cast(*rep);
            }
            throw std::runtime_error("job has no result");
        });

    m.def("measure", &syncMeasure, "vertices"_a, "faces"_a, "params"_a = CheckParams{},
          "Every Mesh Health count of the mesh, plus its crossing face pairs by kind, within a shell and between shells.");
    m.def("crossing_pairs", &syncCrossingPairs, "vertices"_a, "faces"_a, "params"_a = CheckParams{},
          "The face pairs the intersection check reports: dict of arrays a, b, kind (index into CROSSING_KINDS), "
          "shell_a, shell_b.");
    m.def("shell_ids", &syncShellIds, "vertices"_a, "faces"_a,
          "Shell index of every face (0 = the largest shell, as in the Shells panel).");
    m.def("check", &syncCheck, "vertices"_a, "faces"_a, "params"_a = CheckParams{}, "categories"_a = nb::none());
    m.def("repair", &syncRepair, "vertices"_a, "faces"_a, "method"_a, "params"_a = GlobalRepairParams{},
          "face_in_scope"_a = nb::none());

    m.def(
        "start_check",
        [](const VertexArray& vertices, const FaceArray& faces, const CheckParams& params,
           const std::optional<std::vector<DefectCategory>>& categories) {
            MeshInput in = makeInput(vertices, faces, std::nullopt);
            return Job::startCheck(std::move(in.mesh), params, categories ? *categories : allCategories());
        },
        "vertices"_a, "faces"_a, "params"_a = CheckParams{}, "categories"_a = nb::none());
    m.def(
        "start_repair",
        [](const VertexArray& vertices, const FaceArray& faces, Method method, const GlobalRepairParams& params,
           const std::optional<MaskArray>& mask) {
            return Job::startRepair(makeInput(vertices, faces, mask), method, params);
        },
        "vertices"_a, "faces"_a, "method"_a, "params"_a = GlobalRepairParams{}, "face_in_scope"_a = nb::none());
}
