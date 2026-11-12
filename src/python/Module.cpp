#include "Convert.hpp"
#include "Job.hpp"

#include "mesheal/Version.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/unique_ptr.h>
#include <nanobind/stl/vector.h>

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

    nb::class_<RepairOutput>(m, "RepairResult")
        .def_ro("success", &RepairOutput::success)
        .def_ro("cancelled", &RepairOutput::cancelled)
        .def_ro("status", &RepairOutput::status)
        .def_prop_ro("vertices", [](const RepairOutput& r) { return makeVertexArray(r.vertices); })
        .def_prop_ro("faces", [](const RepairOutput& r) { return makeFaceArray(r.faces); })
        .def_prop_ro("changed_faces", [](const RepairOutput& r) { return makeIndexArray(r.changedFaces); })
        .def_prop_ro("removed_faces", [](const RepairOutput& r) { return makeIndexArray(r.removedFaces); })
        .def_ro("report", &RepairOutput::report);

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
