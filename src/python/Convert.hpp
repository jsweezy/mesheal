#pragma once

#include "mesh/Mesh.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace mesheal::py {

namespace nb = nanobind;

using VertexArray = nb::ndarray<const double, nb::shape<-1, 3>, nb::c_contig, nb::device::cpu>;
using FaceArray = nb::ndarray<const std::int32_t, nb::shape<-1, 3>, nb::c_contig, nb::device::cpu>;
using MaskArray = nb::ndarray<const std::uint8_t, nb::ndim<1>, nb::c_contig, nb::device::cpu>;

/// A mesh copied out of numpy, plus the optional face scope (empty = whole mesh).
struct MeshInput {
    Mesh mesh;
    std::vector<std::uint8_t> mask;
};

/// Copies the arrays into a column-major `Mesh` (normals computed). Raises ValueError on a face index
/// outside [0, N) or a mask whose length is not the face count. Call with the GIL held.
[[nodiscard]] MeshInput makeInput(const VertexArray& vertices, const FaceArray& faces, const std::optional<MaskArray>& mask);

/// New numpy arrays (row-major copies) from plain buffers.
[[nodiscard]] nb::object makeVertexArray(const std::vector<double>& rowMajorXyz);
[[nodiscard]] nb::object makeFaceArray(const std::vector<std::int32_t>& rowMajorIdx);
[[nodiscard]] nb::object makeIndexArray(const std::vector<std::int64_t>& values);

} // namespace mesheal::py
