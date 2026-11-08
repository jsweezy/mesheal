#include "Convert.hpp"

#include <Eigen/Core>

#include <algorithm>
#include <stdexcept>
#include <utility>

namespace mesheal::py {

namespace {

template <class T>
nb::object takeVector(std::vector<T> values, std::size_t cols) {
    auto* owner = new std::vector<T>(std::move(values));
    nb::capsule capsule(owner, [](void* p) noexcept { delete static_cast<std::vector<T>*>(p); });
    const std::size_t rows = cols == 0 ? owner->size() : owner->size() / cols;
    if (cols == 0) {
        return nb::cast(nb::ndarray<nb::numpy, T, nb::ndim<1>>(owner->data(), {rows}, capsule));
    }
    return nb::cast(nb::ndarray<nb::numpy, T, nb::ndim<2>>(owner->data(), {rows, cols}, capsule));
}

using RowMajorD = Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor>;
using RowMajorI = Eigen::Matrix<std::int32_t, Eigen::Dynamic, 3, Eigen::RowMajor>;

} // namespace

MeshInput makeInput(const VertexArray& vertices, const FaceArray& faces, const std::optional<MaskArray>& mask) {
    const std::size_t nv = vertices.shape(0);
    const std::size_t nf = faces.shape(0);

    const std::int32_t* idx = faces.data();
    for (std::size_t i = 0; i < nf * 3; ++i) {
        if (idx[i] < 0 || static_cast<std::size_t>(idx[i]) >= nv) {
            throw nb::value_error("faces contains a vertex index outside [0, number of vertices)");
        }
    }
    if (mask && mask->shape(0) != nf) {
        throw nb::value_error("face_in_scope must have one entry per face");
    }

    MeshInput in;
    in.mesh.V = Eigen::Map<const RowMajorD>(vertices.data(), static_cast<Eigen::Index>(nv), 3);
    in.mesh.F = Eigen::Map<const RowMajorI>(faces.data(), static_cast<Eigen::Index>(nf), 3);
    in.mesh.recomputeNormals();
    if (mask) {
        in.mask.assign(mask->data(), mask->data() + nf);
    }
    return in;
}

nb::object makeVertexArray(const std::vector<double>& v) {
    return takeVector<double>(v, 3);
}

nb::object makeFaceArray(const std::vector<std::int32_t>& v) {
    return takeVector<std::int32_t>(v, 3);
}

nb::object makeIndexArray(const std::vector<std::int64_t>& v) {
    return takeVector<std::int64_t>(v, 0);
}

} // namespace mesheal::py
