#include "Job.hpp"

#include <nanobind/nanobind.h>

#include <algorithm>
#include <chrono>
#include <memory>
#include <stdexcept>

namespace mesheal::py {

GlobalRepairMethod toGlobalMethod(Method method) noexcept {
    switch (method) {
    case Method::FixAll: return GlobalRepairMethod::FixAll;
    case Method::Voxel: return GlobalRepairMethod::VoxelHealRemesh;
    case Method::Isosurface: return GlobalRepairMethod::PrecisionIsosurface;
    case Method::AlphaWrap: return GlobalRepairMethod::AlphaWrap;
    case Method::Decimate: return GlobalRepairMethod::Decimate;
    case Method::Merge: return GlobalRepairMethod::MergeVertices;
    }
    return GlobalRepairMethod::FixAll;
}

std::vector<DefectCategory> allCategories() {
    return {kAllDefectCategories.begin(), kAllDefectCategories.end()};
}

CheckOutput runCheckWorker(const Mesh& mesh, const CheckParams& params, const std::vector<DefectCategory>& categories,
                           std::stop_token stop, const CheckObserver& observer) {
    CheckOutput out;
    out.categories.reserve(categories.size());
    for (const DefectCategory category : categories) {
        const CategoryInfo& info = categoryInfo(category);
        if (observer.onCategory) {
            observer.onCategory(info.name);
        }
        if (observer.categoryProgress != nullptr) {
            observer.categoryProgress->store(0.0f);
        }
        const CategoryResult r = runCategoryCheck(category, mesh, params, stop, observer.categoryProgress);

        CategoryOutput o;
        o.category = category;
        o.name = info.name;
        o.count = r.size();
        o.infoCount = r.infoCount;
        o.cancelled = r.cancelled;
        o.elapsedMs = r.elapsedMs;
        o.faces.reserve(r.faces.size());
        for (const Eigen::Index f : r.faces) {
            o.faces.push_back(static_cast<std::int64_t>(f));
        }
        std::sort(o.faces.begin(), o.faces.end());
        o.faces.erase(std::unique(o.faces.begin(), o.faces.end()), o.faces.end());
        for (const CategoryWarning& w : r.warnings) {
            o.warnings.push_back(w.text);
        }
        out.categories.push_back(std::move(o));

        if (observer.categoriesDone != nullptr) {
            observer.categoriesDone->fetch_add(1);
        }
        if (stop.stop_requested()) {
            break;
        }
    }
    return out;
}

RepairOutput runRepairWorker(const MeshInput& input, Method method, const GlobalRepairParams& params,
                             std::stop_token stop, std::atomic<float>* progress, FixAllProgress* fixAllProgress) {
    FixAllRunInput fixInput;
    fixInput.scopeKind = input.mask.empty() ? FixAllScopeKind::Whole : FixAllScopeKind::Faces;
    fixInput.progress = fixAllProgress;

    // Remesh methods never take a scope.
    std::span<const std::uint8_t> scope;
    if (method == Method::FixAll || method == Method::Decimate || method == Method::Merge) {
        scope = input.mask;
    }

    const RepairCandidate c = runGlobalRepair(input.mesh, scope, toGlobalMethod(method), params, stop, progress,
                                              method == Method::FixAll ? &fixInput : nullptr);

    RepairOutput out;
    out.success = c.success;
    out.cancelled = c.cancelled;
    out.status = c.statusMessage;
    const Eigen::Index nv = c.mesh.V.rows();
    const Eigen::Index nf = c.mesh.F.rows();
    out.vertices.resize(static_cast<std::size_t>(nv) * 3);
    out.faces.resize(static_cast<std::size_t>(nf) * 3);
    for (Eigen::Index i = 0; i < nv; ++i) {
        for (Eigen::Index k = 0; k < 3; ++k) {
            out.vertices[static_cast<std::size_t>(i * 3 + k)] = c.mesh.V(i, k);
        }
    }
    for (Eigen::Index i = 0; i < nf; ++i) {
        for (Eigen::Index k = 0; k < 3; ++k) {
            out.faces[static_cast<std::size_t>(i * 3 + k)] = c.mesh.F(i, k);
        }
    }
    for (const Eigen::Index f : c.changedFaces) {
        out.changedFaces.push_back(static_cast<std::int64_t>(f));
    }
    for (const Eigen::Index f : c.removedFaces) {
        out.removedFaces.push_back(static_cast<std::int64_t>(f));
    }
    if (c.fixAll) {
        for (const FixAllReportLine& line : c.fixAll->result.report.lines) {
            out.report.push_back(line.text);
        }
    }
    return out;
}

std::unique_ptr<Job> Job::startCheck(Mesh mesh, CheckParams params, std::vector<DefectCategory> categories) {
    std::unique_ptr<Job> job(new Job());
    Job* self = job.get();
    self->isCheck_ = true;
    self->categoryTotal_ = std::max<std::size_t>(categories.size(), 1);
    self->status_ = "Starting";
    self->thread_ = std::jthread([self, mesh = std::move(mesh), params, categories = std::move(categories)](std::stop_token st) {
        try {
            CheckObserver obs;
            obs.categoryProgress = &self->categoryProgress_;
            obs.categoriesDone = &self->categoriesDone_;
            obs.onCategory = [self](const std::string& name) {
                const std::lock_guard lock(self->mutex_);
                self->status_ = name;
            };
            self->result_ = runCheckWorker(mesh, params, categories, st, obs);
        } catch (const std::exception& e) {
            self->errorText_ = e.what();
            self->error_ = std::current_exception();
        } catch (...) {
            self->errorText_ = "unknown error";
            self->error_ = std::current_exception();
        }
        self->done_.store(true);
    });
    return job;
}

std::unique_ptr<Job> Job::startRepair(MeshInput input, Method method, GlobalRepairParams params) {
    std::unique_ptr<Job> job(new Job());
    Job* self = job.get();
    self->usesFixAllProgress_ = method == Method::FixAll;
    self->status_ = globalRepairMethodLabel(toGlobalMethod(method));
    self->thread_ = std::jthread([self, input = std::move(input), method, params](std::stop_token st) {
        try {
            self->result_ = runRepairWorker(input, method, params, st, &self->repairProgress_,
                                            self->usesFixAllProgress_ ? &self->fixAllProgress_ : nullptr);
        } catch (const std::exception& e) {
            self->errorText_ = e.what();
            self->error_ = std::current_exception();
        } catch (...) {
            self->errorText_ = "unknown error";
            self->error_ = std::current_exception();
        }
        self->done_.store(true);
    });
    return job;
}

Job::~Job() {
    thread_.request_stop();
    if (thread_.joinable()) {
        // The thread never touches Python, but a join can block: let other Python threads run meanwhile.
        if (PyGILState_Check() != 0) {
            nanobind::gil_scoped_release release;
            thread_.join();
        } else {
            thread_.join();
        }
    }
}

float Job::progress() const {
    if (done_.load()) {
        return 1.0f;
    }
    float p = 0.0f;
    if (isCheck_) {
        const float finished = static_cast<float>(categoriesDone_.load());
        p = (finished + categoryProgress_.load()) / static_cast<float>(categoryTotal_);
    } else {
        p = repairProgress_.load();
        if (usesFixAllProgress_) {
            p = std::max(p, fixAllProgress_.fraction.load());
        }
    }
    return std::clamp(p, 0.0f, 1.0f);
}

std::string Job::statusText() const {
    if (!isCheck_ && usesFixAllProgress_) {
        std::string t = fixAllProgress_.text();
        if (!t.empty()) {
            return t;
        }
    }
    const std::lock_guard lock(mutex_);
    return status_;
}

bool Job::wait(double timeoutSeconds) {
    const auto start = std::chrono::steady_clock::now();
    while (!done_.load()) {
        if (timeoutSeconds >= 0.0 &&
            std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() >= timeoutSeconds) {
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return true;
}

const std::variant<std::monostate, CheckOutput, RepairOutput>& Job::result() const {
    if (!done_.load()) {
        throw std::runtime_error("job is not finished");
    }
    if (error_) {
        throw std::runtime_error(errorText_);
    }
    return result_;
}

} // namespace mesheal::py
