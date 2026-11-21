#pragma once

#include "Convert.hpp"
#include "diagnostics/MeshHealth.hpp"
#include "diagnostics/MeshState.hpp"
#include "repair/GlobalRepair.hpp"

#include <atomic>
#include <cstdint>
#include <exception>
#include <functional>
#include <mutex>
#include <optional>
#include <stop_token>
#include <string>
#include <thread>
#include <variant>
#include <vector>

namespace mesheal::py {

/// The Python-facing repair methods, mapped to `GlobalRepairMethod`.
enum class Method { FixAll, Voxel, Isosurface, AlphaWrap, Decimate, Merge };

[[nodiscard]] GlobalRepairMethod toGlobalMethod(Method method) noexcept;

/// One finished check, as plain data (no Eigen, no Python objects).
struct CategoryOutput {
    DefectCategory category = DefectCategory::Shells;
    std::string name;
    std::size_t count = 0;
    std::size_t infoCount = 0;
    std::vector<std::int64_t> faces; ///< Sorted, unique union over all elements
    std::vector<std::string> warnings;
    bool cancelled = false;
    double elapsedMs = 0.0;
};

struct CheckOutput {
    std::vector<CategoryOutput> categories;
};

/// One Fix All step, as plain data (`FixAllStepReport`).
struct StepOutput {
    int pass = 0;
    int phase = 0;
    std::string stage;
    std::string status; ///< "applied", "unchanged", "refused" or "skipped"
    std::string detail;
    std::string reason;
    std::size_t countBefore = 0;
    std::size_t countAfter = 0;
    std::size_t facesAdded = 0;
    std::size_t facesRemoved = 0;
    double elapsedMs = 0.0;
    std::optional<MeshState> state;        ///< `trace_steps`
    std::optional<MeshState> refusedState; ///< `trace_steps`, refused steps
};

/// One Fix All pass's score at its end (`FixAllPassReport`).
struct PassOutput {
    int pass = 0;
    std::size_t closedManifold = 0;
    std::size_t oriented = 0;
    std::size_t intersecting = 0;
    std::size_t junk = 0;
    std::size_t thin = 0;
    bool newBest = false;
    bool repeated = false;
    double elapsedMs = 0.0;
};

struct RepairOutput {
    bool success = false;
    bool cancelled = false;
    std::string status;
    std::vector<double> vertices;       ///< Row-major N x 3
    std::vector<std::int32_t> faces;    ///< Row-major M x 3
    std::vector<std::int64_t> changedFaces;
    std::vector<std::int64_t> removedFaces;
    std::vector<std::string> report;
    std::vector<StepOutput> steps;          ///< Fix All: every step, skipped ones included
    std::vector<PassOutput> passes;         ///< Fix All
    int bestPass = 0;                       ///< Fix All: the pass returned (0 = the input)
    std::optional<MeshState> stateBefore;   ///< Fix All with `trace_steps`
};

/// Optional observers of a check run (a Job's progress).
struct CheckObserver {
    std::atomic<float>* categoryProgress = nullptr; ///< Progress of the running category, 0..1
    std::atomic<std::size_t>* categoriesDone = nullptr;
    std::function<void(const std::string&)> onCategory; ///< Called with the category's label before it runs
};

[[nodiscard]] CheckOutput runCheckWorker(const Mesh& mesh, const CheckParams& params,
                                         const std::vector<DefectCategory>& categories,
                                         std::stop_token stop, const CheckObserver& observer = {});

[[nodiscard]] RepairOutput runRepairWorker(const MeshInput& input, Method method, const GlobalRepairParams& params,
                                           std::stop_token stop, std::atomic<float>* progress = nullptr,
                                           FixAllProgress* fixAllProgress = nullptr);

[[nodiscard]] std::vector<DefectCategory> allCategories();

/// A check or repair running on a C++-owned thread. Python polls it; no Python object is touched by the thread.
class Job {
public:
    /// Inputs are already copied (the GIL was held to copy them).
    static std::unique_ptr<Job> startCheck(Mesh mesh, CheckParams params, std::vector<DefectCategory> categories);
    static std::unique_ptr<Job> startRepair(MeshInput input, Method method, GlobalRepairParams params);

    Job(const Job&) = delete;
    Job& operator=(const Job&) = delete;
    ~Job();

    [[nodiscard]] float progress() const;
    [[nodiscard]] std::string statusText() const;
    [[nodiscard]] bool done() const noexcept { return done_.load(); }
    void cancel() { thread_.request_stop(); }
    /// Blocks (GIL released by the caller) until done or `timeoutSeconds` passes; negative = no limit.
    [[nodiscard]] bool wait(double timeoutSeconds);

    /// Throws std::runtime_error if not done or if the run threw.
    [[nodiscard]] const std::variant<std::monostate, CheckOutput, RepairOutput>& result() const;

private:
    Job() = default;

    bool isCheck_ = false;
    std::atomic<float> repairProgress_{0.0f};
    std::atomic<float> categoryProgress_{0.0f};
    std::atomic<std::size_t> categoriesDone_{0};
    std::size_t categoryTotal_ = 1;
    FixAllProgress fixAllProgress_;
    bool usesFixAllProgress_ = false;
    mutable std::mutex mutex_;
    std::string status_;
    std::atomic<bool> done_{false};
    std::variant<std::monostate, CheckOutput, RepairOutput> result_;
    std::exception_ptr error_;
    std::string errorText_;
    std::jthread thread_; ///< Last: joined first, before the members it uses are destroyed
};

} // namespace mesheal::py
