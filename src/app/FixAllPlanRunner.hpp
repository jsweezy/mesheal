#pragma once

#include "mesh/Mesh.hpp"
#include "mesh/ToleranceConfig.hpp"
#include "repair/FixAll.hpp"
#include "repair/MergePlan.hpp"

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <stop_token>
#include <thread>

namespace mesheal {

/// The plan-time check of the Fix All panel, on a mesh snapshot and a scope: which shells overlap
/// (`planMerge`, which also gives the default of the *Merge shells* box) and what the weld will do.
struct FixAllPlanRequest {
    std::uint64_t meshGeneration = 0;
    std::uint64_t scopeSerial = 0; ///< The controller's count of scope changes, to drop a plan for an old scope
    std::shared_ptr<const Mesh> mesh;
    FixAllScope scope;
    MergePlanParams mergeParams{};
    ToleranceConfig weldTolerance{settings::kDefaultFixAllWeldAbsFloor, settings::kDefaultFixAllWeldRelTol};
};

struct FinishedFixAllPlan {
    std::uint64_t meshGeneration = 0;
    std::uint64_t scopeSerial = 0;
    MergePlan merge;       ///< `complete` false when stopped
    FixAllWeldPlan weld;
};

/// Works the plan out (inline; the runner calls it on its thread).
[[nodiscard]] FinishedFixAllPlan computeFixAllPlan(const FixAllPlanRequest& request, std::stop_token stopToken = {});

/// Runs `computeFixAllPlan` on a `std::jthread`; a new request or `cancel()` stops the old one first.
class FixAllPlanRunner {
public:
    FixAllPlanRunner() = default;
    FixAllPlanRunner(const FixAllPlanRunner&) = delete;
    FixAllPlanRunner& operator=(const FixAllPlanRunner&) = delete;
    ~FixAllPlanRunner();

    void start(FixAllPlanRequest request);
    /// Stops the job (the plan polls the stop token), waits for it and drops its result.
    void cancel();
    /// Blocks until the job is done (tests).
    void waitIdle();

    [[nodiscard]] bool busy() const noexcept {
        return busy_.load();
    }

    /// Running, or finished and not taken yet.
    [[nodiscard]] bool pending() const;

    /// The finished plan, once.
    [[nodiscard]] std::optional<FinishedFixAllPlan> takeFinished();

private:
    std::jthread worker_;
    std::atomic<bool> busy_{false};
    mutable std::mutex mutex_;
    std::optional<FinishedFixAllPlan> finished_;
};

} // namespace mesheal
