#pragma once

#include "mesh/Mesh.hpp"
#include "repair/FixAll.hpp"
#include "repair/GlobalRepair.hpp"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

namespace ogun {

/// One background job: compute a "Repair All" candidate for each method on a mesh snapshot.
struct RepairRequest {
    std::uint64_t meshGeneration = 0;
    std::shared_ptr<const Mesh> mesh;
    std::vector<std::uint8_t> faceInScope;  ///< Empty: every face
    std::vector<GlobalRepairMethod> methods;
    GlobalRepairParams params{};
    FixAllScopeKind fixAllScopeKind = FixAllScopeKind::Whole; ///< Fix All: what `faceInScope` is (faces or whole shells)
    std::shared_ptr<const FixAllTestHooks> fixAllHooks;        ///< Fix All: tests only
};

/// A finished candidate with the mesh generation it was computed for.
struct FinishedCandidate {
    std::uint64_t meshGeneration = 0;
    RepairCandidate candidate;
};

/// Runs global repairs on a `std::jthread` (the same pattern as `HealthCheckRunner`): finished
/// candidates are collected under a mutex and handed to the main thread by `takeFinished()`.
class RepairJobRunner {
public:
    RepairJobRunner() = default;
    RepairJobRunner(const RepairJobRunner&) = delete;
    RepairJobRunner& operator=(const RepairJobRunner&) = delete;
    ~RepairJobRunner();

    /// Cancels any running job (waiting for it) and starts `request`.
    void start(RepairRequest request);

    /// Requests a stop and waits for the worker to exit.
    void cancel();

    /// Requests a stop without waiting: the job ends at its next stop check (Fix All: after the current
    /// Manifold union) and nothing it computes is handed over. `busy()` stays true until it has ended.
    void requestStop();

    /// Blocks until the current job is done (tests).
    void waitIdle();

    [[nodiscard]] bool busy() const noexcept {
        return busy_.load();
    }

    /// A stop was requested and the job has not ended yet.
    [[nodiscard]] bool stopping() const noexcept {
        return busy_.load() && stopRequested_.load();
    }

    /// Overall progress of the current job in [0, 1].
    [[nodiscard]] float progress() const noexcept;

    /// Fix All's progress text ("pass 2 of 3, phase 1, filling holes..."); empty for other methods.
    [[nodiscard]] std::string progressText() const;

    /// Method being computed right now (empty when idle).
    [[nodiscard]] std::optional<GlobalRepairMethod> currentMethod() const noexcept;

    /// Moves out every candidate finished since the last call.
    [[nodiscard]] std::vector<FinishedCandidate> takeFinished();

private:
    void run(std::stop_token stopToken, RepairRequest request, std::shared_ptr<FixAllProgress> fixAllProgress);

    std::jthread worker_;
    std::atomic<bool> busy_{false};
    std::atomic<bool> stopRequested_{false};
    std::atomic<std::size_t> total_{0};
    std::atomic<std::size_t> done_{0};
    std::atomic<int> current_{-1};
    std::atomic<float> methodProgress_{0.0f};
    mutable std::mutex mutex_;
    std::vector<FinishedCandidate> finished_;
    std::shared_ptr<FixAllProgress> fixAllProgress_; ///< The current job's (guarded by `mutex_`)
};

} // namespace ogun
