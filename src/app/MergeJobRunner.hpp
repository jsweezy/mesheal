#pragma once

#include "mesh/Mesh.hpp"
#include "repair/ShellMerge.hpp"

#include <Eigen/Core>

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

namespace ogun {

/// One Merge Shells job on a mesh snapshot.
struct MergeRequest {
    std::uint64_t meshGeneration = 0;
    std::shared_ptr<const Mesh> mesh;
    ShellMergeInput input;                 ///< Operands and cavities
    std::vector<std::size_t> operandIds;   ///< ShellBrowser number of each operand, for messages
    std::vector<std::size_t> cavityIds;    ///< ShellBrowser number of each cavity
};

struct FinishedMerge {
    std::uint64_t meshGeneration = 0;
    ShellMergeInput input;
    std::vector<std::size_t> operandIds;
    std::vector<std::size_t> cavityIds;
    ShellMergeResult result;
};

/// Runs the cavity-safe `mergeShells` on a `std::jthread`. Manifold's boolean cannot be interrupted, so `discard()`
/// lets the job finish and drops its result.
class MergeJobRunner {
public:
    MergeJobRunner() = default;
    MergeJobRunner(const MergeJobRunner&) = delete;
    MergeJobRunner& operator=(const MergeJobRunner&) = delete;
    ~MergeJobRunner();

    /// Starts `request` (waits for a discarded job that is still running).
    void start(MergeRequest request);

    /// Drops the job's result, even one that just finished; `busy()` turns false at once.
    void discard();

    /// Blocks until the current job is done (tests).
    void waitIdle();

    [[nodiscard]] bool busy() const noexcept {
        return running_.load() && !discarded_.load();
    }

    /// The finished job's result, once.
    [[nodiscard]] std::optional<FinishedMerge> takeFinished();

private:
    std::jthread worker_;
    std::atomic<bool> running_{false};
    std::atomic<bool> discarded_{false};
    std::mutex mutex_;
    std::optional<FinishedMerge> finished_;
};

} // namespace ogun
