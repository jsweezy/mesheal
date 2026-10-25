#pragma once

#include "diagnostics/MeshHealth.hpp"
#include "mesh/Mesh.hpp"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>
#include <vector>

namespace ogun {

/// One background job: run `categories` in order on an immutable mesh snapshot.
struct CheckRequest {
    std::uint64_t meshGeneration = 0;
    std::shared_ptr<const Mesh> mesh;
    std::vector<DefectCategory> categories;
    CheckParams params{};
};

/// Runs Mesh Health checks on a `std::jthread` (UI_DESIGN.md §11.2). Finished results are collected
/// under a mutex and handed to the main thread by `takeFinished()`; cancel uses the stop token.
class HealthCheckRunner {
public:
    HealthCheckRunner() = default;
    HealthCheckRunner(const HealthCheckRunner&) = delete;
    HealthCheckRunner& operator=(const HealthCheckRunner&) = delete;
    ~HealthCheckRunner();

    /// Cancels any running job and starts `request`.
    void start(CheckRequest request);

    /// Requests a stop and waits for the worker to exit. Unfinished categories produce no result.
    void cancel();

    /// Blocks until the current job is done (tests).
    void waitIdle();

    [[nodiscard]] bool busy() const noexcept {
        return busy_.load();
    }

    /// Overall progress of the current job in [0, 1].
    [[nodiscard]] float progress() const noexcept;

    /// Category being checked right now (empty when idle).
    [[nodiscard]] std::optional<DefectCategory> currentCategory() const noexcept;

    /// Moves out every result finished since the last call.
    [[nodiscard]] std::vector<CategoryResult> takeFinished();

private:
    void run(std::stop_token stopToken, CheckRequest request);

    std::jthread worker_;
    std::atomic<bool> busy_{false};
    std::atomic<std::size_t> total_{0};
    std::atomic<std::size_t> done_{0};
    std::atomic<int> current_{-1};
    std::atomic<float> categoryProgress_{0.0f};
    mutable std::mutex mutex_;
    std::vector<CategoryResult> finished_;
};

} // namespace ogun
