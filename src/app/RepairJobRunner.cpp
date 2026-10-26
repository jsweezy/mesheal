#include "app/RepairJobRunner.hpp"

#include <utility>

namespace ogun {

RepairJobRunner::~RepairJobRunner() {
    cancel();
}

void RepairJobRunner::start(RepairRequest request) {
    cancel();
    total_ = request.methods.size();
    done_ = 0;
    methodProgress_ = 0.0f;
    busy_ = true;
    worker_ = std::jthread([this](std::stop_token stopToken, RepairRequest req) { run(stopToken, std::move(req)); },
                           std::move(request));
}

void RepairJobRunner::cancel() {
    if (worker_.joinable()) {
        worker_.request_stop();
        worker_.join();
    }
    busy_ = false;
    current_ = -1;
}

void RepairJobRunner::waitIdle() {
    if (worker_.joinable()) {
        worker_.join();
    }
}

float RepairJobRunner::progress() const noexcept {
    const std::size_t total = total_.load();
    if (total == 0) {
        return 0.0f;
    }
    return (static_cast<float>(done_.load()) + methodProgress_.load()) / static_cast<float>(total);
}

std::optional<GlobalRepairMethod> RepairJobRunner::currentMethod() const noexcept {
    const int current = current_.load();
    if (current < 0) {
        return std::nullopt;
    }
    return static_cast<GlobalRepairMethod>(current);
}

std::vector<FinishedCandidate> RepairJobRunner::takeFinished() {
    std::lock_guard lock(mutex_);
    return std::exchange(finished_, {});
}

void RepairJobRunner::run(std::stop_token stopToken, RepairRequest request) {
    for (const GlobalRepairMethod method : request.methods) {
        if (stopToken.stop_requested()) {
            break;
        }
        current_ = static_cast<int>(method);
        RepairCandidate candidate = runGlobalRepair(*request.mesh, request.faceInScope, method, request.params, stopToken, &methodProgress_);
        if (candidate.cancelled || stopToken.stop_requested()) {
            break;
        }
        {
            std::lock_guard lock(mutex_);
            finished_.push_back({request.meshGeneration, std::move(candidate)});
        }
        methodProgress_ = 0.0f;
        ++done_;
    }
    current_ = -1;
    busy_ = false;
}

} // namespace ogun
