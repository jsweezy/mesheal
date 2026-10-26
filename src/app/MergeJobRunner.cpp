#include "app/MergeJobRunner.hpp"

#include <utility>

namespace ogun {

MergeJobRunner::~MergeJobRunner() {
    discard();
    waitIdle();
}

void MergeJobRunner::start(MergeRequest request) {
    waitIdle();
    {
        std::lock_guard lock(mutex_);
        finished_.reset();
    }
    discarded_ = false;
    running_ = true;
    worker_ = std::jthread([this](MergeRequest req) {
        ShellMergeResult result = mergeShells(*req.mesh, req.shells);
        std::lock_guard lock(mutex_);
        if (!discarded_.load()) {
            finished_ = FinishedMerge{req.meshGeneration, std::move(req.shellIds), std::move(result)};
        }
        running_ = false;
    }, std::move(request));
}

void MergeJobRunner::discard() {
    std::lock_guard lock(mutex_);
    discarded_ = true;
    finished_.reset();
}

void MergeJobRunner::waitIdle() {
    if (worker_.joinable()) {
        worker_.join();
    }
}

std::optional<FinishedMerge> MergeJobRunner::takeFinished() {
    std::lock_guard lock(mutex_);
    return std::exchange(finished_, std::nullopt);
}

} // namespace ogun
