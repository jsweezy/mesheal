#include "app/FixAllPlanRunner.hpp"

#include <utility>

namespace mesheal {

FinishedFixAllPlan computeFixAllPlan(const FixAllPlanRequest& request, std::stop_token stopToken) {
    FinishedFixAllPlan finished;
    finished.meshGeneration = request.meshGeneration;
    finished.scopeSerial = request.scopeSerial;
    if (request.mesh == nullptr || request.mesh->empty()) {
        finished.merge.complete = false;
        return finished;
    }
    finished.weld = planFixAllWeld(*request.mesh, request.scope, request.weldTolerance);
    finished.merge = planMerge(*request.mesh, request.scope, request.mergeParams, stopToken);
    return finished;
}

FixAllPlanRunner::~FixAllPlanRunner() {
    cancel();
}

void FixAllPlanRunner::start(FixAllPlanRequest request) {
    cancel();
    busy_ = true;
    worker_ = std::jthread(
        [this](std::stop_token stopToken, FixAllPlanRequest req) {
            FinishedFixAllPlan plan = computeFixAllPlan(req, stopToken);
            {
                std::lock_guard lock(mutex_);
                if (!stopToken.stop_requested() && plan.merge.complete) {
                    finished_ = std::move(plan);
                }
            }
            busy_ = false;
        },
        std::move(request));
}

void FixAllPlanRunner::cancel() {
    if (worker_.joinable()) {
        worker_.request_stop();
        worker_.join();
    }
    busy_ = false;
    std::lock_guard lock(mutex_);
    finished_.reset();
}

void FixAllPlanRunner::waitIdle() {
    if (worker_.joinable()) {
        worker_.join();
    }
}

bool FixAllPlanRunner::pending() const {
    if (busy_.load()) {
        return true;
    }
    std::lock_guard lock(mutex_);
    return finished_.has_value();
}

std::optional<FinishedFixAllPlan> FixAllPlanRunner::takeFinished() {
    std::lock_guard lock(mutex_);
    return std::exchange(finished_, std::nullopt);
}

} // namespace mesheal
