#include "app/HealthCheckRunner.hpp"

#include <utility>

namespace mesheal {

HealthCheckRunner::~HealthCheckRunner() {
    cancel();
}

void HealthCheckRunner::start(CheckRequest request) {
    cancel();
    total_ = request.categories.size();
    done_ = 0;
    categoryProgress_ = 0.0f;
    busy_ = true;
    worker_ = std::jthread([this](std::stop_token stopToken, CheckRequest req) { run(stopToken, std::move(req)); },
                           std::move(request));
}

void HealthCheckRunner::cancel() {
    if (worker_.joinable()) {
        worker_.request_stop();
        worker_.join();
    }
    busy_ = false;
    current_ = -1;
}

void HealthCheckRunner::waitIdle() {
    if (worker_.joinable()) {
        worker_.join();
    }
}

float HealthCheckRunner::progress() const noexcept {
    const std::size_t total = total_.load();
    if (total == 0) {
        return 0.0f;
    }
    return (static_cast<float>(done_.load()) + categoryProgress_.load()) / static_cast<float>(total);
}

std::optional<DefectCategory> HealthCheckRunner::currentCategory() const noexcept {
    const int current = current_.load();
    if (current < 0) {
        return std::nullopt;
    }
    return static_cast<DefectCategory>(current);
}

std::vector<CategoryResult> HealthCheckRunner::takeFinished() {
    std::lock_guard lock(mutex_);
    return std::exchange(finished_, {});
}

void HealthCheckRunner::run(std::stop_token stopToken, CheckRequest request) {
    for (const DefectCategory category : request.categories) {
        if (stopToken.stop_requested()) {
            break;
        }
        current_ = static_cast<int>(category);
        CategoryResult result = runCategoryCheck(category, *request.mesh, request.params, stopToken, &categoryProgress_);
        if (result.cancelled || stopToken.stop_requested()) {
            break;
        }
        result.meshGeneration = request.meshGeneration;
        {
            std::lock_guard lock(mutex_);
            finished_.push_back(std::move(result));
        }
        categoryProgress_ = 0.0f;
        ++done_;
    }
    current_ = -1;
    busy_ = false;
}

} // namespace mesheal
