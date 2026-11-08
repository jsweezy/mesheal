#include "app/RepairJobRunner.hpp"

#include <utility>

namespace mesheal {

RepairJobRunner::~RepairJobRunner() {
    cancel();
}

void RepairJobRunner::start(RepairRequest request) {
    cancel();
    total_ = request.methods.size();
    done_ = 0;
    methodProgress_ = 0.0f;
    stopRequested_ = false;
    auto fixAllProgress = std::make_shared<FixAllProgress>();
    fixAllProgress->observer = [this](float value, const std::string&) { methodProgress_ = value; };
    {
        std::lock_guard lock(mutex_);
        fixAllProgress_ = fixAllProgress;
        finished_.clear(); // results of the job this one replaces are never handed over
    }
    busy_ = true;
    worker_ = std::jthread(
        [this](std::stop_token stopToken, RepairRequest req, std::shared_ptr<FixAllProgress> p) {
            run(stopToken, std::move(req), std::move(p));
        },
        std::move(request), std::move(fixAllProgress));
}

void RepairJobRunner::cancel() {
    if (worker_.joinable()) {
        worker_.request_stop();
        worker_.join();
    }
    busy_ = false;
    stopRequested_ = false;
    current_ = -1;
}

void RepairJobRunner::requestStop() {
    std::lock_guard lock(mutex_);
    if (worker_.joinable() && busy_.load()) {
        worker_.request_stop();
        stopRequested_ = true;
    }
    finished_.clear();
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

std::string RepairJobRunner::progressText() const {
    if (current_.load() != static_cast<int>(GlobalRepairMethod::FixAll)) {
        return {};
    }
    std::shared_ptr<FixAllProgress> p;
    {
        std::lock_guard lock(mutex_);
        p = fixAllProgress_;
    }
    return p != nullptr ? p->text() : std::string();
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

void RepairJobRunner::run(std::stop_token stopToken, RepairRequest request, std::shared_ptr<FixAllProgress> fixAllProgress) {
    for (const GlobalRepairMethod method : request.methods) {
        if (stopToken.stop_requested()) {
            break;
        }
        current_ = static_cast<int>(method);
        const FixAllRunInput fixAll{request.fixAllScopeKind, fixAllProgress.get(), request.fixAllHooks.get()};
        RepairCandidate candidate =
            request.rebuildBase != nullptr
                ? computeShellRebuildCandidate(*request.mesh, request.faceInScope, *request.rebuildBase, request.rebuildShell,
                                               method, request.params, stopToken, &methodProgress_)
                : runGlobalRepair(*request.mesh, request.faceInScope, method, request.params, stopToken, &methodProgress_,
                                  &fixAll);
        {
            // Checked under the lock: a `requestStop()` either sees this candidate (and drops it) or stops it here
            std::lock_guard lock(mutex_);
            if (candidate.cancelled || stopToken.stop_requested()) {
                break;
            }
            finished_.push_back({request.meshGeneration, std::move(candidate), request.rebuildBase != nullptr});
        }
        methodProgress_ = 0.0f;
        ++done_;
    }
    current_ = -1;
    busy_ = false;
}

} // namespace mesheal
