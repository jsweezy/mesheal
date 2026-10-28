#include "app/FramePacer.hpp"

#include "core/Settings.hpp"

#include <algorithm>

namespace ogun {

FrameWait FramePacer::plan(double now, const FrameActivity& activity, int maxFps) {
    if (activity.input || activity.animating) {
        lastActivity_ = now;
    }
    active_ = now - lastActivity_ < settings::kIdleAfterActivitySeconds;

    FrameWait wait;
    if (active_) {
        // Full rate: sleep what is left of the frame budget, so Polyscope's yield loop has nothing to spin
        if (maxFps > 0) {
            const double budget = settings::kFrameBudgetShare / static_cast<double>(maxFps);
            const double remaining = lastFrameStart_ + budget - now;
            if (remaining > 0.0) {
                wait = FrameWait{FrameWaitKind::Sleep, std::min(remaining, budget)};
            }
        }
    } else {
        // Idle: block until input, the next timed change, or the idle interval
        const double seconds = std::min(settings::kIdleFrameIntervalSeconds, std::max(activity.secondsToDeadline, 0.0));
        if (seconds > 0.0) {
            wait = FrameWait{FrameWaitKind::WaitForEvents, seconds};
        }
    }
    lastFrameStart_ = now + (wait.kind == FrameWaitKind::Sleep ? wait.seconds : 0.0);
    return wait;
}

} // namespace ogun
