#pragma once

#include <cstdint>
#include <limits>

namespace ogun {

/// What happened in the current frame, as seen by the frame pacer.
struct FrameActivity {
    bool input = false;     ///< Input events, a held button, an active widget or a redraw request
    bool animating = false; ///< A background job, camera flight, text caret or Always redraw: keep the full rate
    /// Seconds until something changes on its own (a status message expiring); infinity when nothing will.
    double secondsToDeadline = std::numeric_limits<double>::infinity();
};

/// How the main loop should wait before the next frame.
enum class FrameWaitKind : std::uint8_t {
    None,          ///< Go straight on (the frame budget is already used up)
    Sleep,         ///< Sleep the rest of the frame budget; input waits for the next frame
    WaitForEvents, ///< Idle: block until input arrives or `seconds` pass
};

struct FrameWait {
    FrameWaitKind kind = FrameWaitKind::None;
    double seconds = 0.0;
};

/// Paces Polyscope's main loop, which otherwise builds and draws every frame at `maxFPS` and fills the
/// rest of the frame with a busy yield loop. While there is input, a job or an animation (and for a
/// short grace period after), frames run at the full rate with the gap slept rather than spun; when
/// idle, the loop blocks on window events with a long timeout.
class FramePacer {
public:
    /// Called once per frame, at its start: `now` in seconds on a monotonic clock, `maxFps` as
    /// `polyscope::options::maxFPS` (-1: uncapped).
    [[nodiscard]] FrameWait plan(double now, const FrameActivity& activity, int maxFps);

    /// True while the last plan ran at the full frame rate.
    [[nodiscard]] bool active() const noexcept { return active_; }

private:
    double lastActivity_ = -std::numeric_limits<double>::infinity();
    double lastFrameStart_ = -std::numeric_limits<double>::infinity();
    bool active_ = true;
};

} // namespace ogun
