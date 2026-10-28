#include "app/FramePacer.hpp"
#include "core/Settings.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

namespace {

using ogun::FrameActivity;
using ogun::FramePacer;
using ogun::FrameWait;
using ogun::FrameWaitKind;
namespace settings = ogun::settings;

FrameActivity input() {
    FrameActivity a;
    a.input = true;
    return a;
}

FrameActivity animating() {
    FrameActivity a;
    a.animating = true;
    return a;
}

constexpr int kFps = 60;
constexpr double kBudget = settings::kFrameBudgetShare / kFps;

} // namespace

TEST_CASE("The frame pacer blocks on input when idle and sleeps the frame gap when active", "[app][pacing]") {
    SECTION("With no input, job or animation it waits for events after the grace period") {
        FramePacer pacer;
        // The very first frame has no previous frame: nothing to sleep, nothing marked active yet
        FrameWait wait = pacer.plan(100.0, FrameActivity{}, kFps);
        CHECK_FALSE(pacer.active());
        CHECK(wait.kind == FrameWaitKind::WaitForEvents);
        CHECK(wait.seconds == Catch::Approx(settings::kIdleFrameIntervalSeconds));
    }

    SECTION("Input keeps the full rate for the grace period, then the loop idles") {
        FramePacer pacer;
        double t = 10.0;
        CHECK(pacer.plan(t, input(), kFps).kind == FrameWaitKind::None);
        CHECK(pacer.active());

        // A fast frame sleeps the rest of the budget instead of spinning
        t += 0.004;
        FrameWait wait = pacer.plan(t, FrameActivity{}, kFps);
        CHECK(pacer.active());
        REQUIRE(wait.kind == FrameWaitKind::Sleep);
        CHECK(wait.seconds == Catch::Approx(kBudget - 0.004));

        // A slow frame (vsync already waited) sleeps nothing
        t += wait.seconds + 0.020;
        CHECK(pacer.plan(t, FrameActivity{}, kFps).kind == FrameWaitKind::None);

        // Past the grace period since the last input: idle
        t = 10.0 + settings::kIdleAfterActivitySeconds + 0.01;
        wait = pacer.plan(t, FrameActivity{}, kFps);
        CHECK_FALSE(pacer.active());
        CHECK(wait.kind == FrameWaitKind::WaitForEvents);

        // New input wakes it back to the full rate at once
        t += wait.seconds;
        CHECK(pacer.plan(t, input(), kFps).kind == FrameWaitKind::None);
        CHECK(pacer.active());
    }

    SECTION("A running job or animation keeps the full rate however long it lasts") {
        FramePacer pacer;
        double t = 0.0;
        for (int frame = 0; frame < 600; ++frame) { // ten seconds at 60 fps
            const FrameWait wait = pacer.plan(t, animating(), kFps);
            CHECK(pacer.active());
            CHECK(wait.kind != FrameWaitKind::WaitForEvents);
            t += 1.0 / kFps;
        }
        // It finished: the grace period lets the last results show, then the loop idles
        CHECK(pacer.plan(t, FrameActivity{}, kFps).kind != FrameWaitKind::WaitForEvents);
        t += settings::kIdleAfterActivitySeconds + 0.01;
        CHECK(pacer.plan(t, FrameActivity{}, kFps).kind == FrameWaitKind::WaitForEvents);
    }

    SECTION("An idle wait ends in time for a deadline such as a status message expiring") {
        FramePacer pacer;
        FrameActivity expiring;
        expiring.secondsToDeadline = 0.2;
        FrameWait wait = pacer.plan(50.0, expiring, kFps);
        REQUIRE(wait.kind == FrameWaitKind::WaitForEvents);
        CHECK(wait.seconds == Catch::Approx(0.2));

        expiring.secondsToDeadline = 0.0; // due now: draw at once
        CHECK(pacer.plan(50.2, expiring, kFps).kind == FrameWaitKind::None);

        expiring.secondsToDeadline = 30.0; // far off: the idle interval still caps the wait
        wait = pacer.plan(50.3, expiring, kFps);
        CHECK(wait.seconds == Catch::Approx(settings::kIdleFrameIntervalSeconds));
    }

    SECTION("Uncapped frame rate (maxFPS -1) never sleeps while active") {
        FramePacer pacer;
        CHECK(pacer.plan(1.0, input(), -1).kind == FrameWaitKind::None);
        CHECK(pacer.plan(1.001, FrameActivity{}, -1).kind == FrameWaitKind::None);
    }
}
