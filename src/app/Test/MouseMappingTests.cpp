#include "app/MouseMapping.hpp"
#include "core/Settings.hpp"

#include <catch2/catch_test_macros.hpp>

#include <optional>
#include <string>

namespace {

using mesheal::MouseAction;
using mesheal::MouseInput;
using mesheal::MouseModifiers;
using mesheal::MousePreset;

constexpr MouseModifiers S = mesheal::kMouseModShift;
constexpr MouseModifiers C = mesheal::kMouseModCtrl;
constexpr MouseModifiers A = mesheal::kMouseModAlt;

constexpr MouseInput kButtons[] = {MouseInput::Left, MouseInput::Middle, MouseInput::Right};

mesheal::MouseMapping preset(MousePreset p) {
    mesheal::MouseMapping m;
    mesheal::selectMousePreset(m, p);
    return m;
}

std::optional<MouseAction> drag(const mesheal::MouseMapping& m, MouseInput button, MouseModifiers mods) {
    return mesheal::resolveMouseDrag(m, button, mods);
}

std::optional<MouseAction> click(const mesheal::MouseMapping& m, MouseInput button, MouseModifiers mods) {
    const auto match = mesheal::resolveMouseClick(m, button, mods);
    return match ? std::optional<MouseAction>(match->action) : std::nullopt;
}

// The scheme PolyscopeGui hard-coded before Preferences > Mouse existed (handleViewportMouseNavigation)
std::optional<MouseAction> legacyDrag(MouseInput button, bool shift, bool ctrl) {
    if (button != MouseInput::Left) {
        return MouseAction::Pan; // right and middle drags always panned
    }
    if (shift && ctrl) {
        return MouseAction::Zoom;
    }
    if (shift) {
        return MouseAction::Pan;
    }
    if (!ctrl) {
        return MouseAction::Orbit;
    }
    return std::nullopt; // Ctrl + Left drag did nothing
}

std::optional<MouseAction> legacyClick(MouseInput button, bool shift, bool ctrl, bool alt) {
    if (button == MouseInput::Right) {
        return MouseAction::ClearPick;
    }
    if (button != MouseInput::Left) {
        return std::nullopt;
    }
    if (shift && !alt && !ctrl) {
        return MouseAction::ToggleFace;
    }
    return alt ? MouseAction::SelectShell : MouseAction::Select;
}

} // namespace

TEST_CASE("The Mesheal mouse preset is the default and reproduces the original scheme exactly", "[app][mouse]") {
    const mesheal::MouseMapping defaults;
    CHECK(defaults.preset == MousePreset::Mesheal);
    CHECK(defaults.bindings == mesheal::mousePresetBindings(MousePreset::Mesheal));
    CHECK_FALSE(defaults.invertWheelZoom);
    CHECK(defaults.zoomSpeed == 1.0f);
    CHECK(mesheal::findMouseConflicts(defaults).empty());
    CHECK(mesheal::selectToolButton(defaults) == MouseInput::Left);

    for (MouseModifiers mods = 0; mods <= mesheal::kMouseModAll; ++mods) {
        const bool shift = (mods & S) != 0;
        const bool ctrl = (mods & C) != 0;
        const bool alt = (mods & A) != 0;
        INFO("modifiers: " << int(mods));
        for (const MouseInput button : kButtons) {
            INFO("button: " << int(button));
            CHECK(drag(defaults, button, mods) == legacyDrag(button, shift, ctrl));
            CHECK(click(defaults, button, mods) == legacyClick(button, shift, ctrl, alt));
        }
        // Shift + wheel shifted the near clip plane; any other wheel zoomed
        CHECK(mesheal::resolveMouseWheel(defaults, mods) == (shift ? MouseAction::NearClip : MouseAction::WheelZoom));
    }

    // Shell clicks hand back the matched binding, so Ctrl (on top of Alt) toggles
    const auto shell = mesheal::resolveMouseClick(defaults, MouseInput::Left, A | C);
    REQUIRE(shell);
    CHECK(shell->action == MouseAction::SelectShell);
    CHECK(shell->binding == mesheal::MouseBinding{MouseInput::Left, A});
}

TEST_CASE("Mouse presets map buttons like the applications they are named after", "[app][mouse]") {
    SECTION("Blender: middle orbits, Shift+middle pans, Ctrl+middle zooms, left only selects") {
        const mesheal::MouseMapping m = preset(MousePreset::Blender);
        CHECK(drag(m, MouseInput::Middle, 0) == MouseAction::Orbit);
        CHECK(drag(m, MouseInput::Middle, S) == MouseAction::Pan);
        CHECK(drag(m, MouseInput::Middle, C) == MouseAction::Zoom);
        CHECK(drag(m, MouseInput::Left, 0) == std::nullopt); // left-click select: no orbit fight
        CHECK(drag(m, MouseInput::Right, 0) == std::nullopt);
        CHECK(click(m, MouseInput::Left, 0) == MouseAction::Select);
        CHECK(click(m, MouseInput::Left, S) == MouseAction::ToggleFace);
        CHECK(click(m, MouseInput::Middle, 0) == std::nullopt);
        CHECK(mesheal::resolveMouseWheel(m, 0) == MouseAction::WheelZoom);
    }
    SECTION("OpenSCAD: left rotates, right pans, middle or Shift+right zooms") {
        const mesheal::MouseMapping m = preset(MousePreset::OpenScad);
        CHECK(drag(m, MouseInput::Left, 0) == MouseAction::Orbit);
        CHECK(drag(m, MouseInput::Left, S) == MouseAction::Orbit);
        CHECK(drag(m, MouseInput::Right, 0) == MouseAction::Pan);
        CHECK(drag(m, MouseInput::Right, S) == MouseAction::Zoom);
        CHECK(drag(m, MouseInput::Middle, 0) == MouseAction::Zoom);
        CHECK(drag(m, MouseInput::Middle, S) == MouseAction::Zoom); // Shift is not used on middle: ignored
        CHECK(click(m, MouseInput::Left, S) == MouseAction::ToggleFace);
    }
    SECTION("Maya: Alt + left / middle / right tumble, track, dolly; plain drags do nothing") {
        const mesheal::MouseMapping m = preset(MousePreset::Maya);
        CHECK(drag(m, MouseInput::Left, A) == MouseAction::Orbit);
        CHECK(drag(m, MouseInput::Middle, A) == MouseAction::Pan);
        CHECK(drag(m, MouseInput::Right, A) == MouseAction::Zoom);
        for (const MouseInput button : kButtons) {
            CHECK(drag(m, button, 0) == std::nullopt);
        }
        CHECK(click(m, MouseInput::Left, 0) == MouseAction::Select);
        CHECK(click(m, MouseInput::Left, A) == MouseAction::SelectShell); // a click, not the Alt+drag
    }
    SECTION("Fusion 360 and SolidWorks") {
        const mesheal::MouseMapping fusion = preset(MousePreset::Fusion360);
        CHECK(drag(fusion, MouseInput::Middle, 0) == MouseAction::Pan);
        CHECK(drag(fusion, MouseInput::Middle, S) == MouseAction::Orbit);
        CHECK(drag(fusion, MouseInput::Left, 0) == std::nullopt);
        const mesheal::MouseMapping solid = preset(MousePreset::SolidWorks);
        CHECK(drag(solid, MouseInput::Middle, 0) == MouseAction::Orbit);
        CHECK(drag(solid, MouseInput::Middle, C) == MouseAction::Pan);
        CHECK(drag(solid, MouseInput::Middle, S) == MouseAction::Zoom);
    }
    SECTION("Every built-in preset is conflict free and keeps the selection clicks on the left button") {
        for (const MousePreset p : mesheal::kAllMousePresets) {
            INFO(mesheal::mousePresetName(p));
            const mesheal::MouseMapping m = preset(p);
            CHECK(m.preset == p);
            CHECK(mesheal::findMouseConflicts(m).empty());
            CHECK(mesheal::selectToolButton(m) == MouseInput::Left);
            CHECK(mesheal::resolveMouseWheel(m, 0) == MouseAction::WheelZoom);
            for (const MouseAction action : mesheal::kAllMouseActions) {
                for (const mesheal::MouseBinding& b : m.of(action)) {
                    CHECK(mesheal::mouseBindingValidFor(action, b));
                }
            }
        }
        CHECK(preset(MousePreset::Custom).bindings == mesheal::mousePresetBindings(MousePreset::Mesheal));
    }
}

TEST_CASE("Editing a mouse binding switches to Custom; picking a preset keeps the zoom options", "[app][mouse]") {
    mesheal::MouseMapping m;
    m.invertWheelZoom = true;
    m.zoomSpeed = 2.0f;

    // Same binding, bad slot or wrong kind: nothing changes
    CHECK_FALSE(mesheal::setMouseBinding(m, MouseAction::Orbit, 0, {MouseInput::Left, 0}));
    CHECK_FALSE(mesheal::setMouseBinding(m, MouseAction::Orbit, mesheal::kMaxMouseBindingsPerAction, {MouseInput::Middle, 0}));
    CHECK_FALSE(mesheal::setMouseBinding(m, MouseAction::Orbit, 1, {MouseInput::Wheel, 0}));
    CHECK_FALSE(mesheal::setMouseBinding(m, MouseAction::WheelZoom, 0, {MouseInput::Left, 0}));
    CHECK(m.preset == MousePreset::Mesheal);

    REQUIRE(mesheal::setMouseBinding(m, MouseAction::Orbit, 0, {MouseInput::Middle, 0}));
    CHECK(m.preset == MousePreset::Custom);
    CHECK(drag(m, MouseInput::Middle, 0) == MouseAction::Orbit); // Orbit is first: wins over Pan's Middle
    CHECK(drag(m, MouseInput::Left, 0) == std::nullopt);

    // Clearing a slot moves the later inputs up
    REQUIRE(mesheal::setMouseBinding(m, MouseAction::Pan, 0, {}));
    CHECK(m.of(MouseAction::Pan)[0] == mesheal::MouseBinding{MouseInput::Right, 0});
    CHECK(m.of(MouseAction::Pan)[1] == mesheal::MouseBinding{MouseInput::Middle, 0});
    CHECK_FALSE(m.of(MouseAction::Pan)[2].bound());

    mesheal::selectMousePreset(m, MousePreset::Blender);
    CHECK(m.bindings == mesheal::mousePresetBindings(MousePreset::Blender));
    CHECK(m.invertWheelZoom);
    CHECK(m.zoomSpeed == 2.0f);
    mesheal::selectMousePreset(m, MousePreset::Custom); // keeps Blender's bindings to edit
    CHECK(m.bindings == mesheal::mousePresetBindings(MousePreset::Blender));

    // A select button other than Left moves Box / Circle select with it
    REQUIRE(mesheal::setMouseBinding(m, MouseAction::Select, 0, {MouseInput::Right, 0}));
    CHECK(mesheal::selectToolButton(m) == MouseInput::Right);
    REQUIRE(mesheal::setMouseBinding(m, MouseAction::Select, 0, {}));
    CHECK(mesheal::selectToolButton(m) == MouseInput::Left);
    CHECK(click(m, MouseInput::Left, 0) == std::nullopt);
}

TEST_CASE("Mouse conflicts are two actions of one kind on the same input", "[app][mouse]") {
    mesheal::MouseMapping m;
    REQUIRE(mesheal::setMouseBinding(m, MouseAction::Zoom, 1, {MouseInput::Right, 0})); // Pan also has Right
    REQUIRE(mesheal::setMouseBinding(m, MouseAction::NearClip, 0, {MouseInput::Wheel, 0})); // WheelZoom too
    REQUIRE(mesheal::setMouseBinding(m, MouseAction::ClearPick, 0, {MouseInput::Left, S})); // Toggle Face too

    const std::vector<mesheal::MouseConflict> conflicts = mesheal::findMouseConflicts(m);
    REQUIRE(conflicts.size() == 3);
    CHECK(conflicts[0].first == MouseAction::Pan);
    CHECK(conflicts[0].second == MouseAction::Zoom);
    CHECK(conflicts[0].binding == mesheal::MouseBinding{MouseInput::Right, 0});
    CHECK(conflicts[1].first == MouseAction::WheelZoom);
    CHECK(conflicts[1].second == MouseAction::NearClip);
    CHECK(conflicts[2].first == MouseAction::ToggleFace);
    CHECK(conflicts[2].second == MouseAction::ClearPick);
    // The first action acts
    CHECK(drag(m, MouseInput::Right, 0) == MouseAction::Pan);
    CHECK(mesheal::resolveMouseWheel(m, 0) == MouseAction::WheelZoom);

    // A drag and a click on the same input are not a conflict (Orbit drag vs Select click on Left)
    CHECK(mesheal::findMouseConflicts(mesheal::MouseMapping{}).empty());
}

TEST_CASE("Mouse bindings format for display and round trip through the preferences text", "[app][mouse]") {
    const mesheal::MouseMapping m;
    CHECK(mesheal::describeMouseBindings(m, MouseAction::Pan) == "Shift + Left drag, Right drag or Middle drag");
    CHECK(mesheal::describeMouseBindings(m, MouseAction::Zoom) == "Ctrl + Shift + Left drag");
    CHECK(mesheal::describeMouseBindings(m, MouseAction::NearClip) == "Shift + Wheel");
    CHECK(mesheal::describeMouseBindings(m, MouseAction::SelectShell) == "Alt + Left click");
    mesheal::MouseMapping none = m;
    REQUIRE(mesheal::setMouseBinding(none, MouseAction::ClearPick, 0, {}));
    CHECK(mesheal::describeMouseBindings(none, MouseAction::ClearPick) == "(not bound)");

    CHECK(mesheal::formatMouseBindingList(m.of(MouseAction::Pan)) == "shift+left, right, middle");
    CHECK(mesheal::formatMouseBindingList(none.of(MouseAction::ClearPick)) == "none");
    for (const MousePreset p : mesheal::kAllMousePresets) {
        const mesheal::MouseMapping mapping = preset(p);
        for (const MouseAction action : mesheal::kAllMouseActions) {
            const auto back = mesheal::parseMouseBindingList(mesheal::formatMouseBindingList(mapping.of(action)), action);
            REQUIRE(back);
            CHECK(*back == mapping.of(action));
        }
    }

    const auto parsed = mesheal::parseMouseBindingList("  Alt+Shift+CTRL+middle ,right ", MouseAction::Orbit);
    REQUIRE(parsed);
    CHECK((*parsed)[0] == mesheal::MouseBinding{MouseInput::Middle, static_cast<MouseModifiers>(S | C | A)});
    CHECK((*parsed)[1] == mesheal::MouseBinding{MouseInput::Right, 0});
    CHECK(mesheal::parseMouseBindingList("NONE", MouseAction::Orbit) == mesheal::MouseActionBindings{});

    // Malformed or invalid for the action
    for (const char* bad : {"", "banana", "shift", "shift+shift+left", "left+right", "left,", "left, right, middle, left",
                            "wheel", "hyper+left"}) {
        INFO(bad);
        CHECK_FALSE(mesheal::parseMouseBindingList(bad, MouseAction::Orbit));
    }
    CHECK_FALSE(mesheal::parseMouseBindingList("left", MouseAction::WheelZoom));
    CHECK(mesheal::parseMouseBindingList("ctrl+wheel", MouseAction::WheelZoom));
}

namespace {

constexpr float kThreshold = mesheal::settings::kDragIgnoreThreshold;

/// A press over the 3D view and its release after `dragged` of viewport dragging: the click action, if any.
std::optional<MouseAction> viewportClick(const mesheal::MouseMapping& m, mesheal::ViewportClickTracker& tracker,
                                         MouseInput button, MouseModifiers mods, float dragged = 0.0f) {
    tracker.press(button, true);
    if (dragged > 0.0f) {
        tracker.drag(dragged);
    }
    if (!tracker.release(button, kThreshold)) {
        return std::nullopt;
    }
    return click(m, button, mods);
}

} // namespace

TEST_CASE("A plain left click in the 3D view selects the face under every mouse preset", "[app][mouse]") {
    for (const MousePreset p : mesheal::kAllMousePresets) {
        INFO(mesheal::mousePresetName(p));
        const mesheal::MouseMapping m = preset(p);
        mesheal::ViewportClickTracker tracker;
        // Also where Left has no drag binding at all (Blender, Maya, Fusion 360, SolidWorks)
        CHECK(viewportClick(m, tracker, MouseInput::Left, 0) == MouseAction::Select);
        CHECK(viewportClick(m, tracker, MouseInput::Left, S) == MouseAction::ToggleFace);
        CHECK(viewportClick(m, tracker, MouseInput::Left, A) == MouseAction::SelectShell);
        CHECK(viewportClick(m, tracker, MouseInput::Left, C) == MouseAction::Select);
        CHECK(viewportClick(m, tracker, MouseInput::Left, static_cast<MouseModifiers>(C | S)) == MouseAction::Select);
        CHECK(viewportClick(m, tracker, MouseInput::Right, 0) == MouseAction::ClearPick);
        CHECK(viewportClick(m, tracker, MouseInput::Middle, 0) == std::nullopt);
        // A real drag is no click, bound or not, and the next click still works
        CHECK(viewportClick(m, tracker, MouseInput::Left, 0, 0.2f) == std::nullopt);
        CHECK(viewportClick(m, tracker, MouseInput::Left, 0) == MouseAction::Select);
        // Jitter below the threshold is still a click
        CHECK(viewportClick(m, tracker, MouseInput::Left, 0, 0.5f * kThreshold) == MouseAction::Select);
    }
}

TEST_CASE("The viewport click tracker sees every press and release, also over panels", "[app][mouse]") {
    mesheal::ViewportClickTracker tracker;

    SECTION("A drag released over a panel does not swallow the next click") {
        // Blender: middle-drag orbit that ends over a panel; the release is reported but is no click
        tracker.press(MouseInput::Middle, true);
        tracker.drag(0.3f);
        CHECK_FALSE(tracker.release(MouseInput::Middle, kThreshold));
        CHECK(tracker.dragDistance() == 0.0f);
        tracker.press(MouseInput::Left, true);
        CHECK(tracker.release(MouseInput::Left, kThreshold));
    }

    SECTION("A press on a panel (slider drag) released over the 3D view is no click") {
        tracker.press(MouseInput::Left, false);
        CHECK_FALSE(tracker.release(MouseInput::Left, kThreshold));
        tracker.press(MouseInput::Left, true);
        CHECK(tracker.release(MouseInput::Left, kThreshold));
    }

    SECTION("A release without a tracked press, or of the wheel, is no click") {
        CHECK_FALSE(tracker.release(MouseInput::Right, kThreshold));
        tracker.press(MouseInput::Wheel, true);
        CHECK_FALSE(tracker.release(MouseInput::Wheel, kThreshold));
    }

    SECTION("Buttons are tracked separately") {
        tracker.press(MouseInput::Left, true);
        tracker.press(MouseInput::Right, false);
        CHECK_FALSE(tracker.release(MouseInput::Right, kThreshold));
        CHECK(tracker.release(MouseInput::Left, kThreshold));
    }
}
