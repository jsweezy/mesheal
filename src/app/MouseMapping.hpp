#pragma once

#include "core/Settings.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace mesheal {

/// Mouse button or wheel of a binding (`None`: the slot is unused).
enum class MouseInput : std::uint8_t { None, Left, Middle, Right, Wheel };

/// Modifier keys held with a mouse input, as bits. Ctrl is the Control key on every platform (Cmd is not
/// a mouse modifier, except that it toggles in Select Shell).
using MouseModifiers = std::uint8_t;
inline constexpr MouseModifiers kMouseModNone = 0;
inline constexpr MouseModifiers kMouseModShift = 1;
inline constexpr MouseModifiers kMouseModCtrl = 2;
inline constexpr MouseModifiers kMouseModAlt = 4;
inline constexpr MouseModifiers kMouseModAll = kMouseModShift | kMouseModCtrl | kMouseModAlt;

/// How an action is triggered: dragging a button, turning the wheel, or a click (press and release
/// without moving). A drag and a click on the same button never conflict.
enum class MouseActionKind : std::uint8_t { Drag, Wheel, Click };

/// Every viewport mouse action that Preferences > Mouse can rebind.
enum class MouseAction : std::uint8_t {
    Orbit,       ///< Drag: orbit around the Orbit Pivot
    Pan,         ///< Drag: move the camera and the pivot in the view plane
    Zoom,        ///< Drag: dolly toward / away from the pivot
    WheelZoom,   ///< Wheel: dolly
    NearClip,    ///< Wheel: shift the near clip plane
    Select,      ///< Click: pick a face (focus a defect); also the button of Box (B) / Circle (C) select
    ToggleFace,  ///< Click: add the face to the face selection or remove it
    SelectShell, ///< Click: select the shell under the cursor (+Ctrl / Cmd: add or remove it)
    ClearPick,   ///< Click: clear Polyscope's pick
};
inline constexpr std::size_t kMouseActionCount = 9;
inline constexpr std::array<MouseAction, kMouseActionCount> kAllMouseActions = {
    MouseAction::Orbit,  MouseAction::Pan,        MouseAction::Zoom,        MouseAction::WheelZoom, MouseAction::NearClip,
    MouseAction::Select, MouseAction::ToggleFace, MouseAction::SelectShell, MouseAction::ClearPick,
};

/// Built-in schemes; `Custom` is any edited mapping.
enum class MousePreset : std::uint8_t { Mesheal, Blender, OpenScad, Maya, Fusion360, SolidWorks, Custom };
inline constexpr std::size_t kMousePresetCount = 7;
inline constexpr std::array<MousePreset, kMousePresetCount> kAllMousePresets = {
    MousePreset::Mesheal,      MousePreset::Blender,    MousePreset::OpenScad, MousePreset::Maya,
    MousePreset::Fusion360, MousePreset::SolidWorks, MousePreset::Custom,
};

/// One input that triggers an action: a button (or the wheel) with the modifiers that must be held.
struct MouseBinding {
    MouseInput input = MouseInput::None;
    MouseModifiers modifiers = kMouseModNone;

    [[nodiscard]] bool bound() const noexcept { return input != MouseInput::None; }
    bool operator==(const MouseBinding&) const = default;
};

/// Inputs per action (Mesheal's Pan has three: Shift + Left, Right and Middle drag).
inline constexpr std::size_t kMaxMouseBindingsPerAction = 3;
using MouseActionBindings = std::array<MouseBinding, kMaxMouseBindingsPerAction>;
using MouseBindingTable = std::array<MouseActionBindings, kMouseActionCount>;

/// The bindings of `preset` (`Custom` gives Mesheal's), bound slots first.
[[nodiscard]] MouseBindingTable mousePresetBindings(MousePreset preset);

/// Which mouse input drives each viewport action (Preferences > Mouse). Default = Mesheal's scheme.
struct MouseMapping {
    MousePreset preset = MousePreset::Mesheal;
    MouseBindingTable bindings = mousePresetBindings(MousePreset::Mesheal);
    bool invertWheelZoom = false;                       ///< Wheel up zooms out instead of in
    float zoomSpeed = settings::kDefaultMouseZoomSpeed; ///< Multiplies wheel and drag zoom steps

    [[nodiscard]] const MouseActionBindings& of(MouseAction action) const noexcept {
        return bindings[static_cast<std::size_t>(action)];
    }
    bool operator==(const MouseMapping&) const = default;
};

[[nodiscard]] MouseActionKind mouseActionKind(MouseAction action) noexcept;
/// Display name ("Orbit / Rotate") and stable file key word ("orbit").
[[nodiscard]] const char* mouseActionName(MouseAction action) noexcept;
[[nodiscard]] const char* mouseActionKey(MouseAction action) noexcept;
[[nodiscard]] const char* mousePresetName(MousePreset preset) noexcept;
[[nodiscard]] const char* mousePresetKey(MousePreset preset) noexcept;
/// One line about what the preset is modelled on.
[[nodiscard]] const char* mousePresetDescription(MousePreset preset) noexcept;

/// True when `binding` can trigger `action`: the wheel for wheel actions, a button otherwise (or None).
[[nodiscard]] bool mouseBindingValidFor(MouseAction action, const MouseBinding& binding) noexcept;

/// Picks `preset`: a built-in one replaces every binding; `Custom` keeps them. Zoom options stay.
void selectMousePreset(MouseMapping& mapping, MousePreset preset);

/// Sets slot `slot` of `action` (None clears it; bound slots move first). A change switches the preset
/// to Custom. Returns false (nothing changed) for an invalid binding or slot, or the same binding.
bool setMouseBinding(MouseMapping& mapping, MouseAction action, std::size_t slot, const MouseBinding& binding);

/// A built-in preset's bindings replace the stored ones (after loading: newer presets apply as they are).
void applyMousePresetBindings(MouseMapping& mapping);

/// The drag action of `button` with `held` modifiers. Modifiers that no drag binding of that button uses
/// are ignored (Mesheal: Alt + Left drag still orbits); the others must match exactly (Ctrl + Left drag:
/// nothing). On a conflict the first action in `kAllMouseActions` order wins.
[[nodiscard]] std::optional<MouseAction> resolveMouseDrag(const MouseMapping& mapping, MouseInput button,
                                                          MouseModifiers held);

/// The wheel action with `held` modifiers (same rule as drags).
[[nodiscard]] std::optional<MouseAction> resolveMouseWheel(const MouseMapping& mapping, MouseModifiers held);

/// A resolved click and the binding it matched.
struct MouseClickMatch {
    MouseAction action = MouseAction::Select;
    MouseBinding binding{};
};

/// The click action of `button` with `held` modifiers: an exact match first; else the most specific binding
/// whose modifiers are all held, Toggle Face excepted (Mesheal: Ctrl + Shift + click selects, Shift + Alt +
/// click selects the shell).
[[nodiscard]] std::optional<MouseClickMatch> resolveMouseClick(const MouseMapping& mapping, MouseInput button,
                                                               MouseModifiers held);

/// Tells viewport clicks from drags. A click is a press over the 3D view and its release with less than
/// the drag threshold of viewport dragging in between. Every press and release must be reported, also
/// over a panel: a release there still ends the drag (else the next click was swallowed), and a press
/// there (a slider drag) never turns into a click when released over the view.
class ViewportClickTracker {
public:
    /// `button` went down; `overViewport`: the 3D view (not a panel, gizmo or select tool) got it.
    void press(MouseInput button, bool overViewport) noexcept;
    /// Viewport drag distance since the last release (any button).
    void drag(float distance) noexcept;
    /// `button` went up: true for a click. Starts the next press afresh either way.
    [[nodiscard]] bool release(MouseInput button, float dragThreshold) noexcept;

    [[nodiscard]] float dragDistance() const noexcept { return dragDistance_; }

private:
    std::array<bool, 3> pressedOverViewport_{}; ///< Left, Middle, Right
    float dragDistance_ = 0.0f;
};

/// The button Box (B) and Circle (C) select draw with: Select's first button (Left if it has none).
[[nodiscard]] MouseInput selectToolButton(const MouseMapping& mapping) noexcept;

/// Two actions of the same kind bound to the same input: only `first` acts on it.
struct MouseConflict {
    MouseBinding binding{};
    MouseAction first = MouseAction::Orbit;
    MouseAction second = MouseAction::Orbit;
};
[[nodiscard]] std::vector<MouseConflict> findMouseConflicts(const MouseMapping& mapping);

/// "Ctrl + Shift + Left drag", "Shift + Wheel", "Alt + Left click"; "(none)" when unbound.
[[nodiscard]] std::string formatMouseBinding(const MouseBinding& binding, MouseActionKind kind);
/// "Shift + Left drag, Right drag or Middle drag"; "(not bound)" when the action has none.
[[nodiscard]] std::string describeMouseBindings(const MouseMapping& mapping, MouseAction action);

/// Preferences file text of an action's bindings: "shift+left, right, middle", or "none".
[[nodiscard]] std::string formatMouseBindingList(const MouseActionBindings& bindings);
/// Parses `formatMouseBindingList` text; nullopt when malformed or not valid for `action`.
[[nodiscard]] std::optional<MouseActionBindings> parseMouseBindingList(std::string_view text, MouseAction action);

} // namespace mesheal
