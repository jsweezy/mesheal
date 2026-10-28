#include "app/MouseMapping.hpp"

#include <algorithm>
#include <bit>
#include <cctype>

namespace ogun {

namespace {

constexpr MouseBinding bind(MouseInput input, MouseModifiers modifiers = kMouseModNone) noexcept {
    return MouseBinding{input, modifiers};
}

constexpr MouseActionBindings one(MouseBinding a) noexcept {
    return {a, MouseBinding{}, MouseBinding{}};
}

constexpr MouseActionBindings two(MouseBinding a, MouseBinding b) noexcept {
    return {a, b, MouseBinding{}};
}

constexpr std::size_t index(MouseAction action) noexcept {
    return static_cast<std::size_t>(action);
}

/// Clicks, the wheel and the near-clip shift are the same in every preset.
MouseBindingTable commonBindings() {
    MouseBindingTable t{};
    t[index(MouseAction::WheelZoom)] = one(bind(MouseInput::Wheel));
    t[index(MouseAction::NearClip)] = one(bind(MouseInput::Wheel, kMouseModShift));
    t[index(MouseAction::Select)] = one(bind(MouseInput::Left));
    t[index(MouseAction::ToggleFace)] = one(bind(MouseInput::Left, kMouseModShift));
    t[index(MouseAction::SelectShell)] = one(bind(MouseInput::Left, kMouseModAlt));
    t[index(MouseAction::ClearPick)] = one(bind(MouseInput::Right));
    return t;
}

void compact(MouseActionBindings& bindings) {
    std::stable_partition(bindings.begin(), bindings.end(), [](const MouseBinding& b) { return b.bound(); });
}

std::string_view trim(std::string_view s) noexcept {
    const auto first = s.find_first_not_of(" \t\r\n");
    if (first == std::string_view::npos) {
        return {};
    }
    const auto last = s.find_last_not_of(" \t\r\n");
    return s.substr(first, last - first + 1);
}

std::string lower(std::string_view s) {
    std::string out(s);
    std::transform(out.begin(), out.end(), out.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return out;
}

const char* inputKey(MouseInput input) noexcept {
    switch (input) {
    case MouseInput::Left: return "left";
    case MouseInput::Middle: return "middle";
    case MouseInput::Right: return "right";
    case MouseInput::Wheel: return "wheel";
    case MouseInput::None: break;
    }
    return "none";
}

const char* inputName(MouseInput input) noexcept {
    switch (input) {
    case MouseInput::Left: return "Left";
    case MouseInput::Middle: return "Middle";
    case MouseInput::Right: return "Right";
    case MouseInput::Wheel: return "Wheel";
    case MouseInput::None: break;
    }
    return "None";
}

std::optional<MouseBinding> parseBinding(std::string_view text) {
    MouseBinding binding;
    while (true) {
        const auto plus = text.find('+');
        const std::string part = lower(trim(text.substr(0, plus)));
        MouseModifiers bit = kMouseModNone;
        if (part == "shift") {
            bit = kMouseModShift;
        } else if (part == "ctrl") {
            bit = kMouseModCtrl;
        } else if (part == "alt") {
            bit = kMouseModAlt;
        } else if (part == "left" || part == "middle" || part == "right" || part == "wheel") {
            if (binding.bound()) {
                return std::nullopt; // two inputs
            }
            binding.input = part == "left" ? MouseInput::Left
                          : part == "middle" ? MouseInput::Middle
                          : part == "right" ? MouseInput::Right
                                            : MouseInput::Wheel;
        } else {
            return std::nullopt;
        }
        if ((binding.modifiers & bit) != 0) {
            return std::nullopt; // repeated modifier
        }
        binding.modifiers |= bit;
        if (plus == std::string_view::npos) {
            break;
        }
        text = text.substr(plus + 1);
    }
    if (!binding.bound()) {
        return std::nullopt;
    }
    return binding;
}

/// Drag and wheel rule: only the modifiers some binding of `input` uses count, and they must match exactly.
std::optional<MouseAction> resolveExactSignificant(const MouseMapping& mapping, MouseActionKind kind, MouseInput input,
                                                   MouseModifiers held) {
    MouseModifiers significant = kMouseModNone;
    for (const MouseAction action : kAllMouseActions) {
        if (mouseActionKind(action) != kind) {
            continue;
        }
        for (const MouseBinding& b : mapping.of(action)) {
            if (b.input == input) {
                significant |= b.modifiers;
            }
        }
    }
    const MouseModifiers effective = held & significant;
    for (const MouseAction action : kAllMouseActions) {
        if (mouseActionKind(action) != kind) {
            continue;
        }
        for (const MouseBinding& b : mapping.of(action)) {
            if (b.input == input && b.modifiers == effective) {
                return action;
            }
        }
    }
    return std::nullopt;
}

} // namespace

MouseBindingTable mousePresetBindings(MousePreset preset) {
    MouseBindingTable t = commonBindings();
    auto& orbit = t[index(MouseAction::Orbit)];
    auto& pan = t[index(MouseAction::Pan)];
    auto& zoom = t[index(MouseAction::Zoom)];
    using enum MouseInput;
    switch (preset) {
    case MousePreset::Blender:
        orbit = one(bind(Middle));
        pan = one(bind(Middle, kMouseModShift));
        zoom = one(bind(Middle, kMouseModCtrl));
        break;
    case MousePreset::OpenScad:
        orbit = two(bind(Left), bind(Left, kMouseModShift));
        pan = one(bind(Right));
        zoom = two(bind(Middle), bind(Right, kMouseModShift));
        break;
    case MousePreset::Maya:
        orbit = one(bind(Left, kMouseModAlt));
        pan = one(bind(Middle, kMouseModAlt));
        zoom = one(bind(Right, kMouseModAlt));
        break;
    case MousePreset::Fusion360:
        orbit = one(bind(Middle, kMouseModShift));
        pan = one(bind(Middle));
        break;
    case MousePreset::SolidWorks:
        orbit = one(bind(Middle));
        pan = one(bind(Middle, kMouseModCtrl));
        zoom = one(bind(Middle, kMouseModShift));
        break;
    case MousePreset::Ogun:
    case MousePreset::Custom:
        orbit = one(bind(Left));
        pan = {bind(Left, kMouseModShift), bind(Right), bind(Middle)};
        zoom = one(bind(Left, kMouseModCtrl | kMouseModShift));
        break;
    }
    return t;
}

MouseActionKind mouseActionKind(MouseAction action) noexcept {
    switch (action) {
    case MouseAction::Orbit:
    case MouseAction::Pan:
    case MouseAction::Zoom:
        return MouseActionKind::Drag;
    case MouseAction::WheelZoom:
    case MouseAction::NearClip:
        return MouseActionKind::Wheel;
    case MouseAction::Select:
    case MouseAction::ToggleFace:
    case MouseAction::SelectShell:
    case MouseAction::ClearPick:
        break;
    }
    return MouseActionKind::Click;
}

const char* mouseActionName(MouseAction action) noexcept {
    switch (action) {
    case MouseAction::Orbit: return "Orbit / Rotate";
    case MouseAction::Pan: return "Pan";
    case MouseAction::Zoom: return "Zoom (drag)";
    case MouseAction::WheelZoom: return "Zoom (wheel)";
    case MouseAction::NearClip: return "Near clip plane";
    case MouseAction::Select: return "Select face / defect";
    case MouseAction::ToggleFace: return "Toggle face in selection";
    case MouseAction::SelectShell: return "Select shell";
    case MouseAction::ClearPick: return "Clear pick";
    }
    return "?";
}

const char* mouseActionKey(MouseAction action) noexcept {
    switch (action) {
    case MouseAction::Orbit: return "orbit";
    case MouseAction::Pan: return "pan";
    case MouseAction::Zoom: return "zoom_drag";
    case MouseAction::WheelZoom: return "zoom_wheel";
    case MouseAction::NearClip: return "near_clip_wheel";
    case MouseAction::Select: return "select";
    case MouseAction::ToggleFace: return "toggle_face";
    case MouseAction::SelectShell: return "select_shell";
    case MouseAction::ClearPick: return "clear_pick";
    }
    return "unknown";
}

const char* mousePresetName(MousePreset preset) noexcept {
    switch (preset) {
    case MousePreset::Ogun: return "Ogun (default)";
    case MousePreset::Blender: return "Blender";
    case MousePreset::OpenScad: return "OpenSCAD";
    case MousePreset::Maya: return "Maya";
    case MousePreset::Fusion360: return "Fusion 360";
    case MousePreset::SolidWorks: return "SolidWorks";
    case MousePreset::Custom: return "Custom";
    }
    return "?";
}

const char* mousePresetKey(MousePreset preset) noexcept {
    switch (preset) {
    case MousePreset::Ogun: return "ogun";
    case MousePreset::Blender: return "blender";
    case MousePreset::OpenScad: return "openscad";
    case MousePreset::Maya: return "maya";
    case MousePreset::Fusion360: return "fusion360";
    case MousePreset::SolidWorks: return "solidworks";
    case MousePreset::Custom: return "custom";
    }
    return "ogun";
}

const char* mousePresetDescription(MousePreset preset) noexcept {
    switch (preset) {
    case MousePreset::Ogun:
        return "Left drag orbits; Shift + Left, Right or Middle drag pans; the wheel or Ctrl + Shift + Left drag zooms.";
    case MousePreset::Blender:
        return "Blender (left-click select): Middle drag orbits, Shift + Middle pans, Ctrl + Middle or the wheel zooms. "
               "Left drag does not move the camera.";
    case MousePreset::OpenScad:
        return "OpenSCAD: Left drag rotates, Right drag pans, Middle drag, Shift + Right drag or the wheel zooms.";
    case MousePreset::Maya:
        return "Maya: Alt + Left drag tumbles, Alt + Middle tracks (pans), Alt + Right dollies; plain drags do not "
               "move the camera.";
    case MousePreset::Fusion360:
        return "Fusion 360: Shift + Middle drag orbits, Middle drag pans, the wheel zooms.";
    case MousePreset::SolidWorks:
        return "SolidWorks: Middle drag rotates, Ctrl + Middle pans, Shift + Middle or the wheel zooms.";
    case MousePreset::Custom:
        return "Your own bindings.";
    }
    return "";
}

bool mouseBindingValidFor(MouseAction action, const MouseBinding& binding) noexcept {
    if (!binding.bound()) {
        return true;
    }
    if ((binding.modifiers & ~kMouseModAll) != 0) {
        return false;
    }
    return (mouseActionKind(action) == MouseActionKind::Wheel) == (binding.input == MouseInput::Wheel);
}

void selectMousePreset(MouseMapping& mapping, MousePreset preset) {
    mapping.preset = preset;
    applyMousePresetBindings(mapping);
}

void applyMousePresetBindings(MouseMapping& mapping) {
    if (mapping.preset != MousePreset::Custom) {
        mapping.bindings = mousePresetBindings(mapping.preset);
    }
}

bool setMouseBinding(MouseMapping& mapping, MouseAction action, std::size_t slot, const MouseBinding& binding) {
    if (slot >= kMaxMouseBindingsPerAction || !mouseBindingValidFor(action, binding)) {
        return false;
    }
    MouseActionBindings& bindings = mapping.bindings[index(action)];
    if (bindings[slot] == binding) {
        return false;
    }
    bindings[slot] = binding;
    compact(bindings);
    mapping.preset = MousePreset::Custom;
    return true;
}

std::optional<MouseAction> resolveMouseDrag(const MouseMapping& mapping, MouseInput button, MouseModifiers held) {
    if (button == MouseInput::None || button == MouseInput::Wheel) {
        return std::nullopt;
    }
    return resolveExactSignificant(mapping, MouseActionKind::Drag, button, held);
}

std::optional<MouseAction> resolveMouseWheel(const MouseMapping& mapping, MouseModifiers held) {
    return resolveExactSignificant(mapping, MouseActionKind::Wheel, MouseInput::Wheel, held);
}

std::optional<MouseClickMatch> resolveMouseClick(const MouseMapping& mapping, MouseInput button, MouseModifiers held) {
    if (button == MouseInput::None || button == MouseInput::Wheel) {
        return std::nullopt;
    }
    for (const MouseAction action : kAllMouseActions) {
        if (mouseActionKind(action) != MouseActionKind::Click) {
            continue;
        }
        for (const MouseBinding& b : mapping.of(action)) {
            if (b.input == button && b.modifiers == held) {
                return MouseClickMatch{action, b};
            }
        }
    }
    std::optional<MouseClickMatch> best;
    int bestBits = -1;
    for (const MouseAction action : kAllMouseActions) {
        if (mouseActionKind(action) != MouseActionKind::Click || action == MouseAction::ToggleFace) {
            continue;
        }
        for (const MouseBinding& b : mapping.of(action)) {
            const int bits = std::popcount(static_cast<unsigned>(b.modifiers));
            if (b.input == button && (b.modifiers & ~held) == 0 && bits > bestBits) {
                best = MouseClickMatch{action, b};
                bestBits = bits;
            }
        }
    }
    return best;
}

MouseInput selectToolButton(const MouseMapping& mapping) noexcept {
    for (const MouseBinding& b : mapping.of(MouseAction::Select)) {
        if (b.bound() && b.input != MouseInput::Wheel) {
            return b.input;
        }
    }
    return MouseInput::Left;
}

std::vector<MouseConflict> findMouseConflicts(const MouseMapping& mapping) {
    std::vector<MouseConflict> conflicts;
    for (std::size_t i = 0; i < kMouseActionCount; ++i) {
        for (std::size_t j = i + 1; j < kMouseActionCount; ++j) {
            const MouseAction a = kAllMouseActions[i];
            const MouseAction b = kAllMouseActions[j];
            if (mouseActionKind(a) != mouseActionKind(b)) {
                continue;
            }
            for (const MouseBinding& ba : mapping.of(a)) {
                if (!ba.bound()) {
                    continue;
                }
                const MouseActionBindings& other = mapping.of(b);
                if (std::find(other.begin(), other.end(), ba) != other.end()) {
                    conflicts.push_back(MouseConflict{ba, a, b});
                }
            }
        }
    }
    return conflicts;
}

std::string formatMouseBinding(const MouseBinding& binding, MouseActionKind kind) {
    if (!binding.bound()) {
        return "(none)";
    }
    std::string text;
    if ((binding.modifiers & kMouseModCtrl) != 0) {
        text += "Ctrl + ";
    }
    if ((binding.modifiers & kMouseModShift) != 0) {
        text += "Shift + ";
    }
    if ((binding.modifiers & kMouseModAlt) != 0) {
        text += "Alt + ";
    }
    text += inputName(binding.input);
    if (binding.input != MouseInput::Wheel) {
        text += kind == MouseActionKind::Click ? " click" : " drag";
    }
    return text;
}

std::string describeMouseBindings(const MouseMapping& mapping, MouseAction action) {
    std::vector<std::string> parts;
    parts.reserve(kMaxMouseBindingsPerAction);
    for (const MouseBinding& b : mapping.of(action)) {
        if (b.bound()) {
            parts.push_back(formatMouseBinding(b, mouseActionKind(action)));
        }
    }
    if (parts.empty()) {
        return "(not bound)";
    }
    std::string text = parts[0];
    for (std::size_t i = 1; i < parts.size(); ++i) {
        text += (i + 1 == parts.size() ? " or " : ", ") + parts[i];
    }
    return text;
}

std::string formatMouseBindingList(const MouseActionBindings& bindings) {
    std::string text;
    for (const MouseBinding& b : bindings) {
        if (!b.bound()) {
            continue;
        }
        if (!text.empty()) {
            text += ", ";
        }
        if ((b.modifiers & kMouseModCtrl) != 0) {
            text += "ctrl+";
        }
        if ((b.modifiers & kMouseModShift) != 0) {
            text += "shift+";
        }
        if ((b.modifiers & kMouseModAlt) != 0) {
            text += "alt+";
        }
        text += inputKey(b.input);
    }
    return text.empty() ? std::string("none") : text;
}

std::optional<MouseActionBindings> parseMouseBindingList(std::string_view text, MouseAction action) {
    text = trim(text);
    MouseActionBindings bindings{};
    if (lower(text) == "none") {
        return bindings;
    }
    std::size_t count = 0;
    while (true) {
        const auto comma = text.find(',');
        const std::optional<MouseBinding> binding = parseBinding(trim(text.substr(0, comma)));
        if (!binding || count == kMaxMouseBindingsPerAction || !mouseBindingValidFor(action, *binding)) {
            return std::nullopt;
        }
        bindings[count++] = *binding;
        if (comma == std::string_view::npos) {
            break;
        }
        text = text.substr(comma + 1);
    }
    return bindings;
}

} // namespace ogun
