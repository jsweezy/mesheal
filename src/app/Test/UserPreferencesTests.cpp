#include "app/AppController.hpp"
#include "app/UserPreferences.hpp"
#include "core/Settings.hpp"

#include <catch2/catch_test_macros.hpp>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <map>
#include <string>

namespace {

/// Fresh directory under the system temp dir (never the real ~/.config/ogun).
std::filesystem::path uniqueTempDir(const char* tag) {
    const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
    return std::filesystem::temp_directory_path() / (std::string("ogun_prefs_") + tag + "_" + std::to_string(stamp));
}

/// key -> value of every `key = value` line.
std::map<std::string, std::string> keyValues(const std::string& text) {
    std::map<std::string, std::string> values;
    std::size_t start = 0;
    while (start < text.size()) {
        const std::size_t end = std::min(text.find('\n', start), text.size());
        const std::string line = text.substr(start, end - start);
        start = end + 1;
        const std::size_t eq = line.find(" = ");
        if (!line.empty() && line[0] != '#' && eq != std::string::npos) {
            values[line.substr(0, eq)] = line.substr(eq + 3);
        }
    }
    return values;
}

/// Every saved field set to a valid value different from its built-in default.
ogun::UserPreferences customPreferences() {
    ogun::UserPreferences p;
    const auto styled = [](ogun::SurfaceStyle& s, double shade) {
        s.color = Eigen::Vector3d(shade, 0.25, 0.75);
        s.edgeColor = Eigen::Vector3d(0.125, shade, 0.5);
        s.edgeWidthPx = 2.5 + shade;
        s.showEdges = !s.showEdges;
    };
    styled(p.appearance.baseMesh, 0.1);
    styled(p.appearance.selectedFaces, 0.2);
    styled(p.appearance.previewPatch, 0.3);
    p.appearance.backFaceMode = ogun::BackFaceMode::Custom;
    p.appearance.backFaceColor = Eigen::Vector3d(0.11, 0.22, 0.33);
    p.appearance.smoothShade = !p.appearance.smoothShade;
    p.appearance.categoryTintColor = Eigen::Vector3d(0.44, 0.55, 0.66);
    p.appearance.faceSelectionColor = Eigen::Vector3d(0.77, 0.88, 0.99);
    p.appearance.originalEdgeColor = Eigen::Vector3d(0.9, 0.8, 0.7);
    p.appearance.originalEdgeWidthPx = 1.75;
    p.appearance.originalEdgeOpacity = 0.3;
    p.appearance.pointColor = Eigen::Vector3d(0.6, 0.5, 0.4);
    p.appearance.pointRadiusPx = 7.5;
    p.ghostOpacity = 0.6;
    p.repairPreviewVisible = !p.repairPreviewVisible;
    p.previewDisplayMode = ogun::PreviewDisplayMode::TranslucentPatch;
    p.previewBaseOpacity = 0.7;
    p.previewPatchOpacity = 0.8;
    p.tolerance.absFloor = 3.5e-7;
    p.tolerance.relTol = 2.0e-6;
    p.defaults.thinFace.minAltitude = 0.02;
    p.defaults.thinFace.minArea = 4.0e-5;
    p.defaults.thinFace.maxAspectRatio = 250.0;
    p.checkMeshAfterLoad = !p.checkMeshAfterLoad;
    for (bool& autoCheck : p.autoCheckDefaults) {
        autoCheck = !autoCheck;
    }
    p.listCap = 42;
    p.overlayCap = 1234;
    p.uiScale = 1.35f;
    p.backgroundColor = {0.1f, 0.2f, 0.3f, 1.0f};
    p.showCompass = !p.showCompass;
    // A custom mouse scheme that changes every action's bindings
    using ogun::MouseAction;
    using ogun::MouseInput;
    p.mouse.preset = ogun::MousePreset::Custom;
    p.mouse.invertWheelZoom = true;
    p.mouse.zoomSpeed = 1.5f;
    const auto set = [&p](MouseAction action, ogun::MouseActionBindings bindings) {
        p.mouse.bindings[static_cast<std::size_t>(action)] = bindings;
    };
    set(MouseAction::Orbit, {{{MouseInput::Middle, 0}}});
    set(MouseAction::Pan, {{{MouseInput::Middle, ogun::kMouseModShift}, {MouseInput::Right, ogun::kMouseModAlt}}});
    set(MouseAction::Zoom, {{{MouseInput::Middle, ogun::kMouseModCtrl}}});
    set(MouseAction::WheelZoom, {{{MouseInput::Wheel, ogun::kMouseModCtrl}}});
    set(MouseAction::NearClip, {{{MouseInput::Wheel, ogun::kMouseModAlt}}});
    set(MouseAction::Select, {{{MouseInput::Right, 0}}});
    set(MouseAction::ToggleFace, {{{MouseInput::Right, ogun::kMouseModCtrl}}});
    set(MouseAction::SelectShell, {{{MouseInput::Right, ogun::kMouseModAlt}}});
    set(MouseAction::ClearPick, {});
    return p;
}

} // namespace

TEST_CASE("User preferences round trip every field through the config file", "[app][preferences]") {
    const ogun::UserPreferences custom = customPreferences();
    const ogun::UserPreferences defaults;

    // Every key of the file is changed by customPreferences(), so the round trip covers every field
    const auto customValues = keyValues(ogun::formatUserPreferences(custom));
    const auto defaultValues = keyValues(ogun::formatUserPreferences(defaults));
    const std::vector<std::string> keys = ogun::userPreferenceKeys();
    REQUIRE(customValues.size() == keys.size());
    REQUIRE(defaultValues.size() == keys.size());
    for (const std::string& key : keys) {
        INFO("key: " << key);
        REQUIRE(customValues.count(key) == 1);
        CHECK(customValues.at(key) != defaultValues.at(key));
    }
    CHECK_FALSE(ogun::sameUserPreferences(custom, defaults));

    const std::filesystem::path dir = uniqueTempDir("roundtrip");
    const std::filesystem::path file = ogun::preferencesFilePath(dir / "ogun");
    CHECK(file.filename() == ogun::kPreferencesFileName);
    std::string error;
    REQUIRE(ogun::saveUserPreferences(custom, file, &error)); // creates the directory
    REQUIRE(std::filesystem::exists(file));

    const ogun::UserPreferences loaded = ogun::loadUserPreferences(file);
    CHECK(ogun::sameUserPreferences(loaded, custom));
    CHECK(ogun::formatUserPreferences(loaded) == ogun::formatUserPreferences(custom));
    CHECK(loaded.appearance.baseMesh.color.isApprox(custom.appearance.baseMesh.color));
    CHECK(loaded.defaults.thinFace.minAltitude == custom.defaults.thinFace.minAltitude);
    CHECK(loaded.uiScale == custom.uiScale);
    CHECK(loaded.backgroundColor == custom.backgroundColor);
    CHECK(loaded.showCompass == custom.showCompass);
    CHECK(loaded.mouse == custom.mouse);

    // Saving again overwrites in place
    REQUIRE(ogun::saveUserPreferences(defaults, file, &error));
    CHECK(ogun::sameUserPreferences(ogun::loadUserPreferences(file), defaults));
    std::filesystem::remove_all(dir);
}

TEST_CASE("A missing preferences file gives the built-in defaults", "[app][preferences]") {
    const std::filesystem::path dir = uniqueTempDir("missing");
    const ogun::UserPreferences loaded = ogun::loadUserPreferences(ogun::preferencesFilePath(dir));
    CHECK(ogun::sameUserPreferences(loaded, ogun::UserPreferences{}));
    CHECK(ogun::sameUserPreferences(ogun::loadUserPreferences({}), ogun::UserPreferences{}));
    CHECK(ogun::preferencesFilePath({}).empty());
    CHECK_FALSE(std::filesystem::exists(dir));
}

TEST_CASE("Malformed or partial preference files fall back per key", "[app][preferences]") {
    const ogun::UserPreferences defaults;
    const ogun::UserPreferences prefs = ogun::parseUserPreferences(
        "# comment\n"
        "weld_floor = 2e-6\n"                      // valid
        "relative_tolerance = abc\n"               // malformed: default
        "  default_min_altitude=0.5  \r\n"         // spacing and CRLF are fine
        "default_min_area = 1e-3 junk\n"           // trailing junk: default
        "default_max_aspect_ratio = 0.5\n"         // below 1: clamped
        "check_mesh_after_load = maybe\n"          // malformed bool: default
        "base.color = 0.1 0.2\n"                   // too few components: default
        "defect.tint_color = 0.2 0.4 2.0\n"        // out of [0, 1]: clamped
        "defect.point_radius_px = 1000\n"          // clamped to the max
        "preview.mode = sideways\n"                // unknown enum word: default
        "base.back_faces = cull\n"                 // valid
        "list_cap = -5\n"                          // malformed count: default
        "overlay_cap = 77\n"                       // valid
        "ui_scale = nan\n"                         // not finite: default
        "line without equals sign\n");

    CHECK(prefs.tolerance.absFloor == 2.0e-6);
    CHECK(prefs.tolerance.relTol == defaults.tolerance.relTol);
    CHECK(prefs.defaults.thinFace.minAltitude == 0.5);
    CHECK(prefs.defaults.thinFace.minArea == defaults.defaults.thinFace.minArea);
    CHECK(prefs.defaults.thinFace.maxAspectRatio == 1.0);
    CHECK(prefs.checkMeshAfterLoad == defaults.checkMeshAfterLoad);
    CHECK(prefs.appearance.baseMesh.color.isApprox(defaults.appearance.baseMesh.color));
    CHECK(prefs.appearance.categoryTintColor.isApprox(Eigen::Vector3d(0.2, 0.4, 1.0)));
    CHECK(prefs.appearance.pointRadiusPx == ogun::settings::kMaxHighlightPointRadiusPx);
    CHECK(prefs.previewDisplayMode == defaults.previewDisplayMode);
    CHECK(prefs.appearance.backFaceMode == ogun::BackFaceMode::Cull);
    CHECK(prefs.listCap == defaults.listCap);
    CHECK(prefs.overlayCap == 77);
    CHECK(prefs.uiScale == defaults.uiScale);
    // Keys not in the text keep their defaults
    CHECK(prefs.ghostOpacity == defaults.ghostOpacity);
    CHECK(prefs.backgroundColor == defaults.backgroundColor);
    CHECK_FALSE(prefs.showCompass);

    // A truncated / binary file does not crash and gives defaults for what it cannot read
    const ogun::UserPreferences garbage = ogun::parseUserPreferences(std::string("\0\xff=\n= =\nweld_floor", 16));
    CHECK(ogun::sameUserPreferences(garbage, defaults));
}

TEST_CASE("Unknown preference keys are ignored", "[app][preferences]") {
    const ogun::UserPreferences prefs = ogun::parseUserPreferences(
        "future.setting = 12\n"
        "weld_floor = 4e-8\n"
        "base.colour = 0 0 0\n");
    ogun::UserPreferences expected;
    expected.tolerance.absFloor = 4.0e-8;
    CHECK(ogun::sameUserPreferences(prefs, expected));
}

TEST_CASE("Startup applies the saved preferences, live thin-face thresholds included", "[app][preferences]") {
    const std::filesystem::path dir = uniqueTempDir("startup");
    const std::filesystem::path file = ogun::preferencesFilePath(dir);
    const ogun::UserPreferences custom = customPreferences();
    REQUIRE(ogun::saveUserPreferences(custom, file));

    ogun::AppController controller;
    const ogun::UserPreferences loaded = ogun::loadAndApplyUserPreferences(controller, file);
    CHECK(loaded.uiScale == custom.uiScale); // GUI-only fields are handed back

    const ogun::ThinFaceThresholds& live = controller.thresholds();
    CHECK(live.minAltitude == custom.defaults.thinFace.minAltitude);
    CHECK(live.minArea == custom.defaults.thinFace.minArea);
    CHECK(live.maxAspectRatio == custom.defaults.thinFace.maxAspectRatio);
    CHECK(controller.toleranceConfig().absFloor == custom.tolerance.absFloor);
    CHECK(controller.toleranceConfig().relTol == custom.tolerance.relTol);
    CHECK(controller.checkMeshAfterLoad() == custom.checkMeshAfterLoad);
    CHECK(controller.autoCheckDefault(ogun::DefectCategory::ThinFaces) ==
          custom.autoCheckDefaults[ogun::categoryIndex(ogun::DefectCategory::ThinFaces)]);
    CHECK(controller.previewDisplayMode() == custom.previewDisplayMode);
    CHECK(controller.overlayAppearance().pointRadiusPx == custom.appearance.pointRadiusPx);

    // The controller hands back the same values to save (GUI-only fields aside)
    ogun::UserPreferences captured = controller.userPreferences();
    captured.uiScale = custom.uiScale;
    captured.backgroundColor = custom.backgroundColor;
    captured.showCompass = custom.showCompass;
    captured.mouse = custom.mouse;
    CHECK(ogun::sameUserPreferences(captured, custom));

    // A new mesh's Mesh Health rows take the saved Auto defaults
    controller.loadSampleDefectiveMesh();
    CHECK(controller.categoryHealth(ogun::DefectCategory::Degenerate).autoCheck ==
          custom.autoCheckDefaults[ogun::categoryIndex(ogun::DefectCategory::Degenerate)]);

    // Reset to defaults restores the built-in values, live thresholds included
    controller.applyUserPreferences(ogun::UserPreferences{});
    CHECK(controller.thresholds().minAltitude == ogun::settings::kDefaultThinFaceMinAltitude);
    CHECK(ogun::sameUserPreferences(controller.userPreferences(), ogun::UserPreferences{}));
    std::filesystem::remove_all(dir);
}

TEST_CASE("Mouse preferences: presets apply as built, unknown or invalid values fall back to Ogun's", "[app][preferences][mouse]") {
    using ogun::MouseAction;
    using ogun::MouseInput;
    const ogun::MouseMapping ogunDefault;

    // A file from before Preferences > Mouse existed gives the Ogun scheme
    CHECK(ogun::parseUserPreferences("weld_floor = 1e-6\n").mouse == ogunDefault);

    // A built-in preset uses its own bindings, whatever the file stores
    const ogun::UserPreferences blender = ogun::parseUserPreferences(
        "mouse.preset = blender\n"
        "mouse.orbit = right\n"
        "mouse.zoom_speed = 2\n"
        "mouse.invert_wheel_zoom = true\n");
    CHECK(blender.mouse.preset == ogun::MousePreset::Blender);
    CHECK(blender.mouse.bindings == ogun::mousePresetBindings(ogun::MousePreset::Blender));
    CHECK(blender.mouse.zoomSpeed == 2.0f);
    CHECK(blender.mouse.invertWheelZoom);

    // Unknown preset (e.g. from a newer Ogun): the Ogun scheme
    const ogun::UserPreferences future = ogun::parseUserPreferences(
        "mouse.preset = cura\n"
        "mouse.orbit = right\n"
        "mouse.zoom_speed = 1000\n");
    CHECK(future.mouse.preset == ogun::MousePreset::Ogun);
    CHECK(future.mouse.bindings == ogunDefault.bindings);
    CHECK(future.mouse.zoomSpeed == ogun::settings::kMaxMouseZoomSpeed);

    // Custom: valid bindings are kept, invalid ones keep Ogun's binding for that action
    const ogun::UserPreferences custom = ogun::parseUserPreferences(
        "mouse.preset = custom\n"
        "mouse.orbit = middle\n"
        "mouse.pan = banana\n"
        "mouse.zoom_wheel = left\n"     // a button for a wheel action
        "mouse.select = wheel\n"        // the wheel for a click action
        "mouse.clear_pick = none\n"
        "mouse.future_action = left\n"); // unknown key
    CHECK(custom.mouse.preset == ogun::MousePreset::Custom);
    CHECK(custom.mouse.of(MouseAction::Orbit) == ogun::MouseActionBindings{{{MouseInput::Middle, 0}}});
    CHECK(custom.mouse.of(MouseAction::Pan) == ogunDefault.of(MouseAction::Pan));
    CHECK(custom.mouse.of(MouseAction::WheelZoom) == ogunDefault.of(MouseAction::WheelZoom));
    CHECK(custom.mouse.of(MouseAction::Select) == ogunDefault.of(MouseAction::Select));
    CHECK_FALSE(custom.mouse.of(MouseAction::ClearPick)[0].bound());

    // The default file writes the Ogun scheme, readable as text
    const std::string text = ogun::formatUserPreferences(ogun::UserPreferences{});
    CHECK(text.find("mouse.preset = ogun\n") != std::string::npos);
    CHECK(text.find("mouse.pan = shift+left, right, middle\n") != std::string::npos);
}
