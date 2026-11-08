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

/// Fresh directory under the system temp dir (never the real ~/.config/mesheal).
std::filesystem::path uniqueTempDir(const char* tag) {
    const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
    return std::filesystem::temp_directory_path() / (std::string("mesheal_prefs_") + tag + "_" + std::to_string(stamp));
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
mesheal::UserPreferences customPreferences() {
    mesheal::UserPreferences p;
    const auto styled = [](mesheal::SurfaceStyle& s, double shade) {
        s.color = Eigen::Vector3d(shade, 0.25, 0.75);
        s.edgeColor = Eigen::Vector3d(0.125, shade, 0.5);
        s.edgeWidthPx = 2.5 + shade;
        s.showEdges = !s.showEdges;
    };
    styled(p.appearance.baseMesh, 0.1);
    styled(p.appearance.selectedFaces, 0.2);
    styled(p.appearance.previewPatch, 0.3);
    p.appearance.backFaceMode = mesheal::BackFaceMode::Custom;
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
    p.previewDisplayMode = mesheal::PreviewDisplayMode::TranslucentPatch;
    p.previewBaseOpacity = 0.7;
    p.previewPatchOpacity = 0.8;
    p.wholeMeshPreview.original = mesheal::WholeMeshOriginalStyle::Wireframe;
    p.wholeMeshPreview.resultOpacity = 0.45;
    p.wholeMeshPreview.originalOpacity = 0.25;
    p.wholeMeshPreview.changedFacesOnly = true;
    p.tolerance.absFloor = 3.5e-7;
    p.tolerance.relTol = 2.0e-6;
    p.defaults.thinFace.minAltitude = 0.02;
    p.defaults.thinFace.minArea = 4.0e-5;
    p.defaults.thinFace.maxAspectRatio = 250.0;
    p.defaults.fixAll.passes = 5;
    p.defaults.fixAll.timeBudgetSeconds = 120.0;
    p.defaults.fixAll.weld = !p.defaults.fixAll.weld;
    p.defaults.fixAll.weldTolerance.absFloor = 2.0e-5;
    p.defaults.fixAll.weldTolerance.relTol = 3.0e-7;
    p.defaults.fixAll.removeTinyShells = !p.defaults.fixAll.removeTinyShells;
    p.defaults.fixAll.tinyShellFraction = 0.0025;
    p.defaults.fixAll.removeFins = !p.defaults.fixAll.removeFins;
    p.defaults.fixAll.closeCracks = !p.defaults.fixAll.closeCracks;
    p.defaults.fixAll.healThinFaces = !p.defaults.fixAll.healThinFaces;
    p.checkMeshAfterLoad = !p.checkMeshAfterLoad;
    for (bool& autoCheck : p.autoCheckDefaults) {
        autoCheck = !autoCheck;
    }
    p.listCap = 42;
    p.overlayCap = 1234;
    p.uiScale = 1.35f;
    p.backgroundColor = {0.1f, 0.2f, 0.3f, 1.0f};
    p.showCompass = !p.showCompass;
    p.theme = mesheal::UiTheme::Dark;
    // A custom mouse scheme that changes every action's bindings
    using mesheal::MouseAction;
    using mesheal::MouseInput;
    p.mouse.preset = mesheal::MousePreset::Custom;
    p.mouse.invertWheelZoom = true;
    p.mouse.zoomSpeed = 1.5f;
    const auto set = [&p](MouseAction action, mesheal::MouseActionBindings bindings) {
        p.mouse.bindings[static_cast<std::size_t>(action)] = bindings;
    };
    set(MouseAction::Orbit, {{{MouseInput::Middle, 0}}});
    set(MouseAction::Pan, {{{MouseInput::Middle, mesheal::kMouseModShift}, {MouseInput::Right, mesheal::kMouseModAlt}}});
    set(MouseAction::Zoom, {{{MouseInput::Middle, mesheal::kMouseModCtrl}}});
    set(MouseAction::WheelZoom, {{{MouseInput::Wheel, mesheal::kMouseModCtrl}}});
    set(MouseAction::NearClip, {{{MouseInput::Wheel, mesheal::kMouseModAlt}}});
    set(MouseAction::Select, {{{MouseInput::Right, 0}}});
    set(MouseAction::ToggleFace, {{{MouseInput::Right, mesheal::kMouseModCtrl}}});
    set(MouseAction::SelectShell, {{{MouseInput::Right, mesheal::kMouseModAlt}}});
    set(MouseAction::ClearPick, {});
    return p;
}

} // namespace

TEST_CASE("User preferences round trip every field through the config file", "[app][preferences]") {
    const mesheal::UserPreferences custom = customPreferences();
    const mesheal::UserPreferences defaults;

    // Every key of the file is changed by customPreferences(), so the round trip covers every field
    const auto customValues = keyValues(mesheal::formatUserPreferences(custom));
    const auto defaultValues = keyValues(mesheal::formatUserPreferences(defaults));
    const std::vector<std::string> keys = mesheal::userPreferenceKeys();
    REQUIRE(customValues.size() == keys.size());
    REQUIRE(defaultValues.size() == keys.size());
    for (const std::string& key : keys) {
        INFO("key: " << key);
        REQUIRE(customValues.count(key) == 1);
        CHECK(customValues.at(key) != defaultValues.at(key));
    }
    CHECK_FALSE(mesheal::sameUserPreferences(custom, defaults));

    const std::filesystem::path dir = uniqueTempDir("roundtrip");
    const std::filesystem::path file = mesheal::preferencesFilePath(dir / "mesheal");
    CHECK(file.filename() == mesheal::kPreferencesFileName);
    std::string error;
    REQUIRE(mesheal::saveUserPreferences(custom, file, &error)); // creates the directory
    REQUIRE(std::filesystem::exists(file));

    const mesheal::UserPreferences loaded = mesheal::loadUserPreferences(file);
    CHECK(mesheal::sameUserPreferences(loaded, custom));
    CHECK(mesheal::formatUserPreferences(loaded) == mesheal::formatUserPreferences(custom));
    CHECK(loaded.appearance.baseMesh.color.isApprox(custom.appearance.baseMesh.color));
    CHECK(loaded.defaults.thinFace.minAltitude == custom.defaults.thinFace.minAltitude);
    CHECK(loaded.uiScale == custom.uiScale);
    CHECK(loaded.backgroundColor == custom.backgroundColor);
    CHECK(loaded.showCompass == custom.showCompass);
    CHECK(loaded.mouse == custom.mouse);
    CHECK(loaded.theme == custom.theme);

    // Saving again overwrites in place
    REQUIRE(mesheal::saveUserPreferences(defaults, file, &error));
    CHECK(mesheal::sameUserPreferences(mesheal::loadUserPreferences(file), defaults));
    std::filesystem::remove_all(dir);
}

TEST_CASE("A missing preferences file gives the built-in defaults", "[app][preferences]") {
    const std::filesystem::path dir = uniqueTempDir("missing");
    const mesheal::UserPreferences loaded = mesheal::loadUserPreferences(mesheal::preferencesFilePath(dir));
    CHECK(mesheal::sameUserPreferences(loaded, mesheal::UserPreferences{}));
    CHECK(mesheal::sameUserPreferences(mesheal::loadUserPreferences({}), mesheal::UserPreferences{}));
    CHECK(mesheal::preferencesFilePath({}).empty());
    CHECK_FALSE(std::filesystem::exists(dir));
}

TEST_CASE("Malformed or partial preference files fall back per key", "[app][preferences]") {
    const mesheal::UserPreferences defaults;
    const mesheal::UserPreferences prefs = mesheal::parseUserPreferences(
        "# comment\n"
        "weld_floor = 2e-6\n"                      // valid
        "relative_tolerance = abc\n"               // malformed: default
        "  thin_min_height_floor=0.5  \r\n"        // spacing and CRLF are fine
        "thin_min_area_floor = 1e-3 junk\n"        // trailing junk: default
        "thin_max_aspect_ratio = 0.5\n"            // below 1: clamped
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
    CHECK(prefs.appearance.pointRadiusPx == mesheal::settings::kMaxHighlightPointRadiusPx);
    CHECK(prefs.previewDisplayMode == defaults.previewDisplayMode);
    CHECK(prefs.appearance.backFaceMode == mesheal::BackFaceMode::Cull);
    CHECK(prefs.listCap == defaults.listCap);
    CHECK(prefs.overlayCap == 77);
    CHECK(prefs.uiScale == defaults.uiScale);
    // Keys not in the text keep their defaults
    CHECK(prefs.ghostOpacity == defaults.ghostOpacity);
    CHECK(prefs.backgroundColor == defaults.backgroundColor);
    CHECK_FALSE(prefs.showCompass);

    // A truncated / binary file does not crash and gives defaults for what it cannot read
    const mesheal::UserPreferences garbage = mesheal::parseUserPreferences(std::string("\0\xff=\n= =\nweld_floor", 16));
    CHECK(mesheal::sameUserPreferences(garbage, defaults));
}

TEST_CASE("Old thin-face defaults migrate to 100 : 1 with floors off", "[app][preferences]") {
    // A file saved before thin faces were judged by aspect ratio, at the old built-in defaults
    const mesheal::UserPreferences prefs = mesheal::parseUserPreferences(
        "default_min_altitude = 0.0001\n"
        "default_min_area = 1e-08\n"
        "default_max_aspect_ratio = 1000\n");
    CHECK(prefs.defaults.thinFace.maxAspectRatio == mesheal::settings::kDefaultThinFaceMaxAspectRatio);
    CHECK(prefs.defaults.thinFace.minAltitude == 0.0);
    CHECK(prefs.defaults.thinFace.minArea == 0.0);
    CHECK(mesheal::sameUserPreferences(prefs, mesheal::UserPreferences{}));

    // Saving writes only the new keys, so the migration happens once
    const std::string text = mesheal::formatUserPreferences(prefs);
    CHECK(text.find("default_max_aspect_ratio") == std::string::npos);
    CHECK(text.find("thin_max_aspect_ratio = 100\n") != std::string::npos);
    CHECK(text.find("thin_min_height_floor = 0\n") != std::string::npos);
}

TEST_CASE("Customised thin values are kept", "[app][preferences]") {
    // Old keys with values the user changed keep them
    const mesheal::UserPreferences custom = mesheal::parseUserPreferences(
        "default_min_altitude = 0.02\n"
        "default_min_area = 4e-05\n"
        "default_max_aspect_ratio = 250\n");
    CHECK(custom.defaults.thinFace.minAltitude == 0.02);
    CHECK(custom.defaults.thinFace.minArea == 4.0e-5);
    CHECK(custom.defaults.thinFace.maxAspectRatio == 250.0);

    // Each value migrates on its own: a changed aspect stays, untouched floors turn off
    const mesheal::UserPreferences mixed = mesheal::parseUserPreferences(
        "default_min_altitude = 0.0001\n"
        "default_min_area = 1e-08\n"
        "default_max_aspect_ratio = 500\n");
    CHECK(mixed.defaults.thinFace.maxAspectRatio == 500.0);
    CHECK(mixed.defaults.thinFace.minAltitude == 0.0);
    CHECK(mixed.defaults.thinFace.minArea == 0.0);

    // A 1000 : 1 limit chosen after the migration is saved under the new key and kept
    mesheal::UserPreferences chosen;
    chosen.defaults.thinFace.maxAspectRatio = 1000.0;
    chosen.defaults.thinFace.minAltitude = 1.0e-4;
    const mesheal::UserPreferences reloaded = mesheal::parseUserPreferences(mesheal::formatUserPreferences(chosen));
    CHECK(reloaded.defaults.thinFace.maxAspectRatio == 1000.0);
    CHECK(reloaded.defaults.thinFace.minAltitude == 1.0e-4);

    // The new key wins over an old one in the same file, in either order
    const mesheal::UserPreferences both = mesheal::parseUserPreferences(
        "default_max_aspect_ratio = 300\n"
        "thin_max_aspect_ratio = 1000\n"
        "thin_min_height_floor = 0.5\n"
        "default_min_altitude = 0.25\n");
    CHECK(both.defaults.thinFace.maxAspectRatio == 1000.0);
    CHECK(both.defaults.thinFace.minAltitude == 0.5);
}

TEST_CASE("Unknown preference keys are ignored", "[app][preferences]") {
    const mesheal::UserPreferences prefs = mesheal::parseUserPreferences(
        "future.setting = 12\n"
        "weld_floor = 4e-8\n"
        "base.colour = 0 0 0\n");
    mesheal::UserPreferences expected;
    expected.tolerance.absFloor = 4.0e-8;
    CHECK(mesheal::sameUserPreferences(prefs, expected));
}

TEST_CASE("Startup applies the saved preferences, live thin-face thresholds included", "[app][preferences]") {
    const std::filesystem::path dir = uniqueTempDir("startup");
    const std::filesystem::path file = mesheal::preferencesFilePath(dir);
    const mesheal::UserPreferences custom = customPreferences();
    REQUIRE(mesheal::saveUserPreferences(custom, file));

    mesheal::AppController controller;
    const mesheal::UserPreferences loaded = mesheal::loadAndApplyUserPreferences(controller, file);
    CHECK(loaded.uiScale == custom.uiScale); // GUI-only fields are handed back

    const mesheal::ThinFaceThresholds& live = controller.thresholds();
    CHECK(live.minAltitude == custom.defaults.thinFace.minAltitude);
    CHECK(live.minArea == custom.defaults.thinFace.minArea);
    CHECK(live.maxAspectRatio == custom.defaults.thinFace.maxAspectRatio);
    CHECK(controller.toleranceConfig().absFloor == custom.tolerance.absFloor);
    CHECK(controller.toleranceConfig().relTol == custom.tolerance.relTol);
    CHECK(controller.checkMeshAfterLoad() == custom.checkMeshAfterLoad);
    CHECK(controller.autoCheckDefault(mesheal::DefectCategory::ThinFaces) ==
          custom.autoCheckDefaults[mesheal::categoryIndex(mesheal::DefectCategory::ThinFaces)]);
    CHECK(controller.previewDisplayMode() == custom.previewDisplayMode);
    CHECK(controller.overlayAppearance().pointRadiusPx == custom.appearance.pointRadiusPx);

    // The controller hands back the same values to save (GUI-only fields aside)
    mesheal::UserPreferences captured = controller.userPreferences();
    captured.uiScale = custom.uiScale;
    captured.backgroundColor = custom.backgroundColor;
    captured.showCompass = custom.showCompass;
    captured.mouse = custom.mouse;
    captured.theme = custom.theme;
    CHECK(mesheal::sameUserPreferences(captured, custom));

    // A new mesh's Mesh Health rows take the saved Auto defaults
    controller.loadSampleDefectiveMesh();
    CHECK(controller.categoryHealth(mesheal::DefectCategory::Degenerate).autoCheck ==
          custom.autoCheckDefaults[mesheal::categoryIndex(mesheal::DefectCategory::Degenerate)]);

    // Reset to defaults restores the built-in values, live thresholds included
    controller.applyUserPreferences(mesheal::UserPreferences{});
    CHECK(controller.thresholds().minAltitude == mesheal::settings::kDefaultThinFaceMinAltitude);
    CHECK(mesheal::sameUserPreferences(controller.userPreferences(), mesheal::UserPreferences{}));
    std::filesystem::remove_all(dir);
}

TEST_CASE("Mouse preferences: presets apply as built, unknown or invalid values fall back to Mesheal's", "[app][preferences][mouse]") {
    using mesheal::MouseAction;
    using mesheal::MouseInput;
    const mesheal::MouseMapping meshealDefault;

    // A file from before Preferences > Mouse existed gives the Mesheal scheme
    CHECK(mesheal::parseUserPreferences("weld_floor = 1e-6\n").mouse == meshealDefault);

    // A built-in preset uses its own bindings, whatever the file stores
    const mesheal::UserPreferences blender = mesheal::parseUserPreferences(
        "mouse.preset = blender\n"
        "mouse.orbit = right\n"
        "mouse.zoom_speed = 2\n"
        "mouse.invert_wheel_zoom = true\n");
    CHECK(blender.mouse.preset == mesheal::MousePreset::Blender);
    CHECK(blender.mouse.bindings == mesheal::mousePresetBindings(mesheal::MousePreset::Blender));
    CHECK(blender.mouse.zoomSpeed == 2.0f);
    CHECK(blender.mouse.invertWheelZoom);

    // Unknown preset (e.g. from a newer Mesheal): the Mesheal scheme
    const mesheal::UserPreferences future = mesheal::parseUserPreferences(
        "mouse.preset = cura\n"
        "mouse.orbit = right\n"
        "mouse.zoom_speed = 1000\n");
    CHECK(future.mouse.preset == mesheal::MousePreset::Mesheal);
    CHECK(future.mouse.bindings == meshealDefault.bindings);
    CHECK(future.mouse.zoomSpeed == mesheal::settings::kMaxMouseZoomSpeed);

    // Custom: valid bindings are kept, invalid ones keep Mesheal's binding for that action
    const mesheal::UserPreferences custom = mesheal::parseUserPreferences(
        "mouse.preset = custom\n"
        "mouse.orbit = middle\n"
        "mouse.pan = banana\n"
        "mouse.zoom_wheel = left\n"     // a button for a wheel action
        "mouse.select = wheel\n"        // the wheel for a click action
        "mouse.clear_pick = none\n"
        "mouse.future_action = left\n"); // unknown key
    CHECK(custom.mouse.preset == mesheal::MousePreset::Custom);
    CHECK(custom.mouse.of(MouseAction::Orbit) == mesheal::MouseActionBindings{{{MouseInput::Middle, 0}}});
    CHECK(custom.mouse.of(MouseAction::Pan) == meshealDefault.of(MouseAction::Pan));
    CHECK(custom.mouse.of(MouseAction::WheelZoom) == meshealDefault.of(MouseAction::WheelZoom));
    CHECK(custom.mouse.of(MouseAction::Select) == meshealDefault.of(MouseAction::Select));
    CHECK_FALSE(custom.mouse.of(MouseAction::ClearPick)[0].bound());

    // The default file writes the Mesheal scheme, readable as text
    const std::string text = mesheal::formatUserPreferences(mesheal::UserPreferences{});
    CHECK(text.find("mouse.preset = mesheal\n") != std::string::npos);
    CHECK(text.find("mouse.pan = shift+left, right, middle\n") != std::string::npos);
}

TEST_CASE("Fix All preferences: defaults, limits, refused zero, malformed values and the load weld", "[app][preferences][fixall]") {
    using namespace mesheal;
    const UserPreferences defaults;
    CHECK(defaults.defaults.fixAll.passes == 3);
    CHECK(defaults.defaults.fixAll.weld);
    CHECK(defaults.defaults.fixAll.weldTolerance.absFloor == 1.0e-6);
    CHECK(defaults.defaults.fixAll.weldTolerance.relTol == 1.0e-6);
    CHECK(defaults.defaults.fixAll.tinyShellFraction == 1.0e-3);
    CHECK(defaults.defaults.fixAll.removeTinyShells);
    CHECK(defaults.defaults.fixAll.removeFins);
    CHECK(defaults.defaults.fixAll.closeCracks);
    CHECK(defaults.defaults.fixAll.healThinFaces);
    CHECK(defaults.tolerance.absFloor == settings::kDefaultWeldAbsFloor); // the load weld stays 1e-9

    // Limits clamp; rel = 0 (or negative) is refused and keeps the default
    const UserPreferences limits = parseUserPreferences(
        "fixall.passes = 99\nfixall.weld_floor = 1\nfixall.weld_rel = 1e-12\nfixall.tiny_shell_fraction = 5\n"
        "fixall.time_budget = -3\n");
    CHECK(limits.defaults.fixAll.passes == settings::kFixAllMaxPasses);
    CHECK(limits.defaults.fixAll.weldTolerance.absFloor == settings::kFixAllWeldToleranceMax);
    CHECK(limits.defaults.fixAll.weldTolerance.relTol == settings::kFixAllWeldToleranceMin);
    CHECK(limits.defaults.fixAll.tinyShellFraction == settings::kFixAllTinyShellFractionMax);
    CHECK(limits.defaults.fixAll.timeBudgetSeconds == 0.0);
    CHECK(parseUserPreferences("fixall.passes = 0\n").defaults.fixAll.passes == settings::kFixAllMinPasses);
    CHECK(parseUserPreferences("fixall.weld_rel = 0\n").defaults.fixAll.weldTolerance.relTol ==
          settings::kDefaultFixAllWeldRelTol);
    CHECK(parseUserPreferences("fixall.weld_rel = -1e-5\n").defaults.fixAll.weldTolerance.relTol ==
          settings::kDefaultFixAllWeldRelTol);

    // Malformed values fall back per key; the other keys of the file still apply
    const UserPreferences bad = parseUserPreferences(
        "fixall.passes = many\nfixall.weld = maybe\nfixall.weld_floor = nan\nfixall.fins = false\n"
        "fixall.tiny_shell_fraction = 1 2\nfixall.cracks = 0\n");
    CHECK(bad.defaults.fixAll.passes == 3);
    CHECK(bad.defaults.fixAll.weld);
    CHECK(bad.defaults.fixAll.weldTolerance.absFloor == settings::kDefaultFixAllWeldAbsFloor);
    CHECK(bad.defaults.fixAll.tinyShellFraction == settings::kDefaultFixAllTinyShellFraction);
    CHECK_FALSE(bad.defaults.fixAll.removeFins);
    CHECK_FALSE(bad.defaults.fixAll.closeCracks);

    // Editing Fix All never touches the load weld keys, and the written text is stable
    UserPreferences edited = parseUserPreferences("weld_floor = 4e-8\nrelative_tolerance = 5e-8\n");
    edited.defaults.fixAll.weldTolerance.absFloor = 7.5e-5;
    const UserPreferences back = parseUserPreferences(formatUserPreferences(edited));
    CHECK(back.tolerance.absFloor == 4.0e-8);
    CHECK(back.tolerance.relTol == 5.0e-8);
    CHECK(back.defaults.fixAll.weldTolerance.absFloor == 7.5e-5);
    CHECK(keyValues(formatUserPreferences(edited)).at("fixall.weld_floor") == "7.5e-05");
}

TEST_CASE("Fix All preferences reach the controller without touching the merge box or the load weld", "[app][preferences][fixall]") {
    using namespace mesheal;
    AppController controller;
    UserPreferences prefs;
    prefs.defaults.fixAll.passes = 6;
    prefs.defaults.fixAll.weld = false;
    prefs.defaults.fixAll.weldTolerance = {2.0e-5, 4.0e-6};
    prefs.defaults.fixAll.removeTinyShells = false;
    prefs.defaults.fixAll.tinyShellFraction = 0.002;
    prefs.defaults.fixAll.healThinFaces = false;

    controller.setFixAllMergeShells(true);
    controller.applyUserPreferences(prefs);
    CHECK(controller.fixAllParams().mergeShells); // untouched
    CHECK(controller.fixAllParams().passes == 6);
    CHECK_FALSE(controller.fixAllParams().weld);
    CHECK(controller.fixAllParams().weldTolerance.absFloor == 2.0e-5);
    CHECK(controller.fixAllParams().tinyShellFraction == 0.002);
    CHECK_FALSE(controller.fixAllParams().healThinFaces);
    CHECK(controller.toleranceConfig().absFloor == settings::kDefaultWeldAbsFloor);
    CHECK(controller.toleranceConfig().relTol == settings::kDefaultWeldRelTol);
    CHECK(sameUserPreferences(controller.userPreferences(), prefs));

    // The panel opens with the saved settings; One Pass and panel edits are not saved
    controller.loadSampleDefectiveMesh();
    controller.openFixAll(1);
    CHECK(controller.fixAllParams().passes == 1);
    CHECK_FALSE(controller.fixAllParams().weld);
    FixAllParams edited = controller.fixAllParams();
    edited.weld = true;
    controller.setFixAllParams(edited);
    CHECK(controller.userPreferences().defaults.fixAll.passes == 6);
    CHECK_FALSE(controller.userPreferences().defaults.fixAll.weld);
    controller.closeTool();

    // Limits apply and Reset to defaults restores the built-in values
    prefs.defaults.fixAll.weldTolerance = {1.0, 0.0};
    controller.setFixAllDefaults(prefs.defaults.fixAll);
    CHECK(controller.preferenceDefaults().fixAll.weldTolerance.absFloor == settings::kFixAllWeldToleranceMax);
    CHECK(controller.preferenceDefaults().fixAll.weldTolerance.relTol == settings::kFixAllWeldToleranceMin);
    controller.applyUserPreferences(UserPreferences{});
    CHECK(controller.fixAllParams().passes == 3);
    CHECK(controller.fixAllParams().weld);
    CHECK(sameUserPreferences(controller.userPreferences(), UserPreferences{}));
}

TEST_CASE("Theme preference: saved by key, unknown values fall back to Custom Light", "[app][preferences][theme]") {
    CHECK(mesheal::parseUserPreferences("").theme == mesheal::UiTheme::CustomLight);
    CHECK(mesheal::parseUserPreferences("theme = dark\n").theme == mesheal::UiTheme::Dark);
    CHECK(mesheal::parseUserPreferences("theme = polyscope_default\n").theme == mesheal::UiTheme::PolyscopeDefault);
    CHECK(mesheal::parseUserPreferences("theme = solarized\n").theme == mesheal::UiTheme::CustomLight);
}
