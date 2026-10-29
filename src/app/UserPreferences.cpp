#include "app/UserPreferences.hpp"

#include "io/ConfigDirectory.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <fstream>
#include <functional>
#include <iomanip>
#include <limits>
#include <locale>
#include <optional>
#include <sstream>
#include <type_traits>
#include <utility>
#include <vector>

namespace ogun {

namespace {

constexpr double kHuge = std::numeric_limits<double>::max();

/// One saved setting: its key, the section it is written under, and how to format / parse it.
struct Field {
    const char* section;
    std::string key;
    std::function<std::string(const UserPreferences&)> format;
    /// Leaves `prefs` unchanged and returns false when the value is malformed.
    std::function<bool(UserPreferences&, std::string_view)> parse;
    std::function<bool(const UserPreferences&, const UserPreferences&)> equal;
};

template <typename T>
bool sameValue(const T& a, const T& b) {
    if constexpr (std::is_same_v<T, Eigen::Vector3d>) {
        return a.cwiseEqual(b).all();
    } else {
        return a == b;
    }
}

/// Exact comparison of the field `get` reaches in two preference sets.
template <typename Get>
auto equalBy(Get get) {
    return [get](const UserPreferences& a, const UserPreferences& b) { return sameValue(get(a), get(b)); };
}

std::string_view trim(std::string_view s) noexcept {
    const auto first = s.find_first_not_of(" \t\r\n");
    if (first == std::string_view::npos) {
        return {};
    }
    const auto last = s.find_last_not_of(" \t\r\n");
    return s.substr(first, last - first + 1);
}

/// Parses exactly `N` whitespace-separated finite numbers (locale independent).
template <typename T, std::size_t N>
std::optional<std::array<T, N>> parseReals(std::string_view text) {
    std::istringstream in{std::string(text)};
    in.imbue(std::locale::classic());
    std::array<T, N> values{};
    for (T& value : values) {
        if (!(in >> value) || !std::isfinite(value)) {
            return std::nullopt;
        }
    }
    in >> std::ws;
    if (!in.eof()) {
        return std::nullopt;
    }
    return values;
}

/// Shortest of 6..17 significant digits that parses back to exactly `value`.
template <typename T>
std::string formatReal(T value) {
    constexpr int kMaxDigits = std::numeric_limits<T>::max_digits10;
    std::string text;
    for (int digits = 6; digits <= kMaxDigits; ++digits) {
        std::ostringstream out;
        out.imbue(std::locale::classic());
        out << std::setprecision(digits) << value;
        text = out.str();
        const auto back = parseReals<T, 1>(text);
        if (back && (*back)[0] == value) {
            break;
        }
    }
    return text;
}

template <typename T, std::size_t N>
std::string formatReals(const std::array<T, N>& values) {
    std::string text;
    for (std::size_t i = 0; i < N; ++i) {
        text += (i == 0 ? "" : " ") + formatReal(values[i]);
    }
    return text;
}

/// `get` is a generic lambda returning a reference to the field (const or not).
template <typename Get>
Field realField(const char* section, std::string key, Get get, double lo, double hi) {
    using T = std::remove_cvref_t<decltype(get(std::declval<UserPreferences&>()))>;
    return {section, std::move(key),
            [get](const UserPreferences& p) { return formatReal<T>(get(p)); },
            [get, lo, hi](UserPreferences& p, std::string_view text) {
                const auto v = parseReals<double, 1>(text);
                if (!v) {
                    return false;
                }
                get(p) = static_cast<T>(std::clamp((*v)[0], lo, hi));
                return true;
            },
            equalBy(get)};
}

template <typename Get>
Field colorField(const char* section, std::string key, Get get) {
    return {section, std::move(key),
            [get](const UserPreferences& p) {
                const Eigen::Vector3d& c = get(p);
                return formatReals(std::array<double, 3>{c.x(), c.y(), c.z()});
            },
            [get](UserPreferences& p, std::string_view text) {
                const auto v = parseReals<double, 3>(text);
                if (!v) {
                    return false;
                }
                get(p) = Eigen::Vector3d((*v)[0], (*v)[1], (*v)[2]).cwiseMax(0.0).cwiseMin(1.0);
                return true;
            },
            equalBy(get)};
}

template <typename Get>
Field boolField(const char* section, std::string key, Get get) {
    return {section, std::move(key),
            [get](const UserPreferences& p) { return std::string(get(p) ? "true" : "false"); },
            [get](UserPreferences& p, std::string_view text) {
                if (text == "true" || text == "1") {
                    get(p) = true;
                } else if (text == "false" || text == "0") {
                    get(p) = false;
                } else {
                    return false;
                }
                return true;
            },
            equalBy(get)};
}

template <typename Get>
Field countField(const char* section, std::string key, Get get, std::size_t lo) {
    return {section, std::move(key),
            [get](const UserPreferences& p) { return std::to_string(get(p)); },
            [get, lo](UserPreferences& p, std::string_view text) {
                std::size_t value = 0;
                const auto [end, ec] = std::from_chars(text.data(), text.data() + text.size(), value);
                if (ec != std::errc() || end != text.data() + text.size()) {
                    return false;
                }
                get(p) = std::max(value, lo);
                return true;
            },
            equalBy(get)};
}

/// Enum saved by name; `names` lists every value with its key word.
template <typename E, std::size_t N, typename Get>
Field enumField(const char* section, std::string key, Get get, std::array<std::pair<E, const char*>, N> names) {
    return {section, std::move(key),
            [get, names](const UserPreferences& p) {
                for (const auto& [value, name] : names) {
                    if (value == get(p)) {
                        return std::string(name);
                    }
                }
                return std::string(names[0].second);
            },
            [get, names](UserPreferences& p, std::string_view text) {
                for (const auto& [value, name] : names) {
                    if (text == name) {
                        get(p) = value;
                        return true;
                    }
                }
                return false;
            },
            equalBy(get)};
}

/// Stable key word of each Mesh Health row (never the display name, which may change).
const char* categoryKey(DefectCategory category) noexcept {
    switch (category) {
    case DefectCategory::OpenEdges: return "open_edges";
    case DefectCategory::NonManifoldEdges: return "non_manifold_edges";
    case DefectCategory::NonManifoldVertices: return "non_manifold_vertices";
    case DefectCategory::FlippedNormals: return "flipped_normals";
    case DefectCategory::Shells: return "shells";
    case DefectCategory::Intersections: return "intersections";
    case DefectCategory::ThinFaces: return "thin_faces";
    case DefectCategory::Degenerate: return "degenerate";
    case DefectCategory::Duplicates: return "duplicates";
    case DefectCategory::InvertedShells: return "inverted_shells";
    }
    return "unknown";
}

/// Adds color, edges on/off and edge width of one surface style under `prefix`.
template <typename Get>
void addSurfaceStyle(std::vector<Field>& fields, const char* section, const std::string& prefix, Get get) {
    fields.push_back(colorField(section, prefix + "color", [get](auto& p) -> auto& { return get(p).color; }));
    fields.push_back(colorField(section, prefix + "edge_color", [get](auto& p) -> auto& { return get(p).edgeColor; }));
    fields.push_back(realField(section, prefix + "edge_width_px", [get](auto& p) -> auto& { return get(p).edgeWidthPx; },
                               settings::kMinOverlayEdgeWidthPx, settings::kMaxOverlayEdgeWidthPx));
    fields.push_back(boolField(section, prefix + "show_edges", [get](auto& p) -> auto& { return get(p).showEdges; }));
}

const std::vector<Field>& preferenceFields() {
    static const std::vector<Field> fields = [] {
        std::vector<Field> f;
        f.reserve(64);

        const char* tol = "Preferences > Units & Tolerance (file units)";
        f.push_back(realField(tol, "weld_floor", [](auto& p) -> auto& { return p.tolerance.absFloor; }, 0.0, kHuge));
        f.push_back(realField(tol, "relative_tolerance", [](auto& p) -> auto& { return p.tolerance.relTol; }, 0.0, kHuge));
        f.push_back(realField(tol, "thin_max_aspect_ratio",
                              [](auto& p) -> auto& { return p.defaults.thinFace.maxAspectRatio; }, 1.0, kHuge));
        f.push_back(realField(tol, "thin_min_height_floor",
                              [](auto& p) -> auto& { return p.defaults.thinFace.minAltitude; }, 0.0, kHuge));
        f.push_back(realField(tol, "thin_min_area_floor",
                              [](auto& p) -> auto& { return p.defaults.thinFace.minArea; }, 0.0, kHuge));

        const char* diag = "Preferences > Diagnostics";
        f.push_back(boolField(diag, "check_mesh_after_load", [](auto& p) -> auto& { return p.checkMeshAfterLoad; }));
        for (const DefectCategory category : kAllDefectCategories) {
            const std::size_t i = categoryIndex(category);
            f.push_back(boolField(diag, std::string("auto_check.") + categoryKey(category),
                                  [i](auto& p) -> auto& { return p.autoCheckDefaults[i]; }));
        }

        const char* ui = "Preferences > Interface / Performance";
        f.push_back(realField(ui, "ui_scale", [](auto& p) -> auto& { return p.uiScale; },
                              settings::kMinUiScale, settings::kMaxUiScale));
        f.push_back(countField(ui, "list_cap", [](auto& p) -> auto& { return p.listCap; }, 1));
        f.push_back(countField(ui, "overlay_cap", [](auto& p) -> auto& { return p.overlayCap; }, 0));

        const char* base = "Display > Base mesh";
        addSurfaceStyle(f, base, "base.", [](auto& p) -> auto& { return p.appearance.baseMesh; });
        f.push_back(realField(base, "base.ghost_opacity", [](auto& p) -> auto& { return p.ghostOpacity; },
                              settings::kMinGhostOpacity, 1.0));
        f.push_back(enumField(base, "base.back_faces", [](auto& p) -> auto& { return p.appearance.backFaceMode; },
                              std::array<std::pair<BackFaceMode, const char*>, 4>{{
                                  {BackFaceMode::Darker, "darker"},
                                  {BackFaceMode::Identical, "identical"},
                                  {BackFaceMode::Custom, "custom"},
                                  {BackFaceMode::Cull, "cull"},
                              }}));
        f.push_back(colorField(base, "base.back_face_color", [](auto& p) -> auto& { return p.appearance.backFaceColor; }));
        f.push_back(boolField(base, "base.smooth_shading", [](auto& p) -> auto& { return p.appearance.smoothShade; }));
        f.push_back(colorField(base, "face_selection_color",
                               [](auto& p) -> auto& { return p.appearance.faceSelectionColor; }));

        const char* defect = "Display > Defect highlight";
        f.push_back(colorField(defect, "defect.tint_color", [](auto& p) -> auto& { return p.appearance.categoryTintColor; }));
        addSurfaceStyle(f, defect, "defect.selected_", [](auto& p) -> auto& { return p.appearance.selectedFaces; });
        f.push_back(colorField(defect, "defect.point_color", [](auto& p) -> auto& { return p.appearance.pointColor; }));
        f.push_back(realField(defect, "defect.point_radius_px", [](auto& p) -> auto& { return p.appearance.pointRadiusPx; },
                              settings::kMinHighlightPointRadiusPx, settings::kMaxHighlightPointRadiusPx));

        const char* preview = "Display > Repair preview";
        f.push_back(boolField(preview, "preview.show_patch", [](auto& p) -> auto& { return p.repairPreviewVisible; }));
        f.push_back(enumField(preview, "preview.mode", [](auto& p) -> auto& { return p.previewDisplayMode; },
                              std::array<std::pair<PreviewDisplayMode, const char*>, 3>{{
                                  {PreviewDisplayMode::CutoutOriginalEdges, "cutout_original_edges"},
                                  {PreviewDisplayMode::TranslucentBase, "translucent_base"},
                                  {PreviewDisplayMode::TranslucentPatch, "translucent_patch"},
                              }}));
        f.push_back(realField(preview, "preview.base_opacity", [](auto& p) -> auto& { return p.previewBaseOpacity; },
                              settings::kMinPreviewOpacity, 1.0));
        f.push_back(realField(preview, "preview.patch_opacity", [](auto& p) -> auto& { return p.previewPatchOpacity; },
                              settings::kMinPreviewOpacity, 1.0));
        addSurfaceStyle(f, preview, "preview.patch_", [](auto& p) -> auto& { return p.appearance.previewPatch; });
        f.push_back(colorField(preview, "preview.original_edge_color",
                               [](auto& p) -> auto& { return p.appearance.originalEdgeColor; }));
        f.push_back(realField(preview, "preview.original_edge_width_px",
                              [](auto& p) -> auto& { return p.appearance.originalEdgeWidthPx; },
                              settings::kMinOverlayEdgeWidthPx, settings::kMaxOverlayEdgeWidthPx));
        f.push_back(realField(preview, "preview.original_edge_opacity",
                              [](auto& p) -> auto& { return p.appearance.originalEdgeOpacity; },
                              settings::kMinPreviewOpacity, 1.0));

        const char* view = "Display > Appearance";
        f.push_back({view, "background_color",
                     [](const UserPreferences& p) { return formatReals(p.backgroundColor); },
                     [](UserPreferences& p, std::string_view text) {
                         const auto v = parseReals<float, 4>(text);
                         if (!v) {
                             return false;
                         }
                         for (std::size_t i = 0; i < 4; ++i) {
                             p.backgroundColor[i] = std::clamp((*v)[i], 0.0f, 1.0f);
                         }
                         return true;
                     },
                     equalBy([](auto& p) -> auto& { return p.backgroundColor; })});
        f.push_back(boolField("View", "show_compass", [](auto& p) -> auto& { return p.showCompass; }));

        // A built-in preset's bindings are applied after parsing (`applyMousePresetBindings`); the
        // binding keys are written anyway so the file shows the active scheme
        const char* mouse = "Preferences > Mouse";
        f.push_back(enumField(mouse, "mouse.preset", [](auto& p) -> auto& { return p.mouse.preset; },
                              std::array<std::pair<MousePreset, const char*>, kMousePresetCount>{{
                                  {MousePreset::Ogun, mousePresetKey(MousePreset::Ogun)},
                                  {MousePreset::Blender, mousePresetKey(MousePreset::Blender)},
                                  {MousePreset::OpenScad, mousePresetKey(MousePreset::OpenScad)},
                                  {MousePreset::Maya, mousePresetKey(MousePreset::Maya)},
                                  {MousePreset::Fusion360, mousePresetKey(MousePreset::Fusion360)},
                                  {MousePreset::SolidWorks, mousePresetKey(MousePreset::SolidWorks)},
                                  {MousePreset::Custom, mousePresetKey(MousePreset::Custom)},
                              }}));
        f.push_back(boolField(mouse, "mouse.invert_wheel_zoom", [](auto& p) -> auto& { return p.mouse.invertWheelZoom; }));
        f.push_back(realField(mouse, "mouse.zoom_speed", [](auto& p) -> auto& { return p.mouse.zoomSpeed; },
                              settings::kMinMouseZoomSpeed, settings::kMaxMouseZoomSpeed));
        for (const MouseAction action : kAllMouseActions) {
            const std::size_t i = static_cast<std::size_t>(action);
            f.push_back({mouse, std::string("mouse.") + mouseActionKey(action),
                         [i](const UserPreferences& p) { return formatMouseBindingList(p.mouse.bindings[i]); },
                         [i, action](UserPreferences& p, std::string_view text) {
                             const auto bindings = parseMouseBindingList(text, action);
                             if (!bindings) {
                                 return false;
                             }
                             p.mouse.bindings[i] = *bindings;
                             return true;
                         },
                         equalBy([i](auto& p) -> auto& { return p.mouse.bindings[i]; })});
        }
        return f;
    }();
    return fields;
}

/// Thin-face keys written before faces were judged by aspect ratio. Read only when the new key is
/// absent; a value equal to the old default loads as the new default, any other value is kept.
struct LegacyThinKey {
    const char* oldKey;
    const char* newKey;
    double oldDefault;
    double newDefault;
    double ThinFaceThresholds::* member;
};

constexpr std::array<LegacyThinKey, 3> kLegacyThinKeys = {{
    {"default_max_aspect_ratio", "thin_max_aspect_ratio", settings::kLegacyThinFaceMaxAspectRatio,
     settings::kDefaultThinFaceMaxAspectRatio, &ThinFaceThresholds::maxAspectRatio},
    {"default_min_altitude", "thin_min_height_floor", settings::kLegacyThinFaceMinAltitude,
     settings::kDefaultThinFaceMinAltitude, &ThinFaceThresholds::minAltitude},
    {"default_min_area", "thin_min_area_floor", settings::kLegacyThinFaceMinArea, settings::kDefaultThinFaceMinArea,
     &ThinFaceThresholds::minArea},
}};

} // namespace

std::array<bool, kDefectCategoryCount> builtInAutoCheckDefaults() noexcept {
    std::array<bool, kDefectCategoryCount> defaults{};
    for (const DefectCategory category : kAllDefectCategories) {
        defaults[categoryIndex(category)] = categoryInfo(category).autoDefault;
    }
    return defaults;
}

std::filesystem::path preferencesFilePath(const std::filesystem::path& configDir) {
    return configDir.empty() ? std::filesystem::path() : configDir / kPreferencesFileName;
}

std::string formatUserPreferences(const UserPreferences& prefs) {
    std::string text = "# Ogun preferences: written whenever a preference changes. Unknown keys are ignored;\n"
                       "# a malformed value falls back to its default. Colors are \"r g b\" in [0, 1].\n";
    const char* section = nullptr;
    for (const Field& field : preferenceFields()) {
        if (field.section != section) {
            section = field.section;
            text += std::string("\n# ") + section + "\n";
        }
        text += field.key + " = " + field.format(prefs) + "\n";
    }
    return text;
}

bool sameUserPreferences(const UserPreferences& a, const UserPreferences& b) {
    const std::vector<Field>& fields = preferenceFields();
    return std::all_of(fields.begin(), fields.end(), [&](const Field& f) { return f.equal(a, b); });
}

std::vector<std::string> userPreferenceKeys() {
    std::vector<std::string> keys;
    keys.reserve(preferenceFields().size());
    for (const Field& field : preferenceFields()) {
        keys.push_back(field.key);
    }
    return keys;
}

UserPreferences parseUserPreferences(std::string_view text) {
    UserPreferences prefs;
    const std::vector<Field>& fields = preferenceFields();
    std::vector<std::string_view> seenKeys;
    std::array<std::optional<std::string_view>, kLegacyThinKeys.size()> legacyValues;
    while (!text.empty()) {
        const auto eol = text.find('\n');
        const std::string_view line = trim(text.substr(0, eol));
        text = eol == std::string_view::npos ? std::string_view() : text.substr(eol + 1);
        if (line.empty() || line.front() == '#') {
            continue;
        }
        const auto eq = line.find('=');
        if (eq == std::string_view::npos) {
            continue;
        }
        const std::string_view key = trim(line.substr(0, eq));
        const std::string_view value = trim(line.substr(eq + 1));
        const auto it = std::find_if(fields.begin(), fields.end(), [&](const Field& f) { return f.key == key; });
        if (it != fields.end()) {
            it->parse(prefs, value); // a malformed value keeps the default
            seenKeys.push_back(key);
        }
        for (std::size_t i = 0; i < kLegacyThinKeys.size(); ++i) {
            if (key == kLegacyThinKeys[i].oldKey) {
                legacyValues[i] = value;
            }
        }
    }
    for (std::size_t i = 0; i < kLegacyThinKeys.size(); ++i) {
        const LegacyThinKey& legacy = kLegacyThinKeys[i];
        if (!legacyValues[i] || std::find(seenKeys.begin(), seenKeys.end(), legacy.newKey) != seenKeys.end()) {
            continue;
        }
        const auto field = std::find_if(fields.begin(), fields.end(), [&](const Field& f) { return f.key == legacy.newKey; });
        if (field != fields.end() && field->parse(prefs, *legacyValues[i])) {
            double& value = prefs.defaults.thinFace.*legacy.member;
            if (value == legacy.oldDefault) {
                value = legacy.newDefault;
            }
        }
    }
    applyMousePresetBindings(prefs.mouse);
    return prefs;
}

UserPreferences loadUserPreferences(const std::filesystem::path& file) {
    if (file.empty()) {
        return {};
    }
    std::ifstream in(file, std::ios::binary);
    if (!in) {
        return {};
    }
    std::ostringstream text;
    text << in.rdbuf();
    return parseUserPreferences(text.str());
}

bool saveUserPreferences(const UserPreferences& prefs, const std::filesystem::path& file, std::string* error) {
    return writeFileAtomically(file, formatUserPreferences(prefs), error);
}

UserPreferences loadAndApplyUserPreferences(AppController& controller, const std::filesystem::path& file) {
    UserPreferences prefs = loadUserPreferences(file);
    controller.applyUserPreferences(prefs);
    return prefs;
}

} // namespace ogun
