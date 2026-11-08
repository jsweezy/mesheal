#include "app/UiTheme.hpp"

#include <catch2/catch_test_macros.hpp>

#include <set>
#include <string>

TEST_CASE("UI themes have distinct names and keys", "[app][theme]") {
    std::set<std::string> names;
    std::set<std::string> keys;
    for (const mesheal::UiTheme theme : mesheal::kAllUiThemes) {
        names.insert(mesheal::uiThemeName(theme));
        keys.insert(mesheal::uiThemeKey(theme));
    }
    CHECK(names.size() == mesheal::kUiThemeCount);
    CHECK(keys.size() == mesheal::kUiThemeCount);
    CHECK(std::string(mesheal::uiThemeKey(mesheal::UiTheme::PolyscopeDefault)) == "polyscope_default");
}
