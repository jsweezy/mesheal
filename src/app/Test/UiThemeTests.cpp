#include "app/UiTheme.hpp"

#include <catch2/catch_test_macros.hpp>

#include <set>
#include <string>

TEST_CASE("UI themes have distinct names and keys", "[app][theme]") {
    std::set<std::string> names;
    std::set<std::string> keys;
    for (const ogun::UiTheme theme : ogun::kAllUiThemes) {
        names.insert(ogun::uiThemeName(theme));
        keys.insert(ogun::uiThemeKey(theme));
    }
    CHECK(names.size() == ogun::kUiThemeCount);
    CHECK(keys.size() == ogun::kUiThemeCount);
    CHECK(std::string(ogun::uiThemeKey(ogun::UiTheme::PolyscopeDefault)) == "polyscope_default");
}
