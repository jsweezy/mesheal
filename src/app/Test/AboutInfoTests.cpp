#include "app/AboutInfo.hpp"

#include "mesheal/Version.hpp"

#include <catch2/catch_test_macros.hpp>

#include <string>

using namespace mesheal;

TEST_CASE("copyright notice has the exact GPL header", "[about]") {
    const std::string text(copyrightNotice());
    CHECK(text.starts_with("Mesheal -- A STL/Triangle mesh repair utility for engineering applications.\n"
                           "Copyright (C) 2026 by Jeremy Sweezy\n\n"));
    CHECK(text.find("This program is free software: you can redistribute it and/or modify\n") != std::string::npos);
    CHECK(text.find("MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the\n") != std::string::npos);
    CHECK(text.find("<https://www.gnu.org/licenses/>.") != std::string::npos);
}

TEST_CASE("build info text lists version, compiler and every dependency", "[about]") {
    const std::string text = buildInfoText();
    CHECK(text.find(std::string(kVersionString)) != std::string::npos);
    CHECK(text.find(std::string(kCompilerId)) != std::string::npos);
    CHECK(text.find(std::string(kBuildTimestamp)) != std::string::npos);
    for (const DependencyInfo& dep : kDependencies) {
        CHECK(text.find(std::string(dep.name)) != std::string::npos);
        CHECK(text.find(std::string(dep.version)) != std::string::npos);
    }
}
