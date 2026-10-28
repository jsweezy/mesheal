#include "core/Constants.hpp"
#include "core/Settings.hpp"
#include "ogun/Version.hpp"

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <cctype>
#include <cmath>
#include <numbers>
#include <string>

// Static assertions ensuring compile-time mathematical relationships
static_assert(ogun::kPi == std::numbers::pi);
static_assert(ogun::kAlphaX == 1.0);
static_assert(ogun::kAlphaY > 0.618 && ogun::kAlphaY < 0.619);
static_assert(ogun::kAlphaZ > 0.414 && ogun::kAlphaZ < 0.415);
static_assert(ogun::kDegToRad * 180.0 == ogun::kPi);
static_assert(ogun::kRadToDeg * ogun::kPi == 180.0);

TEST_CASE("Core mathematical constants compute expected values", "[core][constants]") {
    SECTION("Pi and radian conversion factors") {
        REQUIRE(ogun::kPi == Catch::Approx(3.141592653589793));
        REQUIRE(180.0 * ogun::kDegToRad == Catch::Approx(ogun::kPi));
        REQUIRE(ogun::kPi * ogun::kRadToDeg == Catch::Approx(180.0));
    }

    SECTION("Projection weights match golden ratio reciprocal and sqrt(2) - 1") {
        const double expectedPhi = (1.0 + std::sqrt(5.0)) * 0.5;
        REQUIRE(ogun::kAlphaY == Catch::Approx(1.0 / expectedPhi));
        REQUIRE(ogun::kAlphaZ == Catch::Approx(std::sqrt(2.0) - 1.0));
    }

    SECTION("Normal ratio tolerance") {
        REQUIRE(ogun::kReliableNormalRatio == Catch::Approx(1.0e-10));
    }
}

TEST_CASE("Core default settings provide expected baseline values", "[core][settings]") {
    SECTION("Highlight defaults") {
        REQUIRE(ogun::settings::kDefaultHighlightPointRadiusPx == Catch::Approx(3.0));
        REQUIRE(ogun::settings::kMinHighlightPointRadiusPx < ogun::settings::kMaxHighlightPointRadiusPx);
        REQUIRE(ogun::settings::kDefaultOverlayEdgeWidthPx == Catch::Approx(1.0));
        REQUIRE(ogun::settings::kMinOverlayEdgeWidthPx < ogun::settings::kDefaultOverlayEdgeWidthPx);
        REQUIRE(ogun::settings::kDefaultOverlayEdgeWidthPx < ogun::settings::kMaxOverlayEdgeWidthPx);
    }

    SECTION("Camera defaults") {
        REQUIRE(ogun::settings::kDefaultCameraFovDegrees == Catch::Approx(45.0));
        REQUIRE(ogun::settings::kMinCameraFovDegrees < ogun::settings::kMaxCameraFovDegrees);
    }

    SECTION("Diagnostic and repair defaults") {
        REQUIRE(ogun::settings::kDefaultThinFaceMinAltitude == Catch::Approx(1.0e-4));
        REQUIRE(ogun::settings::kDefaultThinFaceMinArea == Catch::Approx(1.0e-8));
        REQUIRE(ogun::settings::kDefaultThinFaceMaxAspectRatio == Catch::Approx(1000.0));
        REQUIRE(ogun::settings::kDefaultCapAngleDeg == Catch::Approx(170.0));
        REQUIRE(ogun::settings::kDefaultNeedleEdgeRatio == Catch::Approx(0.05));
    }
}

TEST_CASE("Generated version and build info are sane", "[core][version]") {
    REQUIRE(ogun::kVersion >= 1);
    REQUIRE(ogun::kVersionString == std::to_string(ogun::kVersion));

    // ISO date YYYY-MM-DD
    const std::string date(ogun::kVersionDate);
    REQUIRE(date.size() == 10);
    for (std::size_t i = 0; i < date.size(); ++i) {
        if (i == 4 || i == 7) {
            REQUIRE(date[i] == '-');
        } else {
            REQUIRE(std::isdigit(static_cast<unsigned char>(date[i])));
        }
    }

    REQUIRE_FALSE(ogun::kCompilerId.empty());
    REQUIRE_FALSE(ogun::kCompilerVersion.empty());
    REQUIRE_FALSE(ogun::kBuildTimestamp.empty());
    REQUIRE_FALSE(ogun::kPlatform.empty());
    REQUIRE(ogun::kCxxStandard.starts_with("C++"));
    REQUIRE_FALSE(ogun::kDependencies.empty());
    for (const auto& dep : ogun::kDependencies) {
        REQUIRE_FALSE(dep.name.empty());
        REQUIRE_FALSE(dep.version.empty());
    }
}
