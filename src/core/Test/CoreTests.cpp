#include "core/Constants.hpp"
#include "core/Settings.hpp"
#include "mesheal/Version.hpp"

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <cctype>
#include <cmath>
#include <numbers>
#include <string>

// Static assertions ensuring compile-time mathematical relationships
static_assert(mesheal::kPi == std::numbers::pi);
static_assert(mesheal::kAlphaX == 1.0);
static_assert(mesheal::kAlphaY > 0.618 && mesheal::kAlphaY < 0.619);
static_assert(mesheal::kAlphaZ > 0.414 && mesheal::kAlphaZ < 0.415);
static_assert(mesheal::kDegToRad * 180.0 == mesheal::kPi);
static_assert(mesheal::kRadToDeg * mesheal::kPi == 180.0);

TEST_CASE("Core mathematical constants compute expected values", "[core][constants]") {
    SECTION("Pi and radian conversion factors") {
        REQUIRE(mesheal::kPi == Catch::Approx(3.141592653589793));
        REQUIRE(180.0 * mesheal::kDegToRad == Catch::Approx(mesheal::kPi));
        REQUIRE(mesheal::kPi * mesheal::kRadToDeg == Catch::Approx(180.0));
    }

    SECTION("Projection weights match golden ratio reciprocal and sqrt(2) - 1") {
        const double expectedPhi = (1.0 + std::sqrt(5.0)) * 0.5;
        REQUIRE(mesheal::kAlphaY == Catch::Approx(1.0 / expectedPhi));
        REQUIRE(mesheal::kAlphaZ == Catch::Approx(std::sqrt(2.0) - 1.0));
    }

    SECTION("Normal ratio tolerance") {
        REQUIRE(mesheal::kReliableNormalRatio == Catch::Approx(1.0e-10));
    }
}

TEST_CASE("Core default settings provide expected baseline values", "[core][settings]") {
    SECTION("Highlight defaults") {
        REQUIRE(mesheal::settings::kDefaultHighlightPointRadiusPx == Catch::Approx(3.0));
        REQUIRE(mesheal::settings::kMinHighlightPointRadiusPx < mesheal::settings::kMaxHighlightPointRadiusPx);
        REQUIRE(mesheal::settings::kDefaultOverlayEdgeWidthPx == Catch::Approx(1.0));
        REQUIRE(mesheal::settings::kMinOverlayEdgeWidthPx < mesheal::settings::kDefaultOverlayEdgeWidthPx);
        REQUIRE(mesheal::settings::kDefaultOverlayEdgeWidthPx < mesheal::settings::kMaxOverlayEdgeWidthPx);
    }

    SECTION("Camera defaults") {
        REQUIRE(mesheal::settings::kDefaultCameraFovDegrees == Catch::Approx(45.0));
        REQUIRE(mesheal::settings::kMinCameraFovDegrees < mesheal::settings::kMaxCameraFovDegrees);
    }

    SECTION("Diagnostic and repair defaults") {
        // Thin faces are judged by aspect ratio; the absolute floors are off by default
        REQUIRE(mesheal::settings::kDefaultThinFaceMaxAspectRatio == 100.0);
        REQUIRE(mesheal::settings::kDefaultThinFaceMinAltitude == 0.0);
        REQUIRE(mesheal::settings::kDefaultThinFaceMinArea == 0.0);
        REQUIRE(mesheal::settings::kNormalFloorCoordinateFactor == std::ldexp(1.0, -40));
        REQUIRE(mesheal::settings::kDefaultCapAngleDeg == Catch::Approx(170.0));
        REQUIRE(mesheal::settings::kDefaultNeedleEdgeRatio == Catch::Approx(0.05));
    }
}

TEST_CASE("Generated version and build info are sane", "[core][version]") {
    REQUIRE(mesheal::kVersion >= 1);
    REQUIRE(mesheal::kVersionString == std::to_string(mesheal::kVersion));

    // ISO date YYYY-MM-DD
    const std::string date(mesheal::kVersionDate);
    REQUIRE(date.size() == 10);
    for (std::size_t i = 0; i < date.size(); ++i) {
        if (i == 4 || i == 7) {
            REQUIRE(date[i] == '-');
        } else {
            REQUIRE(std::isdigit(static_cast<unsigned char>(date[i])));
        }
    }

    REQUIRE_FALSE(mesheal::kCompilerId.empty());
    REQUIRE_FALSE(mesheal::kCompilerVersion.empty());
    REQUIRE_FALSE(mesheal::kBuildTimestamp.empty());
    REQUIRE_FALSE(mesheal::kPlatform.empty());
    REQUIRE(mesheal::kCxxStandard.starts_with("C++"));
    REQUIRE_FALSE(mesheal::kDependencies.empty());
    for (const auto& dep : mesheal::kDependencies) {
        REQUIRE_FALSE(dep.name.empty());
        REQUIRE_FALSE(dep.version.empty());
    }
}
