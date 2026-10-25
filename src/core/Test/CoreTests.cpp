#include "core/Constants.hpp"
#include "core/Settings.hpp"

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <cmath>
#include <numbers>

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
