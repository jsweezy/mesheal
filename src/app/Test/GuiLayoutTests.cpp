#include "app/GuiLayout.hpp"

#include <catch2/catch_test_macros.hpp>

#include <cstddef>
#include <vector>

namespace {

using ogun::LayoutRect;

bool overlap(const LayoutRect& a, const LayoutRect& b) {
    return a.x < b.right() && b.x < a.right() && a.y < b.bottom() && b.y < a.bottom();
}

bool inside(const LayoutRect& r, const LayoutRect& area) {
    return r.x >= area.x && r.y >= area.y && r.right() <= area.right() + 1.0e-3f &&
           r.bottom() <= area.bottom() + 1.0e-3f && r.width >= 0.0f && r.height >= 0.0f;
}

void checkTiles(const std::vector<LayoutRect>& tiles, const LayoutRect& area, std::size_t count) {
    REQUIRE(tiles.size() == count);
    for (std::size_t i = 0; i < tiles.size(); ++i) {
        INFO("tile " << i);
        CHECK(inside(tiles[i], area));
        for (std::size_t j = i + 1; j < tiles.size(); ++j) {
            CHECK_FALSE(overlap(tiles[i], tiles[j]));
        }
    }
}

} // namespace

TEST_CASE("Floating windows tile the free 3D view without overlapping", "[app][layout]") {
    constexpr float kGap = 6.0f;

    SECTION("A wide view: one row of preferred-width tiles from the top-left corner") {
        const LayoutRect area{302.0f, 30.0f, 1800.0f, 1200.0f};
        const auto tiles = ogun::tileWindows(area, 4, 330.0f, 240.0f, kGap);
        checkTiles(tiles, area, 4);
        CHECK(tiles[0].x == area.x);
        CHECK(tiles[0].y == area.y);
        for (std::size_t i = 0; i < tiles.size(); ++i) {
            CHECK(tiles[i].width == 330.0f);
            CHECK(tiles[i].y == area.y);
            CHECK(tiles[i].height == area.height);
        }
        CHECK(tiles[1].x == tiles[0].right() + kGap);
    }

    SECTION("A narrower view shrinks the tiles, then wraps them into rows") {
        const LayoutRect medium{0.0f, 0.0f, 1000.0f, 800.0f};
        const auto shrunk = ogun::tileWindows(medium, 4, 330.0f, 240.0f, kGap);
        checkTiles(shrunk, medium, 4);
        CHECK(shrunk[3].y == 0.0f); // still one row
        CHECK(shrunk[0].width < 330.0f);
        CHECK(shrunk[0].width >= 240.0f);

        const LayoutRect narrow{0.0f, 0.0f, 600.0f, 800.0f};
        const auto wrapped = ogun::tileWindows(narrow, 4, 330.0f, 240.0f, kGap);
        checkTiles(wrapped, narrow, 4);
        CHECK(wrapped[2].y > wrapped[0].y); // a 2 x 2 grid
        CHECK(wrapped[2].x == wrapped[0].x);
    }

    SECTION("A tiny or empty view still gives one stacked, clamped tile per window") {
        const LayoutRect tiny{10.0f, 10.0f, 100.0f, 90.0f};
        checkTiles(ogun::tileWindows(tiny, 4, 330.0f, 240.0f, kGap), tiny, 4);
        const LayoutRect none{10.0f, 10.0f, 0.0f, 0.0f};
        const auto collapsed = ogun::tileWindows(none, 4, 330.0f, 240.0f, kGap);
        REQUIRE(collapsed.size() == 4);
        for (const LayoutRect& r : collapsed) {
            CHECK(r.width >= 0.0f);
            CHECK(r.height >= 0.0f);
        }
        CHECK(ogun::tileWindows(tiny, 0, 330.0f, 240.0f, kGap).empty());
    }
}
