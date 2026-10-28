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

    SECTION("Preferred heights size each row to its content and shrink together when short of room") {
        const std::vector<float> heights{200.0f, 400.0f, 500.0f, 150.0f};
        const LayoutRect tall{0.0f, 0.0f, 1800.0f, 1000.0f};
        const auto one = ogun::tileWindows(tall, 4, 330.0f, 240.0f, kGap, heights);
        checkTiles(one, tall, 4);
        for (const LayoutRect& r : one) {
            CHECK(r.height == 500.0f); // one row, as tall as the tallest window
        }

        // One column: stacked, each slot as tall as asked while there is room
        const LayoutRect column{0.0f, 0.0f, 250.0f, 1400.0f};
        const auto stacked = ogun::tileWindows(column, 4, 330.0f, 240.0f, kGap, heights);
        checkTiles(stacked, column, 4);
        for (std::size_t i = 0; i < 4; ++i) {
            CHECK(stacked[i].height == heights[i]);
        }

        // Too short: every row shrinks in proportion, still inside and apart
        const LayoutRect shortArea{0.0f, 0.0f, 250.0f, 600.0f};
        const auto squeezed = ogun::tileWindows(shortArea, 4, 330.0f, 240.0f, kGap, heights);
        checkTiles(squeezed, shortArea, 4);
        CHECK(squeezed[1].height < heights[1]);
        CHECK(squeezed[2].height > squeezed[0].height);
    }
}

TEST_CASE("The empty-state hint wraps into the free view between the columns", "[app][layout]") {
    const auto measure = [](const std::string& s) { return 10.0f * static_cast<float>(s.size()); };
    const std::string text = "No mesh loaded. Open an STL (File > Open) or load a sample (File > Load Sample).";

    const LayoutRect free = ogun::freeViewArea(300.0f, 1000.0f, 30.0f, 800.0f, 6.0f);
    CHECK(free.x == 312.0f);
    CHECK(free.right() == 994.0f);
    CHECK(free.y == 30.0f);
    CHECK(free.height == 770.0f);
    CHECK(ogun::freeViewArea(300.0f, 100.0f, 30.0f, 20.0f, 6.0f).width == 0.0f);

    SECTION("A wide gap keeps one line") {
        CHECK(ogun::wrapText(text, 2000.0f, measure).size() == 1);
    }
    SECTION("A narrow gap wraps every line inside it and keeps all the words") {
        const auto lines = ogun::wrapText(text, 400.0f, measure);
        CHECK(lines.size() > 1);
        std::string joined;
        for (const std::string& line : lines) {
            CHECK(measure(line) <= 400.0f);
            joined += (joined.empty() ? "" : " ") + line;
        }
        CHECK(joined == text);
    }
    SECTION("A word wider than the gap gets its own line") {
        const auto lines = ogun::wrapText("ab supercalifragilistic cd", 100.0f, measure);
        REQUIRE(lines.size() == 3);
        CHECK(lines[1] == "supercalifragilistic");
    }
    CHECK(ogun::wrapText("", 100.0f, measure).empty());
}
