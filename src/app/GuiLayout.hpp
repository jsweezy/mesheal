#pragma once

#include <cstddef>
#include <functional>
#include <string>
#include <string_view>
#include <vector>

namespace mesheal {

/// A screen rectangle in ImGui units (top-left corner and size).
struct LayoutRect {
    float x = 0.0f;
    float y = 0.0f;
    float width = 0.0f;
    float height = 0.0f;

    [[nodiscard]] float right() const noexcept { return x + width; }
    [[nodiscard]] float bottom() const noexcept { return y + height; }
};

/// Non-overlapping slots for `count` floating windows inside `area`, from its top-left corner: one row
/// of tiles up to `preferredWidth` wide while each can be at least `minWidth`, otherwise more rows
/// sharing the height. Tiles are `gap` apart and stay inside `area` (when it has room for the gaps).
/// `preferredHeights` (one per window, or empty) sizes each row to its tallest window's content; the rows
/// shrink in proportion only when they would not all fit. Empty: the rows share the height equally.
[[nodiscard]] std::vector<LayoutRect> tileWindows(const LayoutRect& area, std::size_t count, float preferredWidth,
                                                  float minWidth, float gap,
                                                  const std::vector<float>& preferredHeights = {});

/// The free 3D view between the docked columns: from the left column's right margin to the right column's
/// left margin, and from `top` to `bottom`. Never negative.
[[nodiscard]] LayoutRect freeViewArea(float leftColumnWidth, float rightColumnLeft, float top, float bottom,
                                      float margin);

/// Greedy word wrap of `text` into lines no wider than `maxWidth` by `measure` (a word wider than that gets
/// its own line). Used for the empty-state hint so it stays between the columns.
[[nodiscard]] std::vector<std::string> wrapText(std::string_view text, float maxWidth,
                                                const std::function<float(const std::string&)>& measure);

} // namespace mesheal
