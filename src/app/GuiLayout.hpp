#pragma once

#include <cstddef>
#include <vector>

namespace ogun {

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

} // namespace ogun
