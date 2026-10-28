#include "app/GuiLayout.hpp"

#include <algorithm>
#include <cmath>

namespace ogun {

std::vector<LayoutRect> tileWindows(const LayoutRect& area, std::size_t count, float preferredWidth, float minWidth,
                                    float gap) {
    std::vector<LayoutRect> tiles;
    if (count == 0) {
        return tiles;
    }
    tiles.reserve(count);
    const float width = std::max(area.width, 0.0f);
    const float height = std::max(area.height, 0.0f);
    gap = std::max(gap, 0.0f);

    // As many columns as fit at the minimum width (at least one, at most one per window)
    const auto fit = static_cast<std::size_t>(std::floor((width + gap) / (std::max(minWidth, 1.0f) + gap)));
    const std::size_t columns = std::clamp<std::size_t>(fit, 1, count);
    const std::size_t rows = (count + columns - 1) / columns;

    const float columnWidth = std::max(
        std::min(preferredWidth, (width - gap * static_cast<float>(columns - 1)) / static_cast<float>(columns)), 0.0f);
    const float rowHeight = std::max((height - gap * static_cast<float>(rows - 1)) / static_cast<float>(rows), 0.0f);
    for (std::size_t i = 0; i < count; ++i) {
        const auto column = static_cast<float>(i % columns);
        const auto row = static_cast<float>(i / columns);
        tiles.push_back(LayoutRect{area.x + column * (columnWidth + gap), area.y + row * (rowHeight + gap), columnWidth,
                                   rowHeight});
    }
    return tiles;
}

} // namespace ogun
