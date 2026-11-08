#include "app/GuiLayout.hpp"

#include <algorithm>
#include <cmath>

namespace mesheal {

std::vector<LayoutRect> tileWindows(const LayoutRect& area, std::size_t count, float preferredWidth, float minWidth,
                                    float gap, const std::vector<float>& preferredHeights) {
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

    // Each row is as tall as its tallest window wants, shrunk together if they do not all fit
    const float available = std::max(height - gap * static_cast<float>(rows - 1), 0.0f);
    std::vector<float> rowHeights(rows, available / static_cast<float>(rows));
    if (preferredHeights.size() == count) {
        std::fill(rowHeights.begin(), rowHeights.end(), 0.0f);
        for (std::size_t i = 0; i < count; ++i) {
            rowHeights[i / columns] = std::max(rowHeights[i / columns], std::max(preferredHeights[i], 0.0f));
        }
        float wanted = 0.0f;
        for (const float h : rowHeights) {
            wanted += h;
        }
        if (wanted > available && wanted > 0.0f) {
            for (float& h : rowHeights) {
                h *= available / wanted;
            }
        }
    }
    std::vector<float> rowTops(rows, area.y);
    for (std::size_t r = 1; r < rows; ++r) {
        rowTops[r] = rowTops[r - 1] + rowHeights[r - 1] + gap;
    }
    for (std::size_t i = 0; i < count; ++i) {
        const auto column = static_cast<float>(i % columns);
        tiles.push_back(
            LayoutRect{area.x + column * (columnWidth + gap), rowTops[i / columns], columnWidth, rowHeights[i / columns]});
    }
    return tiles;
}

LayoutRect freeViewArea(float leftColumnWidth, float rightColumnLeft, float top, float bottom, float margin) {
    const float left = leftColumnWidth + 2.0f * margin;
    return LayoutRect{left, top, std::max(rightColumnLeft - margin - left, 0.0f), std::max(bottom - top, 0.0f)};
}

std::vector<std::string> wrapText(std::string_view text, float maxWidth,
                                  const std::function<float(const std::string&)>& measure) {
    std::vector<std::string> lines;
    std::string line;
    std::size_t pos = 0;
    while (pos < text.size()) {
        const std::size_t end = std::min(text.find(' ', pos), text.size());
        const std::string word(text.substr(pos, end - pos));
        pos = end + 1;
        if (word.empty()) {
            continue;
        }
        const std::string candidate = line.empty() ? word : line + " " + word;
        if (!line.empty() && measure(candidate) > maxWidth) {
            lines.push_back(line);
            line = word;
        } else {
            line = candidate;
        }
    }
    if (!line.empty()) {
        lines.push_back(line);
    }
    return lines;
}

} // namespace mesheal
