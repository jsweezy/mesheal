#pragma once

namespace mesheal {

/// Sets the macOS Dock / Command-Tab application icon from an 8-bit RGBA image buffer.
bool setMacApplicationIconFromRgba(const unsigned char* rgbaPixels, int width, int height);

} // namespace mesheal
