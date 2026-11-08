#include "app/MacAppIcon.hpp"

#ifdef __APPLE__
#import <Cocoa/Cocoa.h>
#endif

#include <cstddef>

namespace mesheal {

bool setMacApplicationIconFromRgba(const unsigned char* rgbaPixels, int width, int height) {
#ifdef __APPLE__
    if (rgbaPixels == nullptr || width <= 0 || height <= 0) {
        return false;
    }

    @autoreleasepool {
        NSBitmapImageRep* rep = [[NSBitmapImageRep alloc]
            initWithBitmapDataPlanes:nullptr
                          pixelsWide:width
                          pixelsHigh:height
                       bitsPerSample:8
                     samplesPerPixel:4
                            hasAlpha:YES
                            isPlanar:NO
                      colorSpaceName:NSDeviceRGBColorSpace
                         bytesPerRow:width * 4
                        bitsPerPixel:32];
        if (rep == nil) {
            return false;
        }

        unsigned char* dst = [rep bitmapData];
        const std::size_t numPixels =
            static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
        for (std::size_t i = 0; i < numPixels; ++i) {
            const unsigned int alpha = static_cast<unsigned int>(rgbaPixels[i * 4 + 3]);
            dst[i * 4 + 0] = static_cast<unsigned char>(
                (static_cast<unsigned int>(rgbaPixels[i * 4 + 0]) * alpha + 127u) / 255u);
            dst[i * 4 + 1] = static_cast<unsigned char>(
                (static_cast<unsigned int>(rgbaPixels[i * 4 + 1]) * alpha + 127u) / 255u);
            dst[i * 4 + 2] = static_cast<unsigned char>(
                (static_cast<unsigned int>(rgbaPixels[i * 4 + 2]) * alpha + 127u) / 255u);
            dst[i * 4 + 3] = static_cast<unsigned char>(alpha);
        }

        NSImage* image = [[NSImage alloc]
            initWithSize:NSMakeSize(static_cast<CGFloat>(width), static_cast<CGFloat>(height))];
        [image addRepresentation:rep];
        [[NSApplication sharedApplication] setApplicationIconImage:image];
        return true;
    }
#else
    (void)rgbaPixels;
    (void)width;
    (void)height;
    return false;
#endif
}

} // namespace mesheal
