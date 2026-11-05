#include "app/MacFileOpenHandler.h"
#include "app/FileOpenQueue.hpp"

#ifdef __APPLE__
#import <Cocoa/Cocoa.h>
#import <objc/runtime.h>
#endif

#include <chrono>
#include <iostream>

#ifdef __APPLE__

namespace {

std::function<bool()> gIsBusyPredicate;
bool gInitialLaunchComplete = false;
bool gDisableRelaunch = false;
std::filesystem::path gCurrentLoadedPath;
std::filesystem::path gLastSpawnedPath;
std::chrono::steady_clock::time_point gLastSpawnedTime{};

NSString* normalizeToFilesystemPath(NSString* input) {
    if (input == nil || [input length] == 0) {
        return nil;
    }
    if ([input hasPrefix:@"file://"]) {
        NSURL* url = [NSURL URLWithString:input];
        if (url != nil) {
            return [url path];
        }
    }
    return input;
}

void handleResolvedPath(NSString* rawPath) {
    if (rawPath == nil) {
        return;
    }
    NSString* cleanPath = normalizeToFilesystemPath(rawPath);
    if (cleanPath == nil || [cleanPath length] == 0) {
        return;
    }
    const std::filesystem::path stlPath([cleanPath UTF8String]);

    std::error_code ec;
    // 1. Ignore if already loaded in this window
    if (!gCurrentLoadedPath.empty()) {
        if (stlPath == gCurrentLoadedPath || std::filesystem::equivalent(stlPath, gCurrentLoadedPath, ec)) {
            return;
        }
    }

    // 2. During cold start, this is the launch document. Always load directly into this instance.
    if (!gInitialLaunchComplete || gDisableRelaunch) {
        gCurrentLoadedPath = stlPath;
        ogun::FileOpenQueue::instance().postFile(stlPath);
        return;
    }

    // 3. Debounce rapid duplicate events for the same path
    const auto now = std::chrono::steady_clock::now();
    if (stlPath == gLastSpawnedPath && (now - gLastSpawnedTime) < std::chrono::seconds(3)) {
        return;
    }

    // 4. If this instance already has a mesh loaded, spawn a separate instance for the new file
    const bool isBusy = gIsBusyPredicate ? gIsBusyPredicate() : false;
    if (isBusy) {
        gLastSpawnedPath = stlPath;
        gLastSpawnedTime = now;
        ogun::launchNewMacAppInstance(stlPath);
    } else {
        gCurrentLoadedPath = stlPath;
        ogun::FileOpenQueue::instance().postFile(stlPath);
    }
}

BOOL OgunOpenFile(id self, SEL _cmd, NSApplication* sender, NSString* filename) {
    (void)self;
    (void)_cmd;
    (void)sender;
    handleResolvedPath(filename);
    return YES;
}

void OgunOpenFiles(id self, SEL _cmd, NSApplication* sender, NSArray<NSString*>* filenames) {
    (void)self;
    (void)_cmd;
    for (NSString* file in filenames) {
        handleResolvedPath(file);
    }
    [sender replyToOpenOrPrint:NSApplicationDelegateReplySuccess];
}

void OgunOpenURLs(id self, SEL _cmd, NSApplication* application, NSArray<NSURL*>* urls) {
    (void)self;
    (void)_cmd;
    (void)application;
    for (NSURL* url in urls) {
        if ([url isFileURL]) {
            handleResolvedPath([url path]);
        }
    }
}

void injectOpenFileMethodsIntoClass(Class cls) {
    if (cls == nil) {
        return;
    }
    if (!class_respondsToSelector(cls, @selector(application:openFile:))) {
        class_addMethod(cls, @selector(application:openFile:), (IMP)OgunOpenFile, "c@:@@");
    }
    if (!class_respondsToSelector(cls, @selector(application:openFiles:))) {
        class_addMethod(cls, @selector(application:openFiles:), (IMP)OgunOpenFiles, "v@:@@");
    }
    if (!class_respondsToSelector(cls, @selector(application:openURLs:))) {
        class_addMethod(cls, @selector(application:openURLs:), (IMP)OgunOpenURLs, "v@:@@");
    }
}

} // namespace

@interface OgunAppleEventHandler : NSObject
- (void)handleOpenDocumentsEvent:(NSAppleEventDescriptor*)event
                  withReplyEvent:(NSAppleEventDescriptor*)reply;
@end

@implementation OgunAppleEventHandler

- (void)handleOpenDocumentsEvent:(NSAppleEventDescriptor*)event
                  withReplyEvent:(NSAppleEventDescriptor*)reply {
    (void)reply;
    NSAppleEventDescriptor* docList = [event descriptorForKeyword:keyDirectObject];
    if (docList == nil) {
        return;
    }

    const NSInteger count = [docList numberOfItems];
    if (count > 0) {
        for (NSInteger i = 1; i <= count; ++i) {
            NSAppleEventDescriptor* item = [docList descriptorAtIndex:i];
            NSString* path = [item stringValue];
            if (path == nil) {
                NSAppleEventDescriptor* furl = [item coerceToDescriptorType:typeFileURL];
                if (furl != nil && [furl data] != nil) {
                    path = [[NSString alloc] initWithData:[furl data] encoding:NSUTF8StringEncoding];
                }
            }
            handleResolvedPath(path);
        }
    } else {
        NSString* path = [docList stringValue];
        if (path == nil) {
            NSAppleEventDescriptor* furl = [docList coerceToDescriptorType:typeFileURL];
            if (furl != nil && [furl data] != nil) {
                path = [[NSString alloc] initWithData:[furl data] encoding:NSUTF8StringEncoding];
            }
        }
        handleResolvedPath(path);
    }
}

@end

namespace {
static OgunAppleEventHandler* gAppleEventHandler = nil;
} // namespace

#endif

namespace ogun {

#ifdef __APPLE__

void installMacFileOpenHandler(std::function<bool()> isBusyPredicate) {
    @autoreleasepool {
        gIsBusyPredicate = std::move(isBusyPredicate);

        if (NSApp != nil && [NSApp delegate] != nil) {
            injectOpenFileMethodsIntoClass([[NSApp delegate] class]);
        }
        Class glfwDelegateClass = objc_getClass("GLFWApplicationDelegate");
        if (glfwDelegateClass != nil) {
            injectOpenFileMethodsIntoClass(glfwDelegateClass);
        }

        if (gAppleEventHandler == nil) {
            gAppleEventHandler = [[OgunAppleEventHandler alloc] init];
        }
        [[NSAppleEventManager sharedAppleEventManager]
            setEventHandler:gAppleEventHandler
                andSelector:@selector(handleOpenDocumentsEvent:withReplyEvent:)
              forEventClass:kCoreEventClass
                 andEventID:kAEOpenDocuments];
    }
}

void disableMacRelaunch() {
    gDisableRelaunch = true;
}

void markMacInitialLaunchComplete() {
    gInitialLaunchComplete = true;
}

void setMacCurrentLoadedPath(const std::filesystem::path& path) {
    gCurrentLoadedPath = path;
}

bool launchNewMacAppInstance(const std::filesystem::path& stlPath) {
    @autoreleasepool {
        NSURL* bundleUrl = [[NSBundle mainBundle] bundleURL];
        NSString* pathStr = [NSString stringWithUTF8String:stlPath.string().c_str()];
        if (bundleUrl != nil && [[bundleUrl path] hasSuffix:@".app"]) {
            if (@available(macOS 10.15, *)) {
                NSWorkspaceOpenConfiguration* config = [NSWorkspaceOpenConfiguration configuration];
                config.createsNewApplicationInstance = YES;
                if (!stlPath.empty()) {
                    config.arguments = @[pathStr, @"--no-relaunch"];
                } else {
                    config.arguments = @[@"--no-relaunch"];
                }
                [[NSWorkspace sharedWorkspace] openApplicationAtURL:bundleUrl
                                                      configuration:config
                                                  completionHandler:nil];
                return true;
            }
        }

        // Fallback: spawn via /usr/bin/open -n
        NSTask* task = [[NSTask alloc] init];
        [task setLaunchPath:@"/usr/bin/open"];
        NSMutableArray<NSString*>* args = [NSMutableArray arrayWithObject:@"-n"];
        if (bundleUrl != nil && [[bundleUrl path] hasSuffix:@".app"]) {
            [args addObject:@"-a"];
            [args addObject:[bundleUrl path]];
        } else {
            NSString* execPath = [[NSBundle mainBundle] executablePath];
            if (execPath != nil) {
                [args addObject:execPath];
            }
        }
        if (!stlPath.empty()) {
            [args addObject:@"--args"];
            [args addObject:pathStr];
            [args addObject:@"--no-relaunch"];
        } else {
            [args addObject:@"--args"];
            [args addObject:@"--no-relaunch"];
        }
        [task setArguments:args];
        @try {
            [task launch];
            return true;
        } @catch (NSException* e) {
            std::cerr << "Warning: failed to spawn new Ogun instance: "
                      << [[e reason] UTF8String] << "\n";
            return false;
        }
    }
}

#else

void installMacFileOpenHandler(std::function<bool()> isBusyPredicate) {
    (void)isBusyPredicate;
}

void disableMacRelaunch() {}

void markMacInitialLaunchComplete() {}

void setMacCurrentLoadedPath(const std::filesystem::path& path) {
    (void)path;
}

bool launchNewMacAppInstance(const std::filesystem::path& stlPath) {
    (void)stlPath;
    return false;
}

#endif

} // namespace ogun
