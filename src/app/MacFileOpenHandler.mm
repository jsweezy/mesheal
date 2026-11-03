#include "app/MacFileOpenHandler.h"
#include "app/FileOpenQueue.hpp"

#ifdef __APPLE__
#import <Cocoa/Cocoa.h>
#endif

#include <iostream>

#ifdef __APPLE__

namespace {
std::function<bool()> gIsBusyPredicate;
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
    auto handlePath = [](NSString* nsPath) {
        if (nsPath == nil || [nsPath length] == 0) {
            return;
        }
        const std::filesystem::path stlPath([nsPath UTF8String]);
        const bool isBusy = gIsBusyPredicate ? gIsBusyPredicate() : false;
        if (isBusy) {
            ogun::launchNewMacAppInstance(stlPath);
        } else {
            ogun::FileOpenQueue::instance().postFile(stlPath);
        }
    };

    if (count > 0) {
        for (NSInteger i = 1; i <= count; ++i) {
            NSAppleEventDescriptor* item = [docList descriptorAtIndex:i];
            NSString* path = nil;
            if ([item stringValue] != nil) {
                path = [item stringValue];
            } else if ([item data] != nil) {
                NSURL* url = [NSURL URLWithString:[[NSString alloc] initWithData:[item data] encoding:NSUTF8StringEncoding]];
                if (url != nil && [url isFileURL]) {
                    path = [url path];
                }
            }
            handlePath(path);
        }
    } else {
        NSString* path = [docList stringValue];
        if (path == nil && [docList data] != nil) {
            NSURL* url = [NSURL URLWithString:[[NSString alloc] initWithData:[docList data] encoding:NSUTF8StringEncoding]];
            if (url != nil && [url isFileURL]) {
                path = [url path];
            }
        }
        handlePath(path);
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
        if (gAppleEventHandler == nil) {
            gAppleEventHandler = [[OgunAppleEventHandler alloc] init];
            [[NSAppleEventManager sharedAppleEventManager]
                setEventHandler:gAppleEventHandler
                    andSelector:@selector(handleOpenDocumentsEvent:withReplyEvent:)
                  forEventClass:kCoreEventClass
                     andEventID:kAEOpenDocuments];
        }
    }
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
                    config.arguments = @[pathStr];
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

bool launchNewMacAppInstance(const std::filesystem::path& stlPath) {
    (void)stlPath;
    return false;
}

#endif

} // namespace ogun
