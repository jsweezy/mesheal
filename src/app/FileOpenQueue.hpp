#pragma once

#include <filesystem>
#include <functional>
#include <mutex>
#include <optional>
#include <queue>

namespace ogun {

/// Thread-safe queue for file paths requested to be opened in the application
/// (via Finder double-click, OS AppleEvents, CLI arguments, or viewport drag-and-drop).
class FileOpenQueue {
public:
    static FileOpenQueue& instance();

    /// Register an optional wake callback (e.g. glfwPostEmptyEvent) called when a file is enqueued.
    void setWakeCallback(std::function<void()> callback);

    /// Enqueue a file path to be opened by the active application loop.
    void postFile(const std::filesystem::path& path);

    /// Dequeue the next pending file path, if any.
    std::optional<std::filesystem::path> popFile();

    /// Returns true if there is at least one file pending in the queue.
    bool hasPendingFiles() const;

    /// Clears any pending files in the queue.
    void clear();

private:
    FileOpenQueue() = default;
    mutable std::mutex mutex_;
    std::queue<std::filesystem::path> queue_;
    std::function<void()> wakeCallback_;
};

} // namespace ogun
