#include "app/FileOpenQueue.hpp"

namespace ogun {

FileOpenQueue& FileOpenQueue::instance() {
    static FileOpenQueue sInstance;
    return sInstance;
}

void FileOpenQueue::setWakeCallback(std::function<void()> callback) {
    std::lock_guard<std::mutex> lock(mutex_);
    wakeCallback_ = std::move(callback);
}

void FileOpenQueue::postFile(const std::filesystem::path& path) {
    if (path.empty()) {
        return;
    }
    std::function<void()> wake;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        queue_.push(path);
        wake = wakeCallback_;
    }
    if (wake) {
        wake();
    }
}

std::optional<std::filesystem::path> FileOpenQueue::popFile() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (queue_.empty()) {
        return std::nullopt;
    }
    std::filesystem::path path = std::move(queue_.front());
    queue_.pop();
    return path;
}

bool FileOpenQueue::hasPendingFiles() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return !queue_.empty();
}

void FileOpenQueue::clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    std::queue<std::filesystem::path> empty;
    std::swap(queue_, empty);
}

} // namespace ogun
