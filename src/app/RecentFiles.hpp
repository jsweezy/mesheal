#pragma once

#include <cstddef>
#include <utility>
#include <filesystem>
#include <string>
#include <vector>

namespace mesheal {

/// File in `configDirectory()` holding the recent STL list.
inline constexpr const char* kRecentFilesFileName = "recent_files.txt";

/// Most STL files the Open Recent list keeps.
inline constexpr std::size_t kMaxRecentFiles = 10;

/// `<configDirectory>/recent_files.txt`; empty when `configDirectory` is empty.
[[nodiscard]] std::filesystem::path recentFilesPath(const std::filesystem::path& configDirectory);

/// The last STL files opened (File > Open Recent), most recent first, stored as absolute normalized
/// paths so a file is found again whatever directory Mesheal is launched from. The backing file is plain
/// text, one path per line; an empty file path keeps the list in memory only.
class RecentFiles {
public:
    explicit RecentFiles(std::filesystem::path file = {}) : file_(std::move(file)) {}

    /// Replaces the list with the backing file's lines. A missing or unreadable file gives an empty
    /// list; blank and non-absolute lines are skipped, duplicates collapse and the cap applies.
    void load();

    /// Writes the list atomically. Returns false (and sets `error`) on failure; true if there is no file.
    bool save(std::string* error = nullptr) const;

    /// Puts `path` (made absolute and normalized) first, dropping any older copy and the oldest entries over the cap.
    void add(const std::filesystem::path& path);

    void clear() noexcept { entries_.clear(); }

    [[nodiscard]] const std::vector<std::filesystem::path>& entries() const noexcept { return entries_; }
    [[nodiscard]] bool empty() const noexcept { return entries_.empty(); }

private:
    std::filesystem::path file_;
    std::vector<std::filesystem::path> entries_;
};

} // namespace mesheal
