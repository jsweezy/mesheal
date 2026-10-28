#include "app/RecentFiles.hpp"

#include "io/ConfigDirectory.hpp"

#include <algorithm>
#include <fstream>

namespace ogun {

std::filesystem::path recentFilesPath(const std::filesystem::path& configDirectory) {
    return configDirectory.empty() ? std::filesystem::path{} : configDirectory / kRecentFilesFileName;
}

void RecentFiles::load() {
    entries_.clear();
    if (file_.empty()) {
        return;
    }
    std::ifstream in(file_);
    std::string line;
    // add() puts each path first, so read oldest-first to end up with the file's order
    std::vector<std::string> lines;
    while (std::getline(in, line)) {
        if (!line.empty() && line.back() == '\r') {
            line.pop_back();
        }
        if (!line.empty() && std::filesystem::path(line).is_absolute()) {
            lines.push_back(line);
        }
    }
    for (auto it = lines.rbegin(); it != lines.rend(); ++it) {
        add(*it);
    }
}

bool RecentFiles::save(std::string* error) const {
    if (file_.empty()) {
        return true;
    }
    std::string text;
    for (const auto& p : entries_) {
        text += p.string();
        text += '\n';
    }
    return writeFileAtomically(file_, text, error);
}

void RecentFiles::add(const std::filesystem::path& path) {
    if (path.empty()) {
        return;
    }
    std::error_code ec;
    std::filesystem::path full = std::filesystem::absolute(path, ec);
    if (ec) {
        return;
    }
    full = full.lexically_normal();
    std::erase(entries_, full);
    entries_.insert(entries_.begin(), std::move(full));
    if (entries_.size() > kMaxRecentFiles) {
        entries_.resize(kMaxRecentFiles);
    }
}

} // namespace ogun
