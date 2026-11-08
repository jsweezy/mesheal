#pragma once

#include <atomic>
#include <cstdint>

namespace mesheal {

/// A number never handed out before in this process. A state stamped with it on every change can be
/// told apart from every earlier state by comparing stamps, and a copy keeps its stamp (it is the same
/// state), so caches can key on it.
[[nodiscard]] inline std::uint64_t nextRevision() noexcept {
    static std::atomic<std::uint64_t> counter{0};
    return counter.fetch_add(1, std::memory_order_relaxed) + 1;
}

} // namespace mesheal
