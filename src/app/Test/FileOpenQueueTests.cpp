#include "app/FileOpenQueue.hpp"

#include <catch2/catch_test_macros.hpp>

#include <filesystem>
#include <thread>
#include <vector>

TEST_CASE("FileOpenQueue enqueues, dequeues and clears paths", "[app][file_queue]") {
    ogun::FileOpenQueue& queue = ogun::FileOpenQueue::instance();
    queue.clear();

    REQUIRE_FALSE(queue.hasPendingFiles());
    REQUIRE_FALSE(queue.popFile().has_value());

    SECTION("Ignore empty paths") {
        queue.postFile("");
        REQUIRE_FALSE(queue.hasPendingFiles());
    }

    SECTION("Enqueue and pop in FIFO order") {
        queue.postFile("/path/to/first.stl");
        queue.postFile("/path/to/second.stl");

        REQUIRE(queue.hasPendingFiles());

        const auto first = queue.popFile();
        REQUIRE(first.has_value());
        REQUIRE(*first == std::filesystem::path("/path/to/first.stl"));

        REQUIRE(queue.hasPendingFiles());

        const auto second = queue.popFile();
        REQUIRE(second.has_value());
        REQUIRE(*second == std::filesystem::path("/path/to/second.stl"));

        REQUIRE_FALSE(queue.hasPendingFiles());
        REQUIRE_FALSE(queue.popFile().has_value());
    }

    SECTION("Clear queue") {
        queue.postFile("/path/to/model.stl");
        REQUIRE(queue.hasPendingFiles());
        queue.clear();
        REQUIRE_FALSE(queue.hasPendingFiles());
        REQUIRE_FALSE(queue.popFile().has_value());
    }

    SECTION("Concurrent posts and pops") {
        constexpr int kItemsPerThread = 50;
        std::vector<std::thread> producers;
        producers.reserve(4);

        for (int t = 0; t < 4; ++t) {
            producers.emplace_back([&queue, t]() {
                for (int i = 0; i < kItemsPerThread; ++i) {
                    queue.postFile("/dummy/model_" + std::to_string(t) + "_" + std::to_string(i) + ".stl");
                }
            });
        }

        for (auto& th : producers) {
            th.join();
        }

        int poppedCount = 0;
        while (queue.popFile().has_value()) {
            ++poppedCount;
        }

        REQUIRE(poppedCount == 4 * kItemsPerThread);
        REQUIRE_FALSE(queue.hasPendingFiles());
    }
}
