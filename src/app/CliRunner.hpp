#pragma once

#include "repair/GlobalRepair.hpp"

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace mesheal {

enum class CliAction {
    None,
    Help,
    Version,
    Diagnostics,
    FixAll,
    RemeshVoxel,
    RemeshIso,
    RebuildAlpha,
    RebuildUnion,
    RebuildWinding
};

struct CliOptions {
    CliAction action = CliAction::None;
    std::filesystem::path inputPath;
    std::optional<std::filesystem::path> outputPath;

    int passes = 3;
    double timeBudgetSeconds = 0.0;

    double edgeLength = 0.0;
    int smoothingPasses = 3;
    double chordRatio = 0.10;

    double alpha = 0.0;
    double offset = 0.0;

    int resolution = 128;

    bool jsonOutput = false;
    bool quiet = false;
    bool noRelaunch = false;
};

/// Parses command-line arguments into CliOptions.
[[nodiscard]] CliOptions parseCliArguments(int argc, char const* const* argv);

/// Checks if the supplied options indicate a headless CLI execution.
[[nodiscard]] bool isHeadlessCommand(const CliOptions& options) noexcept;

/// Executes the headless command (repair, remesh, diagnostics, help, version).
/// Emits human-readable or JSON output based on options.jsonOutput.
/// Returns process exit code (0 for success, non-zero for error).
int executeCliCommand(const CliOptions& options);

} // namespace mesheal

