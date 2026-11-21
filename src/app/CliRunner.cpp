#include "app/CliRunner.hpp"
#include "app/AboutInfo.hpp"
#include "diagnostics/MeshHealth.hpp"
#include "diagnostics/MeshState.hpp"
#include "io/StlReader.hpp"
#include "io/StlWriter.hpp"
#include "mesh/Mesh.hpp"
#include "repair/FixAll.hpp"
#include "repair/GlobalRepair.hpp"
#include "repair/MergePlan.hpp"
#include "repair/MeshCleanup.hpp"
#include "repair/RepairMetrics.hpp"
#include "repair/TopologyCleanup.hpp"
#include "mesheal/Version.hpp"

#include <array>
#include <chrono>
#include <exception>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string_view>

namespace mesheal {

namespace {

void printJsonEscape(std::ostream& os, std::string_view s) {
    for (const char c : s) {
        switch (c) {
        case '"':  os << "\\\""; break;
        case '\\': os << "\\\\"; break;
        case '\b': os << "\\b"; break;
        case '\f': os << "\\f"; break;
        case '\n': os << "\\n"; break;
        case '\r': os << "\\r"; break;
        case '\t': os << "\\t"; break;
        default:
            if (static_cast<unsigned char>(c) < 0x20) {
                os << "\\u" << std::hex << std::setw(4) << std::setfill('0') << static_cast<int>(c);
            } else {
                os << c;
            }
            break;
        }
    }
}

const char* stepStatusName(FixAllStepStatus status) {
    switch (status) {
    case FixAllStepStatus::Applied:   return "applied";
    case FixAllStepStatus::Unchanged: return "unchanged";
    case FixAllStepStatus::Refused:   return "refused";
    case FixAllStepStatus::Skipped:   return "skipped";
    }
    return "unknown";
}

/// Fix All's steps that did something or were refused (skipped and unchanged steps are left out, except
/// when traced: then only skipped steps are).
std::vector<const FixAllStepReport*> reportedSteps(const RepairCandidate& candidate) {
    std::vector<const FixAllStepReport*> steps;
    if (candidate.fixAll) {
        for (const FixAllStepReport& step : candidate.fixAll->result.steps) {
            const bool traced = step.state.has_value();
            if (step.status == FixAllStepStatus::Applied || step.status == FixAllStepStatus::Refused ||
                (traced && step.status == FixAllStepStatus::Unchanged)) {
                steps.push_back(&step);
            }
        }
    }
    return steps;
}

void printJsonKindCounts(std::ostream& os, const std::array<std::size_t, kCrossingKindCount>& counts) {
    os << "{";
    for (std::size_t k = 0; k < kCrossingKindCount; ++k) {
        os << (k == 0 ? "" : ",") << "\"" << crossingKindName(static_cast<CrossingKind>(k)) << "\":" << counts[k];
    }
    os << "}";
}

void printJsonState(std::ostream& os, const MeshState& state) {
    os << "{\"faces\":" << state.faces << ",\"vertices\":" << state.vertices << ",\"shells\":" << state.shells
       << ",\"volume\":" << state.volume << ",\"open_edges\":" << state.openEdges
       << ",\"non_manifold_edges\":" << state.nonManifoldEdges << ",\"non_manifold_vertices\":" << state.nonManifoldVertices
       << ",\"flipped_faces\":" << state.flippedFaces << ",\"inverted_shells\":" << state.invertedShells
       << ",\"degenerate_faces\":" << state.degenerateFaces << ",\"duplicate_faces\":" << state.duplicateFaces
       << ",\"thin_faces\":" << state.thinFaces << ",\"crossing_faces\":" << state.crossingFaces
       << ",\"pairs_within_shell\":";
    printJsonKindCounts(os, state.pairsWithinShell);
    os << ",\"pairs_between_shells\":";
    printJsonKindCounts(os, state.pairsBetweenShells);
    os << ",\"complete\":" << (state.complete ? "true" : "false") << "}";
}

void printJsonSteps(std::ostream& os, const RepairCandidate& candidate) {
    os << "\"steps\":[";
    bool first = true;
    for (const FixAllStepReport* step : reportedSteps(candidate)) {
        os << (first ? "" : ",") << "{\"pass\":" << step->pass << ",\"phase\":" << static_cast<int>(step->phase)
           << ",\"stage\":\"";
        printJsonEscape(os, step->label);
        os << "\",\"status\":\"" << stepStatusName(step->status) << "\",\"count_before\":" << step->countBefore
           << ",\"count_after\":" << step->countAfter << ",\"faces_removed\":" << step->facesRemoved
           << ",\"faces_added\":" << step->facesAdded << ",\"elapsed_ms\":" << step->elapsedMs << ",\"detail\":\"";
        printJsonEscape(os, step->status == FixAllStepStatus::Refused ? step->reason : step->detail);
        os << "\"";
        if (step->state) {
            os << ",\"state\":";
            printJsonState(os, *step->state);
        }
        if (step->refusedState) {
            os << ",\"refused_state\":";
            printJsonState(os, *step->refusedState);
        }
        os << "}";
        first = false;
    }
    os << "],";
    if (candidate.fixAll) {
        const FixAllResult& result = candidate.fixAll->result;
        if (result.stateBefore) {
            os << "\"state_before\":";
            printJsonState(os, *result.stateBefore);
            os << ",";
        }
        os << "\"best_pass\":" << result.bestPass << ",\"stop_reason\":" << static_cast<int>(result.stopReason)
           << ",\"passes\":[";
        for (std::size_t i = 0; i < result.passes.size(); ++i) {
            const FixAllPassReport& pass = result.passes[i];
            os << (i == 0 ? "" : ",") << "{\"pass\":" << pass.pass << ",\"closed_manifold\":" << pass.after.closedManifold
               << ",\"oriented\":" << pass.after.oriented << ",\"intersecting\":" << pass.after.intersecting
               << ",\"junk\":" << pass.after.junk << ",\"thin\":" << pass.after.thin
               << ",\"new_best\":" << (pass.newBest ? "true" : "false") << ",\"repeated\":" << (pass.repeated ? "true" : "false")
               << ",\"elapsed_ms\":" << pass.elapsedMs << "}";
        }
        os << "],";
    }
}

/// One line of a traced step's state: what is left that Fix All scores, and the crossing pairs by kind.
void printTextState(std::ostream& os, const MeshState& state) {
    os << "      state: open " << state.openEdges << ", nm edges " << state.nonManifoldEdges << ", nm vertices "
       << state.nonManifoldVertices << ", flipped " << state.flippedFaces << ", thin " << state.thinFaces
       << ", shells " << state.shells << ", crossing faces " << state.crossingFaces << " (pairs";
    for (std::size_t k = 0; k < kCrossingKindCount; ++k) {
        const std::size_t within = state.pairsWithinShell[k];
        const std::size_t between = state.pairsBetweenShells[k];
        if (within + between > 0) {
            os << " " << crossingKindName(static_cast<CrossingKind>(k)) << " " << within << "+" << between;
        }
    }
    os << ")\n";
}

void printTextSteps(std::ostream& os, const RepairCandidate& candidate) {
    if (candidate.fixAll && candidate.fixAll->result.stateBefore) {
        os << "  input\n";
        printTextState(os, *candidate.fixAll->result.stateBefore);
    }
    for (const FixAllStepReport* step : reportedSteps(candidate)) {
        os << "  pass " << step->pass << "." << static_cast<int>(step->phase) << " " << step->label << ": "
           << stepStatusName(step->status) << " (" << step->countBefore << " -> " << step->countAfter << ", -"
           << step->facesRemoved << "/+" << step->facesAdded << " faces)";
        const std::string& text = step->status == FixAllStepStatus::Refused ? step->reason : step->detail;
        if (!text.empty()) {
            os << " " << text;
        }
        os << "\n";
        if (step->state && step->status != FixAllStepStatus::Unchanged) {
            printTextState(os, *step->state);
        }
        if (step->refusedState) {
            os << "      refused result:\n";
            printTextState(os, *step->refusedState);
        }
    }
    if (candidate.fixAll) {
        const FixAllResult& result = candidate.fixAll->result;
        for (const FixAllPassReport& pass : result.passes) {
            os << "  pass " << pass.pass << " score: closed/manifold " << pass.after.closedManifold << ", oriented "
               << pass.after.oriented << ", intersecting " << pass.after.intersecting << ", junk " << pass.after.junk
               << ", thin " << pass.after.thin << (pass.newBest ? " (best so far)" : "") << "\n";
        }
        os << "  returned pass " << result.bestPass << "\n";
    }
}

std::string actionName(CliAction action) {
    switch (action) {
    case CliAction::Dedup:          return "dedup";
    case CliAction::FixAll:         return "fix_all";
    case CliAction::RemeshVoxel:    return "remesh_voxel";
    case CliAction::RemeshIso:      return "remesh_iso";
    case CliAction::RebuildAlpha:   return "rebuild_alpha";
    case CliAction::RebuildUnion:   return "rebuild_union";
    case CliAction::RebuildWinding: return "rebuild_winding";
    case CliAction::Diagnostics:    return "diagnostics";
    case CliAction::Help:           return "help";
    case CliAction::Version:        return "version";
    case CliAction::None:           return "none";
    }
    return "unknown";
}

RepairCandidate runDedupPass(const Mesh& mesh, const GlobalRepairParams& params) {
    RepairCandidate candidate;
    Mesh work = mesh;

    const MergeByDistanceResult weld = mergeByDistance(work, params.fixAll.weldTolerance, {}, true);
    if (!weld.success) {
        candidate.statusMessage = weld.statusMessage;
        return candidate;
    }
    if ((weld.verticesMerged + weld.degenerateRemoved + weld.duplicatesRemoved) > 0) {
        work = weld.mesh;
    }

    const RepeatedVertexFacesResult dropped = removeRepeatedVertexFaces(work);
    if (dropped.removed > 0) {
        work = dropped.mesh;
    }

    const DuplicateFacesResult deduped = removeDuplicateFaces(work);
    if (deduped.removed() > 0) {
        work = deduped.mesh;
    }

    if (work.empty()) {
        candidate.statusMessage = "Deduplication removed all faces";
        return candidate;
    }

    candidate.mesh = std::move(work);
    candidate.success = true;
    candidate.metrics = computeRepairMetrics(mesh, candidate.mesh, params.checkParams);
    return candidate;
}

} // namespace

std::string usageText(std::string_view prog) {
    std::ostringstream os;
    os << "Mesheal " << kVersionString << " - inspect and repair STL meshes for simulation\n\n"
       << "Usage:\n"
       << "  " << prog << " [<input.stl>]\n"
       << "      Open the 3D desktop app, optionally with a file.\n"
       << "  " << prog << " <action> [options] <input.stl>\n"
       << "      Run headless (no window): check or repair the file, write the result and exit.\n\n"
       << "Actions (pick one):\n"
       << "  --diagnostics, --inspect Report mesh health; never changes or writes the mesh\n"
       << "  --dedup                  Weld nearby vertices and remove duplicate/collapsed faces only\n"
       << "  --fix-all                Multi-pass Fix All: weld cracks, remove collapsed and duplicate\n"
       << "                           faces, heal slivers, split non-manifold contacts, fix normals,\n"
       << "                           fill holes and repair intersections\n"
       << "  --remesh-voxel           Voxel Heal & Remesh (fast; Taubin smoothed, keeps cavities)\n"
       << "  --remesh-iso             Precision Isosurface remesh (FEA grade, Delaunay refinement)\n"
       << "  --rebuild-alpha          Shrink-Wrap outer envelope (CGAL 3D Alpha Wrap; fills cavities)\n"
       << "  --rebuild-union          Self-union outer surface (CGAL autorefinement)\n"
       << "  --rebuild-winding        Generalized winding number marching cubes rebuild\n\n"
       << "Repair options:\n"
       << "  --passes <N>             Maximum Fix All passes (default: 3)\n"
       << "  --time-budget <sec>      Stop Fix All after this many seconds (default: 0 = no limit)\n"
       << "  --merge-shells           Fix All always merges shells that overlap or nest (default: when\n"
       << "                           shells cross, checked on the input and again after each pass)\n"
       << "  --no-merge-shells        Fix All never merges shells\n"
       << "  --no-unite-shells        Fix All never cuts a closed shell that crosses itself along its\n"
       << "                           crossings (default: it does, keeping the outer surface)\n"
       << "  --report                 List each Fix All step that ran (with --json: a \"steps\" array)\n"
       << "  --trace                  --report, plus the whole mesh measured after every Fix All step\n"
       << "                           (every check, crossing pairs by kind; slow on large meshes)\n"
       << "  --edge-length <len>      Target edge length for remeshing, in file units (default: auto)\n"
       << "  --smoothing <N>          Smoothing passes for --remesh-voxel (default: 3)\n"
       << "  --chord-ratio <ratio>    Surface approximation for --remesh-iso (default: 0.10)\n"
       << "  --alpha <val>            Alpha (probe size) for --rebuild-alpha (default: auto)\n"
       << "  --offset <val>           Offset distance for --rebuild-alpha (default: auto)\n"
       << "  --resolution <N>         Grid resolution for --rebuild-winding (default: 128, min 16)\n\n"
       << "Output options:\n"
       << "  -o, --output <file>      Where to write the repaired STL. Never overwrites the input:\n"
       << "                           without -o, writes <stem>_fixed_1.stl (_2, _3, ... if taken)\n"
       << "  --json                   Print one JSON object with before/after metrics to stdout\n"
       << "  --quiet                  Suppress informational output (errors still print)\n\n"
       << "Other:\n"
       << "  -v, --version            Print version and build information\n"
       << "  -h, --help               Print this help\n\n"
       << "Examples:\n"
       << "  " << prog << " part.stl --diagnostics\n"
       << "  " << prog << " part.stl --fix-all -o part_clean.stl\n"
       << "  " << prog << " part.stl --fix-all --json > metrics.json\n"
       << "  " << prog << " scan.stl --remesh-voxel --edge-length 0.5\n\n"
       << "Exit status: 0 on success, 1 on error (bad input, failed repair, failed write).\n"
       << "Documentation: " << kDocumentationUrl << "\n";
    return os.str();
}

namespace {

void printUsage(const char* prog) {
    std::cout << usageText(prog);
}

} // namespace

CliOptions parseCliArguments(int argc, char const* const* argv) {
    CliOptions opt;
    for (int i = 1; i < argc; ++i) {
        const std::string_view arg(argv[i]);
        if (arg == "-h" || arg == "--help") {
            opt.action = CliAction::Help;
        } else if (arg == "-v" || arg == "--version") {
            opt.action = CliAction::Version;
        } else if (arg == "--diagnostics" || arg == "--inspect") {
            opt.action = CliAction::Diagnostics;
        } else if (arg == "--dedup") {
            opt.action = CliAction::Dedup;
        } else if (arg == "--fix-all") {
            opt.action = CliAction::FixAll;
        } else if (arg == "--remesh-voxel") {
            opt.action = CliAction::RemeshVoxel;
        } else if (arg == "--remesh-iso") {
            opt.action = CliAction::RemeshIso;
        } else if (arg == "--rebuild-alpha") {
            opt.action = CliAction::RebuildAlpha;
        } else if (arg == "--rebuild-union") {
            opt.action = CliAction::RebuildUnion;
        } else if (arg == "--rebuild-winding") {
            opt.action = CliAction::RebuildWinding;
        } else if (arg == "--json") {
            opt.jsonOutput = true;
        } else if (arg == "--quiet") {
            opt.quiet = true;
        } else if (arg == "--no-relaunch") {
            opt.noRelaunch = true;
        } else if ((arg == "-o" || arg == "--output") && i + 1 < argc) {
            opt.outputPath = std::filesystem::path(argv[++i]);
        } else if (arg == "--passes" && i + 1 < argc) {
            opt.passes = std::max(1, std::stoi(argv[++i]));
        } else if (arg == "--report") {
            opt.report = true;
        } else if (arg == "--trace") {
            opt.report = true;
            opt.trace = true;
        } else if (arg == "--merge-shells") {
            opt.mergeShells = true;
        } else if (arg == "--no-merge-shells") {
            opt.mergeShells = false;
        } else if (arg == "--no-unite-shells") {
            opt.uniteShells = false;
        } else if (arg == "--time-budget" && i + 1 < argc) {
            opt.timeBudgetSeconds = std::stod(argv[++i]);
        } else if (arg == "--edge-length" && i + 1 < argc) {
            opt.edgeLength = std::stod(argv[++i]);
        } else if (arg == "--smoothing" && i + 1 < argc) {
            opt.smoothingPasses = std::max(0, std::stoi(argv[++i]));
        } else if (arg == "--chord-ratio" && i + 1 < argc) {
            opt.chordRatio = std::stod(argv[++i]);
        } else if (arg == "--alpha" && i + 1 < argc) {
            opt.alpha = std::stod(argv[++i]);
        } else if (arg == "--offset" && i + 1 < argc) {
            opt.offset = std::stod(argv[++i]);
        } else if (arg == "--resolution" && i + 1 < argc) {
            opt.resolution = std::max(16, std::stoi(argv[++i]));
        } else if (!arg.starts_with("-") && !arg.starts_with("-psn_")) {
            if (opt.inputPath.empty()) {
                opt.inputPath = std::filesystem::path(arg);
            }
        }
    }
    return opt;
}

bool isHeadlessCommand(const CliOptions& options) noexcept {
    return options.action != CliAction::None || options.jsonOutput || options.outputPath.has_value();
}

int executeCliCommand(const CliOptions& options) {
    if (options.action == CliAction::Help) {
        printUsage("mesheal");
        return 0;
    }
    if (options.action == CliAction::Version) {
        std::cout << buildInfoText() << "\n";
        return 0;
    }

    if (options.inputPath.empty()) {
        if (options.jsonOutput) {
            std::cout << "{\"status\":\"error\",\"error\":\"No input STL file specified.\"}\n";
        } else {
            std::cerr << "Error: No input STL file specified. Use --help for usage.\n";
        }
        return 1;
    }

    Mesh mesh;
    try {
        mesh = loadStl(options.inputPath);
    } catch (const std::exception& e) {
        if (options.jsonOutput) {
            std::cout << "{\"status\":\"error\",\"error\":\"Failed to load STL: ";
            printJsonEscape(std::cout, e.what());
            std::cout << "\"}\n";
        } else {
            std::cerr << "Error: Failed to load STL: " << e.what() << "\n";
        }
        return 1;
    }

    const CheckParams checkParams{};

    // Inspection / Diagnostics only
    if (options.action == CliAction::Diagnostics) {
        const auto startTime = std::chrono::steady_clock::now();
        std::array<CategoryResult, kDefectCategoryCount> results;
        for (const DefectCategory cat : kAllDefectCategories) {
            results[categoryIndex(cat)] = runCategoryCheck(cat, mesh, checkParams);
        }

        const double vol = signedVolume(mesh);
        const double area = surfaceArea(mesh);
        const double elapsedMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - startTime).count();

        const std::size_t openEdges = results[categoryIndex(DefectCategory::OpenEdges)].infoCount;
        const std::size_t nonManifoldEdges = results[categoryIndex(DefectCategory::NonManifoldEdges)].size();
        const std::size_t nonManifoldVertices = results[categoryIndex(DefectCategory::NonManifoldVertices)].size();
        const std::size_t flippedFaces = results[categoryIndex(DefectCategory::FlippedNormals)].size();
        const std::size_t invertedShells = results[categoryIndex(DefectCategory::InvertedShells)].size();
        const std::size_t shells = results[categoryIndex(DefectCategory::Shells)].infoCount;
        const std::size_t selfIntersections = results[categoryIndex(DefectCategory::Intersections)].size();
        const std::size_t thinFaces = results[categoryIndex(DefectCategory::ThinFaces)].size();
        const std::size_t degenerateFaces = results[categoryIndex(DefectCategory::Degenerate)].size();
        const std::size_t duplicateFaces = results[categoryIndex(DefectCategory::Duplicates)].size();
        const bool watertight = openEdges == 0 && nonManifoldEdges == 0;

        if (options.jsonOutput) {
            std::cout << "{"
                      << "\"status\":\"success\","
                      << "\"action\":\"diagnostics\","
                      << "\"input\":\"";
            printJsonEscape(std::cout, options.inputPath.string());
            std::cout << "\","
                      << "\"elapsed_ms\":" << elapsedMs << ","
                      << "\"metrics\":{"
                      << "\"volume\":" << vol << ","
                      << "\"surface_area\":" << area << ","
                      << "\"faces\":" << mesh.numFaces() << ","
                      << "\"shells\":" << shells << ","
                      << "\"watertight\":" << (watertight ? "true" : "false") << ","
                      << "\"open_edges\":" << openEdges << ","
                      << "\"non_manifold_edges\":" << nonManifoldEdges << ","
                      << "\"non_manifold_vertices\":" << nonManifoldVertices << ","
                      << "\"flipped_faces\":" << flippedFaces << ","
                      << "\"inverted_shells\":" << invertedShells << ","
                      << "\"self_intersecting_faces\":" << selfIntersections << ","
                      << "\"thin_faces\":" << thinFaces << ","
                      << "\"degenerate_faces\":" << degenerateFaces << ","
                      << "\"duplicate_faces\":" << duplicateFaces
                      << "}}\n";
        } else if (!options.quiet) {
            std::cout << "Mesh Diagnostics for: " << options.inputPath.string() << "\n"
                      << "  Faces: " << mesh.numFaces() << " | Shells: " << shells << "\n"
                      << "  Signed Volume: " << vol << " mm³ | Surface Area: " << area << " mm²\n"
                      << "  Watertight: " << (watertight ? "Yes" : "No") << "\n"
                      << "  Open Edges: " << openEdges << "\n"
                      << "  Non-manifold Edges: " << nonManifoldEdges << "\n"
                      << "  Non-manifold Vertices: " << nonManifoldVertices << "\n"
                      << "  Self-intersecting Faces: " << selfIntersections << "\n"
                      << "  Thin Faces: " << thinFaces << "\n"
                      << "  Flipped Faces: " << flippedFaces << "\n"
                      << "  Elapsed: " << elapsedMs << " ms\n";
        }
        return 0;
    }

    // Default to FixAll if an output or headless mode is requested without specific rebuild action
    CliAction effectiveAction = options.action;
    if (effectiveAction == CliAction::None) {
        effectiveAction = CliAction::FixAll;
    }

    GlobalRepairMethod method = GlobalRepairMethod::FixAll;
    switch (effectiveAction) {
    case CliAction::FixAll:         method = GlobalRepairMethod::FixAll; break;
    case CliAction::RemeshVoxel:    method = GlobalRepairMethod::VoxelHealRemesh; break;
    case CliAction::RemeshIso:      method = GlobalRepairMethod::PrecisionIsosurface; break;
    case CliAction::RebuildAlpha:   method = GlobalRepairMethod::AlphaWrap; break;
    case CliAction::RebuildUnion:   method = GlobalRepairMethod::SelfUnion; break;
    case CliAction::RebuildWinding: method = GlobalRepairMethod::WindingRebuild; break;
    default:                        method = GlobalRepairMethod::FixAll; break;
    }

    std::filesystem::path outputPath;
    try {
        outputPath = resolveAvailableOutputPath(options.inputPath, options.outputPath);
    } catch (const std::exception& e) {
        if (options.jsonOutput) {
            std::cout << "{\"status\":\"error\",\"error\":\"";
            printJsonEscape(std::cout, e.what());
            std::cout << "\"}\n";
        } else {
            std::cerr << "Error: " << e.what() << "\n";
        }
        return 1;
    }

    GlobalRepairParams params{};
    params.checkParams = checkParams;
    params.fixAll.passes = options.passes;
    params.fixAll.timeBudgetSeconds = options.timeBudgetSeconds;
    if (effectiveAction == CliAction::FixAll) {
        // Unless forced either way, merge when shells cross (not when one only sits inside another), and
        // let Fix All check again after each pass's repairs
        MergePlanParams planParams;
        planParams.intersectionTolerance = checkParams.intersectionTolerance;
        params.fixAll.mergeShells =
            options.mergeShells.value_or(shellsCross(mesh, FixAllScope::whole(mesh), planParams).value_or(false));
        params.fixAll.autoMergeShells = !options.mergeShells.has_value();
        params.fixAll.uniteSelfCrossingShells = options.uniteShells;
        params.fixAll.traceSteps = options.trace;
    }
    params.remeshTargetSize = options.edgeLength;
    params.voxelSmoothingPasses = options.smoothingPasses;
    params.isosurfaceSmoothingPasses = options.smoothingPasses;
    params.isosurfaceChordRatio = options.chordRatio;
    params.windingResolution = options.resolution;

    if (!options.quiet && !options.jsonOutput) {
        std::cout << "Executing " << actionName(effectiveAction) << " on " << options.inputPath.string() << "...\n";
    }

    const auto startTime = std::chrono::steady_clock::now();
    const RepairCandidate candidate = (effectiveAction == CliAction::Dedup)
                                          ? runDedupPass(mesh, params)
                                          : runGlobalRepair(mesh, {}, method, params);
    const double elapsedMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - startTime).count();

    if (!candidate.success || candidate.mesh.empty()) {
        const std::string err = candidate.statusMessage.empty() ? "Repair failed to produce valid mesh" : candidate.statusMessage;
        if (options.jsonOutput) {
            std::cout << "{\"status\":\"error\",\"action\":\"" << actionName(effectiveAction) << "\",\"error\":\"";
            printJsonEscape(std::cout, err);
            std::cout << "\"}\n";
        } else {
            std::cerr << "Error: " << err << "\n";
        }
        return 1;
    }

    std::string saveErr;
    if (!saveStl(outputPath, candidate.mesh, StlFormat::Binary, &saveErr)) {
        if (options.jsonOutput) {
            std::cout << "{\"status\":\"error\",\"error\":\"Failed to save output STL: ";
            printJsonEscape(std::cout, saveErr);
            std::cout << "\"}\n";
        } else {
            std::cerr << "Error: Failed to save output STL: " << saveErr << "\n";
        }
        return 1;
    }

    const RepairMetrics& m = candidate.metrics;
    if (options.jsonOutput) {
        std::cout << "{"
                  << "\"status\":\"success\","
                  << "\"action\":\"" << actionName(effectiveAction) << "\","
                  << "\"input\":\"";
        printJsonEscape(std::cout, options.inputPath.string());
        std::cout << "\","
                  << "\"output\":\"";
        printJsonEscape(std::cout, outputPath.string());
        std::cout << "\",";
        if (effectiveAction == CliAction::FixAll) {
            const int turnedOn = candidate.fixAll ? candidate.fixAll->result.mergeTurnedOnInPass : 0;
            std::cout << "\"merge_shells\":" << (params.fixAll.mergeShells || turnedOn > 0 ? "true" : "false") << ","
                      << "\"merge_turned_on_in_pass\":" << turnedOn << ",";
        }
        if (options.report) {
            printJsonSteps(std::cout, candidate);
        }
        std::cout << "\"elapsed_ms\":" << elapsedMs << ","
                  << "\"metrics\":{"
                  << "\"volume_before\":" << m.volumeBefore << ","
                  << "\"volume_after\":" << m.volumeAfter << ","
                  << "\"volume_change_pct\":" << m.volumeChangePercent() << ","
                  << "\"area_before\":" << m.areaBefore << ","
                  << "\"area_after\":" << m.areaAfter << ","
                  << "\"area_change_pct\":" << m.areaChangePercent() << ","
                  << "\"max_deviation\":" << m.maxDeviation << ","
                  << "\"max_lost\":" << m.maxLost << ","
                  << "\"faces_before\":" << m.facesBefore << ","
                  << "\"faces_after\":" << m.facesAfter << ","
                  << "\"shells_before\":" << m.shellsBefore << ","
                  << "\"shells_after\":" << m.shellsAfter << ","
                  << "\"watertight_before\":" << (m.openEdgesBefore == 0 && m.nonManifoldEdgesBefore == 0 ? "true" : "false") << ","
                  << "\"watertight_after\":" << (m.watertight() ? "true" : "false") << ","
                  << "\"open_edges_before\":" << m.openEdgesBefore << ","
                  << "\"open_edges_after\":" << m.openEdgesAfter << ","
                  << "\"non_manifold_edges_before\":" << m.nonManifoldEdgesBefore << ","
                  << "\"non_manifold_edges_after\":" << m.nonManifoldEdgesAfter << ","
                  << "\"non_manifold_vertices_before\":" << m.nonManifoldVerticesBefore << ","
                  << "\"non_manifold_vertices_after\":" << m.nonManifoldVerticesAfter << ","
                  << "\"self_intersecting_faces_before\":" << m.intersectingFacesBefore << ","
                  << "\"self_intersecting_faces_after\":" << m.intersectingFacesAfter << ","
                  << "\"thin_faces_before\":" << m.thinFacesBefore << ","
                  << "\"thin_faces_after\":" << m.thinFacesAfter << ","
                  << "\"flipped_faces_before\":" << m.flippedFacesBefore << ","
                  << "\"flipped_faces_after\":" << m.flippedFacesAfter << ","
                  << "\"inverted_shells_before\":" << m.invertedShellsBefore << ","
                  << "\"inverted_shells_after\":" << m.invertedShellsAfter << ","
                  << "\"degenerate_faces_before\":" << m.degenerateFacesBefore << ","
                  << "\"degenerate_faces_after\":" << m.degenerateFacesAfter << ","
                  << "\"duplicate_faces_before\":" << m.duplicateFacesBefore << ","
                  << "\"duplicate_faces_after\":" << m.duplicateFacesAfter
                  << "}}\n";
    } else if (!options.quiet) {
        std::cout << "Successfully saved repaired mesh to: " << outputPath.string() << "\n"
                  << "  Faces: " << m.facesBefore << " -> " << m.facesAfter << "\n"
                  << "  Volume: " << m.volumeBefore << " -> " << m.volumeAfter
                  << " (" << (m.volumeChangePercent() >= 0 ? "+" : "") << m.volumeChangePercent() << "%)\n"
                  << "  Watertight: " << (m.watertight() ? "Yes" : "No") << "\n"
                  << "  Elapsed: " << elapsedMs << " ms\n";
        if (effectiveAction == CliAction::FixAll) {
            const int turnedOn = candidate.fixAll ? candidate.fixAll->result.mergeTurnedOnInPass : 0;
            if (params.fixAll.mergeShells) {
                std::cout << (options.mergeShells.has_value() ? "  Merge shells: on (--merge-shells)\n"
                                                              : "  Merge shells: on (shells cross)\n");
            } else if (turnedOn > 0) {
                std::cout << "  Merge shells: turned on in pass " << turnedOn << " (its repairs left shells that cross)\n";
            }
        }
        if (options.report) {
            printTextSteps(std::cout, candidate);
        }
    }

    return 0;
}

} // namespace mesheal
