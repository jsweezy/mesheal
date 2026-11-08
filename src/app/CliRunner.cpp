#include "app/CliRunner.hpp"
#include "app/AboutInfo.hpp"
#include "diagnostics/MeshHealth.hpp"
#include "io/StlReader.hpp"
#include "io/StlWriter.hpp"
#include "mesh/Mesh.hpp"
#include "repair/GlobalRepair.hpp"
#include "repair/RepairMetrics.hpp"

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

std::string actionName(CliAction action) {
    switch (action) {
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

void printUsage(const char* prog) {
    std::cout
        << "Mesheal STL Repair Tool — Headless CLI & 3D Viewer\n\n"
        << "Usage:\n"
        << "  " << prog << " [options] <input.stl>\n\n"
        << "Repair & Rebuild Actions:\n"
        << "  --fix-all                Execute multi-pass automated Fix All repair pipeline\n"
        << "  --remesh-voxel           Pipeline A: Voxel Heal & Remesh (Fast, Taubin smoothed)\n"
        << "  --remesh-iso             Pipeline B: Precision Isosurface Remesh (FEA grade)\n"
        << "  --rebuild-alpha          Surface rebuild via CGAL 3D Alpha Wrap\n"
        << "  --rebuild-union          Self-union outer hull (CGAL autorefinement)\n"
        << "  --rebuild-winding        Generalized winding number marching cubes\n"
        << "  --diagnostics, --inspect Inspect mesh health without modifying geometry\n\n"
        << "Repair Tuning Options:\n"
        << "  --passes <N>             Maximum passes for Fix All (default: 3)\n"
        << "  --time-budget <sec>      Time budget in seconds for Fix All (default: 0 = none)\n"
        << "  --edge-length <len>      Target edge length for remeshing (default: auto)\n"
        << "  --smoothing <N>          Smoothing passes for remeshing (default: 3)\n"
        << "  --chord-ratio <ratio>    Chord ratio for precision remeshing (default: 0.10)\n"
        << "  --alpha <val>            Alpha parameter for alpha wrap (default: auto)\n"
        << "  --offset <val>           Offset parameter for alpha wrap (default: auto)\n"
        << "  --resolution <N>         Grid resolution for winding rebuild (default: 128)\n\n"
        << "Output Options:\n"
        << "  -o, --output <file>      Explicit output path (never overwrites input)\n"
        << "                           If omitted, auto-names to <stem>_fixed_1.stl, _2.stl, etc.\n"
        << "  --json                   Stream machine-readable JSON metrics to stdout\n"
        << "  --quiet                  Suppress non-JSON informational stdout\n"
        << "  -v, --version            Display version information\n"
        << "  -h, --help               Display this help text\n";
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
    params.remeshTargetSize = options.edgeLength;
    params.voxelSmoothingPasses = options.smoothingPasses;
    params.isosurfaceSmoothingPasses = options.smoothingPasses;
    params.isosurfaceChordRatio = options.chordRatio;
    params.windingResolution = options.resolution;

    if (!options.quiet && !options.jsonOutput) {
        std::cout << "Executing " << actionName(effectiveAction) << " on " << options.inputPath.string() << "...\n";
    }

    const auto startTime = std::chrono::steady_clock::now();
    const RepairCandidate candidate = runGlobalRepair(mesh, {}, method, params);
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
        std::cout << "\","
                  << "\"elapsed_ms\":" << elapsedMs << ","
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
    }

    return 0;
}

} // namespace mesheal
