// Phase 1 of Fix All: the stage table and the stages (steps 0b to 11). Each stage is a plain function
// that runs one existing primitive on the current mesh and scope, and returns the mesh, its face map,
// the counts of its own target and a report line. The engine (FixAll.cpp) checks the guards.

#include "repair/FixAll.hpp"
#include "repair/FixAllDetail.hpp"

#include "diagnostics/HoleLoops.hpp"
#include "diagnostics/MeshChecks.hpp"
#include "diagnostics/ShellRelations.hpp"
#include "repair/MeshCleanup.hpp"
#include "repair/Orientation.hpp"
#include "repair/RepairMetrics.hpp"
#include "repair/SliverHealer.hpp"
#include "repair/TopologyCleanup.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cstdio>
#include <unordered_map>
#include <utility>

namespace ogun {

namespace fixall_detail {

std::string counted(std::size_t n, const char* one, const char* many) {
    return std::to_string(n) + " " + (n == 1 ? one : many);
}

std::string formatLength(double value, Unit unit) {
    std::array<char, 32> text{};
    std::snprintf(text.data(), text.size(), "%.3g", value);
    return std::string(text.data()) + " " + std::string(unitSymbol(unit));
}

std::string formatVolume(double value, Unit unit) {
    std::array<char, 32> text{};
    std::snprintf(text.data(), text.size(), "%.6g", value);
    return std::string(text.data()) + " " + std::string(unitSymbol(unit)) + "^3";
}

std::string shellList(std::span<const int> shells) {
    std::string text;
    for (std::size_t i = 0; i < shells.size(); ++i) {
        if (i > 0) {
            text += i + 1 == shells.size() ? " and " : ", ";
        }
        text += "#" + std::to_string(shells[i] + 1);
    }
    return text;
}

std::vector<Eigen::Index> faceMapByGeometry(const Mesh& before, const Mesh& after) {
    std::unordered_map<FaceGeometryKey, std::vector<Eigen::Index>, FaceGeometryKeyHash> rows;
    rows.reserve(static_cast<std::size_t>(after.numFaces()));
    for (Eigen::Index f = after.numFaces() - 1; f >= 0; --f) {
        rows[faceGeometryKey(after, f)].push_back(f); // popped from the back: lowest row first
    }
    std::vector<Eigen::Index> map(static_cast<std::size_t>(before.numFaces()), -1);
    for (Eigen::Index f = 0; f < before.numFaces(); ++f) {
        const auto it = rows.find(faceGeometryKey(before, f));
        if (it != rows.end() && !it->second.empty()) {
            map[static_cast<std::size_t>(f)] = it->second.back();
            it->second.pop_back();
        }
    }
    return map;
}

std::vector<int> shellOfFaces(Eigen::Index faceCount, std::span<const std::vector<Eigen::Index>> shells) {
    std::vector<int> shellOf(static_cast<std::size_t>(std::max<Eigen::Index>(faceCount, 0)), -1);
    for (std::size_t s = 0; s < shells.size(); ++s) {
        for (const Eigen::Index f : shells[s]) {
            if (f >= 0 && f < faceCount) {
                shellOf[static_cast<std::size_t>(f)] = static_cast<int>(s);
            }
        }
    }
    return shellOf;
}

EdgeDefectCounts countEdgeDefects(const Mesh& mesh) {
    EdgeDefectCounts counts;
    const EdgeTopology topo = buildEdgeTopology(mesh);
    for (std::size_t e = 0; e < topo.numEdges(); ++e) {
        const std::uint32_t faces = topo.faceOffsets[e + 1] - topo.faceOffsets[e];
        counts.open += faces == 1 ? 1 : 0;
        counts.nonManifold += faces > 2 ? 1 : 0;
    }
    return counts;
}

} // namespace fixall_detail

namespace {

using fixall_detail::counted;
using fixall_detail::formatLength;
using fixall_detail::shellList;

[[nodiscard]] std::string capitalised(std::string text) {
    if (!text.empty()) {
        text[0] = static_cast<char>(std::toupper(static_cast<unsigned char>(text[0])));
    }
    return text;
}

[[nodiscard]] std::string joined(const std::vector<std::string>& parts, const char* separator) {
    std::string text;
    for (const std::string& part : parts) {
        if (part.empty()) {
            continue;
        }
        if (!text.empty()) {
            text += separator;
        }
        text += part;
    }
    return text;
}

[[nodiscard]] std::string percentText(double fraction) {
    std::array<char, 32> text{};
    std::snprintf(text.data(), text.size(), "%.2g %%", 100.0 * fraction);
    return text.data();
}

[[nodiscard]] bool mergeRuns(const FixAllStageContext& context) {
    return context.params.mergeShells && context.scope.kind != FixAllScopeKind::Faces;
}

/// Faces of the listed shells, mapped through `faceMap` (empty = rows kept); removed faces are left out.
[[nodiscard]] std::vector<std::vector<Eigen::Index>> mappedShells(std::span<const std::vector<Eigen::Index>> shells,
                                                                  std::span<const int> which,
                                                                  std::span<const Eigen::Index> faceMap) {
    std::vector<std::vector<Eigen::Index>> faces;
    faces.reserve(which.size());
    for (const int s : which) {
        std::vector<Eigen::Index> list;
        if (s >= 0 && static_cast<std::size_t>(s) < shells.size()) {
            for (const Eigen::Index f : shells[static_cast<std::size_t>(s)]) {
                const Eigen::Index to = faceMap.empty() ? f : faceMap[static_cast<std::size_t>(f)];
                if (to >= 0) {
                    list.push_back(to);
                }
            }
        }
        faces.push_back(std::move(list));
    }
    return faces;
}

[[nodiscard]] bool repeatsVertex(const Mesh& mesh) {
    for (Eigen::Index f = 0; f < mesh.numFaces(); ++f) {
        if (mesh.F(f, 0) == mesh.F(f, 1) || mesh.F(f, 1) == mesh.F(f, 2) || mesh.F(f, 0) == mesh.F(f, 2)) {
            return true;
        }
    }
    return false;
}

/// Farthest a corner of a kept face moved (Close Crack snaps rim vertices and may renumber them).
[[nodiscard]] double maxMoveOfKeptFaces(const Mesh& before, const Mesh& after, std::span<const Eigen::Index> faceMap) {
    double move = 0.0;
    for (std::size_t f = 0; f < faceMap.size(); ++f) {
        const Eigen::Index to = faceMap[f];
        if (to < 0 || to >= after.numFaces()) {
            continue;
        }
        for (int c = 0; c < 3; ++c) {
            move = std::max(move, (after.V.row(after.F(to, c)) - before.V.row(before.F(static_cast<Eigen::Index>(f), c))).norm());
        }
    }
    return move;
}

// ---------------------------------------------------------------------------------------------
// Steps 0b to 3: weld and combinatorial clean-up
// ---------------------------------------------------------------------------------------------

[[nodiscard]] FixAllStageOutcome weldStage(const Mesh& mesh, const FixAllStageContext& context) {
    FixAllStageOutcome outcome;
    MergeByDistanceResult weld =
        mergeByDistance(mesh, context.params.weldTolerance, context.scope.faceInScope, true, context.scope.pinned);
    if (!weld.success) {
        outcome.failure = weld.statusMessage;
        return outcome;
    }
    outcome.targetBefore = weld.verticesMerged + weld.degenerateRemoved + weld.duplicatesRemoved;
    if (outcome.targetBefore == 0) {
        return outcome; // the weld would only drop unused vertices
    }
    outcome.changed = true;
    outcome.mesh = std::move(weld.mesh);
    outcome.faceMap = std::move(weld.faceMap);
    outcome.maxMove = weld.maxMove;
    outcome.detail = "Welded " + counted(weld.verticesMerged, "vertex", "vertices") + " (max move " +
                     formatLength(weld.maxMove, mesh.fileUnit) + "; " +
                     counted(weld.degenerateRemoved, "collapsed face", "collapsed faces") + " removed";
    if (weld.duplicatesRemoved > 0) {
        outcome.detail += ", " + counted(weld.duplicatesRemoved, "copy", "copies");
    }
    outcome.detail += ")";
    return outcome;
}

[[nodiscard]] FixAllStageOutcome dropRepeatedStage(const Mesh& mesh, const FixAllStageContext& context) {
    FixAllStageOutcome outcome;
    RepeatedVertexFacesResult dropped = removeRepeatedVertexFaces(mesh, context.scope.faceInScope);
    outcome.targetBefore = dropped.removed;
    if (dropped.removed == 0) {
        return outcome;
    }
    outcome.changed = true;
    outcome.mesh = std::move(dropped.mesh);
    outcome.faceMap = std::move(dropped.faceMap);
    outcome.detail = "Removed " + counted(dropped.removed, "collapsed face", "collapsed faces");
    return outcome;
}

[[nodiscard]] FixAllStageOutcome dedupeStage(const Mesh& mesh, const FixAllStageContext& context) {
    FixAllStageOutcome outcome;
    DuplicateFacesResult deduped = removeDuplicateFaces(mesh, context.scope.faceInScope);
    outcome.targetBefore = deduped.removed();
    if (deduped.removed() == 0) {
        return outcome;
    }
    outcome.changed = true;
    outcome.mesh = std::move(deduped.mesh);
    outcome.faceMap = std::move(deduped.faceMap);
    outcome.detail = "Removed " + counted(deduped.removed(), "duplicate face", "duplicate faces") + " (" +
                     std::to_string(deduped.removedSameWinding) + " same, " +
                     std::to_string(deduped.removedOppositeWinding) + " opposite winding)";
    if (deduped.groupsSkipped > 0) {
        outcome.detail += "; " + counted(deduped.groupsSkipped, "set of copies was", "sets of copies were") +
                          " left: a copy outside the scope would have to go";
    }
    return outcome;
}

[[nodiscard]] FixAllStageOutcome splitTouchingStage(const Mesh& mesh, const FixAllStageContext& context) {
    // As the load does (`separateTouchingContacts`): with every touching edge paired, each vertex is
    // split into its fans. Only vertices whose faces are all in scope are split.
    FixAllStageOutcome outcome;
    const EdgeTopology topo = buildEdgeTopology(mesh);
    const std::vector<std::uint8_t> touching = touchingEdges(mesh, topo);
    const std::vector<std::uint32_t> fans = cornerFans(mesh, topo, touching);

    const auto nv = static_cast<std::size_t>(mesh.numVertices());
    std::vector<std::uint32_t> firstFan(nv, kNoFan);
    std::vector<std::uint8_t> manyFans(nv, 0);
    std::vector<std::uint8_t> outside(nv, 0);
    for (Eigen::Index f = 0; f < mesh.numFaces(); ++f) {
        const bool in = context.scope.inScope(f);
        for (int c = 0; c < 3; ++c) {
            const auto v = static_cast<std::size_t>(mesh.F(f, c));
            outside[v] = static_cast<std::uint8_t>(outside[v] | (in ? 0 : 1));
            const std::uint32_t fan = fans[static_cast<std::size_t>(f) * 3 + static_cast<std::size_t>(c)];
            if (fan == kNoFan) {
                continue;
            }
            if (firstFan[v] == kNoFan) {
                firstFan[v] = fan;
            } else if (firstFan[v] != fan) {
                manyFans[v] = 1;
            }
        }
    }
    std::vector<std::uint8_t> target(nv, 0);
    for (std::size_t v = 0; v < nv; ++v) {
        target[v] = static_cast<std::uint8_t>(manyFans[v] != 0 && outside[v] == 0 ? 1 : 0);
    }
    outcome.targetBefore = static_cast<std::size_t>(std::count(target.begin(), target.end(), std::uint8_t{1}));
    if (outcome.targetBefore == 0) {
        return outcome;
    }
    std::size_t edgesSplit = 0;
    for (std::size_t e = 0; e < topo.numEdges(); ++e) {
        if (touching[e] != 0 && target[static_cast<std::size_t>(topo.edges[e][0])] != 0 &&
            target[static_cast<std::size_t>(topo.edges[e][1])] != 0) {
            ++edgesSplit;
        }
    }

    VertexSplit split = splitVerticesByFan(mesh, fans, target);
    const std::size_t shellsBefore = shellsLargestFirst(mesh).size();
    const std::size_t shellsAfter = shellsLargestFirst(split.mesh).size();
    outcome.changed = true;
    outcome.mesh = std::move(split.mesh);
    outcome.detail = "Split " + counted(edgesSplit, "touching edge", "touching edges") + " and " +
                     counted(split.verticesSplit, "pinched vertex", "pinched vertices") + "; shells " +
                     std::to_string(shellsBefore) + " to " + std::to_string(shellsAfter);
    return outcome;
}

// ---------------------------------------------------------------------------------------------
// Steps 4 to 6: tiny shells, fins, orientation
// ---------------------------------------------------------------------------------------------

[[nodiscard]] FixAllStageOutcome tinyShellsStage(const Mesh& mesh, const FixAllStageContext& context) {
    FixAllStageOutcome outcome;
    TinyShellRemovalResult removal = removeTinyFloatingShells(mesh, context.scope.faceInScope, context.params.tinyShellFraction,
                                                              context.params.checkParams.intersectionTolerance, context.stopToken);
    if (!removal.found.complete) {
        outcome.cancelled = true;
        return outcome;
    }
    outcome.targetBefore = removal.found.shells.size();
    if (removal.removedFaces == 0) {
        return outcome;
    }
    outcome.changed = true;
    outcome.mesh = std::move(removal.mesh);
    outcome.faceMap = std::move(removal.faceMap);
    for (const TinyFloatingShell& shell : removal.found.shells) {
        outcome.notCavityFaces.insert(outcome.notCavityFaces.end(), shell.faces.begin(), shell.faces.end());
    }
    outcome.detail = "Deleted " + counted(removal.found.shells.size(), "tiny floating shell", "tiny floating shells") +
                     " (" + (removal.found.shells.size() == 1 ? "" : "largest ") +
                     percentText(removal.found.largestFraction()) + " of the model's size)";
    return outcome;
}

[[nodiscard]] FixAllStageOutcome finsStage(const Mesh& mesh, const FixAllStageContext& context) {
    FixAllStageOutcome outcome;
    FinRemovalResult fins = removeFinFaces(mesh, context.scope.faceInScope);
    outcome.targetBefore = fins.finEdgesBefore;
    outcome.targetAfter = fins.finEdgesLeft;
    const std::string left = fins.finEdgesLeft == 0
                                 ? std::string()
                                 : counted(fins.finEdgesLeft, "Fin / other edge", "Fin / other edges") +
                                       " left: no sheet there is clearly the extra one";
    if (fins.removedFaces.empty()) {
        outcome.detail = capitalised(left);
        return outcome;
    }
    outcome.changed = true;
    outcome.mesh = std::move(fins.mesh);
    outcome.faceMap = std::move(fins.faceMap);
    std::vector<std::string> removed;
    if (fins.finFaces > 0) {
        removed.push_back(counted(fins.finFaces, "fin face", "fin faces"));
    }
    if (fins.wallFaces > 0) {
        removed.push_back(counted(fins.wallFaces, "internal-wall face", "internal-wall faces"));
    }
    outcome.detail = "Removed " + joined(removed, " and ");
    if (fins.seamEdgesOpened > 0) {
        outcome.detail += " (opened " + counted(fins.seamEdgesOpened, "seam edge", "seam edges") + ", filled below)";
    }
    if (!left.empty()) {
        outcome.detail += "; " + left;
    }
    return outcome;
}

[[nodiscard]] FixAllStageOutcome orientStage(const Mesh& mesh, const FixAllStageContext& context) {
    FixAllStageOutcome outcome;
    OrientShellsOptions options;
    options.flipFaces = true;
    options.reverseInsideOut = context.scope.kind != FixAllScopeKind::Faces; // a whole-shell flip needs whole shells
    options.intersectionTolerance = context.params.checkParams.intersectionTolerance;
    OrientShellsResult oriented = orientShells(mesh, context.scope.faceInScope, options, context.stopToken);
    if (!oriented.complete) {
        outcome.cancelled = true;
        return outcome;
    }
    const auto& shells = oriented.shells;
    std::vector<std::string> parts;
    const auto note = [&](std::string text, std::span<const int> which) {
        outcome.notes.push_back({text, mappedShells(shells, which, {})});
        parts.push_back(std::move(text));
    };
    if (oriented.flippedFaces > 0) {
        parts.push_back("flipped " + counted(oriented.flippedFaces, "face", "faces") + " in " +
                        counted(oriented.shellsWithFlippedFaces.size(), "shell", "shells"));
    }
    if (!oriented.reversedShells.empty()) {
        note("reversed " + counted(oriented.reversedShells.size(), "inside-out shell", "inside-out shells") + ": " +
                 shellList(oriented.reversedShells),
             oriented.reversedShells);
        for (const int s : oriented.reversedShells) {
            const auto& faces = shells[static_cast<std::size_t>(s)];
            outcome.notCavityFaces.insert(outcome.notCavityFaces.end(), faces.begin(), faces.end());
        }
    }
    if (!oriented.cavitiesKept.empty()) {
        note("kept " + counted(oriented.cavitiesKept.size(), "cavity", "cavities") + " inward: " +
                 shellList(oriented.cavitiesKept),
             oriented.cavitiesKept);
    }
    for (const std::vector<int>& group : oriented.stopped) {
        const std::span<const int> inner(group.data() + 1, group.size() - 1);
        note(shellList(std::span<const int>(group.data(), 1)) + " and its " +
                 (inner.size() == 1 ? "inner shell " : "inner shells ") + shellList(inner) +
                 " look inside out together: left for you (Flip all or Flip outer only, under Inverted shells)",
             group);
    }
    for (const int s : oriented.unclassified) {
        const int one[] = {s};
        note(shellList(one) + " is wound inward where a cavity and an inside-out shell look alike: left as it is", one);
    }
    for (const int s : oriented.selfCrossing) {
        const int one[] = {s};
        note(shellList(one) + " crosses itself, so inside and outside are not clear: left as it is", one);
    }
    for (const int s : oriented.nonOrientable) {
        const int one[] = {s};
        note(shellList(one) + " has a twist (like a Mobius strip), so no winding agrees everywhere: left as it is", one);
    }
    if (!oriented.outOfScope.empty()) {
        parts.push_back(counted(oriented.outOfScope.size(), "inside-out shell", "inside-out shells") +
                        " outside the scope left");
    }
    outcome.detail = capitalised(joined(parts, "; "));
    outcome.targetBefore = oriented.flippedFaces + oriented.reversedShells.size();
    if (outcome.targetBefore == 0) {
        return outcome;
    }
    outcome.changed = true;
    outcome.mesh = std::move(oriented.mesh); // identity face map
    return outcome;
}

// ---------------------------------------------------------------------------------------------
// Steps 7 to 10: cracks, slivers, holes, islands
// ---------------------------------------------------------------------------------------------

[[nodiscard]] FixAllStageOutcome closeCracksStage(const Mesh& mesh, const FixAllStageContext& context) {
    FixAllStageOutcome outcome;
    std::vector<HoleLoop> cracks = findHoleLoops(mesh, context.params.holeFill.shape);
    std::erase_if(cracks, [](const HoleLoop& hole) { return hole.shape != HoleShape::Crack || !hole.closed; });
    if (cracks.empty()) {
        return outcome;
    }
    HoleBatchOptions options;
    options.crossingPolicy = HoleCrossingPolicy::IgnoreCrossings; // a crossing it makes is the dead-end guard's
    options.checkParams = context.params.checkParams;
    options.faceInScope = context.scope.faceInScope;
    HoleBatchResult closed = fillHoles(mesh, cracks, HoleFillMethod::CloseCrack, context.params.holeFill, options,
                                       context.stopToken);
    if (context.stopToken.stop_requested()) {
        outcome.cancelled = true;
        return outcome;
    }
    const auto considered = static_cast<std::size_t>(std::count_if(
        closed.outcomes.begin(), closed.outcomes.end(),
        [](const HoleFillOutcome& hole) { return hole.skipped != HoleSkipReason::OutOfScope; }));
    // Cracks whose closing would move a vertex the scope's edge pins (or a face outside the scope)
    const auto pinnedCracks = static_cast<std::size_t>(std::count_if(
        closed.outcomes.begin(), closed.outcomes.end(), [](const HoleFillOutcome& hole) {
            return !hole.closed && std::any_of(hole.attempts.begin(), hole.attempts.end(),
                                               [](const HoleFillAttempt& a) { return a.outsideScope; });
        }));
    const std::string pinnedText =
        pinnedCracks == 0 ? std::string()
                          : counted(pinnedCracks, "crack", "cracks") +
                                (pinnedCracks == 1 ? " would move" : " would each move") +
                                " a vertex on the selection's edge: left for the hole fill";
    outcome.targetBefore = considered;
    if (!closed.success || closed.holesClosed == 0) {
        const std::size_t failed = considered - pinnedCracks;
        outcome.detail = capitalised(joined({failed > 0 ? counted(failed, "crack", "cracks") +
                                                              " could not be closed: left for the hole fill"
                                                        : std::string(),
                                             pinnedText},
                                            "; "));
        return outcome;
    }
    outcome.changed = true;
    outcome.targetAfter = considered - std::min(considered, closed.holesClosed);
    outcome.maxMove = maxMoveOfKeptFaces(mesh, closed.mesh, closed.faceMap);
    outcome.mesh = std::move(closed.mesh);
    outcome.faceMap = std::move(closed.faceMap);
    outcome.detail = "Closed " + counted(closed.holesClosed, "crack", "cracks") + " (max vertex move " +
                     formatLength(outcome.maxMove, mesh.fileUnit) + ")";
    if (outcome.targetAfter > pinnedCracks) {
        outcome.detail += "; " + std::to_string(outcome.targetAfter - pinnedCracks) + " left for the hole fill";
    }
    if (!pinnedText.empty()) {
        outcome.detail += "; " + pinnedText;
    }
    return outcome;
}

/// Thin faces of `mesh` that are in `scope`.
[[nodiscard]] std::vector<Eigen::Index> thinFacesInScope(const Mesh& mesh, const FixAllScope& scope, const CheckParams& params,
                                                         std::stop_token stopToken, bool& cancelled) {
    const CategoryResult thin = runCategoryCheck(DefectCategory::ThinFaces, mesh, params, stopToken);
    cancelled = thin.cancelled;
    std::vector<Eigen::Index> faces;
    for (const DefectElement& element : thin.elements) {
        if (scope.inScope(element.index)) {
            faces.push_back(element.index);
        }
    }
    return faces;
}

[[nodiscard]] FixAllStageOutcome sliversStage(const Mesh& mesh, const FixAllStageContext& context) {
    FixAllStageOutcome outcome;
    // The healer reads the whole mesh through CGAL; duplicates or collapsed faces would make it read a
    // polygon soup and tear them apart. Steps 1 and 2 remove those in scope.
    if (runCategoryCheck(DefectCategory::Duplicates, mesh, context.params.checkParams).size() > 0) {
        outcome.skipped = "The mesh still has duplicate faces (outside the scope), which the healer cannot read "
                          "without tearing them apart.";
        return outcome;
    }
    if (repeatsVertex(mesh)) {
        outcome.skipped = "The mesh still has faces that repeat a vertex (outside the scope), which the healer cannot read.";
        return outcome;
    }
    bool cancelled = false;
    std::vector<Eigen::Index> faces =
        thinFacesInScope(mesh, context.scope, context.params.checkParams, context.stopToken, cancelled);
    if (cancelled) {
        outcome.cancelled = true;
        return outcome;
    }
    // Phase 3: only the thin faces the merge made (the union's cut lines), by geometry
    const auto madeByMerge = [&](const Mesh& now, std::vector<Eigen::Index>& list) {
        if (context.preMerge == nullptr) {
            return;
        }
        std::vector<std::uint8_t> added(static_cast<std::size_t>(now.numFaces()), 0);
        for (const Eigen::Index f : faceDiff(*context.preMerge, now).added) {
            added[static_cast<std::size_t>(f)] = 1;
        }
        std::erase_if(list, [&](Eigen::Index f) { return added[static_cast<std::size_t>(f)] == 0; });
    };
    madeByMerge(mesh, faces);
    const char* one = context.preMerge != nullptr ? "cut sliver" : "thin face";
    const char* many = context.preMerge != nullptr ? "cut slivers" : "thin faces";
    outcome.targetBefore = faces.size();
    if (faces.empty()) {
        return outcome;
    }
    SliverHealerSettings settings;
    settings.needleEdgeRatio = context.params.checkParams.needleEdgeRatio;
    settings.capAngleDeg = context.params.checkParams.capAngleDeg;
    SliverHealOptions options;
    options.stopToken = context.stopToken;
    options.pinnedVertices = context.scope.pinned;
    RepairResults healed =
        SliverHealer::healFaces(mesh, faces, context.params.checkParams.thinThresholds, settings, options);
    if (healed.cancelled) {
        outcome.cancelled = true;
        return outcome;
    }
    if (!healed.success) {
        outcome.detail = "None of the " + counted(faces.size(), one, many) + " could be healed safely";
        return outcome;
    }
    if (healed.faceMap.empty()) {
        outcome.failure = "The healer had to read the mesh as a polygon soup, which loses the face numbers.";
        return outcome;
    }
    FixAllScope carried = context.scope;
    carried.carry(healed.faceMap, healed.mesh);
    std::vector<Eigen::Index> left =
        thinFacesInScope(healed.mesh, carried, context.params.checkParams, context.stopToken, cancelled);
    if (cancelled) {
        outcome.cancelled = true;
        return outcome;
    }
    madeByMerge(healed.mesh, left);
    for (std::size_t v = 0; v < healed.vertexMap.size(); ++v) {
        const Eigen::Index to = healed.vertexMap[v];
        if (to >= 0) {
            outcome.maxMove =
                std::max(outcome.maxMove, (healed.mesh.V.row(to) - mesh.V.row(static_cast<Eigen::Index>(v))).norm());
        }
    }
    outcome.changed = true;
    outcome.targetAfter = left.size();
    outcome.mesh = std::move(healed.mesh);
    outcome.faceMap = std::move(healed.faceMap);
    const std::size_t fixed = faces.size() - std::min(faces.size(), left.size());
    outcome.detail = "Healed " + std::to_string(fixed) + " of " + counted(faces.size(), one, many) + " (" +
                     counted(static_cast<std::size_t>(healed.edgeFlips), "flip", "flips") + ", " +
                     counted(static_cast<std::size_t>(healed.edgeCollapses), "collapse", "collapses") + ")";
    return outcome;
}

[[nodiscard]] std::string skippedHoleText(std::size_t hole, HoleSkipReason reason, int maxEdges) {
    const std::string name = "hole " + std::to_string(hole + 1);
    switch (reason) {
    case HoleSkipReason::OpenChain:
        return name + " is an open chain and cannot be filled";
    case HoleSkipReason::TooBig:
        return name + " has more than " + std::to_string(maxEdges) + " edges (Holes settings)";
    case HoleSkipReason::NoMethod:
        return "no fill applies to " + name;
    case HoleSkipReason::OutOfScope:
    case HoleSkipReason::None:
        break;
    }
    return {};
}

[[nodiscard]] FixAllStageOutcome fillHolesStage(const Mesh& mesh, const FixAllStageContext& context) {
    FixAllStageOutcome outcome;
    const std::vector<HoleLoop> holes = findHoleLoops(mesh, context.params.holeFill.shape);
    if (holes.empty()) {
        return outcome;
    }
    HoleBatchOptions options;
    options.crossingPolicy = HoleCrossingPolicy::AlwaysFillFewestCrossings; // a hole is always filled
    options.checkParams = context.params.checkParams;
    options.faceInScope = context.scope.faceInScope;
    HoleBatchResult filled = fillHoles(mesh, holes, HoleFillMethod::None, context.params.holeFill, options, context.stopToken);
    if (context.stopToken.stop_requested()) {
        outcome.cancelled = true;
        return outcome;
    }

    std::size_t considered = 0;
    std::vector<std::string> crossing;
    std::vector<std::string> skipped;
    std::size_t ownCrossings = 0;
    std::size_t otherCrossings = 0;
    const bool merge = mergeRuns(context);
    const std::string mergeNote = merge ? std::string("left for the merge")
                                        : context.params.mergeShells ? std::string("reported (the merge needs whole shells)")
                                                                     : std::string("reported (the merge is off)");
    const std::vector<std::vector<Eigen::Index>> shells = shellsLargestFirst(mesh);
    for (const HoleFillOutcome& hole : filled.outcomes) {
        if (hole.skipped == HoleSkipReason::OutOfScope) {
            continue;
        }
        ++considered;
        const std::string name = "hole " + std::to_string(hole.hole + 1);
        if (hole.skipped != HoleSkipReason::None) {
            skipped.push_back(skippedHoleText(hole.hole, hole.skipped, context.params.holeFill.maxHoleEdges));
            continue;
        }
        if (!hole.closed) {
            skipped.push_back(name + " could not be filled");
            continue;
        }
        ownCrossings += hole.crossingsOwnShell;
        otherCrossings += hole.crossingsOtherShells;
        if (hole.crossingsOtherShells > 0) {
            std::vector<int> named{hole.shell};
            named.insert(named.end(), hole.otherShells.begin(), hole.otherShells.end());
            std::string text = name + ": its patch crosses " + shellList(hole.otherShells) + ", " + mergeNote;
            outcome.notes.push_back({"#" + std::to_string(hole.shell + 1) + "'s " + text,
                                     mappedShells(shells, named, filled.faceMap)});
            crossing.push_back(std::move(text));
        } else if (hole.crossingsOwnShell > 0) {
            crossing.push_back(name + ": every fill crosses its shell, " + holeFillMethodLabel(hole.method) +
                               " used, left for the island step");
        }
    }
    outcome.targetBefore = considered;
    const std::string skippedText = joined(skipped, "; ");
    if (!filled.success || filled.holesClosed == 0) {
        outcome.detail = capitalised(skippedText);
        outcome.notes.clear();
        return outcome;
    }
    outcome.changed = true;
    outcome.targetAfter = considered - std::min(considered, filled.holesClosed);
    outcome.mesh = std::move(filled.mesh);
    outcome.faceMap = std::move(filled.faceMap);
    outcome.detail = "Filled " + std::to_string(filled.holesClosed) + " of " + counted(considered, "hole", "holes");
    if (!crossing.empty()) {
        outcome.detail += " (" + joined(crossing, "; ") + ")";
    }
    if (!skippedText.empty()) {
        outcome.detail += "; " + skippedText;
    }
    if (ownCrossings > 0) {
        outcome.deferred.push_back({FixAllDeferredKind::CrossingsWithinShell, ownCrossings,
                                    counted(ownCrossings, "crossing", "crossings") + " within the shell: left for the island step"});
    }
    if (otherCrossings > 0) {
        outcome.deferred.push_back({FixAllDeferredKind::CrossingsWithShell, otherCrossings,
                                    counted(otherCrossings, "crossing", "crossings") + " with other shells: " + mergeNote});
    }
    return outcome;
}

/// Faces of `faces` plus `rings` rings of faces around them (sharing a vertex).
[[nodiscard]] std::vector<Eigen::Index> withRings(const Mesh& mesh, const std::vector<std::vector<Eigen::Index>>& facesOfVertex,
                                                 std::vector<Eigen::Index> faces, int rings) {
    std::vector<std::uint8_t> in(static_cast<std::size_t>(mesh.numFaces()), 0);
    for (const Eigen::Index f : faces) {
        in[static_cast<std::size_t>(f)] = 1;
    }
    std::size_t front = 0;
    for (int ring = 0; ring < rings; ++ring) {
        const std::size_t end = faces.size();
        for (std::size_t i = front; i < end; ++i) {
            for (int c = 0; c < 3; ++c) {
                for (const Eigen::Index g : facesOfVertex[static_cast<std::size_t>(mesh.F(faces[i], c))]) {
                    if (in[static_cast<std::size_t>(g)] == 0) {
                        in[static_cast<std::size_t>(g)] = 1;
                        faces.push_back(g);
                    }
                }
            }
        }
        front = end;
    }
    return faces;
}

/// Islands of crossing faces within shells, made of faces in `scope`.
[[nodiscard]] std::vector<IntersectionIsland> islandsInScope(const Mesh& mesh, const FixAllScope& scope,
                                                             const CheckParams& params, std::stop_token stopToken,
                                                             bool& cancelled) {
    const CategoryResult check = runCategoryCheck(DefectCategory::Intersections, mesh, params, std::move(stopToken));
    cancelled = check.cancelled;
    if (cancelled) {
        return {};
    }
    return clusterIntersectionIslands(mesh, check, shellIdsOfFaces(mesh), scope.faceInScope);
}

[[nodiscard]] std::string islandName(const IntersectionIsland& island) {
    return "island at face #" + std::to_string(*std::min_element(island.faces.begin(), island.faces.end()));
}

/// The island step on part of the mesh: an island whose cut (the island plus its rings) would leave the
/// scope is skipped and reported; so is one whose repair would change a face outside the scope anyway
/// (the region-growing and self-union fallbacks reach further than the rings).
[[nodiscard]] FixAllStageOutcome scopedIslandsStage(const Mesh& mesh, const FixAllStageContext& context) {
    FixAllStageOutcome outcome;
    const CheckParams& params = context.params.checkParams;
    const FixAllScope& scope = context.scope;
    bool cancelled = false;
    const std::vector<IntersectionIsland> islands = islandsInScope(mesh, scope, params, context.stopToken, cancelled);
    if (cancelled || context.stopToken.stop_requested()) {
        outcome.cancelled = true;
        return outcome;
    }
    outcome.targetBefore = islands.size();
    if (islands.empty()) {
        return outcome;
    }

    std::vector<std::vector<Eigen::Index>> facesOfVertex(static_cast<std::size_t>(mesh.numVertices()));
    for (Eigen::Index f = 0; f < mesh.numFaces(); ++f) {
        for (int c = 0; c < 3; ++c) {
            facesOfVertex[static_cast<std::size_t>(mesh.F(f, c))].push_back(f);
        }
    }
    std::vector<std::string> skipped;
    std::vector<const IntersectionIsland*> eligible;
    std::vector<Eigen::Index> seeds;
    const int rings = std::max(context.params.islandRepair.dilationRings, 0);
    for (const IntersectionIsland& island : islands) {
        const std::vector<Eigen::Index> cut = withRings(mesh, facesOfVertex, island.faces, rings);
        if (std::all_of(cut.begin(), cut.end(), [&](Eigen::Index f) { return scope.inScope(f); })) {
            eligible.push_back(&island);
            seeds.insert(seeds.end(), island.faces.begin(), island.faces.end());
        } else {
            skipped.push_back(islandName(island) + " extends beyond the selection");
        }
    }

    Mesh current = mesh;
    std::size_t notRepaired = 0; // eligible islands the healer could not repair
    if (!eligible.empty()) {
        IslandBatchResult healed = healIntersectionIslands(mesh, params, scope.faceInScope, seeds,
                                                           context.params.islandRepair, context.stopToken);
        if (context.stopToken.stop_requested()) {
            outcome.cancelled = true;
            return outcome;
        }
        if (!healed.success) {
            notRepaired = eligible.size();
        } else if (fixall_detail::outsideScopeKept(mesh, healed.mesh, fixall_detail::faceMapByGeometry(mesh, healed.mesh), scope)) {
            current = std::move(healed.mesh);
        } else {
            // Island by island, keeping each repair that leaves the faces outside the scope alone
            FixAllScope currentScope = scope;
            for (const IntersectionIsland* island : eligible) {
                std::unordered_map<FaceGeometryKey, Eigen::Index, FaceGeometryKeyHash> rowOf;
                for (Eigen::Index f = 0; f < current.numFaces(); ++f) {
                    rowOf.emplace(faceGeometryKey(current, f), f);
                }
                std::vector<Eigen::Index> faces;
                for (const Eigen::Index f : island->faces) {
                    if (const auto it = rowOf.find(faceGeometryKey(mesh, f)); it != rowOf.end()) {
                        faces.push_back(it->second);
                    }
                }
                if (faces.empty()) {
                    continue; // repaired with an earlier island
                }
                IslandBatchResult one = healIntersectionIslands(current, params, currentScope.faceInScope, faces,
                                                                context.params.islandRepair, context.stopToken);
                if (context.stopToken.stop_requested()) {
                    outcome.cancelled = true;
                    return outcome;
                }
                if (!one.success) {
                    continue;
                }
                const std::vector<Eigen::Index> map = fixall_detail::faceMapByGeometry(current, one.mesh);
                if (!fixall_detail::outsideScopeKept(current, one.mesh, map, currentScope)) {
                    skipped.push_back(islandName(*island) + ": its repair would change faces outside the selection");
                    continue;
                }
                currentScope.carry(map, one.mesh);
                current = std::move(one.mesh);
            }
        }
    }

    const std::string skippedText = joined(skipped, "; ");
    const std::vector<Eigen::Index> map = fixall_detail::faceMapByGeometry(mesh, current);
    if (current.numFaces() == mesh.numFaces() && std::find(map.begin(), map.end(), Eigen::Index{-1}) == map.end()) {
        const std::string none = notRepaired == 0 ? std::string()
                                                  : "no island could be repaired (" +
                                                        counted(notRepaired, "island", "islands") + " left)";
        outcome.detail = capitalised(joined({none, skippedText}, "; "));
        return outcome;
    }
    FixAllScope carried = scope;
    carried.carry(map, current);
    const std::vector<IntersectionIsland> left = islandsInScope(current, carried, params, context.stopToken, cancelled);
    if (cancelled || context.stopToken.stop_requested()) {
        outcome.cancelled = true;
        return outcome;
    }
    outcome.changed = true;
    outcome.targetAfter = left.size();
    outcome.faceMap = map;
    outcome.mesh = std::move(current);
    const std::size_t repaired = islands.size() - std::min(islands.size(), left.size());
    outcome.detail = "Repaired " + std::to_string(repaired) + " of " + counted(islands.size(), "island", "islands");
    if (!skippedText.empty()) {
        outcome.detail += "; " + skippedText;
    }
    return outcome;
}

[[nodiscard]] FixAllStageOutcome islandsStage(const Mesh& mesh, const FixAllStageContext& context) {
    if (!context.scope.everyFace()) {
        return scopedIslandsStage(mesh, context);
    }
    FixAllStageOutcome outcome;
    IslandBatchResult healed = healIntersectionIslands(mesh, context.params.checkParams, context.scope.faceInScope, {},
                                                       context.params.islandRepair, context.stopToken);
    if (context.stopToken.stop_requested()) {
        outcome.cancelled = true;
        return outcome;
    }
    outcome.targetBefore = healed.islandsBefore;
    if (!healed.success) {
        if (healed.islandsBefore > 0) {
            outcome.detail = "No island could be repaired (" + counted(healed.islandsBefore, "island", "islands") + " left)";
        }
        return outcome;
    }
    outcome.changed = true;
    outcome.targetAfter = healed.islandsAfter;
    outcome.faceMap = fixall_detail::faceMapByGeometry(mesh, healed.mesh); // the healer re-orders shells
    outcome.mesh = std::move(healed.mesh);
    const std::size_t repaired = healed.islandsBefore - std::min(healed.islandsBefore, healed.islandsAfter);
    outcome.detail = "Repaired " + std::to_string(repaired) + " of " + counted(healed.islandsBefore, "island", "islands");
    if (healed.islandsAfter > 0) {
        outcome.detail += "; " + std::to_string(healed.islandsAfter) + " left";
    }
    return outcome;
}

// ---------------------------------------------------------------------------------------------
// Skip rules
// ---------------------------------------------------------------------------------------------

constexpr const char* kTurnedOff = "Turned off in the options.";

[[nodiscard]] std::string weldSkipReason(const FixAllParams& params, const FixAllScope&) {
    return params.weld ? std::string() : std::string(kTurnedOff);
}

[[nodiscard]] std::string tinyShellsSkipReason(const FixAllParams& params, const FixAllScope& scope) {
    if (!params.removeTinyShells) {
        return kTurnedOff;
    }
    return scope.kind == FixAllScopeKind::Faces ? std::string("Needs whole shells: select shells, or clear the selection.")
                                                : std::string();
}

[[nodiscard]] std::string finsSkipReason(const FixAllParams& params, const FixAllScope&) {
    return params.removeFins ? std::string() : std::string(kTurnedOff);
}

[[nodiscard]] std::string cracksSkipReason(const FixAllParams& params, const FixAllScope&) {
    return params.closeCracks ? std::string() : std::string(kTurnedOff);
}

[[nodiscard]] std::string sliversSkipReason(const FixAllParams& params, const FixAllScope&) {
    return params.healThinFaces ? std::string() : std::string(kTurnedOff);
}

} // namespace

std::vector<FixAllStage> fixAllRepairStages() {
    constexpr double quick = settings::kFixAllQuickStageProgressWeight;
    constexpr double heal = settings::kFixAllHealStageProgressWeight;
    std::vector<FixAllStage> stages;
    stages.reserve(12);
    stages.push_back({FixAllStageId::Weld, "Weld by distance", "welding", quick, weldSkipReason, weldStage, true});
    stages.push_back({FixAllStageId::DropRepeated, "Remove collapsed faces", "removing collapsed faces", quick, {},
                      dropRepeatedStage, false});
    stages.push_back({FixAllStageId::Dedupe, "Remove duplicate faces", "removing duplicate faces", quick, {}, dedupeStage, false});
    stages.push_back({FixAllStageId::SplitTouching, "Split touching contacts", "splitting touching contacts", quick, {},
                      splitTouchingStage, false});
    stages.push_back({FixAllStageId::TinyShells, "Remove tiny floating shells", "removing tiny floating shells", quick,
                      tinyShellsSkipReason, tinyShellsStage, false});
    stages.push_back({FixAllStageId::Fins, "Remove fins and internal walls", "removing fins", quick, finsSkipReason,
                      finsStage, false});
    stages.push_back({FixAllStageId::Orient, "Orient", "orienting", quick, {}, orientStage, false});
    stages.push_back({FixAllStageId::CloseCracks, "Close cracks", "closing cracks", quick, cracksSkipReason,
                      closeCracksStage, true});
    stages.push_back({FixAllStageId::Slivers, "Heal thin faces", "healing thin faces", heal, sliversSkipReason,
                      sliversStage, true});
    stages.push_back({FixAllStageId::FillHoles, "Fill holes", "filling holes", heal, {}, fillHolesStage, false});
    stages.push_back({FixAllStageId::Islands, "Patch intersection islands", "patching intersection islands", heal, {},
                      islandsStage, true});
    stages.push_back({FixAllStageId::OrientAgain, "Orient again", "orienting again", heal, {}, orientStage, false});
    return stages;
}

std::vector<FixAllStage> fixAllCleanupStages() {
    // Progress weights are not used: phase 3 shares its own part of the pass
    constexpr double quick = settings::kFixAllQuickStageProgressWeight;
    constexpr double heal = settings::kFixAllHealStageProgressWeight;
    std::vector<FixAllStage> stages;
    stages.reserve(7);
    stages.push_back({FixAllStageId::DropRepeated, "Remove collapsed faces", "removing collapsed faces", quick, {},
                      dropRepeatedStage, false});
    stages.push_back({FixAllStageId::Dedupe, "Remove duplicate faces", "removing duplicate faces", quick, {}, dedupeStage, false});
    stages.push_back({FixAllStageId::SplitTouching, "Split touching contacts", "splitting touching contacts", quick, {},
                      splitTouchingStage, false});
    stages.push_back({FixAllStageId::Orient, "Orient", "orienting", quick, {}, orientStage, false});
    stages.push_back({FixAllStageId::Slivers, "Heal cut slivers", "healing cut slivers", heal, sliversSkipReason,
                      sliversStage, true});
    stages.push_back({FixAllStageId::FillHoles, "Fill holes", "filling holes", heal, {}, fillHolesStage, false});
    stages.push_back({FixAllStageId::Islands, "Patch intersection islands", "patching intersection islands", heal, {},
                      islandsStage, true});
    return stages;
}

} // namespace ogun
