// =============================================================================
//  BridgeOrchestration.cpp
//  -----------------------
//  Runtime state machine that drives the combined bridge from XML parse
//  completion to final module-origin restoration:
//
//      Phase 0   startBridgeOrchestration
//                  ↓
//      FORWARD   processBridgeTask          (per-task state setup)
//                runForwardRound            (R1: built?  R2: dispatch one)
//                handleForwardMotionEnd     (step / arrival / reroute)
//                onForwardRoundComplete     (record placements, next round)
//                verifyBridge               (post-fill flow check)
//                  ↓
//      RETURN    startReturnPhase           (filter still-displaced modules)
//                executeReturnIteration     (one BFS return at a time)
//                handleReturnMotionEnd      (advance / reroute / arrive)
//                verifyAllOriginsRestored
//                  ↓
//      IDLE      clearBridgeTaskState
//                advanceToNextTask
// =============================================================================

#include "Catoms3DFlowBridgeCode.h"
#include "FlowGraph.h"
#include "FlowGraphViz.h"
#include "robots/catoms3D/catoms3DRotationEvents.h"
#include "events/scheduler.h"

#include <algorithm>
#include <climits>
#include <exception>
#include <iostream>
#include <numeric>
#include <queue>
#include <set>

using namespace std;
using namespace Catoms3D;

// =============================================================================
//  File-local helper: non-interfering path selection
//
//  Greedy selection of the largest non-interfering subset from candidate
//  paths.  Criteria:
//    (a) Distinct destination per path.
//    (b) Distinct start (idle module) per path.
//    (c) No two selected paths share any position along their full length.
//    (d) No selected destination is a lattice neighbour of another selected
//        destination OR of any position in occupiedNeighbors.
//
//  Paths are processed in ascending length order.  Returns a vector of
//  (start, dest) pairs for the accepted paths.
// =============================================================================

static vector<pair<Cell3DPosition,Cell3DPosition>>
findNonInterferingAssignment(const vector<vector<Cell3DPosition>>& paths,
                             const set<Cell3DPosition>& occupiedNeighbors) {

    vector<size_t> order(paths.size());
    iota(order.begin(), order.end(), 0);
    sort(order.begin(), order.end(), [&](size_t a, size_t b) {
        return paths[a].size() < paths[b].size();
    });

    auto* lattice = BaseSimulator::getWorld()->lattice;
    set<Cell3DPosition> usedPositions;
    set<Cell3DPosition> usedStarts;
    set<Cell3DPosition> usedDests;
    vector<pair<Cell3DPosition,Cell3DPosition>> assignment;

    for (size_t i : order) {
        const auto& path = paths[i];
        if (path.size() < 2) continue;

        Cell3DPosition start = path.front();
        Cell3DPosition dest  = path.back();

        if (usedStarts.count(start)) continue;  // (b)
        if (usedDests.count(dest))   continue;  // (a)

        // (c) no shared position
        bool posConflict = false;
        for (const auto& pos : path) {
            if (usedPositions.count(pos)) { posConflict = true; break; }
        }
        if (posConflict) continue;

        // (d) dest not adjacent to any selected dest or to any occupiedNeighbor
        bool neighborConflict = false;
        for (const auto& nb : lattice->getNeighborhood(dest)) {
            if (usedDests.count(nb) || occupiedNeighbors.count(nb)) {
                neighborConflict = true; break;
            }
        }
        if (neighborConflict) continue;

        // Accept
        usedStarts.insert(start);
        usedDests.insert(dest);
        for (const auto& pos : path) usedPositions.insert(pos);
        assignment.push_back({start, dest});
    }

    return assignment;
}

// =============================================================================
//  Phase 0 — startBridgeOrchestration
//
//  Push the single combined BridgeTask and launch it.  Called once from
//  startup().  A single BridgeTask spanning the combined bridge is built; its
//  intermediatePath is the deduplicated union of all augmenting-path
//  intermediates produced by findCombinedBridge() — i.e. every empty cell on
//  which an idle module must be placed to bypass the entire min-cut at once.
//
//  The minCutEdge field carries a representative (first u, first v) for
//  diagnostic logging only; the bridge itself spans all merged min-cut
//  endpoints.
// =============================================================================

void Catoms3DFlowBridgeCode::startBridgeOrchestration() {
    cout << "\n=== [Phase 0] Building combined bridge task ===\n";

    while (!pendingBridges.empty()) pendingBridges.pop();

    if (combinedBridgePositions.empty()) {
        cout << "Combined bridge is empty (no min-cut edges or no empty-space paths). "
                "Halting.\n";
        return;
    }

    Cell3DPosition repU = allMinCutEdges.empty() ? Cell3DPosition{}
                                                  : allMinCutEdges.front().first;
    Cell3DPosition repV = allMinCutEdges.empty() ? Cell3DPosition{}
                                                  : allMinCutEdges.front().second;
    pendingBridges.push({ {repU, repV}, combinedBridgePositions, 0, 0 });

    cout << "1 combined bridge task queued (" << combinedBridgePositions.size()
         << " bridge cell(s) across " << allMinCutEdges.size()
         << " min-cut arc(s)).\n";

    BridgeTask first = pendingBridges.front();
    pendingBridges.pop();
    processBridgeTask(first);
}

// =============================================================================
//  processBridgeTask — initialise per-task state and start forward round 0
// =============================================================================

void Catoms3DFlowBridgeCode::processBridgeTask(const BridgeTask& task) {
    currentBridgeTask = task;

    cout << "\n=== [BridgeTask] Combined bridge across "
         << allMinCutEdges.size() << " min-cut arc(s), "
         << task.intermediatePath.size() << " bridge cell(s) ===\n";
    cout << "Intermediates:";
    for (const auto& p : task.intermediatePath) cout << " " << p.to_string();
    cout << "\n";

    // -------- Clear all per-task state --------
    travelLog.clear();
    roundAssignments.clear();
    placedIntermediates.clear();
    pendingMotions.clear();
    pendingArrival.clear();
    forwardStepIndex.clear();
    forwardVisited.clear();
    returnTargets.clear();
    pendingOrigins.clear();
    activeReturnStep        = 0;
    currentForwardRound     = 0;
    currentPhase            = BridgePhase::FORWARD;

    // -------- Build activity-diagram donor-selection state --------
    computeBridgeSelectionState();

    // -------- Start first forward round --------
    runForwardRound();
}

// =============================================================================
//  runForwardRound — stage walker over the SelectionStage state machine.
//
//  Each iteration:
//    R1. If every combined-bridge cell is occupied -> bridge built; verify
//        and proceed to the return phase.
//    R2. If the SelectionStage state machine has reached SEL_DONE without
//        filling every bridge cell -> exit cleanly with an unfilled-cells
//        warning.
//    R3. Otherwise, hand control to planAndDispatchUnderCurrentStage(),
//        which iterates max-flow + teleport INTERNALLY until the stage's
//        donor pool is drained, the bridge is full, or max-flow stops
//        producing paths on the current configuration.  Then advance the
//        SelectionStage and continue.
//
//  Because teleport is synchronous, the entire forward phase runs in a
//  single call to runForwardRound — no motion-end event drives subsequent
//  rounds.  currentForwardRound is bumped once per stage for logging
//  continuity with the old (motion-based) output format.
// =============================================================================

void Catoms3DFlowBridgeCode::runForwardRound() {
    auto* lattice = BaseSimulator::getWorld()->lattice;

    while (true) {
        // -------- R1: bridge built? --------
        size_t remaining = 0;
        for (const auto& pos : currentBridgeTask.intermediatePath)
            if (!lattice->getBlock(pos)) ++remaining;

        if (remaining == 0) {
            cout << "[Bridge] All " << currentBridgeTask.intermediatePath.size()
                 << " combined bridge cell(s) filled.\n";
            verifyBridge();
            startReturnPhase();
            return;
        }

        // -------- R2: all selection stages exhausted? --------
        if (currentSelectionStage == SelectionStage::SEL_DONE) {
            cout << "[Bridge] Selection state machine exhausted; "
                 << remaining << " bridge cell(s) remain unfilled.\n";
            verifyBridge();
            startReturnPhase();
            return;
        }

        cout << "\n=== [Bridge Round " << (currentForwardRound + 1) << "] "
             << remaining << " / " << currentBridgeTask.intermediatePath.size()
             << " combined bridge cell(s) remaining ===\n";

        // -------- R3: fully drain the current stage. --------
        // planAndDispatchUnderCurrentStage iterates max-flow + teleport until
        // the pool is empty, the bridge is full, or max-flow returns 0 paths.
        // Whether it teleported anything or not, the stage is done after this
        // call — advance the state machine and let the outer loop re-check
        // R1 / R2 with the next pool.
        planAndDispatchUnderCurrentStage();
        currentForwardRound++;
        advanceSelectionStage();
    }
    // unreachable: the loop exits via the return statements in R1 / R2.
}

// =============================================================================
//  onForwardRoundComplete — all modules of the round have arrived or been
//  declared stuck.  Record newly placed bridge cells, then start the next
//  round.
// =============================================================================

void Catoms3DFlowBridgeCode::onForwardRoundComplete() {
    auto* lattice = BaseSimulator::getWorld()->lattice;

    // Scan returnTargets: any entry whose current position is a bridge cell is
    // a successfully placed module — record it in placedIntermediates.
    set<Cell3DPosition> bridgeCells(currentBridgeTask.intermediatePath.begin(),
                                    currentBridgeTask.intermediatePath.end());
    for (const auto& [curPos, orig] : returnTargets) {
        if (bridgeCells.count(curPos) && !placedIntermediates.count(curPos)) {
            placedIntermediates.insert(curPos);
            cout << "  [PLACED] Bridge cell " << curPos.to_string()
                 << " filled by module from " << orig.to_string() << ".\n";
        }
    }

    currentForwardRound++;
    runForwardRound();
}

// =============================================================================
//  verifyBridge — post-fill flow check (called when the forward loop exits)
// =============================================================================

void Catoms3DFlowBridgeCode::verifyBridge() {
    cout << "\n=== [Bridge Verification] ===\n";
    auto* lattice = BaseSimulator::getWorld()->lattice;

    // Log the combined min-cut edges for diagnostics.
    cout << "  Combined min-cut arcs (" << allMinCutEdges.size() << "):\n";
    for (size_t i = 0; i < allMinCutEdges.size(); ++i)
        cout << "    [" << (i+1) << "] "
             << allMinCutEdges[i].first.to_string() << " -> "
             << allMinCutEdges[i].second.to_string() << "\n";

    for (const auto& pos : currentBridgeTask.intermediatePath) {
        if (lattice->getBlock(pos))
            cout << "  [OK]      " << pos.to_string() << " occupied.\n";
        else
            cerr << "  [MISSING] " << pos.to_string() << " NOT occupied.\n";
    }

    cout << "  Re-running max-flow (baseline = " << originalFlowValue << ")...\n";
    FlowGraph* fg = new FlowGraph();
    fg->buildFlowGraph();
    auto __viz_origCap = fg->backupCapacities();
    int newFlow = fg->findAndPrintAugmentingPaths(fg->superSource, fg->superSink);

    // [viz] Stage 6 — main flow graph after the bridge is in place.  Bridge
    // cells now act as pivots; the augmenting paths reflect the improved flow.
    {
        FlowGraphViz::ModuleSets sets;
        sets.sourceModules    = &movingBlocks;
        sets.targetCells      = &targetPositions;
        sets.structuralBlocks = &structuralBlocks;
        sets.bridgeFilled     = &currentBridgeTask.intermediatePath;
        std::string ttl = "Post-bridge main flow graph (flow=" +
                          std::to_string(newFlow) + " vs baseline=" +
                          std::to_string(originalFlowValue) + ")";
        FlowGraphViz::dump(*fg, "6_post_bridge", ttl, sets,
                            &__viz_origCap, &fg->augmentingPathPositions);
    }

    delete fg;

    if (newFlow > originalFlowValue)
        cout << "  Flow increased: " << originalFlowValue << " → " << newFlow << ". Bridge successful!\n";
    else if (newFlow == originalFlowValue)
        cout << "  Flow unchanged at " << newFlow << ".\n";
    else
        cerr << "  Flow decreased: " << originalFlowValue << " → " << newFlow << ". Unexpected!\n";

    int numMinCutEdges = (int)Catoms3DFlowBridgeCode::allMinCutEdges.size();
    cout << "  Original min-cut edge count (diagnostic): " << numMinCutEdges << ".\n";
    int expectedFlow = originalFlowValue + numMinCutEdges;
    if (newFlow >= expectedFlow)
        cout << "  [SUCCESS] New flow " << newFlow << " >= expected " << expectedFlow
             << " (baseline + " << numMinCutEdges << " min-cut edges).\n";
    else
        cout << "  [INFO] New flow " << newFlow << " < expected " << expectedFlow
             << ". Some min-cut edges may not yet be bypassed by placed intermediates.\n";
}

// =============================================================================
//  startReturnPhase — transition from forward to return phase.
//
//  Modules that successfully reached a bridge cell stay there.  All other
//  dispatched modules (stuck mid-path) are returned to their origins.
// =============================================================================

void Catoms3DFlowBridgeCode::startReturnPhase() {
    // Keep only modules that did NOT reach a bridge position — those must
    // return.  Modules that successfully reached a bridge cell are already in
    // placedIntermediates and must not be moved.
    set<Cell3DPosition> bridgeCells(currentBridgeTask.intermediatePath.begin(),
                                    currentBridgeTask.intermediatePath.end());
    map<Cell3DPosition, Cell3DPosition> toReturn;
    for (const auto& [curPos, orig] : returnTargets)
        if (!bridgeCells.count(curPos))
            toReturn[curPos] = orig;
    returnTargets = toReturn;

    if (returnTargets.empty()) {
        cout << "\n[Return Phase] No displaced modules to return. Advancing.\n";
        clearBridgeTaskState();
        advanceToNextTask();
        return;
    }

    cout << "\n=== [Return Phase] Combined bridge ===\n";
    cout << "  " << returnTargets.size() << " displaced module(s) to return.\n";

    pendingOrigins.clear();
    for (const auto& [curPos, orig] : returnTargets)
        pendingOrigins.insert(orig);

    currentPhase = BridgePhase::RETURN;
    executeReturnIteration();
}

// =============================================================================
//  executeReturnIteration — select next module, schedule its first return step
// =============================================================================

void Catoms3DFlowBridgeCode::executeReturnIteration() {
    if (pendingOrigins.empty()) {
        verifyAllOriginsRestored();
        return;
    }

    auto* lattice = BaseSimulator::getWorld()->lattice;

    cout << "\n  [RETURN ITER] " << pendingOrigins.size() << " origin(s) remaining.\n";

    // -------------------------------------------------------------------------
    //  Step Ret.1 — select module with shortest BFS path back to its origin.
    //  Use findPathForModule so the first step is validated against the actual
    //  module.
    // -------------------------------------------------------------------------
    Cell3DPosition bestOrigin, bestModulePos;
    vector<Cell3DPosition> bestPath;
    int bestLen = INT_MAX;

    for (const auto& [currentPos, origin] : returnTargets) {
        if (!pendingOrigins.count(origin)) continue;
        Catoms3DBlock* candMod = static_cast<Catoms3DBlock*>(lattice->getBlock(currentPos));
        if (!candMod) continue;
        vector<Cell3DPosition> path = findPathForModule(candMod, currentPos, origin);
        if (!path.empty() && (int)path.size() < bestLen) {
            bestLen       = (int)path.size();
            bestOrigin    = origin;
            bestModulePos = currentPos;
            bestPath      = path;
        }
    }

    if (bestPath.empty()) {
        cout << "  [RETURN WARNING] No path to any pending origin. Skipping remaining.\n";
        for (const auto& origin : pendingOrigins)
            cout << "  [SKIPPED ORIGIN] " << origin.to_string() << "\n";
        pendingOrigins.clear();
        verifyAllOriginsRestored();
        return;
    }

    // -------------------------------------------------------------------------
    //  Step Ret.2 — schedule first step of selected module.
    // -------------------------------------------------------------------------
    Catoms3DBlock* mod = static_cast<Catoms3DBlock*>(lattice->getBlock(bestModulePos));
    if (!mod) {
        cerr << "[RETURN ERROR] No module at " << bestModulePos.to_string() << "\n";
        pendingOrigins.erase(bestOrigin);
        returnTargets.erase(bestModulePos);
        executeReturnIteration();
        return;
    }

    Cell3DPosition nextPos = bestPath[1];
    // findPathForModule guarantees first step is valid; verify once more for safety.
    if (!mod->canMoveTo(nextPos)) {
        cout << "  [RETURN WARNING] Cannot move " << bestModulePos.to_string()
             << " toward origin " << bestOrigin.to_string() << ". Skipping.\n";
        pendingOrigins.erase(bestOrigin);
        returnTargets.erase(bestModulePos);
        executeReturnIteration();
        return;
    }

    activeReturnModule = bestModulePos;
    activeReturnPath   = bestPath;
    activeReturnStep   = 1;
    activeReturnOrigin = bestOrigin;

    cout << "  [RETURN] " << bestModulePos.to_string()
         << " → " << bestOrigin.to_string()
         << " via " << (bestLen - 1) << "-step path.\n";

    auto* sched = getScheduler();
    sched->schedule(new Catoms3DRotationStartEvent(sched->now(), mod, nextPos));
}

// =============================================================================
//  handleReturnMotionEnd — advance active module or start next iteration
// =============================================================================

void Catoms3DFlowBridgeCode::handleReturnMotionEnd(const Cell3DPosition& arrivedAt) {
    Cell3DPosition newPos = arrivedAt;
    auto* lattice = BaseSimulator::getWorld()->lattice;

    // Update returnTargets: module moved from activeReturnModule to newPos.
    auto rtIt = returnTargets.find(activeReturnModule);
    if (rtIt != returnTargets.end()) {
        Cell3DPosition orig = rtIt->second;
        returnTargets.erase(rtIt);
        returnTargets[newPos] = orig;
    }
    activeReturnModule = newPos;

    // ------------------------------ Arrived ----------------------------------
    if (newPos == activeReturnOrigin) {
        pendingOrigins.erase(activeReturnOrigin);
        returnTargets.erase(newPos);
        cout << "  [RETURNED] " << newPos.to_string() << " restored to origin.\n";
        executeReturnIteration();
        return;
    }

    // -------------------------- More steps remaining -------------------------
    activeReturnStep++;
    if (activeReturnStep < (int)activeReturnPath.size()) {
        Cell3DPosition nextPos = activeReturnPath[activeReturnStep];
        Catoms3DBlock* mod = static_cast<Catoms3DBlock*>(lattice->getBlock(newPos));
        if (!mod) {
            cerr << "[RETURN ERROR] No module at " << newPos.to_string() << " mid-return.\n";
            pendingOrigins.erase(activeReturnOrigin);
            returnTargets.erase(newPos);
            executeReturnIteration();
            return;
        }

        if (!mod->canMoveTo(nextPos)) {
            // Path invalidated mid-execution — recompute from current position
            // using the actual module so first step is guaranteed valid.
            cout << "  [RETURN REROUTE] Path invalid at step " << activeReturnStep
                 << ". Recomputing from " << newPos.to_string() << ".\n";
            vector<Cell3DPosition> newPath = findPathForModule(mod, newPos, activeReturnOrigin);
            if (newPath.size() < 2) {
                cout << "  [RETURN WARNING] Cannot reroute "
                     << newPos.to_string() << " to " << activeReturnOrigin.to_string()
                     << ". Skipping module.\n";
                pendingOrigins.erase(activeReturnOrigin);
                returnTargets.erase(newPos);
                executeReturnIteration();
                return;
            }
            activeReturnPath = newPath;
            activeReturnStep = 1;
            nextPos = activeReturnPath[1];
            // Double-check the recomputed first step is valid.
            if (!mod->canMoveTo(nextPos)) {
                cout << "  [RETURN WARNING] Recomputed path first step still invalid for "
                     << newPos.to_string() << ". Skipping module.\n";
                pendingOrigins.erase(activeReturnOrigin);
                returnTargets.erase(newPos);
                executeReturnIteration();
                return;
            }
        }

        auto* sched = getScheduler();
        sched->schedule(new Catoms3DRotationStartEvent(sched->now(), mod, nextPos));
    } else {
        // Step overflow — should have been caught by arrival check above.
        cerr << "[RETURN ERROR] Step overflow for module at " << newPos.to_string() << ".\n";
        executeReturnIteration();
    }
}

// =============================================================================
//  verifyAllOriginsRestored — final check; then clear and advance
// =============================================================================

void Catoms3DFlowBridgeCode::verifyAllOriginsRestored() {
    auto* lattice = BaseSimulator::getWorld()->lattice;
    for (const auto& [origin, path] : travelLog) {
        if (!lattice->getBlock(origin))
            cout << "  [RETURN WARNING] Origin " << origin.to_string() << " still unoccupied.\n";
        else
            cout << "  [VERIFIED] " << origin.to_string() << " restored.\n";
    }

    cout << "=== [Combined bridge] Return complete. ===\n";

    clearBridgeTaskState();
    advanceToNextTask();
}

// =============================================================================
//  clearBridgeTaskState — reset all per-task state, set phase to IDLE
// =============================================================================

void Catoms3DFlowBridgeCode::clearBridgeTaskState() {
    currentPhase        = BridgePhase::IDLE;
    travelLog.clear();
    roundAssignments.clear();
    placedIntermediates.clear();
    pendingMotions.clear();
    pendingArrival.clear();
    forwardStepIndex.clear();
    forwardVisited.clear();
    returnTargets.clear();
    pendingOrigins.clear();
    activeReturnPath.clear();
    activeReturnStep    = 0;
    currentForwardRound = 0;

    // Reset activity-diagram selection state.
    currentSelectionStage = SelectionStage::SEL_DONE;
    sourceModulesPool.clear();
    srcSideIdlePool.clear();
    sinkSideIdlePool.clear();
    bridgeSinkMostOrder.clear();
    bridgeSrcMostOrder.clear();
    distToSource.clear();
    distToSink.clear();
}

// =============================================================================
//  Phase 5 — advanceToNextTask
//
//  Pop the next BridgeTask and call processBridgeTask(), or log completion if
//  the queue is empty.  In this application the queue contains at most one
//  task (the combined bridge), so this terminates the run.
// =============================================================================

void Catoms3DFlowBridgeCode::advanceToNextTask() {
    if (pendingBridges.empty()) {
        cout << "All bridge tasks completed.\n";
        return;
    }
    BridgeTask next = pendingBridges.front();
    pendingBridges.pop();
    processBridgeTask(next);
}

// =============================================================================
//  handleForwardMotionEnd — step advancement for forward phase
//
//  This handler is called after each single step of any in-flight module.
//  Multiple modules may be moving concurrently within the same round;
//  pendingArrival maps each expected next position to its origin.
// =============================================================================

void Catoms3DFlowBridgeCode::handleForwardMotionEnd(const Cell3DPosition& arrivedAt) {
    auto it = pendingArrival.find(arrivedAt);
    if (it == pendingArrival.end()) return;   // stale event — ignore

    Cell3DPosition origin = it->second;
    pendingArrival.erase(it);

    // Mark this position as visited to prevent the module from returning here.
    forwardVisited[origin].insert(arrivedAt);

    // Retrieve the module's planned destination.
    auto tlIt = travelLog.find(origin);
    if (tlIt == travelLog.end()) {
        cerr << "[Bridge] No travelLog for module from " << origin.to_string() << "\n";
        pendingMotions.erase(origin);
        forwardVisited.erase(origin);
        if (pendingMotions.empty()) onForwardRoundComplete();
        return;
    }
    Cell3DPosition finalDest = tlIt->second.back();

    // Update returnTargets so the return phase always knows where this module
    // is.
    for (auto rtIt = returnTargets.begin(); rtIt != returnTargets.end(); ++rtIt) {
        if (rtIt->second == origin && rtIt->first != arrivedAt) {
            returnTargets[arrivedAt] = origin;
            returnTargets.erase(rtIt);
            break;
        }
    }

    // -------------------------------------------------------------------------
    //  Arrived at the bridge cell — module is placed.
    // -------------------------------------------------------------------------
    if (arrivedAt == finalDest) {
        cout << "  [ARRIVED] Module from " << origin.to_string()
             << " placed at bridge cell " << arrivedAt.to_string() << ".\n";
        forwardVisited.erase(origin);
        pendingMotions.erase(origin);
        if (pendingMotions.empty())
            onForwardRoundComplete();
        return;
    }

    // -------------------------------------------------------------------------
    //  More steps remain.  Recompute the path from the current position using
    //  findPathForModule (exact first-step check) but excluding visited cells
    //  so the module cannot oscillate back through positions it already
    //  passed.
    // -------------------------------------------------------------------------
    auto* lattice = BaseSimulator::getWorld()->lattice;
    Catoms3DBlock* mod = static_cast<Catoms3DBlock*>(lattice->getBlock(arrivedAt));

    if (!mod) {
        cerr << "[Bridge] Module vanished at " << arrivedAt.to_string() << ".\n";
        returnTargets.erase(finalDest);
        returnTargets[arrivedAt] = origin;
        travelLog.erase(origin);
        forwardStepIndex.erase(origin);
        forwardVisited.erase(origin);
        pendingMotions.erase(origin);
        if (pendingMotions.empty()) onForwardRoundComplete();
        return;
    }

    const set<Cell3DPosition>& visited = forwardVisited[origin];

    // Build candidate list: free neighbours reachable via canMoveTo, not yet
    // visited.
    vector<Cell3DPosition> candidates;
    for (auto& nb : lattice->getFreeNeighborCells(arrivedAt))
        if (!visited.count(nb) && mod->canMoveTo(nb))
            candidates.push_back(nb);

    bool scheduled = false;

    if (!candidates.empty()) {
        // Among candidates, pick the one whose onward path to finalDest is
        // shortest and does not pass through any visited cell.
        vector<Cell3DPosition> bestPath;
        Cell3DPosition bestNext;
        int bestLen = INT_MAX;

        for (const auto& cand : candidates) {
            vector<Cell3DPosition> onward = findPath(cand, finalDest);
            if (onward.empty()) continue;
            bool passesVisited = false;
            for (const auto& step : onward)
                if (visited.count(step)) { passesVisited = true; break; }
            if (passesVisited) continue;
            int len = 1 + (int)onward.size();
            if (len < bestLen) {
                bestLen  = len;
                bestNext = cand;
                bestPath.clear();
                bestPath.push_back(arrivedAt);
                bestPath.push_back(cand);
                for (size_t k = 1; k < onward.size(); ++k)
                    bestPath.push_back(onward[k]);
            }
        }

        if (!bestPath.empty()) {
            travelLog[origin]        = bestPath;
            forwardStepIndex[origin] = 0;
            // Purge any stale pendingArrival entry for this module.
            for (auto paIt = pendingArrival.begin(); paIt != pendingArrival.end(); )
                paIt = (paIt->second == origin) ? pendingArrival.erase(paIt) : ++paIt;

            pendingArrival[bestNext] = origin;
            auto* sched = getScheduler();
            try {
                sched->schedule(
                    new Catoms3DRotationStartEvent(sched->now(), mod, bestNext));
                scheduled = true;
            } catch (const exception& e) {
                cerr << "[Bridge] Exception scheduling step: " << e.what() << "\n";
                pendingArrival.erase(bestNext);
            }
        }
    }

    if (!scheduled) {
        // All unvisited reachable neighbours are dead ends — module is stuck.
        // Keep it in returnTargets at its current position so the return phase
        // brings it back home.  The bridge cell (finalDest) stays unoccupied
        // and will be retried with the next available idle module.
        cerr << "[Bridge] Module from " << origin.to_string()
             << " stuck at " << arrivedAt.to_string()
             << " (cannot reach " << finalDest.to_string()
             << " without revisiting). Will be returned home.\n";

        returnTargets.erase(finalDest);
        returnTargets[arrivedAt] = origin;
        travelLog.erase(origin);
        forwardStepIndex.erase(origin);
        forwardVisited.erase(origin);
        pendingMotions.erase(origin);
        // This module is done for this round; if it was the last, advance to
        // next round.
        if (pendingMotions.empty()) onForwardRoundComplete();
    }
}
