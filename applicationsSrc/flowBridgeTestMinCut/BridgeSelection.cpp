// =============================================================================
//  BridgeSelection.cpp
//  -------------------
//  Activity-diagram-driven donor-selection state machine for the Catoms3D
//  Flow Bridge.
//
//  These routines do NOT alter the motion mechanism — they only decide which
//  module (and which bridge cell) to dispatch next.
//
//      +----------------------------------+
//      | |S| (sources) vs |B| (bridge)    |
//      +----------------------------------+
//           |S| > |B|           |S| ≤ |B|
//                |                  |
//        SOURCE_ONLY     +----------+----------+
//        (sources only)  | A: SRC_SIDE_IDLE    |
//                        |    src-side idle    |
//                        |    -> sink-most     |
//                        +----------+----------+
//                                   | not full
//                        +----------+----------+
//                        | B: SRC_MODULES      |
//                        |    sources          |
//                        |    -> source-most   |
//                        +----------+----------+
//                                   | not full
//                        +----------+----------+
//                        | C: SINK_SIDE_IDLE   |
//                        |    sink-side idle   |
//                        |    -> closest       |
//                        +---------------------+
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
#include <queue>

using namespace std;
using namespace Catoms3D;

// =============================================================================
//  bfsDistancesFromSet
//    Multi-source BFS through lattice->getNeighborhood from every cell in
//    `starts`.  Returns the lattice distance (in cells) to every reachable
//    cell.
// =============================================================================

map<Cell3DPosition, int>
Catoms3DFlowBridgeCode::bfsDistancesFromSet(const vector<Cell3DPosition>& starts) {
    auto* lattice = BaseSimulator::getWorld()->lattice;
    map<Cell3DPosition, int> dist;
    queue<Cell3DPosition> q;
    for (const auto& s : starts) {
        if (dist.emplace(s, 0).second) q.push(s);
    }
    while (!q.empty()) {
        Cell3DPosition cur = q.front(); q.pop();
        int d = dist[cur];
        for (const auto& nb : lattice->getNeighborhood(cur)) {
            if (dist.find(nb) == dist.end()) {
                dist[nb] = d + 1;
                q.push(nb);
            }
        }
    }
    return dist;
}

// =============================================================================
//  computeBridgeSelectionState
//    Snapshot the moving-block pool, classify every valid idle module as
//    src-side or sink-side, order the bridge cells by both source-distance
//    and sink-distance, and pick the initial SelectionStage.
// =============================================================================

void Catoms3DFlowBridgeCode::computeBridgeSelectionState() {
    // -------- Snapshot source modules and reset pools --------
    sourceModulesPool   = movingBlocks;
    srcSideIdlePool .clear();
    sinkSideIdlePool.clear();
    bridgeSinkMostOrder.clear();
    bridgeSrcMostOrder .clear();

    // -------- BFS distance to the nearest source / sink ------
    distToSource = bfsDistancesFromSet(movingBlocks);
    distToSink   = bfsDistancesFromSet(targetPositions);

    auto distOr = [](const map<Cell3DPosition,int>& m,
                     const Cell3DPosition& p) {
        auto it = m.find(p);
        return (it == m.end()) ? INT_MAX : it->second;
    };

    // -------- Classify idle modules by side ------------------
    for (const auto& p : validIdleModules) {
        int dS = distOr(distToSource, p);
        int dT = distOr(distToSink,   p);
        if (dS <= dT) srcSideIdlePool .push_back(p);
        else           sinkSideIdlePool.push_back(p);
    }

    // -------- Order bridge cells -----------------------------
    bridgeSinkMostOrder = combinedBridgePositions;
    bridgeSrcMostOrder  = combinedBridgePositions;
    sort(bridgeSinkMostOrder.begin(), bridgeSinkMostOrder.end(),
         [&](const Cell3DPosition& a, const Cell3DPosition& b) {
             return distOr(distToSink, a) < distOr(distToSink, b);
         });
    sort(bridgeSrcMostOrder.begin(), bridgeSrcMostOrder.end(),
         [&](const Cell3DPosition& a, const Cell3DPosition& b) {
             return distOr(distToSource, a) < distOr(distToSource, b);
         });

    // -------- Decide initial stage ---------------------------
    const size_t S = sourceModulesPool.size();
    const size_t B = combinedBridgePositions.size();
    if (S > B) {
        currentSelectionStage = SelectionStage::SOURCE_ONLY;
    } else {
        currentSelectionStage = SelectionStage::SRC_SIDE_IDLE_FILL;
    }

    cout << "\n--- Bridge selection state ---\n";
    cout << "  |source modules|       = " << S << "\n";
    cout << "  |bridge cells|         = " << B << "\n";
    cout << "  |src-side idle pool|   = " << srcSideIdlePool.size()  << "\n";
    cout << "  |sink-side idle pool|  = " << sinkSideIdlePool.size() << "\n";
    cout << "  initial stage          = "
         << (currentSelectionStage == SelectionStage::SOURCE_ONLY
                 ? "SOURCE_ONLY"
                 : "SRC_SIDE_IDLE_FILL") << "\n\n";
}

// =============================================================================
//  advanceSelectionStage
//    Walk the state machine forward when the current stage is exhausted.
// =============================================================================

void Catoms3DFlowBridgeCode::advanceSelectionStage() {
    auto label = [](SelectionStage s) {
        switch (s) {
            case SelectionStage::SOURCE_ONLY:         return "SOURCE_ONLY";
            case SelectionStage::SRC_SIDE_IDLE_FILL:  return "SRC_SIDE_IDLE_FILL";
            case SelectionStage::SRC_MODULES_FILL:    return "SRC_MODULES_FILL";
            case SelectionStage::SINK_SIDE_IDLE_FILL: return "SINK_SIDE_IDLE_FILL";
            case SelectionStage::SEL_DONE:            return "SEL_DONE";
        }
        return "?";
    };
    SelectionStage previous = currentSelectionStage;
    switch (currentSelectionStage) {
        case SelectionStage::SOURCE_ONLY:
            // Source-only branch exits the state machine when exhausted.
            currentSelectionStage = SelectionStage::SEL_DONE;
            break;
        case SelectionStage::SRC_SIDE_IDLE_FILL:
            currentSelectionStage = SelectionStage::SRC_MODULES_FILL;
            break;
        case SelectionStage::SRC_MODULES_FILL:
            currentSelectionStage = SelectionStage::SINK_SIDE_IDLE_FILL;
            break;
        case SelectionStage::SINK_SIDE_IDLE_FILL:
        case SelectionStage::SEL_DONE:
            currentSelectionStage = SelectionStage::SEL_DONE;
            break;
    }
    cout << "  [STAGE] " << label(previous)
         << " → " << label(currentSelectionStage) << "\n";
}

// =============================================================================
//  planAndDispatchUnderCurrentStage
//
//  Drain the current SelectionStage's donor pool into the bridge via an
//  ITERATIVE max-flow loop.  Each iteration:
//
//    1. Snapshot the donors of the current pool that are STILL at origin
//       (modules teleported in a previous iteration are excluded automatically
//       since lattice->getBlock(origin) returns null at their original cells).
//    2. Snapshot the bridge cells that are STILL empty (cells filled in a
//       previous iteration are excluded).
//    3. Build a fresh unit-capacity flow graph that reflects the new lattice
//       configuration — the topology can change between iterations because
//       previously-teleported donors now act as pivots for cells in the empty
//       motion graph.
//    4. Run Edmonds-Karp.  For every augmenting path, teleport the donor
//       (path.front()) directly to the bridge cell (path.back()).
//
//  The loop terminates when ANY of these holds:
//    * The donor pool has no module still at origin (pool drained).
//    * Every combined-bridge cell is occupied (bridge filled).
//    * Max-flow yields zero augmenting paths (the remaining donors of this
//      pool cannot reach any empty bridge cell on the current configuration).
//
//  This matches the contract requested by the application:
//    "repeatedly until the current pool is empty (all the donors have been
//     used) or the entire bridge is filled, the algorithm should build
//     another flow graph with the same capacities but now taking into
//     consideration the new configuration".
//
//  Motion is NOT the contribution of this application — the contribution is
//  the bridge-finding pipeline (Picard-Queyranne all-min-cuts + EDP combined
//  bridge + max-flow assignment of donors to bridge cells).  To keep the
//  visualisation focused on that contribution and to avoid the cost of
//  step-by-step rotations, every assigned donor is removed from its origin
//  cell and re-inserted at its target bridge cell directly in the lattice.
//
//  Graph topology per iteration (FlowGraph::buildBridgeFlowGraph):
//      super-source -> each donor                  (cap 1)
//      donor expands through empty cells           (cap 1, via motion graph)
//      each empty bridge cell -> super-sink        (cap 1)
//
//  Cap-1 super edges enforce "one path per donor / one module per bridge
//  cell"; cap-1 internal edges make the augmenting paths edge-disjoint.
//  Only the (donor, bridge-cell) endpoints of each path are used — the
//  intermediate hops are discarded because the module is teleported.
//
//  Donor pool per stage (same as before — the activity-diagram state machine
//  still drives WHICH modules are considered, only the dispatch itself
//  changes):
//
//      SOURCE_ONLY         : sourceModulesPool
//      SRC_SIDE_IDLE_FILL  : srcSideIdlePool
//      SRC_MODULES_FILL    : sourceModulesPool
//      SINK_SIDE_IDLE_FILL : sinkSideIdlePool
//
//  Returns true if at least one module was teleported across the entire
//  iteration loop; false if the stage produced no progress (e.g. pool was
//  empty on entry).
// =============================================================================

bool Catoms3DFlowBridgeCode::planAndDispatchUnderCurrentStage() {
    auto* lattice = BaseSimulator::getWorld()->lattice;

    // -------------------------------------------------------------------------
    //  Choose donor pool for this stage.
    // -------------------------------------------------------------------------
    const vector<Cell3DPosition>* donorPool = nullptr;
    const char* stageLabel = "?";
    switch (currentSelectionStage) {
        case SelectionStage::SOURCE_ONLY:
            donorPool  = &sourceModulesPool;
            stageLabel = "SOURCE_ONLY";
            break;
        case SelectionStage::SRC_SIDE_IDLE_FILL:
            donorPool  = &srcSideIdlePool;
            stageLabel = "STAGE_A_SRC_IDLE";
            break;
        case SelectionStage::SRC_MODULES_FILL:
            donorPool  = &sourceModulesPool;
            stageLabel = "STAGE_B_SRC_MODULES";
            break;
        case SelectionStage::SINK_SIDE_IDLE_FILL:
            donorPool  = &sinkSideIdlePool;
            stageLabel = "STAGE_C_SINK_IDLE";
            break;
        case SelectionStage::SEL_DONE:
            return false;
    }

    // =========================================================================
    //  Inner iteration loop — rebuild the flow graph after every teleport
    //  batch using the updated lattice configuration.
    // =========================================================================
    int totalTeleported = 0;
    int iter            = 0;

    while (true) {
        ++iter;

        // ---------------------------------------------------------------------
        //  Snapshot donors of this pool still physically at origin.
        // ---------------------------------------------------------------------
        vector<Cell3DPosition> availDonors;
        availDonors.reserve(donorPool->size());
        for (const auto& o : *donorPool)
            if (lattice->getBlock(o) != nullptr) availDonors.push_back(o);

        if (availDonors.empty()) {
            cout << "  [PLAN/" << stageLabel << "][iter " << iter
                 << "] Donor pool drained.\n";
            break;
        }

        // ---------------------------------------------------------------------
        //  Snapshot still-empty bridge cells.
        // ---------------------------------------------------------------------
        vector<Cell3DPosition> emptyTargets;
        emptyTargets.reserve(combinedBridgePositions.size());
        for (const auto& pos : combinedBridgePositions)
            if (!lattice->getBlock(pos)) emptyTargets.push_back(pos);

        if (emptyTargets.empty()) {
            cout << "  [PLAN/" << stageLabel << "][iter " << iter
                 << "] Bridge fully filled.\n";
            break;
        }

        // ---------------------------------------------------------------------
        //  Build the unit-capacity bridge flow graph and run Edmonds-Karp on
        //  the CURRENT lattice configuration.
        // ---------------------------------------------------------------------
        cout << "  [PLAN/" << stageLabel << "][iter " << iter << "] "
             << availDonors.size()  << " donor(s) at origin, "
             << emptyTargets.size() << " empty bridge cell(s).\n";

        FlowGraph fg;
        fg.buildBridgeFlowGraph(availDonors, emptyTargets);

        // [viz] Snapshot original capacities BEFORE EK so the post-EK dump
        // can recover flow values.  Pure read; the algorithm reads the live
        // residual graph from `fg` itself.
        auto __viz_origCap = fg.backupCapacities();

        // [viz] Build the "still empty" / "already filled" classification
        // for the bridge cells used in the dumps below.
        std::vector<Cell3DPosition> __viz_filled;
        for (const auto& bp : combinedBridgePositions) {
            bool empty = false;
            for (const auto& e : emptyTargets) if (e == bp) { empty = true; break; }
            if (!empty) __viz_filled.push_back(bp);
        }

        // [viz] Stage 4 — bridge sub-graph BEFORE Edmonds-Karp; the unsolved
        // unit-capacity assignment problem (donors=red -> bridge cells=orange).
        {
            FlowGraphViz::ModuleSets sets;
            sets.sourceModules    = &availDonors;
            sets.targetCells      = &targetPositions;
            sets.bridgeEmpty      = &emptyTargets;
            sets.bridgeFilled     = &__viz_filled;
            sets.structuralBlocks = &structuralBlocks;
            std::string st  = std::string("4_bridge_iter") + std::to_string(iter) +
                              "_" + stageLabel;
            std::string ttl = std::string("Bridge-construction flow graph — iter ") +
                              std::to_string(iter) + " (" + stageLabel +
                              "): donors=red, bridge cells=orange";
            FlowGraphViz::dump(fg, st, ttl, sets, &__viz_origCap);
        }

        int nPaths = fg.findAndPrintAugmentingPaths(fg.superSource, fg.superSink);

        // [viz] Stage 5 — same sub-graph AFTER Edmonds-Karp with augmenting
        // paths showing which donor goes to which bridge cell.
        {
            FlowGraphViz::ModuleSets sets;
            sets.sourceModules    = &availDonors;
            sets.targetCells      = &targetPositions;
            sets.bridgeEmpty      = &emptyTargets;
            sets.bridgeFilled     = &__viz_filled;
            sets.structuralBlocks = &structuralBlocks;
            std::string st  = std::string("5_bridge_iter") + std::to_string(iter) +
                              "_" + stageLabel + "_assignment";
            std::string ttl = std::string("Teleport assignment — iter ") +
                              std::to_string(iter) + " (" + stageLabel +
                              "): augmenting paths = donor->bridge-cell pairs";
            FlowGraphViz::dump(fg, st, ttl, sets,
                                &__viz_origCap, &fg.augmentingPathPositions);
        }

        if (nPaths == 0) {
            cout << "  [PLAN/" << stageLabel << "][iter " << iter
                 << "] Max-flow yielded 0 paths; no further progress in this "
                    "stage.\n";
            break;
        }

        // ---------------------------------------------------------------------
        //  Teleport every assigned (donor, bridge-cell) pair.
        //
        //  Each augmenting path's positions vector is
        //      [donor, intermediate_1, ..., intermediate_k, bridgeCell].
        //  We use only path.front() (donor) and path.back() (bridge cell) —
        //  the hops in between are discarded because the module is teleported.
        //
        //  Steps per teleport mirror what Catoms3DRotationStopEvent::consume()
        //  does at the end of a normal rotation (catoms3DRotationEvents.cpp:
        //  271-305) so the next max-flow iteration sees a fully-refreshed
        //  lattice configuration:
        //
        //    1. Read the module's current orientation from its GL block
        //       matrix.
        //    2. World::disconnectBlock(mod, /*count=*/false)
        //         - tears down this module's P2P interfaces (and the
        //           connected interfaces of every neighbour);
        //         - removes the module from the origin lattice cell.
        //    3. mod->setPositionAndOrientation(target, orient)
        //         - updates the module's position field;
        //         - rebuilds its GL transformation matrix.
        //    4. World::connectBlock(mod, /*count=*/false)
        //         - inserts the module at the target lattice cell;
        //         - re-links this module's P2P interfaces against its new
        //           neighbours (linkBlock);
        //         - re-links EACH NEIGHBOUR's P2P interfaces too
        //           (linkNeighbors), so any module adjacent to the new
        //           target sees the teleported module as a fresh active
        //           neighbour on its next motion-graph query.
        //
        //  Without the link/unlink steps the motion engine can fail to
        //  enumerate pivots that should be available once the teleported
        //  module is in place, which leaves the last few bridge cells
        //  unreachable by max-flow even though the geometry permits them.
        // ---------------------------------------------------------------------
        World* world = BaseSimulator::getWorld();
        int teleportedThisIter = 0;

        for (const auto& path : fg.augmentingPathPositions) {
            if (path.size() < 2) continue;

            Cell3DPosition origin = path.front();
            Cell3DPosition target = path.back();

            // Defensive check: graph snapshot may be stale if a previous
            // teleport in this batch consumed the same cell — shouldn't
            // happen with cap-1 edges, but verify anyway.
            Catoms3DBlock* mod =
                static_cast<Catoms3DBlock*>(lattice->getBlock(origin));
            if (!mod) continue;
            if (lattice->getBlock(target)) continue;   // bridge cell taken

            try {
                uint8_t orient = Catoms3DBlock::getOrientationFromMatrix(
                    mod->getGlBlock()->mat);

                world->disconnectBlock(mod, /*count=*/false);
                mod->setPositionAndOrientation(target, orient);
                world->connectBlock(mod,    /*count=*/false);
            } catch (const exception& e) {
                cerr << "[Bridge] Exception teleporting "
                     << origin.to_string() << " -> " << target.to_string()
                     << ": " << e.what() << "\n";
                continue;
            }

            // Record the displacement for post-bridge logging /
            // verification.  pendingMotions / pendingArrival stay empty
            // because teleport is synchronous — no motion-end event will
            // fire.
            travelLog[origin]     = { origin, target };
            returnTargets[target] = origin;
            roundAssignments.push_back({ origin });
            placedIntermediates.insert(target);

            cout << "  [TELEPORT/" << stageLabel << "][iter " << iter
                 << "] Module " << origin.to_string()
                 << " -> bridge cell " << target.to_string() << "\n";
            ++teleportedThisIter;
        }

        cout << "  [PLAN/" << stageLabel << "][iter " << iter << "] "
             << teleportedThisIter
             << " module(s) teleported into the bridge this iteration.\n";

        totalTeleported += teleportedThisIter;

        // If max-flow returned >0 paths but every one failed the defensive
        // teleport check, we made zero progress — stop to avoid an infinite
        // loop.
        if (teleportedThisIter == 0) {
            cout << "  [PLAN/" << stageLabel << "][iter " << iter
                 << "] No teleports succeeded; aborting stage to avoid spin.\n";
            break;
        }
    }

    cout << "  [PLAN/" << stageLabel << "] Stage drained: "
         << totalTeleported << " module(s) teleported across " << iter
         << " iteration(s).\n";

    return totalTeleported > 0;
}
