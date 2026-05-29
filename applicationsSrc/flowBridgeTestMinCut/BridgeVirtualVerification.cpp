// =============================================================================
//  BridgeVirtualVerification.cpp
//  -----------------------------
//  Virtual bridge pre-verification.
//
//  After findCombinedBridge() has populated combinedBridgePositions but BEFORE
//  any physical orchestration begins, temporarily insert phantom Catoms3DBlock
//  objects at every combined-bridge cell, rebuild a fresh FlowGraph from
//  scratch over the new lattice state, and run Edmonds-Karp.  The phantoms
//  act as pivots for surrounding empty cells, so the resulting max-flow
//  reflects what the flow would be once the physical bridge is in place.
//  Phantoms are removed and destroyed before returning — the lattice is
//  restored byte-for-byte.
//
//  Logs whether the virtual flow strictly exceeds originalFlowValue (bridge
//  will help), equals it (bridge does not improve flow), or is lower
//  (unexpected — typically indicates a graph-construction issue).
// =============================================================================

#include "Catoms3DFlowBridgeCode.h"
#include "FlowGraph.h"

#include <algorithm>
#include <climits>
#include <iostream>
#include <queue>
#include <set>

using namespace std;
using namespace Catoms3D;

// =============================================================================
//  verifyBridgeVirtually
// =============================================================================

void Catoms3DFlowBridgeCode::verifyBridgeVirtually() {
    cout << "\n=== [Virtual Bridge Pre-Verification] ===\n";

    if (combinedBridgePositions.empty()) {
        cout << "  No combined-bridge cells; skipping virtual verification.\n";
        return;
    }

    cout << "  Combined bridge has " << combinedBridgePositions.size()
         << " cell(s).  Inserting phantom modules and re-running Edmonds-Karp\n"
         << "  on the resulting lattice state (baseline flow = "
         << originalFlowValue << ").\n";

    auto* lattice = BaseSimulator::getWorld()->lattice;

    // -------------------------------------------------------------------------
    //  Sanity check: every bridge cell must currently be empty.  If any is
    //  already occupied we cannot safely insert a phantom there — skip it and
    //  warn the user (the physical orchestration would also skip it).
    // -------------------------------------------------------------------------
    vector<Cell3DPosition> insertCells;
    insertCells.reserve(combinedBridgePositions.size());
    for (const auto& pos : combinedBridgePositions) {
        if (!lattice->isInGrid(pos)) {
            cerr << "  [SKIP] " << pos.to_string()
                 << " is not in the grid; cannot insert phantom.\n";
            continue;
        }
        if (lattice->getBlock(pos) != nullptr) {
            cerr << "  [SKIP] " << pos.to_string()
                 << " is already occupied; cannot insert phantom.\n";
            continue;
        }
        insertCells.push_back(pos);
    }

    if (insertCells.empty()) {
        cout << "  No free bridge cells available for phantom insertion.\n";
        return;
    }

    // -------------------------------------------------------------------------
    //  Virtually move source modules to bridge cells.
    //
    //  "As if the bridge was built" means the source modules that fill the
    //  bridge are no longer at their original source positions.  Leaving them
    //  in place causes the motion engine's "opposite of FROM must be free"
    //  rule (catoms3DMotionRules.cpp:401) to fail for every phantom-pivoted
    //  motion whose mirror cell happens to be a source position — which is
    //  exactly why the naive virtual graph reports 0 augmenting paths.
    //
    //  We don't know in advance which sources the donor-selection state
    //  machine will pick (computeBridgeSelectionState runs later), but for
    //  max-flow purposes all super-source-connected sources are equivalent —
    //  only their lattice occupancy changes the motion graph.  We pick the
    //  |insertCells| sources with the SMALLEST Manhattan distance to their
    //  nearest bridge cell (i.e., the sources the donor logic would naturally
    //  consume first) and remove them from the lattice until the FlowGraph
    //  build is done.
    // -------------------------------------------------------------------------
    vector<pair<int, Cell3DPosition>> srcByDist;
    srcByDist.reserve(movingBlocks.size());
    for (const auto& s : movingBlocks) {
        int minDist = INT_MAX;
        for (const auto& b : insertCells) {
            int d = s.dist_taxi(b);
            if (d < minDist) minDist = d;
        }
        srcByDist.emplace_back(minDist, s);
    }
    sort(srcByDist.begin(), srcByDist.end(),
         [](const auto& a, const auto& b) { return a.first < b.first; });

    size_t nMove = min(insertCells.size(), srcByDist.size());

    // Save (pos, module*) pairs so we can re-insert exactly what we removed.
    vector<pair<Cell3DPosition, BuildingBlock*>> savedSources;
    savedSources.reserve(nMove);
    for (size_t i = 0; i < nMove; ++i) {
        const Cell3DPosition& sp = srcByDist[i].second;
        BuildingBlock* mod = lattice->getBlock(sp);
        if (!mod) {
            cerr << "  [WARN] source " << sp.to_string()
                 << " is unexpectedly empty; skipping its virtual move.\n";
            continue;
        }
        lattice->remove(sp, /*count=*/false);
        savedSources.emplace_back(sp, mod);
    }
    cout << "  Virtually moved " << savedSources.size() << " of "
         << movingBlocks.size() << " source module(s) (closest to the "
         << "bridge) out of their original positions.\n";

    // -------------------------------------------------------------------------
    //  Pick a phantom-ID range that does not collide with any existing module
    //  ID.  We never call world->addBlock(), so maxBlockId is not touched —
    //  phantoms exist only in the lattice grid and are removed before exit.
    // -------------------------------------------------------------------------
    bID phantomIdBase = 1;
    {
        const auto& blocks = BaseSimulator::getWorld()->getMap();
        for (const auto& kv : blocks)
            if (kv.first >= phantomIdBase) phantomIdBase = kv.first + 1;
        // Add a generous offset to make phantoms easy to spot in any log that
        // might accidentally print their IDs.
        phantomIdBase += 1000000;
    }

    vector<Catoms3DBlock*>   phantoms;
    vector<Catoms3DGlBlock*> phantomGls;
    phantoms.reserve(insertCells.size());
    phantomGls.reserve(insertCells.size());

    // -------------------------------------------------------------------------
    //  Insert one phantom per bridge cell.
    // -------------------------------------------------------------------------
    for (size_t i = 0; i < insertCells.size(); ++i) {
        const Cell3DPosition& pos = insertCells[i];
        bID id = phantomIdBase + (bID)i;

        // Create a phantom Catoms3DBlock.  BuildingBlock's ctor instantiates a
        // BlockCode via our buildNewBlockCode, but startup() is gated by a
        // static `hasRun` flag so it will not re-execute.
        auto* phantom = new Catoms3DBlock(id,
                                          Catoms3DFlowBridgeCode::buildNewBlockCode);

        // A Catoms3DGlBlock holds the per-module transformation matrix (`mat`)
        // that Catoms3DBlock::getConnectorId() / getNeighborPos() dereference
        // unconditionally.  Without it those methods crash with a null-pointer
        // access when the motion engine treats the phantom as a pivot.  We
        // must therefore allocate a real GlBlock, wire it into the phantom via
        // setGlBlock(), and let setPositionAndOrientation populate the
        // rotation matrix.
        auto* gl = new Catoms3DGlBlock(id);
        phantom->setGlBlock(gl);
        // Now ptrGlBlock is valid; this populates gl->mat with the rotation
        // matrix for (pos, orientCode=0).
        phantom->setPositionAndOrientation(pos, 0);

        try {
            lattice->insert(phantom, pos, /*count=*/false);
        } catch (...) {
            cerr << "  [ERROR] lattice->insert failed at " << pos.to_string()
                 << "; aborting virtual verification.\n";
            delete gl;
            delete phantom;
            // Roll back anything we already inserted.
            for (size_t j = 0; j < phantoms.size(); ++j) {
                lattice->remove(phantoms[j]->position, /*count=*/false);
                delete phantoms[j];
                delete phantomGls[j];
            }
            // Restore virtually-moved source modules.
            for (auto& [sp, mod] : savedSources)
                lattice->insert(mod, sp, /*count=*/false);
            return;
        }
        phantoms.push_back(phantom);
        phantomGls.push_back(gl);
    }

    cout << "  Inserted " << phantoms.size()
         << " phantom module(s) at bridge cells.\n";

    // -------------------------------------------------------------------------
    //  Sanity check: every phantom is visible through lattice->getBlock and
    //  its location is correctly reported as non-free.
    // -------------------------------------------------------------------------
    {
        int seenOk = 0, seenBad = 0;
        for (const Cell3DPosition& pos : insertCells) {
            if (lattice->getBlock(pos) != nullptr && !lattice->isFree(pos))
                ++seenOk;
            else
                ++seenBad;
        }
        cout << "  Lattice phantom visibility: " << seenOk
             << " ok, " << seenBad << " missing.\n";
    }

    // -------------------------------------------------------------------------
    //  Probe: enumerate motion possibilities that USE phantoms as pivots.
    //  This confirms the motion engine can produce links from phantom pivots
    //  before we hand the graph to Edmonds-Karp.
    // -------------------------------------------------------------------------
    {
        int linkCount = 0;
        for (const auto& bridgePos : insertCells) {
            for (const auto& nb :
                 lattice->getNeighborhood(bridgePos)) {
                if (!lattice->isInGrid(nb) || !lattice->isFree(nb)) continue;
                vector<Cell3DPosition> reachable;
                getAllPossibleMotionsFromPosition(nb, reachable);
                for (const auto& dest : reachable) {
                    // Only count motions that actually pivot off this phantom
                    // (the phantom must be a common active neighbour of nb and
                    // dest); a cheap check is that the phantom is in the
                    // active-neighbours of nb.
                    bool usesPhantom = false;
                    for (const auto& act :
                         lattice->getActiveNeighborCells(nb)) {
                        if (act == bridgePos) { usesPhantom = true; break; }
                    }
                    if (usesPhantom) ++linkCount;
                }
            }
        }
        cout << "  Phantom-pivoted motion links (over all bridge cells, "
             << "with multiplicity): " << linkCount << "\n";
    }

    // -------------------------------------------------------------------------
    //  Per-source quick reachability probe: from each moving block at its
    //  current position, how many cells does the motion engine return?  With
    //  phantoms in place this MUST still be positive for sources whose
    //  neighbourhood is unaffected by phantoms (sources sit at x=1..4).
    // -------------------------------------------------------------------------
    {
        cout << "  Per-source one-step reachable counts:\n";
        for (const auto& s : movingBlocks) {
            vector<Cell3DPosition> r;
            getAllPossibleMotionsFromPosition(s, r);
            cout << "    " << s.to_string() << " -> " << r.size()
                 << " reachable cell(s)\n";
        }
    }

    // -------------------------------------------------------------------------
    //  Build a fresh FlowGraph over the modified lattice state and run
    //  Edmonds-Karp from scratch.
    // -------------------------------------------------------------------------
    cout << "  Building virtual FlowGraph...\n" << flush;
    FlowGraph* fgVirtual = new FlowGraph();
    fgVirtual->buildFlowGraph();
    cout << "  Virtual FlowGraph node count: " << fgVirtual->nodes.size()
         << "\n";

    // -------------------------------------------------------------------------
    //  Diagnostic: count edges leaving super-source and entering super-sink,
    //  and BFS-reachability from super-source over positive-capacity edges.
    // -------------------------------------------------------------------------
    {
        int srcOutEdges = 0;
        for (Edge* e : fgVirtual->superSource->edges)
            if (e->capacity > 0) ++srcOutEdges;
        int sinkInEdges = 0;
        for (const auto& kv : fgVirtual->nodes)
            for (Edge* e : kv.second->edges)
                if (e->to == fgVirtual->superSink && e->capacity > 0)
                    ++sinkInEdges;
        cout << "  Super-source out-edges (positive cap): " << srcOutEdges
             << "\n  Super-sink   in-edges  (positive cap): " << sinkInEdges
             << "\n";

        set<Node*> reach;
        queue<Node*> bfs;
        bfs.push(fgVirtual->superSource);
        reach.insert(fgVirtual->superSource);
        while (!bfs.empty()) {
            Node* u = bfs.front(); bfs.pop();
            for (Edge* e : u->edges) {
                if (e->capacity > 0 && !reach.count(e->to)) {
                    reach.insert(e->to);
                    bfs.push(e->to);
                }
            }
        }
        cout << "  BFS reachable from SUPER_SOURCE: " << reach.size()
             << " node(s). "
             << (reach.count(fgVirtual->superSink) ? "SUPER_SINK IS reachable.\n"
                                                  : "SUPER_SINK is NOT reachable.\n");
    }

    int virtualFlow = fgVirtual->findAndPrintAugmentingPaths(fgVirtual->superSource,
                                                             fgVirtual->superSink);
    delete fgVirtual;

    // -------------------------------------------------------------------------
    //  Restore the lattice exactly as it was.
    // -------------------------------------------------------------------------
    for (size_t i = 0; i < phantoms.size(); ++i) {
        lattice->remove(phantoms[i]->position, /*count=*/false);
        delete phantoms[i];
        delete phantomGls[i];
    }
    phantoms.clear();
    phantomGls.clear();
    // Re-insert the source modules we temporarily lifted out.
    for (auto& [sp, mod] : savedSources)
        lattice->insert(mod, sp, /*count=*/false);
    savedSources.clear();

    // -------------------------------------------------------------------------
    //  Report the virtual flow against the baseline.
    // -------------------------------------------------------------------------
    cout << "\n  [Virtual Flow Result]\n"
         << "    Baseline (no bridge)        = " << originalFlowValue << "\n"
         << "    Virtual (bridge as if built)= " << virtualFlow << "\n";

    int numMinCutEdges = (int)allMinCutEdges.size();
    int expectedFlow   = originalFlowValue + numMinCutEdges;

    if (virtualFlow > originalFlowValue) {
        cout << "  [VIRTUAL OK] Building the bridge WILL increase flow ("
             << originalFlowValue << " -> " << virtualFlow << ").\n";
        if (virtualFlow >= expectedFlow)
            cout << "  [VIRTUAL SUCCESS] Virtual flow " << virtualFlow
                 << " >= expected " << expectedFlow
                 << " (baseline + " << numMinCutEdges << " min-cut arc(s)).\n";
        else
            cout << "  [VIRTUAL INFO] Virtual flow " << virtualFlow
                 << " < expected " << expectedFlow
                 << " — bridge bypasses only a subset of the min-cut arcs.\n";
    } else if (virtualFlow == originalFlowValue) {
        cout << "  [VIRTUAL WARNING] Building the bridge does NOT increase flow"
             << " (still " << virtualFlow
             << ").  The combined bridge may not unlock any new augmenting path.\n";
    } else {
        cerr << "  [VIRTUAL ERROR] Virtual flow " << virtualFlow
             << " < baseline " << originalFlowValue
             << " — unexpected; check phantom-insertion correctness.\n";
    }
    cout << flush;
}
