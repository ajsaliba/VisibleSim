// =============================================================================
//  Catoms3DFlowBridgeCode.cpp
//  --------------------------
//  Core translation unit for the Catoms3D Flow Bridge application
//  (combined min-cut-edge variant).
//
//  This file is kept small on purpose — it holds only the pieces that don't
//  belong anywhere else:
//
//      * Static-member storage for Catoms3DFlowBridgeCode
//      * Constructor
//      * startup()                — XML parse + analysis pipeline + launch
//      * onMotionEnd()            — top-level motion dispatcher
//
//  Everything else lives in a sibling .cpp:
//
//      FlowGraph.cpp              — flow graph + Edmonds-Karp + EDP + PQ
//      BridgeXmlParsing.cpp       — XML parsing, getters/setters, printers
//      BridgeLatticeHelpers.cpp   — findPath, motion queries, pivot queries
//      BridgeIdleFiltering.cpp    — block / articulation classification
//      BridgeVirtualVerification.cpp — phantom-bridge pre-verification
//      BridgeSelection.cpp        — activity-diagram donor selection
//      BridgeOrchestration.cpp    — forward / return state machine
// =============================================================================

#include "Catoms3DFlowBridgeCode.h"
#include "FlowGraph.h"
#include "FlowGraphViz.h"

#include <iostream>

using namespace std;
using namespace Catoms3D;

// =============================================================================
//  Static member definitions
// =============================================================================

// -------- Existing analysis state --------------------------------------------
vector<Cell3DPosition>                        Catoms3DFlowBridgeCode::targetPositions;
vector<Cell3DPosition>                        Catoms3DFlowBridgeCode::movingBlocks;
vector<Cell3DPosition>                        Catoms3DFlowBridgeCode::structuralBlocks;
vector<Cell3DPosition>                        Catoms3DFlowBridgeCode::idleStructuralBlocks;
vector<Cell3DPosition>                        Catoms3DFlowBridgeCode::validIdleModules;
vector<pair<Cell3DPosition, Cell3DPosition>>  Catoms3DFlowBridgeCode::allMinCutEdges;
int                                           Catoms3DFlowBridgeCode::originalFlowValue = 0;
vector<Cell3DPosition>                        Catoms3DFlowBridgeCode::combinedBridgePositions;
vector<vector<Cell3DPosition>>                Catoms3DFlowBridgeCode::combinedBridgePaths;

// -------- Bridge orchestration -----------------------------------------------
queue<BridgeTask>                             Catoms3DFlowBridgeCode::pendingBridges;
BridgeTask                                    Catoms3DFlowBridgeCode::currentBridgeTask;
BridgePhase                                   Catoms3DFlowBridgeCode::currentPhase = BridgePhase::IDLE;

map<Cell3DPosition, vector<Cell3DPosition>>   Catoms3DFlowBridgeCode::travelLog;
vector<vector<Cell3DPosition>>                Catoms3DFlowBridgeCode::roundAssignments;
set<Cell3DPosition>                           Catoms3DFlowBridgeCode::placedIntermediates;

map<Cell3DPosition, size_t>                   Catoms3DFlowBridgeCode::forwardStepIndex;
map<Cell3DPosition, Cell3DPosition>           Catoms3DFlowBridgeCode::pendingArrival;
set<Cell3DPosition>                           Catoms3DFlowBridgeCode::pendingMotions;
map<Cell3DPosition, set<Cell3DPosition>>     Catoms3DFlowBridgeCode::forwardVisited;

map<Cell3DPosition, Cell3DPosition>           Catoms3DFlowBridgeCode::returnTargets;
set<Cell3DPosition>                           Catoms3DFlowBridgeCode::pendingOrigins;
Cell3DPosition                                Catoms3DFlowBridgeCode::activeReturnModule;
vector<Cell3DPosition>                        Catoms3DFlowBridgeCode::activeReturnPath;
int                                           Catoms3DFlowBridgeCode::activeReturnStep   = 0;
Cell3DPosition                                Catoms3DFlowBridgeCode::activeReturnOrigin;

int                                           Catoms3DFlowBridgeCode::currentForwardRound = 0;

// -------- Activity-diagram donor-selection state ----------------------------
SelectionStage                                Catoms3DFlowBridgeCode::currentSelectionStage = SelectionStage::SEL_DONE;
vector<Cell3DPosition>                        Catoms3DFlowBridgeCode::sourceModulesPool;
vector<Cell3DPosition>                        Catoms3DFlowBridgeCode::srcSideIdlePool;
vector<Cell3DPosition>                        Catoms3DFlowBridgeCode::sinkSideIdlePool;
vector<Cell3DPosition>                        Catoms3DFlowBridgeCode::bridgeSinkMostOrder;
vector<Cell3DPosition>                        Catoms3DFlowBridgeCode::bridgeSrcMostOrder;
map<Cell3DPosition, int>                      Catoms3DFlowBridgeCode::distToSource;
map<Cell3DPosition, int>                      Catoms3DFlowBridgeCode::distToSink;

// =============================================================================
//  Constructor
// =============================================================================

Catoms3DFlowBridgeCode::Catoms3DFlowBridgeCode(Catoms3DBlock *host)
    : Catoms3DBlockCode(host), catom(host) {}

// =============================================================================
//  startup()
// =============================================================================
//
//  Pipeline:
//
//   [1] Parse XML  → targetPositions, movingBlocks, structuralBlocks.
//
//   [2] Build main FlowGraph (movingBlocks → targetPositions), run
//       Edmonds-Karp, then apply the Picard-Queyranne characterisation to
//       extract the union of all min-cut arcs across all minimum cuts
//       (allMinCutEdges).  Merge every min-cut arc into ONE big min-cut edge
//       that ranges from the source-most boundary (union of all tails {u_i})
//       to the sink-most boundary (union of all heads {v_j}).  Build a single
//       combined sub-flow-graph through empty space spanning that big edge
//       and run the EDP search; the deduplicated union of all augmenting
//       paths' intermediate empty cells is the combined bridge — exactly the
//       cells on which idle modules will be placed to bypass the entire
//       minimum cut in a single pass.
//
//       Identify idle structural blocks (not used as pivots in augmenting
//       paths).
//
//   [3] Filter to validIdleModules (non-blocked, non-articulation).
//
//   [4] Virtual pre-verification: simulate the bridge being in place and
//       confirm that re-running Edmonds-Karp from scratch increases the flow
//       BEFORE committing to physical orchestration.
//
//   [Phase 0] Push ONE BridgeTask whose intermediatePath spans the combined
//             bridge and launch it.
//       → Forward loop: iterative round-based filling of bridge positions.
//       → Return loop:  iterative shortest-path backtracking of displaced
//                       modules.
// =============================================================================

void Catoms3DFlowBridgeCode::startup() {
    static bool hasRun = false;
    if (hasRun) return;
    hasRun = true;

    // -------- [1] XML parse --------
    TiXmlDocument *doc = Simulator::getSimulator()->getConfigDocument();
    parseTargetPositions(doc);
    parseMovingBlocks(doc);
    parseStructuralBlocks(doc);
    printTargetList();
    printMovingBlockList();
    printStructuralBlocks();

    // [viz] route all dumps to viz_flowgraph/<config_stem>/ before any
    //       observation hook fires.
    FlowGraphViz::setConfigStemFromSimulator();

    // -------- [2] FlowGraph + Edmonds-Karp + Picard-Queyranne + bridge --------
    FlowGraph* fg = new FlowGraph();
    fg->buildFlowGraph();
    fg->printNodes();
    fg->printEdges();

    // [viz] Stage 1 — initial main flow graph, before any algorithm runs.
    {
        FlowGraphViz::ModuleSets sets;
        sets.sourceModules    = &movingBlocks;
        sets.targetCells      = &targetPositions;
        sets.structuralBlocks = &structuralBlocks;
        FlowGraphViz::dump(
            *fg, "1_initial",
            "Initial main flow graph (sources=red, targets=blue, "
            "structural=hollow)",
            sets);
    }

    originalFlowValue = fg->startProcess(fg->superSource, fg->superSink, allMinCutEdges);
    fg->printIdleStructuralBlocks(idleStructuralBlocks);
    delete fg;

    // -------- [3] Idle-module filtering --------
    computeValidIdleModules();

    // -------- [4] Virtual bridge pre-verification --------
    verifyBridgeVirtually();

    // -------- [Phase 0] Launch bridge orchestration --------
    startBridgeOrchestration();
}

// =============================================================================
//  onMotionEnd() — dispatches by currentPhase
// =============================================================================

void Catoms3DFlowBridgeCode::onMotionEnd() {
    switch (currentPhase) {
        case BridgePhase::FORWARD:
            handleForwardMotionEnd(catom->position);
            break;
        case BridgePhase::RETURN:
            handleReturnMotionEnd(catom->position);
            break;
        case BridgePhase::IDLE:
            break;
    }
}
