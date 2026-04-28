#ifndef CATOMS3DFLOWBRIDGECODE_H_
#define CATOMS3DFLOWBRIDGECODE_H_

#include <vector>
#include <string>
#include <unordered_map>
#include <map>
#include <set>
#include <queue>
#include <utility>
#include "robots/catoms3D/catoms3DSimulator.h"
#include "robots/catoms3D/catoms3DBlockCode.h"
#include "robots/catoms3D/catoms3DRotationEvents.h"

using namespace std;
using namespace Catoms3D;

/**
 * One work item: a single (min-cut edge, bridge candidate path) pair.
 * Each allBridges[e].second[p] entry generates one independent BridgeTask,
 * processed sequentially — all paths of edge e before edge e+1.
 */
struct BridgeTask {
    pair<Cell3DPosition, Cell3DPosition> minCutEdge;
    vector<Cell3DPosition>               intermediatePath; ///< ONE candidate path's intermediates
    int                                  edgeIndex;        ///< index into allBridges
    int                                  pathIndex;        ///< index within allBridges[edgeIndex].second
};

/** Phase of the bridge orchestration state machine. */
enum class BridgePhase { IDLE, FORWARD, RETURN };

/**
 * Block-code for the Catoms3D Flow Bridge application.
 *
 * Uses Edmonds-Karp max-flow to find augmenting paths from moving blocks to
 * target positions, identifies min-cut edges that represent bottlenecks, and
 * discovers bridge candidate paths through empty space that could bypass
 * those bottlenecks.
 *
 * After startup() finishes the bridge orchestration state machine takes over:
 *
 *   Phase 0 – build the BridgeTask queue (one entry per (edge, path) pair).
 *
 *   Forward loop (per BridgeTask) — iterative multi-round filling:
 *     Round R — build per-round flow graph, run Edmonds-Karp, select
 *               non-interfering assignment S_r (shortest path if only one
 *               intermediate remains), record traversal paths in travelLog,
 *               populate returnTargets, schedule forward motions.
 *
 *   Return loop — iterative shortest-path return:
 *     executeReturnIteration() repeatedly selects the moved module with the
 *     shortest BFS path back to its origin, moves it one step at a time, and
 *     repeats until all origins are restored.
 *
 *   Advance — pop next BridgeTask and repeat, or halt when queue is empty.
 */
class Catoms3DFlowBridgeCode : public Catoms3DBlockCode {
private:
    Catoms3DBlock *catom;

    // -----------------------------------------------------------------------
    // Existing analysis state
    // -----------------------------------------------------------------------
    static vector<Cell3DPosition> targetPositions;
    static vector<Cell3DPosition> movingBlocks;
    static vector<Cell3DPosition> structuralBlocks;
    static vector<Cell3DPosition> idleStructuralBlocks;
    static vector<Cell3DPosition> validIdleModules;
    static vector<pair<Cell3DPosition, Cell3DPosition>> allMinCutEdges;
    static int originalFlowValue;

    // -----------------------------------------------------------------------
    // Bridge orchestration state
    // -----------------------------------------------------------------------
    static queue<BridgeTask>   pendingBridges;
    static BridgeTask          currentBridgeTask;
    static BridgePhase         currentPhase;

    // --- Per-task traversal log (keyed by module's origin idle position) ---
    /// travelLog[origin] = complete forward path [origin, step1, …, destination]
    static map<Cell3DPosition, vector<Cell3DPosition>> travelLog;

    /// roundAssignments[r] = list of origin positions dispatched in forward round r
    static vector<vector<Cell3DPosition>>              roundAssignments;

    /// Intermediate positions currently occupied by placed modules
    static set<Cell3DPosition>                         placedIntermediates;

    // --- Forward-motion tracking (current round) ---
    /// forwardStepIndex[origin] = index in travelLog[origin] the module is currently AT
    static map<Cell3DPosition, size_t>                 forwardStepIndex;
    /// pendingArrival[expectedNextPos] = origin  (forward; erased on arrival)
    static map<Cell3DPosition, Cell3DPosition>         pendingArrival;
    /// Origins of modules still in flight for the current forward round
    static set<Cell3DPosition>                         pendingMotions;

    // --- Return-phase tracking ---
    /// returnTargets[currentPos] = originPos the module at currentPos must return to.
    /// Populated in R5 (forward dispatch); key updated as the module moves.
    static map<Cell3DPosition, Cell3DPosition>         returnTargets;
    /// Set of origin positions not yet re-occupied.
    static set<Cell3DPosition>                         pendingOrigins;
    /// Current position of the module being actively returned (one at a time).
    static Cell3DPosition                              activeReturnModule;
    /// Full BFS path for the active return: [currentPos, …, origin].
    static vector<Cell3DPosition>                      activeReturnPath;
    /// Next index to execute in activeReturnPath (1 = first move).
    static int                                         activeReturnStep;
    /// Origin that the active return module must reach.
    static Cell3DPosition                              activeReturnOrigin;

    // --- Round counter ---
    static int currentForwardRound;  ///< 0-based index of the running forward round

    // -----------------------------------------------------------------------
    // Internal orchestration steps (implemented in .cpp)
    // -----------------------------------------------------------------------
    /// Execute one forward round: build graph → EK → select → dispatch.
    static void runForwardRound();
    /// Called when all modules of a forward round have arrived.
    static void onForwardRoundComplete();
    /// Transition from forward to return phase.
    static void startReturnPhase();
    /// Select and move the next module toward its origin (iterative BFS return).
    static void executeReturnIteration();
    /// Forward onMotionEnd handler: advance step or complete round.
    static void handleForwardMotionEnd(const Cell3DPosition& arrivedAt);
    /// Return onMotionEnd handler: advance active module or start next iteration.
    static void handleReturnMotionEnd(const Cell3DPosition& arrivedAt);
    /// Verify all module origins are restored after return; then advance task.
    static void verifyAllOriginsRestored();
    /// Reset all per-BridgeTask state and set phase to IDLE.
    static void clearBridgeTaskState();

public:
    /// For each min-cut edge (u,v), stores the list of bridge candidate paths.
    static vector<pair<pair<Cell3DPosition, Cell3DPosition>,
                        vector<vector<Cell3DPosition>>>> allBridges;

    // -----------------------------------------------------------------------

    Catoms3DFlowBridgeCode(Catoms3DBlock *host);
    ~Catoms3DFlowBridgeCode() {};

    // --- Lattice helpers ---

    static vector<Cell3DPosition> findPath(const Cell3DPosition &start,
                                           const Cell3DPosition &goal);
    static bool getAllPossibleMotionsFromPosition(Cell3DPosition position,
                                                  vector<Cell3DPosition> &reachablePositions);
    static void getPivotsForMotion(const Cell3DPosition &fromPos,
                                   const Cell3DPosition &toPos,
                                   set<Cell3DPosition> &pivots);

    // --- Getters / setters ---

    static const vector<Cell3DPosition>& getTargetPositions();
    static const vector<Cell3DPosition>& getMovingBlocks();
    static const vector<Cell3DPosition>& getStructuralBlocks();
    static const vector<Cell3DPosition>& getIdleStructuralBlocks();
    static void setTargetPositions(const vector<Cell3DPosition>& positions);
    static void setMovingBlocks(const vector<Cell3DPosition>& positions);
    static void setStructuralBlocks(const vector<Cell3DPosition>& positions);
    static void setIdleStructuralBlocks(const vector<Cell3DPosition>& positions);

    // --- Startup & XML parsing ---

    /**
     * Entry point executed once. Parses XML, runs the max-flow / min-cut /
     * bridge-path pipeline, filters idle modules, then launches the bridge
     * orchestration state machine via startBridgeOrchestration().
     */
    void startup() override;

    static void parseTargetPositions(TiXmlDocument *doc);
    static void parseMovingBlocks(TiXmlDocument *doc);
    static void parseStructuralBlocks(TiXmlDocument *doc);
    static void printTargetList();
    static void printMovingBlockList();
    static void printStructuralBlocks();

    // --- Idle-module filtering ---

    static bool isModuleBlocked(const Cell3DPosition &pos);
    static bool isArticulationPoint(const Cell3DPosition &pos);
    static void computeValidIdleModules();

    // --- Bridge orchestration (public entry points) ---

    /**
     * Phase 0: Populate pendingBridges with one entry per (edge, path) pair
     * and launch the first BridgeTask.  Called once from startup().
     */
    static void startBridgeOrchestration();

    /**
     * Clear per-task state, log the banner, and kick off the forward loop by
     * calling runForwardRound() for the first round.
     */
    static void processBridgeTask(const BridgeTask &task);

    /**
     * Confirm occupancy of placed intermediates and re-run max-flow to measure
     * the improvement.  Called when the forward loop exits successfully.
     */
    static void verifyBridge();

    /**
     * Pop the next BridgeTask and call processBridgeTask(), or log completion
     * if the queue is empty.
     */
    static void advanceToNextTask();

    /**
     * Motion-end callback — dispatches to handleForwardMotionEnd() or
     * handleReturnMotionEnd() based on currentPhase.
     */
    void onMotionEnd() override;

    // -----------------------------------------------------------------------

    static BlockCode *buildNewBlockCode(BuildingBlock *host) {
        return new Catoms3DFlowBridgeCode(static_cast<Catoms3DBlock *>(host));
    }
};

#endif /* CATOMS3DFLOWBRIDGECODE_H_ */