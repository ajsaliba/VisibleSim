#ifndef CATOMS3DFLOWBRIDGECODE_H_
#define CATOMS3DFLOWBRIDGECODE_H_

#include <vector>
#include <string>
#include <unordered_map>
#include <map>
#include <set>
#include <queue>
#include <utility>
#include <climits>
#include "robots/catoms3D/catoms3DSimulator.h"
#include "robots/catoms3D/catoms3DBlockCode.h"
#include "robots/catoms3D/catoms3DRotationEvents.h"

using namespace std;
using namespace Catoms3D;

/**
 * One work item: the combined bridge.  Its intermediatePath is the
 * deduplicated union of empty cells produced by the combined sub-flow-graph
 * spanning every Picard–Queyranne min-cut arc (see findCombinedBridge).
 *
 * Only one BridgeTask is ever produced by this application — the entire
 * combined bridge is filled by idle modules during a single orchestration
 * pass.  The minCutEdge field carries a representative pair for diagnostic
 * logging only.
 */
struct BridgeTask {
    pair<Cell3DPosition, Cell3DPosition> minCutEdge;      ///< representative (u,v) for logging
    vector<Cell3DPosition>               intermediatePath; ///< combined bridge cells
    int                                  edgeIndex;        ///< always 0 (single task)
    int                                  pathIndex;        ///< always 0 (single task)
};

/** Phase of the bridge orchestration state machine. */
enum class BridgePhase { IDLE, FORWARD, RETURN };

/**
 * Donor-selection stage for the activity-diagram-driven module choice.
 *
 *   SOURCE_ONLY          — |sourceModules| > |bridge cells|.  Bridge is
 *                          filled exclusively with source modules.
 *   SRC_SIDE_IDLE_FILL   — Stage A: prioritise source-side idle modules
 *                          and fill bridge cells starting from the
 *                          sink/target-most end.
 *   SRC_MODULES_FILL     — Stage B: bridge not yet full after A.  Switch
 *                          to source modules and fill from the source-most
 *                          end.
 *   SINK_SIDE_IDLE_FILL  — Stage C: bridge not yet full after B.  Match
 *                          each remaining bridge cell with the closest
 *                          available sink/target-side idle module.
 *   SEL_DONE             — Selection state machine exhausted; no further
 *                          donor available, the forward phase terminates.
 */
enum class SelectionStage {
    SOURCE_ONLY,
    SRC_SIDE_IDLE_FILL,
    SRC_MODULES_FILL,
    SINK_SIDE_IDLE_FILL,
    SEL_DONE
};

/**
 * Block-code for the Catoms3D Flow Bridge (combined min-cut-edge variant).
 *
 * Uses Edmonds-Karp max-flow for connectivity analysis and the
 * Picard–Queyranne characterization of all minimum cuts in a network
 * (J.-C. Picard & M. Queyranne, "On the structure of all minimum cuts in a
 * network and applications", Rapport Technique EP-79-R-15, École
 * Polytechnique de Montréal, 1979) to identify every arc that belongs to
 * some minimum cut.
 *
 * Combined-bridge construction: every Picard–Queyranne min-cut arc
 * (u_i, v_j) is merged conceptually into one "big min-cut edge" that
 * ranges from the source-most boundary (the union of all tails {u_i})
 * to the sink-most boundary (the union of all heads {v_j}).  A SINGLE
 * sub-flow-graph is built over the empty-cell motion graph spanning
 * that big edge (super-source → every u_i, every v_j → super-sink,
 * expansion via addEdgesRec from each u_i with a shared nodeVisited
 * set so each motion-graph cell is walked once).  Edmonds-Karp on this
 * sub-graph enumerates augmenting paths through empty space; the
 * deduplicated union of their intermediate empty cells is the combined
 * bridge.  Idle modules are placed on those bridge cells so that the
 * entire minimum cut is bypassed in a single orchestration pass.
 *
 * Picard–Queyranne (Theorem 1 + Corollary 6):
 *   Given a maximum flow f in N=(V,A,c), define the binary relation R on V:
 *       i R j  ⇔  ((i,j) ∈ A and f_ij < c_ij)  OR  ((j,i) ∈ A and f_ji > 0)
 *   A subset S is a closure for R containing s and not t iff (S, V\S) is a
 *   minimum cut.  Equivalently (Corollary 6): a saturated arc (i,j) belongs
 *   to some minimum cut iff i and j lie in different strongly connected
 *   components of R.
 *
 * Pipeline (startup):
 *   [1] Parse XML → targetPositions, movingBlocks, structuralBlocks.
 *   [2] Build main FlowGraph (movingBlocks → targetPositions), run Edmonds-Karp.
 *   [3] Picard–Queyranne all-min-cuts detection:
 *         - compute SCCs of the residual graph
 *         - collect every saturated forward arc whose endpoints lie in
 *           different SCCs → allMinCutEdges.
 *   [4] Combined-bridge construction (findCombinedBridge):
 *         - source-most boundary = union of all min-cut tails {u_i}
 *         - sink-most   boundary = union of all min-cut heads {v_j}
 *         - build one sub-flow-graph through empty space spanning both
 *         - run Edmonds-Karp; combinedBridgePositions = deduplicated union
 *           of every augmenting path's intermediate empty cells.
 *   [5] Filter idle structural blocks to validIdleModules.
 *
 * After startup() the bridge orchestration state machine takes over:
 *
 *   Phase 0 – push the single combined BridgeTask onto pendingBridges.
 *
 *   Forward loop — iterative multi-round filling of the combined bridge:
 *     Round R — collect unoccupied bridge cells and available idle modules,
 *               build a bipartite (idle → bridge) matching via Edmonds-Karp,
 *               record traversal paths in travelLog, populate returnTargets,
 *               and dispatch forward motions concurrently.
 *
 *   Return loop — iterative shortest-path return:
 *     executeReturnIteration() repeatedly selects the moved module with the
 *     shortest BFS path back to its origin, moves it one step at a time, and
 *     repeats until all origins are restored.
 *
 *   Halt when the combined bridge task is finished.
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
    /// Union of all min-cut arcs across all minimum cuts (Picard–Queyranne).
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
    /// forwardVisited[origin] = set of positions this module has occupied during the
    /// current forward trip.  Used to exclude already-visited cells from reroute BFS
    /// so a module cannot oscillate back to a position it already passed through.
    static map<Cell3DPosition, set<Cell3DPosition>>    forwardVisited;

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
    // Activity-diagram donor-selection state (per BridgeTask)
    // -----------------------------------------------------------------------
    /// Current stage of the donor-selection state machine.
    static SelectionStage currentSelectionStage;

    /// Source modules snapshot for the current task (= movingBlocks copy).
    static vector<Cell3DPosition> sourceModulesPool;
    /// Idle modules classified as residing on the source side of the bridge.
    static vector<Cell3DPosition> srcSideIdlePool;
    /// Idle modules classified as residing on the sink/target side of the bridge.
    static vector<Cell3DPosition> sinkSideIdlePool;

    /// Bridge cells ordered with the SINK-most cell first
    /// (ascending lattice BFS distance to the nearest target/sink).
    /// Used in Stage A to fill from the sink/target-most side.
    static vector<Cell3DPosition> bridgeSinkMostOrder;
    /// Bridge cells ordered with the SOURCE-most cell first
    /// (ascending lattice BFS distance to the nearest source/moving block).
    /// Used in Stage B to fill from the source-most side.
    static vector<Cell3DPosition> bridgeSrcMostOrder;

    /// Lattice BFS distance from any reachable cell to the nearest source.
    static map<Cell3DPosition, int> distToSource;
    /// Lattice BFS distance from any reachable cell to the nearest sink.
    static map<Cell3DPosition, int> distToSink;

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

    // -----------------------------------------------------------------------
    // Activity-diagram selection helpers
    // -----------------------------------------------------------------------
    /// Multi-source BFS from `starts` through `lattice->getNeighborhood`,
    /// returning the lattice distance to every reachable cell.
    static map<Cell3DPosition, int>
        bfsDistancesFromSet(const vector<Cell3DPosition>& starts);

    /// Build per-task selection state (pools, side classification, orderings,
    /// initial SelectionStage) from `combinedBridgePositions`, `movingBlocks`,
    /// `validIdleModules`, `targetPositions`.  Called once at task start.
    static void computeBridgeSelectionState();

    /// Try to dispatch one module under the current selection stage.
    /// Returns true if a module was dispatched, false if the stage is
    /// exhausted (caller should advance to the next stage and retry).
    static bool dispatchOneUnderCurrentStage();

    /// Advance the SelectionStage to the next non-empty pool, or to
    /// SEL_DONE if all are exhausted.
    static void advanceSelectionStage();

public:
    /// The combined bridge: the deduplicated union of EMPTY cells produced by
    /// Edmonds-Karp over the merged sub-flow-graph that spans the union of
    /// all Picard–Queyranne min-cut tails on the source-most boundary and
    /// all min-cut heads on the sink-most boundary.  Idle modules are placed
    /// here; the bridge is built when every cell here is occupied.
    static vector<Cell3DPosition>          combinedBridgePositions;

    /// Per-path intermediates of the combined sub-flow-graph (diagnostic).
    /// Each entry is the ordered intermediate sequence of one augmenting
    /// path; the deduplicated union of these is combinedBridgePositions.
    static vector<vector<Cell3DPosition>>  combinedBridgePaths;

    // -----------------------------------------------------------------------

    Catoms3DFlowBridgeCode(Catoms3DBlock *host);
    ~Catoms3DFlowBridgeCode() {};

    // --- Lattice helpers ---

    static vector<Cell3DPosition> findPath(const Cell3DPosition &start,
                                           const Cell3DPosition &goal);
    /// Module-aware BFS: uses mod->canMoveTo at each step so the path is
    /// guaranteed physically traversable by this specific module from 'start'.
    /// The module must currently be physically at 'start'.
    static vector<Cell3DPosition> findPathForModule(Catoms3DBlock *mod,
                                                     const Cell3DPosition &start,
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
     * Entry point executed once. Parses XML, runs the main Edmonds-Karp flow,
     * applies the Picard–Queyranne algorithm to identify every arc belonging
     * to some minimum cut, merges every min-cut arc into one big edge,
     * builds a single combined bridge of empty cells spanning that big edge,
     * filters idle modules, and launches the bridge orchestration.
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
     * Virtual-bridge pre-verification.
     *
     * After findCombinedBridge() has populated combinedBridgePositions but
     * BEFORE any physical orchestration begins, temporarily insert phantom
     * Catoms3DBlock objects at every combined-bridge cell, rebuild a fresh
     * FlowGraph from scratch over the new lattice state, and run Edmonds-
     * Karp.  The phantoms act as pivots for surrounding empty cells, so the
     * resulting max-flow reflects what the flow would be once the physical
     * bridge is in place.  Phantoms are removed and destroyed before
     * returning — the lattice is restored byte-for-byte.
     *
     * Logs whether the virtual flow strictly exceeds originalFlowValue
     * (bridge will help), equals it (bridge does not improve flow), or is
     * lower (unexpected — typically indicates a graph-construction issue).
     */
    static void verifyBridgeVirtually();

    /**
     * Phase 0: Push the single combined BridgeTask and launch it.
     * Called once from startup().
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
     * if the queue is empty.  In this application the queue contains at most
     * one task (the combined bridge), so this terminates the run.
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
