#ifndef CATOMS3DFLOWBRIDGECODE_H_
#define CATOMS3DFLOWBRIDGECODE_H_

#include <vector>
#include <queue>
#include <map>
#include <set>
#include <stack>
#include <unordered_map>
#include <unordered_set>
#include <functional>

#include "robots/catoms3D/catoms3DSimulator.h"
#include "robots/catoms3D/catoms3DBlockCode.h"
#include "robots/catoms3D/catoms3DMotionEngine.h"
#include "robots/catoms3D/catoms3DRotationEvents.h"

using namespace Catoms3D;

// ============================================================================
// Position hash for use in unordered containers
// ============================================================================
struct Cell3DPositionHash {
    size_t operator()(const Cell3DPosition &p) const {
        size_t h = 0;
        h ^= std::hash<short>()(p[0]) + 0x9e3779b9 + (h << 6) + (h >> 2);
        h ^= std::hash<short>()(p[1]) + 0x9e3779b9 + (h << 6) + (h >> 2);
        h ^= std::hash<short>()(p[2]) + 0x9e3779b9 + (h << 6) + (h >> 2);
        return h;
    }
};

using PosSet = std::unordered_set<Cell3DPosition, Cell3DPositionHash>;
using PosMap = std::unordered_map<Cell3DPosition, int, Cell3DPositionHash>;

// ============================================================================
// FlowGraph: Unit-capacity directed graph for max-flow / min-cut
// ============================================================================
class FlowGraph {
public:
    std::unordered_map<Cell3DPosition, int, Cell3DPositionHash> posToNode;
    std::vector<Cell3DPosition> nodeToPos;

    int source;
    int sink;
    int nodeCount;

    struct Edge {
        int to;
        int cap;
        int rev; // index of reverse edge in adj[to]
    };
    std::vector<std::vector<Edge>> adj;

    FlowGraph() : source(-1), sink(-1), nodeCount(0) {}

    void clear();
    int addNode(const Cell3DPosition &pos);
    int addVirtualNode();
    int getOrAddNode(const Cell3DPosition &pos);
    void addEdge(int from, int to, int cap);

    int maxFlow();
    std::vector<bool> getSourceReachable() const;
    std::vector<std::pair<int, int>> getMinCutEdges() const;
    std::vector<std::vector<Cell3DPosition>> extractStreamlines() const;

private:
    bool bfs(std::vector<int> &parent, std::vector<int> &parentEdge) const;
    int maxFlowValue = 0;
};

// ============================================================================
// FlowBridgeCode: The main BlockCode implementing the 5-phase algorithm
// ============================================================================
class FlowBridgeCode : public Catoms3DBlockCode {
private:
    Catoms3DBlock *module;

    // Shared algorithm state (centralized, run from leader)
    static bool algorithmStarted;
    static bool algorithmComplete;

    // Module classification sets
    static PosSet corePositions;
    static PosSet supplyPositions;
    static PosSet demandPositions;

    // Flow graph
    static FlowGraph flowGraph;

    // Bridge tracking
    static PosSet bridgePositions;

    // Iteration control
    static int iterationCount;

    // Currently moving module and its target demand position
    static Catoms3DBlock *movingModule;
    static Cell3DPosition movingTarget;

    // BFS distance map toward demand (recomputed each iteration)
    static PosMap distToDemand;

public:
    FlowBridgeCode(Catoms3DBlock *host);
    ~FlowBridgeCode();

    void startup() override;
    void onMotionEnd() override;

    // Main algorithm orchestrator (event-driven, one move at a time)
    void scheduleNextIteration();

    // Phase 1: Flow Graph Construction
    void identifySets();
    void buildFlowGraph();

    // Phase 4: Bridge Planning & Construction
    bool planAndDeployBridges();

    // Motion: find best one-step move for a supply module toward demand
    bool tryMoveSupplyModule();
    Cell3DPosition findBestMoveTarget(Catoms3DBlock *block) const;

    // Utility
    bool isStructureConnected(const PosSet &occupied,
                              const Cell3DPosition &exclude) const;
    FCCLattice* getLattice() const;
    Catoms3DBlock* getBlockAt(const Cell3DPosition &pos) const;
    PosSet getAllOccupied() const;

    // BFS distance from any position to nearest demand position
    void computeDistanceToDemand();

    static BlockCode *buildNewBlockCode(BuildingBlock *host) {
        return new FlowBridgeCode(static_cast<Catoms3DBlock*>(host));
    }
};

#endif /* CATOMS3DFLOWBRIDGECODE_H_ */
