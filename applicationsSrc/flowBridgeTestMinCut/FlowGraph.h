#ifndef FLOWGRAPH_H_
#define FLOWGRAPH_H_

// =============================================================================
//  FlowGraph
//  ---------
//  Lightweight residual-network flow graph used by the Catoms3D Flow Bridge
//  (combined min-cut-edge variant).  Carries:
//      * Edmonds–Karp max-flow with explicit augmenting-path logging.
//      * Greedy maximum Edge-Disjoint Paths (Kleinberg–Salavatipour 2004).
//      * Picard–Queyranne all-min-cuts enumeration via residual SCCs.
//      * Combined-bridge construction (sub-flow-graph over empty cells).
//
//  Node, Edge and FlowGraph were previously file-local types inside
//  Catoms3DFlowBridgeCode.cpp; they are now declared here so the multiple
//  translation units that make up the application can share them.
// =============================================================================

#include <vector>
#include <string>
#include <queue>
#include <stack>
#include <set>
#include <map>
#include <unordered_map>
#include <utility>

#include "robots/catoms3D/catoms3DSimulator.h"

// -----------------------------------------------------------------------------
//  Forward declarations
// -----------------------------------------------------------------------------
struct Edge;

// -----------------------------------------------------------------------------
//  Node
// -----------------------------------------------------------------------------
struct Node {
    short                x, y, z;
    std::vector<Edge*>   edges;
    std::string          specialKey;

    Node(short x_, short y_, short z_) : x(x_), y(y_), z(z_) {}
    Node() : x(0), y(0), z(0) {}

    std::string key() const {
        if (!specialKey.empty()) return specialKey;
        return std::to_string(x) + "," + std::to_string(y) + "," + std::to_string(z);
    }
};

// -----------------------------------------------------------------------------
//  Edge
// -----------------------------------------------------------------------------
struct Edge {
    Node *from, *to;
    int   capacity;
    Edge *rev;

    Edge(Node* f, Node* t, int cap)
        : from(f), to(t), capacity(cap), rev(nullptr) {}
};

// -----------------------------------------------------------------------------
//  FlowGraph
// -----------------------------------------------------------------------------
class FlowGraph {
public:
    static constexpr int INF_CAPACITY = 1000000000;

    std::unordered_map<std::string, Node*>   nodes;
    Node                                     *superSource = nullptr;
    Node                                     *superSink   = nullptr;
    std::vector<std::vector<Cell3DPosition>> augmentingPathPositions;

    ~FlowGraph();

    // --- Node construction --------------------------------------------------
    Node* getOrCreateNode(short x, short y, short z);
    Node* getOrCreateNode(const Cell3DPosition& pos);
    Node* getOrCreateSpecial(const std::string& name);

    // --- Edge construction --------------------------------------------------
    void addEdge(Node* from, Node* to, int capacity);
    void addEdgesRec(Node* fromNode,
                     const Cell3DPosition& fromPos,
                     std::set<std::pair<std::string, std::string>>& edgeVisited,
                     std::set<std::string>& nodeVisited);

    // --- Top-level build ----------------------------------------------------
    void buildFlowGraph();

    /**
     * Build a unit-capacity flow graph for the bridge-fill motion planner.
     *
     *   super-source -> every donor cell (cap 1)
     *   each donor expands through the empty-cell motion graph (cap 1 edges)
     *   every empty bridge cell -> super-sink (cap 1)
     *
     * Cap-1 super edges enforce "one path per donor / one module per bridge
     * cell".  Cap-1 internal edges enforce edge-disjoint paths through the
     * motion graph, so the augmenting paths discovered by Edmonds-Karp are
     * physically non-overlapping and can be dispatched concurrently.
     *
     * The emptyTargets must currently be free lattice cells; donors must
     * currently hold a module.
     */
    void buildBridgeFlowGraph(const std::vector<Cell3DPosition>& donors,
                              const std::vector<Cell3DPosition>& emptyTargets);

    // --- Capacity helpers ---------------------------------------------------
    std::unordered_map<Edge*, int> backupCapacities();

    // --- Path algorithms ----------------------------------------------------
    int  findAndPrintAugmentingPaths(Node* source, Node* sink);
    int  findGreedyEdpPaths(Node* source, Node* sink);

    // --- Connectivity helpers -----------------------------------------------
    std::set<Node*> findReachable(Node* source);

    // --- Diagnostics --------------------------------------------------------
    void printNodes();
    void printEdges();

    // --- Picard–Queyranne all-min-cuts --------------------------------------
    std::unordered_map<Node*, int> computeResidualSCCs();
    std::vector<std::pair<Cell3DPosition, Cell3DPosition>>
        findAllMinCutEdgesPicardQueyranne(
            const std::unordered_map<Edge*, int>& originalCap);

    // --- Combined-bridge construction ---------------------------------------
    void findCombinedBridge(
        const std::vector<std::pair<Cell3DPosition, Cell3DPosition>>& minCutEdges);

    // --- Pivot bookkeeping --------------------------------------------------
    std::set<Cell3DPosition> findPivotsInAugmentingPaths();
    void printIdleStructuralBlocks(std::vector<Cell3DPosition>& idleOut);

    // --- Reporting ----------------------------------------------------------
    void reportSourceSinkFlows(
        const std::unordered_map<Edge*, int>& originalCap,
        Node* source, Node* sink) const;

    // --- Pipeline driver ----------------------------------------------------
    int  startProcess(
        Node* source, Node* sink,
        std::vector<std::pair<Cell3DPosition, Cell3DPosition>>& outMinCutEdges);
};

#endif /* FLOWGRAPH_H_ */
