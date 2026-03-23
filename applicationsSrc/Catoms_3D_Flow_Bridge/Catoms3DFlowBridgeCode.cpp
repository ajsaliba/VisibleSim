#include "Catoms3DFlowBridgeCode.h"
#include "robots/catoms3D/catoms3DMotionEngine.h"
#include <iostream>
#include <vector>
#include <string>
#include <unordered_map>
#include <queue>
#include <stack>
#include <set>
#include <algorithm>
#include <cassert>

using namespace std;

struct Edge;

struct Node {

    short x, y, z;
    vector<Edge*> edges;
    string specialKey;

    string key() const {

        if (!specialKey.empty()) return specialKey;
        return std::to_string(x) + "," + std::to_string(y) + "," + std::to_string(z);

    }

    Node(short x_, short y_, short z_) : x(x_), y(y_), z(z_) {}
    Node() : x(0), y(0), z(0) {}

};

struct Edge {

    Node *from, *to;
    int capacity;
    // pointer to reverse edge
    Edge *rev;

    Edge(Node* f, Node* t, int cap) : from(f), to(t), capacity(cap), rev(nullptr) {}

};


class FlowGraph {
public:

    unordered_map<string, Node*> nodes;
    // "infinity" for flow graph edges coming out from the super source and coming in to the super sink
    static constexpr int INF_CAPACITY = 1000000000;
    Node *superSource = nullptr;
    Node *superSink = nullptr;
    // Each augmenting path stored as the ordered sequence of grid positions
    // (SUPER_SOURCE and SUPER_SINK are excluded).
    vector<vector<Cell3DPosition>> augmentingPathPositions;

    ~FlowGraph() {

        // Delete all edges first
        for (auto& kv : nodes) {

            Node* n = kv.second;

            for (auto e : n->edges) {

                if (e) delete e;

            }
            n->edges.clear();

        }

        // Then delete all nodes
        for (auto& kv : nodes) {

            delete kv.second;

        }

        nodes.clear();

    }

    Node* getOrCreateNode(short x, short y, short z) {

        string k = to_string(x) + "," + to_string(y) + "," + to_string(z);
        auto it = nodes.find(k);

        if (it != nodes.end()) {

            // Node already exists, return it
            return it->second;

        }

        // Assert that this key is not present
        assert(nodes.count(k) == 0 && "Duplicate node key detected in getOrCreateNode!");

        Node* n = new Node(x, y, z);
        nodes[k] = n;

        return n;

    }

    Node* getOrCreateNode(const Cell3DPosition& pos) {

        return getOrCreateNode(pos.pt[0], pos.pt[1], pos.pt[2]);

    }

    Node* getOrCreateSpecial(const string& name) {

        if (nodes.count(name)) return nodes[name];

        Node* n = new Node(-1, -1, -1); // special
        n->specialKey = name;
        nodes[name] = n;

        return n;

    }

    void addEdge(Node* from, Node* to, int capacity) {

        Edge* e1 = new Edge(from, to, capacity);
        Edge* e2 = new Edge(to, from, 0); // reverse edge
        e1->rev = e2;
        e2->rev = e1;
        from->edges.push_back(e1);
        to->edges.push_back(e2);

    }

    // --- Build the residual flow graph ---
    void buildFlowGraph() {

        // Create super source and sink
        superSource = getOrCreateSpecial("SUPER_SOURCE");
        superSink = getOrCreateSpecial("SUPER_SINK");

        // Track visited node pairs to prevent duplicate edges
        set<pair<string, string>> edgeVisited;
        set<string> nodeVisited;

        // Recursively build graph from each moving block
        auto addEdgesRec = [&](Node* fromNode, const Cell3DPosition& fromPos, auto&& addEdgesRecRef) -> void {

            string k = fromNode->key();

            if (nodeVisited.count(k)) return;
            nodeVisited.insert(k);

            vector<Cell3DPosition> reachable;
            Catoms3DFlowBridgeCode::getAllPossibleMotionsFromPosition(fromPos, reachable);

            for (const auto& toPos : reachable) {

                Node* toNode = getOrCreateNode(toPos);

                if (!toNode) continue;

                pair<string, string> edgeKey = make_pair(fromNode->key(), toNode->key());

                if (!edgeVisited.count(edgeKey)) {

                    addEdge(fromNode, toNode, 1);
                    edgeVisited.insert(edgeKey);

                }

                addEdgesRecRef(toNode, toPos, addEdgesRecRef);

            }

        };

        // Connect super source to all moving blocks
        for (const auto& start : Catoms3DFlowBridgeCode::getMovingBlocks()) {

            Node* startNode = getOrCreateNode(start);
            pair<string, string> edgeKey = make_pair(superSource->key(), startNode->key());

            if (!edgeVisited.count(edgeKey)) {
                
                // Set capacity to "infinity" for super source edges
                addEdge(superSource, startNode, INF_CAPACITY);
                edgeVisited.insert(edgeKey);

            }

            addEdgesRec(startNode, start, addEdgesRec);

        }

        // Connect all target positions to super sink
        for (const auto& target : Catoms3DFlowBridgeCode::getTargetPositions()) {

            Node* targetNode = getOrCreateNode(target);
            pair<string, string> edgeKey = make_pair(targetNode->key(), superSink->key());

            if (!edgeVisited.count(edgeKey)) {

                // Set capacity to "infinity" for sink to super sink edges
                addEdge(targetNode, superSink, INF_CAPACITY);
                edgeVisited.insert(edgeKey);

            }

        }

    }

    unordered_map<Edge*, int> backupCapacities() {

        unordered_map<Edge*, int> originalCap;

        for (auto& kv : nodes) {

            for (Edge* e : kv.second->edges) {

                originalCap[e] = e->capacity;

            }

        }

        return originalCap;

    }

    // Helper: Restore all edge capacities
    void restoreCapacities(const unordered_map<Edge*, int>& originalCap) {

        for (auto& kv : nodes) {

            for (Edge* e : kv.second->edges) {

                e->capacity = originalCap.at(e);

            }

        }

    }

    // Helper: Find all augmenting paths and print them, updating residuals
    int findAndPrintAugmentingPaths(Node* source, Node* sink) {

        int pathNum = 0;

        cout << "\n--- Edmonds-Karp Augmenting Paths ---\n";

        while (true) {

            unordered_map<Node*, Edge*> parent;
            queue<Node*> q;

            q.push(source);
            parent[source] = nullptr;

            while (!q.empty() && parent.find(sink) == parent.end()) {

                Node* u = q.front(); q.pop();

                for (Edge* e : u->edges) {

                    if (e->capacity > 0 && parent.find(e->to) == parent.end()) {

                        parent[e->to] = e;
                        q.push(e->to);

                    }

                }

            }

            if (parent.find(sink) == parent.end()) break;

            vector<Edge*> path;
            Node* curr = sink;

            while (curr != source) {

                Edge* e = parent[curr];
                path.push_back(e);
                curr = e->from;

            }

            reverse(path.begin(), path.end());
            printAugmentingPath(source, path, ++pathNum);
            updateResiduals(path);

            // Store the grid positions of this augmenting path
            // (skip SUPER_SOURCE and SUPER_SINK)
            vector<Cell3DPosition> pathPositions;

            for (Edge* e : path) {

                if (e->from->specialKey.empty()) {

                    pathPositions.push_back(Cell3DPosition(e->from->x, e->from->y, e->from->z));

                }

            }

            // Add the last node if it's a grid node
            if (!path.empty() && path.back()->to->specialKey.empty()) {

                pathPositions.push_back(Cell3DPosition(path.back()->to->x, path.back()->to->y, path.back()->to->z));

            }
            
            augmentingPathPositions.push_back(pathPositions);

        }

        if (pathNum == 0) {

            cout << "No augmenting paths found.\n";

        }

        return pathNum;

    }

    // Helper: Print a single augmenting path
    void printAugmentingPath(Node* source, const vector<Edge*>& path, int pathNum) {

        cout << "Augmenting path " << pathNum << ": ";
        cout << source->key();

        for (Edge* e : path) {

            cout << " -> " << e->to->key();

        }

        cout << "\n";

    }

    // Helper: Update residual capacities for a path
    void updateResiduals(const vector<Edge*>& path) {

        for (Edge* e : path) {

            e->capacity -= 1;
            e->rev->capacity += 1;

        }

    }

    // Helper: Find all nodes reachable from source in residual graph
    set<Node*> findReachable(Node* source) {

        set<Node*> reachable;
        stack<Node*> s;

        s.push(source);
        reachable.insert(source);

        while (!s.empty()) {

            Node* u = s.top(); s.pop();

            for (Edge* e : u->edges) {

                if (e->capacity > 0 && !reachable.count(e->to)) {

                    reachable.insert(e->to);
                    s.push(e->to);

                }

            }

        }

        return reachable;

    }

    void printNodes() {

        cout << "\n";
        cout << "\n--- Flow Graph Nodes ---\n";

        for (const auto& kv : nodes) {

            if (kv.first == "SUPER_SOURCE") cout << "Node: SUPER_SOURCE\n";

            else if (kv.first == "SUPER_SINK") cout << "Node: SUPER_SINK\n";

            else cout << "Node: " << kv.first << "\n";

        }

        cout << "\n";

    }

    void printEdges() {

        cout << "\n";
        cout << "\n--- Flow Graph Edges (including reverse) ---\n";
        set<pair<string, string>> printed;

        for (const auto& kv : nodes) {

            for (const auto& e : kv.second->edges) {

                string from = (kv.first == "SUPER_SOURCE") ? "SUPER_SOURCE" : kv.first;
                string to;

                if (e->to == superSource) to = "SUPER_SOURCE";
                else if (e->to == superSink) to = "SUPER_SINK";
                else to = e->to->key();

                pair<string, string> edgeKey = make_pair(from, to);

                if (printed.count(edgeKey)) continue;

                cout << "Edge: " << from << " -> " << to << " (cap=" << e->capacity << ")\n";
                
                printed.insert(edgeKey);

            }

        }

        cout << "\n";

    }

    // Helper: Print min-cut edges and return them as position pairs
    vector<pair<Cell3DPosition, Cell3DPosition>> printMinCutEdges(const unordered_map<Edge*, int>& originalCap, Node* source, Node* sink) {

        vector<pair<Cell3DPosition, Cell3DPosition>> minCutEdges;
        set<Node*> reachable = findReachable(source);
        cout << "\n";
        cout << "\n--- Min-Cut Edges (from reachable to non-reachable) ---\n";
        set<pair<string, string>> minCutPrinted;

        for (auto& kv : nodes) {

            Node* u = kv.second;

            if (!reachable.count(u)) continue;

            for (Edge* e : u->edges) {

                if (e->capacity == 0 && reachable.count(e->to) == 0 && originalCap.at(e) > 0) {

                    string from = u->key();
                    string to = e->to->key();
                    pair<string, string> edgeKey = make_pair(from, to);

                    if (!minCutPrinted.count(edgeKey)) {

                        cout << "Min-cut edge: " << from << " -> " << to << "\n";
                        minCutPrinted.insert(edgeKey);

                        // Collect grid-position pairs (skip special nodes)
                        if (u->specialKey.empty() && e->to->specialKey.empty()) {

                            Cell3DPosition uPos(u->x, u->y, u->z);
                            Cell3DPosition vPos(e->to->x, e->to->y, e->to->z);
                            minCutEdges.push_back(make_pair(uPos, vPos));

                        }

                    }

                }

            }

        }

        cout << "\n";
        return minCutEdges;

    }

    /* Iterate the stored augmenting paths and, for each consecutive pair of
     positions, query the motion engine for the exact pivots that enable the
     motion.  Returns the union of all pivot positions across all paths. */
    set<Cell3DPosition> findPivotsInAugmentingPaths() {

        set<Cell3DPosition> usedPivots;

        for (const auto& path : augmentingPathPositions) {

            for (size_t i = 0; i + 1 < path.size(); ++i) {

                Catoms3DFlowBridgeCode::getPivotsForMotion(path[i], path[i + 1], usedPivots);

            }

        }

        return usedPivots;

    }

    /* Print which structural blocks are / are not used as a pivot for any
     motion within the augmenting paths (i.e. actual flow).
     Idle blocks are candidates that can be moved to build bridges. */
    void printIdleStructuralBlocks() {

        set<Cell3DPosition> usedPivots = findPivotsInAugmentingPaths();
        const auto& structural = Catoms3DFlowBridgeCode::getStructuralBlocks();

        cout << "\n";
        cout << "\n--- Idle Structural Blocks (not pivot in any augmenting path) ---\n";

        vector<Cell3DPosition> idle;

        for (const auto& pos : structural) {

            if (usedPivots.count(pos)) {

                cout << "  [PIVOT in aug. path] " << pos.to_string() << "\n";

            } else {

                cout << "  [IDLE]               " << pos.to_string() << "\n";
                idle.push_back(pos);

            }

        }

        if (idle.empty()) {

            cout << "All structural blocks serve as pivots in augmenting paths.\n";

        } else {

            cout << idle.size() << " idle structural block(s) available for bridge building.\n";

        }

        cout << "\n";

    }

    // For each min-cut edge (u,v), build a fresh flow graph with
    // SUPER_SOURCE -> u and v -> SUPER_SINK, run Edmonds-Karp, and print the
    // bridge paths (intermediate positions between u and v).
    void findBridgePaths(const vector<pair<Cell3DPosition, Cell3DPosition>>& minCutEdges) {

        cout << "\n";
        cout << "\n--- Bridge Paths for Min-Cut Edges ---\n";

        for (size_t idx = 0; idx < minCutEdges.size(); ++idx) {

            const Cell3DPosition& uPos = minCutEdges[idx].first;
            const Cell3DPosition& vPos = minCutEdges[idx].second;

            cout << "\n=== Min-cut edge " << (idx + 1) << ": "
                 << uPos.to_string() << " -> " << vPos.to_string() << " ===\n";

            // Build a dedicated flow graph for this (u, v) pair
            FlowGraph* bridgeGraph = new FlowGraph();
            bridgeGraph->superSource = bridgeGraph->getOrCreateSpecial("SUPER_SOURCE");
            bridgeGraph->superSink   = bridgeGraph->getOrCreateSpecial("SUPER_SINK");

            set<pair<string, string>> edgeVisited;
            set<string> nodeVisited;

            // Recursive exploration (same logic as buildFlowGraph)
            auto addEdgesRec = [&](Node* fromNode, const Cell3DPosition& fromPos, auto&& self) -> void {

                string k = fromNode->key();
                if (nodeVisited.count(k)) return;
                nodeVisited.insert(k);

                vector<Cell3DPosition> reachable;
                Catoms3DFlowBridgeCode::getAllPossibleMotionsFromPosition(fromPos, reachable);

                for (const auto& toPos : reachable) {

                    Node* toNode = bridgeGraph->getOrCreateNode(toPos);
                    pair<string, string> ek = make_pair(fromNode->key(), toNode->key());

                    if (!edgeVisited.count(ek)) {

                        bridgeGraph->addEdge(fromNode, toNode, 1);
                        edgeVisited.insert(ek);

                    }

                    self(toNode, toPos, self);

                }

            };

            // SUPER_SOURCE -> u
            Node* uNode = bridgeGraph->getOrCreateNode(uPos);
            bridgeGraph->addEdge(bridgeGraph->superSource, uNode, FlowGraph::INF_CAPACITY);

            // Explore reachable positions from u
            addEdgesRec(uNode, uPos, addEdgesRec);

            // v -> SUPER_SINK (only if v was discovered during exploration)
            Node* vNode = bridgeGraph->getOrCreateNode(vPos);
            bridgeGraph->addEdge(vNode, bridgeGraph->superSink, FlowGraph::INF_CAPACITY);

            // Run Edmonds-Karp
            int flow = bridgeGraph->findAndPrintAugmentingPaths(
                bridgeGraph->superSource, bridgeGraph->superSink);

            if (flow == 0) {

                cout << "  No bridge path found for this min-cut edge.\n";

            } else {

                cout << "  " << flow << " bridge path(s) found.\n";

                // Print bridge positions (excluding u, v) for each path
                for (size_t p = 0; p < bridgeGraph->augmentingPathPositions.size(); ++p) {

                    const auto& path = bridgeGraph->augmentingPathPositions[p];
                    cout << "  Bridge " << (p + 1) << " intermediate positions:";

                    bool hasBridge = false;

                    for (const auto& pos : path) {

                        if (pos != uPos && pos != vPos) {

                            cout << " " << pos.to_string();
                            hasBridge = true;

                        }

                    }

                    if (!hasBridge) cout << " (direct edge, no intermediates)";

                    cout << "\n";

                }

            }

            delete bridgeGraph;

        }

        cout << "\n";

    }

    // Edmonds-Karp: Find and print all augmenting paths from source to sink
    void startProcess(Node* source, Node* sink) {

        auto originalCap = backupCapacities();
        int pathNum = findAndPrintAugmentingPaths(source, sink);
        auto minCutEdges = printMinCutEdges(originalCap, source, sink);
        findBridgePaths(minCutEdges);

    }

};

// Static member definitions
vector<Cell3DPosition> Catoms3DFlowBridgeCode::targetPositions;
vector<Cell3DPosition> Catoms3DFlowBridgeCode::movingBlocks;
vector<Cell3DPosition> Catoms3DFlowBridgeCode::structuralBlocks;

Catoms3DFlowBridgeCode::Catoms3DFlowBridgeCode(Catoms3DBlock *host)
    : Catoms3DBlockCode(host), catom(host) {}

/*
 * Read the XML document and extracts the positions of all blocks, and store them in blockPositions
 * In addition, extracts the target list positions, and store them in targetPositions
*/
void Catoms3DFlowBridgeCode::startup() {

    static bool hasRun = false;

    if (hasRun) return;

    hasRun = true;
    TiXmlDocument *doc = Simulator::getSimulator()->getConfigDocument();

    parseTargetPositions(doc);
    parseMovingBlocks(doc);
    parseStructuralBlocks(doc);
    printTargetList();
    printMovingBlockList();
    printStructuralBlocks();
    FlowGraph *flowGraph = new FlowGraph();
    flowGraph->buildFlowGraph();
    flowGraph->printNodes();
    flowGraph->printEdges();
    flowGraph->startProcess(flowGraph->superSource, flowGraph->superSink);
    flowGraph->printIdleStructuralBlocks();

    delete flowGraph;

    return;

}

const vector<Cell3DPosition>& Catoms3DFlowBridgeCode::getTargetPositions() {

    return targetPositions;

}

const vector<Cell3DPosition>& Catoms3DFlowBridgeCode::getMovingBlocks() {

    return movingBlocks;

}

const vector<Cell3DPosition>& Catoms3DFlowBridgeCode::getStructuralBlocks() {

    return structuralBlocks;

}

void Catoms3DFlowBridgeCode::setTargetPositions(const vector<Cell3DPosition>& positions) {

    targetPositions = positions;

}

void Catoms3DFlowBridgeCode::setMovingBlocks(const vector<Cell3DPosition>& positions) {

    movingBlocks = positions;

}

void Catoms3DFlowBridgeCode::setStructuralBlocks(const vector<Cell3DPosition>& positions) {

    structuralBlocks = positions;

}

// Fetches all target positions and transform them into Cell3DPosition
void Catoms3DFlowBridgeCode::parseTargetPositions(TiXmlDocument *doc) {

    TiXmlElement *root = doc->RootElement();
    TiXmlElement *targetList = root->FirstChildElement("targetList");

    if (targetList == nullptr) {

        cout << "No target list found..." << endl;
        return;

    }

    TiXmlElement *target = targetList->FirstChildElement("target");

    if (target == nullptr) {

        cout << "Target list empty..." << endl;
        return;

    }

    TiXmlElement *block = target->FirstChildElement("cell");

    vector<Cell3DPosition> positions;
    // Iterating over all target tags | target -> target
    while (block) {

        const char *posAttr = block->Attribute("position");
        if (posAttr) {

            int x, y, z;
            if (sscanf(posAttr, "%d,%d,%d", &x, &y, &z) == 3) {

                positions.push_back(Cell3DPosition(x, y, z));

            }

        }

        block = block->NextSiblingElement("cell");

    }

    Catoms3DFlowBridgeCode::setTargetPositions(positions);
    return;

}

void Catoms3DFlowBridgeCode::parseMovingBlocks(TiXmlDocument *doc) {

    TiXmlElement *root = doc->RootElement();
    TiXmlElement *blockList = root->FirstChildElement("movingBlocks");

    if (blockList == nullptr) {

        cout << "No moving blocks found..." << endl;
        return;

    }

    TiXmlElement *block = blockList->FirstChildElement("block");
    vector<Cell3DPosition> positions;

    // Iterating over all block tags | block -> block
    while (block != nullptr) {

        const char *positionAttr = block->Attribute("position");
        if (positionAttr != nullptr) {

                int x, y, z;
                if (sscanf(positionAttr, "%d,%d,%d", &x, &y, &z) == 3) {

                    positions.push_back(Cell3DPosition(x, y, z));

                }

        }

        block = block->NextSiblingElement("block");

    }

    Catoms3DFlowBridgeCode::setMovingBlocks(positions);
    return;

}

void Catoms3DFlowBridgeCode::parseStructuralBlocks(TiXmlDocument *doc) {

    TiXmlElement *root = doc->RootElement();
    TiXmlElement *blockList = root->FirstChildElement("blockList");

    if (blockList == nullptr) {

        cout << "No structural blocks found..." << endl;
        return;

    }

    TiXmlElement *block = blockList->FirstChildElement("block");
    vector<Cell3DPosition> positions;

    // Iterating over all block tags | block -> block
    while (block != nullptr) {

        const char *positionAttr = block->Attribute("position");
        if (positionAttr != nullptr) {

            int x, y, z;

            if (sscanf(positionAttr, "%d,%d,%d", &x, &y, &z) == 3) {

                Cell3DPosition curPos(x, y, z);

                if (find(movingBlocks.begin(), movingBlocks.end(), curPos) == movingBlocks.end()) {

                    positions.push_back(curPos);

                }

            }

        }

        block = block->NextSiblingElement("block");

    }

    Catoms3DFlowBridgeCode::setStructuralBlocks(positions);
    return;

}

void Catoms3DFlowBridgeCode::printTargetList() {

    cout << "\n";
    for (Cell3DPosition targetListPos : Catoms3DFlowBridgeCode::getTargetPositions()) {

        cout << "Target position: " << targetListPos.to_string() << endl;

    }
    cout << "\n";

    return;

}

void Catoms3DFlowBridgeCode::printMovingBlockList() {

    cout << "\n";
    for (Cell3DPosition movingBlockListPos : Catoms3DFlowBridgeCode::getMovingBlocks()) {

        cout << "Moving block position: " << movingBlockListPos.to_string() << endl;

    }
    cout << "\n";

    return;

}

void Catoms3DFlowBridgeCode::printStructuralBlocks() {

    cout << "\n";
    for (Cell3DPosition structuralBlockListPos : Catoms3DFlowBridgeCode::getStructuralBlocks()) {

        cout << "Structural block position: " << structuralBlockListPos.to_string() << endl;

    }
    cout << "\n";

    return;

}

// Finds a path from start to goal using BFS and Catoms3D motion rules
vector<Cell3DPosition> Catoms3DFlowBridgeCode::findPath(const Cell3DPosition &start, const Cell3DPosition &goal) {
    // BFS that returns the path from start to goal using getAllPossibleMotionsFromPosition
    vector<Cell3DPosition> path;
    set<Cell3DPosition> visited;
    map<Cell3DPosition, Cell3DPosition> parentMap;
    queue<Cell3DPosition> q;
    q.push(start);
    visited.insert(start);
    bool found = false;
    while (!q.empty() && !found) {
        Cell3DPosition current = q.front();
        q.pop();
        if (current == goal) {
            found = true;
            break;
        }
        vector<Cell3DPosition> reachablePositions;
        bool success = getAllPossibleMotionsFromPosition(current, reachablePositions);
        if (success) {
            for (auto &pos : reachablePositions) {
                if (visited.find(pos) == visited.end()) {
                    visited.insert(pos);
                    parentMap[pos] = current;
                    q.push(pos);
                }
            }
        }
    }
    if (found) {
        cout << "Found path from " << start.to_string() << " to " << goal.to_string() << "\n";
        Cell3DPosition step = goal;
        vector<Cell3DPosition> revPath;
        while (step != start) {
            revPath.push_back(step);
            step = parentMap[step];
        }
        revPath.push_back(start);
        reverse(revPath.begin(), revPath.end());
        path = revPath;
    } else {
        cout << "No path found from " << start.to_string() << " to " << goal.to_string() << "\n";
    }
    return path;
}

// Gets all reachable positions from a given position (one Catoms3D move)
bool Catoms3DFlowBridgeCode::getAllPossibleMotionsFromPosition(Cell3DPosition position, vector<Cell3DPosition> &reachablePositions) {
    // Try to get the Catoms3DBlock at the given position
    Catoms3DBlock* mod = static_cast<Catoms3DBlock*>(BaseSimulator::getWorld()->lattice->getBlock(position));
    if (mod) {
        // For each free neighbor, check if the module can move there
        for (auto &neighPos : BaseSimulator::getWorld()->lattice->getFreeNeighborCells(position)) {
            if (mod->canMoveTo(neighPos)) {
                reachablePositions.push_back(neighPos);
            }
        }
        return !reachablePositions.empty();
    }
    // If no module at position, try to find reachable positions via active neighbors (rare)
    bool found = false;
    for (auto &neighPos : BaseSimulator::getWorld()->lattice->getActiveNeighborCells(position)) {
        Catoms3DBlock* neigh = static_cast<Catoms3DBlock*>(BaseSimulator::getWorld()->lattice->getBlock(neighPos));
        vector<Catoms3DMotionRulesLink*> vec;
        Catoms3DMotionRules motionRulesInstance;
        short conFrom = neigh->getConnectorId(position);
        motionRulesInstance.getValidMotionListFromPivot(neigh, conFrom, vec, static_cast<FCCLattice*>(BaseSimulator::getWorld()->lattice), nullptr);
        for (auto link : vec) {
            Cell3DPosition toPos;
            neigh->getNeighborPos(link->getConToID(), toPos);
            reachablePositions.push_back(toPos);
            found = true;
        }
    }
    return found;
}

/* For a motion from fromPos to toPos, finds the exact pivots that the motion
 engine would use and inserts their positions into the provided set.

 - Block exists at fromPos: calls findPivotLinkPairsForTargetCell which
   returns every (pivot, link) pair that enables the rotation.
 - No block at fromPos: iterates active neighbors as potential pivots via
   getValidMotionListFromPivot, same logic as getAllPossibleMotionsFromPosition. */
void Catoms3DFlowBridgeCode::getPivotsForMotion(const Cell3DPosition& fromPos, const Cell3DPosition& toPos, set<Cell3DPosition>& pivots) {

    auto* lattice = BaseSimulator::getWorld()->lattice;
    Catoms3DBlock* mod = static_cast<Catoms3DBlock*>(lattice->getBlock(fromPos));

    if (mod) {

        // Use the motion engine to get exact pivot-link pairs
        auto pairs = Catoms3DMotionEngine::findPivotLinkPairsForTargetCell(mod, toPos);

        for (auto& [pivot, link] : pairs) {

            if (pivot) pivots.insert(pivot->position);

        }

    } else {

        /* No block at fromPos — replicate the "no block" branch of
        getAllPossibleMotionsFromPosition and track which neighbor
        acts as pivot for this specific toPos. */
        for (auto& neighPos : lattice->getActiveNeighborCells(fromPos)) {

            Catoms3DBlock* neigh = static_cast<Catoms3DBlock*>(lattice->getBlock(neighPos));
            if (!neigh) continue;

            vector<Catoms3DMotionRulesLink*> vec;
            Catoms3DMotionRules motionRulesInstance;
            short conFrom = neigh->getConnectorId(fromPos);
            motionRulesInstance.getValidMotionListFromPivot(neigh, conFrom, vec,
                static_cast<FCCLattice*>(lattice), nullptr);

            for (auto link : vec) {

                Cell3DPosition destPos;
                neigh->getNeighborPos(link->getConToID(), destPos);

                if (destPos == toPos) {

                    pivots.insert(neighPos);

                }

            }

        }

    }

}