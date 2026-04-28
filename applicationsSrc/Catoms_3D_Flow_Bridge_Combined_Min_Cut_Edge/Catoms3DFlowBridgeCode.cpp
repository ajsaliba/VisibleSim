#include "Catoms3DFlowBridgeCode.h"
#include "robots/catoms3D/catoms3DMotionEngine.h"
#include "robots/catoms3D/catoms3DRotationEvents.h"
#include "events/scheduler.h"
#include "events/events.h"
#include <iostream>
#include <vector>
#include <string>
#include <unordered_map>
#include <queue>
#include <stack>
#include <set>
#include <map>
#include <algorithm>
#include <numeric>
#include <cassert>
#include <climits>

using namespace std;

// =============================================================================
//  Internal flow-graph data structures (file-local)
// =============================================================================

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
    Edge *rev;
    Edge(Node* f, Node* t, int cap) : from(f), to(t), capacity(cap), rev(nullptr) {}
};

class FlowGraph {
public:
    unordered_map<string, Node*> nodes;
    static constexpr int INF_CAPACITY = 1000000000;
    Node *superSource = nullptr;
    Node *superSink   = nullptr;
    vector<vector<Cell3DPosition>> augmentingPathPositions;

    ~FlowGraph() {
        for (auto& kv : nodes) {
            for (auto e : kv.second->edges) { if (e) delete e; }
            kv.second->edges.clear();
        }
        for (auto& kv : nodes) delete kv.second;
        nodes.clear();
    }

    Node* getOrCreateNode(short x, short y, short z) {
        string k = to_string(x) + "," + to_string(y) + "," + to_string(z);
        auto it = nodes.find(k);
        if (it != nodes.end()) return it->second;
        Node* n = new Node(x, y, z);
        nodes[k] = n;
        return n;
    }

    Node* getOrCreateNode(const Cell3DPosition& pos) {
        return getOrCreateNode(pos.pt[0], pos.pt[1], pos.pt[2]);
    }

    Node* getOrCreateSpecial(const string& name) {
        if (nodes.count(name)) return nodes[name];
        Node* n = new Node(-1, -1, -1);
        n->specialKey = name;
        nodes[name] = n;
        return n;
    }

    void addEdge(Node* from, Node* to, int capacity) {
        Edge* e1 = new Edge(from, to, capacity);
        Edge* e2 = new Edge(to, from, 0);
        e1->rev = e2; e2->rev = e1;
        from->edges.push_back(e1);
        to->edges.push_back(e2);
    }

    void addEdgesRec(Node* fromNode, const Cell3DPosition& fromPos,
                     set<pair<string,string>>& edgeVisited,
                     set<string>& nodeVisited) {
        if (nodeVisited.count(fromNode->key())) return;
        nodeVisited.insert(fromNode->key());

        vector<Cell3DPosition> reachable;
        Catoms3DFlowBridgeCode::getAllPossibleMotionsFromPosition(fromPos, reachable);

        for (const auto& toPos : reachable) {
            Node* toNode = getOrCreateNode(toPos);
            pair<string,string> ek = {fromNode->key(), toNode->key()};
            if (!edgeVisited.count(ek)) {
                addEdge(fromNode, toNode, 1);
                edgeVisited.insert(ek);
            }
            addEdgesRec(toNode, toPos, edgeVisited, nodeVisited);
        }
    }

    void buildFlowGraph() {
        superSource = getOrCreateSpecial("SUPER_SOURCE");
        superSink   = getOrCreateSpecial("SUPER_SINK");

        set<pair<string,string>> ev;
        set<string> nv;

        for (const auto& start : Catoms3DFlowBridgeCode::getMovingBlocks()) {
            Node* startNode = getOrCreateNode(start);
            pair<string,string> ek = {superSource->key(), startNode->key()};
            if (!ev.count(ek)) { addEdge(superSource, startNode, INF_CAPACITY); ev.insert(ek); }
            addEdgesRec(startNode, start, ev, nv);
        }

        for (const auto& target : Catoms3DFlowBridgeCode::getTargetPositions()) {
            Node* targetNode = getOrCreateNode(target);
            pair<string,string> ek = {targetNode->key(), superSink->key()};
            if (!ev.count(ek)) { addEdge(targetNode, superSink, INF_CAPACITY); ev.insert(ek); }
        }
    }

    unordered_map<Edge*, int> backupCapacities() {
        unordered_map<Edge*, int> cap;
        for (auto& kv : nodes)
            for (Edge* e : kv.second->edges)
                cap[e] = e->capacity;
        return cap;
    }

    int findAndPrintAugmentingPaths(Node* source, Node* sink) {
        int pathNum = 0;
        cout << "\n--- Edmonds-Karp Augmenting Paths ---\n";

        while (true) {
            unordered_map<Node*, Edge*> parent;
            queue<Node*> q;
            q.push(source); parent[source] = nullptr;

            while (!q.empty() && !parent.count(sink)) {
                Node* u = q.front(); q.pop();
                for (Edge* e : u->edges)
                    if (e->capacity > 0 && !parent.count(e->to)) {
                        parent[e->to] = e; q.push(e->to);
                    }
            }

            if (!parent.count(sink)) break;

            vector<Edge*> path;
            Node* curr = sink;
            while (curr != source) { Edge* e = parent[curr]; path.push_back(e); curr = e->from; }
            reverse(path.begin(), path.end());
            cout << "Augmenting path " << ++pathNum << ": " << source->key();
            for (Edge* e : path) cout << " -> " << e->to->key();
            cout << "\n";
            for (Edge* e : path) { e->capacity -= 1; e->rev->capacity += 1; }

            vector<Cell3DPosition> positions;
            for (Edge* e : path)
                if (e->from->specialKey.empty())
                    positions.push_back({e->from->x, e->from->y, e->from->z});
            if (!path.empty() && path.back()->to->specialKey.empty())
                positions.push_back({path.back()->to->x,
                                     path.back()->to->y,
                                     path.back()->to->z});
            augmentingPathPositions.push_back(positions);
        }

        if (pathNum == 0) cout << "No augmenting paths found.\n";
        return pathNum;
    }

    set<Node*> findReachable(Node* source) {
        set<Node*> reachable;
        stack<Node*> s;
        s.push(source); reachable.insert(source);
        while (!s.empty()) {
            Node* u = s.top(); s.pop();
            for (Edge* e : u->edges)
                if (e->capacity > 0 && !reachable.count(e->to)) {
                    reachable.insert(e->to); s.push(e->to);
                }
        }
        return reachable;
    }

    void printNodes() {
        cout << "\n\n--- Flow Graph Nodes ---\n";
        for (const auto& kv : nodes) {
            if      (kv.first == "SUPER_SOURCE") cout << "Node: SUPER_SOURCE\n";
            else if (kv.first == "SUPER_SINK")   cout << "Node: SUPER_SINK\n";
            else                                  cout << "Node: " << kv.first << "\n";
        }
        cout << "\n";
    }

    void printEdges() {
        cout << "\n\n--- Flow Graph Edges (including reverse) ---\n";
        set<pair<string,string>> printed;
        for (const auto& kv : nodes) {
            for (const auto& e : kv.second->edges) {
                string from = (kv.first == "SUPER_SOURCE") ? "SUPER_SOURCE" : kv.first;
                string to = (e->to == superSource) ? "SUPER_SOURCE"
                          : (e->to == superSink)   ? "SUPER_SINK"
                          :                          e->to->key();
                pair<string,string> ek = {from, to};
                if (printed.count(ek)) continue;
                cout << "Edge: " << from << " -> " << to << " (cap=" << e->capacity << ")\n";
                printed.insert(ek);
            }
        }
        cout << "\n";
    }

    vector<pair<Cell3DPosition, Cell3DPosition>>
    printMinCutEdges(const unordered_map<Edge*, int>& originalCap,
                     Node* source, Node* /*sink*/) {
        vector<pair<Cell3DPosition, Cell3DPosition>> minCutEdges;
        set<Node*> reachable = findReachable(source);
        cout << "\n\n--- Min-Cut Edges ---\n";
        set<pair<string,string>> printed;
        for (auto& kv : nodes) {
            Node* u = kv.second;
            if (!reachable.count(u)) continue;
            for (Edge* e : u->edges) {
                if (e->capacity == 0 && !reachable.count(e->to) &&
                    originalCap.at(e) > 0) {
                    pair<string,string> ek = {u->key(), e->to->key()};
                    if (!printed.count(ek)) {
                        cout << "Min-cut edge: " << u->key() << " -> " << e->to->key() << "\n";
                        printed.insert(ek);
                        if (u->specialKey.empty() && e->to->specialKey.empty())
                            minCutEdges.push_back({{u->x,u->y,u->z},{e->to->x,e->to->y,e->to->z}});
                    }
                }
            }
        }
        cout << "\n";
        return minCutEdges;
    }

    set<Cell3DPosition> findPivotsInAugmentingPaths() {
        set<Cell3DPosition> usedPivots;
        for (const auto& path : augmentingPathPositions)
            for (size_t i = 0; i + 1 < path.size(); ++i)
                Catoms3DFlowBridgeCode::getPivotsForMotion(path[i], path[i+1], usedPivots);
        return usedPivots;
    }

    void printIdleStructuralBlocks(vector<Cell3DPosition>& idleOut) {
        set<Cell3DPosition> usedPivots = findPivotsInAugmentingPaths();
        cout << "\n\n--- Idle Structural Blocks ---\n";
        for (const auto& pos : Catoms3DFlowBridgeCode::getStructuralBlocks()) {
            if (usedPivots.count(pos)) cout << "  [PIVOT]  " << pos.to_string() << "\n";
            else { cout << "  [IDLE]   " << pos.to_string() << "\n"; idleOut.push_back(pos); }
        }
        cout << idleOut.size() << " idle structural block(s).\n\n";
    }

    void findBridgePaths(const vector<pair<Cell3DPosition, Cell3DPosition>>& minCutEdges) {
        Catoms3DFlowBridgeCode::allBridges.clear();
        cout << "\n\n--- Bridge Paths for Min-Cut Edges ---\n";

        for (size_t idx = 0; idx < minCutEdges.size(); ++idx) {
            const Cell3DPosition& uPos = minCutEdges[idx].first;
            const Cell3DPosition& vPos = minCutEdges[idx].second;

            cout << "\n=== Min-cut edge " << (idx+1) << ": "
                 << uPos.to_string() << " -> " << vPos.to_string() << " ===\n";

            FlowGraph* bg = new FlowGraph();
            bg->superSource = bg->getOrCreateSpecial("SUPER_SOURCE");
            bg->superSink   = bg->getOrCreateSpecial("SUPER_SINK");

            set<pair<string,string>> ev;
            set<string> nv;

            Node* uNode = bg->getOrCreateNode(uPos);
            bg->addEdge(bg->superSource, uNode, FlowGraph::INF_CAPACITY);
            bg->addEdgesRec(uNode, uPos, ev, nv);

            Node* vNode = bg->getOrCreateNode(vPos);
            bg->addEdge(vNode, bg->superSink, FlowGraph::INF_CAPACITY);

            int flow = bg->findAndPrintAugmentingPaths(bg->superSource, bg->superSink);

            vector<vector<Cell3DPosition>> pathsForEdge;
            if (flow == 0) {
                cout << "  No bridge paths found.\n";
            } else {
                cout << "  " << flow << " augmenting path(s) in sub-graph.\n";
                for (size_t p = 0; p < bg->augmentingPathPositions.size(); ++p) {
                    vector<Cell3DPosition> intermediates;
                    for (const auto& pos : bg->augmentingPathPositions[p])
                        if (pos != uPos && pos != vPos)
                            intermediates.push_back(pos);
                    if (intermediates.empty()) {
                        cout << "  Path " << (p+1) << ": direct edge (skipped)\n";
                    } else {
                        cout << "  Path " << (p+1) << " intermediates:";
                        for (const auto& ip : intermediates) cout << " " << ip.to_string();
                        cout << "\n";
                        pathsForEdge.push_back(intermediates);
                    }
                }
            }
            Catoms3DFlowBridgeCode::allBridges.push_back({minCutEdges[idx], pathsForEdge});
            delete bg;
        }

        size_t total = 0;
        for (const auto& e : Catoms3DFlowBridgeCode::allBridges)
            total += e.second.size();
        cout << "\n  Total bridge candidate paths: " << total << "\n\n";
    }

    int startProcess(Node* source, Node* sink,
                     vector<pair<Cell3DPosition,Cell3DPosition>>& outMinCutEdges) {
        auto originalCap = backupCapacities();
        int flow = findAndPrintAugmentingPaths(source, sink);
        outMinCutEdges = printMinCutEdges(originalCap, source, sink);
        findBridgePaths(outMinCutEdges);
        return flow;
    }
};

// =============================================================================
//  Static member definitions
// =============================================================================

// --- existing analysis state ---
vector<Cell3DPosition> Catoms3DFlowBridgeCode::targetPositions;
vector<Cell3DPosition> Catoms3DFlowBridgeCode::movingBlocks;
vector<Cell3DPosition> Catoms3DFlowBridgeCode::structuralBlocks;
vector<Cell3DPosition> Catoms3DFlowBridgeCode::idleStructuralBlocks;
vector<Cell3DPosition>                      Catoms3DFlowBridgeCode::validIdleModules;
vector<pair<Cell3DPosition,Cell3DPosition>> Catoms3DFlowBridgeCode::allMinCutEdges;
int                                         Catoms3DFlowBridgeCode::originalFlowValue = 0;
vector<pair<pair<Cell3DPosition,Cell3DPosition>, vector<vector<Cell3DPosition>>>>
    Catoms3DFlowBridgeCode::allBridges;

// --- orchestration ---
queue<BridgeTask>                           Catoms3DFlowBridgeCode::pendingBridges;
BridgeTask                                  Catoms3DFlowBridgeCode::currentBridgeTask;
BridgePhase                                 Catoms3DFlowBridgeCode::currentPhase = BridgePhase::IDLE;

map<Cell3DPosition, vector<Cell3DPosition>> Catoms3DFlowBridgeCode::travelLog;
vector<vector<Cell3DPosition>>              Catoms3DFlowBridgeCode::roundAssignments;
set<Cell3DPosition>                         Catoms3DFlowBridgeCode::placedIntermediates;

map<Cell3DPosition, size_t>                 Catoms3DFlowBridgeCode::forwardStepIndex;
map<Cell3DPosition, Cell3DPosition>         Catoms3DFlowBridgeCode::pendingArrival;
set<Cell3DPosition>                         Catoms3DFlowBridgeCode::pendingMotions;

map<Cell3DPosition, Cell3DPosition>         Catoms3DFlowBridgeCode::returnTargets;
set<Cell3DPosition>                         Catoms3DFlowBridgeCode::pendingOrigins;
Cell3DPosition                              Catoms3DFlowBridgeCode::activeReturnModule;
vector<Cell3DPosition>                      Catoms3DFlowBridgeCode::activeReturnPath;
int                                         Catoms3DFlowBridgeCode::activeReturnStep = 0;
Cell3DPosition                              Catoms3DFlowBridgeCode::activeReturnOrigin;

int                                         Catoms3DFlowBridgeCode::currentForwardRound = 0;

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
//  [1] Parse XML  → targetPositions, movingBlocks, structuralBlocks.
//  [2] Build main FlowGraph (movingBlocks → targetPositions), run Edmonds-Karp.
//  [3] Extract min-cut edges.
//  [4] Find bridge paths through empty space (findBridgePaths).
//  [5] Identify idle structural blocks (not used as pivots in augmenting paths).
//  [6] Filter to validIdleModules (non-blocked, non-articulation).
//  [Phase 0] Build BridgeTask queue (one per (edge,path) pair); launch first task.
//    → Forward loop: iterative round-based filling of intermediate positions.
//    → Return loop:  reverse-order round-based backtracking of each module.
//    → Advance to next BridgeTask, or halt.
//
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

    FlowGraph* fg = new FlowGraph();
    fg->buildFlowGraph();
    fg->printNodes();
    fg->printEdges();

    originalFlowValue = fg->startProcess(fg->superSource, fg->superSink, allMinCutEdges);
    fg->printIdleStructuralBlocks(idleStructuralBlocks);
    delete fg;

    computeValidIdleModules();

    // Phase 0 — launch bridge orchestration
    startBridgeOrchestration();
}

// =============================================================================
//  Idle-module filtering
// =============================================================================

bool Catoms3DFlowBridgeCode::isModuleBlocked(const Cell3DPosition& pos) {
    vector<Cell3DPosition> r;
    return !getAllPossibleMotionsFromPosition(pos, r);
}

bool Catoms3DFlowBridgeCode::isArticulationPoint(const Cell3DPosition& pos) {
    auto* lattice = BaseSimulator::getWorld()->lattice;

    vector<Cell3DPosition> remaining;
    for (const auto& p : structuralBlocks) if (p != pos) remaining.push_back(p);
    for (const auto& p : movingBlocks)     remaining.push_back(p);

    if (remaining.empty()) return false;

    set<Cell3DPosition> moduleSet(remaining.begin(), remaining.end());
    set<Cell3DPosition> visited;
    queue<Cell3DPosition> q;
    q.push(remaining[0]);
    visited.insert(remaining[0]);

    while (!q.empty()) {
        Cell3DPosition cur = q.front(); q.pop();
        for (const auto& nb : lattice->getActiveNeighborCells(cur)) {
            if (nb != pos && moduleSet.count(nb) && !visited.count(nb)) {
                visited.insert(nb);
                q.push(nb);
            }
        }
    }

    return visited.size() < moduleSet.size();
}

void Catoms3DFlowBridgeCode::computeValidIdleModules() {
    validIdleModules.clear();
    cout << "\n--- Computing Valid Idle Modules ---\n";
    for (const auto& pos : idleStructuralBlocks) {
        if (isModuleBlocked(pos)) {
            cout << "  [BLOCKED]            " << pos.to_string() << "\n";
            continue;
        }
        if (isArticulationPoint(pos)) {
            cout << "  [ARTICULATION_POINT] " << pos.to_string() << "\n";
            continue;
        }
        cout << "  [VALID_IDLE]         " << pos.to_string() << "\n";
        validIdleModules.push_back(pos);
    }
    cout << validIdleModules.size() << " valid idle module(s).\n\n";
}

// =============================================================================
//  Getters / setters
// =============================================================================

const vector<Cell3DPosition>& Catoms3DFlowBridgeCode::getTargetPositions()      { return targetPositions; }
const vector<Cell3DPosition>& Catoms3DFlowBridgeCode::getMovingBlocks()         { return movingBlocks; }
const vector<Cell3DPosition>& Catoms3DFlowBridgeCode::getStructuralBlocks()     { return structuralBlocks; }
const vector<Cell3DPosition>& Catoms3DFlowBridgeCode::getIdleStructuralBlocks() { return idleStructuralBlocks; }
void Catoms3DFlowBridgeCode::setTargetPositions(const vector<Cell3DPosition>& p)      { targetPositions = p; }
void Catoms3DFlowBridgeCode::setMovingBlocks(const vector<Cell3DPosition>& p)         { movingBlocks = p; }
void Catoms3DFlowBridgeCode::setStructuralBlocks(const vector<Cell3DPosition>& p)     { structuralBlocks = p; }
void Catoms3DFlowBridgeCode::setIdleStructuralBlocks(const vector<Cell3DPosition>& p) { idleStructuralBlocks = p; }

// =============================================================================
//  XML parsing helpers
// =============================================================================

void Catoms3DFlowBridgeCode::parseTargetPositions(TiXmlDocument *doc) {
    TiXmlElement *root = doc->RootElement();
    TiXmlElement *targetList = root->FirstChildElement("targetList");
    if (!targetList) { cout << "No target list found.\n"; return; }
    TiXmlElement *target = targetList->FirstChildElement("target");
    if (!target) { cout << "Target list empty.\n"; return; }
    TiXmlElement *block = target->FirstChildElement("cell");
    vector<Cell3DPosition> positions;
    while (block) {
        const char *a = block->Attribute("position");
        if (a) { int x,y,z; if (sscanf(a,"%d,%d,%d",&x,&y,&z)==3) positions.push_back({(short)x,(short)y,(short)z}); }
        block = block->NextSiblingElement("cell");
    }
    setTargetPositions(positions);
}

void Catoms3DFlowBridgeCode::parseMovingBlocks(TiXmlDocument *doc) {
    TiXmlElement *root = doc->RootElement();
    TiXmlElement *blockList = root->FirstChildElement("movingBlocks");
    if (!blockList) { cout << "No moving blocks found.\n"; return; }
    TiXmlElement *block = blockList->FirstChildElement("block");
    vector<Cell3DPosition> positions;
    while (block) {
        const char *a = block->Attribute("position");
        if (a) { int x,y,z; if (sscanf(a,"%d,%d,%d",&x,&y,&z)==3) positions.push_back({(short)x,(short)y,(short)z}); }
        block = block->NextSiblingElement("block");
    }
    setMovingBlocks(positions);
}

void Catoms3DFlowBridgeCode::parseStructuralBlocks(TiXmlDocument *doc) {
    TiXmlElement *root = doc->RootElement();
    TiXmlElement *blockList = root->FirstChildElement("blockList");
    if (!blockList) { cout << "No structural blocks found.\n"; return; }
    TiXmlElement *block = blockList->FirstChildElement("block");
    vector<Cell3DPosition> positions;
    while (block) {
        const char *a = block->Attribute("position");
        if (a) {
            int x,y,z;
            if (sscanf(a,"%d,%d,%d",&x,&y,&z)==3) {
                Cell3DPosition p((short)x,(short)y,(short)z);
                if (find(movingBlocks.begin(), movingBlocks.end(), p) == movingBlocks.end())
                    positions.push_back(p);
            }
        }
        block = block->NextSiblingElement("block");
    }
    setStructuralBlocks(positions);
}

void Catoms3DFlowBridgeCode::printTargetList() {
    cout << "\n";
    for (const auto& p : getTargetPositions()) cout << "Target position: " << p.to_string() << "\n";
    cout << "\n";
}
void Catoms3DFlowBridgeCode::printMovingBlockList() {
    cout << "\n";
    for (const auto& p : getMovingBlocks()) cout << "Moving block: " << p.to_string() << "\n";
    cout << "\n";
}
void Catoms3DFlowBridgeCode::printStructuralBlocks() {
    cout << "\n";
    for (const auto& p : getStructuralBlocks()) cout << "Structural block: " << p.to_string() << "\n";
    cout << "\n";
}

// =============================================================================
//  Motion / lattice helpers
// =============================================================================

vector<Cell3DPosition> Catoms3DFlowBridgeCode::findPath(const Cell3DPosition &start,
                                                         const Cell3DPosition &goal) {
    vector<Cell3DPosition> path;
    set<Cell3DPosition> visited;
    map<Cell3DPosition, Cell3DPosition> parentMap;
    queue<Cell3DPosition> q;
    q.push(start);
    visited.insert(start);
    bool found = false;
    while (!q.empty() && !found) {
        Cell3DPosition current = q.front(); q.pop();
        if (current == goal) { found = true; break; }
        vector<Cell3DPosition> reachable;
        if (getAllPossibleMotionsFromPosition(current, reachable)) {
            for (auto &pos : reachable) {
                if (!visited.count(pos)) {
                    visited.insert(pos);
                    parentMap[pos] = current;
                    q.push(pos);
                }
            }
        }
    }
    if (found) {
        Cell3DPosition step = goal;
        vector<Cell3DPosition> rev;
        while (step != start) { rev.push_back(step); step = parentMap[step]; }
        rev.push_back(start);
        reverse(rev.begin(), rev.end());
        path = rev;
    }
    return path;
}

bool Catoms3DFlowBridgeCode::getAllPossibleMotionsFromPosition(
    Cell3DPosition position, vector<Cell3DPosition> &reachablePositions) {

    Catoms3DBlock* mod = static_cast<Catoms3DBlock*>(
        BaseSimulator::getWorld()->lattice->getBlock(position));

    if (mod) {
        for (auto& nb : BaseSimulator::getWorld()->lattice->getFreeNeighborCells(position))
            if (mod->canMoveTo(nb)) reachablePositions.push_back(nb);
        return !reachablePositions.empty();
    }

    bool found = false;
    for (auto& nb : BaseSimulator::getWorld()->lattice->getActiveNeighborCells(position)) {
        Catoms3DBlock* neigh = static_cast<Catoms3DBlock*>(
            BaseSimulator::getWorld()->lattice->getBlock(nb));
        vector<Catoms3DMotionRulesLink*> vec;
        Catoms3DMotionRules motionRules;
        short conFrom = neigh->getConnectorId(position);
        motionRules.getValidMotionListFromPivot(
            neigh, conFrom, vec,
            static_cast<FCCLattice*>(BaseSimulator::getWorld()->lattice), nullptr);
        for (auto link : vec) {
            Cell3DPosition toPos;
            neigh->getNeighborPos(link->getConToID(), toPos);
            reachablePositions.push_back(toPos);
            found = true;
        }
    }
    return found;
}

void Catoms3DFlowBridgeCode::getPivotsForMotion(
    const Cell3DPosition& fromPos, const Cell3DPosition& toPos,
    set<Cell3DPosition>& pivots) {

    auto* lattice = BaseSimulator::getWorld()->lattice;
    Catoms3DBlock* mod = static_cast<Catoms3DBlock*>(lattice->getBlock(fromPos));

    if (mod) {
        for (auto& [pivot, link] :
             Catoms3DMotionEngine::findPivotLinkPairsForTargetCell(mod, toPos))
            if (pivot) pivots.insert(pivot->position);
    } else {
        for (auto& nb : lattice->getActiveNeighborCells(fromPos)) {
            Catoms3DBlock* neigh = static_cast<Catoms3DBlock*>(lattice->getBlock(nb));
            if (!neigh) continue;
            vector<Catoms3DMotionRulesLink*> vec;
            Catoms3DMotionRules motionRules;
            short conFrom = neigh->getConnectorId(fromPos);
            motionRules.getValidMotionListFromPivot(
                neigh, conFrom, vec, static_cast<FCCLattice*>(lattice), nullptr);
            for (auto link : vec) {
                Cell3DPosition dest;
                neigh->getNeighborPos(link->getConToID(), dest);
                if (dest == toPos) pivots.insert(nb);
            }
        }
    }
}

// =============================================================================
//  File-local helper: non-interfering path selection
// =============================================================================

/**
 * Greedy selection of the largest non-interfering subset from candidate paths.
 *
 * Criteria:
 *   (a) Distinct destination per path.
 *   (b) Distinct start (idle module) per path.
 *   (c) No two selected paths share any position along their full length.
 *   (d) No selected destination is a lattice neighbour of another selected
 *       destination OR of any position in occupiedNeighbors.
 *
 * Paths are processed in ascending length order.
 * Returns a vector of (start, dest) pairs for the accepted paths.
 */
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
//  Phase 0 — start orchestration
// =============================================================================

void Catoms3DFlowBridgeCode::startBridgeOrchestration() {
    cout << "\n=== [Phase 0] Building bridge task queue ===\n";

    while (!pendingBridges.empty()) pendingBridges.pop();

    // One BridgeTask per (min-cut edge, candidate path) pair.
    for (size_t e = 0; e < allBridges.size(); ++e) {
        const auto& [edge, paths] = allBridges[e];
        for (size_t p = 0; p < paths.size(); ++p) {
            if (!paths[p].empty())
                pendingBridges.push({ edge, paths[p], (int)e, (int)p });
        }
    }

    cout << pendingBridges.size() << " bridge task(s) queued.\n";

    if (pendingBridges.empty()) {
        cout << "No bridge tasks to process. Halting.\n";
        return;
    }

    BridgeTask first = pendingBridges.front();
    pendingBridges.pop();
    processBridgeTask(first);
}

// =============================================================================
//  processBridgeTask — initialise per-task state and start forward round 0
// =============================================================================

void Catoms3DFlowBridgeCode::processBridgeTask(const BridgeTask& task) {
    currentBridgeTask = task;

    int totalEdges  = (int)allBridges.size();
    int pathsInEdge = (task.edgeIndex < totalEdges)
                      ? (int)allBridges[task.edgeIndex].second.size() : 0;

    cout << "\n=== [BridgeTask] Min-cut edge " << (task.edgeIndex + 1) << "/" << totalEdges
         << ", Candidate path " << (task.pathIndex + 1) << "/" << pathsInEdge << " ===\n";
    cout << "Intermediates:";
    for (const auto& p : task.intermediatePath) cout << " " << p.to_string();
    cout << "\n";

    // --- Clear all per-task state ---
    travelLog.clear();
    roundAssignments.clear();
    placedIntermediates.clear();
    pendingMotions.clear();
    pendingArrival.clear();
    forwardStepIndex.clear();
    returnTargets.clear();
    pendingOrigins.clear();
    activeReturnStep    = 0;
    currentForwardRound = 0;
    currentPhase        = BridgePhase::FORWARD;

    // --- Start first forward round ---
    runForwardRound();
}

// =============================================================================
//  Forward loop — run one round
// =============================================================================

void Catoms3DFlowBridgeCode::runForwardRound() {
    auto* lattice = BaseSimulator::getWorld()->lattice;

    int totalEdges  = (int)allBridges.size();
    int pathsInEdge = (currentBridgeTask.edgeIndex < totalEdges)
                      ? (int)allBridges[currentBridgeTask.edgeIndex].second.size() : 0;

    // --- R1: identify unoccupied intermediate positions ---
    vector<Cell3DPosition> unoccupied;
    for (const auto& pos : currentBridgeTask.intermediatePath) {
        if (!lattice->getBlock(pos))
            unoccupied.push_back(pos);
    }

    if (unoccupied.empty()) {
        cout << "[Forward] All intermediate positions filled after "
             << currentForwardRound << " round(s).\n";
        verifyBridge();
        startReturnPhase();
        return;
    }

    // --- R1: identify available idle modules (not yet dispatched) ---
    vector<Cell3DPosition> availableIdle;
    for (const auto& pos : validIdleModules) {
        if (lattice->getBlock(pos) != nullptr)   // still at its original idle position
            availableIdle.push_back(pos);
    }

    // --- R7: deadlock guard ---
    if (availableIdle.empty()) {
        cout << "[WARNING] BridgeTask [" << (currentBridgeTask.edgeIndex+1) << ","
             << (currentBridgeTask.pathIndex+1) << "]: deadlock — "
             << unoccupied.size() << " intermediate position(s) unreachable."
             << " Skipping remaining positions.\n";
        verifyBridge();
        startReturnPhase();
        return;
    }

    bool singleTarget = (unoccupied.size() == 1);

    if (singleTarget) {
        cout << "\n=== [Forward Round " << (currentForwardRound+1) << " — SINGLE TARGET] BridgeTask "
             << (currentBridgeTask.edgeIndex+1) << "/" << totalEdges
             << ", Path " << (currentBridgeTask.pathIndex+1) << "/" << pathsInEdge << " ===\n";
        cout << "  Single remaining position: " << unoccupied[0].to_string() << ".\n";
    } else {
        cout << "\n=== [Forward Round " << (currentForwardRound+1) << "] BridgeTask "
             << (currentBridgeTask.edgeIndex+1) << "/" << totalEdges
             << ", Path " << (currentBridgeTask.pathIndex+1) << "/" << pathsInEdge << " ===\n";
    }

    // --- R2: build per-round flow graph ---
    FlowGraph* bg = new FlowGraph();
    bg->superSource = bg->getOrCreateSpecial("SUPER_SOURCE");
    bg->superSink   = bg->getOrCreateSpecial("SUPER_SINK");

    set<pair<string,string>> ev;
    set<string> nv;

    for (const auto& idlePos : availableIdle) {
        Node* startNode = bg->getOrCreateNode(idlePos);
        pair<string,string> srcEk = {"SUPER_SOURCE", startNode->key()};
        if (!ev.count(srcEk)) {
            bg->addEdge(bg->superSource, startNode, FlowGraph::INF_CAPACITY);
            ev.insert(srcEk);
        }
        bg->addEdgesRec(startNode, idlePos, ev, nv);
    }

    for (const auto& intPos : unoccupied) {
        Node* intNode = bg->getOrCreateNode(intPos);
        pair<string,string> sinkEk = {intNode->key(), "SUPER_SINK"};
        if (!ev.count(sinkEk)) {
            bg->addEdge(intNode, bg->superSink, FlowGraph::INF_CAPACITY);
            ev.insert(sinkEk);
        }
    }

    // --- R3: Edmonds-Karp (always runs, even for single target) ---
    int flow = bg->findAndPrintAugmentingPaths(bg->superSource, bg->superSink);
    vector<vector<Cell3DPosition>> roundPaths = bg->augmentingPathPositions;
    delete bg;

    if (singleTarget)
        cout << "  " << flow << " augmenting path(s) found. Selecting shortest.\n";

    // --- R7: deadlock guard (no flow) ---
    if (flow == 0 || roundPaths.empty()) {
        cout << "[WARNING] BridgeTask [" << (currentBridgeTask.edgeIndex+1) << ","
             << (currentBridgeTask.pathIndex+1) << "]: deadlock — "
             << unoccupied.size() << " intermediate position(s) unreachable."
             << " Skipping remaining positions.\n";
        verifyBridge();
        startReturnPhase();
        return;
    }

    // --- R4: selection ---
    vector<pair<Cell3DPosition, Cell3DPosition>> S_r;

    if (singleTarget) {
        // Select shortest path; break ties by lexicographic ordering of source position.
        // Criteria (c) and (d) are vacuous for a single assignment and are omitted.
        auto bestIt = roundPaths.begin();
        for (auto it = roundPaths.begin() + 1; it != roundPaths.end(); ++it) {
            if (it->size() < bestIt->size() ||
                (it->size() == bestIt->size() && it->front() < bestIt->front()))
                bestIt = it;
        }
        S_r.push_back({bestIt->front(), bestIt->back()});
        cout << "  [SELECTED] " << bestIt->front().to_string()
             << " → " << bestIt->back().to_string()
             << " (" << (bestIt->size() - 1) << " steps)\n";
    } else {
        S_r = findNonInterferingAssignment(roundPaths, placedIntermediates);
        if (S_r.empty()) {
            cout << "[WARNING] BridgeTask [" << (currentBridgeTask.edgeIndex+1) << ","
                 << (currentBridgeTask.pathIndex+1) << "]: no valid non-interfering assignment."
                 << " Deadlock — skipping remaining positions.\n";
            verifyBridge();
            startReturnPhase();
            return;
        }
        cout << "  Assigning " << S_r.size() << " module(s) this round.\n";
    }

    // --- R5: record traversal paths in travelLog; populate returnTargets ---
    vector<Cell3DPosition> roundOrigins;
    for (auto& [origin, dest] : S_r) {
        vector<Cell3DPosition> path = findPath(origin, dest);
        if (path.size() < 2) {
            cerr << "[Forward] WARNING: findPath returned no usable path for "
                 << origin.to_string() << " -> " << dest.to_string() << "\n";
            continue;
        }
        travelLog[origin]        = path;
        forwardStepIndex[origin] = 0;
        // Pre-record: module currently at 'origin' must return there from 'dest'.
        // returnTargets key will be updated as the module moves (in handleForwardMotionEnd).
        returnTargets[dest] = origin;
        roundOrigins.push_back(origin);

        cout << "  [DISPATCH] " << origin.to_string()
             << " → " << dest.to_string()
             << " via " << path.size() << "-step path\n";
    }

    if (roundOrigins.empty()) {
        verifyBridge();
        startReturnPhase();
        return;
    }

    roundAssignments.push_back(roundOrigins);

    // --- R6: pre-dispatch destination assertion, then schedule ---
    pendingMotions.clear();
    pendingArrival.clear();

    // Bug 1 fix: verify all destinations are unoccupied before scheduling any module.
    for (const auto& origin : roundOrigins) {
        Cell3DPosition dest = travelLog[origin].back();
        if (lattice->getBlock(dest) != nullptr) {
            cerr << "[FATAL] Destination " << dest.to_string()
                 << " already occupied before dispatch. Selection bug — aborting task.\n";
            clearBridgeTaskState();
            advanceToNextTask();
            return;
        }
    }

    auto* sched = getScheduler();

    for (const auto& origin : roundOrigins) {
        const auto& path = travelLog[origin];
        Cell3DPosition nextPos = path[1];

        Catoms3DBlock* mod = static_cast<Catoms3DBlock*>(lattice->getBlock(origin));
        if (!mod) {
            cerr << "[Forward] Module not found at " << origin.to_string() << "\n";
            continue;
        }
        if (!mod->canMoveTo(nextPos)) {
            cerr << "[Forward] Module at " << origin.to_string()
                 << " cannot move to " << nextPos.to_string() << ".\n";
            continue;
        }

        pendingMotions.insert(origin);
        pendingArrival[nextPos] = origin;

        try {
            sched->schedule(new Catoms3DRotationStartEvent(sched->now(), mod, nextPos));
        } catch (const exception& e) {
            cerr << "[Forward] Exception scheduling rotation: " << e.what() << "\n";
            pendingArrival.erase(nextPos);
            pendingMotions.erase(origin);
        }
    }

    if (pendingMotions.empty()) {
        onForwardRoundComplete();
    }
}

// =============================================================================
//  Forward round completion
// =============================================================================

void Catoms3DFlowBridgeCode::onForwardRoundComplete() {
    // Mark newly arrived intermediates as placed
    for (const auto& origin : roundAssignments[currentForwardRound]) {
        if (travelLog.count(origin)) {
            Cell3DPosition dest = travelLog[origin].back();
            placedIntermediates.insert(dest);
        }
    }
    currentForwardRound++;
    runForwardRound();   // next round (or transition to return if all done)
}

// =============================================================================
//  Bridge verification (called when forward loop exits)
// =============================================================================

void Catoms3DFlowBridgeCode::verifyBridge() {
    cout << "\n=== [Bridge Verification] ===\n";
    auto* lattice = BaseSimulator::getWorld()->lattice;

    for (const auto& pos : currentBridgeTask.intermediatePath) {
        if (lattice->getBlock(pos))
            cout << "  [OK]      " << pos.to_string() << " occupied.\n";
        else
            cerr << "  [MISSING] " << pos.to_string() << " NOT occupied.\n";
    }

    cout << "  Re-running max-flow (baseline = " << originalFlowValue << ")...\n";
    FlowGraph* fg = new FlowGraph();
    fg->buildFlowGraph();
    int newFlow = fg->findAndPrintAugmentingPaths(fg->superSource, fg->superSink);
    delete fg;

    if (newFlow > originalFlowValue)
        cout << "  Flow increased: " << originalFlowValue << " → " << newFlow << ". Bridge successful!\n";
    else if (newFlow == originalFlowValue)
        cout << "  Flow unchanged at " << newFlow << ".\n";
    else
        cerr << "  Flow decreased: " << originalFlowValue << " → " << newFlow << ". Unexpected!\n";
}

// =============================================================================
//  Transition to return phase — initialize returnTargets tracking
// =============================================================================

void Catoms3DFlowBridgeCode::startReturnPhase() {
    if (returnTargets.empty()) {
        cout << "\n[Return Phase] No modules were dispatched. Advancing.\n";
        clearBridgeTaskState();
        advanceToNextTask();
        return;
    }

    int totalEdges  = (int)allBridges.size();
    int pathsInEdge = (currentBridgeTask.edgeIndex < totalEdges)
                      ? (int)allBridges[currentBridgeTask.edgeIndex].second.size() : 0;

    cout << "\n=== [Return Phase] BridgeTask "
         << (currentBridgeTask.edgeIndex+1) << "/" << totalEdges
         << ", Path " << (currentBridgeTask.pathIndex+1) << "/" << pathsInEdge << " ===\n";
    cout << "  " << returnTargets.size() << " module(s) to return.\n";

    pendingOrigins.clear();
    for (const auto& [currentPos, origin] : returnTargets)
        pendingOrigins.insert(origin);

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

    // Step Ret.1 — select module with shortest BFS path back to its origin
    Cell3DPosition bestOrigin, bestModulePos;
    vector<Cell3DPosition> bestPath;
    int bestLen = INT_MAX;

    for (const auto& [currentPos, origin] : returnTargets) {
        if (!pendingOrigins.count(origin)) continue;
        vector<Cell3DPosition> path = findPath(currentPos, origin);
        if (!path.empty() && (int)path.size() < bestLen) {
            bestLen       = (int)path.size();
            bestOrigin    = origin;
            bestModulePos = currentPos;
            bestPath      = path;
        }
    }

    if (bestPath.empty()) {
        cout << "  [RETURN WARNING] No path to any pending origin. Deadlock.\n";
        for (const auto& origin : pendingOrigins)
            cout << "  [SKIPPED ORIGIN] " << origin.to_string() << "\n";
        pendingOrigins.clear();
        verifyAllOriginsRestored();
        return;
    }

    // Step Ret.2 — schedule first step of selected module
    Catoms3DBlock* mod = static_cast<Catoms3DBlock*>(lattice->getBlock(bestModulePos));
    if (!mod) {
        cerr << "[RETURN ERROR] No module at " << bestModulePos.to_string() << "\n";
        pendingOrigins.erase(bestOrigin);
        returnTargets.erase(bestModulePos);
        executeReturnIteration();
        return;
    }

    Cell3DPosition nextPos = bestPath[1];
    if (!mod->canMoveTo(nextPos)) {
        // Recompute path
        bestPath = findPath(bestModulePos, bestOrigin);
        if (bestPath.empty() || bestPath.size() < 2) {
            cout << "  [RETURN WARNING] Cannot move " << bestModulePos.to_string()
                 << " toward origin " << bestOrigin.to_string() << ". Skipping.\n";
            pendingOrigins.erase(bestOrigin);
            returnTargets.erase(bestModulePos);
            executeReturnIteration();
            return;
        }
        nextPos = bestPath[1];
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

    // Update returnTargets: module moved from activeReturnModule to newPos
    auto rtIt = returnTargets.find(activeReturnModule);
    if (rtIt != returnTargets.end()) {
        Cell3DPosition orig = rtIt->second;
        returnTargets.erase(rtIt);
        returnTargets[newPos] = orig;
    }
    activeReturnModule = newPos;

    if (newPos == activeReturnOrigin) {
        // Module has reached its origin
        pendingOrigins.erase(activeReturnOrigin);
        returnTargets.erase(newPos);
        cout << "  [RETURNED] " << newPos.to_string() << " restored to origin.\n";
        executeReturnIteration();
        return;
    }

    // More steps remaining on current path
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
            cout << "  [RETURN REROUTE] Path invalid at step " << activeReturnStep
                 << ". Recomputing.\n";
            vector<Cell3DPosition> newPath = findPath(newPos, activeReturnOrigin);
            if (newPath.empty() || newPath.size() < 2) {
                cout << "  [RETURN WARNING] Cannot reroute "
                     << newPos.to_string() << " to " << activeReturnOrigin.to_string() << ".\n";
                pendingOrigins.erase(activeReturnOrigin);
                returnTargets.erase(newPos);
                executeReturnIteration();
                return;
            }
            activeReturnPath = newPath;
            activeReturnStep = 1;
            nextPos = activeReturnPath[1];
        }

        auto* sched = getScheduler();
        sched->schedule(new Catoms3DRotationStartEvent(sched->now(), mod, nextPos));
    } else {
        // Step overflow — should have been caught by arrival check above
        cerr << "[RETURN ERROR] Step overflow for module at " << newPos.to_string() << ".\n";
        executeReturnIteration();
    }
}

// =============================================================================
//  verifyAllOriginsRestored — final check; then clear and advance
// =============================================================================

void Catoms3DFlowBridgeCode::verifyAllOriginsRestored() {
    int totalEdges  = (int)allBridges.size();
    int pathsInEdge = (currentBridgeTask.edgeIndex < totalEdges)
                      ? (int)allBridges[currentBridgeTask.edgeIndex].second.size() : 0;

    auto* lattice = BaseSimulator::getWorld()->lattice;
    for (const auto& [origin, path] : travelLog) {
        if (!lattice->getBlock(origin))
            cout << "  [RETURN WARNING] Origin " << origin.to_string() << " still unoccupied.\n";
        else
            cout << "  [VERIFIED] " << origin.to_string() << " restored.\n";
    }

    cout << "=== [BridgeTask " << (currentBridgeTask.edgeIndex+1) << "/" << totalEdges
         << ", Path " << (currentBridgeTask.pathIndex+1) << "/" << pathsInEdge
         << "] Return complete. ===\n";

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
    returnTargets.clear();
    pendingOrigins.clear();
    activeReturnPath.clear();
    activeReturnStep    = 0;
    currentForwardRound = 0;
}

// =============================================================================
//  Phase 5 — advance to next task
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
// =============================================================================

void Catoms3DFlowBridgeCode::handleForwardMotionEnd(const Cell3DPosition& arrivedAt) {
    auto it = pendingArrival.find(arrivedAt);
    if (it == pendingArrival.end()) return;   // not one of our in-flight modules

    Cell3DPosition origin = it->second;
    pendingArrival.erase(it);

    auto tlIt = travelLog.find(origin);
    if (tlIt == travelLog.end()) {
        cerr << "[Forward] onMotionEnd: no travelLog for origin "
             << origin.to_string() << "\n";
        pendingMotions.erase(origin);
        if (pendingMotions.empty()) onForwardRoundComplete();
        return;
    }

    size_t& idx = forwardStepIndex[origin];
    idx++;   // module is now at travelLog[origin][idx]

    const auto& path = tlIt->second;

    // Update returnTargets: module moved from path[idx-1] to arrivedAt.
    // This is normally a NO-OP (returnTargets is keyed by destination, not
    // intermediate positions), but handles C6 rerouting edge cases.
    {
        Cell3DPosition prevPos = path[idx - 1];
        auto rtIt = returnTargets.find(prevPos);
        if (rtIt != returnTargets.end()) {
            Cell3DPosition orig = rtIt->second;
            returnTargets.erase(rtIt);
            returnTargets[arrivedAt] = orig;
        }
    }

    if (idx + 1 < path.size()) {
        // More steps remain — schedule next rotation
        Cell3DPosition nextPos = path[idx + 1];
        auto* lattice = BaseSimulator::getWorld()->lattice;
        Catoms3DBlock* mod = static_cast<Catoms3DBlock*>(lattice->getBlock(arrivedAt));

        bool scheduled = false;
        if (mod && mod->canMoveTo(nextPos)) {
            pendingArrival[nextPos] = origin;
            auto* sched = getScheduler();
            try {
                sched->schedule(new Catoms3DRotationStartEvent(sched->now(), mod, nextPos));
                scheduled = true;
            } catch (const exception& e) {
                cerr << "[Forward] Exception: " << e.what() << "\n";
                pendingArrival.erase(nextPos);
            }
        }

        if (!scheduled) {
            // C6 fallback: recompute path from current position
            if (mod) {
                Cell3DPosition finalDest = path.back();
                vector<Cell3DPosition> newSuffix = findPath(arrivedAt, finalDest);
                if (newSuffix.size() >= 2) {
                    // Splice: keep recorded prefix, then new suffix
                    vector<Cell3DPosition> updated(path.begin(), path.begin() + idx + 1);
                    updated.insert(updated.end(), newSuffix.begin() + 1, newSuffix.end());
                    travelLog[origin] = updated;
                    Cell3DPosition np = updated[idx + 1];
                    if (mod->canMoveTo(np)) {
                        pendingArrival[np] = origin;
                        auto* sched = getScheduler();
                        try {
                            sched->schedule(new Catoms3DRotationStartEvent(
                                sched->now(), mod, np));
                            return;
                        } catch (...) {}
                    }
                }
            }
            cerr << "[Forward] Cannot continue path for origin "
                 << origin.to_string() << ". Dropping module.\n";
            pendingMotions.erase(origin);
            if (pendingMotions.empty()) onForwardRoundComplete();
        }
    } else {
        // Arrived at final destination for this round
        cout << "[Forward] Module from " << origin.to_string()
             << " arrived at " << arrivedAt.to_string() << "\n";
        pendingMotions.erase(origin);
        if (pendingMotions.empty()) {
            cout << "[Forward] Round " << (currentForwardRound+1) << " complete.\n";
            onForwardRoundComplete();
        }
    }
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