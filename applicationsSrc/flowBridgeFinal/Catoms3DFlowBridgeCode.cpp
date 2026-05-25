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

    // -----------------------------------------------------------------------
    // Picard–Queyranne all-min-cuts detection
    //
    //   J.-C. Picard & M. Queyranne, "On the structure of all minimum cuts
    //   in a network and applications", EP-79-R-15, École Polytechnique de
    //   Montréal, 1979.
    //
    // After running max-flow, define the residual relation R on V:
    //     i R j ⇔ ((i,j)∈A and f_ij<c_ij) OR ((j,i)∈A and f_ji>0)
    // i.e. the residual graph.  Compute its strongly connected components.
    // By Corollary 6, a saturated arc (i,j) belongs to SOME minimum cut iff
    // i and j lie in different SCCs of R.  Returning every such arc thus
    // yields the union of all minimum-cut arcs across all minimum cuts.
    // -----------------------------------------------------------------------

    /**
     * Compute strongly connected components of the residual graph.
     * An arc u→v exists in the residual graph iff some edge in u->edges with
     * positive residual capacity ends at v.  This includes both forward edges
     * (cap>0 ⇔ f<c) and reverse edges (cap>0 ⇔ f>0 on the original arc),
     * which is exactly the relation R defined by Picard & Queyranne.
     *
     * Uses an iterative implementation of Tarjan's algorithm to handle deep
     * stacks safely.  Returns a map Node* → SCC index; SCCs are numbered in
     * the order they are popped from the stack.
     */
    unordered_map<Node*, int> computeResidualSCCs() {
        unordered_map<Node*, int> sccOf;
        unordered_map<Node*, int> index;
        unordered_map<Node*, int> lowlink;
        unordered_map<Node*, bool> onStack;
        vector<Node*> tarjanStack;
        int nextIndex = 0;
        int nextScc   = 0;

        // Iterative Tarjan: each call-stack frame holds (node, edge-iterator).
        struct Frame { Node* v; size_t ei; };
        vector<Frame> callStack;

        for (auto& kv : nodes) {
            Node* root = kv.second;
            if (index.count(root)) continue;

            // Push root.
            index[root]    = nextIndex;
            lowlink[root]  = nextIndex;
            ++nextIndex;
            tarjanStack.push_back(root);
            onStack[root]  = true;
            callStack.push_back({root, 0});

            while (!callStack.empty()) {
                Frame& f = callStack.back();
                Node* v  = f.v;

                if (f.ei < v->edges.size()) {
                    Edge* e = v->edges[f.ei++];
                    if (e->capacity <= 0) continue;       // not in residual
                    Node* w = e->to;
                    auto it = index.find(w);
                    if (it == index.end()) {
                        // Descend into w.
                        index[w]   = nextIndex;
                        lowlink[w] = nextIndex;
                        ++nextIndex;
                        tarjanStack.push_back(w);
                        onStack[w] = true;
                        callStack.push_back({w, 0});
                    } else if (onStack[w]) {
                        lowlink[v] = min(lowlink[v], index[w]);
                    }
                } else {
                    // Done with v.  If it's an SCC root, pop the component.
                    if (lowlink[v] == index[v]) {
                        while (true) {
                            Node* w = tarjanStack.back();
                            tarjanStack.pop_back();
                            onStack[w] = false;
                            sccOf[w]   = nextScc;
                            if (w == v) break;
                        }
                        ++nextScc;
                    }
                    callStack.pop_back();
                    if (!callStack.empty()) {
                        Node* parent = callStack.back().v;
                        lowlink[parent] = min(lowlink[parent], lowlink[v]);
                    }
                }
            }
        }
        return sccOf;
    }

    /**
     * Picard–Queyranne union of all min-cut arcs.  An original forward arc
     * (u,v) (originalCap>0) is in the union iff:
     *   (a) it is saturated  (current capacity == 0, i.e. f_uv = c_uv), AND
     *   (b) SCC(u) ≠ SCC(v) in the residual graph.
     * Reverse edges (originalCap == 0) are skipped.  Special source/sink
     * nodes are excluded from the reported Cell3DPosition pairs.
     */
    vector<pair<Cell3DPosition, Cell3DPosition>>
    findAllMinCutEdgesPicardQueyranne(const unordered_map<Edge*, int>& originalCap) {
        vector<pair<Cell3DPosition, Cell3DPosition>> minCutEdges;
        auto sccOf = computeResidualSCCs();

        cout << "\n\n--- Picard–Queyranne All-Min-Cut Edges ---\n";

        // Diagnostic: how many SCCs, where do source/sink fall.
        int totalSccs = 0;
        for (const auto& kv : sccOf) totalSccs = max(totalSccs, kv.second + 1);
        cout << "  Residual graph SCC count: " << totalSccs << "\n";
        if (superSource && sccOf.count(superSource))
            cout << "  SCC(SUPER_SOURCE) = " << sccOf[superSource] << "\n";
        if (superSink && sccOf.count(superSink))
            cout << "  SCC(SUPER_SINK)   = " << sccOf[superSink] << "\n";

        set<pair<string,string>> printed;
        for (auto& kv : nodes) {
            Node* u = kv.second;
            for (Edge* e : u->edges) {
                // Only consider original forward arcs that were saturated.
                auto itCap = originalCap.find(e);
                if (itCap == originalCap.end() || itCap->second <= 0) continue;
                if (e->capacity != 0) continue;             // not saturated
                Node* v = e->to;
                auto sU = sccOf.find(u);
                auto sV = sccOf.find(v);
                if (sU == sccOf.end() || sV == sccOf.end()) continue;
                if (sU->second == sV->second) continue;     // same SCC ⇒ not min-cut

                pair<string,string> ek = {u->key(), v->key()};
                if (printed.count(ek)) continue;
                printed.insert(ek);
                cout << "Min-cut edge: " << u->key() << " -> " << v->key()
                     << "  [SCC " << sU->second << " → SCC " << sV->second << "]\n";

                if (u->specialKey.empty() && v->specialKey.empty())
                    minCutEdges.push_back({{u->x,u->y,u->z},{v->x,v->y,v->z}});
            }
        }
        cout << "  " << minCutEdges.size()
             << " min-cut arc(s) in union (Picard–Queyranne).\n\n";
        return minCutEdges;
    }

    /**
     * Combined-bridge construction via merged sub-flow-graph through empty
     * space.
     *
     * Picard–Queyranne returns every min-cut arc across every minimum cut.
     * We merge them conceptually into one "big min-cut edge" that ranges
     * from the source-most boundary (the union of all min-cut tails {u_i})
     * to the sink-most boundary (the union of all min-cut heads {v_j}).
     * A single sub-flow-graph is built once over the empty-cell motion
     * graph spanning this big edge:
     *      super-source → every u_i        (INF capacity)
     *      every v_j    → super-sink       (INF capacity)
     * Expansion via addEdgesRec from each u_i populates the motion graph
     * through empty cells; heads {v_j} are pre-marked visited so expansion
     * stops at the sink boundary.  Edmonds-Karp enumerates augmenting
     * paths; the deduplicated union of their intermediate empty cells is
     * the combined bridge — exactly the cells on which idle modules will
     * be placed to bypass the entire min-cut at once.
     *
     * Unlike the per-edge approach, this builds and solves a SINGLE sub-
     * graph and walks the motion graph at most once (shared nodeVisited
     * set), so the bridge spans every min-cut arc simultaneously rather
     * than being computed independently for each arc.
     */
    void findCombinedBridge(const vector<pair<Cell3DPosition, Cell3DPosition>>& minCutEdges) {
        Catoms3DFlowBridgeCode::combinedBridgePositions.clear();
        Catoms3DFlowBridgeCode::combinedBridgePaths.clear();

        cout << "\n\n--- Combined Bridge for All Min-Cut Edges ---\n" << flush;

        if (minCutEdges.empty()) {
            cout << "  No min-cut edges to bridge.\n";
            return;
        }

        // Source-most boundary = union of all tails u_i.
        // Sink-most   boundary = union of all heads v_j.
        set<Cell3DPosition> tails, heads;
        for (const auto& [u, v] : minCutEdges) {
            tails.insert(u);
            heads.insert(v);
        }
        // Any node that is both a tail and a head is a "pass-through" — it
        // sits inside the merged big edge.  Drop it from both boundaries so
        // it is neither a source seed nor a sink seed (it can still appear
        // as an intermediate inside the sub-graph).
        set<Cell3DPosition> overlap;
        for (const auto& p : tails) if (heads.count(p)) overlap.insert(p);
        for (const auto& p : overlap) { tails.erase(p); heads.erase(p); }

        cout << "  |tails (source-most)| = " << tails.size()
             << ", |heads (sink-most)| = "   << heads.size()
             << ", |min-cut arcs| = "        << minCutEdges.size() << "\n";
        cout << flush;

        if (tails.empty() || heads.empty()) {
            cout << "  Empty boundary (no separable big edge); nothing to bridge.\n";
            return;
        }

        FlowGraph* bg = new FlowGraph();
        bg->superSource = bg->getOrCreateSpecial("SUPER_SOURCE");
        bg->superSink   = bg->getOrCreateSpecial("SUPER_SINK");

        set<pair<string,string>> ev;
        set<string> nv;

        // Pre-wire heads to super-sink and mark them visited so addEdgesRec
        // does not expand past them — they form the sink boundary of the
        // combined sub-graph.
        for (const auto& v : heads) {
            string vKey = to_string((int)v.pt[0]) + "," +
                          to_string((int)v.pt[1]) + "," +
                          to_string((int)v.pt[2]);
            Node* vNode = bg->getOrCreateNode(v);
            pair<string,string> ek = { vNode->key(), bg->superSink->key() };
            if (ev.insert(ek).second) {
                bg->addEdge(vNode, bg->superSink, FlowGraph::INF_CAPACITY);
            }
            nv.insert(vKey);
        }

        // Wire each tail to super-source and expand through empty space.
        // Expansions across distinct tails share the nodeVisited set, so
        // each motion-graph cell is processed at most once across the
        // entire combined sub-graph.
        for (const auto& u : tails) {
            string uKey = to_string((int)u.pt[0]) + "," +
                          to_string((int)u.pt[1]) + "," +
                          to_string((int)u.pt[2]);
            Node* uNode = bg->getOrCreateNode(u);
            pair<string,string> ek = { bg->superSource->key(), uNode->key() };
            if (!ev.count(ek)) {
                bg->addEdge(bg->superSource, uNode, FlowGraph::INF_CAPACITY);
                ev.insert(ek);
            }
            if (!nv.count(uKey))
                bg->addEdgesRec(uNode, u, ev, nv);
        }

        cout << "  Sub-graph built: " << bg->nodes.size() << " node(s).\n" << flush;

        int flow = bg->findAndPrintAugmentingPaths(bg->superSource, bg->superSink);
        cout << "  " << flow << " augmenting path(s) in combined sub-graph.\n" << flush;

        // Combined bridge = ∪ intermediate cells over all augmenting paths.
        //
        // The motion-graph expansion via addEdgesRec can produce nodes whose
        // lattice cell is currently occupied (e.g. when an empty-cell pivot
        // computation lands on a filled neighbor, or when a tail/head is
        // revisited as an intermediate in another path).  Idle modules must
        // only be placed on EMPTY lattice cells, so we filter intermediates
        // down to those whose lattice cell is currently free.  Tails and
        // heads themselves are also excluded — they form the existing
        // structural boundary of the merged big min-cut edge and are
        // already occupied.
        auto* lattice = BaseSimulator::getWorld()->lattice;
        set<Cell3DPosition> bridgeSet;
        int skippedFilled = 0;
        for (size_t p = 0; p < bg->augmentingPathPositions.size(); ++p) {
            vector<Cell3DPosition> intermediates;
            for (const auto& pos : bg->augmentingPathPositions[p]) {
                if (tails.count(pos) || heads.count(pos)) continue;
                if (lattice->getBlock(pos) != nullptr) {
                    ++skippedFilled;
                    continue;        // already occupied — cannot place a module here
                }
                intermediates.push_back(pos);
            }
            if (intermediates.empty()) {
                cout << "  Path " << (p+1) << ": no empty intermediates\n";
            } else {
                cout << "  Path " << (p+1) << " empty intermediates:";
                for (const auto& ip : intermediates) cout << " " << ip.to_string();
                cout << "\n";
                Catoms3DFlowBridgeCode::combinedBridgePaths.push_back(intermediates);
                for (const auto& ip : intermediates) bridgeSet.insert(ip);
            }
        }

        Catoms3DFlowBridgeCode::combinedBridgePositions.assign(
            bridgeSet.begin(), bridgeSet.end());

        cout << "\n  Combined bridge size: "
             << Catoms3DFlowBridgeCode::combinedBridgePositions.size()
             << " empty cell(s) to fill"
             << " (skipped " << skippedFilled
             << " already-occupied path cell(s)).\n";
        if (!Catoms3DFlowBridgeCode::combinedBridgePositions.empty()) {
            cout << "  Bridge positions:";
            for (const auto& p : Catoms3DFlowBridgeCode::combinedBridgePositions)
                cout << " " << p.to_string();
            cout << "\n";
        }
        cout << "\n" << flush;

        delete bg;
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

    /**
     * Top-level analysis: run Edmonds-Karp, then apply Picard–Queyranne to
     * enumerate every arc belonging to some minimum cut, then merge them
     * into one big min-cut edge and build a single combined bridge across
     * empty space spanning all the min-cut endpoints.
     */
    /**
     * Report flow conservation on the super-source / super-sink boundary
     * after the initial Edmonds-Karp run.
     *
     * For every original forward arc (originalCap > 0):
     *   flow(e) = originalCap[e] - e->capacity
     * Total flow exiting the sources = Σ flow(e) over e ∈ source->edges with
     * originalCap > 0 (the super-source → real-source arcs).
     * Total flow entering the sinks  = Σ flow(e) over original forward arcs
     * whose head is the super-sink (i.e. the real-sink → super-sink arcs).
     * By the flow-conservation theorem both totals equal the max-flow value.
     */
    void reportSourceSinkFlows(const unordered_map<Edge*, int>& originalCap,
                                Node* source, Node* sink) const {
        cout << "\n--- Initial Edmonds-Karp Flow at Source / Sink Boundary ---\n";

        int totalOut = 0;
        cout << "  Flow exiting SUPER_SOURCE per source:\n";
        for (Edge* e : source->edges) {
            auto it = originalCap.find(e);
            if (it == originalCap.end() || it->second <= 0) continue;   // skip reverse edges
            int f = it->second - e->capacity;
            cout << "    " << source->key() << " -> " << e->to->key()
                 << "  flow = " << f << " / cap " << it->second << "\n";
            totalOut += f;
        }
        cout << "  Total flow exiting sources = " << totalOut << "\n";

        int totalIn = 0;
        cout << "  Flow entering SUPER_SINK per sink:\n";
        for (auto& kv : nodes) {
            for (Edge* e : kv.second->edges) {
                if (e->to != sink) continue;
                auto it = originalCap.find(e);
                if (it == originalCap.end() || it->second <= 0) continue; // skip reverse edges
                int f = it->second - e->capacity;
                cout << "    " << e->from->key() << " -> " << sink->key()
                     << "  flow = " << f << " / cap " << it->second << "\n";
                totalIn += f;
            }
        }
        cout << "  Total flow entering sinks  = " << totalIn << "\n";
    }

    int startProcess(Node* source, Node* sink,
                     vector<pair<Cell3DPosition,Cell3DPosition>>& outMinCutEdges) {
        auto originalCap = backupCapacities();
        int flow = findAndPrintAugmentingPaths(source, sink);
        reportSourceSinkFlows(originalCap, source, sink);
        outMinCutEdges = findAllMinCutEdgesPicardQueyranne(originalCap);
        findCombinedBridge(outMinCutEdges);
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
vector<Cell3DPosition>           Catoms3DFlowBridgeCode::combinedBridgePositions;
vector<vector<Cell3DPosition>>   Catoms3DFlowBridgeCode::combinedBridgePaths;

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
map<Cell3DPosition, set<Cell3DPosition>>    Catoms3DFlowBridgeCode::forwardVisited;

// --- Activity-diagram donor-selection state ---
SelectionStage                              Catoms3DFlowBridgeCode::currentSelectionStage = SelectionStage::SEL_DONE;
vector<Cell3DPosition>                      Catoms3DFlowBridgeCode::sourceModulesPool;
vector<Cell3DPosition>                      Catoms3DFlowBridgeCode::srcSideIdlePool;
vector<Cell3DPosition>                      Catoms3DFlowBridgeCode::sinkSideIdlePool;
vector<Cell3DPosition>                      Catoms3DFlowBridgeCode::bridgeSinkMostOrder;
vector<Cell3DPosition>                      Catoms3DFlowBridgeCode::bridgeSrcMostOrder;
map<Cell3DPosition, int>                    Catoms3DFlowBridgeCode::distToSource;
map<Cell3DPosition, int>                    Catoms3DFlowBridgeCode::distToSink;

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
//  [2] Build main FlowGraph (movingBlocks → targetPositions), run Edmonds-Karp,
//      then apply the Picard–Queyranne characterization to extract the union
//      of all min-cut arcs across all minimum cuts (allMinCutEdges).  Merge
//      every min-cut arc into ONE big min-cut edge that ranges from the
//      source-most boundary (union of all tails {u_i}) to the sink-most
//      boundary (union of all heads {v_j}).  Build a single combined sub-
//      flow-graph through empty space spanning that big edge and run
//      Edmonds-Karp; the deduplicated union of all augmenting paths'
//      intermediate empty cells is the combined bridge — exactly the cells
//      on which idle modules will be placed to bypass the entire minimum
//      cut in a single pass.
//      Identify idle structural blocks (not used as pivots in augmenting paths).
//  [3] Filter to validIdleModules (non-blocked, non-articulation).
//  [Phase 0] Push ONE BridgeTask whose intermediatePath spans the combined
//            bridge and launch it.
//    → Forward loop: iterative round-based filling of bridge positions.
//    → Return loop:  iterative shortest-path backtracking of displaced modules.
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

    // Run Edmonds-Karp, then Picard–Queyranne min-cut union, then bridge paths.
    originalFlowValue = fg->startProcess(fg->superSource, fg->superSink, allMinCutEdges);
    fg->printIdleStructuralBlocks(idleStructuralBlocks);
    delete fg;

    computeValidIdleModules();

    // Virtual pre-verification — simulate the bridge being in place and
    // confirm that re-running Edmonds-Karp from scratch increases the flow
    // BEFORE committing to physical orchestration.
    verifyBridgeVirtually();

    // Phase 0 — launch bridge orchestration
    startBridgeOrchestration();
}

// =============================================================================
//  Virtual bridge pre-verification
//
//  Insert phantom Catoms3DBlock objects at every cell of combinedBridgePositions
//  into the lattice, rebuild a fresh FlowGraph, run Edmonds-Karp, and compare
//  the resulting flow against originalFlowValue.  The phantoms serve as pivots
//  for the empty cells adjacent to them, so the FlowGraph naturally discovers
//  the new motion paths the physical bridge would unlock.  Phantoms are removed
//  and destroyed before returning — no persistent state is left in the lattice.
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

    // Sanity check: every bridge cell must currently be empty.  If any is
    // already occupied we cannot safely insert a phantom there — skip it
    // and warn the user (the physical orchestration would also skip it).
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

    // -------------------------------------------------------------------
    // Virtually move source modules to bridge cells.
    //
    // "As if the bridge was built" means the source modules that fill the
    // bridge are no longer at their original source positions.  Leaving
    // them in place causes the motion engine's "opposite of FROM must be
    // free" rule (catoms3DMotionRules.cpp:401) to fail for every phantom-
    // pivoted motion whose mirror cell happens to be a source position —
    // which is exactly why the naive virtual graph reports 0 augmenting
    // paths.
    //
    // We don't know in advance which sources the donor-selection state
    // machine will pick (computeBridgeSelectionState runs later), but for
    // max-flow purposes all super-source-connected sources are equivalent
    // — only their lattice occupancy changes the motion graph.  We pick
    // the |insertCells| sources with the SMALLEST Manhattan distance to
    // their nearest bridge cell (i.e., the sources the donor logic would
    // naturally consume first) and remove them from the lattice until the
    // FlowGraph build is done.
    // -------------------------------------------------------------------
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

    // Pick a phantom-ID range that does not collide with any existing module
    // ID.  We never call world->addBlock(), so maxBlockId is not touched —
    // phantoms exist only in the lattice grid and are removed before exit.
    bID phantomIdBase = 1;
    {
        const auto& blocks = BaseSimulator::getWorld()->getMap();
        for (const auto& kv : blocks)
            if (kv.first >= phantomIdBase) phantomIdBase = kv.first + 1;
        // Add a generous offset to make phantoms easy to spot in any log
        // that might accidentally print their IDs.
        phantomIdBase += 1000000;
    }

    vector<Catoms3DBlock*> phantoms;
    vector<Catoms3DGlBlock*> phantomGls;
    phantoms.reserve(insertCells.size());
    phantomGls.reserve(insertCells.size());

    for (size_t i = 0; i < insertCells.size(); ++i) {
        const Cell3DPosition& pos = insertCells[i];
        bID id = phantomIdBase + (bID)i;

        // Create a phantom Catoms3DBlock.  BuildingBlock's ctor instantiates
        // a BlockCode via our buildNewBlockCode, but startup() is gated by
        // a static `hasRun` flag so it will not re-execute.
        auto* phantom = new Catoms3DBlock(id,
                                          Catoms3DFlowBridgeCode::buildNewBlockCode);

        // A Catoms3DGlBlock holds the per-module transformation matrix
        // (`mat`) that Catoms3DBlock::getConnectorId() / getNeighborPos()
        // dereference unconditionally.  Without it those methods crash with
        // a null-pointer access when the motion engine treats the phantom
        // as a pivot.  We must therefore allocate a real GlBlock, wire it
        // into the phantom via setGlBlock(), and let setPositionAndOrientation
        // populate the rotation matrix.
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

    // Sanity check: every phantom is visible through lattice->getBlock
    // and its location is correctly reported as non-free.
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

    // Probe: enumerate motion possibilities that USE phantoms as pivots.
    // This confirms the motion engine can produce links from phantom pivots
    // before we hand the graph to Edmonds-Karp.
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
                    // (the phantom must be a common active neighbour of nb
                    // and dest); a cheap check is that the phantom is in the
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

    // Per-source quick reachability probe: from each moving block at its
    // current position, how many cells does the motion engine return?
    // With phantoms in place this MUST still be positive for sources whose
    // neighbourhood is unaffected by phantoms (sources sit at x=1..4).
    {
        cout << "  Per-source one-step reachable counts:\n";
        for (const auto& s : movingBlocks) {
            vector<Cell3DPosition> r;
            getAllPossibleMotionsFromPosition(s, r);
            cout << "    " << s.to_string() << " -> " << r.size()
                 << " reachable cell(s)\n";
        }
    }

    // Build a fresh FlowGraph over the modified lattice state and run
    // Edmonds-Karp from scratch.
    cout << "  Building virtual FlowGraph...\n" << flush;
    FlowGraph* fgVirtual = new FlowGraph();
    fgVirtual->buildFlowGraph();
    cout << "  Virtual FlowGraph node count: " << fgVirtual->nodes.size()
         << "\n";

    // Diagnostic: count edges leaving super-source and entering super-sink,
    // and BFS-reachability from super-source over positive-capacity edges.
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

    // Restore the lattice exactly as it was.
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

vector<Cell3DPosition> Catoms3DFlowBridgeCode::findPathForModule(
    Catoms3DBlock *mod, const Cell3DPosition &start, const Cell3DPosition &goal) {
    // Module-aware BFS: at each position along the BFS frontier we simulate the
    // module being there and query mod->canMoveTo for its free neighbours.
    // This is accurate only when the module is physically AT 'start' (the lattice
    // pivot structure is correct for its actual location).  For intermediate BFS
    // nodes we fall back to the empty-cell pivot query (getAllPossibleMotions),
    // which may overestimate reachability — but the first step is guaranteed
    // correct because we call mod->canMoveTo directly from start.
    //
    // Strategy:
    //   Step 0 → step 1 : use mod->canMoveTo (exact).
    //   Step 1 onward   : use getAllPossibleMotionsFromPosition (approximate but
    //                     validated again by canMoveTo at dispatch time).

    vector<Cell3DPosition> path;
    if (start == goal) { path.push_back(start); return path; }

    auto* lattice = BaseSimulator::getWorld()->lattice;
    set<Cell3DPosition> visited;
    map<Cell3DPosition, Cell3DPosition> parentMap;
    queue<Cell3DPosition> q;

    q.push(start);
    visited.insert(start);
    bool found = false;

    while (!q.empty() && !found) {
        Cell3DPosition current = q.front(); q.pop();

        vector<Cell3DPosition> neighbours;
        if (current == start) {
            // Exact first-step reachability using the real module.
            for (auto& nb : lattice->getFreeNeighborCells(current))
                if (mod->canMoveTo(nb))
                    neighbours.push_back(nb);
        } else {
            // Approximate for deeper BFS nodes.
            getAllPossibleMotionsFromPosition(current, neighbours);
        }

        for (auto& nb : neighbours) {
            if (nb == goal) {
                parentMap[nb] = current;
                found = true;
                break;
            }
            if (!visited.count(nb)) {
                visited.insert(nb);
                parentMap[nb] = current;
                q.push(nb);
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
    cout << "\n=== [Phase 0] Building combined bridge task ===\n";

    while (!pendingBridges.empty()) pendingBridges.pop();

    if (combinedBridgePositions.empty()) {
        cout << "Combined bridge is empty (no min-cut edges or no empty-space paths). "
                "Halting.\n";
        return;
    }

    // A single BridgeTask spanning the combined bridge.  Its intermediatePath
    // is the deduplicated union of all augmenting-path intermediates produced
    // by findCombinedBridge() — i.e. every empty cell on which an idle module
    // must be placed to bypass the entire min-cut at once.
    //
    // The minCutEdge field carries a representative (first u, first v) for
    // diagnostic logging only; the bridge itself spans all merged min-cut
    // endpoints.
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

    // --- Clear all per-task state ---
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

    // --- Build activity-diagram donor-selection state ---
    computeBridgeSelectionState();

    // --- Start first forward round ---
    runForwardRound();
}

// =============================================================================
//  Activity-diagram selection helpers
// =============================================================================
//
//  These routines implement the donor-selection state machine described by
//  the activity diagram supplied with the application.  The motion mechanism
//  is left untouched — only the choice of which module (and which bridge
//  cell) to dispatch each round is rewired.
//
//      ┌────────────────────────────────────────────┐
//      │  |S| vs |B|                                │
//      └────────────────────────────────────────────┘
//           |S| > |B|                |S| ≤ |B|
//                │                       │
//        SOURCE_ONLY        ┌───────────┴────────────┐
//        (source modules    │ Stage A: SRC_SIDE_IDLE │
//         fill bridge)      │   src-side idle →      │
//                           │   sink-most cells      │
//                           └───────────┬────────────┘
//                                       │
//                       all bridge cells filled? — yes → done
//                                       │ no
//                           ┌───────────┴────────────┐
//                           │ Stage B: SRC_MODULES   │
//                           │   source modules →     │
//                           │   source-most cells    │
//                           └───────────┬────────────┘
//                                       │
//                       all bridge cells filled? — yes → done
//                                       │ no
//                           ┌───────────┴────────────┐
//                           │ Stage C: SINK_SIDE_IDLE│
//                           │   sink-side idle →     │
//                           │   closest remaining    │
//                           └────────────────────────┘
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

void Catoms3DFlowBridgeCode::computeBridgeSelectionState() {
    // ---- Snapshot source modules and reset pools ----
    sourceModulesPool   = movingBlocks;
    srcSideIdlePool .clear();
    sinkSideIdlePool.clear();
    bridgeSinkMostOrder.clear();
    bridgeSrcMostOrder .clear();

    // ---- BFS distance to the nearest source and to the nearest sink ----
    distToSource = bfsDistancesFromSet(movingBlocks);
    distToSink   = bfsDistancesFromSet(targetPositions);

    auto distOr = [](const map<Cell3DPosition,int>& m,
                     const Cell3DPosition& p) {
        auto it = m.find(p);
        return (it == m.end()) ? INT_MAX : it->second;
    };

    // ---- Classify idle modules by side ----
    for (const auto& p : validIdleModules) {
        int dS = distOr(distToSource, p);
        int dT = distOr(distToSink,   p);
        if (dS <= dT) srcSideIdlePool .push_back(p);
        else           sinkSideIdlePool.push_back(p);
    }

    // ---- Order bridge cells by sink-most and source-most distance ----
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

    // ---- Decide initial stage ----
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

// -----------------------------------------------------------------------------
// Greedy stage-ordered (donor → bridge cell) selection.
//
//   SOURCE_ONLY         : source modules → source-most bridge cells first.
//   SRC_SIDE_IDLE_FILL  : source-side idle modules → sink-most bridge cells.
//   SRC_MODULES_FILL    : source modules → source-most bridge cells.
//   SINK_SIDE_IDLE_FILL : sink-side idle modules → closest remaining cells.
//
// Per round we pick at most one (donor, bridge cell) pair according to the
// stage's strategy and dispatch a single rotation step.  The motion mechanism
// (findPathForModule + step-by-step rotations) is unchanged.
// -----------------------------------------------------------------------------
bool Catoms3DFlowBridgeCode::dispatchOneUnderCurrentStage() {
    auto* lattice = BaseSimulator::getWorld()->lattice;

    // ---- Choose donor pool and target ordering for this stage ----
    const vector<Cell3DPosition>* donorPool   = nullptr;
    const vector<Cell3DPosition>* targetOrder = nullptr;
    bool closestRemainingMode = false;     // true ⇒ Stage C (no fixed order)

    switch (currentSelectionStage) {
        case SelectionStage::SOURCE_ONLY:
            donorPool   = &sourceModulesPool;
            targetOrder = &bridgeSrcMostOrder;        // fill source-most first
            break;
        case SelectionStage::SRC_SIDE_IDLE_FILL:
            donorPool   = &srcSideIdlePool;
            targetOrder = &bridgeSinkMostOrder;       // fill sink-most first
            break;
        case SelectionStage::SRC_MODULES_FILL:
            donorPool   = &sourceModulesPool;
            targetOrder = &bridgeSrcMostOrder;        // fill source-most first
            break;
        case SelectionStage::SINK_SIDE_IDLE_FILL:
            donorPool   = &sinkSideIdlePool;
            targetOrder = nullptr;
            closestRemainingMode = true;              // closest pairing
            break;
        case SelectionStage::SEL_DONE:
            return false;
    }

    // ---- Filter donor pool to modules still at origin ----
    vector<Cell3DPosition> availDonors;
    availDonors.reserve(donorPool->size());
    for (const auto& o : *donorPool)
        if (lattice->getBlock(o) != nullptr) availDonors.push_back(o);

    if (availDonors.empty()) return false;            // stage exhausted

    // ---- Pick (origin, target) according to stage's strategy ----
    Cell3DPosition bestOrigin{}, bestTarget{};
    vector<Cell3DPosition> bestPath;

    if (closestRemainingMode) {
        // Stage C: for each remaining empty bridge cell, find the closest
        // sink-side idle donor by squared-Euclidean distance; pick the
        // overall closest feasible pair.
        int bestDist = INT_MAX;
        for (const auto& target : combinedBridgePositions) {
            if (lattice->getBlock(target)) continue;  // already filled
            for (const auto& origin : availDonors) {
                short dx = target.pt[0] - origin.pt[0];
                short dy = target.pt[1] - origin.pt[1];
                short dz = target.pt[2] - origin.pt[2];
                int d = dx*dx + dy*dy + dz*dz;
                if (d < bestDist) {
                    Catoms3DBlock* mod =
                        static_cast<Catoms3DBlock*>(lattice->getBlock(origin));
                    if (!mod) continue;
                    vector<Cell3DPosition> path =
                        findPathForModule(mod, origin, target);
                    if (path.size() < 2) continue;
                    bestDist  = d;
                    bestOrigin = origin;
                    bestTarget = target;
                    bestPath   = path;
                }
            }
        }
    } else {
        // Stages SOURCE_ONLY / A / B: iterate targets in their priority order,
        // and for each target pick the donor whose module-aware path is the
        // shortest.  As soon as one target is matched, dispatch immediately.
        for (const auto& target : *targetOrder) {
            if (lattice->getBlock(target)) continue;  // already filled
            int bestLen = INT_MAX;
            vector<Cell3DPosition> bestForThisTarget;
            Cell3DPosition originForThisTarget;
            for (const auto& origin : availDonors) {
                Catoms3DBlock* mod =
                    static_cast<Catoms3DBlock*>(lattice->getBlock(origin));
                if (!mod) continue;
                vector<Cell3DPosition> path =
                    findPathForModule(mod, origin, target);
                if (path.size() < 2) continue;
                if ((int)path.size() < bestLen) {
                    bestLen = (int)path.size();
                    bestForThisTarget   = path;
                    originForThisTarget = origin;
                }
            }
            if (!bestForThisTarget.empty()) {
                bestOrigin = originForThisTarget;
                bestTarget = target;
                bestPath   = bestForThisTarget;
                break;                                // honour stage ordering
            }
        }
    }

    if (bestPath.empty()) return false;               // no feasible pair

    // ---- Dispatch the single chosen module (motion mechanism unchanged) ----
    pendingMotions.clear();
    pendingArrival.clear();

    Catoms3DBlock* mod =
        static_cast<Catoms3DBlock*>(lattice->getBlock(bestOrigin));
    if (!mod) return false;
    Cell3DPosition nextPos = bestPath[1];
    if (!mod->canMoveTo(nextPos)) return false;

    travelLog[bestOrigin]        = bestPath;
    forwardStepIndex[bestOrigin] = 0;
    forwardVisited[bestOrigin]   = { bestOrigin };
    returnTargets[bestTarget]    = bestOrigin;
    pendingMotions.insert(bestOrigin);
    pendingArrival[nextPos]      = bestOrigin;
    roundAssignments.push_back({ bestOrigin });

    const char* stageLabel = "?";
    switch (currentSelectionStage) {
        case SelectionStage::SOURCE_ONLY:         stageLabel = "SOURCE_ONLY"; break;
        case SelectionStage::SRC_SIDE_IDLE_FILL:  stageLabel = "STAGE_A_SRC_IDLE"; break;
        case SelectionStage::SRC_MODULES_FILL:    stageLabel = "STAGE_B_SRC_MODULES"; break;
        case SelectionStage::SINK_SIDE_IDLE_FILL: stageLabel = "STAGE_C_SINK_IDLE"; break;
        case SelectionStage::SEL_DONE:            stageLabel = "SEL_DONE"; break;
    }
    cout << "  [DISPATCH/" << stageLabel << "] Module "
         << bestOrigin.to_string() << " → bridge cell "
         << bestTarget.to_string()
         << " (" << (bestPath.size() - 1) << " step(s))\n";

    auto* sched = getScheduler();
    try {
        sched->schedule(new Catoms3DRotationStartEvent(sched->now(), mod, nextPos));
    } catch (const exception& e) {
        cerr << "[Bridge] Exception scheduling first step for "
             << bestOrigin.to_string() << ": " << e.what() << "\n";
        pendingArrival.erase(nextPos);
        pendingMotions.erase(bestOrigin);
        returnTargets.erase(bestTarget);
        travelLog.erase(bestOrigin);
        forwardStepIndex.erase(bestOrigin);
        forwardVisited.erase(bestOrigin);
        return false;
    }
    return true;
}

// =============================================================================
//  Forward loop — strictly serialised dispatch (ONE module at a time).
//
//  Each call to runForwardRound() picks the single best (idle, bridge cell)
//  pair available right now and dispatches that one module.  No two modules
//  are ever in flight simultaneously: the lattice changes that a moving
//  module induces are settled (one full forward trip, step by step) before
//  the next module is chosen.
//
//  Donor selection follows the activity-diagram state machine driven by
//  currentSelectionStage (see dispatchOneUnderCurrentStage / advanceSelectionStage).
//  The motion mechanism (BFS-based path finding + step-by-step rotations) is
//  unchanged from the previous variant.
// =============================================================================

void Catoms3DFlowBridgeCode::runForwardRound() {
    auto* lattice = BaseSimulator::getWorld()->lattice;

    // --- R1: bridge built? ---
    bool anyUnoccupied = false;
    for (const auto& pos : currentBridgeTask.intermediatePath)
        if (!lattice->getBlock(pos)) { anyUnoccupied = true; break; }

    if (!anyUnoccupied) {
        cout << "[Bridge] All " << currentBridgeTask.intermediatePath.size()
             << " combined bridge cell(s) filled.\n";
        verifyBridge();
        startReturnPhase();
        return;
    }

    // Count remaining cells for the round banner.
    size_t remaining = 0;
    for (const auto& pos : currentBridgeTask.intermediatePath)
        if (!lattice->getBlock(pos)) ++remaining;

    cout << "\n=== [Bridge Round " << (currentForwardRound + 1) << "] "
         << remaining << " / " << currentBridgeTask.intermediatePath.size()
         << " combined bridge cell(s) remaining ===\n";

    // --- R2: dispatch one module per the activity-diagram stage. ---
    // If the current stage cannot dispatch (its donor pool exhausted or no
    // feasible (donor, target) pair), advance to the next stage and retry.
    while (currentSelectionStage != SelectionStage::SEL_DONE) {
        if (dispatchOneUnderCurrentStage()) return;     // one module on the way
        advanceSelectionStage();                        // try next stage
    }

    // No stage can dispatch a module: forward phase ends here.
    cout << "[Bridge] Selection state machine exhausted; "
         << remaining << " bridge cell(s) remain unfilled.\n";
    verifyBridge();
    startReturnPhase();
}

// =============================================================================
//  Forward round completion — all modules of the round have arrived or been
//  declared stuck.  Record newly placed bridge cells, then start the next round.
// =============================================================================

void Catoms3DFlowBridgeCode::onForwardRoundComplete() {
    auto* lattice = BaseSimulator::getWorld()->lattice;

    // Scan returnTargets: any entry whose current position is a bridge cell is a
    // successfully placed module — record it in placedIntermediates.
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
//  Bridge verification (called when forward loop exits)
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
    int newFlow = fg->findAndPrintAugmentingPaths(fg->superSource, fg->superSink);
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
//  Transition to return phase.
//  Modules that successfully reached a bridge cell stay there.
//  All other dispatched modules (stuck mid-path) are returned to their origins.
// =============================================================================

void Catoms3DFlowBridgeCode::startReturnPhase() {
    // Keep only modules that did NOT reach a bridge position — those must return.
    // Modules that successfully reached a bridge cell are already in placedIntermediates
    // and must not be moved.
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

    // Step Ret.1 — select module with shortest BFS path back to its origin.
    // Use findPathForModule so the first step is validated against the actual module.
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

    // Step Ret.2 — schedule first step of selected module.
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
        // Step overflow — should have been caught by arrival check above
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

    // Reset activity-diagram selection state
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
    // -----------------------------------------------------------------------
    // This handler is called after each single step of any in-flight module.
    // Multiple modules may be moving concurrently within the same round;
    // pendingArrival maps each expected next position to its origin.
    // -----------------------------------------------------------------------
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

    // Update returnTargets so the return phase always knows where this module is.
    for (auto rtIt = returnTargets.begin(); rtIt != returnTargets.end(); ++rtIt) {
        if (rtIt->second == origin && rtIt->first != arrivedAt) {
            returnTargets[arrivedAt] = origin;
            returnTargets.erase(rtIt);
            break;
        }
    }

    // -----------------------------------------------------------------------
    // Arrived at the bridge cell — module is placed.
    // -----------------------------------------------------------------------
    if (arrivedAt == finalDest) {
        cout << "  [ARRIVED] Module from " << origin.to_string()
             << " placed at bridge cell " << arrivedAt.to_string() << ".\n";
        forwardVisited.erase(origin);
        pendingMotions.erase(origin);
        if (pendingMotions.empty())
            onForwardRoundComplete();
        return;
    }

    // -----------------------------------------------------------------------
    // More steps remain.  Recompute the path from the current position using
    // findPathForModule (exact first-step check) but excluding visited cells
    // so the module cannot oscillate back through positions it already passed.
    // -----------------------------------------------------------------------
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

    // Build candidate list: free neighbours reachable via canMoveTo, not yet visited.
    vector<Cell3DPosition> candidates;
    for (auto& nb : lattice->getFreeNeighborCells(arrivedAt))
        if (!visited.count(nb) && mod->canMoveTo(nb))
            candidates.push_back(nb);

    bool scheduled = false;

    if (!candidates.empty()) {
        // Among candidates, pick the one whose onward path to finalDest is shortest
        // and does not pass through any visited cell.
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
        // This module is done for this round; if it was the last, advance to next round.
        if (pendingMotions.empty()) onForwardRoundComplete();
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
