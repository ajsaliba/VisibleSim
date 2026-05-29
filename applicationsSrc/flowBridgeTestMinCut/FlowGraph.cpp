// =============================================================================
//  FlowGraph.cpp
//  -------------
//  Implementation of the residual-network flow graph used by the Catoms3D
//  Flow Bridge (combined min-cut-edge variant).
//
//  The graph carries:
//    * Edmonds-Karp max-flow with augmenting-path logging.
//    * Greedy maximum Edge-Disjoint Paths (Kleinberg & Salavatipour 2004),
//      as recommended in EDPC §3.1 (Beverland, Kliuchnikov & Schoute,
//      arXiv:2110.11493v2 — "Surface code compilation via edge-disjoint paths").
//    * Picard-Queyranne characterisation of every arc that belongs to some
//      minimum cut (EP-79-R-15, École Polytechnique de Montréal, 1979).
//    * Combined-bridge construction over the empty-cell motion graph.
// =============================================================================

#include "FlowGraph.h"
#include "Catoms3DFlowBridgeCode.h"
#include "FlowGraphViz.h"

#include <iostream>
#include <algorithm>
#include <climits>

using namespace std;

// =============================================================================
//  Destruction
// =============================================================================

FlowGraph::~FlowGraph() {
    for (auto& kv : nodes) {
        for (auto e : kv.second->edges) { if (e) delete e; }
        kv.second->edges.clear();
    }
    for (auto& kv : nodes) delete kv.second;
    nodes.clear();
}

// =============================================================================
//  Node construction
// =============================================================================

Node* FlowGraph::getOrCreateNode(short x, short y, short z) {
    string k = to_string(x) + "," + to_string(y) + "," + to_string(z);
    auto it = nodes.find(k);
    if (it != nodes.end()) return it->second;
    Node* n = new Node(x, y, z);
    nodes[k] = n;
    return n;
}

Node* FlowGraph::getOrCreateNode(const Cell3DPosition& pos) {
    return getOrCreateNode(pos.pt[0], pos.pt[1], pos.pt[2]);
}

Node* FlowGraph::getOrCreateSpecial(const string& name) {
    if (nodes.count(name)) return nodes[name];
    Node* n = new Node(-1, -1, -1);
    n->specialKey = name;
    nodes[name] = n;
    return n;
}

// =============================================================================
//  Edge construction
// =============================================================================

void FlowGraph::addEdge(Node* from, Node* to, int capacity) {
    Edge* e1 = new Edge(from, to, capacity);
    Edge* e2 = new Edge(to, from, 0);
    e1->rev = e2; e2->rev = e1;
    from->edges.push_back(e1);
    to->edges.push_back(e2);
}

void FlowGraph::addEdgesRec(Node* fromNode,
                            const Cell3DPosition& fromPos,
                            set<pair<string, string>>& edgeVisited,
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

// =============================================================================
//  Top-level build (super-source / super-sink wiring)
// =============================================================================

void FlowGraph::buildFlowGraph() {
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

// =============================================================================
//  Bridge-fill flow graph (unit-capacity motion planner)
//
//  Wires:
//      super-source -> every donor cell        (cap 1)
//      donor expands through empty cells       (cap 1, via addEdgesRec)
//      every empty bridge cell -> super-sink   (cap 1)
//
//  Cap-1 super edges enforce "one path per donor" (super-source bottleneck)
//  and "one module per bridge cell" (super-sink bottleneck).  Cap-1 internal
//  edges make the resulting Edmonds-Karp augmenting paths edge-disjoint, so
//  they describe physically non-overlapping module trajectories that can be
//  dispatched concurrently.
//
//  emptyTargets must currently be free lattice cells; donors must currently
//  hold a module.  The motion-graph expansion follows the same rules as the
//  main FlowGraph build (see addEdgesRec).
// =============================================================================

void FlowGraph::buildBridgeFlowGraph(
    const vector<Cell3DPosition>& donors,
    const vector<Cell3DPosition>& emptyTargets) {

    superSource = getOrCreateSpecial("SUPER_SOURCE");
    superSink   = getOrCreateSpecial("SUPER_SINK");

    set<pair<string,string>> ev;
    set<string> nv;

    // -------- Pre-wire empty bridge cells -> super-sink (cap 1) --------
    // Mark each target's key as visited so addEdgesRec does NOT expand the
    // motion graph past the sink boundary.
    for (const auto& t : emptyTargets) {
        Node* tNode = getOrCreateNode(t);
        pair<string,string> ek = { tNode->key(), superSink->key() };
        if (ev.insert(ek).second) {
            addEdge(tNode, superSink, 1);   // unit capacity per bridge cell
        }
        nv.insert(tNode->key());
    }

    // -------- Wire each donor -> super-source (cap 1) and expand --------
    for (const auto& d : donors) {
        Node* dNode = getOrCreateNode(d);
        pair<string,string> ek = { superSource->key(), dNode->key() };
        if (!ev.count(ek)) {
            addEdge(superSource, dNode, 1);  // unit capacity per donor
            ev.insert(ek);
        }
        // Don't re-expand a donor that has already been marked visited
        // (which happens if a donor cell coincides with a target — unusual
        // but possible if the lattice is unusual).
        if (!nv.count(dNode->key()))
            addEdgesRec(dNode, d, ev, nv);
    }
}

// =============================================================================
//  Capacity helpers
// =============================================================================

unordered_map<Edge*, int> FlowGraph::backupCapacities() {
    unordered_map<Edge*, int> cap;
    for (auto& kv : nodes)
        for (Edge* e : kv.second->edges)
            cap[e] = e->capacity;
    return cap;
}

// =============================================================================
//  Edmonds-Karp augmenting paths
// =============================================================================

int FlowGraph::findAndPrintAugmentingPaths(Node* source, Node* sink) {
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

// =============================================================================
//  Greedy maximum Edge-Disjoint Paths set
//    (Kleinberg & Salavatipour 2004 [KS04], recommended in EDPC paper
//     Section 3.1 — Beverland, Kliuchnikov & Schoute, arXiv:2110.11493v2,
//     "Surface code compilation via edge-disjoint paths".)
//
//  Iteratively find the shortest source->sink path in the current graph (BFS,
//  no flow cancellation), commit it to the EDP set, and ZERO the capacity of
//  every forward edge along it (the EDP edge-disjoint constraint).  REVERSE
//  edges are NOT incremented — the paper's greedy is pure edge-removal, not
//  max-flow with residuals.  This forces every committed path to be a true
//  forward path through the network, which is what we need for the bridge:
//  each path is an independent parallel module-flow channel through the
//  bottleneck.
//
//  For unit-capacity networks this is an O(sqrt(N))-approximation to max-EDP
//  on grids [KS04].  Optimal max-EDP would require residual Edmonds-Karp
//  (Menger's theorem); the paper deliberately favours the greedy because
//  shortest-path-first yields a compact, low-depth EDP set in practice and is
//  what EDPC implements.
// =============================================================================

int FlowGraph::findGreedyEdpPaths(Node* source, Node* sink) {
    int pathNum = 0;
    cout << "\n--- Greedy Max-EDP Paths (Kleinberg-Salavatipour 2004) ---\n";

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

        cout << "EDP path " << ++pathNum << ": " << source->key();
        for (Edge* e : path) cout << " -> " << e->to->key();
        cout << "\n";

        // Pure greedy edge-removal: zero the forward capacity.  Do NOT
        // increment the reverse edge — flow cancellation is not permitted in
        // the paper's greedy EDP algorithm.
        for (Edge* e : path) { e->capacity = 0; }

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

    if (pathNum == 0) cout << "No EDP paths found.\n";
    return pathNum;
}

// =============================================================================
//  Connectivity helpers
// =============================================================================

set<Node*> FlowGraph::findReachable(Node* source) {
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

// =============================================================================
//  Diagnostics
// =============================================================================

void FlowGraph::printNodes() {
    cout << "\n\n--- Flow Graph Nodes ---\n";
    for (const auto& kv : nodes) {
        if      (kv.first == "SUPER_SOURCE") cout << "Node: SUPER_SOURCE\n";
        else if (kv.first == "SUPER_SINK")   cout << "Node: SUPER_SINK\n";
        else                                  cout << "Node: " << kv.first << "\n";
    }
    cout << "\n";
}

void FlowGraph::printEdges() {
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

// =============================================================================
//  Picard-Queyranne all-min-cuts detection
//
//    J.-C. Picard & M. Queyranne, "On the structure of all minimum cuts in a
//    network and applications", EP-79-R-15, École Polytechnique de Montréal,
//    1979.
//
//  After running max-flow, define the residual relation R on V:
//      i R j  iff  ((i,j) in A and f_ij < c_ij)  OR  ((j,i) in A and f_ji > 0)
//  i.e. the residual graph.  Compute its strongly connected components.
//  By Corollary 6, a saturated arc (i,j) belongs to SOME minimum cut iff i and
//  j lie in different SCCs of R.  Returning every such arc thus yields the
//  union of all minimum-cut arcs across all minimum cuts.
// =============================================================================

// -----------------------------------------------------------------------------
//  computeResidualSCCs
//    Compute strongly connected components of the residual graph.  An arc
//    u->v exists iff some edge in u->edges has positive residual capacity to
//    v.  This includes both forward edges (cap>0 ⇔ f<c) and reverse edges
//    (cap>0 ⇔ f>0 on the original arc) — exactly Picard & Queyranne's R.
//
//    Uses an iterative implementation of Tarjan's algorithm to avoid deep
//    recursion.  Returns a map Node* -> SCC index.
// -----------------------------------------------------------------------------
unordered_map<Node*, int> FlowGraph::computeResidualSCCs() {
    unordered_map<Node*, int>  sccOf;
    unordered_map<Node*, int>  index;
    unordered_map<Node*, int>  lowlink;
    unordered_map<Node*, bool> onStack;
    vector<Node*>              tarjanStack;
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

// -----------------------------------------------------------------------------
//  findAllMinCutEdgesPicardQueyranne
//    Union of all min-cut arcs.  An original forward arc (u,v) (originalCap>0)
//    is in the union iff:
//      (a) it is saturated (current capacity == 0, i.e. f_uv = c_uv), AND
//      (b) SCC(u) != SCC(v) in the residual graph.
//    Reverse edges (originalCap == 0) are skipped.  Special source/sink
//    nodes are excluded from the reported Cell3DPosition pairs.
// -----------------------------------------------------------------------------
vector<pair<Cell3DPosition, Cell3DPosition>>
FlowGraph::findAllMinCutEdgesPicardQueyranne(
    const unordered_map<Edge*, int>& originalCap) {

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

// =============================================================================
//  Combined-bridge construction
//
//  The combined min-cut edge is identified EXACTLY as in
//  applicationsSrc/flowBridgeFinal: every Picard-Queyranne min-cut arc is
//  merged conceptually into one "big min-cut edge" with
//      start = source-most boundary = union of all tails  {u_i}
//      end   = sink-most   boundary = union of all heads  {v_j}
//  (with the overlap — nodes that are both tails and heads, i.e. pass-through
//  nodes inside the big edge — removed from both boundaries).
//
//  A single sub-flow-graph is built once over the empty-cell motion graph
//  spanning this big edge:
//      super-source -> every u_i  (INF capacity)
//      every v_j   -> super-sink  (INF capacity)
//  Expansion via addEdgesRec from each u_i populates the motion graph through
//  empty cells; heads {v_j} are pre-marked visited so expansion stops at the
//  sink boundary.  This sub-graph IS the geometric region in which the
//  combined min-cut edge lies — the bottleneck region — and the EDP search
//  runs over exactly this graph.
//
//  Best-bridge identification: instead of Edmonds-Karp with flow cancellation
//  (max-flow with residuals), we use the EDPC paper's algorithm — the greedy
//  maximum Edge-Disjoint Paths algorithm of Kleinberg & Salavatipour 2004
//  [KS04].  At each iteration we find the shortest super-source -> super-sink
//  path (BFS by hop count) and ZERO the forward capacity of every edge along
//  it; no reverse-edge cancellation is permitted.  Each committed path is a
//  true forward route through empty space, and the set of all committed paths
//  is an EDP set.
//
//  Why EDP for the bridge? Per EDPC §3.2, each path in an EDP set corresponds
//  to one operation executable in CONSTANT depth.  In the modular-robot
//  setting this becomes one INDEPENDENT parallel module-flow channel through
//  the bottleneck, so |EDP set| equals the parallelism delivered by the
//  bridge.  Bridge cells = deduplicated union of intermediate empty cells
//  across the EDP set.
// =============================================================================

void FlowGraph::findCombinedBridge(
    const vector<pair<Cell3DPosition, Cell3DPosition>>& minCutEdges) {

    Catoms3DFlowBridgeCode::combinedBridgePositions.clear();
    Catoms3DFlowBridgeCode::combinedBridgePaths.clear();

    cout << "\n\n--- Combined Bridge for All Min-Cut Edges ---\n" << flush;

    if (minCutEdges.empty()) {
        cout << "  No min-cut edges to bridge.\n";
        return;
    }

    // -------------------------------------------------------------------------
    //  Combined min-cut edge boundaries (matches flowBridgeFinal).
    //    Source-most boundary = union of all tails u_i.
    //    Sink-most   boundary = union of all heads v_j.
    //  Any node that is both a tail and a head is a "pass-through" — it sits
    //  inside the merged big edge.  Drop it from both boundaries so it is
    //  neither a source seed nor a sink seed (it can still appear as an
    //  intermediate inside the sub-graph).
    // -------------------------------------------------------------------------
    set<Cell3DPosition> tails, heads;
    for (const auto& [u, v] : minCutEdges) {
        tails.insert(u);
        heads.insert(v);
    }
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

    // -------------------------------------------------------------------------
    //  Sub-flow-graph build (matches flowBridgeFinal).
    //    super-source -> every u_i  (INF capacity)
    //    every v_j   -> super-sink  (INF capacity)
    //  Pre-wire heads to super-sink and mark them visited so addEdgesRec does
    //  not expand past them — they form the sink boundary of the combined
    //  sub-graph.  Wire each tail to super-source and expand through empty
    //  space; expansions across distinct tails share the nodeVisited set, so
    //  each motion-graph cell is processed at most once across the entire
    //  combined sub-graph.  This sub-graph IS the area where the combined
    //  min-cut edge lies — the bottleneck region.
    // -------------------------------------------------------------------------
    FlowGraph* bg = new FlowGraph();
    bg->superSource = bg->getOrCreateSpecial("SUPER_SOURCE");
    bg->superSink   = bg->getOrCreateSpecial("SUPER_SINK");

    set<pair<string,string>> ev;
    set<string> nv;

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

    // -------------------------------------------------------------------------
    //  Best-bridge identification via the paper's algorithm: greedy maximum
    //  Edge-Disjoint Paths (Kleinberg-Salavatipour 2004), the algorithm
    //  recommended in EDPC §3.1.  Each committed path is a parallel
    //  module-flow channel across the bottleneck; the EDP set's size is the
    //  bridge's parallelism.
    // -------------------------------------------------------------------------
    int flow = bg->findGreedyEdpPaths(bg->superSource, bg->superSink);
    cout << "  " << flow << " edge-disjoint path(s) in combined sub-graph.\n"
         << flush;

    // -------------------------------------------------------------------------
    //  Combined bridge = ⋃ intermediate cells over all EDP paths.
    //
    //  The motion-graph expansion can yield path nodes whose lattice cell is
    //  currently occupied (e.g. when an empty-cell pivot computation lands on
    //  a filled neighbour, or when a tail/head is revisited as an intermediate
    //  in another path).  Idle modules must only be placed on EMPTY lattice
    //  cells, so we filter intermediates down to those whose lattice cell is
    //  currently free.  Tails and heads themselves are also excluded — they
    //  form the existing structural boundary of the merged big min-cut edge
    //  and are already occupied.
    // -------------------------------------------------------------------------
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
            cout << "  EDP path " << (p+1) << ": no empty intermediates\n";
        } else {
            cout << "  EDP path " << (p+1) << " empty intermediates:";
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

// =============================================================================
//  Pivot bookkeeping
// =============================================================================

set<Cell3DPosition> FlowGraph::findPivotsInAugmentingPaths() {
    set<Cell3DPosition> usedPivots;
    for (const auto& path : augmentingPathPositions)
        for (size_t i = 0; i + 1 < path.size(); ++i)
            Catoms3DFlowBridgeCode::getPivotsForMotion(path[i], path[i+1], usedPivots);
    return usedPivots;
}

void FlowGraph::printIdleStructuralBlocks(vector<Cell3DPosition>& idleOut) {
    set<Cell3DPosition> usedPivots = findPivotsInAugmentingPaths();
    cout << "\n\n--- Idle Structural Blocks ---\n";
    for (const auto& pos : Catoms3DFlowBridgeCode::getStructuralBlocks()) {
        if (usedPivots.count(pos)) cout << "  [PIVOT]  " << pos.to_string() << "\n";
        else { cout << "  [IDLE]   " << pos.to_string() << "\n"; idleOut.push_back(pos); }
    }
    cout << idleOut.size() << " idle structural block(s).\n\n";
}

// =============================================================================
//  Source / sink flow reporting
//
//  For every original forward arc (originalCap > 0):
//      flow(e) = originalCap[e] - e->capacity
//  Total flow exiting the sources = Σ flow(e) over e in source->edges with
//  originalCap > 0 (the super-source -> real-source arcs).
//  Total flow entering the sinks  = Σ flow(e) over original forward arcs
//  whose head is the super-sink (i.e. the real-sink -> super-sink arcs).
//  By the flow-conservation theorem both totals equal the max-flow value.
// =============================================================================

void FlowGraph::reportSourceSinkFlows(
    const unordered_map<Edge*, int>& originalCap,
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

// =============================================================================
//  Top-level pipeline driver
//
//  Runs Edmonds-Karp, applies Picard-Queyranne to enumerate every arc
//  belonging to some minimum cut, then merges them into one big min-cut edge
//  and builds a single combined bridge across empty space spanning all the
//  min-cut endpoints.
// =============================================================================

int FlowGraph::startProcess(
    Node* source, Node* sink,
    vector<pair<Cell3DPosition,Cell3DPosition>>& outMinCutEdges) {

    auto originalCap = backupCapacities();
    int flow = findAndPrintAugmentingPaths(source, sink);
    reportSourceSinkFlows(originalCap, source, sink);

    // [viz] Stage 2 — main flow graph after Edmonds-Karp.
    {
        FlowGraphViz::ModuleSets sets;
        sets.sourceModules    = &Catoms3DFlowBridgeCode::getMovingBlocks();
        sets.targetCells      = &Catoms3DFlowBridgeCode::getTargetPositions();
        sets.structuralBlocks = &Catoms3DFlowBridgeCode::getStructuralBlocks();
        FlowGraphViz::dump(
            *this, "2_max_flow",
            "Main flow graph after Edmonds-Karp (augmenting paths overlaid)",
            sets, &originalCap, &augmentingPathPositions, nullptr);
    }

    outMinCutEdges = findAllMinCutEdgesPicardQueyranne(originalCap);

    // [viz] Stage 3 — Picard-Queyranne min-cut arcs over the residual graph.
    {
        FlowGraphViz::ModuleSets sets;
        sets.sourceModules    = &Catoms3DFlowBridgeCode::getMovingBlocks();
        sets.targetCells      = &Catoms3DFlowBridgeCode::getTargetPositions();
        sets.structuralBlocks = &Catoms3DFlowBridgeCode::getStructuralBlocks();
        FlowGraphViz::dump(
            *this, "3_min_cut",
            "Picard-Queyranne min-cut arcs (bold red) over residual graph",
            sets, &originalCap, nullptr, &outMinCutEdges);
    }

    findCombinedBridge(outMinCutEdges);
    return flow;
}
