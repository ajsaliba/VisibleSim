#include <iostream>
#include <climits>
#include <algorithm>
#include <cassert>
#include <sstream>
#include "Catoms3DFlowBridgeCode.h"

using namespace std;

// ============================================================================
// Static member initialization
// ============================================================================
bool FlowBridgeCode::algorithmStarted = false;
bool FlowBridgeCode::algorithmComplete = false;
PosSet FlowBridgeCode::corePositions;
PosSet FlowBridgeCode::supplyPositions;
PosSet FlowBridgeCode::demandPositions;
FlowGraph FlowBridgeCode::flowGraph;
PosSet FlowBridgeCode::bridgePositions;
int FlowBridgeCode::iterationCount = 0;
Catoms3DBlock* FlowBridgeCode::movingModule = nullptr;
Cell3DPosition FlowBridgeCode::movingTarget;
PosMap FlowBridgeCode::distToDemand;

// ============================================================================
// FlowGraph Implementation
// ============================================================================

void FlowGraph::clear() {
    posToNode.clear();
    nodeToPos.clear();
    adj.clear();
    nodeCount = 0;
    source = -1;
    sink = -1;
    maxFlowValue = 0;
}

int FlowGraph::addNode(const Cell3DPosition &pos) {
    int id = nodeCount++;
    posToNode[pos] = id;
    nodeToPos.push_back(pos);
    adj.push_back({});
    return id;
}

int FlowGraph::addVirtualNode() {
    // Virtual nodes (source/sink) use a sentinel position
    Cell3DPosition virt(-999 - nodeCount, -999 - nodeCount, -999 - nodeCount);
    return addNode(virt);
}

int FlowGraph::getOrAddNode(const Cell3DPosition &pos) {
    auto it = posToNode.find(pos);
    if (it != posToNode.end()) return it->second;
    return addNode(pos);
}

void FlowGraph::addEdge(int from, int to, int cap) {
    Edge fwd = {to, cap, static_cast<int>(adj[to].size())};
    Edge rev = {from, 0, static_cast<int>(adj[from].size())};
    adj[from].push_back(fwd);
    adj[to].push_back(rev);
}

bool FlowGraph::bfs(vector<int> &parent, vector<int> &parentEdge) const {
    parent.assign(nodeCount, -1);
    parentEdge.assign(nodeCount, -1);
    parent[source] = source;
    queue<int> q;
    q.push(source);
    while (!q.empty()) {
        int u = q.front(); q.pop();
        for (int i = 0; i < static_cast<int>(adj[u].size()); i++) {
            const Edge &e = adj[u][i];
            if (parent[e.to] == -1 && e.cap > 0) {
                parent[e.to] = u;
                parentEdge[e.to] = i;
                if (e.to == sink) return true;
                q.push(e.to);
            }
        }
    }
    return false;
}

// Phase 2: Edmonds-Karp max-flow
int FlowGraph::maxFlow() {
    maxFlowValue = 0;
    vector<int> parent, parentEdge;

    while (bfs(parent, parentEdge)) {
        // Find bottleneck (always 1 for unit-capacity)
        int pathFlow = INT_MAX;
        for (int v = sink; v != source; v = parent[v]) {
            int u = parent[v];
            int ei = parentEdge[v];
            pathFlow = min(pathFlow, adj[u][ei].cap);
        }

        // Symmetrization: update residual graph
        for (int v = sink; v != source; v = parent[v]) {
            int u = parent[v];
            int ei = parentEdge[v];
            adj[u][ei].cap -= pathFlow;                       // decrement forward
            adj[adj[u][ei].to][adj[u][ei].rev].cap += pathFlow; // increment reverse
        }

        maxFlowValue += pathFlow;
    }

    cout << "[Phase 2] Max flow = " << maxFlowValue << endl;
    return maxFlowValue;
}

// Phase 3: Source-reachable set in residual graph
vector<bool> FlowGraph::getSourceReachable() const {
    vector<bool> visited(nodeCount, false);
    stack<int> stk;
    stk.push(source);
    visited[source] = true;
    while (!stk.empty()) {
        int u = stk.top(); stk.pop();
        for (const Edge &e : adj[u]) {
            if (!visited[e.to] && e.cap > 0) {
                visited[e.to] = true;
                stk.push(e.to);
            }
        }
    }
    return visited;
}

// Phase 3: Min-cut edges
vector<pair<int, int>> FlowGraph::getMinCutEdges() const {
    vector<bool> reachable = getSourceReachable();
    vector<pair<int, int>> cutEdges;
    for (int u = 0; u < nodeCount; u++) {
        if (!reachable[u]) continue;
        for (const Edge &e : adj[u]) {
            // A saturated forward edge crossing the cut
            if (!reachable[e.to] && e.cap == 0) {
                cutEdges.push_back({u, e.to});
            }
        }
    }
    return cutEdges;
}

// Phase 5: Extract disjoint streamlines from the flow
vector<vector<Cell3DPosition>> FlowGraph::extractStreamlines() const {
    vector<vector<Cell3DPosition>> paths;

    // Build flow adjacency from saturated forward edges
    // For each node, track which outgoing edges carry flow
    vector<vector<pair<int, bool>>> flowOut(nodeCount); // {neighbor, consumed}
    for (int u = 0; u < nodeCount; u++) {
        for (const Edge &e : adj[u]) {
            // A forward edge carries flow if its capacity is 0 and the reverse has cap > 0
            if (e.cap == 0 && adj[e.to][e.rev].cap > 0) {
                flowOut[u].push_back({e.to, false});
            }
        }
    }

    // Greedily trace paths from source to sink
    while (true) {
        // BFS/DFS to find one path
        vector<int> parent(nodeCount, -1);
        vector<int> parentIdx(nodeCount, -1);
        parent[source] = source;
        queue<int> q;
        q.push(source);
        bool found = false;

        while (!q.empty() && !found) {
            int u = q.front(); q.pop();
            for (int i = 0; i < static_cast<int>(flowOut[u].size()); i++) {
                auto &[v, consumed] = flowOut[u][i];
                if (!consumed && parent[v] == -1) {
                    parent[v] = u;
                    parentIdx[v] = i;
                    if (v == sink) { found = true; break; }
                    q.push(v);
                }
            }
        }

        if (!found) break;

        // Reconstruct and mark consumed
        vector<int> nodePath;
        for (int v = sink; v != source; v = parent[v]) {
            nodePath.push_back(v);
            flowOut[parent[v]][parentIdx[v]].second = true; // mark consumed
        }
        nodePath.push_back(source);
        reverse(nodePath.begin(), nodePath.end());

        // Convert to positions, skipping source and sink virtual nodes
        vector<Cell3DPosition> posPath;
        for (size_t i = 1; i + 1 < nodePath.size(); i++) {
            posPath.push_back(nodeToPos[nodePath[i]]);
        }
        if (!posPath.empty()) {
            paths.push_back(posPath);
        }
    }

    return paths;
}

// ============================================================================
// FlowBridgeCode Implementation
// ============================================================================

FlowBridgeCode::FlowBridgeCode(Catoms3DBlock *host) : Catoms3DBlockCode(host), module(host) {}

FlowBridgeCode::~FlowBridgeCode() {}

FCCLattice* FlowBridgeCode::getLattice() const {
    return static_cast<FCCLattice*>(Catoms3D::getWorld()->lattice);
}

Catoms3DBlock* FlowBridgeCode::getBlockAt(const Cell3DPosition &pos) const {
    return static_cast<Catoms3DBlock*>(getLattice()->getBlock(pos));
}

PosSet FlowBridgeCode::getAllOccupied() const {
    PosSet occupied;
    auto &blocks = Catoms3D::getWorld()->getMap();
    for (auto &pair : blocks) {
        occupied.insert(pair.second->position);
    }
    return occupied;
}

// ============================================================================
// startup: Leader module orchestrates the centralized algorithm
// ============================================================================
void FlowBridgeCode::startup() {
    if (module->blockId != 1 || algorithmStarted) return;

    algorithmStarted = true;
    module->setColor(YELLOW);
    cout << "\033[1;36m=== Flow Bridge Reconfiguration Algorithm ===\033[0m" << endl;

    identifySets();

    if (demandPositions.empty()) {
        cout << "[Done] Configuration is already complete." << endl;
        algorithmComplete = true;
        return;
    }
    if (supplyPositions.empty()) {
        cout << "[Error] No supply modules available." << endl;
        algorithmComplete = true;
        return;
    }

    scheduleNextIteration();
}

// ============================================================================
// Compute BFS distance from every occupied/demand cell to nearest demand
// ============================================================================
void FlowBridgeCode::computeDistanceToDemand() {
    distToDemand.clear();
    FCCLattice *lattice = getLattice();

    queue<Cell3DPosition> q;
    for (const auto &dp : demandPositions) {
        distToDemand[dp] = 0;
        q.push(dp);
    }

    // BFS through ALL in-grid positions so that empty rotation destinations
    // (returned by getAllMotions()) also get valid distance values.
    while (!q.empty()) {
        Cell3DPosition cur = q.front(); q.pop();
        int curDist = distToDemand[cur];
        vector<Cell3DPosition> neighbors = lattice->getNeighborhood(cur);
        for (const auto &npos : neighbors) {
            if (!distToDemand.count(npos) && lattice->isInGrid(npos)) {
                distToDemand[npos] = curDist + 1;
                q.push(npos);
            }
        }
    }
}

// ============================================================================
// Event-driven iteration: pick one supply module and move it one step
// ============================================================================
void FlowBridgeCode::scheduleNextIteration() {
    const int MAX_ITERATIONS = 500;

    if (demandPositions.empty()) {
        cout << "\n\033[1;32m=== Reconfiguration Complete! ===\033[0m" << endl;
        algorithmComplete = true;
        return;
    }
    if (supplyPositions.empty() || iterationCount >= MAX_ITERATIONS) {
        cout << "\n\033[1;31m=== Reconfiguration Incomplete. Remaining demand: "
             << demandPositions.size() << " ===\033[0m" << endl;
        algorithmComplete = true;
        return;
    }

    iterationCount++;
    cout << "\n--- Iteration " << iterationCount << " ---" << endl;
    cout << "  Supply: " << supplyPositions.size()
         << "  Demand: " << demandPositions.size() << endl;

    // Phase 1: Build flow graph
    buildFlowGraph();

    // Phase 2: Compute max-flow
    int flow = flowGraph.maxFlow();

    // Phase 3: Min-cut bottleneck analysis
    auto cutEdges = flowGraph.getMinCutEdges();
    cout << "[Phase 3] " << cutEdges.size() << " bottleneck edges." << endl;

    // Phase 4: Bridge deployment if flow is limited
    if (flow < static_cast<int>(min(supplyPositions.size(), demandPositions.size()))) {
        bool bridgesDeployed = planAndDeployBridges();
        if (bridgesDeployed) {
            // Bridge module is now moving via moveTo(); onMotionEnd will continue
            return;
        }
    }

    // Phase 5: Move one supply module one step toward demand using moveTo()
    computeDistanceToDemand();

    if (!tryMoveSupplyModule()) {
        cout << "[Phase 5] No valid move found. Stopping." << endl;
        algorithmComplete = true;
    }
}

// ============================================================================
// Find best moveTo destination for a block: closest to any demand position
// ============================================================================
Cell3DPosition FlowBridgeCode::findBestMoveTarget(Catoms3DBlock *block) const {
    auto allMotions = block->getAllMotions();
    if (allMotions.empty()) return Cell3DPosition(-1, -1, -1);

    Cell3DPosition bestDest(-1, -1, -1);
    int bestDist = INT_MAX;

    for (const auto &[dest, orient] : allMotions) {
        // Prefer destinations that decrease distance to demand
        auto it = distToDemand.find(dest);
        if (it != distToDemand.end()) {
            if (it->second < bestDist) {
                bestDist = it->second;
                bestDest = dest;
            }
        }
        // Also consider demand positions directly (distance 0)
        if (demandPositions.count(dest) && 0 < bestDist) {
            bestDist = 0;
            bestDest = dest;
        }
    }

    return bestDest;
}

// ============================================================================
// Phase 5: Try to move one supply module one step toward demand
// ============================================================================
bool FlowBridgeCode::tryMoveSupplyModule() {
    PosSet occupied = getAllOccupied();

    // Strategy: pick the supply module that can make the most progress
    // toward a demand position (smallest resulting distance to demand).
    Catoms3DBlock *bestBlock = nullptr;
    Cell3DPosition bestDest(-1, -1, -1);
    int bestDist = INT_MAX;

    for (const auto &spos : supplyPositions) {
        Catoms3DBlock *block = getBlockAt(spos);
        if (!block) continue;

        // Skip if removing this module would disconnect the structure
        if (!isStructureConnected(occupied, spos)) continue;

        Cell3DPosition dest = findBestMoveTarget(block);
        if (dest[0] == -1) continue; // no valid move

        int dist = distToDemand.count(dest) ? distToDemand[dest] : INT_MAX;
        if (demandPositions.count(dest)) dist = 0;

        // Prefer moving into demand directly, then smallest distance
        if (dist < bestDist) {
            bestDist = dist;
            bestBlock = block;
            bestDest = dest;
        }
    }

    // Also try core modules that are NOT in any demand position AND can reach
    // closer to demand. This handles cases where supply needs to traverse the
    // core surface.
    if (!bestBlock) {
        // No supply module can move directly; try to find any non-essential
        // occupied module on the boundary that can shuffle toward demand
        for (const auto &cpos : corePositions) {
            // Don't move modules that are already in target positions that are needed
            if (demandPositions.count(cpos)) continue;
            Catoms3DBlock *block = getBlockAt(cpos);
            if (!block) continue;
            if (!isStructureConnected(occupied, cpos)) continue;

            Cell3DPosition dest = findBestMoveTarget(block);
            if (dest[0] == -1) continue;

            int dist = distToDemand.count(dest) ? distToDemand[dest] : INT_MAX;
            if (demandPositions.count(dest)) dist = 0;

            int curDist = distToDemand.count(cpos) ? distToDemand[cpos] : INT_MAX;
            // Only move if it gets closer to demand
            if (dist < curDist && dist < bestDist) {
                bestDist = dist;
                bestBlock = block;
                bestDest = dest;
            }
        }
    }

    if (!bestBlock) return false;

    cout << "  Move: " << bestBlock->position << " -> " << bestDest << endl;

    movingModule = bestBlock;
    movingTarget = bestDest;

    bool ok = bestBlock->moveTo(bestDest);
    if (!ok) {
        cout << "  moveTo() failed for " << bestBlock->position << " -> " << bestDest << endl;
        return false;
    }

    return true;
}

// ============================================================================
// onMotionEnd: Called when a module finishes its rotation
// ============================================================================
void FlowBridgeCode::onMotionEnd() {
    if (algorithmComplete) return;

    // Re-classify after the move
    identifySets();

    // Check if complete
    if (demandPositions.empty()) {
        cout << "\n\033[1;32m=== Reconfiguration Complete! ===\033[0m" << endl;
        algorithmComplete = true;
        return;
    }

    // If the module that just moved is still a supply module (not yet in target),
    // see if it can continue moving toward demand
    Catoms3DBlock *movedBlock = static_cast<Catoms3DBlock*>(hostBlock);
    if (supplyPositions.count(movedBlock->position)) {
        computeDistanceToDemand();
        PosSet occupied = getAllOccupied();

        if (isStructureConnected(occupied, movedBlock->position)) {
            Cell3DPosition nextDest = findBestMoveTarget(movedBlock);
            if (nextDest[0] != -1) {
                int curDist = distToDemand.count(movedBlock->position)
                    ? distToDemand[movedBlock->position] : INT_MAX;
                int nextDistVal = demandPositions.count(nextDest) ? 0
                    : (distToDemand.count(nextDest) ? distToDemand[nextDest] : INT_MAX);

                if (nextDistVal < curDist) {
                    cout << "  Continue: " << movedBlock->position << " -> " << nextDest << endl;
                    if (movedBlock->moveTo(nextDest)) {
                        return; // wait for next onMotionEnd
                    }
                }
            }
        }
    }

    // Otherwise, start a fresh iteration
    scheduleNextIteration();
}

// ============================================================================
// Phase 1: Identify Core, Supply, and Demand sets
// ============================================================================
void FlowBridgeCode::identifySets() {
    corePositions.clear();
    supplyPositions.clear();
    demandPositions.clear();

    PosSet occupied = getAllOccupied();

    if (target == nullptr) {
        cout << "[Warning] No target defined. Using default behavior." << endl;
        // Without a target, treat all modules as core
        corePositions = occupied;
        return;
    }

    // Core = modules in target positions
    // Supply = modules NOT in target positions (surplus)
    for (const auto &pos : occupied) {
        if (target->isInTarget(pos)) {
            corePositions.insert(pos);
        } else {
            supplyPositions.insert(pos);
        }
    }

    // Demand = target positions that are empty
    // We need to scan the target. For TargetGrid, iterate tCells.
    // Since Target is abstract, we probe positions in the grid neighborhood
    // of existing modules to discover demand positions.
    FCCLattice *lattice = getLattice();
    Cell3DPosition gsz = lattice->gridSize;

    for (short x = 0; x < gsz[0]; x++) {
        for (short y = 0; y < gsz[1]; y++) {
            for (short z = 0; z < gsz[2]; z++) {
                Cell3DPosition pos(x, y, z);
                if (lattice->isInGrid(pos) && target->isInTarget(pos)
                    && !occupied.count(pos)) {
                    demandPositions.insert(pos);
                }
            }
        }
    }

    // Color modules for visualization
    auto &blocks = Catoms3D::getWorld()->getMap();
    for (auto &pair : blocks) {
        Cell3DPosition p = pair.second->position;
        if (supplyPositions.count(p)) {
            pair.second->setColor(RED);      // supply = red
        } else if (corePositions.count(p)) {
            pair.second->setColor(GREEN);    // core = green
        }
    }

    cout << "[Phase 1] Core: " << corePositions.size()
         << ", Supply: " << supplyPositions.size()
         << ", Demand: " << demandPositions.size() << endl;
}

// ============================================================================
// Phase 1: Build the directed flow graph G=(V, E)
// ============================================================================
void FlowBridgeCode::buildFlowGraph() {
    flowGraph.clear();

    PosSet occupied = getAllOccupied();
    FCCLattice *lattice = getLattice();

    // Add only occupied positions and demand positions as nodes.
    // Nav layer (empty) cells are excluded — the bubble handover only works
    // through occupied cells where blocks can chain-shift.
    for (const auto &pos : corePositions) {
        flowGraph.getOrAddNode(pos);
    }
    for (const auto &pos : supplyPositions) {
        flowGraph.getOrAddNode(pos);
    }
    for (const auto &pos : demandPositions) {
        flowGraph.getOrAddNode(pos);
    }
    for (const auto &pos : bridgePositions) {
        if (occupied.count(pos)) flowGraph.getOrAddNode(pos);
    }

    // Create supersource (s) and supersink (t)
    flowGraph.source = flowGraph.addVirtualNode();
    flowGraph.sink = flowGraph.addVirtualNode();

    // The "bubble perspective": flow = virtual bubbles from demand toward supply
    // supersource -> all demand roots
    for (const auto &dpos : demandPositions) {
        int dnode = flowGraph.posToNode[dpos];
        flowGraph.addEdge(flowGraph.source, dnode, 1);
    }

    // all supply roots -> supersink
    for (const auto &spos : supplyPositions) {
        int snode = flowGraph.posToNode[spos];
        flowGraph.addEdge(snode, flowGraph.sink, 1);
    }

    // Internal edges: directed from demand toward supply through occupied cells.
    // The bubble handover moves vacancies from demand toward supply, so edges
    // point from cells closer to demand toward cells farther from demand.

    // Compute BFS distance from demand positions
    unordered_map<Cell3DPosition, int, Cell3DPositionHash> distFromDemand;
    queue<Cell3DPosition> bfsQ;
    for (const auto &dp : demandPositions) {
        distFromDemand[dp] = 0;
        bfsQ.push(dp);
    }
    while (!bfsQ.empty()) {
        Cell3DPosition cur = bfsQ.front(); bfsQ.pop();
        int curDist = distFromDemand[cur];
        vector<Cell3DPosition> neighbors = lattice->getNeighborhood(cur);
        for (const auto &npos : neighbors) {
            if (!distFromDemand.count(npos) && flowGraph.posToNode.count(npos)) {
                distFromDemand[npos] = curDist + 1;
                bfsQ.push(npos);
            }
        }
    }

    // Add directed edges: from closer-to-demand to further-from-demand
    PosSet allGraphNodes;
    for (const auto &p : flowGraph.posToNode) {
        // Skip virtual nodes
        if (p.first[0] <= -999) continue;
        allGraphNodes.insert(p.first);
    }

    for (const auto &pos : allGraphNodes) {
        if (!distFromDemand.count(pos)) continue;
        int myDist = distFromDemand[pos];
        int myNode = flowGraph.posToNode[pos];

        vector<Cell3DPosition> neighbors = lattice->getNeighborhood(pos);
        for (const auto &npos : neighbors) {
            if (!flowGraph.posToNode.count(npos)) continue;
            if (!distFromDemand.count(npos)) continue;

            int nDist = distFromDemand[npos];
            int nNode = flowGraph.posToNode[npos];

            // Direct edge from demand toward supply (increasing distance)
            if (nDist > myDist) {
                flowGraph.addEdge(myNode, nNode, 1);
            }
        }
    }

    cout << "[Phase 1] Graph: " << flowGraph.nodeCount << " nodes, "
         << "source=" << flowGraph.source << ", sink=" << flowGraph.sink << endl;
}

// ============================================================================
// Phase 4: Plan and deploy bridge modules at bottleneck positions
// Uses moveTo for proper physical motion. Only deploys a bridge if a supply
// module can physically rotate into the candidate position.
// ============================================================================
bool FlowBridgeCode::planAndDeployBridges() {
    auto cutEdges = flowGraph.getMinCutEdges();
    if (cutEdges.empty()) return false;

    FCCLattice *lattice = getLattice();
    PosSet occupied = getAllOccupied();

    for (const auto &[uIdx, vIdx] : cutEdges) {
        Cell3DPosition uPos = flowGraph.nodeToPos[uIdx];
        Cell3DPosition vPos = flowGraph.nodeToPos[vIdx];

        if (uPos[0] <= -999 || vPos[0] <= -999) continue;

        vector<Cell3DPosition> uNeighbors = lattice->getNeighborhood(uPos);

        for (const auto &candidate : uNeighbors) {
            if (!lattice->isInGrid(candidate)) continue;
            if (occupied.count(candidate)) continue;
            if (bridgePositions.count(candidate)) continue;

            // Find an idle supply module that can moveTo the candidate
            for (const auto &spos : supplyPositions) {
                if (demandPositions.size() >= supplyPositions.size()) break;
                Catoms3DBlock *sBlock = getBlockAt(spos);
                if (!sBlock) continue;
                if (!isStructureConnected(occupied, spos)) continue;
                if (!sBlock->canMoveTo(candidate)) continue;

                cout << "[Phase 4] Deploying bridge: " << spos
                     << " -> " << candidate << endl;

                bridgePositions.insert(candidate);
                sBlock->moveTo(candidate);
                return true; // One bridge at a time, wait for onMotionEnd
            }
        }
    }

    return false;
}

// ============================================================================
// Utility: Check if removing a module disconnects the structure
// ============================================================================
bool FlowBridgeCode::isStructureConnected(const PosSet &occupied,
                                           const Cell3DPosition &exclude) const {
    if (occupied.size() <= 1) return true;

    FCCLattice *lattice = getLattice();

    // Find a starting position (not the excluded one)
    Cell3DPosition start(-1, -1, -1);
    for (const auto &pos : occupied) {
        if (pos != exclude) {
            start = pos;
            break;
        }
    }
    if (start[0] == -1) return true;

    // BFS from start, count reachable
    PosSet visited;
    queue<Cell3DPosition> q;
    q.push(start);
    visited.insert(start);

    while (!q.empty()) {
        Cell3DPosition cur = q.front(); q.pop();
        vector<Cell3DPosition> neighbors = lattice->getNeighborhood(cur);
        for (const auto &npos : neighbors) {
            if (npos != exclude && occupied.count(npos) && !visited.count(npos)) {
                visited.insert(npos);
                q.push(npos);
            }
        }
    }

    // Expected count = occupied.size() - 1 (excluding the removed module)
    size_t expected = occupied.count(exclude) ? occupied.size() - 1 : occupied.size();
    return visited.size() == expected;
}
