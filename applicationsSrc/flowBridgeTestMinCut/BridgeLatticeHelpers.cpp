// =============================================================================
//  BridgeLatticeHelpers.cpp
//  ------------------------
//  Low-level lattice / motion utilities used everywhere else:
//
//      findPath(start, goal)                       — empty-cell BFS
//      findPathForModule(mod, start, goal)         — module-aware BFS
//      getAllPossibleMotionsFromPosition(pos, out) — one-hop neighbour query
//      getPivotsForMotion(from, to, out)           — pivots enabling a motion
//
//  These wrap the Catoms3D motion-rules engine but expose a Cell3DPosition
//  surface, so callers never need to touch Catoms3DMotionRulesLink directly.
// =============================================================================

#include "Catoms3DFlowBridgeCode.h"
#include "robots/catoms3D/catoms3DMotionEngine.h"

#include <algorithm>
#include <queue>
#include <set>
#include <map>

using namespace std;
using namespace Catoms3D;

// =============================================================================
//  findPath  —  empty-cell BFS using getAllPossibleMotionsFromPosition
// =============================================================================

vector<Cell3DPosition>
Catoms3DFlowBridgeCode::findPath(const Cell3DPosition &start,
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

// =============================================================================
//  findPathForModule  —  module-aware BFS
//
//  At each position along the BFS frontier we simulate the module being there
//  and query mod->canMoveTo for its free neighbours.  This is accurate only
//  when the module is physically AT 'start' (the lattice pivot structure is
//  correct for its actual location).  For intermediate BFS nodes we fall back
//  to the empty-cell pivot query (getAllPossibleMotions), which may
//  overestimate reachability — but the first step is guaranteed correct
//  because we call mod->canMoveTo directly from start.
//
//  Strategy:
//    Step 0 -> step 1 : use mod->canMoveTo (exact).
//    Step 1 onward    : use getAllPossibleMotionsFromPosition (approximate,
//                       re-validated at dispatch time).
// =============================================================================

vector<Cell3DPosition>
Catoms3DFlowBridgeCode::findPathForModule(Catoms3DBlock *mod,
                                          const Cell3DPosition &start,
                                          const Cell3DPosition &goal) {
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

// =============================================================================
//  getAllPossibleMotionsFromPosition
//
//  Two cases:
//    (a) A module currently sits at 'position'  ->  query canMoveTo on its
//                                                   free neighbours.
//    (b) The cell is empty                      ->  enumerate motions OUT OF
//                                                   each active neighbour's
//                                                   connector that begins at
//                                                   this position.
// =============================================================================

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

// =============================================================================
//  getPivotsForMotion
//
//  Reports every pivot that could enable the rotation fromPos -> toPos.  If a
//  module is physically at fromPos we ask the motion engine directly; if the
//  source cell is empty we enumerate from each active neighbour.
// =============================================================================

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
