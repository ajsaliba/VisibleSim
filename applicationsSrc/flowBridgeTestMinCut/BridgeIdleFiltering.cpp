// =============================================================================
//  BridgeIdleFiltering.cpp
//  -----------------------
//  Reduces the raw idleStructuralBlocks list (structural blocks NOT used as a
//  pivot in any augmenting path) to validIdleModules: idle blocks that are
//  actually MOVABLE without breaking lattice connectivity.
//
//  A block is dropped when either:
//    * it has no reachable neighbour cell             (isModuleBlocked), or
//    * removing it would split the lattice            (isArticulationPoint).
// =============================================================================

#include "Catoms3DFlowBridgeCode.h"

#include <iostream>
#include <queue>
#include <set>

using namespace std;
using namespace Catoms3D;

// =============================================================================
//  isModuleBlocked  —  empty reachable-cell list ⇒ module cannot move
// =============================================================================

bool Catoms3DFlowBridgeCode::isModuleBlocked(const Cell3DPosition& pos) {
    vector<Cell3DPosition> r;
    return !getAllPossibleMotionsFromPosition(pos, r);
}

// =============================================================================
//  isArticulationPoint
//    Returns true iff removing `pos` would disconnect the rest of the module
//    graph (structuralBlocks ∪ movingBlocks, restricted to active neighbours).
// =============================================================================

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

// =============================================================================
//  computeValidIdleModules  —  classifies and logs every idle block
// =============================================================================

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
