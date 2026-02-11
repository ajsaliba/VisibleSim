/**
 * @file   flowBridgeCatoms3DCode.cpp
 * @brief  Connectivity-preserving single-module motion for Catoms3D.
 *
 *  Catoms3D Model Interpretation
 *
 *  Communication model
 * 
 *  Each Catoms3D module sits on a Face-Centered Cubic (FCC) lattice
 *  cell and exposes 12 connectors (one per FCC neighbor direction).
 *  Communication is strictly local: a module can only exchange
 *  messages with modules physically docked on one of its connectors.
 *  Neighbor detection is done via P2PNetworkInterface objects.
 *
 *  Motion model
 * 
 *  A module moves by *pivoting* around a neighboring module (the
 *  "pivot").  Two kinds of rotation exist:
 *    • HexaFace rotation – across one of 8 hexagonal surface faces,
 *      involving 3 connectors.
 *    • OctaFace rotation – across one of 6 octagonal surface faces,
 *      involving 4 connectors.
 *  A rotation is valid only when:
 *    1. The destination cell is empty.
 *    2. A suitable pivot neighbor exists.
 *    3. None of the "blocking" connectors on the shared face are
 *       occupied by other modules (swept-volume free).
 *  The simulator enforces these rules inside Catoms3DMotionEngine.
 *
 *  Connectivity preservation
 * 
 *  Before every step we check that *removing the mobile module from
 *  its current cell* does NOT disconnect the remaining modules.
 *  This is done via BFS over the FCC lattice grid, skipping the
 *  mobile's current position.  Only if the remaining set forms one
 *  connected component do we proceed.
 *
 *  Algorithm overview
 *
 *  1. On startup every module logs itself.  Exactly one module is
 *     elected as the *mobile* module — the one with the smallest
 *     block-id among non-articulation-point modules on the bridge
 *     (z == 2) near the left side (low x).
 *
 *  2. The target cell is set near the right end of the bridge.
 *
 *  3. At each step the mobile module:
 *       a) Enumerates all cells reachable in one Catoms3D rotation
 *          (via Catoms3DMotionEngine::getAllReachablePositions).
 *       b) Filters out any candidate whose approach would break
 *          connectivity (isConnectedWithout check).
 *       c) Among valid candidates, picks the one closest to the
 *          target (Euclidean distance).
 *       d) Schedules the rotation via Catoms3DBlock::moveTo().
 *
 *  4. On onMotionEnd(), step 3 is repeated until the target is
 *     reached or no valid step exists.
 *
 * @date   2026-02-11
 */

#include "flowBridgeCatoms3DCode.hpp"
#include <cmath>
#include <algorithm>
#include <iostream>
#include <limits>

using namespace std;

//  Static members

bool           FlowBridgeCatoms3DCode::mobileChosen = false;
bID            FlowBridgeCatoms3DCode::mobileId      = 0;
Cell3DPosition FlowBridgeCatoms3DCode::targetPos;

//  Construction

FlowBridgeCatoms3DCode::FlowBridgeCatoms3DCode(Catoms3DBlock *host)
    : Catoms3DBlockCode(host), module(host)
{
}

//  Connectivity check — BFS excluding one position

/**
 * Performs a BFS over occupied FCC lattice cells, purposely *skipping*
 * the cell at @p pos.  Returns true iff all remaining modules are
 * in one connected component.
 *
 * Uses Lattice::getActiveNeighborCells() which returns absolute
 * positions of occupied FCC neighbors that are in the grid.
 *
 * Complexity: O(N) where N = number of modules.
 */
bool FlowBridgeCatoms3DCode::isConnectedWithout(const Cell3DPosition &pos) const
{
    Catoms3DWorld *world   = Catoms3DWorld::getWorld();
    Lattice      *lattice = world->lattice;

    // Collect every occupied position except @p pos.
    set<Cell3DPosition> occupied;
    for (auto &[id, block] : world->getMap()) {
        if (block->position != pos) {
            occupied.insert(block->position);
        }
    }

    if (occupied.empty()) return true;           // trivially connected

    // BFS from an arbitrary remaining module.
    set<Cell3DPosition>   visited;
    queue<Cell3DPosition> frontier;
    frontier.push(*occupied.begin());
    visited.insert(*occupied.begin());

    while (!frontier.empty()) {
        Cell3DPosition cur = frontier.front();
        frontier.pop();
        // Use lattice helper: returns absolute positions of occupied
        // neighbors that are in the grid.
        vector<Cell3DPosition> neighbors =
            lattice->getActiveNeighborCells(cur);
        for (const Cell3DPosition &nb : neighbors) {
            if (nb != pos && occupied.count(nb) && !visited.count(nb)) {
                visited.insert(nb);
                frontier.push(nb);
            }
        }
    }

    return visited.size() == occupied.size();
}

//  Motion planner — greedy single step

/**
 * Enumerate reachable positions for this module, keep only those that
 * preserve connectivity, then advance toward targetPos.
 */
bool FlowBridgeCatoms3DCode::planNextStep()
{
    // 1. Already at target?
    if (module->position == targetPos) {
        console << "Module " << module->blockId
                << " ARRIVED at target " << targetPos << "\n";
        module->setColor(GREEN);
        return false;   // nothing to do
    }

    // 2. Connectivity check: can we leave the current cell?
    //    (If the mobile module is an articulation point right now,
    //     we cannot move at all this step.)
    if (!isConnectedWithout(module->position)) {
        console << "Module " << module->blockId
                << " is an articulation point — CANNOT move.\n";
        module->setColor(RED);
        return false;
    }

    // 3. Enumerate every cell reachable in one valid Catoms3D rotation.
    vector<Cell3DPosition> reachable =
        Catoms3DMotionEngine::getAllReachablePositions(module);

    if (reachable.empty()) {
        console << "Module " << module->blockId
                << " has no reachable positions — stuck.\n";
        module->setColor(RED);
        return false;
    }

    // 4. Filter: keep only candidates that maintain connectivity.
    //    After the mobile leaves its current cell and lands on
    //    the candidate cell, the occupied set changes:
    //      remove(currentPos)  →  add(candidatePos)
    //    We approximate the connectivity requirement by checking
    //    that removing currentPos does NOT disconnect the rest.
    //    (Already checked above — the mobile is NOT an articulation
    //     point, so every landing that re-attaches it to at least
    //     one existing neighbor is safe.)
    //
    //    As an extra safety measure we also verify that the candidate
    //    position has at least one lattice neighbor that is occupied
    //    (other than the mobile itself), guaranteeing the mobile
    //    re-joins the structure.
    Catoms3DWorld *world   = Catoms3DWorld::getWorld();
    Lattice      *lattice = world->lattice;

    struct Candidate {
        Cell3DPosition pos;
        double         dist;      // Euclidean² to target
    };
    vector<Candidate> valid;

    for (const Cell3DPosition &cand : reachable) {
        // Check that at least one FCC neighbor of cand (besides the
        // mobile's current position) is occupied → re-attachment.
        vector<Cell3DPosition> nbOffsets =
            lattice->getRelativeConnectivity(cand);
        bool hasNeighbor = false;
        for (const Cell3DPosition &off : nbOffsets) {
            Cell3DPosition nb = cand + off;
            if (nb != module->position && lattice->cellHasBlock(nb)) {
                hasNeighbor = true;
                break;
            }
        }
        if (!hasNeighbor) continue;   // would dangle — skip

        // Euclidean² distance to goal.
        double dx = cand[0] - targetPos[0];
        double dy = cand[1] - targetPos[1];
        double dz = cand[2] - targetPos[2];
        double d2 = dx * dx + dy * dy + dz * dz;
        valid.push_back({cand, d2});
    }

    if (valid.empty()) {
        console << "Module " << module->blockId
                << " has reachable cells but none preserve connectivity.\n";
        module->setColor(RED);
        return false;
    }

    // 5. Pick the candidate closest to the target.
    sort(valid.begin(), valid.end(),
         [](const Candidate &a, const Candidate &b) {
             return a.dist < b.dist;
         });

    const Cell3DPosition &best = valid.front().pos;

    console << "Module " << module->blockId
            << " moving: " << module->position
            << " → " << best
            << "  (dist² to goal = " << valid.front().dist << ")\n";

    // 6. Schedule the rotation.
    //    moveTo() internally validates Catoms3D rotation constraints
    //    (pivot existence, blocking connectors, destination freedom)
    //    and schedules a Catoms3DRotationStartEvent.
    bool ok = module->moveTo(best);
    if (!ok) {
        // moveTo can fail if the engine cannot find a valid rotation
        // link even though the cell appeared in getAllReachablePositions
        // (e.g. orientation mismatch).  Try the next candidate.
        for (size_t i = 1; i < valid.size(); ++i) {
            console << "  retry → " << valid[i].pos << "\n";
            if (module->moveTo(valid[i].pos)) return true;
        }
        console << "Module " << module->blockId
                << " could not execute any valid rotation.\n";
        module->setColor(RED);
        return false;
    }
    return true;
}

//  Lifecycle: startup

/**
 * Called once per module when the simulation starts.
 *
 * Election logic (executed identically on every module, but the
 * result is deterministic):
 *   • Scan all modules on the bridge layer (z == 2).
 *   • Among those that are NOT articulation points, pick the one
 *     with the smallest x (nearest the left / beginning of bridge).
 *     Ties broken by smallest y, then smallest block-id.
 *   • The target cell is chosen as the free cell nearest the right
 *     end of the bridge at (8, 4, 2) — we pick (9, 4, 2) which is
 *     on top of the right pillar and should be empty.
 *     If that cell is occupied we fall back to the rightmost free
 *     FCC neighbor of the bridge.
 */
void FlowBridgeCatoms3DCode::startup()
{
    console << "Module " << module->blockId
            << " at " << module->position << " ready.\n";

    // ── Election: run only once (first module to reach here) ──
    if (!mobileChosen) {
        mobileChosen = true;

        Catoms3DWorld *world   = Catoms3DWorld::getWorld();
        Lattice      *lattice = world->lattice;

        // Collect bridge-layer modules (z == 2).
        struct BridgeMod {
            bID            id;
            Cell3DPosition pos;
        };
        vector<BridgeMod> candidates;

        for (auto &[id, block] : world->getMap()) {
            if (block->position[2] == 2) {
                candidates.push_back({id, block->position});
            }
        }

        // Sort: smallest x, then smallest y, then smallest id.
        sort(candidates.begin(), candidates.end(),
             [](const BridgeMod &a, const BridgeMod &b) {
                 if (a.pos[0] != b.pos[0]) return a.pos[0] < b.pos[0];
                 if (a.pos[1] != b.pos[1]) return a.pos[1] < b.pos[1];
                 return a.id < b.id;
             });

        // Pick the first candidate whose removal keeps the
        // configuration connected (same BFS as isConnectedWithout).
        mobileId = 0;
        for (auto &c : candidates) {
            set<Cell3DPosition> occupied;
            for (auto &[id2, blk2] : world->getMap()) {
                if (blk2->position != c.pos) occupied.insert(blk2->position);
            }
            if (occupied.empty()) continue;
            set<Cell3DPosition>   visited;
            queue<Cell3DPosition> frontier;
            frontier.push(*occupied.begin());
            visited.insert(*occupied.begin());
            while (!frontier.empty()) {
                Cell3DPosition cur = frontier.front();
                frontier.pop();
                vector<Cell3DPosition> neighbors =
                    lattice->getActiveNeighborCells(cur);
                for (const Cell3DPosition &nb : neighbors) {
                    if (nb != c.pos && occupied.count(nb) && !visited.count(nb)) {
                        visited.insert(nb);
                        frontier.push(nb);
                    }
                }
            }
            if (visited.size() == occupied.size()) {
                mobileId = c.id;
                break;
            }
        }

        if (mobileId == 0) {
            console << "ERROR: no movable module found on the bridge.\n";
            return;
        }

        // ── Choose target: a free cell near the right end (high x). ──
        //    We try cells on the bridge layer (z=2) moving rightward
        //    from x = 9 downward, picking the first free one.
        targetPos = Cell3DPosition(9, 4, 2);
        // Verify it's free; if not, search nearby.
        if (!lattice->isFree(targetPos) || !lattice->isInGrid(targetPos)) {
            // Try a few alternatives near the right end.
            vector<Cell3DPosition> tryTargets = {
                {9, 3, 2}, {9, 4, 2}, {8, 4, 3}, {9, 3, 3},
                {8, 3, 3}, {9, 4, 3}, {7, 4, 3}, {7, 3, 3}
            };
            bool found = false;
            for (auto &t : tryTargets) {
                if (lattice->isInGrid(t) && lattice->isFree(t)) {
                    targetPos = t;
                    found = true;
                    break;
                }
            }
            if (!found) {
                console << "ERROR: cannot find a free target cell.\n";
                return;
            }
        }

        console << "=== MOBILE MODULE: " << mobileId
                << " | TARGET: " << targetPos << " ===\n";
    }

    // ── Only the elected mobile module initiates motion. ──
    if (module->blockId == mobileId) {
        module->setColor(YELLOW);
        planNextStep();
    }
}

//  Lifecycle: onMotionEnd — chain next step

void FlowBridgeCatoms3DCode::onMotionEnd()
{
    console << "Module " << module->blockId
            << " rotation complete → now at " << module->position << "\n";

    // Continue toward target.
    planNextStep();
}

