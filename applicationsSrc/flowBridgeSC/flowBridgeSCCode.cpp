/**
 * @file   flowBridgeSCCode.cpp
 * @brief  Connectivity-preserving single-module motion for Sliding Cubes.
 *
 *  Sliding Cube Model Interpretation
 *
 *  Communication model
 * 
 *  Each Sliding Cube module sits on a simple-cubic (SC) lattice cell
 *  and has 6 connectors — one per axis-aligned face (±X, ±Y, ±Z).
 *  Communication is strictly local: a module can only exchange
 *  messages with modules physically adjacent on one of its 6 faces,
 *  through P2PNetworkInterface objects.
 *
 *  Motion model
 * 
 *  Two types of motion exist:
 *
 *  1. Translation (slide along a face)
 *     The module slides from its current cell to an axis-adjacent
 *     cell.  Requirements:
 *       • The pivot cell (neighbor in the pivot direction) must be
 *         occupied.
 *       • The cell beyond the pivot in the movement direction must
 *         also be occupied (sliding support).
 *       • The destination cell (neighbor in the movement direction)
 *         must be free.
 *
 *  2. Rotation (pivot around a neighbor)
 *     The module rotates around a pivot neighbor, sweeping through
 *     a larger volume.  Requirements:
 *       • The pivot cell must be occupied.
 *       • 6 cells along the rotation sweep must all be free
 *         (including the destination).
 *
 *  The simulator validates these constraints inside
 *  SlidingCubesMotionRules.  There are 24 translation rules and
 *  24 rotation rules (48 total).  The high-level API is
 *  SlidingCubesBlock::moveTo(dest) and getAllMotions().
 *
 *  Connectivity preservation
 * 
 *  Before every step we verify that *removing the mobile module
 *  from its current cell* does NOT disconnect the remaining modules.
 *  This is done via BFS on the SC lattice grid, skipping the
 *  mobile's current position.  Additionally we verify the candidate
 *  destination has at least one occupied SC neighbor (besides the
 *  mobile itself) so that the mobile re-joins the structure.
 *
 *  Algorithm overview
 *
 *  1. On startup every module logs itself.  Exactly one module is
 *     elected as the *mobile* — the first non-articulation-point
 *     module on the bridge layer (z == 2) near the left side.
 *
 *  2. The target cell is set near the right end of the bridge.
 *
 *  3. At each step the mobile module:
 *       a) Calls getAllMotions() to enumerate every position
 *          reachable in one valid Sliding Cube move.
 *       b) Checks connectivity preservation (isConnectedWithout).
 *       c) Filters: candidate must have at least one occupied
 *          SC neighbor (other than the mobile) at the destination.
 *       d) Picks the candidate closest (Euclidean) to the target.
 *       e) Calls moveTo() to schedule the motion.
 *
 *  4. On onMotionEnd(), step 3 is repeated until the target is
 *     reached or no valid step exists.
 *
 * @date   2026-02-11
 */

#include "flowBridgeSCCode.hpp"
#include <cmath>
#include <algorithm>
#include <iostream>
#include <limits>

using namespace std;

//  Static members

bool           FlowBridgeSCCode::mobileChosen = false;
bID            FlowBridgeSCCode::mobileId      = 0;
Cell3DPosition FlowBridgeSCCode::targetPos;

//  Construction

FlowBridgeSCCode::FlowBridgeSCCode(SlidingCubesBlock *host)
    : SlidingCubesBlockCode(host), module(host)
{
}

//  Connectivity check — BFS excluding one position

/**
 * Performs a BFS over occupied SC lattice cells, purposely *skipping*
 * the cell at @p pos.  Returns true iff all remaining modules form
 * one connected component.
 *
 * Uses Lattice::getActiveNeighborCells() which returns absolute
 * positions of occupied 6-neighbors that are in the grid.
 *
 * Complexity: O(N) where N = number of modules.
 */
bool FlowBridgeSCCode::isConnectedWithout(const Cell3DPosition &pos) const
{
    SlidingCubesWorld *world   = SlidingCubesWorld::getWorld();
    Lattice           *lattice = world->lattice;

    // Collect every occupied position except @p pos.
    set<Cell3DPosition> occupied;
    for (auto &[id, block] : world->getMap()) {
        if (block->position != pos) {
            occupied.insert(block->position);
        }
    }

    if (occupied.empty()) return true;   // trivially connected

    // BFS from an arbitrary remaining module.
    set<Cell3DPosition>   visited;
    queue<Cell3DPosition> frontier;
    frontier.push(*occupied.begin());
    visited.insert(*occupied.begin());

    while (!frontier.empty()) {
        Cell3DPosition cur = frontier.front();
        frontier.pop();

        // getActiveNeighborCells returns absolute positions of
        // occupied neighbors that are in-grid.
        vector<Cell3DPosition> neighbors =
            lattice->getActiveNeighborCells(cur);
        for (const Cell3DPosition &nb : neighbors) {
            // Skip the excluded cell — it is being "removed".
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
 * Enumerate valid motions for this module, keep only those that
 * preserve connectivity, and advance toward targetPos.
 */
bool FlowBridgeSCCode::planNextStep()
{
    // 1. Already at target?
    if (module->position == targetPos) {
        console << "Module " << module->blockId
                << " ARRIVED at target " << targetPos << "\n";
        module->setColor(GREEN);
        return false;
    }

    // 2. Connectivity check: can we leave the current cell?
    //    If the mobile is an articulation point, we cannot move.
    if (!isConnectedWithout(module->position)) {
        console << "Module " << module->blockId
                << " is an articulation point — CANNOT move.\n";
        module->setColor(RED);
        return false;
    }

    // 3. Enumerate every (position, orientation) reachable in one
    //    valid Sliding Cube motion (translation or rotation).
    //    getAllMotions() internally checks all 48 rules' validity.
    vector<pair<Cell3DPosition, uint8_t>> motions = module->getAllMotions();

    if (motions.empty()) {
        console << "Module " << module->blockId
                << " has no valid motions — stuck.\n";
        module->setColor(RED);
        return false;
    }

    // 4. Filter: keep only candidates where the destination has at
    //    least one occupied SC neighbor (other than the mobile itself)
    //    so that the mobile re-attaches to the structure.
    //
    //    Since we already verified the mobile is NOT an articulation
    //    point, removing it preserves connectivity.  Any landing with
    //    a neighbor in the remaining set guarantees the full config
    //    (remaining + mobile at new position) stays connected.
    Lattice *lattice = SlidingCubesWorld::getWorld()->lattice;

    struct Candidate {
        Cell3DPosition pos;
        uint8_t        orient;
        double         dist;   // Euclidean² to target
    };
    vector<Candidate> valid;

    for (auto &[cPos, cOrient] : motions) {
        // Check re-attachment: at least one SC neighbor of the
        // candidate cell (besides the mobile's current position)
        // must be occupied.
        vector<Cell3DPosition> relNeighbors =
            lattice->getRelativeConnectivity(cPos);
        bool hasNeighbor = false;
        for (const Cell3DPosition &off : relNeighbors) {
            Cell3DPosition nb = cPos + off;
            if (nb != module->position && lattice->cellHasBlock(nb)) {
                hasNeighbor = true;
                break;
            }
        }
        if (!hasNeighbor) continue;   // would dangle — skip

        // Euclidean² distance to goal.
        double dx = cPos[0] - targetPos[0];
        double dy = cPos[1] - targetPos[1];
        double dz = cPos[2] - targetPos[2];
        double d2 = dx * dx + dy * dy + dz * dz;
        valid.push_back({cPos, cOrient, d2});
    }

    if (valid.empty()) {
        console << "Module " << module->blockId
                << " has motions but none preserve connectivity.\n";
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
            << " -> " << best
            << "  (dist² to goal = " << valid.front().dist << ")\n";

    // 6. Schedule the motion.
    //    moveTo() internally validates Sliding Cube constraints
    //    (pivot, support, blocking cells) and schedules a
    //    TeleportationStartEvent.
    bool ok = module->moveTo(best);
    if (!ok) {
        // moveTo can fail if rule matching diverges from
        // getAllMotions (unlikely).  Try next candidates.
        for (size_t i = 1; i < valid.size(); ++i) {
            console << "  retry -> " << valid[i].pos << "\n";
            if (module->moveTo(valid[i].pos)) return true;
        }
        console << "Module " << module->blockId
                << " could not execute any valid motion.\n";
        module->setColor(RED);
        return false;
    }
    return true;
}

//  Lifecycle: startup

/**
 * Called once per module when the simulation starts.
 *
 * Election logic (deterministic, executed on every module):
 *   • Scan all modules on the bridge layer (z == 2).
 *   • Sort by smallest x, then smallest y, then smallest block-id.
 *   • Pick the first one whose removal does NOT disconnect the
 *     configuration.
 *   • Target: first free cell near the right end of the bridge.
 */
void FlowBridgeSCCode::startup()
{
    console << "Module " << module->blockId
            << " at " << module->position << " ready.\n";

    // ── Election: run only once ──
    if (!mobileChosen) {
        mobileChosen = true;

        SlidingCubesWorld *world   = SlidingCubesWorld::getWorld();
        Lattice           *lattice = world->lattice;

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
        targetPos = Cell3DPosition(9, 4, 2);
        if (!lattice->isFree(targetPos) || !lattice->isInGrid(targetPos)) {
            vector<Cell3DPosition> tryTargets = {
                {9, 3, 2}, {10, 3, 2}, {10, 4, 2},
                {8, 4, 3}, {9, 3, 3}, {8, 3, 3},
                {9, 4, 3}, {7, 4, 3}, {7, 3, 3}
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

void FlowBridgeSCCode::onMotionEnd()
{
    console << "Module " << module->blockId
            << " motion complete -> now at " << module->position << "\n";

    // Continue toward target.
    planNextStep();
}
