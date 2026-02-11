/**
 * @file   flowBridgeCatoms3DCode.hpp
 * @brief  Connectivity-preserving single-module motion for Catoms3D.
 *
 * This block code selects one mobile module near the beginning of the
 * bridge structure and moves it step-by-step toward a target cell near
 * the end, respecting:
 *   1. Catoms3D rotation constraints (HexaFace / OctaFace pivoting)
 *   2. Global connectivity preservation (articulation-point check)
 *
 * @date   2026-02-11
 */
#ifndef FlowBridgeCatoms3DCode_H_
#define FlowBridgeCatoms3DCode_H_

#include <set>
#include <vector>
#include <queue>
#include <map>

#include "robots/catoms3D/catoms3DSimulator.h"
#include "robots/catoms3D/catoms3DBlockCode.h"
#include "robots/catoms3D/catoms3DBlock.h"
#include "robots/catoms3D/catoms3DMotionEngine.h"
#include "robots/catoms3D/catoms3DRotationEvents.h"
#include "robots/catoms3D/catoms3DWorld.h"
#include "grid/lattice.h"

using namespace Catoms3D;

class FlowBridgeCatoms3DCode : public Catoms3DBlockCode {
private:
    Catoms3DBlock *module = nullptr;   ///< Pointer to the host Catoms3D module

    // ── Mobile-module state (only meaningful on the chosen mobile module) ──
    static bool             mobileChosen;    ///< True once the mobile module is picked
    static bID              mobileId;        ///< Block-id of the mobile module
    static Cell3DPosition   targetPos;       ///< Goal position (near end of bridge)

    // Connectivity helpers

    /**
     * @brief Check whether removing \p pos from the current occupied set
     *        keeps the configuration connected.
     *
     * Performs a BFS over the FCC lattice grid, skipping \p pos.
     * If the number of visited modules == total − 1, the set remains connected.
     *
     * @param pos  The cell whose removal is being tested.
     * @return true if the remaining modules form a single connected component.
     */
    bool isConnectedWithout(const Cell3DPosition &pos) const;

    // Motion planning helpers

    /**
     * @brief Choose the next valid step toward \p targetPos.
     *
     * Enumerates every position reachable in one Catoms3D rotation,
     * filters out those whose prior removal would disconnect the structure,
     * and picks the candidate closest (Manhattan / Euclidean) to the goal.
     *
     * @return true if a motion was scheduled, false if stuck.
     */
    bool planNextStep();

public:
    FlowBridgeCatoms3DCode(Catoms3DBlock *host);
    ~FlowBridgeCatoms3DCode() override = default;

    // BlockCode lifecycle
    void startup() override;

    /**
     * @brief Called by the simulator when a rotation finishes.
     *
     * Chains the next step of the motion, or reports arrival.
     */
    void onMotionEnd() override;

    // Factory
    static BlockCode *buildNewBlockCode(BuildingBlock *host) {
        return new FlowBridgeCatoms3DCode(static_cast<Catoms3DBlock *>(host));
    }
};

#endif /* FlowBridgeCatoms3DCode_H_ */
