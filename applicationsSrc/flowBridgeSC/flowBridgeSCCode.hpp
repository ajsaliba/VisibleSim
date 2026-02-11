/**
 * @file   flowBridgeSCCode.hpp
 * @brief  Connectivity-preserving single-module motion for Sliding Cubes.
 *
 * This block code selects one mobile module near the beginning of the
 * bridge structure and moves it step-by-step toward a target cell near
 * the end, respecting:
 *   1. Sliding Cube motion constraints (translation / rotation rules)
 *   2. Global connectivity preservation (articulation-point check)
 *
 * @date   2026-02-11
 */
#ifndef FlowBridgeSCCode_H_
#define FlowBridgeSCCode_H_

#include <set>
#include <vector>
#include <queue>
#include <map>

#include "robots/slidingCubes/slidingCubesSimulator.h"
#include "robots/slidingCubes/slidingCubesBlockCode.h"
#include "robots/slidingCubes/slidingCubesBlock.h"
#include "robots/slidingCubes/slidingCubesWorld.h"
#include "grid/lattice.h"

using namespace SlidingCubes;

class FlowBridgeSCCode : public SlidingCubesBlockCode {
private:
    SlidingCubesBlock *module = nullptr;   ///< Pointer to the host module

    // ── Mobile-module state (shared across all instances) ──
    static bool             mobileChosen;    ///< True once the mobile is elected
    static bID              mobileId;        ///< Block-id of the mobile module
    static Cell3DPosition   targetPos;       ///< Goal cell (near end of bridge)

    // ── Connectivity helper ──

    /**
     * @brief Check whether removing @p pos from the occupied set keeps
     *        the configuration connected.
     *
     * BFS over the simple-cubic lattice grid, skipping @p pos.
     * Connected iff visited count == total modules − 1.
     *
     * @param pos  Cell whose removal is being tested.
     * @return true if remaining modules form one connected component.
     */
    bool isConnectedWithout(const Cell3DPosition &pos) const;

    // ── Motion planning helper ──

    /**
     * @brief Choose the next valid step toward targetPos.
     *
     * Enumerates all motions returned by SlidingCubesBlock::getAllMotions(),
     * filters by connectivity preservation, and picks the candidate
     * closest (Euclidean) to the target.
     *
     * @return true if a motion was scheduled, false if stuck or arrived.
     */
    bool planNextStep();

public:
    FlowBridgeSCCode(SlidingCubesBlock *host);
    ~FlowBridgeSCCode() override = default;

    // ── BlockCode lifecycle ──
    void startup() override;

    /**
     * @brief Called by the simulator when a motion finishes.
     *
     * Chains the next motion step, or reports arrival / stuck.
     */
    void onMotionEnd() override;

    // ── Factory ──
    static BlockCode *buildNewBlockCode(BuildingBlock *host) {
        return new FlowBridgeSCCode(static_cast<SlidingCubesBlock *>(host));
    }
};

#endif /* FlowBridgeSCCode_H_ */
