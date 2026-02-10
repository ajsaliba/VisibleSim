/**
 * @file flowBridgeSCCode.hpp
 * @brief Minimal SlidingCubes block code – renders the configuration only.
 * @date 2026-02-10
 */
#ifndef FlowBridgeSCCode_H_
#define FlowBridgeSCCode_H_

#include "robots/slidingCubes/slidingCubesSimulator.h"
#include "robots/slidingCubes/slidingCubesBlockCode.h"

using namespace SlidingCubes;

class FlowBridgeSCCode : public SlidingCubesBlockCode {
private:
    SlidingCubesBlock *module = nullptr;

public:
    FlowBridgeSCCode(SlidingCubesBlock *host);
    ~FlowBridgeSCCode() {};

    void startup() override;

    static BlockCode *buildNewBlockCode(BuildingBlock *host) {
        return (new FlowBridgeSCCode((SlidingCubesBlock *)host));
    }
};

#endif /* FlowBridgeSCCode_H_ */
