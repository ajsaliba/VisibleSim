/**
 * @file flowBridgeCatoms3DCode.hpp
 * @brief Minimal Catoms3D block code – renders the configuration only.
 * @date 2026-02-10
 */
#ifndef FlowBridgeCatoms3DCode_H_
#define FlowBridgeCatoms3DCode_H_

#include "robots/catoms3D/catoms3DSimulator.h"
#include "robots/catoms3D/catoms3DBlockCode.h"
#include "robots/catoms3D/catoms3DBlock.h"

using namespace Catoms3D;

class FlowBridgeCatoms3DCode : public Catoms3DBlockCode {
private:
    Catoms3DBlock *module = nullptr;

public:
    FlowBridgeCatoms3DCode(Catoms3DBlock *host);
    ~FlowBridgeCatoms3DCode() {};

    void startup() override;

    static BlockCode *buildNewBlockCode(BuildingBlock *host) {
        return (new FlowBridgeCatoms3DCode((Catoms3DBlock *)host));
    }
};

#endif /* FlowBridgeCatoms3DCode_H_ */
