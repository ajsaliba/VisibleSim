#ifndef CATOMS3DFLOWBRIDGECODE_H_
#define CATOMS3DFLOWBRIDGECODE_H_

#include "robots/catoms3D/catoms3DSimulator.h"
#include "robots/catoms3D/catoms3DBlockCode.h"

using namespace Catoms3D;
using namespace std;

class Catoms3DFlowBridgeCode : public Catoms3DBlockCode {
private:
    Catoms3DBlock *catom;
    static vector<Cell3DPosition> blockPositions;
    static vector<Cell3DPosition> targetPositions;
    static vector<Cell3DPosition> movingBlocks;
    static bool configParsed;
public:

    // Constructor and destructor
    Catoms3DFlowBridgeCode(Catoms3DBlock *host);
    ~Catoms3DFlowBridgeCode() {};

    // Static getters and setters for block and target positions, and configParsed flag
    static const vector<Cell3DPosition>& getBlockPositions();
    static const vector<Cell3DPosition>& getTargetPositions();
    static const vector<Cell3DPosition>& getMovingBlocks();
    static bool isConfigParsed();
    static void setConfigParsed(bool parsed);
    static void setBlockPositions(const vector<Cell3DPosition>& positions);
    static void setTargetPositions(const vector<Cell3DPosition>& positions);
    static void setMovingBlocks(const vector<Cell3DPosition>& positions);

    // Override of startup function to parse configuration and extract block and target positions
    void startup() override;
    static void parseBlockPositions(TiXmlDocument *doc);
    static void parseTargetPositions(TiXmlDocument *doc);
    static void parseMovingBlocks(TiXmlDocument *doc);

    // Utility functions to print block and target positions
    static void printBlockList();
    static void printTargetList();
    static void printMovingBlockList();

    // Check whether all target positions have been reached by blocks
    static bool checkTargetReached();

    // Teleport all moving blocks to their target positions using VisibleSim event system
    static void teleportAllMovingBlocksToTargetsWithEvent();

    // Static function to build a new instance of Catoms3DFlowBridgeCode, used as block code builder for the simulation
    static BlockCode *buildNewBlockCode(BuildingBlock *host) {
        return new Catoms3DFlowBridgeCode(static_cast<Catoms3DBlock *>(host));
    }
};

#endif /* CATOMS3DFLOWBRIDGECODE_H_ */
