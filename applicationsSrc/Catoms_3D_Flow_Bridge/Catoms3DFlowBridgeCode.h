#ifndef CATOMS3DFLOWBRIDGECODE_H_
#define CATOMS3DFLOWBRIDGECODE_H_

#include <vector>
#include <string>
#include <unordered_map>
#include <set>
#include "robots/catoms3D/catoms3DSimulator.h"
#include "robots/catoms3D/catoms3DBlockCode.h"

using namespace std;
using namespace Catoms3D;

class Catoms3DFlowBridgeCode : public Catoms3DBlockCode {
private:
    Catoms3DBlock *catom;
    static vector<Cell3DPosition> targetPositions;
    static vector<Cell3DPosition> movingBlocks;
    static vector<Cell3DPosition> structuralBlocks;
public:
    // Constructor and destructor
    Catoms3DFlowBridgeCode(Catoms3DBlock *host);
    ~Catoms3DFlowBridgeCode() {};

    // Finds a path from start to goal using BFS and Catoms3D motion rules
    static vector<Cell3DPosition> findPath(const Cell3DPosition &start, const Cell3DPosition &goal);

    // Gets all reachable positions from a given position (one Catoms3D move)
    static bool getAllPossibleMotionsFromPosition(Cell3DPosition position, vector<Cell3DPosition> &reachablePositions);

    static void getPivotsForMotion(const Cell3DPosition& fromPos, const Cell3DPosition& toPos, set<Cell3DPosition>& pivots);

    // Static getters and setters for block and target positions, and configParsed flag
    static const vector<Cell3DPosition>& getTargetPositions();
    static const vector<Cell3DPosition>& getMovingBlocks();
    static const vector<Cell3DPosition>& getStructuralBlocks();
    static void setTargetPositions(const vector<Cell3DPosition>& positions);
    static void setMovingBlocks(const vector<Cell3DPosition>& positions);
    static void setStructuralBlocks(const vector<Cell3DPosition>& positions);

    // Override of startup function to parse configuration and extract moving blocks and target positions
    void startup() override;
    static void parseTargetPositions(TiXmlDocument *doc);
    static void parseMovingBlocks(TiXmlDocument *doc);
    static void parseStructuralBlocks(TiXmlDocument *doc);

    // Utility functions to print moving blocks and target positions
    static void printTargetList();
    static void printMovingBlockList();
    static void printStructuralBlocks();

    // Static function to build a new instance of Catoms3DFlowBridgeCode, used as block code builder for the simulation
    static BlockCode *buildNewBlockCode(BuildingBlock *host) {

        return new Catoms3DFlowBridgeCode(static_cast<Catoms3DBlock *>(host));

    }

};

#endif /* CATOMS3DFLOWBRIDGECODE_H_ */
