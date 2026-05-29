// =============================================================================
//  BridgeXmlParsing.cpp
//  --------------------
//  XML-driven population of the three lattice-state vectors used by the Flow
//  Bridge:
//
//      targetPositions     <- <targetList><target><cell .../></target>
//      movingBlocks        <- <movingBlocks><block .../>
//      structuralBlocks    <- <blockList><block .../>   (movingBlocks excluded)
//
//  Plus the matching getters/setters and pretty-printers.
// =============================================================================

#include "Catoms3DFlowBridgeCode.h"

#include <algorithm>
#include <iostream>

using namespace std;
using namespace Catoms3D;

// =============================================================================
//  Getters / setters
// =============================================================================

const vector<Cell3DPosition>& Catoms3DFlowBridgeCode::getTargetPositions()      { return targetPositions; }
const vector<Cell3DPosition>& Catoms3DFlowBridgeCode::getMovingBlocks()         { return movingBlocks; }
const vector<Cell3DPosition>& Catoms3DFlowBridgeCode::getStructuralBlocks()     { return structuralBlocks; }
const vector<Cell3DPosition>& Catoms3DFlowBridgeCode::getIdleStructuralBlocks() { return idleStructuralBlocks; }

void Catoms3DFlowBridgeCode::setTargetPositions(const vector<Cell3DPosition>& p)      { targetPositions = p; }
void Catoms3DFlowBridgeCode::setMovingBlocks(const vector<Cell3DPosition>& p)         { movingBlocks = p; }
void Catoms3DFlowBridgeCode::setStructuralBlocks(const vector<Cell3DPosition>& p)     { structuralBlocks = p; }
void Catoms3DFlowBridgeCode::setIdleStructuralBlocks(const vector<Cell3DPosition>& p) { idleStructuralBlocks = p; }

// =============================================================================
//  XML parsing
// =============================================================================

// -----------------------------------------------------------------------------
//  parseTargetPositions  —  <targetList><target><cell position="x,y,z"/>...
// -----------------------------------------------------------------------------
void Catoms3DFlowBridgeCode::parseTargetPositions(TiXmlDocument *doc) {
    TiXmlElement *root = doc->RootElement();
    TiXmlElement *targetList = root->FirstChildElement("targetList");
    if (!targetList) { cout << "No target list found.\n"; return; }
    TiXmlElement *target = targetList->FirstChildElement("target");
    if (!target) { cout << "Target list empty.\n"; return; }
    TiXmlElement *block = target->FirstChildElement("cell");
    vector<Cell3DPosition> positions;
    while (block) {
        const char *a = block->Attribute("position");
        if (a) { int x,y,z; if (sscanf(a,"%d,%d,%d",&x,&y,&z)==3) positions.push_back({(short)x,(short)y,(short)z}); }
        block = block->NextSiblingElement("cell");
    }
    setTargetPositions(positions);
}

// -----------------------------------------------------------------------------
//  parseMovingBlocks  —  <movingBlocks><block position="x,y,z"/>...
// -----------------------------------------------------------------------------
void Catoms3DFlowBridgeCode::parseMovingBlocks(TiXmlDocument *doc) {
    TiXmlElement *root = doc->RootElement();
    TiXmlElement *blockList = root->FirstChildElement("movingBlocks");
    if (!blockList) { cout << "No moving blocks found.\n"; return; }
    TiXmlElement *block = blockList->FirstChildElement("block");
    vector<Cell3DPosition> positions;
    while (block) {
        const char *a = block->Attribute("position");
        if (a) { int x,y,z; if (sscanf(a,"%d,%d,%d",&x,&y,&z)==3) positions.push_back({(short)x,(short)y,(short)z}); }
        block = block->NextSiblingElement("block");
    }
    setMovingBlocks(positions);
}

// -----------------------------------------------------------------------------
//  parseStructuralBlocks  —  <blockList><block position="x,y,z"/>...
//                            (cells already present in movingBlocks are skipped)
// -----------------------------------------------------------------------------
void Catoms3DFlowBridgeCode::parseStructuralBlocks(TiXmlDocument *doc) {
    TiXmlElement *root = doc->RootElement();
    TiXmlElement *blockList = root->FirstChildElement("blockList");
    if (!blockList) { cout << "No structural blocks found.\n"; return; }
    TiXmlElement *block = blockList->FirstChildElement("block");
    vector<Cell3DPosition> positions;
    while (block) {
        const char *a = block->Attribute("position");
        if (a) {
            int x,y,z;
            if (sscanf(a,"%d,%d,%d",&x,&y,&z)==3) {
                Cell3DPosition p((short)x,(short)y,(short)z);
                if (find(movingBlocks.begin(), movingBlocks.end(), p) == movingBlocks.end())
                    positions.push_back(p);
            }
        }
        block = block->NextSiblingElement("block");
    }
    setStructuralBlocks(positions);
}

// =============================================================================
//  Pretty-printers (diagnostic)
// =============================================================================

void Catoms3DFlowBridgeCode::printTargetList() {
    cout << "\n";
    for (const auto& p : getTargetPositions()) cout << "Target position: " << p.to_string() << "\n";
    cout << "\n";
}

void Catoms3DFlowBridgeCode::printMovingBlockList() {
    cout << "\n";
    for (const auto& p : getMovingBlocks()) cout << "Moving block: " << p.to_string() << "\n";
    cout << "\n";
}

void Catoms3DFlowBridgeCode::printStructuralBlocks() {
    cout << "\n";
    for (const auto& p : getStructuralBlocks()) cout << "Structural block: " << p.to_string() << "\n";
    cout << "\n";
}
