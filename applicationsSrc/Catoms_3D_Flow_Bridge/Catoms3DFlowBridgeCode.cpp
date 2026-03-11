#include "Catoms3DFlowBridgeCode.h"
#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include "motion/teleportationEvents.h"

using namespace std;

// Static member definitions
vector<Cell3DPosition> Catoms3DFlowBridgeCode::blockPositions;
vector<Cell3DPosition> Catoms3DFlowBridgeCode::targetPositions;
vector<Cell3DPosition> Catoms3DFlowBridgeCode::movingBlocks;
bool Catoms3DFlowBridgeCode::configParsed = false;

Catoms3DFlowBridgeCode::Catoms3DFlowBridgeCode(Catoms3DBlock *host)
    : Catoms3DBlockCode(host), catom(host) {}


/*
 * Read the XML document and extracts the positions of all blocks, and store them in blockPositions
 * In addition, extracts the target list positions, and store them in targetPositions
*/
void Catoms3DFlowBridgeCode::startup() {

    if (!Catoms3DFlowBridgeCode::isConfigParsed()) {

        TiXmlDocument *doc = Simulator::getSimulator()->getConfigDocument();
        parseBlockPositions(doc);
        parseTargetPositions(doc);
        parseMovingBlocks(doc);
        setConfigParsed(true);
        printBlockList();
        printTargetList();
        printMovingBlockList();

    }

    return;

}

const vector<Cell3DPosition>& Catoms3DFlowBridgeCode::getBlockPositions() {

    return blockPositions;

}

const vector<Cell3DPosition>& Catoms3DFlowBridgeCode::getTargetPositions() {

    return targetPositions;

}

const vector<Cell3DPosition>& Catoms3DFlowBridgeCode::getMovingBlocks() {

    return movingBlocks;

}

bool Catoms3DFlowBridgeCode::isConfigParsed() {

    return configParsed;

}

void Catoms3DFlowBridgeCode::setBlockPositions(const vector<Cell3DPosition>& positions) {

    blockPositions = positions;

}

void Catoms3DFlowBridgeCode::setTargetPositions(const vector<Cell3DPosition>& positions) {

    targetPositions = positions;

}

void Catoms3DFlowBridgeCode::setMovingBlocks(const vector<Cell3DPosition>& positions) {

    movingBlocks = positions;

}

void Catoms3DFlowBridgeCode::setConfigParsed(bool parsed) {

    configParsed = parsed;

}

// Fetches the entire configuration document and transform them into Cell3DPosition
void Catoms3DFlowBridgeCode::parseBlockPositions(TiXmlDocument *doc) {

    TiXmlElement *root = doc->RootElement();
    TiXmlElement *blockList = root->FirstChildElement("blockList");

    if (blockList == nullptr) {

        cout << "No configuration found..." << endl;
        return;

    }

    TiXmlElement *block = blockList->FirstChildElement("block");
    vector<Cell3DPosition> positions;

    // Iterating over all block tags | block -> block
    while (block) {

        const char *posAttr = block->Attribute("position");
        if (posAttr) {

            int x, y, z;
            if (sscanf(posAttr, "%d,%d,%d", &x, &y, &z) == 3) {

                positions.push_back(Cell3DPosition(x, y, z));

            }

        }

        block = block->NextSiblingElement("block");

    }

    Catoms3DFlowBridgeCode::setBlockPositions(positions);
    return;

}

// Fetches all target positions and transform them into Cell3DPosition
void Catoms3DFlowBridgeCode::parseTargetPositions(TiXmlDocument *doc) {

    TiXmlElement *root = doc->RootElement();
    TiXmlElement *targetList = root->FirstChildElement("targetList");

    if (targetList == nullptr) {

        cout << "No target list found..." << endl;
        return;

    }

    TiXmlElement *target = targetList->FirstChildElement("target");

    if (target == nullptr) {

        cout << "Target list empty..." << endl;
        return;

    }

    TiXmlElement *block = target->FirstChildElement("cell");

    vector<Cell3DPosition> positions;
    // Iterating over all target tags | target -> target
    while (block) {

        const char *posAttr = block->Attribute("position");
        if (posAttr) {

            int x, y, z;
            if (sscanf(posAttr, "%d,%d,%d", &x, &y, &z) == 3) {

                positions.push_back(Cell3DPosition(x, y, z));

            }

        }

        block = block->NextSiblingElement("cell");

    }

    Catoms3DFlowBridgeCode::setTargetPositions(positions);
    return;

}

void Catoms3DFlowBridgeCode::parseMovingBlocks(TiXmlDocument *doc) {

    TiXmlElement *root = doc->RootElement();
    TiXmlElement *blockList = root->FirstChildElement("movingBlocks");

    if (blockList == nullptr) {

        cout << "No moving blocks found..." << endl;
        return;

    }

    TiXmlElement *block = blockList->FirstChildElement("block");
    vector<Cell3DPosition> positions;

    // Iterating over all block tags | block -> block
    while (block != nullptr) {

        const char *positionAttr = block->Attribute("position");
        if (positionAttr != nullptr) {

                int x, y, z;
                if (sscanf(positionAttr, "%d,%d,%d", &x, &y, &z) == 3) {

                    positions.push_back(Cell3DPosition(x, y, z));

                }

        }

        block = block->NextSiblingElement("block");

    }

    Catoms3DFlowBridgeCode::setMovingBlocks(positions);
    return;

}

void Catoms3DFlowBridgeCode::printBlockList() {

    for (Cell3DPosition blockListPos : Catoms3DFlowBridgeCode::getBlockPositions()) {

        cout << "Block position: " << blockListPos.to_string() << endl;

    }

    return;

}

void Catoms3DFlowBridgeCode::printTargetList() {

    for (Cell3DPosition targetListPos : Catoms3DFlowBridgeCode::getTargetPositions()) {

        cout << "Target position: " << targetListPos.to_string() << endl;

    }

    return;

}

void Catoms3DFlowBridgeCode::printMovingBlockList() {

    for (Cell3DPosition movingBlockListPos : Catoms3DFlowBridgeCode::getMovingBlocks()) {

        cout << "Moving block position: " << movingBlockListPos.to_string() << endl;

    }

    return;

}