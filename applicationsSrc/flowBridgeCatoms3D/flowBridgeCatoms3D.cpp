/**
 * @file flowBridgeCatoms3D.cpp
 * @brief Main entry point for the FlowBridgeCatoms3D simulator application.
 * @date 2026-02-10
 */
#include <iostream>
#include "flowBridgeCatoms3DCode.hpp"

using namespace std;
using namespace Catoms3D;

int main(int argc, char **argv) {
    try {
        createSimulator(argc, argv, FlowBridgeCatoms3DCode::buildNewBlockCode);
        getSimulator()->printInfo();
        BaseSimulator::getWorld()->printInfo();
        deleteSimulator();
    } catch (std::exception const &e) {
        cerr << "Uncaught exception: " << e.what();
    }

    return 0;
}
