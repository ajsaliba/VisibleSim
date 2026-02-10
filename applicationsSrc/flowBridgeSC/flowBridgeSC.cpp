/**
 * @file flowBridgeSC.cpp
 * @brief Main entry point for the FlowBridgeSC simulator application.
 * @date 2026-02-10
 */
#include <iostream>
#include "flowBridgeSCCode.hpp"

using namespace std;
using namespace SlidingCubes;

int main(int argc, char **argv) {
    try {
        createSimulator(argc, argv, FlowBridgeSCCode::buildNewBlockCode);
        getSimulator()->printInfo();
        BaseSimulator::getWorld()->printInfo();
        deleteSimulator();
    } catch (std::exception const &e) {
        cerr << "Uncaught exception: " << e.what();
    }

    return 0;
}
