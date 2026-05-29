// =============================================================================
//  Catoms3DFlowBridge.cpp
//  ----------------------
//  Application entry point.  Spins up the Catoms3D simulator with our
//  block-code factory and tears it down on exit.  All the real work happens
//  in Catoms3DFlowBridgeCode::startup() (and the orchestration .cpp files
//  it transitively calls).
// =============================================================================

#include <iostream>

#include "robots/catoms3D/catoms3DSimulator.h"
#include "robots/catoms3D/catoms3DBlockCode.h"

#include "Catoms3DFlowBridgeCode.h"

using namespace std;
using namespace Catoms3D;

// =============================================================================
//  main
// =============================================================================

int main(int argc, char **argv) {

    cout << "\033[1;33m"
         << "Starting Catoms 3D Flow Bridge simulation ..."
         << "\033[0m" << endl;

    createSimulator(argc, argv, Catoms3DFlowBridgeCode::buildNewBlockCode);
    deleteSimulator();

    return 0;
}
