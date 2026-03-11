#include <iostream>
#include "robots/catoms3D/catoms3DSimulator.h"
#include "robots/catoms3D/catoms3DBlockCode.h"
#include "Catoms3DFlowBridgeCode.h"

using namespace std;
using namespace Catoms3D;

int main(int argc, char **argv) {
    cout << "\033[1;33m" << "Starting Catoms 3D Flow Bridge simulation ..." << "\033[0m" << endl;

    createSimulator(argc, argv, Catoms3DFlowBridgeCode::buildNewBlockCode);
    deleteSimulator();

    return 0;
}
