/**
 * @file flowBridgeCatoms3DCode.cpp
 * @brief Minimal implementation – just logs startup for each module.
 * @date 2026-02-10
 */
#include "flowBridgeCatoms3DCode.hpp"

FlowBridgeCatoms3DCode::FlowBridgeCatoms3DCode(Catoms3DBlock *host)
    : Catoms3DBlockCode(host), module(host) {
}

void FlowBridgeCatoms3DCode::startup() {
    console << "Module " << module->blockId << " ready\n";
}
