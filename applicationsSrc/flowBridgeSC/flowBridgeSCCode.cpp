/**
 * @file flowBridgeSCCode.cpp
 * @brief Minimal implementation – just logs startup for each module.
 * @date 2026-02-10
 */
#include "flowBridgeSCCode.hpp"

FlowBridgeSCCode::FlowBridgeSCCode(SlidingCubesBlock *host)
    : SlidingCubesBlockCode(host), module(host) {
}

void FlowBridgeSCCode::startup() {
    console << "Module " << module->blockId << " ready\n";
}
