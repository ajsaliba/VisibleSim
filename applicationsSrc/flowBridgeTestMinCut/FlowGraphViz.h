#ifndef FLOWGRAPH_VIZ_H_
#define FLOWGRAPH_VIZ_H_

// =============================================================================
//  FlowGraphViz
//  ------------
//  Non-invasive 2D visualisation hooks for the Catoms3D Flow Bridge
//  (combined min-cut-edge variant).
//
//  This compilation unit is PURELY observational: every function takes
//  `const FlowGraph&` (and other read-only inputs) and writes one JSON file
//  to ./viz_flowgraph/<config_stem>/stage_*.json.  Nothing in the algorithm
//  is altered.
//
//  The companion script visualize_flowgraph.py turns each JSON into a 2D PNG
//  with role-coloured nodes, capacity-labelled edges, a legend and a title.
//
//  Phases dumped:
//      1_initial               — initial main flow graph after build.
//      2_max_flow              — after Edmonds-Karp; flow values + aug. paths.
//      3_min_cut               — Picard-Queyranne min-cut arcs.
//      4_bridge_iter<N>_<stage>            — bridge sub-graph BEFORE EK.
//      5_bridge_iter<N>_<stage>_assignment — same sub-graph AFTER EK with
//                                            donor->bridge-cell assignment
//                                            paths overlaid.
//      6_post_bridge           — main flow graph rebuilt after bridge cells
//                                are filled; shows the new flow.
// =============================================================================

#include "FlowGraph.h"

#include <string>
#include <vector>
#include <utility>

namespace FlowGraphViz {

/// Pure-data classification of lattice cells; pointers are read-only.
struct ModuleSets {
    const std::vector<Cell3DPosition>* sourceModules    = nullptr;
    const std::vector<Cell3DPosition>* idleModules      = nullptr;
    const std::vector<Cell3DPosition>* targetCells      = nullptr;
    const std::vector<Cell3DPosition>* structuralBlocks = nullptr;
    const std::vector<Cell3DPosition>* bridgeEmpty      = nullptr;
    const std::vector<Cell3DPosition>* bridgeFilled     = nullptr;
};

/// Override the per-config output stem (basename without extension).  Call
/// once at startup so all dumps land in viz_flowgraph/<stem>/.
void setConfigStem(const std::string& stem);

/// Convenience: pull the config file name from the active simulator and use
/// its basename as the stem.  Falls back to "default" when unavailable.
void setConfigStemFromSimulator();

/// Dump the current FlowGraph + supplied auxiliary data to
/// viz_flowgraph/<stem>/stage_<stage>.json.  Any optional pointer may be
/// null; the consumer ignores absent keys.
void dump(const FlowGraph& g,
          const std::string& stage,
          const std::string& title,
          const ModuleSets& sets,
          const std::unordered_map<Edge*, int>* originalCap = nullptr,
          const std::vector<std::vector<Cell3DPosition>>* augPaths = nullptr,
          const std::vector<std::pair<Cell3DPosition, Cell3DPosition>>*
              minCutEdges = nullptr);

}  // namespace FlowGraphViz

#endif  /* FLOWGRAPH_VIZ_H_ */
