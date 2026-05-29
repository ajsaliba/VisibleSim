// =============================================================================
//  FlowGraphViz.cpp
//  ----------------
//  Implementation of the non-invasive 2D visualisation hooks.  Pure JSON
//  output; never mutates the FlowGraph or any application state.
// =============================================================================

#include "FlowGraphViz.h"

#include "base/simulator.h"

#include <algorithm>
#include <fstream>
#include <iostream>
#include <string>
#include <sys/stat.h>
#ifdef _WIN32
#include <direct.h>
#endif

namespace FlowGraphViz {

namespace {

// ---------------------------------------------------------------------------
//  Per-config output sub-folder management
// ---------------------------------------------------------------------------

std::string g_configStem = "default";

std::string stemOf(const std::string& path) {
    if (path.empty()) return std::string("default");
    std::string p = path;
    size_t s = p.find_last_of("/\\");
    if (s != std::string::npos) p = p.substr(s + 1);
    size_t d = p.find_last_of('.');
    if (d != std::string::npos) p = p.substr(0, d);
    if (p.empty()) p = "default";
    return p;
}

std::string ensureVizDir() {
#ifdef _WIN32
    _mkdir("viz_flowgraph");
#else
    mkdir("viz_flowgraph", 0755);
#endif
    std::string sub = "viz_flowgraph/" + g_configStem;
#ifdef _WIN32
    _mkdir(sub.c_str());
#else
    mkdir(sub.c_str(), 0755);
#endif
    return sub;
}

// ---------------------------------------------------------------------------
//  Stable key formatting
// ---------------------------------------------------------------------------

std::string nodeKey(const Node* n) {
    if (!n->specialKey.empty()) return n->specialKey;
    return std::to_string(n->x) + "," + std::to_string(n->y) + "," +
           std::to_string(n->z);
}

std::string cellKey(const Cell3DPosition& p) {
    return std::to_string((int)p.pt[0]) + "," +
           std::to_string((int)p.pt[1]) + "," +
           std::to_string((int)p.pt[2]);
}

std::string escapeJson(const std::string& s) {
    std::string r;
    r.reserve(s.size() + 2);
    for (char c : s) {
        if (c == '"' || c == '\\') { r.push_back('\\'); r.push_back(c); }
        else r.push_back(c);
    }
    return r;
}

// ---------------------------------------------------------------------------
//  Role classification
// ---------------------------------------------------------------------------

bool inVector(const std::vector<Cell3DPosition>* v,
              const Cell3DPosition& p) {
    if (!v) return false;
    for (const auto& q : *v) if (q == p) return true;
    return false;
}

std::string classify(const FlowGraph& g, const Node* n,
                     const ModuleSets& sets) {
    if (n == g.superSource) return "super_source";
    if (n == g.superSink)   return "super_sink";
    Cell3DPosition p((short)n->x, (short)n->y, (short)n->z);
    if (inVector(sets.sourceModules,    p)) return "source";
    if (inVector(sets.idleModules,      p)) return "idle";
    if (inVector(sets.targetCells,      p)) return "target";
    if (inVector(sets.bridgeFilled,     p)) return "bridge_filled";
    if (inVector(sets.bridgeEmpty,      p)) return "bridge_empty";
    if (inVector(sets.structuralBlocks, p)) return "structural";
    return "empty";
}

// ---------------------------------------------------------------------------
//  Average position of a special node's lattice neighbours, used to push
//  the super-source / super-sink off the lattice for the 2D layout.
// ---------------------------------------------------------------------------

struct AvgPos { double x, y, z; int count; };

AvgPos avgNeighbours(Node* sp) {
    AvgPos a{0, 0, 0, 0};
    if (!sp) return a;
    for (Edge* e : sp->edges) {
        Node* o = (e->to == sp) ? e->from : e->to;
        if (o->specialKey.empty()) {
            a.x += o->x; a.y += o->y; a.z += o->z; ++a.count;
        }
    }
    if (a.count > 0) { a.x /= a.count; a.y /= a.count; a.z /= a.count; }
    return a;
}

}  // anonymous namespace

// ---------------------------------------------------------------------------
//  Public API
// ---------------------------------------------------------------------------

void setConfigStem(const std::string& stem) {
    g_configStem = stemOf(stem);
}

void setConfigStemFromSimulator() {
    using BaseSimulator::Simulator;
    std::string cfg;
    if (Simulator::getSimulator()) {
        cfg = Simulator::getSimulator()->getCmdLine().getConfigFile();
        if (cfg.empty()) {
            TiXmlDocument* doc = Simulator::getSimulator()->getConfigDocument();
            if (doc) cfg = doc->ValueStr();
        }
    }
    if (cfg.empty()) cfg = Simulator::configFileName;
    setConfigStem(cfg);
}

void dump(const FlowGraph& g,
          const std::string& stage,
          const std::string& title,
          const ModuleSets& sets,
          const std::unordered_map<Edge*, int>* originalCap,
          const std::vector<std::vector<Cell3DPosition>>* augPaths,
          const std::vector<std::pair<Cell3DPosition, Cell3DPosition>>*
              minCutEdges) {

    std::string dir = ensureVizDir();
    std::string path = dir + "/stage_" + stage + ".json";
    std::ofstream out(path);
    if (!out) {
        std::cerr << "[viz] failed to open " << path << " for write\n";
        return;
    }

    // Place super-source / super-sink off to the side for readability.
    AvgPos ss = avgNeighbours(g.superSource);
    AvgPos ts = avgNeighbours(g.superSink);

    out << "{\n";
    out << "  \"stage\": \"" << escapeJson(stage) << "\",\n";
    out << "  \"title\": \"" << escapeJson(title) << "\",\n";

    // --- Stable node ordering: sort by key string ---
    std::vector<std::pair<std::string, Node*>> nodeList(g.nodes.begin(),
                                                        g.nodes.end());
    std::sort(nodeList.begin(), nodeList.end(),
              [](const std::pair<std::string, Node*>& a,
                 const std::pair<std::string, Node*>& b){
                  return a.first < b.first;
              });

    // --- nodes ---
    out << "  \"nodes\": [\n";
    bool firstNode = true;
    for (auto& kv : nodeList) {
        Node* n = kv.second;
        if (!firstNode) out << ",\n";
        firstNode = false;
        double x = n->x, y = n->y, z = n->z;
        if (n == g.superSource && ss.count > 0) {
            x = ss.x - 3.0; y = ss.y; z = ss.z;
        }
        if (n == g.superSink && ts.count > 0) {
            x = ts.x + 3.0; y = ts.y; z = ts.z;
        }
        out << "    {\"key\": \"" << escapeJson(nodeKey(n)) << "\""
            << ", \"x\": " << x << ", \"y\": " << y << ", \"z\": " << z
            << ", \"role\": \"" << classify(g, n, sets) << "\""
            << "}";
    }
    out << "\n  ],\n";

    // --- min-cut marker set ---
    std::set<std::pair<std::string, std::string>> mcSet;
    if (minCutEdges) {
        for (auto& [u, v] : *minCutEdges) {
            mcSet.insert({cellKey(u), cellKey(v)});
        }
    }

    // --- edges (forward arcs only) ---
    out << "  \"edges\": [\n";
    bool firstEdge = true;
    for (auto& kv : nodeList) {
        Node* u = kv.second;
        for (Edge* e : u->edges) {
            int origCap = -1;
            if (originalCap) {
                auto it = originalCap->find(e);
                if (it != originalCap->end()) origCap = it->second;
            }
            bool isForward =
                (originalCap ? (origCap > 0) : (e->capacity > 0));
            if (!isForward) continue;

            int displayCap = (origCap > 0 ? origCap : e->capacity);
            int flowVal    = (originalCap ? (origCap - e->capacity) : 0);
            bool saturated =
                (originalCap && origCap > 0 && e->capacity == 0);

            std::string fromK = nodeKey(u);
            std::string toK   = nodeKey(e->to);
            bool isMincut = mcSet.count({fromK, toK}) > 0;

            if (!firstEdge) out << ",\n";
            firstEdge = false;
            out << "    {\"from\": \""    << escapeJson(fromK)
                << "\", \"to\": \""        << escapeJson(toK)
                << "\", \"cap\": "         << displayCap
                << ", \"flow\": "          << flowVal
                << ", \"has_flow\": "      << (originalCap ? "true" : "false")
                << ", \"saturated\": "     << (saturated ? "true" : "false")
                << ", \"is_mincut\": "     << (isMincut ? "true" : "false")
                << "}";
        }
    }
    out << "\n  ],\n";

    // --- augmenting paths (sequence of lattice cell keys) ---
    out << "  \"paths\": [";
    if (augPaths) {
        bool firstPath = true;
        for (auto& p : *augPaths) {
            if (!firstPath) out << ",";
            firstPath = false;
            out << "\n    [";
            for (size_t i = 0; i < p.size(); ++i) {
                if (i) out << ", ";
                out << "\"" << cellKey(p[i]) << "\"";
            }
            out << "]";
        }
        if (!augPaths->empty()) out << "\n  ";
    }
    out << "]\n";

    out << "}\n";
    std::cout << "[viz] wrote " << path << "\n";
}

}  // namespace FlowGraphViz
