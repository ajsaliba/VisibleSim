# Catoms3D Flow Bridge - Combined Min-Cut Edge (Combined Test)

## Scientific Description of the Algorithm

> Application: `Catoms_3D_Flow_Bridge_Combined_Min_Cut_Edge_Combined_Test`
> Pipeline: Edmonds-Karp max-flow → Picard–Queyranne all-min-cuts →
> merged-boundary combined bridge → serialised one-module-at-a-time
> dispatch.

---

## Table of Contents

1. [Problem statement and notation](#1-problem-statement-and-notation)
2. [Phase 1 - Lattice and Configuration Parsing](#phase-1--lattice-and-configuration-parsing)
3. [Phase 2 - Motion Flow-Graph Construction](#phase-2--motion-flow-graph-construction)
4. [Phase 3 - Edmonds-Karp Maximum Flow](#phase-3--edmonds-karp-maximum-flow)
5. [Phase 4 - Picard–Queyranne All-Min-Cuts Detection](#phase-4--picardqueyranne-all-min-cuts-detection)
6. [Phase 5 - Combined-Bridge Construction (Merged Big Min-Cut Edge)](#phase-5--combined-bridge-construction-merged-big-min-cut-edge)
7. [Phase 6 - Idle-Module Selection](#phase-6--idle-module-selection)
8. [Phase 7 - Serialised Forward Dispatch](#phase-7--serialised-forward-dispatch)
9. [Phase 8 - Return Phase](#phase-8--return-phase)
10. [Complete Pseudocode of the Algorithm](#complete-pseudocode-of-the-algorithm)
11. [End-to-End Complexity Summary](#end-to-end-complexity-summary)
12. [Correctness Argument](#correctness-argument)
13. [References](#references)

---

## 1. Problem statement and notation

Let `L ⊂ ℤ³` be a finite FCC lattice and let

- `M ⊂ L` - set of *moving* module positions (sources),
- `T ⊂ L` - set of *target* positions (sinks),
- `S ⊂ L \ M` - set of *structural* (stationary) module positions.

A *catom* at cell `c` may rotate around an adjacent pivot to land
on a free neighbour cell. The lattice's motion graph is

```
G = (V, A)
V = M ∪ T ∪ S ∪ (empty cells reachable as motion targets)
A = { (u → v) : a module at u can move to v in one step }
```

Each arc carries unit capacity. Adding a super-source `s* → m`
for every `m ∈ M` and a super-sink `t → t*` for every `t ∈ T`,
both with capacity `∞`, yields the *motion flow network*
`N = (V*, A*, c)` with `V* = V ∪ {s*, t*}`.

The maximum flow `|f*|` measures how many modules can simultaneously
travel `M → T` per cycle. The minimum cut bounds it (Ford-Fulkerson),
and the cuts' arcs are the lattice's bottleneck transitions.

**Goal.** Identify *every* arc that belongs to *some* minimum cut
(not just one min-cut as elementary Edmonds-Karp gives), merge them
conceptually into one *big min-cut edge* whose source-most boundary
is the union of all min-cut tails and whose sink-most boundary is the
union of all min-cut heads, and build a single bridge of empty cells
spanning that big edge. Idle modules are then moved onto the bridge
*one at a time*, after which the residual max-flow strictly increases.

Throughout we write `n = |V*|`, `m = |A*|`, and `F = |f*|`.

---

## Phase 1 - Lattice and Configuration Parsing

### Description

The simulator reads an XML configuration file. Three vectors are
populated:

| Symbol | XML element | Meaning |
|---|---|---|
| `M` | `<movingBlocks><block .../>` | source module cells |
| `T` | `<targetList><target><cell .../>` | sink cells |
| `S` | `<blockList><block .../>` | structural module cells (cells in `M` are filtered out at parse time) |

### Pseudocode

```
parseTargetPositions(doc)       → T
parseMovingBlocks(doc)          → M
parseStructuralBlocks(doc)      → S = S_raw \ M
```

### Complexity

| Resource | Bound |
|---|---|
| Time  | `Θ(\|M\| + \|T\| + \|S\|)` |
| Space | `Θ(\|M\| + \|T\| + \|S\|)` |

---

## Phase 2 - Motion Flow-Graph Construction

### Description

`FlowGraph::buildFlowGraph()` materialises the directed graph
`N = (V*, A*, c)`:

1. Create special nodes `s*` and `t*`.
2. For each `m ∈ M`, add the arc `s* → m` with capacity `∞`.
3. From each `m`, recursively expand via
   `getAllPossibleMotionsFromPosition(p)`:
   - if `p` is *filled*, return free neighbours reachable via
     `mod->canMoveTo`;
   - if `p` is *empty*, iterate active neighbour pivots and read off
     the resulting motion destinations.
   For each new arc `(p → q)` of unit capacity, also create the
   reverse arc with capacity `0` (Ford-Fulkerson residual encoding).
4. For each `t ∈ T`, add the arc `t → t*` with capacity `∞`.

The recursion `addEdgesRec` uses a shared `nodeVisited` set so each
node is expanded at most once. The graph has `O(n)` nodes and
`O(m)` arcs (the FCC neighbourhood degree is bounded by a constant).

### Flow chart

```
   Parse XML ──► M, T, S
                  │
                  ▼
         create s*, t* nodes
                  │
        ┌─────────┴──────────┐
        ▼                    ▼
   for m ∈ M:           for t ∈ T:
   addEdge s* → m       addEdge t → t*
   addEdgesRec(m)
        │
        ▼
   G' = (V*, A*, c) ready for Edmonds-Karp
```

### Complexity

| Resource | Bound |
|---|---|
| Time  | `O(n · Δ)` where `Δ` is the bounded FCC-motion fan-out (`Δ ≤ 12`); effectively `O(n)`. |
| Space | `Θ(n + m)`. |

---

## Phase 3 - Edmonds-Karp Maximum Flow

### Description

`FlowGraph::findAndPrintAugmentingPaths(s*, t*)` repeatedly runs
breadth-first search in the residual graph, augmenting one unit
of flow per iteration along the shortest residual `s* → t*` path,
until no such path exists. Each augmentation saturates at least one
residual arc.

### Pseudocode

```
function EdmondsKarp(s*, t*):
    f ← 0
    repeat:
        parent ← BFS(s*, t*)                  // residual cap > 0
        if t* ∉ parent: return f
        Δ ← min residual capacity along the parent chain
        augment by Δ on the parent chain
        f ← f + Δ
```

In this application every internal arc has capacity 1, so each
augmentation increases `f` by exactly 1 and total flow is bounded
by the lattice fan-out at the sources.

### Complexity

| Resource | Bound |
|---|---|
| Time  | `O(n · m²)` worst case; `O(F · (n + m))` per augmentation, with `F` augmentations. |
| Space | `O(n + m)`. |

---

## Phase 4 - Picard–Queyranne All-Min-Cuts Detection

### Description (Picard & Queyranne 1979)

Given the max-flow `f*`, define the *residual relation* `R` on `V*`:

> `i R j  ⇔  ((i,j) ∈ A* ∧ f*_{ij} < c_{ij})  ∨  ((j,i) ∈ A* ∧ f*_{ji} > 0)`

Equivalently, `R` is the residual graph. The classical theorem
states:

> **Theorem 1 (Picard–Queyranne).** *A cut `(S, V\S)` separating
> `s*` from `t*` is a minimum cut iff `S` is a closure for `R`
> containing `s*` but not `t*`.*

> **Corollary 6.** *A saturated arc `(i, j)` belongs to some
> minimum cut iff `i` and `j` lie in different strongly connected
> components of the transitive closure `R̄`.*

We therefore (a) compute the SCC decomposition of the residual
graph using an iterative Tarjan algorithm, and (b) collect every
forward arc `(u, v)` with `originalCap(u,v) > 0`, `residualCap(u,v)
= 0` and `SCC(u) ≠ SCC(v)`. This set, `K`, is the *union of all
min-cut arcs across all minimum cuts*.

### Pseudocode

```
function PicardQueyranne(G_resid):
    SCC ← TarjanSCC(G_resid)                  // O(n + m)
    K ← ∅
    for each forward arc (u, v) with
        originalCap(u,v) > 0 ∧ residualCap(u,v) = 0:
        if SCC[u] ≠ SCC[v]:
            K ← K ∪ {(u, v)}
    return K
```

### Flow chart

```
   max-flow f*
        │
        ▼
   build residual graph
        │
        ▼
   Tarjan SCC (iterative)
        │
        ▼
   for each saturated forward arc (u,v):
       SCC[u] = SCC[v] ?
       │            │
      same         differ
       │            │
      skip      add (u,v) to K
        │
        ▼
        K = ⋃ min-cut arcs
```

### Complexity

| Resource | Bound |
|---|---|
| Time  | `O(n + m)` (Tarjan + linear arc scan). |
| Space | `O(n)` for SCC labels + Tarjan stack + index/lowlink. |

---

## Phase 5 - Combined-Bridge Construction (Merged Big Min-Cut Edge)

### Description

The set `K` contains every arc that participates in some minimum
cut. We do **not** treat each arc individually. Instead, we
*merge* every arc into one big min-cut edge `E*`:

- `U = { u : ∃ v, (u, v) ∈ K }`  (source-most boundary)
- `V_h = { v : ∃ u, (u, v) ∈ K }` (sink-most boundary)
- positions in `U ∩ V_h` are dropped from both sides (they lie
  inside `E*` and may serve as intermediates).

A *single* sub-flow-graph `G_b` is constructed:

```
   s*_b  ──∞──►  u      for every u ∈ U
                u ──► motion graph through empty cells
   v ──∞──►  t*_b       for every v ∈ V_h
```

Crucially, the heads `v ∈ V_h` are pre-marked visited in
`addEdgesRec`'s shared `nodeVisited` set so motion expansion stops
exactly at the sink boundary. Tails are expanded with the *same*
shared `nodeVisited`, so the entire merged motion graph is walked
**once** rather than once per min-cut arc.

Edmonds-Karp on `G_b` enumerates `f_b` augmenting paths through
empty space. For each path, its *intermediate* cells (those that
are not tails, not heads, and currently *empty* in the live
lattice) are collected. The *combined bridge* is

```
   B = ⋃_{p in EK augmenting paths} { intermediates(p) }
```

and is the set of lattice positions where idle modules must be
placed in order to bypass `E*` simultaneously.

### Pseudocode

```
function findCombinedBridge(K):
    U   ← { u : (u,_) ∈ K }
    V_h ← { v : (_,v) ∈ K }
    overlap ← U ∩ V_h
    U   ← U \ overlap
    V_h ← V_h \ overlap

    G_b ← new FlowGraph
    add s*_b, t*_b
    for v ∈ V_h:
        addEdge(v, t*_b, ∞)
        nodeVisited.insert(v)              // stop expansion at heads
    for u ∈ U:
        addEdge(s*_b, u, ∞)
        if u ∉ nodeVisited:
            addEdgesRec(u, nodeVisited)    // expand once across all tails

    EdmondsKarp(s*_b, t*_b)                // f_b augmenting paths

    B ← ∅
    for p in augmenting paths:
        for cell c in p:
            if c ∉ U  ∧  c ∉ V_h  ∧  lattice.empty(c):
                B ← B ∪ {c}
    return B
```

### Flow chart

```
        K (all min-cut arcs)
              │
              ▼
   split into U (tails) and V_h (heads)
              │
              ▼
       drop  U ∩ V_h  from both
              │
              ▼
   build single sub-flow-graph G_b
   • s*_b → every u ∈ U      (cap ∞)
   • every v ∈ V_h → t*_b    (cap ∞)
   • expand motion graph from tails;
     heads pre-marked visited
              │
              ▼
       Edmonds-Karp on G_b
              │
              ▼
   for each augmenting path p:
       keep cells of p that are
       not in U ∪ V_h and empty
              │
              ▼
        B = combined bridge
```

### Complexity

Let `n_b = |V(G_b)|` and `m_b = |A(G_b)|`. Since the shared
`nodeVisited` ensures every motion-graph cell is expanded at most
once, `n_b = O(n)` and `m_b = O(m)`.

| Resource | Bound |
|---|---|
| Time  | `O(n + m)` to build `G_b`, plus `O(f_b · (n + m))` for Edmonds-Karp where `f_b ≤ \|U\|`. |
| Space | `O(n + m)` (graph + EK working set). |

---

## Phase 6 - Idle-Module Selection

### Description

The donor pool is built in two filtering steps:

1. **Pivot exclusion.** Any structural block used as a rotation
   pivot in the main flow's augmenting paths is *not* available;
   moving it would invalidate those motions.
2. **Validity check.** Among the remaining candidates,
   `computeValidIdleModules` removes:
   - **blocked** modules - `getAllPossibleMotionsFromPosition` returns
     `∅`, meaning the module has no first move;
   - **articulation points** - modules whose removal would disconnect
     the structure (BFS over `(S ∪ M) \ {p}` shows the structure is
     no longer connected).

The output is `validIdleModules ⊂ S`.

### Pseudocode

```
P  ← pivots used in main flow's augmenting paths
I₀ ← S \ P
I  ← { p ∈ I₀ :
        ¬isModuleBlocked(p)  ∧
        ¬isArticulationPoint(p) }
```

### Complexity

Let `|I₀| = k`. The articulation check does one BFS per candidate,
each costing `O(|S| + |M|)`.

| Resource | Bound |
|---|---|
| Time  | `O(k · (\|S\| + \|M\|))`. |
| Space | `O(\|S\| + \|M\|)`. |

---

## Phase 7 - Serialised Forward Dispatch

### Description

The bridge orchestration state machine carries exactly **one**
bridge task whose `intermediatePath` is `B`. The forward phase is
strictly serialised: at any moment, at most one module is in
motion. The unit of work is a single round:

1. Collect `U_b ⊂ B` of currently empty bridge cells.
2. Collect `I_avail ⊂ I` of donor modules still physically present
   at their original idle position.
3. For every pair `(o, b) ∈ I_avail × U_b`, run a module-aware BFS
   `findPathForModule(mod, o, b)` and keep the *shortest* feasible
   path. Call this pair `(o*, b*)` and the path `π*`.
4. Dispatch the *single* module at `o*` along the first step of
   `π*`. The motion-end callback `handleForwardMotionEnd` then
   advances the module one rotation at a time along `π*`, until it
   arrives at `b*` (success) or is declared *stuck* (cannot
   continue without revisiting).
5. When the module finishes, `onForwardRoundComplete` re-enters
   step 1 of the next round.

Termination occurs when either `U_b = ∅` (bridge built), or
`I_avail = ∅`, or no `(o, b)` pair admits a feasible path.

### Pseudocode

```
function runForwardRound():
    U_b ← { b ∈ B : lattice.empty(b) }
    if U_b = ∅: verifyBridge(); startReturnPhase(); return

    I_avail ← { o ∈ I : lattice.filled(o) }
    if I_avail = ∅: verifyBridge(); startReturnPhase(); return

    best ← (⊥, ⊥, ⊥, +∞)            // (origin, target, path, length)
    for o ∈ I_avail:
        mod ← lattice.block(o)
        for b ∈ U_b:
            π ← findPathForModule(mod, o, b)
            if |π| ≥ 2 ∧ |π| < best.length:
                best ← (o, b, π, |π|)

    if best.path = ⊥: verifyBridge(); startReturnPhase(); return

    schedule first step of best.path
    // handleForwardMotionEnd drives the remaining rotations
    // and re-enters runForwardRound when this module is done.
```

### Flow chart

```
   ┌──────────────────────────────────────┐
   │           runForwardRound            │
   └──────────────────────────────────────┘
                    │
                    ▼
        U_b ← empty bridge cells
                    │
              U_b empty?
              ┌─────┴─────┐
            yes           no
              │            │
              ▼            ▼
       verifyBridge   I_avail ← idle donors at origin
       startReturn        │
                     I_avail empty?
                     ┌────┴─────┐
                   yes          no
                     │           │
                     ▼           ▼
                verifyBridge    pick best (o*, b*, π*)
                startReturn          │
                                π* exists?
                                ┌───┴───┐
                              yes      no
                                │       │
                                ▼       ▼
                         schedule  verifyBridge
                         step 1    startReturn
                                │
                                ▼
                        handleForwardMotionEnd
                        steps through π* one
                        rotation at a time;
                        on arrival/stuck:
                        onForwardRoundComplete()
                                │
                                ▼
                        runForwardRound (next round)
```

### Complexity per round

| Operation | Time | Space |
|---|---|---|
| Collect `U_b` and `I_avail` | `O(\|B\| + \|I\|)` | `O(\|B\| + \|I\|)` |
| Pair-wise BFS | `O(\|I\| · \|U_b\| · (n + m))` | `O(n)` per BFS |
| Dispatch | `O(1)` | `O(1)` |
| **Per round total** | `O(\|I\| · \|U_b\| · (n + m))` | `O(n + m)` |

A round either fills one bridge cell or removes one donor from
`I_avail`, so at most `|B| + |I|` rounds occur. Total forward
phase cost: `O((|B| + |I|) · |I| · |U_b| · (n + m))`. For the
typical case `|B|, |I|, |U_b| ≪ n`, this remains polynomial and
small.

---

## Phase 8 - Return Phase

### Description

The forward phase may leave some donor modules stuck mid-path
(not at a bridge cell and not at their origin). `startReturnPhase`
selects only modules whose current position is *not* a bridge cell
and returns them home one at a time:

1. Among all stuck modules, pick the one with the shortest BFS
   path back to its origin.
2. Schedule its first rotation step.
3. `handleReturnMotionEnd` steps the module forward, recomputing
   the path if a step turns out infeasible at runtime.
4. On origin arrival, remove the donor from `pendingOrigins`.
5. When `pendingOrigins = ∅`, call `verifyAllOriginsRestored`.

This is identical in structure to the forward serialisation: one
module per iteration.

### Pseudocode

```
function executeReturnIteration():
    if pendingOrigins = ∅: verifyAllOriginsRestored(); return
    best ← (⊥, ⊥, ⊥, +∞)
    for (curPos, origin) ∈ returnTargets:
        if origin ∉ pendingOrigins: continue
        π ← findPathForModule(lattice.block(curPos), curPos, origin)
        if |π| ≥ 1 ∧ |π| < best.length:
            best ← (curPos, origin, π, |π|)
    if best.path = ⊥: verifyAllOriginsRestored(); return
    activeReturn ← best
    schedule first rotation step
```

### Complexity

Same shape as the forward phase, bounded by `O(|I| · (n + m))`
per iteration and `O(|I|)` iterations.

---

## Complete Pseudocode of the Algorithm

This section gives a single, self-contained listing of the entire
algorithm. Subroutines are introduced in the order they are called
by the top-level driver. Each routine is annotated with:

- **Inputs / outputs.** Type-level signature.
- **Preconditions.** Invariants assumed at entry.
- **Postconditions.** Invariants guaranteed at exit.
- **Invariants.** Loop / recursion invariants that justify
  correctness.
- **Notes.** Implementation-relevant remarks (data structures,
  termination arguments, complexity hints).

Throughout, `lattice` is the live `Lattice` object,
`lattice.empty(p)` is true iff cell `p` carries no module,
`lattice.block(p)` returns the module at `p` (or `⊥`), and
`scheduler.schedule(e)` enqueues an event into the discrete-event
scheduler. `mod.canMoveTo(q)` is the motion-engine query that
determines whether the catom `mod` (currently at some cell `p`) can
perform one rotation step into the free cell `q`.

---

### 10.1 Top-level driver

```
ALGORITHM CatomsFlowBridge_CombinedMinCut_CombinedTest
INPUT      : XML configuration file
OUTPUT     : A combined bridge built in-place on the lattice and
             a verified increase of the network max-flow.
GLOBAL STATE (all initially empty / 0):
    targetPositions, movingBlocks, structuralBlocks
    idleStructuralBlocks, validIdleModules
    allMinCutEdges
    combinedBridgePositions, combinedBridgePaths
    pendingBridges (queue), currentBridgeTask
    currentPhase ← IDLE
    travelLog, returnTargets, pendingMotions, pendingArrival
    pendingOrigins, activeReturnModule, activeReturnPath
    originalFlowValue ← 0

procedure  startup():
    // ── Phase 1: parse the configuration ─────────────────────
    doc ← simulator.getConfigDocument()
    targetPositions   ← parseTargetPositions(doc)
    movingBlocks      ← parseMovingBlocks(doc)
    structuralBlocks  ← parseStructuralBlocks(doc) \ movingBlocks

    // ── Phase 2-5: build the analysis graph, then the bridge ─
    G ← new FlowGraph
    G.buildFlowGraph(movingBlocks, targetPositions)

    originalFlowValue ← G.startProcess(G.s*, G.t*, allMinCutEdges)
    // startProcess runs Phases 3, 4, and 5 in order:
    //   – Edmonds-Karp max-flow
    //   – Picard–Queyranne all-min-cut-arcs detection
    //   – findCombinedBridge  (writes combinedBridgePositions)

    // ── Phase 6: identify donor pool ────────────────────────
    idleStructuralBlocks ← G.identifyIdleStructuralBlocks()
    computeValidIdleModules()    //   → validIdleModules

    // ── Phase 7-8: orchestrate the build ────────────────────
    startBridgeOrchestration()
```

#### Postcondition

- `originalFlowValue` holds `|f*|` before any module is moved.
- `allMinCutEdges` = `K` = `⋃` over all min-cuts of their cut arcs.
- `combinedBridgePositions` = set of empty lattice cells to fill.
- `validIdleModules` ⊆ structural blocks that are safe to dispatch.
- The orchestration state machine starts at `BridgePhase.FORWARD`.

---

### 10.2 Motion-graph helpers

```
function  getAllPossibleMotionsFromPosition(p):
    INPUT  : cell p ∈ L
    OUTPUT : list of cells reachable in one rotation step from p
    PRE    : lattice initialised
    POST   : returned list contains every q such that
             ∃ catom mod with mod.canMoveTo(q) from p

    mod ← lattice.block(p)
    if mod ≠ ⊥:                              // p is filled
        return [ q ∈ lattice.freeNeighbours(p)
                   : mod.canMoveTo(q) ]
    out ← []
    for nb ∈ lattice.activeNeighbours(p):    // p is empty: pivot via nb
        neigh ← lattice.block(nb)
        for link ∈ motionRules.validMotionsFromPivot(neigh, p):
            q ← neigh.neighborPos( link.connectorTo )
            out.append(q)
    return out


function  findPath(start, goal):
    // Plain BFS over the cell-level motion graph.
    INPUT  : start, goal ∈ L
    OUTPUT : ordered list of cells [start, …, goal] or [] if none
    PRE    : –
    POST   : returned path is a sequence of one-step motions

    visited ← {start};  parent ← {};  Q ← queue([start])
    while Q not empty:
        cur ← Q.pop()
        if cur = goal: break
        for nb ∈ getAllPossibleMotionsFromPosition(cur):
            if nb ∉ visited:
                visited.add(nb);  parent[nb] ← cur;  Q.push(nb)
    if goal ∉ visited: return []
    return reconstructPath(start, goal, parent)


function  findPathForModule(mod, start, goal):
    // Module-aware BFS.  For the FIRST hop, use mod.canMoveTo
    // directly so the produced path is guaranteed feasible at
    // dispatch time.  Beyond the first hop, fall back to the
    // generic motion-graph BFS.
    INPUT  : mod = lattice.block(start),  start, goal ∈ L
    OUTPUT : ordered list of cells [start, …, goal] or []
    PRE    : mod is physically at  start
    POST   : returned list's first step is verified by canMoveTo
```

---

### 10.3 Phase 2 - `buildFlowGraph`

```
class FlowGraph:
    nodes              : map[ key → Node ]
    superSource s*     : Node
    superSink   t*     : Node
    augmentingPaths    : list of cell-sequences           // diagnostic

procedure  buildFlowGraph():
    PRE    : movingBlocks, targetPositions are populated
    POST   : nodes contains s*, t*, and every cell reachable
             from some moving block; edges encode unit-capacity
             rotation transitions; super-source/sink edges are ∞
    INV    : addEdgesRec recursion uses a shared nodeVisited set,
             so each cell is processed at most once

    s*  ← getOrCreateSpecial("SUPER_SOURCE")
    t*  ← getOrCreateSpecial("SUPER_SINK")

    edgeVisited ← {};  nodeVisited ← {}

    for m ∈ movingBlocks:
        n_m ← getOrCreateNode(m)
        if ⟨s*, n_m⟩ ∉ edgeVisited:
            addEdge(s*, n_m, ∞);   edgeVisited.add(⟨s*, n_m⟩)
        addEdgesRec(n_m, m, edgeVisited, nodeVisited)

    for t ∈ targetPositions:
        n_t ← getOrCreateNode(t)
        if ⟨n_t, t*⟩ ∉ edgeVisited:
            addEdge(n_t, t*, ∞);   edgeVisited.add(⟨n_t, t*⟩)


procedure  addEdgesRec(node, p, edgeVisited, nodeVisited):
    if node.key ∈ nodeVisited: return
    nodeVisited.add(node.key)
    for q ∈ getAllPossibleMotionsFromPosition(p):
        n_q ← getOrCreateNode(q)
        ek  ← ⟨node.key, n_q.key⟩
        if ek ∉ edgeVisited:
            addEdge(node, n_q, 1);    edgeVisited.add(ek)
        addEdgesRec(n_q, q, edgeVisited, nodeVisited)


procedure  addEdge(u, v, cap):
    // Ford-Fulkerson encoding: pair forward and reverse edges.
    e1 ← new Edge(u, v, cap);  e2 ← new Edge(v, u, 0)
    e1.rev ← e2;  e2.rev ← e1
    u.edges.append(e1);  v.edges.append(e2)
```

---

### 10.4 Phase 3 - Edmonds-Karp

```
function  backupCapacities():
    return { e ↦ e.capacity : e is any edge }


function  findAndPrintAugmentingPaths(s, t):
    // Standard Edmonds-Karp (BFS-based Ford-Fulkerson).
    INPUT  : source s, sink t in this FlowGraph
    OUTPUT : value of the maximum flow s → t
    INV    : after each iteration the flow value increases by ≥ 1
    POST   : every  edge.capacity  holds the residual capacity;
             augmentingPaths records the cell-level layout of
             each augmenting path discovered (special nodes
             stripped)

    flow ← 0
    repeat:
        parent ← BFS_residual(s, t)
        if t ∉ parent: return flow
        Δ ← min residual capacity along the parent chain      // = 1
        augment by Δ on every edge of the chain
        record the cell-sequence of the path
        flow ← flow + Δ


function  BFS_residual(s, t):
    parent ← {s ↦ ⊥};  Q ← queue([s])
    while Q not empty and t ∉ parent:
        u ← Q.pop()
        for e ∈ u.edges:
            if e.capacity > 0 and e.to ∉ parent:
                parent[e.to] ← e;  Q.push(e.to)
    return parent
```

---

### 10.5 Phase 4 - Picard–Queyranne

```
function  computeResidualSCCs():
    // Iterative Tarjan SCC on the residual graph.
    // Edges with current capacity ≤ 0 are excluded.
    OUTPUT : map Node → integer scc-id

    index, lowlink, onStack ← {} ; tarjanStack ← []
    sccOf ← {} ;  nextIndex ← 0 ;  nextScc ← 0
    callStack ← []      // (node, edge-iterator)

    for root in nodes.values():
        if root ∈ index: continue
        push (root, 0) onto callStack
        index[root] ← lowlink[root] ← nextIndex++
        tarjanStack.push(root);  onStack[root] ← true

        while callStack not empty:
            (v, ei) ← top(callStack)
            if ei < |v.edges|:
                e ← v.edges[ei];  top.ei ← ei + 1
                if e.capacity ≤ 0: continue
                w ← e.to
                if w ∉ index:
                    index[w] ← lowlink[w] ← nextIndex++
                    tarjanStack.push(w);  onStack[w] ← true
                    push (w, 0)
                else if onStack[w]:
                    lowlink[v] ← min(lowlink[v], index[w])
            else:
                if lowlink[v] = index[v]:                 // SCC root
                    repeat
                        w ← tarjanStack.pop();  onStack[w] ← false
                        sccOf[w] ← nextScc
                    until w = v
                    nextScc ← nextScc + 1
                pop callStack
                if callStack not empty:
                    parent ← top(callStack).v
                    lowlink[parent] ← min(lowlink[parent], lowlink[v])
    return sccOf


function  findAllMinCutEdgesPicardQueyranne(originalCap):
    INPUT  : originalCap = snapshot of edge capacities BEFORE EK
    OUTPUT : list of (Cell3DPosition, Cell3DPosition) pairs =
             union of min-cut arcs across all minimum cuts
    POST   : returned set contains, for every minimum cut C of the
             network, every arc of C  (Corollary 6 of Picard &
             Queyranne, 1979)

    sccOf ← computeResidualSCCs()
    K     ← []
    for each node u, each edge e ∈ u.edges:
        if originalCap[e] ≤ 0:           continue   // reverse edge
        if e.capacity ≠ 0:               continue   // not saturated
        v ← e.to
        if sccOf[u] = sccOf[v]:          continue   // same component
        if u or v is a special node:     continue
        K.append( ( cellOf(u), cellOf(v) ) )
    return K
```

---

### 10.6 Phase 5 - `findCombinedBridge`

```
procedure  findCombinedBridge(K):
    INPUT  : K = union of all min-cut arcs (Phase 4 output)
    OUTPUT : combinedBridgePositions (list of empty cells to fill)
             combinedBridgePaths     (per-augmenting-path layout)
    POST   : combinedBridgePositions ⊆ empty lattice cells, and
             once they are all occupied, the augmenting paths of
             the auxiliary sub-graph become physical detours
             around every  (u, v) ∈ K  simultaneously

    combinedBridgePositions ← [];  combinedBridgePaths ← []
    if K = ∅: return

    // Source-most boundary U  and sink-most boundary V_h.
    U   ← { u : (u, _) ∈ K }
    V_h ← { v : (_, v) ∈ K }
    overlap ← U ∩ V_h          // positions inside the big edge
    U   ← U \ overlap
    V_h ← V_h \ overlap
    if U = ∅ or V_h = ∅: return

    G_b ← new FlowGraph         // auxiliary sub-flow-graph
    s*_b ← G_b.getOrCreateSpecial("SUPER_SOURCE")
    t*_b ← G_b.getOrCreateSpecial("SUPER_SINK")
    edgeVisited ← {} ;  nodeVisited ← {}     // shared across tails!

    // Pre-wire every head and PIN it as visited so that
    // motion-graph expansion stops at the sink boundary.
    for v ∈ V_h:
        n_v ← G_b.getOrCreateNode(v)
        if ⟨n_v, t*_b⟩ ∉ edgeVisited:
            G_b.addEdge(n_v, t*_b, ∞);  edgeVisited.add(⟨n_v, t*_b⟩)
        nodeVisited.add( key(v) )

    // Wire every tail and expand the motion graph ONCE across
    // all tails (shared nodeVisited makes each empty cell be
    // visited at most one time in total).
    for u ∈ U:
        n_u ← G_b.getOrCreateNode(u)
        if ⟨s*_b, n_u⟩ ∉ edgeVisited:
            G_b.addEdge(s*_b, n_u, ∞);  edgeVisited.add(⟨s*_b, n_u⟩)
        if key(u) ∉ nodeVisited:
            G_b.addEdgesRec(n_u, u, edgeVisited, nodeVisited)

    G_b.findAndPrintAugmentingPaths(s*_b, t*_b)

    // Collect empty intermediates from every augmenting path.
    B ← ∅
    for path in G_b.augmentingPaths:
        inter ← []
        for cell c in path:
            if c ∈ U or c ∈ V_h:        continue
            if not lattice.empty(c):    continue       // occupied!
            inter.append(c)
        if inter ≠ []:
            combinedBridgePaths.append(inter)
            for c in inter: B.add(c)
    combinedBridgePositions ← list(B)
```

#### Invariants

1. After the call, every `c ∈ combinedBridgePositions` satisfies
   `lattice.empty(c) = true`.
2. No cell of `U ∪ V_h` is included in `combinedBridgePositions`.
3. The auxiliary graph is built in `O(n + m)` time because the
   shared `nodeVisited` set ensures each lattice cell is walked
   once across all tails.

---

### 10.7 Phase 6 - Idle-module filtering

```
function  isModuleBlocked(p):
    return getAllPossibleMotionsFromPosition(p) = []


function  isArticulationPoint(p):
    // Returns true iff removing the module at p disconnects
    // the structure  (M ∪ S) \ {p}.
    moduleSet ← (movingBlocks ∪ structuralBlocks) \ {p}
    if moduleSet = ∅: return false
    visited ← {pick any element of moduleSet}
    Q ← queue([first element])
    while Q not empty:
        cur ← Q.pop()
        for nb ∈ lattice.activeNeighbours(cur):
            if nb ≠ p and nb ∈ moduleSet and nb ∉ visited:
                visited.add(nb);  Q.push(nb)
    return  |visited| < |moduleSet|


procedure  computeValidIdleModules():
    validIdleModules ← []
    for p ∈ idleStructuralBlocks:
        if isModuleBlocked(p):       continue
        if isArticulationPoint(p):   continue
        validIdleModules.append(p)
```

---

### 10.8 Phase 7 - Orchestration: forward dispatch

```
struct  BridgeTask:
    minCutEdge      : (Cell3DPosition, Cell3DPosition)   // logging
    intermediatePath: list of Cell3DPosition             // = bridge cells
    edgeIndex       : int       // always 0 in this app
    pathIndex       : int       // always 0 in this app

procedure  startBridgeOrchestration():
    pendingBridges.clear()
    if combinedBridgePositions = []: return
    rep_u ← (allMinCutEdges = [] ? ⊥ : allMinCutEdges[0].first)
    rep_v ← (allMinCutEdges = [] ? ⊥ : allMinCutEdges[0].second)
    pendingBridges.push( BridgeTask{
        (rep_u, rep_v),                       // representative pair
        combinedBridgePositions,              // the bridge
        0, 0 } )
    task ← pendingBridges.pop()
    processBridgeTask(task)


procedure  processBridgeTask(task):
    currentBridgeTask ← task
    // Clear all per-task working state.
    travelLog        ← {} ;  roundAssignments ← []
    placedIntermediates ← ∅
    pendingMotions   ← ∅ ;  pendingArrival   ← {}
    forwardStepIndex ← {} ;  forwardVisited  ← {}
    returnTargets    ← {} ;  pendingOrigins  ← ∅
    activeReturnStep ← 0   ;  currentForwardRound ← 0
    currentPhase     ← FORWARD
    runForwardRound()


procedure  runForwardRound():
    INVARIANT (entering the round) :
        currentPhase = FORWARD
        pendingMotions = ∅      // no module in flight
    POSTCONDITION (after dispatch) :
        |pendingMotions| ≤ 1    // at most one in flight
        or the phase has transitioned to RETURN.

    U_b ← [ b ∈ currentBridgeTask.intermediatePath
              : lattice.empty(b) ]
    if U_b = ∅:                              // bridge built
        verifyBridge();  startReturnPhase();  return

    I_avail ← [ o ∈ validIdleModules : lattice.filled(o) ]
    if I_avail = ∅:
        verifyBridge();  startReturnPhase();  return

    // Greedy SHORTEST-PATH selection of one (origin, target).
    best ← ( ⊥, ⊥, [], +∞ )    // (origin, target, path, length)
    for o in I_avail:
        mod ← lattice.block(o)
        if mod = ⊥: continue
        for b in U_b:
            π ← findPathForModule(mod, o, b)
            if |π| ≥ 2 and |π| < best.length:
                best ← (o, b, π, |π|)

    if best.path = []:
        verifyBridge();  startReturnPhase();  return

    pendingMotions  ← ∅ ;  pendingArrival ← {}
    mod  ← lattice.block(best.origin)
    next ← best.path[1]
    if mod = ⊥ or not mod.canMoveTo(next):
        verifyBridge();  startReturnPhase();  return

    travelLog[best.origin]        ← best.path
    forwardStepIndex[best.origin] ← 0
    forwardVisited[best.origin]   ← {best.origin}
    returnTargets[best.target]    ← best.origin
    pendingMotions.add(best.origin)
    pendingArrival[next]          ← best.origin
    roundAssignments.append([best.origin])

    scheduler.schedule( Catoms3DRotationStartEvent(now, mod, next) )
    // handleForwardMotionEnd will be invoked after the rotation.


procedure  handleForwardMotionEnd(arrivedAt):
    if arrivedAt ∉ pendingArrival: return            // stale event
    origin ← pendingArrival.pop(arrivedAt)
    forwardVisited[origin].add(arrivedAt)

    if origin ∉ travelLog:
        pendingMotions.remove(origin)
        if pendingMotions = ∅: onForwardRoundComplete()
        return

    finalDest ← travelLog[origin].last

    // Update returnTargets so we know where the module currently is.
    for (curPos, orig) in returnTargets snapshot:
        if orig = origin and curPos ≠ arrivedAt:
            returnTargets.pop(curPos)
            returnTargets[arrivedAt] ← origin
            break

    if arrivedAt = finalDest:                        // success
        forwardVisited.pop(origin)
        pendingMotions.remove(origin)
        if pendingMotions = ∅: onForwardRoundComplete()
        return

    mod ← lattice.block(arrivedAt)
    if mod = ⊥:
        // module disappeared (shouldn't happen)
        cleanup(origin, finalDest)
        if pendingMotions = ∅: onForwardRoundComplete()
        return

    // Choose the next single step, avoiding visited cells.
    visited ← forwardVisited[origin]
    candidates ← [ q ∈ lattice.freeNeighbours(arrivedAt)
                    : mod.canMoveTo(q) ∧ q ∉ visited ]
    if candidates = []:                              // stuck
        // Leave the module here; it will be brought back by Phase 8.
        returnTargets.pop(finalDest, default=⊥)
        returnTargets[arrivedAt] ← origin
        travelLog.pop(origin) ; forwardStepIndex.pop(origin)
        forwardVisited.pop(origin)
        pendingMotions.remove(origin)
        if pendingMotions = ∅: onForwardRoundComplete()
        return

    // Pick the candidate whose onward BFS path to finalDest is
    // shortest AND avoids all visited cells.
    bestNext ← ⊥ ; bestPath ← [] ; bestLen ← +∞
    for cand in candidates:
        onward ← findPath(cand, finalDest)
        if onward = []: continue
        if onward intersects visited: continue
        len ← 1 + |onward|
        if len < bestLen:
            bestLen ← len
            bestNext ← cand
            bestPath ← [arrivedAt, cand] ++ onward[1:]

    if bestPath = []:
        // No clean onward path → treat as stuck.
        returnTargets.pop(finalDest, default=⊥)
        returnTargets[arrivedAt] ← origin
        travelLog.pop(origin) ; forwardStepIndex.pop(origin)
        forwardVisited.pop(origin)
        pendingMotions.remove(origin)
        if pendingMotions = ∅: onForwardRoundComplete()
        return

    travelLog[origin]        ← bestPath
    forwardStepIndex[origin] ← 0
    // Remove any stale pendingArrival entries for this module.
    drop any (k → origin) from pendingArrival
    pendingArrival[bestNext] ← origin
    scheduler.schedule( Catoms3DRotationStartEvent(now, mod, bestNext) )


procedure  onForwardRoundComplete():
    // Record every successful placement, then start next round.
    bridgeCells ← set(currentBridgeTask.intermediatePath)
    for (curPos, orig) in returnTargets:
        if curPos ∈ bridgeCells ∧ curPos ∉ placedIntermediates:
            placedIntermediates.add(curPos)
    currentForwardRound ← currentForwardRound + 1
    runForwardRound()                      // single-module dispatch
```

#### Termination of the forward phase

Each completed round either:

1. reduces `|U_b|` by 1 (the chosen module reached its bridge
   cell), or
2. removes a donor from `I_avail` (the chosen module is now
   stuck mid-path, no longer at its origin), or
3. exits via `verifyBridge → startReturnPhase`.

Both `|U_b|` and `|I_avail|` are non-negative integers that
decrease monotonically, so the loop terminates in at most
`|combinedBridgePositions| + |validIdleModules|` rounds.

---

### 10.9 Phase 8 - Return phase

```
procedure  startReturnPhase():
    bridgeCells ← set(currentBridgeTask.intermediatePath)
    keep ← {}
    for (curPos, orig) in returnTargets:
        if curPos ∉ bridgeCells: keep[curPos] ← orig
    returnTargets ← keep
    if returnTargets = {}:
        clearBridgeTaskState();  advanceToNextTask();  return
    pendingOrigins ← { orig for (_, orig) in returnTargets }
    currentPhase ← RETURN
    executeReturnIteration()


procedure  executeReturnIteration():
    INVARIANT : at most one module in motion at any instant

    if pendingOrigins = ∅:
        verifyAllOriginsRestored();  return

    // Pick the stuck module whose BFS path home is shortest.
    best ← (⊥, ⊥, [], +∞)        // (origin, modulePos, path, len)
    for (curPos, origin) in returnTargets:
        if origin ∉ pendingOrigins: continue
        cand ← lattice.block(curPos)
        if cand = ⊥: continue
        π ← findPathForModule(cand, curPos, origin)
        if π ≠ [] and |π| < best.length:
            best ← (origin, curPos, π, |π|)

    if best.path = []:
        pendingOrigins ← ∅
        verifyAllOriginsRestored();  return

    mod ← lattice.block(best.modulePos)
    next ← best.path[1]
    if mod = ⊥ or not mod.canMoveTo(next):
        pendingOrigins.remove(best.origin)
        returnTargets.pop(best.modulePos)
        executeReturnIteration();  return

    activeReturnModule ← best.modulePos
    activeReturnPath   ← best.path
    activeReturnStep   ← 1
    activeReturnOrigin ← best.origin
    scheduler.schedule( Catoms3DRotationStartEvent(now, mod, next) )


procedure  handleReturnMotionEnd(arrivedAt):
    if activeReturnModule in returnTargets:
        orig ← returnTargets.pop(activeReturnModule)
        returnTargets[arrivedAt] ← orig
    activeReturnModule ← arrivedAt

    if arrivedAt = activeReturnOrigin:
        pendingOrigins.remove(activeReturnOrigin)
        returnTargets.pop(arrivedAt, default=⊥)
        executeReturnIteration();  return

    activeReturnStep ← activeReturnStep + 1
    if activeReturnStep < |activeReturnPath|:
        next ← activeReturnPath[activeReturnStep]
        mod  ← lattice.block(arrivedAt)
        if mod = ⊥ or not mod.canMoveTo(next):
            // Re-plan from the current position.
            newπ ← findPathForModule(mod, arrivedAt,
                                     activeReturnOrigin)
            if |newπ| < 2:
                pendingOrigins.remove(activeReturnOrigin)
                returnTargets.pop(arrivedAt)
                executeReturnIteration();  return
            activeReturnPath ← newπ ; activeReturnStep ← 1
            next ← newπ[1]
        scheduler.schedule( Catoms3DRotationStartEvent(now, mod, next) )
    else:
        executeReturnIteration()


procedure  verifyAllOriginsRestored():
    for (origin, _) in travelLog:
        log whether lattice.filled(origin)
    clearBridgeTaskState();  advanceToNextTask()
```

---

### 10.10 Verification and end-of-run

```
procedure  verifyBridge():
    // Re-run the WHOLE max-flow analysis on the modified lattice.
    log min-cut edge representative of currentBridgeTask
    for p in currentBridgeTask.intermediatePath:
        log whether lattice.filled(p)
    H ← new FlowGraph;  H.buildFlowGraph()
    newFlow ← H.findAndPrintAugmentingPaths(H.s*, H.t*)
    delete H
    compare newFlow vs originalFlowValue


procedure  clearBridgeTaskState():
    currentPhase ← IDLE
    clear travelLog, roundAssignments, placedIntermediates,
          pendingMotions, pendingArrival, forwardStepIndex,
          forwardVisited, returnTargets, pendingOrigins,
          activeReturnPath
    activeReturnStep ← 0 ;  currentForwardRound ← 0


procedure  advanceToNextTask():
    if pendingBridges = ∅: halt
    else:
        task ← pendingBridges.pop()
        processBridgeTask(task)
```

Because `startBridgeOrchestration` pushes exactly one task,
`advanceToNextTask` terminates the run on its first invocation.

---

### 10.11 Global pseudocode invariants

The following invariants hold at every quiescent point (i.e.
whenever the scheduler returns control to the discrete-event loop
between rotations):

```
(I1)  currentPhase ∈ { IDLE, FORWARD, RETURN }
(I2)  |pendingMotions| ≤ 1                                   (forward)
(I3)  activeReturnModule is the unique in-flight return        (return)
(I4)  ∀ b ∈ combinedBridgePositions:
        once placed, b ∈ placedIntermediates and the lattice
        cell at b is filled and stays filled.
(I5)  ∀ o ∈ {origins ever dispatched}:
        the module at o either has reached a bridge cell or
        appears as a value in returnTargets.
(I6)  travelLog[o] is defined ⇔ o is in flight in the current
        forward round and headed to travelLog[o].last.
(I7)  Each forward round monotonically decreases either
        |unoccupied bridge cells|  or  |donors still at origin|.
(I8)  Each return iteration monotonically decreases
        |pendingOrigins|  or  removes one (curPos, origin) entry
        from returnTargets.
```

Invariants (I7) and (I8) jointly bound the orchestration runtime
by `O((|B| + |I|))` rounds + iterations, which establishes
termination.

---

## End-to-End Complexity Summary

Let `n, m, F` be the size of the motion network and its max-flow,
`B` the combined bridge, and `I` the valid idle pool.

| Phase | Time | Space |
|---|---|---|
| 1 - XML parsing | `Θ(\|M\| + \|T\| + \|S\|)` | `Θ(\|M\| + \|T\| + \|S\|)` |
| 2 - Flow graph build | `O(n)` | `Θ(n + m)` |
| 3 - Edmonds-Karp max-flow | `O(F · (n + m))` | `O(n + m)` |
| 4 - Picard–Queyranne SCC + scan | `O(n + m)` | `O(n)` |
| 5 - Combined-bridge sub-graph + EK | `O((1 + f_b) · (n + m))` | `O(n + m)` |
| 6 - Idle filtering | `O(\|S\|² + \|S\|·\|M\|)` | `O(n)` |
| 7 - Forward dispatch | `O((\|B\| + \|I\|) · \|I\| · \|B\| · (n + m))` | `O(n + m)` |
| 8 - Return | `O(\|I\|² · (n + m))` | `O(n + m)` |
| **Total (analysis)** | `O((F + f_b) · (n + m))` | `O(n + m)` |
| **Total (orchestration)** | `O(\|B\| · \|I\|² · (n + m))` | `O(n + m)` |

---

## Correctness Argument

### Min-cut union correctness

Phase 4 implements *exactly* Corollary 6 of Picard & Queyranne
(1979). The residual graph after Edmonds-Karp encodes the
relation `R`; Tarjan's SCC computes its strongly-connected
classes. The classical proof of Corollary 6 directly gives
soundness and completeness:

- *Soundness.* If `(u, v)` is saturated and `SCC(u) ≠ SCC(v)`,
  then take the closure `C` containing `u`'s SCC and not `v`'s
  SCC; the resulting cut is minimum (Theorem 1) and contains
  `(u, v)`.
- *Completeness.* If `(u, v) ∈ K_some_min_cut`, then `u` lies in
  the source-side closure and `v` in the complement; in the
  residual graph there is no `j → i` residual path returning, so
  `SCC(u) ≠ SCC(v)`.

### Combined-bridge correctness

Every cell `c ∈ B` is produced by an augmenting path of `G_b`
that does not reuse any node, and is constrained to:

- `c ∉ U` and `c ∉ V_h` (not a min-cut endpoint),
- `lattice.empty(c)` at the time of bridge construction.

After all `B` cells are filled, the augmenting paths of `G_b`
become physical detours around every `(u, v) ∈ K` simultaneously,
so the residual capacity of the *main* network strictly increases.
The `verifyBridge` phase re-runs Edmonds-Karp on the modified
lattice to confirm the new max-flow exceeds the original.

### Termination

Phase 7 is finite because each forward round either places a
module on `B` (reducing `|U_b|` by 1) or declares a donor stuck
(reducing `|I_avail|` by 1); both quantities are monotone
non-increasing and bounded by `|B| + |I|`. Phase 8 terminates by
the same monotone argument on `pendingOrigins`.

---

## References

1. **J.-C. Picard and M. Queyranne**, *On the Structure of All
   Minimum Cuts in a Network and Applications*, Rapport
   Technique EP-79-R-15, École Polytechnique de Montréal, 1979.
2. **L. R. Ford and D. R. Fulkerson**, *Flows in Networks*,
   Princeton Univ. Press, 1962.
3. **J. Edmonds and R. M. Karp**, *Theoretical Improvements in
   Algorithmic Efficiency for Network Flow Problems*, J. ACM
   19(2):248–264, 1972.
4. **R. E. Tarjan**, *Depth-First Search and Linear Graph
   Algorithms*, SIAM J. Comput. 1(2):146–160, 1972.
