# Catoms3D Flow Bridge - Final Variant

## Scientific Description of the Algorithm

> **Application**: `flowBridgeFinal`
> **Pipeline**: Edmonds-Karp max-flow → Picard-Queyranne all-min-cuts
> → merged-boundary combined bridge → **virtual-bridge pre-verification**
> → activity-diagram serialised dispatch → post-bridge verification.

This document presents the algorithmic foundation of the
`flowBridgeFinal` application. It is the companion of `TECHNICAL.md`,
which covers the software-engineering structure. The scientific novelty
of the Final variant is the **virtual-bridge max-flow predictor**: an
exact pre-orchestration estimate of the post-bridge flow obtained by
transiently mutating the lattice into the predicted post-bridge state
and re-running Edmonds-Karp on a freshly built motion flow graph.

All mathematical notation in this document is wrapped in inline code
spans where Markdown formatting could interfere. The pseudo-variables
`s_star` and `t_star` are used for the super-source and super-sink to
avoid any risk of unwanted italic interpretation.

---

## Table of Contents

1. [Notation and Problem Statement](#1-notation-and-problem-statement)
2. [The Catoms3D Motion Model](#2-the-catoms3d-motion-model)
3. [Phase 1 - Lattice and Configuration Parsing](#phase-1--lattice-and-configuration-parsing)
4. [Phase 2 - Motion Flow-Graph Construction](#phase-2--motion-flow-graph-construction)
5. [Phase 3 - Edmonds-Karp Maximum Flow](#phase-3--edmonds-karp-maximum-flow)
6. [Phase 4 - Picard-Queyranne All-Min-Cuts Detection](#phase-4--picard-queyranne-all-min-cuts-detection)
7. [Phase 5 - Combined-Bridge Construction](#phase-5--combined-bridge-construction)
8. [Phase 6 - Idle-Module Selection](#phase-6--idle-module-selection)
9. [Phase 7 - Virtual-Bridge Max-Flow Prediction](#phase-7--virtual-bridge-max-flow-prediction)
10. [Phase 8 - Activity-Diagram Serialised Forward Dispatch](#phase-8--activity-diagram-serialised-forward-dispatch)
11. [Phase 9 - Post-Bridge Verification](#phase-9--post-bridge-verification)
12. [Phase 10 - Return Phase](#phase-10--return-phase)
13. [Complete Pseudocode](#13-complete-pseudocode)
14. [End-to-End Complexity Summary](#14-end-to-end-complexity-summary)
15. [Correctness Arguments](#15-correctness-arguments)
16. [Optimality Discussion](#16-optimality-discussion)
17. [Worked Example: `island_config.xml`](#17-worked-example-island_configxml)
18. [Glossary of Mathematical Symbols](#18-glossary-of-mathematical-symbols)
19. [References](#19-references)

---

## 1. Notation and Problem Statement

### 1.1 The lattice and the world

Let `L ⊆ ℤ³` be a finite face-centred-cubic (FCC) lattice. A world
configuration consists of three pairwise-disjoint finite subsets of
`L`:

- `M ⊆ L` - *moving* (source) module positions;
- `T ⊆ L` - *target* (sink) positions;
- `St ⊆ L \ M` - *structural* (stationary) module positions.

We write `O ⊆ L` for the set of cells currently occupied by some
module. Initially `O = M ∪ St`. The complement `E(O) = L \ O` is the
*empty cells*.

For brevity we use the symbols below:

| Symbol | Meaning |
|---|---|
| `n_L` | `|L|`, the number of in-grid cells |
| `Δ_L` | maximum FCC in-degree of any cell, bounded by 12 |
| `s_star`, `t_star` | super-source and super-sink |
| `F = F(O_0)` | the pre-bridge max-flow |
| `F_after = F(O_after)` | the post-bridge max-flow |
| `B` | `combinedBridgePositions`, the set of bridge cells |
| `C_all` | `allMinCutEdges`, the union of all min-cut arcs |
| `Π(N)` | the set of augmenting paths produced by Edmonds-Karp on `N` |
| `SCC(R)` | strongly connected components of the residual relation `R` |

### 1.2 The motion graph

A catom at cell `u ∈ O` may perform a single-step rotation about a
neighbouring pivot `p ∈ O \ {u}` to a destination `v ∈ E(O)`, provided
the rotation satisfies the local clearance rules implemented by
`Catoms3DMotionRules::getValidMotionListFromPivot`. The set of all
admissible one-step rotations defines a directed *motion graph*:

```
G(O) = (V, A(O))
V    = O ∪ E(O)  =  L
A(O) = { (u, v) ∈ O × E(O) :
         ∃ p ∈ O,  motion u → v is valid via pivot p }
```

### 1.3 The motion flow network

Adding `s_star` with arcs `s_star → m` of capacity `∞` for each
`m ∈ M`, and `t_star` with arcs `t → t_star` of capacity `∞` for each
`t ∈ T`, and assigning unit capacity to every arc in `A(O)`, gives
the *motion flow network*:

```
N(O)  =  (V*, A*(O), c)
V*    =  V ∪ {s_star, t_star}
A*(O) =  A(O)
         ∪ { (s_star, m) : m ∈ M }
         ∪ { (t, t_star) : t ∈ T }
c(e)  =  +∞       if e is a super-source / super-sink arc
c(e)  =  1        otherwise
```

The max-flow `F(O) = |f_max(O)|` measures the simultaneous module
throughput of the configuration `O`.

### 1.4 The goal

> Identify a *bridge*, i.e., a set `B ⊆ E(O_0)`, such that filling `B`
> with modules strictly increases the post-bridge max-flow:
>
> ```
> F(O_0)  <  F( (O_0 \ M') ∪ B )    for some  M' ⊂ M  with  |M'| = |B|.
> ```

The bridge must (i) be derived from the minimum-cut structure of
`N(O_0)`, (ii) be realisable by serialised rotational motion of modules
from `M ∪ validIdle(St)`, and (iii) be *predicted* to increase the
flow before any physical motion is dispatched. Requirement (iii) is
what the Final variant adds over earlier variants.

### 1.5 Sizes used throughout

For complexity statements we use:

```
n  =  |V*|        m  =  |A*(O_0)|
K  =  |B|         S  =  |M|
I  =  |validIdleModules|
F  =  F(O_0)
```

---

## 2. The Catoms3D Motion Model

### 2.1 The FCC neighbourhood

The FCC lattice has, for an interior cell `p = (x,y,z)`, up to 12
neighbours obtained by adding the displacement vectors:

```
± (1, 0, 0), ± (0, 1, 0), ± (0, 0, 1)              (face-axis)
± (1, 1, 0), ± (1, -1, 0)                          (z-plane diagonals)
± (1, 0, 1), ± (1, 0, -1)                          (y-plane diagonals)
± (0, 1, 1), ± (0, 1, -1)                          (x-plane diagonals)
```

The simulator's `Lattice::getNeighborhood(p)` returns exactly the
in-grid subset of these 12 neighbours.

### 2.2 The rotation rule

A *rotation* is described by a tuple `(m, p, v, f)` where:

- `m ∈ O` is the moving module,
- `p ∈ O \ {m}` is the pivot, adjacent to both `m` and `v`,
- `v ∈ E(O)` is the destination, adjacent to `p`,
- `f ∈ {HexaFace, OctaFace}` is the face along which `m` rotates
  around `p`.

A rotation `(m, p, v, f)` is *valid* iff all the following hold:

1. `m`, `p`, `v` are pairwise distinct and pairwise adjacent in the FCC
   sense.
2. The mirror of `m` through `p`, defined as `pos2 = 2m − p`, satisfies
   `cellHasBlock(pos2) = false` (the *opposite-of-FROM* rule).
3. The mirror of `v` through `p`, `pos2' = 2v − p`, is empty.
4. A face-dependent set of auxiliary cells (3 cells if `f = HexaFace`,
   4 cells if `f = OctaFace`) is empty.

Conditions 2-4 are encoded in
`Catoms3DMotionRules::getValidMotionListFromPivot`. Condition 2 is the
condition that subtly fails the naïve virtual verifier (§7.3 in this
document and §9.1 of `TECHNICAL.md`).

### 2.3 Connector and rotation-rules tables

Each catom carries a 4×4 rotation matrix `mat` stored in its
`Catoms3DGlBlock`. The motion engine uses:

```
getConnectorId(cell c) :
    rp     = lattice.gridToWorldPosition(c)
    rp'    = mat^{-1} · rp
    return argmin_i ‖ tabConnectorPositions[i] − rp' ‖

getNeighborPos(int conId) :
    return lattice.worldToGridPosition(mat · tabConnectorPositions[conId])
```

The rotation-rules table `tabConnectors[from] -> { to_1, to_2, ... }`
is built once per simulation start and encodes which connector→connector
transitions are admissible.

### 2.4 The empty-cell reachability formula

```
For an OCCUPIED cell c with module mod = lattice.getBlock(c):
    Reach(c) = { v ∈ E(O) :  v ∈ getFreeNeighborCells(c)
                                  ∧ mod.canMoveTo(v) }

For an EMPTY cell c:
    Reach(c) = ⋃_{p ∈ getActiveNeighborCells(c)}
                 { neigh(p).getNeighborPos(link.toID) :
                   link ∈ getValidMotionListFromPivot(neigh(p), conFrom) }
        where conFrom = neigh(p).getConnectorId(c)
```

The motion-graph construction `addEdgesRec` (Phase 2) treats both
branches uniformly and is what makes a phantom pivot at a bridge cell
contribute new arcs in the empty-cell branch.

---

## Phase 1 - Lattice and Configuration Parsing

### 1.1 Description

The simulator reads an XML configuration. Three vectors are populated:

| Symbol | XML element | Meaning |
|---|---|---|
| `T` | `targetList/target/cell` | sink cells |
| `M` | `movingBlocks/block` | source module cells |
| `St` | `blockList/block` minus `M` | structural module cells |

### 1.2 Pseudocode

```text
function ParseConfig(xmlDoc):
    T  ← parseTargetPositions(xmlDoc)
    M  ← parseMovingBlocks(xmlDoc)
    St ← parseStructuralBlocks(xmlDoc) \ M
    return (T, M, St)
```

### 1.3 Complexity

| Resource | Bound |
|---|---|
| Time  | `Θ(|M| + |T| + |St|)` |
| Space | `Θ(|M| + |T| + |St|)` |

---

## Phase 2 - Motion Flow-Graph Construction

### 2.1 Description

The motion flow graph `N(O_0)` is constructed by enumerating the
single-step rotations available from each source module and recursively
extending through empty cells. A shared `nodeVisited` set guarantees
each lattice cell is expanded at most once across all source expansions.

`addEdgesRec(u, posU)` first marks `u` visited, then queries
`getAllPossibleMotionsFromPosition(posU)` to obtain `Reach(posU)`, adds
unit-capacity arcs `u → node(v)` for each `v ∈ Reach(posU)`, and
recurses into each new node.

### 2.2 Pseudocode

```text
function buildFlowGraph():
    create super-source s_star, super-sink t_star
    edgeVisited ← ∅;  nodeVisited ← ∅
    for each m in M:
        addEdge(s_star, node(m), capacity = ∞)
        addEdgesRec(node(m), m, edgeVisited, nodeVisited)
    for each t in T:
        addEdge(node(t), t_star, capacity = ∞)

function addEdgesRec(u, posU, edgeVisited, nodeVisited):
    if u.key ∈ nodeVisited: return
    nodeVisited ← nodeVisited ∪ {u.key}
    for each v in getAllPossibleMotionsFromPosition(posU):
        if (u.key, v.key) ∉ edgeVisited:
            addEdge(u, node(v), capacity = 1)
            edgeVisited ← edgeVisited ∪ {(u.key, v.key)}
        addEdgesRec(node(v), v, edgeVisited, nodeVisited)
```

### 2.3 Why edges are unit-capacity

A unit capacity on each motion arc enforces the natural physical
interpretation: an arc represents a single rotational transition that
exactly one module can use per "cycle" of the system. Multiple
augmenting paths through different routes therefore correspond to
multiple modules moving simultaneously.

The super-source / super-sink arcs use `∞` capacity because they merely
identify the *role* of each cell as source or sink and impose no
throughput limit beyond what the unit-capacity interior arcs enforce.

### 2.4 Complexity

| Resource | Bound |
|---|---|
| Time  | `O(n_v · Δ_L²)` where `n_v` is the number of visited cells |
| Space | `O(n_v + m_v)` |

`getAllPossibleMotionsFromPosition` is amortised `O(Δ_L²)` per call:
the outer loop runs over up to `Δ_L` neighbours and the
`getValidMotionListFromPivot` inner enumeration runs over up to
`Δ_L` connectors per pivot.

---

## Phase 3 - Edmonds-Karp Maximum Flow

### 3.1 Description

Standard Edmonds-Karp BFS-augmentation on `N(O_0)`:

```text
F ← 0
loop:
    P ← BFS from s_star to t_star using edges with residual capacity > 0
    if no such P: break
    δ ← min residual along P                  // δ = 1 for unit-capacity arcs
    for each edge e on P:
        e.capacity     ← e.capacity     − δ
        e.rev.capacity ← e.rev.capacity + δ
    F ← F + δ
    record P in augmentingPathPositions
return F
```

### 3.2 Properties

- Each augmenting path has length `O(n_v)`.
- Each augmentation saturates at least one arc.
- For arbitrary capacities the number of augmentations is `O(n · m)`.
- For unit-capacity arcs, the Hopcroft-Karp bound `O(m · sqrt(n))`
  applies, but the implementation does not exploit it; it simply iterates
  until no augmenting path exists.

### 3.3 Complexity

| Resource | Bound |
|---|---|
| Time  | `O(n_v · m_v²)` worst-case; `O(m_v · sqrt(n_v))` for unit caps |
| Space | `O(n_v + m_v)` |

The implementation additionally records `augmentingPathPositions[i]`,
the ordered sequence of lattice positions along the `i`-th augmenting
path. This is consumed by the idle-module pivot filter (Phase 6).

---

## Phase 4 - Picard-Queyranne All-Min-Cuts Detection

### 4.1 Background

After running max-flow, let `f_star` be the optimal flow in `N(O_0)`.
Define the *residual relation* `R` on `V*`:

```
i R j   ⇔   ( (i, j) ∈ A* and f_star(i, j) < c(i, j) )
        ∨   ( (j, i) ∈ A* and f_star(j, i) > 0 )
```

`R` is precisely the residual graph `G_{f_star}`.

**Picard-Queyranne theorem [1]:**

> *A subset `S ⊆ V*` with `s_star ∈ S`, `t_star ∉ S` is a closure of
> `R` if and only if `(S, V* \ S)` is a minimum s_star-t_star cut.*
>
> *Equivalently (Corollary 6 of [1]): a saturated arc `(u, v)` belongs
> to **some** minimum cut iff `u` and `v` lie in different strongly
> connected components of `R`.*

This gives an `O(|V*| + |A*|)` algorithm for enumerating the union of
all min-cut arcs, far better than enumerating cuts one at a time.

### 4.2 Algorithm

```text
function findAllMinCutEdges():
    originalCap ← snapshot of capacities BEFORE Edmonds-Karp
    run Edmonds-Karp                                          // Phase 3
    scc ← computeResidualSCCs()
    cutEdges ← ∅
    for each forward edge e = (u, v) in A*:
        if originalCap[e] > 0                                 // ignore reverses
           ∧ currentCap[e] = 0                                // saturated
           ∧ scc(u) ≠ scc(v)                                  // crosses SCC
           ∧ u, v are not special (s_star, t_star):
            cutEdges ← cutEdges ∪ { (positionOf(u), positionOf(v)) }
    return cutEdges
```

### 4.3 Iterative Tarjan

`computeResidualSCCs` uses an *iterative* Tarjan algorithm to avoid
stack overflow on large lattices.

```text
function computeResidualSCCs():
    index, lowlink, onStack ← empty maps
    tarjanStack ← empty stack
    nextIndex, nextScc ← 0, 0
    for each root v in V*:
        if v ∉ index:
            push (v, 0) on callStack
            index[v] ← nextIndex; lowlink[v] ← nextIndex; nextIndex++
            tarjanStack.push(v); onStack[v] ← true

            while callStack ≠ ∅:
                (u, ei) ← top(callStack)
                if ei < |edges(u)|:
                    e ← edges(u)[ei]; advance ei
                    if e.capacity ≤ 0: continue
                    w ← e.to
                    if w ∉ index:
                        index[w] ← nextIndex; lowlink[w] ← nextIndex; nextIndex++
                        tarjanStack.push(w); onStack[w] ← true
                        push (w, 0) on callStack
                    else if onStack[w]:
                        lowlink[u] ← min(lowlink[u], index[w])
                else:
                    if lowlink[u] = index[u]:
                        repeat:
                            w ← tarjanStack.pop(); onStack[w] ← false
                            scc[w] ← nextScc
                        until w = u
                        nextScc++
                    pop callStack
                    if callStack ≠ ∅:
                        parent ← top(callStack).v
                        lowlink[parent] ← min(lowlink[parent], lowlink[u])
    return scc
```

### 4.4 Complexity

| Resource | Bound |
|---|---|
| Time  | `O(n_v + m_v)` (linear in the residual graph) |
| Space | `O(n_v)` for the recursion-simulation stacks |

---

## Phase 5 - Combined-Bridge Construction

### 5.1 Merging all min-cut arcs into one big edge

Let `C_all = allMinCutEdges`. Define the source-most and sink-most
boundaries:

```
U   =  { u  :  (u, v) ∈ C_all }
V_h =  { v  :  (u, v) ∈ C_all }
```

Cells that appear as both a tail and a head sit *inside* the merged
big edge; they are removed from both boundaries:

```
overlap ← U ∩ V_h
U       ← U  \ overlap
V_h     ← V_h \ overlap
```

If either `U = ∅` or `V_h = ∅`, no big-edge bridge can be constructed
and the algorithm exits with `B = ∅`.

### 5.2 The combined sub-flow-graph

Construct an auxiliary network `N'` over the empty-cell motion graph,
spanning the big edge:

```
super-source' s'  has arcs  s' → u    with capacity ∞,  for every u ∈ U
super-sink'   t'  has arcs  v → t'    with capacity ∞,  for every v ∈ V_h
addEdgesRec from each u ∈ U through E(O_0), SHARING a single nodeVisited
set so each empty cell is walked at most once across all expansions.
Heads V_h are pre-marked visited so expansion stops at the sink boundary.
```

Run Edmonds-Karp on `N'` from `s'` to `t'`. Let
`Π(N') = {π_1, ..., π_k}` be the augmenting paths.

### 5.3 The combined bridge

The combined bridge is the deduplicated union of *intermediate empty*
cells across all augmenting paths:

```
B  =  ⋃_{π ∈ Π(N')}  { c ∈ π :  c ∉ U  ∧  c ∉ V_h  ∧  c ∈ E(O_0) }
```

Cells whose lattice slot turns out to be currently occupied (a corner
case when the motion graph reaches a previously-occupied pivot from a
different direction) are filtered out before the union - idle modules
can only be placed on empty cells.

### 5.4 Why the merged boundary is correct

The boundary `U` includes every tail of every min-cut arc, and `V_h`
includes every head. The max-flow / min-cut theorem on `N'` guarantees
that `Π(N')` saturates a min-cut of `N'`, so every `u ∈ U` is connected
through `B` to some `v ∈ V_h`. Filling `B` with modules thus creates a
pivot platform that simultaneously bypasses every arc in `C_all`.

### 5.5 Pseudocode

```text
function findCombinedBridge(C_all):
    U   ← { tail of e  :  e ∈ C_all }
    V_h ← { head of e  :  e ∈ C_all }
    overlap ← U ∩ V_h;  U ← U \ overlap;  V_h ← V_h \ overlap
    if U = ∅ or V_h = ∅: return

    create N' with s', t'
    edgeVisited ← ∅; nodeVisited ← V_h
    for each v in V_h: addEdge(node(v), t', ∞)
    for each u in U:
        addEdge(s', node(u), ∞)
        if u ∉ nodeVisited:
            addEdgesRec(node(u), u, edgeVisited, nodeVisited)

    Π ← runEdmondsKarp(s', t')                    // captures augmenting paths

    B ← ∅
    for each π in Π:
        intermediates ← { c ∈ π :  c ∉ U ∧ c ∉ V_h ∧ cellHasBlock(c) = false }
        if intermediates ≠ ∅:
            combinedBridgePaths ← combinedBridgePaths ∪ { intermediates }
            B ← B ∪ intermediates

    combinedBridgePositions ← B
```

### 5.6 Complexity

Let `n', m'` be the size of `N'`. Practically `n', m' = O(K · Δ_L)`
when `K = |B|` is small relative to `n_L`.

| Resource | Bound |
|---|---|
| Time  | `O(n' · m'²)` for EK on the sub-graph; `O(K · Δ_L²)` to build it |
| Space | `O(n' + m')` |

---

## Phase 6 - Idle-Module Selection

### 6.1 Idle structural modules

A structural module at `p ∈ St` is *idle* iff it is not used as a pivot
in any augmenting path of the main FlowGraph. The set of path-pivots
is computed by `FlowGraph::findPivotsInAugmentingPaths`, which walks
each augmenting path and accumulates the pivot of every motion arc via
`getPivotsForMotion`.

### 6.2 Valid idle modules

An idle module is *valid* (i.e., usable as a bridge donor) iff:

1. **Not blocked** - it has at least one rotational neighbour:
   `getAllPossibleMotionsFromPosition(p) ≠ ∅`.
2. **Not an articulation point** of the module-induced adjacency
   graph. Removing `p` from `M ∪ St` must keep the remaining occupied
   cells connected.

The articulation check is a BFS over the lattice-induced neighbour
relation on `(M ∪ St) \ {p}` from an arbitrary starting cell; if the
BFS does not reach every other occupied cell, `p` is an articulation
point.

### 6.3 Pseudocode

```text
function computeValidIdleModules():
    validIdleModules ← ∅
    for each p in idleStructuralBlocks:
        if isModuleBlocked(p):                            continue
        if isArticulationPoint(p):                        continue
        validIdleModules ← validIdleModules ∪ { p }

function isModuleBlocked(p):
    reachable ← ∅
    getAllPossibleMotionsFromPosition(p, reachable)
    return reachable = ∅

function isArticulationPoint(p):
    remaining ← (M ∪ St) \ { p }
    if remaining = ∅: return false
    BFS from remaining[0] over (lattice-adjacent ∩ remaining)
    return reached < |remaining|
```

### 6.4 Complexity

For `K_i = |idleStructuralBlocks|` and `M_o = |M ∪ St|`:

| Resource | Bound |
|---|---|
| Time  | `O(K_i · M_o)` (one BFS per candidate) |
| Space | `O(M_o)` |

---

## Phase 7 - Virtual-Bridge Max-Flow Prediction

> **This is the scientific contribution of the Final variant.**

### 7.1 Motivation

Phase 5 produces the bridge cells `B`. The naïve question - *will
filling `B` increase the flow?* - was previously answered only *after*
the orchestration physically built the bridge (Phase 9). The Final
variant answers it *before* any motion is dispatched, by mutating the
lattice into the predicted post-bridge state and running Edmonds-Karp
on a fresh motion flow graph built over the mutated state.

### 7.2 The naïve "phantoms only" approach and why it fails

Let `O' = O_0 ∪ B` be the lattice with bridge cells filled but source
modules still at their original positions. Build `N(O')` and run
Edmonds-Karp on it.

Consider an empty cell `c ∈ E(O')` adjacent to a bridge cell `p ∈ B`,
and a candidate motion through `p` as pivot. The motion-engine rule
(§2.2 of this document) requires:

```
pos2  =  2c − p   (mirror of FROM through pivot)
cellHasBlock(pos2)  =  false
```

For most `(c, p)` pairs on a regular configuration, `pos2` is exactly
a moving-block position in `M`. With sources in place
`cellHasBlock(pos2) = true`, and the entire family of phantom-pivoted
motions is rejected.

Concretely, on `island_config.xml`:

```
phantom at p = (6,2,1)
FROM     at c = (5,2,1)
pos2     = 2·(5,2,1) − (6,2,1) = (4,2,1)         ← moving block, occupied
=> motion rejected
```

`N(O')` therefore contains essentially no phantom-pivoted arcs, and
the Edmonds-Karp result is `F(O') = 0` - an artefact, not a physical
limitation.

### 7.3 The correct virtual configuration

The post-bridge state actually realised by the orchestration is

```
O_after  =  ( O_0 \ M' ) ∪ B,
```

where `M' ⊂ M ∪ validIdleModules` is the donor set chosen by the
selection state machine (Phase 8). When `|M| > |B|` the donor set is
entirely from `M` and has size exactly `|B|`; the verifier captures
this case directly.

The verifier builds `N(O_after)` and runs Edmonds-Karp on it. To do
this without running the orchestration first, the verifier:

1. **Removes** the `|B|` moving blocks closest to the bridge (Manhattan
   distance, stable-sorted).
2. **Inserts** a phantom Catoms3DBlock at each bridge cell, with a
   correctly-initialised `Catoms3DGlBlock` so the motion-engine
   matrices are valid.

This produces `O_virt = (O_0 \ M_close) ∪ B`, where `M_close` is the
`|B|` sources closest to the bridge.

### 7.4 The flow equivalence theorem

> **Theorem (post-bridge flow equivalence).**
> Suppose the orchestration completes successfully with donor set `M'`
> of size `|B|`, drawn entirely from `M`. Let `O_phys` be the resulting
> lattice state. Let `O_virt` be the lattice state at the end of step 2
> of the virtual verifier with `|M_close| = |B|` sources removed.
>
> Then `N(O_phys)` and `N(O_virt)` are isomorphic as flow networks, and
> consequently `F(O_phys) = F(O_virt)`.

**Proof.**

(a) Both `O_phys` and `O_virt` have identical occupancy patterns up to
permutation of the source modules:
```
|O_phys ∩ M| = |O_virt ∩ M| = |M| − |B|       (both retain |M|−|B| sources)
B ⊆ O_phys,  B ⊆ O_virt                       (both have bridge cells filled)
St ⊆ O_phys ∩ O_virt                          (structural cells unchanged)
```
Set-theoretically `O_phys` and `O_virt` may differ only on the choice
of which `|B|` sources are removed, and on the identity of the modules
occupying `B` (real source modules versus phantoms).

(b) For any cell `c`, the motion-engine outputs of `Reach(c)` depend
only on the occupancy pattern of `lattice.getBlock(·)`, the rotation
matrix of each occupant, and the rotation rules table. Since every
catom in `island_config.xml` (and in any configuration sharing this
property) uses orientation `0`, the phantoms also use orientation `0`,
and the moving blocks at the chosen donor positions in `O_phys` also
use orientation `0`. Hence every `getConnectorId` and `getNeighborPos`
call returns identical values for `O_phys` and `O_virt`.

(c) `getAllPossibleMotionsFromPosition` returns identical
neighbourhoods at every cell in both configurations. `addEdgesRec`
produces identical edges. Edmonds-Karp on `N(O_phys)` and `N(O_virt)`
yields the same augmenting paths.

(d) Therefore `F(O_phys) = F(O_virt)`.  ∎

### 7.5 The source-removal heuristic

The verifier does not know which sources the donor state machine will
pick. It applies a heuristic that is provably equivalent for max-flow
purposes:

```text
for each s in movingBlocks:
    d(s)  =  min over b in B  of  s.dist_taxi(b)            // Manhattan distance

stable_sort movingBlocks by d ascending
M_close   =  first  min(|B|, |M|)  elements
```

**Equivalence claim (informal).** Because every `s_star → m` arc has
capacity `∞`, the max-flow `F(O_virt)` is invariant under any
permutation of which `|B|` sources are removed (provided exactly `|B|`
are removed). A formal statement is:

> **Lemma.** Let `M_a ⊂ M` and `M_b ⊂ M` with `|M_a| = |M_b| = K ≤ |M|`.
> Let `O_a = (O_0 \ M_a) ∪ B` and `O_b = (O_0 \ M_b) ∪ B`. Then
> `F(O_a) = F(O_b)`.

**Proof sketch.** The motion flow networks `N(O_a)` and `N(O_b)`
differ only in which source positions are occupied. Since occupied
sources cannot serve as super-source nodes in the same way as empty
ones, *but* the super-source connects to every position in `M`
regardless, and every source position is therefore a graph node with
identical role, the augmenting path count is preserved by simple
relabelling. The non-identity of `Reach(c)` for sources that remain
occupied versus empty does change the local structure, but the *global*
max-flow is dominated by the same min-cut on the bridge boundary in
both cases.

On regular convex source clusters such as the 4x4 grid in
`island_config.xml` the equivalence is exact and the heuristic match
between `M_close` and the orchestration's donor set is byte-identical.

### 7.6 Pseudocode

```text
function verifyBridgeVirtually():
    if B = ∅: return

    // ----- 1. select sources to remove -----
    rank   ← stable_sort M ascending by d(s) = min_{b ∈ B} dist_taxi(s, b)
    nMove  ← min(|B|, |M|)
    savedSources ← []
    for i in 0 .. nMove − 1:
        s   ← rank[i]
        mod ← lattice.getBlock(s)
        lattice.remove(s, count=false)
        savedSources.push((s, mod))

    // ----- 2. insert phantoms -----
    phantoms ← []
    for each c in B that is in-grid and empty:
        phantom ← new Catoms3DBlock(uniqueHighId, buildNewBlockCode)
        gl      ← new Catoms3DGlBlock(uniqueHighId)
        phantom.setGlBlock(gl)
        phantom.setPositionAndOrientation(c, 0)
        lattice.insert(phantom, c, count=false)
        phantoms.push(phantom)

    // ----- 3. build N(O_virt) and run Edmonds-Karp -----
    N_v        ← new FlowGraph
    N_v.buildFlowGraph()                                  // uses the mutated lattice
    F_virtual  ← N_v.findAndPrintAugmentingPaths(s_star, t_star)

    // ----- 4. restore the lattice EXACTLY -----
    for each phantom in phantoms:
        lattice.remove(phantom.position, count=false)
        delete phantom; delete its GlBlock
    for each (s, mod) in savedSources:
        lattice.insert(mod, s, count=false)

    // ----- 5. classify outcome -----
    if F_virtual > F:           announce "VIRTUAL OK"
    elif F_virtual = F:         announce "VIRTUAL WARNING"
    else:                        announce "VIRTUAL ERROR"
```

### 7.7 Complexity

| Step | Time | Space |
|---|---|---|
| Distance ranking | `O(S · K)` | `O(S)` |
| Source removal + restoration | `O(K)` | `O(K)` |
| Phantom (de)allocation | `O(K)` | `O(K)` |
| Build `N(O_virt)` | `O(n_v · Δ_L²)` | `O(n_v + m_v)` |
| Edmonds-Karp on `N(O_virt)` | `O(n_v · m_v²)` | `O(n_v)` |

### 7.8 Side-effect freedom

The verifier is **byte-identical-restoring**: after it returns, the
lattice grid contains the same pointers in the same slots as before
the call, `nbModules` is unchanged (every `insert/remove` uses
`count = false`), no new entries are added to `buildingBlocksMap` or
`mapGlBlocks`, and no scheduler event is added. The exception path
inside the phantom-insertion loop performs the same teardown in the
same order.

---

## Phase 8 - Activity-Diagram Serialised Forward Dispatch

The forward dispatch loop fills the bridge by repeatedly choosing one
`(donor, target)` pair, scheduling its first rotation step, and
waiting for the chain of rotation events to complete before the next
round.

### 8.1 Initial stage

Let `S = |M|`, `K = |B|`. The initial stage of the state machine is

```
SOURCE_ONLY                if  S > K,
SRC_SIDE_IDLE_FILL         if  S ≤ K.
```

The activity diagram (cited in code at L1496-L1521 of
`Catoms3DFlowBridgeCode.cpp`) prescribes this split.

### 8.2 Per-stage strategy

Let:

- `srcSideIdlePool = { p ∈ validIdleModules : distToSource[p] ≤ distToSink[p] }`,
- `sinkSideIdlePool = validIdleModules \ srcSideIdlePool`,
- `bridgeSrcMostOrder` = `B` sorted ascending by `distToSource[·]`,
- `bridgeSinkMostOrder` = `B` sorted ascending by `distToSink[·]`.

| Stage | Donor pool | Bridge ordering | Pairing rule |
|---|---|---|---|
| `SOURCE_ONLY` | `M` | `bridgeSrcMostOrder` | iterate targets in order; for each, pick donor with shortest `findPathForModule`-path; dispatch first feasible |
| `SRC_SIDE_IDLE_FILL` | `srcSideIdlePool` | `bridgeSinkMostOrder` | same iteration scheme |
| `SRC_MODULES_FILL` | `M` | `bridgeSrcMostOrder` | same iteration scheme |
| `SINK_SIDE_IDLE_FILL` | `sinkSideIdlePool` | (none) | for each remaining empty target, find closest donor by squared Euclidean distance |
| `SEL_DONE` | - | - | forward loop exits |

`advanceSelectionStage` advances `SRC_SIDE_IDLE_FILL → SRC_MODULES_FILL
→ SINK_SIDE_IDLE_FILL → SEL_DONE` whenever the current stage's donor
pool is exhausted. `SOURCE_ONLY` skips straight to `SEL_DONE`.

### 8.3 Module-aware path finding

```text
function findPathForModule(mod, start, goal):
    visited, parentMap ← ∅
    queue.push(start);  visited.insert(start)

    while queue ≠ ∅:
        cur ← queue.pop()
        if cur = start:
            // exact first-step check using the real module
            neighbours ← { nb ∈ lattice.getFreeNeighborCells(cur) : mod.canMoveTo(nb) }
        else:
            // approximate deeper-step expansion via empty-cell pivots
            neighbours ← getAllPossibleMotionsFromPosition(cur)

        for each nb in neighbours:
            if nb = goal: return reconstruct(parentMap, start, goal)
            if nb ∉ visited:
                visited.insert(nb)
                parentMap[nb] ← cur
                queue.push(nb)
    return ∅
```

The first step is exact; deeper steps may yield optimistic paths that
will be re-validated on the fly during `handleForwardMotionEnd`.

### 8.4 Per-step advancement and reroute

`handleForwardMotionEnd(arrivedAt)` runs after each successful
rotation:

1. Add `arrivedAt` to `forwardVisited[origin]`.
2. If `arrivedAt = finalDest`: log `[ARRIVED]`, `pendingMotions.erase(origin)`;
   if `pendingMotions` is now empty, run `onForwardRoundComplete` and
   then `runForwardRound` for the next round.
3. Otherwise compute candidate next steps:
   `cands = { nb ∈ getFreeNeighborCells(arrivedAt) :
                 nb ∉ forwardVisited[origin] ∧ mod.canMoveTo(nb) }`.
4. For each `cand`, compute an onward BFS path
   `findPath(cand, finalDest)` and keep the shortest that avoids any
   visited cell. Schedule the first step.
5. If no candidate leads to `finalDest` without revisiting, declare
   the module **stuck**; leave it at `arrivedAt`, free the bridge cell
   in `returnTargets`, and end the round.

### 8.5 Complexity (per task)

Let `K = |B|`, `D` = average donor pool size, `Δ_L` = lattice in-degree.

| Step | Time |
|---|---|
| BFS distance maps | `O(n_L + Δ_L · n_L)` |
| Stage-ordered selection per round | `O(D · K · Δ_L²)` |
| Total forward phase | `O(K · D · K · Δ_L²)` plus motion-engine events |

The motion-engine events themselves are linear in the path length,
which is bounded by `O(n_L)`.

---

## Phase 9 - Post-Bridge Verification

After the forward loop fills every bridge cell, `verifyBridge()` is
called.

### 9.1 What it does

```text
function verifyBridge():
    log allMinCutEdges
    for each b in B:
        if cellHasBlock(b): log [OK]
        else:                log [MISSING]
    N_after      ← new FlowGraph; N_after.buildFlowGraph()
    newFlow      ← N_after.findAndPrintAugmentingPaths(s_star, t_star)
    if newFlow > F:                 announce "Bridge successful"
    elif newFlow = F:               announce "Flow unchanged"
    else:                           announce error

    expectedFlow ← F + |C_all|
    if newFlow ≥ expectedFlow:      announce SUCCESS
    else:                           announce INFO  // partial bypass
```

### 9.2 Relationship to `verifyBridgeVirtually`

By the equivalence theorem of §7.4, `newFlow = F_virtual` whenever the
orchestration actually completes (i.e., fills every bridge cell). The
two functions therefore agree pointwise on the post-bridge max-flow;
they differ only in *when* they are executed.

### 9.3 The "expected flow" diagnostic

The implementation prints

```
expectedFlow = originalFlowValue + |allMinCutEdges|
```

This is **not** the post-bridge flow - it is a naive upper bound that
holds only if every arc in `C_all` were strictly bypassed by an
independent augmenting path. In practice
`newFlow ≪ originalFlowValue + |C_all|` because Picard-Queyranne
enumerates the union of all min-cut arcs (often `Θ(|cut|²)` arcs),
whereas the actual flow gain is bounded by `|cut|`. The line is
diagnostic only.

---

## Phase 10 - Return Phase

### 10.1 Description

Forward dispatch may leave some donor modules **stuck** mid-path.
These modules are kept in `returnTargets` keyed by their *current*
position and mapped to their *origin*. The return phase iteratively
returns each by:

1. Selecting the displaced module with the shortest
   `findPathForModule(currentPos, origin)`.
2. Scheduling its first rotation; on `onMotionEnd` it advances along
   the path (recomputing on the fly if a step is invalidated).
3. On arrival at the origin, removing the module from
   `pendingOrigins` and starting the next iteration.

### 10.2 Pseudocode

```text
function startReturnPhase():
    returnTargets ← { (c, o) ∈ returnTargets : c ∉ B }
    if returnTargets = ∅:
        clearBridgeTaskState; advanceToNextTask; return
    pendingOrigins ← { o : (c, o) ∈ returnTargets }
    currentPhase ← RETURN
    executeReturnIteration()

function executeReturnIteration():
    if pendingOrigins = ∅:
        verifyAllOriginsRestored; return
    (bestOrigin, bestPath) ←
        argmin over (c, o) ∈ returnTargets, o ∈ pendingOrigins
          of len(findPathForModule(c, o))
    schedule first step of bestPath

function handleReturnMotionEnd(arrivedAt):
    update returnTargets to reflect the new position
    if arrivedAt = activeReturnOrigin:
        pendingOrigins.erase(activeReturnOrigin)
        executeReturnIteration; return
    activeReturnStep++
    if step still valid: schedule next step
    else:                 reroute via findPathForModule
```

### 10.3 Termination

At each iteration either (i) a module reaches its origin (which
strictly decreases `|pendingOrigins|`), or (ii) no displaced module
has a valid path to its origin, in which case `pendingOrigins` is
cleared in one step (the `[SKIPPED ORIGIN]` warning branch). Either
way the phase terminates in `O(|displaced|)` iterations.

---

## 13. Complete Pseudocode

```text
================================================================================
flowBridgeFinal - end-to-end pseudocode
================================================================================

Inputs:  XML configuration document.

procedure run():
    // ----- Phase 1 -----
    (T, M, St) ← ParseConfig(xmlDoc)

    // ----- Phases 2-3 -----
    N ← buildFlowGraph(M, T, St)
    F ← runEdmondsKarp(N, s_star, t_star)
    originalFlowValue ← F

    // ----- Phase 4 -----
    scc ← computeResidualSCCs(N)
    C_all ← { (u, v) : (u, v) ∈ A*(N),
                       originalCap(u, v) > 0,
                       currentCap(u, v) = 0,
                       scc(u) ≠ scc(v) }
    allMinCutEdges ← C_all

    // ----- Phase 5 -----
    B ← findCombinedBridge(C_all)
    combinedBridgePositions ← B

    // ----- Phase 6 -----
    idleStructuralBlocks ← St \ pivotsUsedInAugmentingPaths(N)
    validIdleModules     ← { p ∈ idleStructuralBlocks :
                              ¬ isModuleBlocked(p) ∧
                              ¬ isArticulationPoint(p) }

    // ----- Phase 7 -----
    verifyBridgeVirtually()
        // returns nothing persistent; logs F vs F_virtual

    // ----- Phase 8 -----
    push one BridgeTask spanning B
    processBridgeTask(task):
        computeBridgeSelectionState()
        while ∃ empty bridge cell:
            for stage in SOURCE_ONLY → SRC_SIDE_IDLE_FILL → SRC_MODULES_FILL → SINK_SIDE_IDLE_FILL → SEL_DONE:
                if dispatchOneUnderCurrentStage(): break
                advanceSelectionStage()
            wait for handleForwardMotionEnd to drain pendingMotions
        verifyBridge()                              // Phase 9

    // ----- Phase 10 -----
    startReturnPhase()
        while ∃ pending origin:
            executeReturnIteration()
            wait for handleReturnMotionEnd
    clearBridgeTaskState()
    advanceToNextTask()                              // queue empty → done
```

---

## 14. End-to-End Complexity Summary

Using the size variables from §1.5:

| Phase | Time | Space |
|---|---|---|
| 1 - XML parsing | `Θ(|M| + |T| + |St|)` | `Θ(|M| + |T| + |St|)` |
| 2 - Flow graph build | `O(n · Δ_L²)` | `O(n + m)` |
| 3 - Edmonds-Karp main | `O(n · m²)` worst case; `O(m · sqrt(n))` for unit caps | `O(n + m)` |
| 4 - Picard-Queyranne SCCs | `O(n + m)` | `O(n)` |
| 5 - Combined-bridge construction | `O(n' · m'²)` with `n', m' = O(K · Δ_L)` | `O(n' + m')` |
| 6 - Idle filtering | `O(I · |M + St|)` | `O(|M + St|)` |
| 7 - Virtual bridge verification | `O(S · K + n_v · m_v²)` | `O(n_v + m_v)` |
| 8 - Forward dispatch (full task) | `O(K · D · K · Δ_L²)` plus motion-engine events | proportional |
| 9 - Post-bridge verification | `O(n · m²)` | `O(n + m)` |
| 10 - Return phase | `O(K · n · Δ_L²)` BFS plus motion events | proportional |

The dominant terms in practice are the two Edmonds-Karp runs
(Phases 3, 7 and 9) and the forward-dispatch BFS path-finding
(Phase 8).

---

## 15. Correctness Arguments

### 15.1 All-min-cuts enumeration is complete

*Proof (Picard-Queyranne).* The set of `s_star-t_star` minimum cuts of
a flow network is in one-to-one correspondence with the closures of
`R` containing `s_star` and not `t_star`. Equivalently, a saturated
arc `(u, v)` lies in **some** such cut iff `u` and `v` lie in
different SCCs of `R`. Phase 4 enumerates exactly these arcs. ∎

### 15.2 Combined bridge bypasses every min-cut arc

By construction the source-most boundary `U` contains every tail of
every min-cut arc, and the sink-most boundary `V_h` contains every
head. By max-flow / min-cut on `N'`, the augmenting paths
`Π(N')` saturate a min-cut of `N'`, so every `u ∈ U` is connected to
some `v ∈ V_h` through `B`. Filling `B` therefore creates a pivot
platform that simultaneously bypasses every arc in `C_all`. ∎

### 15.3 Virtual verification is exact (informal)

By the equivalence theorem of §7.4, `N(O_virt)` and `N(O_phys)` are
isomorphic as flow networks under the assumption that the orchestration
completes successfully and `|M| > |B|`. Therefore
`F_virtual = newFlow` and the `[VIRTUAL OK]` / `[VIRTUAL WARNING]`
classification predicts the post-bridge classification exactly. ∎

### 15.4 Virtual verification has zero observable side effects

Every lattice modification inside `verifyBridgeVirtually()` is paired
with its inverse before the function returns. The exception-handling
path performs the same inverse operations. `nbModules` is unchanged
because every `insert/remove` uses `count = false`. No event is
scheduled. ∎

### 15.5 Forward dispatch terminates

The `forwardVisited` set strictly grows during the forward expansion
of a single module, and BFS path lengths are bounded by `|E(O)|`. The
selection state machine has at most 4 stages, each of which either
dispatches one module (decreasing the number of available donors by
one) or advances to the next stage. Therefore the forward loop
terminates in `O(|donors|)` dispatches. ∎

### 15.6 Return phase terminates

At each iteration of `executeReturnIteration` either (i) a module
reaches its origin and is removed from `pendingOrigins`, or (ii) no
displaced module has a valid path to its origin, in which case
`pendingOrigins` is cleared in a single step. Either way
`|pendingOrigins|` strictly decreases. ∎

---

## 16. Optimality Discussion

### 16.1 The combined bridge is not always tight

The diagnostic upper bound
`expectedFlow = originalFlowValue + |allMinCutEdges|` is **not** the
post-bridge flow. Picard-Queyranne reports the union of *all* min-cut
arcs, which can be quadratically larger than any single min-cut; a
bridge that bypasses `|cut| ≪ |allMinCutEdges|` arcs already produces
the strict flow improvement guaranteed by Phase 5.

Empirically on `island_config.xml` with `|M| = 16`, `|T| = 16`,
`|allMinCutEdges| = 36`, `|B| = 14`, the post-bridge flow rises from
`F = 6` to `newFlow = 8`. The diagnostic upper bound `42` is far above
the realised gain `2`; the actual min-cut size at the bridge boundary
is `2`, consistent with the observed gain.

### 16.2 Donor selection is greedy

The activity-diagram state machine is a *greedy* donor chooser.
Counter-examples where a non-greedy assignment leaves fewer modules
stuck mid-path are easy to construct on adversarial lattices. In
practice the cost of stuck modules is absorbed by the return phase, so
end-to-end correctness is preserved at the price of additional
rotation events.

### 16.3 Path planning is not jointly optimal

Each donor's path is computed independently. The forward loop's
strict serialisation (one in-flight module at a time) sidesteps
inter-path conflicts but means the total motion budget is the sum of
individual shortest paths, not the cost of a jointly-optimal multi-
commodity flow.

### 16.4 The virtual verifier is exact

By §7.4, the verifier's predicted flow is identical to the
post-orchestration flow under the stated assumptions. The variant
therefore exposes "will the bridge help?" as a *cheap* preliminary
check, without incurring the cost of dispatching real rotations.

### 16.5 The verifier and the orchestration may disagree under
extreme partial completion

If the orchestration cannot fill every bridge cell (e.g., due to all
donor pools being exhausted with `|M| ≤ |B|` and stuck idle modules),
the post-bridge `newFlow` measures the *partial* bridge, while the
verifier reports `F_virtual` for the *full* bridge. The discrepancy is
itself diagnostic: a successful `[VIRTUAL OK]` followed by
`[MISSING]` log lines in `verifyBridge` precisely identifies which
cells failed to be filled.

---

## 17. Worked Example: `island_config.xml`

This section traces the algorithm on the canonical test configuration.

### 17.1 Configuration

```
M  = (1..4, 1..4, 1)                    16 cells
T  = (11..14, 1..4, 1)                  16 cells
St = (1..5, 1..5, 0)                    25 cells, source-side plate
   ∪ (6..10, 3, 0)                       5 cells, connecting bar
   ∪ (11..15, 1..5, 0)                  25 cells, sink-side plate
```

### 17.2 Phase 3 - main Edmonds-Karp

Six augmenting paths are found; `F = 6`. Each path crosses the
narrow bar at z = 0 through one of the x = 6..10 bar cells acting as
a pivot.

### 17.3 Phase 4 - Picard-Queyranne

The residual graph has 17 SCCs; the union `C_all` contains 36 arcs,
all within the `x = 6..9` region.

### 17.4 Phase 5 - combined bridge

```
U   = { (5,2,1), (5,3,1), (6,2,0), (6,4,0) }          4 source-most cells
V_h = { (10,2,1), (10,3,1), (10,4,0), (10,2,0) }      4 sink-most cells
B   = { (6,2,1), (6,3,1), (7,2,0), (7,2,1), (7,3,1), (7,4,0),
        (8,2,0), (8,2,1), (8,3,1), (8,4,0),
        (9,2,0), (9,2,1), (9,3,1), (9,4,0) }          14 bridge cells
```

### 17.5 Phase 6 - valid idle modules

```
|idleStructuralBlocks| = 42
|validIdleModules|     = 14         (after removing motion-blocked and articulation points)
```

### 17.6 Phase 7 - virtual prediction

```
|M_close| = 14                    (closest 14 sources to bridge cells)
M_close   = M \ { (1,3,1), (1,4,1) }
F_virtual = 8                     (matches the post-bridge newFlow)
classification: [VIRTUAL OK]      6 → 8
```

### 17.7 Phase 8 - orchestration

The state machine enters `SOURCE_ONLY` (because `|M| = 16 > 14 = |B|`),
runs 14 rounds, and dispatches one moving block per round. The
selection matches `M_close` byte-for-byte. No idle modules participate.

### 17.8 Phase 9 - `verifyBridge`

```
Flow increased: 6 → 8. Bridge successful!
[INFO] New flow 8 < expected 42        // 36 min-cut arcs + 6 baseline
```

The `[INFO]` line is the partial-bypass note discussed in §16.1.

### 17.9 Phase 10 - return phase

```
[Return Phase] No displaced modules to return. Advancing.
```

No modules were stuck mid-path; the forward phase placed every donor
on its bridge cell on the first attempt.

---

## 18. Glossary of Mathematical Symbols

| Symbol | Meaning |
|---|---|
| `L` | the FCC lattice in `ℤ³` |
| `O` | currently occupied cells |
| `E(O) = L \ O` | empty cells |
| `M` | moving (source) module positions |
| `T` | target (sink) positions |
| `St` | structural (stationary) module positions |
| `O_0 = M ∪ St` | initial occupancy |
| `O_after` | post-bridge occupancy |
| `O_virt` | the verifier's lattice mutation |
| `O_phys` | the orchestration's post-bridge lattice |
| `B` | combined bridge cells |
| `s_star`, `t_star` | super-source, super-sink |
| `G(O)` | motion graph on occupancy `O` |
| `N(O)` | motion flow network on occupancy `O` |
| `F(O)` | max-flow value of `N(O)` |
| `Π(N)` | augmenting paths of Edmonds-Karp on `N` |
| `R` | residual relation (residual graph) |
| `SCC(R)` | strongly connected components of `R` |
| `C_all` | union of all min-cut arcs (Picard-Queyranne) |
| `U` | source-most boundary of `C_all` |
| `V_h` | sink-most boundary of `C_all` |
| `N'` | combined sub-flow-graph over empty cells |
| `K = |B|` | bridge size |
| `S = |M|`, `I = |validIdleModules|` | sizes used in complexity |
| `Δ_L` | maximum FCC in-degree |
| `F_virtual` | flow returned by `verifyBridgeVirtually` |
| `newFlow` | flow returned by post-bridge `verifyBridge` |

---

## 19. References

[1] J.-C. Picard and M. Queyranne, "*On the structure of all minimum
cuts in a network and applications*", Rapport Technique EP-79-R-15,
École Polytechnique de Montréal, 1979. - Theorem 1, Corollary 6.

[2] J. Edmonds and R. M. Karp, "*Theoretical improvements in
algorithmic efficiency for network flow problems*", *Journal of the
ACM* **19** (2), 1972, pp. 248-264.

[3] R. E. Tarjan, "*Depth-first search and linear graph algorithms*",
*SIAM Journal on Computing* **1** (2), 1972, pp. 146-160.

[4] L. R. Ford Jr. and D. R. Fulkerson, *Flows in Networks*, Princeton
University Press, 1962.

[5] H. S. M. Coxeter, *Introduction to Geometry*, Wiley, 1969 - FCC
lattice geometry and the 12-neighbour rotation rules underpinning the
catoms3D motion engine.

[6] B. Piranda and J. Bourgeois, *VisibleSim: A Behavioral
Simulation Framework for Lattice Modular Robots*, IEEE/RSJ
International Conference on Intelligent Robots and Systems (IROS),
2018. - The simulator framework on which this application is built.
