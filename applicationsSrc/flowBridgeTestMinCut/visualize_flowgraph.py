#!/usr/bin/env python3
"""2D visualizer for the flowBridgeTestMinCut flow-graph phases.

Reads stage_*.json files emitted by FlowGraph::dumpFlowGraph2D and renders one
PNG per phase using matplotlib in 2D, in the style of the reference images:

  * white background, grid with integer ticks (x, y axes shown);
  * Source modules        -> red filled circle;
  * Target cells          -> green outlined circle;
  * Structure modules     -> black outlined circle;
  * Empty/virtual cells   -> light blue;
  * Unused / generic      -> hollow gray;
  * Bridge cells          -> orange filled with "B";
  * Bridge candidates     -> light-blue filled with blue outline;
  * Super-source          -> yellow filled;
  * Super-sink            -> purple filled;
  * Edges                 -> light-gray arrows, capacity labelled at midpoint;
  * Flow edges            -> bold red arrows (saturated / on aug. path);
  * Min-cut arcs          -> bold red arrows.

Phases emitted by the C++ side (per-config directory):
    stage_1_initial.json
        Initial main flow graph.
    stage_2_max_flow.json
        Main graph after Edmonds-Karp; augmenting paths overlaid.
    stage_3_min_cut.json
        Main graph with Picard-Queyranne min-cut arcs in bold red.
    stage_4_bridge_iter<N>_<stageLabel>.json
        Bridge-construction flow graph for iteration N.
    stage_5_bridge_iter<N>_<stageLabel>_assignment.json
        Same graph after EK — augmenting paths show donor->bridge-cell
        teleport assignments.

Usage:
    python3 visualize_flowgraph.py [INPUT_DIR] [OUTPUT_DIR]

Defaults: INPUT_DIR and OUTPUT_DIR both default to the same directory.  If you
point INPUT_DIR at a folder containing per-config subfolders (the standard
layout), the script recurses one level and renders every config.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import FancyArrowPatch
import numpy as np


# ---------------------------------------------------------------------------
# Role / style table
# ---------------------------------------------------------------------------

# Node visual style per role.
#   fill   : facecolor ('none' means hollow)
#   edge   : edgecolor
#   text   : annotation drawn inside the circle (empty for none)
#   size   : marker size (in points^2)
#   z      : matplotlib z-order
ROLE_STYLE = {
    "super_source":  dict(fill="#ffd700", edge="#b58900", text="S", size=320, z=10),
    "super_sink":    dict(fill="#8e44ad", edge="#5c2a86", text="T", size=320, z=10),
    "source":        dict(fill="#e63946", edge="black",   text="",  size=260, z=7),
    "idle":          dict(fill="#2ecc71", edge="black",   text="",  size=260, z=6),
    "target":        dict(fill="none",     edge="#2ecc71", text="",  size=260, z=6),
    "bridge_empty":  dict(fill="#cfe6ff", edge="#1f77b4", text="",  size=260, z=6),
    "bridge_filled": dict(fill="#ff8c00", edge="black",   text="B", size=260, z=8),
    "structural":    dict(fill="none",     edge="black",   text="",  size=260, z=5),
    "empty":         dict(fill="#dde7f0", edge="#a8b6c2", text="",  size=140, z=3),
}

ROLE_LEGEND = {
    "super_source":  ("super-source",        "#ffd700", "#b58900", "filled"),
    "super_sink":    ("super-sink",          "#8e44ad", "#5c2a86", "filled"),
    "source":        ("source / donor",      "#e63946", "black",   "filled"),
    "idle":          ("idle module",         "#2ecc71", "black",   "filled"),
    "target":        ("target cell",         "none",     "#2ecc71", "hollow"),
    "bridge_empty":  ("bridge candidate",    "#cfe6ff", "#1f77b4", "filled"),
    "bridge_filled": ("bridge cell (B)",     "#ff8c00", "black",   "filled"),
    "structural":    ("structure",            "none",     "black",   "hollow"),
    "empty":         ("virtual (empty)",      "#dde7f0", "#a8b6c2", "filled"),
}


# ---------------------------------------------------------------------------
# Projection
# ---------------------------------------------------------------------------

def project(node) -> tuple[float, float]:
    """2D projection.  Z layers are separated by a small shear so they don't
    collide; x and y otherwise come straight from the lattice."""
    return (node["x"] + 0.4 * node["z"], node["y"] + 0.4 * node["z"])


# ---------------------------------------------------------------------------
# Drawing primitives
# ---------------------------------------------------------------------------

def draw_node(ax, x, y, role: str):
    st = ROLE_STYLE.get(role, ROLE_STYLE["empty"])
    ax.scatter([x], [y], s=st["size"],
               facecolors=st["fill"] if st["fill"] != "none" else "white",
               edgecolors=st["edge"],
               linewidths=1.6 if st["fill"] == "none" else 1.4,
               zorder=st["z"])
    if st["text"]:
        ax.text(x, y, st["text"], ha="center", va="center",
                fontsize=8, fontweight="bold",
                color="white" if role in ("bridge_filled", "super_sink") else
                      "black",
                zorder=st["z"] + 1)


def draw_edge(ax, p1, p2, *, color, lw, alpha=1.0, zorder=4,
              shrink=10, head=10):
    patch = FancyArrowPatch(
        p1, p2, arrowstyle="-|>", mutation_scale=head,
        color=color, lw=lw, alpha=alpha,
        shrinkA=shrink, shrinkB=shrink, zorder=zorder,
    )
    ax.add_patch(patch)


def edge_label(e) -> str:
    if e.get("has_flow"):
        return f"{e['flow']}/{e['cap']}"
    return str(e["cap"])


# ---------------------------------------------------------------------------
# Stage renderer
# ---------------------------------------------------------------------------

def render_stage(stage: dict, out_path: Path):
    nodes = stage["nodes"]
    edges = stage["edges"]
    positions = {n["key"]: project(n) for n in nodes}

    n_nodes = len(nodes)
    n_edges = len(edges)

    fig, ax = plt.subplots(figsize=(14, 9))
    ax.set_title(stage.get("title", stage.get("stage", "flow graph")),
                 fontsize=13, fontweight="bold", pad=14)
    ax.set_axisbelow(True)
    ax.grid(True, color="#cccccc", linewidth=0.6, zorder=0)
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_aspect("equal")

    # Edge style sets.
    aug_path_edge_set: set[tuple[str, str]] = set()
    for p in stage.get("paths", []):
        for i in range(len(p) - 1):
            aug_path_edge_set.add((p[i], p[i + 1]))

    # Draw all forward edges with capacity labels.
    # Heuristic: drop capacity labels when there are too many edges to stay
    # readable; still draw arrows for context.
    label_caps = n_edges <= 220

    for e in edges:
        a = positions.get(e["from"])
        b = positions.get(e["to"])
        if a is None or b is None:
            continue
        is_mincut = e.get("is_mincut", False)
        is_sat = e.get("saturated", False)
        is_aug = (e["from"], e["to"]) in aug_path_edge_set
        if is_mincut:
            color, lw, alpha, z = "#d62728", 2.4, 1.0, 7
        elif is_sat or is_aug:
            color, lw, alpha, z = "#d62728", 1.8, 1.0, 6
        elif e.get("has_flow") and e["flow"] > 0:
            color, lw, alpha, z = "#1f77b4", 1.2, 0.9, 5
        else:
            color, lw, alpha, z = "#cccccc", 0.6, 0.7, 2
        draw_edge(ax, a, b, color=color, lw=lw, alpha=alpha, zorder=z,
                  head=8 if (is_mincut or is_sat or is_aug) else 6)

        if label_caps:
            mx = 0.5 * (a[0] + b[0])
            my = 0.5 * (a[1] + b[1])
            ax.text(mx, my, edge_label(e),
                    fontsize=6, ha="center", va="center", color="black",
                    bbox=dict(boxstyle="round,pad=0.10",
                              facecolor="white", edgecolor="none", alpha=0.78),
                    zorder=z + 1)

    # Augmenting paths overlay (distinct hues), drawn on top of red.
    cmap = matplotlib.colormaps["tab10"]
    path_handles: list[Line2D] = []
    for i, path in enumerate(stage.get("paths", [])):
        col = cmap(i % 10)
        for j in range(len(path) - 1):
            a = positions.get(path[j])
            b = positions.get(path[j + 1])
            if a is None or b is None:
                continue
            draw_edge(ax, a, b, color=col, lw=2.2, alpha=0.95, zorder=8,
                      shrink=12, head=10)
        path_handles.append(
            Line2D([0], [0], color=col, lw=2, label=f"aug. path {i + 1}"))

    # Nodes.
    used_roles: set[str] = set()
    for n in nodes:
        role = n.get("role", "empty")
        x, y = positions[n["key"]]
        draw_node(ax, x, y, role)
        used_roles.add(role)

    # Labels for super-source / super-sink.
    for n in nodes:
        if n["role"] in ("super_source", "super_sink"):
            p = positions[n["key"]]
            offx = -14 if n["role"] == "super_source" else 14
            ha = "right" if n["role"] == "super_source" else "left"
            ax.annotate(n["key"], p, xytext=(offx, 6),
                        textcoords="offset points", fontsize=8,
                        fontweight="bold", color="black",
                        ha=ha, zorder=11)

    # ----- Legend -----
    role_handles: list[Line2D] = []
    legend_order = ("super_source", "super_sink", "source", "idle",
                    "target", "bridge_empty", "bridge_filled", "structural",
                    "empty")
    for role in legend_order:
        if role not in used_roles:
            continue
        label, fill, edge, kind = ROLE_LEGEND[role]
        if kind == "filled":
            handle = Line2D([0], [0], marker="o", color="none",
                            markerfacecolor=fill, markeredgecolor=edge,
                            markersize=11, linewidth=0, label=label)
        else:
            handle = Line2D([0], [0], marker="o", color="none",
                            markerfacecolor="white", markeredgecolor=edge,
                            markeredgewidth=1.8, markersize=11,
                            linewidth=0, label=label)
        role_handles.append(handle)

    edge_handles: list[Line2D] = []
    if any(e.get("is_mincut", False) for e in edges):
        edge_handles.append(Line2D([0], [0], color="#d62728", lw=2,
                                    label="min-cut arc"))
    if any(e.get("saturated", False) or
           (e.get("has_flow") and e["flow"] > 0) for e in edges):
        edge_handles.append(Line2D([0], [0], color="#1f77b4", lw=1.6,
                                    label="edge with flow"))
    edge_handles.append(Line2D([0], [0], color="#cccccc", lw=1,
                                label="forward edge (cap label)"))

    leg = ax.legend(
        handles=role_handles + edge_handles + path_handles,
        loc="upper right", fontsize=9, framealpha=0.95, ncol=1,
        title=f"|V|={n_nodes}    |E|={n_edges}",
        title_fontsize=9,
    )
    leg.set_zorder(50)

    # Tight bounds with padding; integer ticks.
    xs = [p[0] for p in positions.values()]
    ys = [p[1] for p in positions.values()]
    if xs:
        pad = 1.0
        ax.set_xlim(min(xs) - pad, max(xs) + pad)
        ax.set_ylim(min(ys) - pad, max(ys) + pad)
        import math
        xmin, xmax = int(math.floor(min(xs))), int(math.ceil(max(xs)))
        ymin, ymax = int(math.floor(min(ys))), int(math.ceil(max(ys)))
        ax.set_xticks(range(xmin, xmax + 1))
        ax.set_yticks(range(ymin, ymax + 1))

    fig.tight_layout()
    fig.savefig(out_path, dpi=160, facecolor="white")
    plt.close(fig)


# ---------------------------------------------------------------------------
# Driver
# ---------------------------------------------------------------------------

def render_directory(in_dir: Path, out_dir: Path) -> int:
    out_dir.mkdir(parents=True, exist_ok=True)
    sources = sorted(in_dir.glob("stage_*.json"))
    if not sources:
        return 0
    n = 0
    for src in sources:
        with src.open() as f:
            data = json.load(f)
        out = out_dir / (src.stem + ".png")
        try:
            render_stage(data, out)
            print(f"[viz] wrote {out}")
            n += 1
        except Exception as exc:
            print(f"[viz] failed to render {src.name}: {exc}",
                  file=sys.stderr)
    return n


def main(argv: list[str]) -> int:
    in_dir = Path(argv[1] if len(argv) > 1 else "viz_flowgraph")
    out_dir = Path(argv[2] if len(argv) > 2 else str(in_dir))

    if not in_dir.is_dir():
        print(f"[viz] input directory not found: {in_dir}", file=sys.stderr)
        return 1

    total = render_directory(in_dir, out_dir)

    # If no JSONs at the top level, treat each subdirectory as a per-config
    # bucket and render recursively (one level).
    if total == 0:
        for sub in sorted(p for p in in_dir.iterdir() if p.is_dir()):
            target = out_dir / sub.name if in_dir != out_dir else sub
            total += render_directory(sub, target)

    if total == 0:
        print(f"[viz] no stage_*.json files found under {in_dir}",
              file=sys.stderr)
        return 2

    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
