#!/usr/bin/env bash
# =============================================================================
#  run_and_visualize.sh
#  --------------------
#  One-shot driver for the Catoms3D Flow Bridge analysis:
#    1. Run the simulator in text mode on a chosen XML config.
#    2. Dump JSON per flow-graph phase (handled by the C++ side).
#    3. Render 2D PNGs per phase via visualize_flowgraph.py.
#
#  All outputs land in ./viz_flowgraph/<config_stem>/ next to this script
#  (i.e. inside applicationsBin/flowBridge/).
#
#  Usage:
#      ./run_and_visualize.sh short_pinch_4x4.xml
#      ./run_and_visualize.sh --all      # render every *.xml in this dir
# =============================================================================
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$HERE"

EXE="${EXE:-$HERE/../../build-mingw64/flowBridgeTestMinCut.exe}"
if [[ ! -x "$EXE" && -x "$HERE/../../build/flowBridgeTestMinCut" ]]; then
    EXE="$HERE/../../build/flowBridgeTestMinCut"
fi
VIZ="${VIZ:-$HERE/../../applicationsSrc/flowBridgeTestMinCut/visualize_flowgraph.py}"
PYTHON="${PYTHON:-python3}"

# On Windows under MSYS2, make sure the mingw64 DLLs are reachable.
if [[ -d /c/msys64/mingw64/bin ]]; then
    export PATH="/c/msys64/mingw64/bin:/c/msys64/usr/bin:$PATH"
fi

run_one() {
    local xml="$1"
    [[ -f "$xml" ]] || { echo "missing: $xml" >&2; return 1; }
    echo
    echo "===== $xml ====="
    "$EXE" -t -c "$xml" 2>&1 | grep -E '\[viz\]' || true
    local stem="${xml%.*}"; stem="${stem##*/}"
    local dir="viz_flowgraph/$stem"
    if [[ ! -d "$dir" ]]; then
        echo "no JSON output for $stem" >&2
        return 0
    fi
    "$PYTHON" "$VIZ" "$dir" "$dir"
}

if [[ "${1:-}" == "--all" ]]; then
    for xml in *.xml; do run_one "$xml"; done
elif [[ -n "${1:-}" ]]; then
    run_one "$1"
else
    cat <<EOF
Usage:
  $0 <config.xml>
  $0 --all

Available configs:
$(ls *.xml 2>/dev/null | sed 's/^/  /')
EOF
fi
