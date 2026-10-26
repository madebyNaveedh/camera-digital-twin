#!/usr/bin/env bash
# ==============================================================================
# Master Launch Script for the Real-Time Digital Twin System
# ==============================================================================
set -e

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONPATH="$WORKSPACE_DIR:$PYTHONPATH"
export COPPELIASIM_ROOT="/home/mohammed/Downloads/CoppeliaSim_Edu_V4_10_0_rev0_Ubuntu22_04"
export DISPLAY="${DISPLAY:-:1}"

echo "======================================================================"
echo " REAL-TIME DIGITAL TWIN: PHYSICAL ROOM & AUTONOMOUS NAVIGATION"
echo "======================================================================"
echo "Workspace:      $WORKSPACE_DIR"
echo "CoppeliaSim:    $COPPELIASIM_ROOT"
echo "Display:        $DISPLAY"

HEADLESS=false
for arg in "$@"; do
    if [ "$arg" == "--headless" ]; then
        HEADLESS=true
    fi
done

# Step 1: Run hardware inspection
echo ""
echo "--> Checking connected hardware and environment..."
python3 "$WORKSPACE_DIR/scripts/inspect_hardware.py"

# Step 2: Check if CoppeliaSim is already running
if pgrep -x "coppeliaSim" > /dev/null; then
    echo "--> CoppeliaSim is already running."
else
    echo "--> Launching CoppeliaSim..."
    if [ "$HEADLESS" = true ]; then
        echo "    Mode: Headless (-h)"
        "$COPPELIASIM_ROOT/coppeliaSim.sh" -h "$COPPELIASIM_ROOT/scenes/workspace.ttt" </dev/null >/dev/null 2>&1 &
    else
        echo "    Mode: GUI (Scene will be visible on display $DISPLAY)"
        "$COPPELIASIM_ROOT/coppeliaSim.sh" "$COPPELIASIM_ROOT/scenes/workspace.ttt" </dev/null >/dev/null 2>&1 &
    fi
    sleep 3
fi

# Step 3: Run the integrated demonstration
echo ""
echo "--> Launching Digital Twin Engine & Dashboard..."
echo "    Dashboard: http://localhost:8085"
echo ""

python3 "$WORKSPACE_DIR/scripts/run_demonstration.py"

echo "======================================================================"
echo " System execution completed."
echo "======================================================================"
