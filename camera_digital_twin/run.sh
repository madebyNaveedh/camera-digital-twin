#!/usr/bin/env bash
set -e

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export PYTHONPATH="$DIR:$PYTHONPATH"
export COPPELIASIM_ROOT="${COPPELIASIM_ROOT:-/home/mohammed/Downloads/CoppeliaSim_Edu_V4_10_0_rev0_Ubuntu22_04}"
export DISPLAY="${DISPLAY:-:1}"

SCENE_PATH="$DIR/camera_twin_scene.ttt"

# Start CoppeliaSim if not already running
if pgrep -f "coppeliaSim" > /dev/null; then
    echo "CoppeliaSim is already running."
else
    if [ ! -d "$COPPELIASIM_ROOT" ]; then
        echo "Error: CoppeliaSim not found at $COPPELIASIM_ROOT"
        echo "Please export COPPELIASIM_ROOT=/path/to/coppeliaSim"
        exit 1
    fi
    echo "Starting CoppeliaSim..."
    "$COPPELIASIM_ROOT/coppeliaSim.sh" "$SCENE_PATH" </dev/null >/dev/null 2>&1 &
    
    echo "Waiting for Remote API (port 23000)..."
    for i in {1..20}; do
        if (echo > /dev/tcp/127.0.0.1/23000) 2>/dev/null; then
            echo "Connected to CoppeliaSim."
            break
        fi
        sleep 0.5
    done
fi

echo "Starting camera digital twin..."
echo "Keys: 1: Blue | 2: Green | 3: Red | A: ArUco | +/-: Sensitivity | Q: Quit"

python3 "$DIR/main.py"
