#!/usr/bin/env bash
set -e

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUTPUT_FILE="$DIR/demo_recording_$(date +%Y%m%d_%H%M%S).mp4"
DISPLAY_NUM="${DISPLAY:-:1}"

echo "Starting screen recording on display $DISPLAY_NUM (1920x1080 @ 30 FPS)..."
echo "Output file: $OUTPUT_FILE"
echo "Press 'q' or 'Ctrl+C' to stop recording."

ffmpeg -loglevel warning -y \
    -f x11grab \
    -framerate 30 \
    -video_size 1920x1080 \
    -i "${DISPLAY_NUM}.0" \
    -c:v libx264 \
    -preset fast \
    -pix_fmt yuv420p \
    "$OUTPUT_FILE"

echo "Saved: $OUTPUT_FILE"
