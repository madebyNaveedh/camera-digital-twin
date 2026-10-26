# Real-Time Camera Digital Twin (CoppeliaSim + OpenCV)

A real-time digital twin that links physical desktop tracking to a 3D CoppeliaSim simulation. Physical obstacles (tracked via webcam using HSV color segmentation or ArUco markers) are mapped in real time into a simulated 3m × 3m arena. An autonomous differential-drive Pioneer P3DX mobile robot patrols the room perimeter and reacts dynamically when obstacles enter its path.

Included in the repo is also the full [2-page technical report](digital_twin_report.pdf) summarizing the mathematical transforms, architecture, and benchmark latencies.

## Overview

1. **Physical Tracking**: A standard webcam monitors a defined desk workspace at 30 FPS.
2. **Coordinate Transform**: Pixel positions are scaled and filtered with an Exponential Moving Average (EMA) to smooth hand jitter.
3. **CoppeliaSim Synchronization**: Centroid coordinates are streamed to CoppeliaSim over the ZeroMQ Remote API (< 15ms round-trip).
4. **Autonomous Navigation**: The Pioneer P3DX mobile robot follows a square perimeter waypoint loop, performs zero-radius in-place turns at corners, and halts safely if an active obstacle blocks its path.

---

## Prerequisites

- **OS**: Ubuntu 20.04 / 22.04 LTS
- **Python**: 3.10+
- **Simulator**: [CoppeliaSim](https://www.coppeliarobotics.com/) (Edu version 4.6+ or 4.10)
- **Camera**: Standard USB webcam (Logitech C270 or integrated webcam at `/dev/video0` or `/dev/video2`)

---

## Setup

1. **Clone the repository:**
   ```bash
   git clone https://github.com/madebyNaveedh/camera-digital-twin.git
   cd camera-digital-twin
   ```

2. **Create and activate a virtual environment:**
   ```bash
   python3 -m venv venv
   source venv/bin/activate
   ```

3. **Install dependencies:**
   ```bash
   pip install -r requirements.txt
   ```

4. **Set CoppeliaSim path:**
   Point `COPPELIASIM_ROOT` to your CoppeliaSim installation directory:
   ```bash
   export COPPELIASIM_ROOT="/path/to/CoppeliaSim_Edu_Ubuntu22_04"
   ```

---

## Running the System

### 1. Launch Everything via Script
The easiest way is to run the launcher script, which opens CoppeliaSim with the arena scene and starts the Python tracking loop:
```bash
bash camera_digital_twin/run.sh
```

### 2. Manual Launch
Alternatively, you can launch each component manually:
1. Open CoppeliaSim and load the scene:
   ```bash
   $COPPELIASIM_ROOT/coppeliaSim.sh camera_digital_twin/camera_twin_scene.ttt
   ```
2. In a separate terminal, run the tracker:
   ```bash
   python3 camera_digital_twin/main.py
   ```

---

## Keyboard Controls (OpenCV Window)

While the camera window is focused:

| Key | Action |
|---|---|
| `1` | Switch to **Blue** color preset |
| `2` | Switch to **Green** color preset |
| `3` | Switch to **Red** color preset |
| `A` | Switch to **ArUco marker** tracking (`DICT_4X4_50`, ID 0) |
| `+` / `-` | Increase / decrease desk sensitivity gain |
| `Q` / `ESC` | Exit application |

---

## Testing

Run unit tests covering marker generation, color segmentation, and coordinate transforms:
```bash
python3 camera_digital_twin/test_camera_digital_twin.py
```

---

## Project Structure

```
camera-digital-twin/
├── camera_digital_twin/
│   ├── main.py                    # Main control loop
│   ├── camera_tracker.py          # OpenCV HSV & ArUco tracking
│   ├── coordinate_mapping.py      # Pixel-to-metric bilinear transform & EMA
│   ├── coppelia_interface.py      # ZeroMQ Remote API interface & robot controller
│   ├── camera_twin_scene.ttt      # CoppeliaSim 3D arena scene
│   ├── config.yaml                # Configuration parameters (camera, bounds, speeds)
│   ├── run.sh                     # Launch script
│   ├── record_screen.sh           # Screen recording helper script
│   └── test_camera_digital_twin.py# Unit tests
├── digital_twin_report.pdf        # 2-page summary document
├── requirements.txt               # Python package requirements
└── README.md
```

## License
MIT
