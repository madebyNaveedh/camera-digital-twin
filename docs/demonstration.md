# Live Demonstration Guide & Evaluation Checklist

This document provides the exact checklist and operational procedures for demonstrating the Real-Time Digital Twin to faculty and evaluators.

---

## 1. Pre-Demonstration Checklist

- [x] Ubuntu 22.04 LTS host with CoppeliaSim Edu V4.10.0 confirmed.
- [x] ZeroMQ Remote API communication operational on port `23000`.
- [x] Camera device `/dev/video0` accessible.
- [x] Python dependencies verified (`numpy`, `opencv-python`, `coppeliasim_zmqremoteapi_client`, `pyserial`).
- [x] All 18 automated unit and integration tests passing (`python3 -m unittest discover tests`).

---

## 2. Launching the System

Execute the master launch script from the project root:
```bash
./scripts/run_system.sh
```
*(Or with `--headless` if running in terminal-only environments).*

The script will:
1. Run `inspect_hardware.py` and output device health.
2. Launch CoppeliaSim with the 3D room, walls, and Pioneer P3DX robot.
3. Start the Web Monitoring Dashboard on `http://localhost:8085`.
4. Initiate the closed-loop perception and navigation engine.

Open `http://localhost:8085` in any browser to display the live dashboard.

---

## 3. Live Demonstration Sequence

During the demonstration, the following causal chain is demonstrated live:

### Stage 1: Initial Virtual Twin State
- The physical room boundaries (5.0m x 4.0m) and Pioneer P3DX robot appear inside CoppeliaSim.
- The live webcam feed is streaming on the dashboard.
- The robot begins moving smoothly from $(-1.6, -1.2)$ toward the green goal marker at $(1.6, 1.2)$.
- Navigation state shows `NAVIGATING`.

### Stage 2: Physical Obstacle Introduction
- **Action:** A physical obstacle (or test cylinder via dashboard button `+ Place Obstacle in Robot Path (0, 0)`) is introduced.
- **System Response:**
  1. LiDAR scans detect the new reflection cluster.
  2. The perception module clusters the points and calculates the centroid.
  3. The Digital Twin state confirms the object after 2 frames ($< 200$ ms).
  4. A red obstacle cylinder dynamically appears inside CoppeliaSim at $(0.0, 0.0)$.
  5. The Pioneer P3DX detects the obstacle in its forward path.
  6. The robot safely slows down, deflects laterally (`AVOIDING`), or stops if blocked (`BLOCKED`).

### Stage 3: Dynamic Obstacle Relocation
- **Action:** The physical obstacle is moved to $(0.8, -0.2)$.
- **System Response:**
  1. The tracking synchronizer maintains object association ($d_{assoc} \le 0.45$ m).
  2. The virtual cylinder in CoppeliaSim smoothly updates its position without flickering or duplicating objects.
  3. The robot adjusts its trajectory in response to the new obstacle coordinates.

### Stage 4: Obstacle Removal & Mission Resumption
- **Action:** The physical obstacle is removed.
- **System Response:**
  1. LiDAR scans observe clear room space.
  2. The synchronizer retains the obstacle during a brief persistence window ($2.0$ s), preventing dropouts.
  3. After the timeout, the obstacle is cleanly removed from CoppeliaSim.
  4. The robot transitions back to `NAVIGATING`.
  5. The robot drives forward to the goal marker and transitions to `GOAL_REACHED`.

---

## 4. Hardware Transition (Plugging in Physical RPLIDAR)

When the physical RPLIDAR A1 M8 USB cable is connected:
1. Verify device recognition:
   ```bash
   python3 scripts/inspect_hardware.py
   ```
   *(Should display `/dev/ttyUSB0`)*
2. Check permissions: `ls -l /dev/ttyUSB0` (User is in `dialout` group).
3. The engine will automatically detect and connect to `/dev/ttyUSB0` on startup.
