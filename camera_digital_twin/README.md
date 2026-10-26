# Camera-Driven Digital Twin

This module implements the camera perception pipeline and CoppeliaSim integration.

## Files
- `main.py`: Main execution loop. Reads camera frames, runs tracking, and sends updates to simulation.
- `camera_tracker.py`: OpenCV tracker supporting ArUco markers (`DICT_4X4_50`) and HSV color presets (Blue, Green, Red).
- `coordinate_mapping.py`: Normalizes desk pixel coordinates to arena metric coordinates with EMA smoothing.
- `coppelia_interface.py`: Handles ZeroMQ Remote API communication, scene objects, and Pioneer P3DX navigation.
- `camera_twin_scene.ttt`: CoppeliaSim scene containing the 3m x 3m arena and Pioneer P3DX robot.
- `config.yaml`: Camera device settings, color thresholds, and simulation parameters.
- `run.sh`: Launch helper script.
- `record_screen.sh`: Screen recording script (1080p @ 30 FPS).
- `test_camera_digital_twin.py`: Unit tests.

## Controls
- `1` / `2` / `3`: Switch color presets (Blue, Green, Red).
- `A`: Switch to ArUco marker tracking.
- `+` / `-`: Adjust tracking sensitivity.
- `Q`: Quit.
