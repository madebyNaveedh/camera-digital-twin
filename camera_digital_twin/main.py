#!/usr/bin/env python3
"""
Main entry point for the camera digital twin.
Connects OpenCV tracker to CoppeliaSim simulation.
"""

import os
import sys
import time
import yaml
import cv2
import logging

from camera_tracker import CameraTracker
from coordinate_mapping import CoordinateMapper
from coppelia_interface import CoppeliaInterface

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
logger = logging.getLogger("CameraDigitalTwin")

WINDOW_NAME = "Digital Twin Camera Feed (Press Q to Exit)"


def load_config(config_path: str = "config.yaml") -> dict:
    if not os.path.isabs(config_path):
        config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), config_path)
    with open(config_path, "r") as f:
        return yaml.safe_load(f)


def run_system(config_file: str = "config.yaml", show_gui: bool = True, max_duration: float = None):
    cfg = load_config(config_file)
    cam_cfg = cfg["camera"]
    track_cfg = cfg["tracker"]
    map_cfg = cfg["coordinate_mapping"]
    sim_cfg = cfg["simulation"]

    # 1. Initialize Camera Tracker
    tracker = CameraTracker(
        device_index=cam_cfg.get("device_index", 2),
        width=cam_cfg.get("width", 640),
        height=cam_cfg.get("height", 480),
        fps=cam_cfg.get("fps", 30),
        mode=track_cfg.get("mode", "color"),
        aruco_dict_name=track_cfg.get("aruco", {}).get("dictionary", "DICT_4X4_50"),
        target_marker_id=track_cfg.get("aruco", {}).get("target_id", 0),
        color_preset=track_cfg.get("color_preset", "auto"),
        color_min_area=track_cfg.get("color", {}).get("min_area_pixels", 300)
    )

    # 2. Initialize Coordinate Mapper with Sensitivity Scaling
    roi = map_cfg["image_roi"]
    sim_ws = map_cfg["sim_workspace"]
    safety_z = sim_cfg["safety_zone"]
    
    mapper = CoordinateMapper(
        u_min=roi.get("u_min", 140),
        u_max=roi.get("u_max", 500),
        v_min=roi.get("v_min", 100),
        v_max=roi.get("v_max", 420),
        sim_x_min=sim_ws.get("x_min", -1.4),
        sim_x_max=sim_ws.get("x_max", 1.4),
        sim_y_min=sim_ws.get("y_min", -1.4),
        sim_y_max=sim_ws.get("y_max", 1.4),
        sensitivity_gain=map_cfg.get("sensitivity_gain", 1.0),
        smoothing_alpha=map_cfg.get("smoothing_alpha", 0.70),
        loss_timeout=map_cfg.get("loss_timeout_seconds", 2.0),
        safety_zone=(safety_z["x_min"], safety_z["x_max"], safety_z["y_min"], safety_z["y_max"])
    )

    # 3. Initialize CoppeliaSim Interface (Square Loop Patrol)
    sim_interface = CoppeliaInterface(
        host=sim_cfg["host"],
        port=sim_cfg["port"],
        enable_robot=sim_cfg["enable_robot"],
        safety_zone=(safety_z["x_min"], safety_z["x_max"], safety_z["y_min"], safety_z["y_max"]),
        patrol_speed=sim_cfg.get("robot_patrol_speed", 0.16)
    )

    # Start Camera
    logger.info(f"Opening camera {cam_cfg.get('device_index', 2)}...")
    if not tracker.start():
        logger.error("Failed to open camera device. Please ensure webcam is connected.")
        return False

    # Connect Simulation
    logger.info("Connecting to CoppeliaSim...")
    if sim_interface.connect():
        sim_interface.setup_scene()
    else:
        logger.warning("CoppeliaSim not connected. Running in camera-only visualization mode.")

    roi_tuple = (int(mapper.u_min), int(mapper.u_max), int(mapper.v_min), int(mapper.v_max))
    start_time = time.time()

    if show_gui:
        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_AUTOSIZE)

    logger.info("Live digital twin loop running!")
    logger.info("Color Presets: [1: Blue] [2: Green] [3: Red] | A: ArUco | +/-: Sensitivity | Q: Quit")

    try:
        while True:
            if max_duration and (time.time() - start_time) > max_duration:
                break

            # A. Capture Camera Frame
            ret, frame = tracker.read_frame()
            if not ret or frame is None:
                time.sleep(0.01)
                continue

            # B. Track Object
            track_res = tracker.track(frame)

            # C. Map Pixel Coordinates -> CoppeliaSim Coordinates
            sim_pos, is_active, in_safety_zone = mapper.update(track_res["center"])

            # D. Synchronize with CoppeliaSim
            if sim_interface.is_connected and sim_pos is not None:
                sim_interface.update_virtual_obstacle(sim_pos[0], sim_pos[1], is_active)

            # E. Update Robot Navigation & Collision Avoidance
            nav_status = sim_interface.update_robot_interaction(in_safety_zone=in_safety_zone, is_active=is_active)

            # F. Render OpenCV Display HUD
            if show_gui:
                hud_frame = tracker.draw_hud(
                    frame=frame,
                    track_res=track_res,
                    roi_bounds=roi_tuple,
                    sim_coords=sim_pos,
                    sensitivity=mapper.sensitivity_gain,
                    in_safety_zone=in_safety_zone,
                    nav_status=nav_status
                )

                try:
                    cv2.imshow(WINDOW_NAME, hud_frame)
                    key = cv2.waitKey(1) & 0xFF
                    if key in (ord('q'), ord('Q'), 27):  # 'q' or ESC
                        break
                    elif key == ord('1'):
                        tracker.set_preset("blue")
                    elif key == ord('2'):
                        tracker.set_preset("green")
                    elif key == ord('3'):
                        tracker.set_preset("red")
                    elif key in (ord('a'), ord('A')):
                        tracker.mode = "aruco"
                        logger.info("Switched to ArUco marker tracking mode.")
                    elif key in (ord('c'), ord('C')):
                        tracker.mode = "color"
                        logger.info("Switched to Color tracking mode.")
                    elif key in (ord('+'), ord('=')):
                        mapper.set_sensitivity(+0.2)
                        logger.info(f"Sensitivity increased to: {mapper.sensitivity_gain:.1f}x")
                    elif key in (ord('-'), ord('_')):
                        mapper.set_sensitivity(-0.2)
                        logger.info(f"Sensitivity decreased to: {mapper.sensitivity_gain:.1f}x")
                    elif key in (ord('p'), ord('P')):
                        sim_interface.patrol_mode = "circuit" if sim_interface.patrol_mode == "corridor" else "corridor"
                        logger.info(f"Switched robot patrol mode to: {sim_interface.patrol_mode.upper()}")
                except cv2.error:
                    pass

    except KeyboardInterrupt:
        logger.info("Interrupted by user.")

    finally:
        logger.info("Cleaning up resources...")
        tracker.release()
        sim_interface.stop()
        if show_gui:
            try:
                cv2.destroyAllWindows()
            except Exception:
                pass
        logger.info("Shutdown complete.")
        return True


if __name__ == "__main__":
    run_system(show_gui=True)
