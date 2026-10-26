#!/usr/bin/env python3
"""
Hardware & Environment Inspection Utility.
Checks OS, CPU, GPU, Camera, LiDAR serial ports, and CoppeliaSim availability.
"""

import os
import sys
import glob
import subprocess
import cv2

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial = None
    list_ports = None

try:
    from coppeliasim_zmqremoteapi_client import RemoteAPIClient
except ImportError:
    RemoteAPIClient = None


def inspect():
    print("=" * 60)
    print(" DIGITAL TWIN HARDWARE & ENVIRONMENT INSPECTION")
    print("=" * 60)

    # 1. OS & Python
    print("\n[1] SYSTEM & RUNTIME:")
    print(f"  Python Version: {sys.version.split()[0]} ({sys.executable})")
    try:
        distro = subprocess.check_output(["lsb_release", "-d"], text=True).strip()
        print(f"  OS Distribution: {distro.split(':', 1)[1].strip()}")
    except Exception:
        print("  OS Distribution: Linux (generic)")

    # 2. Camera Devices
    print("\n[2] CAMERA DEVICES:")
    video_devices = sorted(glob.glob("/dev/video*"))
    print(f"  Detected Video Nodes: {video_devices if video_devices else 'None'}")
    
    for idx in range(3):
        cap = cv2.VideoCapture(idx)
        if cap.isOpened():
            w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            fps = cap.get(cv2.CAP_PROP_FPS)
            backend = cap.getBackendName()
            ret, _ = cap.read()
            cap.release()
            print(f"  -> Camera Index {idx}: AVAILABLE (Resolution: {w}x{h}, FPS: {fps}, Backend: {backend}, Frame Capture: {ret})")
        else:
            cap.release()

    # 3. Serial / LiDAR Ports
    print("\n[3] SERIAL & LIDAR HARDWARE:")
    serial_nodes = sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))
    print(f"  Serial Device Nodes: {serial_nodes if serial_nodes else 'None currently connected'}")
    
    if list_ports:
        ports = list(list_ports.comports())
        if ports:
            for p in ports:
                print(f"  -> Port: {p.device} | Description: {p.description} | HWID: {p.hwid}")
        else:
            print("  -> No USB-UART serial converters found on USB bus.")
    print("  Note: If physical RPLIDAR is plugged in, it will appear as /dev/ttyUSB0.")

    # 4. CoppeliaSim
    print("\n[4] COPPELIASIM SIMULATOR:")
    cop_dir = "/home/mohammed/Downloads/CoppeliaSim_Edu_V4_10_0_rev0_Ubuntu22_04"
    if os.path.isdir(cop_dir):
        print(f"  Installation found at: {cop_dir}")
    else:
        print(f"  Installation path {cop_dir} not found.")

    if RemoteAPIClient is not None:
        print("  coppeliasim_zmqremoteapi_client Python package: INSTALLED")
        try:
            client = RemoteAPIClient()
            client.timeout = 1.0
            sim = client.require("sim")
            ver = sim.getInt32Param(sim.intparam_program_version)
            print(f"  Active CoppeliaSim Remote API Server: CONNECTED (Version code: {ver})")
        except Exception:
            print("  Active CoppeliaSim Remote API Server: NOT CURRENTLY RUNNING (Start CoppeliaSim to connect)")
    else:
        print("  coppeliasim_zmqremoteapi_client Python package: NOT INSTALLED")

    print("\n" + "=" * 60)


if __name__ == "__main__":
    inspect()
