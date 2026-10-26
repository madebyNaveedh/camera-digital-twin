#!/usr/bin/env python3
"""
ArUco Marker Generator.
Generates printable / phone-displayable ArUco markers with quiet zone borders.
"""

import os
import cv2
import numpy as np


def generate_marker(marker_id: int = 0, dict_name: str = "DICT_4X4_50", size_px: int = 400, output_dir: str = "."):
    aruco_dict_id = getattr(cv2.aruco, dict_name, cv2.aruco.DICT_4X4_50)
    dictionary = cv2.aruco.getPredefinedDictionary(aruco_dict_id)
    
    # Generate marker
    marker_core = cv2.aruco.generateImageMarker(dictionary, marker_id, size_px)
    
    # Add white margin (quiet zone) - critical for ArUco detection!
    border_px = int(size_px * 0.15)
    marker_img = cv2.copyMakeBorder(
        marker_core,
        border_px, border_px, border_px, border_px,
        cv2.BORDER_CONSTANT,
        value=255
    )

    filename = os.path.join(output_dir, f"marker_id_{marker_id}.png")
    cv2.imwrite(filename, marker_img)
    print(f"Generated ArUco marker (ID: {marker_id}, Dict: {dict_name}) saved to {filename}")
    return filename


if __name__ == "__main__":
    out_dir = os.path.dirname(os.path.abspath(__file__))
    generate_marker(0, "DICT_4X4_50", 400, out_dir)
