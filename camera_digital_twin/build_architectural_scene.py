"""
Build Clean Seamless Square Box Arena Scene in CoppeliaSim.
Features:
- Completely enclosed, open square arena (3.0m x 3.0m interior).
- Clean perimeter walls with flush corners (zero intersecting crossbars).
- 5m x 5m textured concrete floor underneath (robot cannot fall off).
- Pioneer P3DX mobile robot positioned at (+0.95, -0.95, 0.15) for perimeter loop patrol.
- Tracked virtual obstacle cylinder in the arena.
- Default camera view calibrated to frame the square arena.
"""

import sys
import math
import time
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

COPPELIASIM_DIR = "/home/mohammed/Downloads/CoppeliaSim_Edu_V4_10_0_rev0_Ubuntu22_04"
FLOOR_MODEL = f"{COPPELIASIM_DIR}/models/infrastructure/floors/5mX5m concrete floor.ttm"
ROBOT_MODEL = f"{COPPELIASIM_DIR}/models/robots/mobile/pioneer p3dx.ttm"
SCENE_SAVE_PATH = "/home/mohammed/digital_twin_ws/camera_digital_twin/camera_twin_scene.ttt"

def build_scene():
    print("Connecting to CoppeliaSim...")
    client = RemoteAPIClient()
    sim = client.require("sim")
    print(f"Connected! CoppeliaSim version: {sim.getInt32Param(sim.intparam_program_version)}")

    # Stop simulation if currently running
    sim.stopSimulation()
    time.sleep(0.3)

    # 1. Clean up old custom objects from previous builds
    protected = ['DefaultCamera', 'XYZCameraProxy', 'DefaultLights', 'LightA', 'LightB', 'LightC', 'LightD',
                 'XViewCamera', 'YViewCamera', 'ZViewCamera', 'NXViewCamera', 'NYViewCamera', 'NZViewCamera']

    for o in sim.getObjectsInTree(sim.handle_scene):
        try:
            alias = sim.getObjectAlias(o)
            if alias not in protected and not any(p in alias for p in ['Light', 'Camera', 'Proxy']):
                sim.removeObject(o)
        except Exception:
            pass

    # 2. Build 5m x 5m Solid Concrete Floor Tile centered at (0, 0, 0)
    print("Loading tiled concrete floor...")
    floor_handle = sim.loadModel(FLOOR_MODEL)
    sim.setObjectPosition(floor_handle, [0.0, 0.0, 0.0])
    sim.setObjectAlias(floor_handle, "Arena_Floor")

    # 3. Build Clean Perimeter Walls for 3.0m x 3.0m Open Square Room (Zero Crossbars)
    wall_height = 0.45
    wall_thick = 0.12

    # North Wall (along X)
    w_n = sim.createPrimitiveShape(sim.primitiveshape_cuboid, [3.24, wall_thick, wall_height])
    sim.setObjectAlias(w_n, "Arena_Wall_North")
    sim.setObjectPosition(w_n, [0.0, 1.56, wall_height / 2.0])
    sim.setObjectColor(w_n, 0, sim.colorcomponent_ambient_diffuse, [0.85, 0.85, 0.88])
    sim.setObjectInt32Param(w_n, sim.shapeintparam_static, 1)
    sim.setObjectInt32Param(w_n, sim.shapeintparam_respondable, 1)

    # South Wall (along X)
    w_s = sim.createPrimitiveShape(sim.primitiveshape_cuboid, [3.24, wall_thick, wall_height])
    sim.setObjectAlias(w_s, "Arena_Wall_South")
    sim.setObjectPosition(w_s, [0.0, -1.56, wall_height / 2.0])
    sim.setObjectColor(w_s, 0, sim.colorcomponent_ambient_diffuse, [0.88, 0.88, 0.90])
    sim.setObjectInt32Param(w_s, sim.shapeintparam_static, 1)
    sim.setObjectInt32Param(w_s, sim.shapeintparam_respondable, 1)

    # East Wall (along Y, flush between North and South)
    w_e = sim.createPrimitiveShape(sim.primitiveshape_cuboid, [wall_thick, 3.00, wall_height])
    sim.setObjectAlias(w_e, "Arena_Wall_East")
    sim.setObjectPosition(w_e, [1.56, 0.0, wall_height / 2.0])
    sim.setObjectColor(w_e, 0, sim.colorcomponent_ambient_diffuse, [0.88, 0.88, 0.90])
    sim.setObjectInt32Param(w_e, sim.shapeintparam_static, 1)
    sim.setObjectInt32Param(w_e, sim.shapeintparam_respondable, 1)

    # West Wall (along Y, flush between North and South)
    w_w = sim.createPrimitiveShape(sim.primitiveshape_cuboid, [wall_thick, 3.00, wall_height])
    sim.setObjectAlias(w_w, "Arena_Wall_West")
    sim.setObjectPosition(w_w, [-1.56, 0.0, wall_height / 2.0])
    sim.setObjectColor(w_w, 0, sim.colorcomponent_ambient_diffuse, [0.88, 0.88, 0.90])
    sim.setObjectInt32Param(w_w, sim.shapeintparam_static, 1)
    sim.setObjectInt32Param(w_w, sim.shapeintparam_respondable, 1)

    # 4. Center Reference Marker (subtle dot at (0, 0))
    center_marker = sim.createPrimitiveShape(sim.primitiveshape_cylinder, [0.20, 0.20, 0.01])
    sim.setObjectAlias(center_marker, "Arena_CenterMarker")
    sim.setObjectPosition(center_marker, [0.0, 0.0, 0.005])
    sim.setObjectColor(center_marker, 0, sim.colorcomponent_ambient_diffuse, [0.35, 0.65, 0.85])
    sim.setObjectInt32Param(center_marker, sim.shapeintparam_static, 1)
    sim.setObjectInt32Param(center_marker, sim.shapeintparam_respondable, 0)

    # 5. Tracked Virtual Obstacle Cylinder
    print("Creating dynamic tracked obstacle cylinder...")
    obs = sim.createPrimitiveShape(sim.primitiveshape_cylinder, [0.28, 0.28, 0.45])
    sim.setObjectAlias(obs, "Corridor_TrackedObstacle")
    sim.setObjectPosition(obs, [0.0, 0.0, 0.225])
    sim.setObjectColor(obs, 0, sim.colorcomponent_ambient_diffuse, [0.95, 0.45, 0.15])
    sim.setObjectInt32Param(obs, sim.shapeintparam_static, 1)
    sim.setObjectInt32Param(obs, sim.shapeintparam_respondable, 1)

    # 6. Pioneer P3DX Mobile Robot (Positioned safely on the square perimeter)
    print("Loading Pioneer P3DX robot...")
    pioneer = sim.loadModel(ROBOT_MODEL)
    sim.setObjectAlias(pioneer, "PioneerP3DX")
    # Position at (+0.95, -0.95, 0.15), facing North (+Y)
    sim.setObjectPosition(pioneer, [0.95, -0.95, 0.15])
    sim.setObjectOrientation(pioneer, [0.0, 0.0, math.pi / 2.0])

    # Disable internal script so external Python controller drives the wheels
    ps = sim.getScript(sim.scripttype_childscript, pioneer)
    if ps != -1:
        sim.setObjectInt32Param(ps, sim.scriptintparam_enabled, 0)
        print("Disabled Pioneer internal child script for external Python control.")

    # 7. Set default camera viewpoint to high isometric overview perfectly framing the square box
    try:
        cam = sim.getObject("/DefaultCamera")
        sim.setObjectPosition(cam, [0.0, -4.0, 4.6])
        sim.setObjectOrientation(cam, [math.radians(-132), 0.0, math.radians(180)])
        print("Default camera view aligned to square arena.")
    except Exception as e:
        print(f"Note: Camera adjustment skipped ({e})")

    # 8. Save the scene
    sim.saveScene(SCENE_SAVE_PATH)
    print(f"\n=======================================================")
    print(f"SUCCESS: Clean, open Square Arena saved to:")
    print(f"  {SCENE_SAVE_PATH}")
    print(f"=======================================================")

if __name__ == "__main__":
    build_scene()
