# System Architecture: Real-Time Digital Twin of a Physical Room

## 1. System Overview

The system establishes a closed-loop digital twin that continuously mirrors a physical laboratory room inside CoppeliaSim, enabling an autonomous mobile robot to plan and react in real time to physical environmental changes.

```mermaid
graph TD
    subgraph Physical_Layer["Physical Environment"]
        Sensors_Cam["Webcam (/dev/video0)"]
        Sensors_Lidar["LiDAR (RPLIDAR /dev/ttyUSB0)"]
        Real_Obs["Physical Obstacle (Box/Person)"]
        Real_Obs -->|Reflects Laser| Sensors_Lidar
        Real_Obs -->|Optical Photons| Sensors_Cam
    end

    subgraph Acquisition_Layer["Sensor Acquisition Layer"]
        ModA_Cam["Camera Interface<br/>(OpenCV Threaded V4L2)"]
        ModA_Lidar["LiDAR Interface<br/>(Serial Driver + Synthetic Fallback)"]
        Sensors_Cam --> ModA_Cam
        Sensors_Lidar --> ModA_Lidar
    end

    subgraph Perception_Layer["Perception & Transformation"]
        ModB_Proc["LiDAR Preprocessor<br/>(Bounds & Noise Filtering)"]
        ModC_Trans["Coordinate Transformer<br/>(LiDAR -> Room -> Sim)"]
        ModB_Detect["Obstacle Detector<br/>(Euclidean Clustering & Arc Compensation)"]
        ModA_Lidar --> ModB_Proc
        ModB_Proc --> ModC_Trans
        ModC_Trans --> ModB_Detect
    end

    subgraph Digital_Twin_Layer["Digital Twin Synchronization"]
        ModD_State["Environment State<br/>(Tracked Obstacle Registry)"]
        ModD_Sync["Synchronizer<br/>(Association, Low-Pass Filter, Timeout)"]
        ModB_Detect --> ModD_Sync
        ModD_Sync <--> ModD_State
    end

    subgraph Simulation_Layer["CoppeliaSim Simulation"]
        ModE_Client["Remote API Client (ZeroMQ)"]
        ModE_Scene["Scene Manager<br/>(Room Geometry, Primitives)"]
        ModF_Nav["Robot Controller<br/>(Artificial Potential Field)"]
        Sim_Robot["Pioneer P3DX Robot Model"]
        ModD_Sync -->|Update Objects| ModE_Scene
        ModE_Scene --> ModE_Client
        ModE_Client <--> Sim_Robot
        ModF_Nav -->|Target Wheel Speeds| ModE_Scene
    end

    subgraph Presentation_Layer["Telemetry & Dashboard"]
        ModG_Dash["Live Monitoring Dashboard<br/>(HTTP:8085 / Streamlit)"]
        ModA_Cam --> ModG_Dash
        ModD_Sync --> ModG_Dash
        ModF_Nav --> ModG_Dash
    end
```

---

## 2. Module Specifications

### Module A: Physical Sensor Acquisition
- **Camera Interface (`sensors/camera_interface.py`):** Threaded OpenCV capture over V4L2 (`/dev/video0`) capturing up to 1280x720 / 640x480 @ 30 FPS with non-blocking frame retrieval to prevent queue latency.
- **LiDAR Interface (`sensors/lidar_interface.py`):** Native pure Python driver decoding RPLIDAR A1/A2 packets over serial port (`/dev/ttyUSB0`) with transparent high-fidelity synthetic emulation fallback when disconnected.

### Module B & C: Perception and Coordinate Transformations
- **LiDAR Processing (`perception/lidar_processing.py`):** Filters non-finite points, range thresholds ($[0.15, 10.0]$ m), and clips perimeter wall boundaries to isolate free-space obstacles.
- **Coordinate Transformer (`calibration/coordinate_transforms.py`):** Rigid transformations from LiDAR polar $(r, \theta)$ to Cartesian sensor frame $(x_s, y_s)$, then to Physical Room frame $(x_{room}, y_{room})$, and into CoppeliaSim world coordinates.
- **Obstacle Detector (`perception/obstacle_detection.py`):** 2D Euclidean clustering with arc-center compensation ($p_{center} = p_{arc} + \frac{2}{\pi} R \hat{u}_{los}$) to recover true cylinder geometric centers from front-surface reflections.

### Module D: Digital-Twin State and Synchronizer
- **Environment State (`digital_twin/environment_state.py`):** Central repository for tracked obstacles with confidence, observation history, and lifecycle metrics.
- **Synchronizer (`digital_twin/synchronizer.py`):**
  - **Association:** Nearest-neighbor matching within $d_{assoc} = 0.45$ m.
  - **Filtering:** Exponential moving average position smoothing ($\alpha = 0.65$).
  - **Temporal Confirmation:** Requires $N=2$ consecutive scans before instantiating objects in CoppeliaSim, preventing sensor noise flicker.
  - **Obstacle Pruning:** Retains unobserved obstacles for a persistence timeout ($t_{timeout} = 2.0$ s) to prevent premature deletion during beam dropouts.

### Module E & F: Simulation and Autonomous Navigation
- **Scene Manager (`simulation/scene_manager.py`):** Programmatically constructs walls, floor, goal marker, and loads Pioneer P3DX robot. Dynamically creates/updates obstacle cylinders via CoppeliaSim ZeroMQ API.
- **Robot Controller (`navigation/robot_controller.py`):** Differential-drive kinematics combined with Artificial Potential Field (APF) guidance. Generates attractive forces towards goal and repulsive deflection away from physical obstacles. Stops when obstacles are within critical stop distance ($0.38$ m).

### Module G: Monitoring Dashboard
- **Web Dashboard (`dashboard/app.py`):** Pure-Python HTTP server providing real-time MJPEG camera stream, 2D interactive radar canvas, obstacle list, robot telemetry, and interactive controls on `http://localhost:8085`.
