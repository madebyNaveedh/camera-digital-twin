# Calibration and Coordinate Frames Documentation

## 1. Coordinate Frame Definitions

The system defines 5 interconnected coordinate frames:

| Frame Name | Origin | +X Axis | +Y Axis | +Z Axis |
| :--- | :--- | :--- | :--- | :--- |
| **LiDAR Sensor Frame** | Center of spinning mirror | Forward ($0^\circ$) | Left ($90^\circ$) | Upwards along rotation axis |
| **Camera Frame** | Optical center of lens | Right | Downwards | Forward along optical axis |
| **Physical Room Frame** | Room floor center $(0,0,0)$ | East / Room Width | North / Room Length | Vertical Upwards |
| **CoppeliaSim World Frame** | Simulation scene center | Aligned with Room +X | Aligned with Room +Y | Aligned with Room +Z |
| **Robot Ego Frame** | Ground contact center between drive wheels | Forward driving direction | Left wheel axis | Vertical Upwards |

---

## 2. Mathematical Transformations

### 2.1 Polar to LiDAR Cartesian
Given measurement angle $\theta \in [0, 360^\circ)$ and range $r \in \mathbb{R}^+$:
$$x_s = r \cos(\theta)$$
$$y_s = r \sin(\theta)$$

### 2.2 LiDAR Frame to Physical Room Frame
For planar sensor mounting with position offset $(t_x, t_y, t_z)$ and yaw mounting angle $\psi_{mount}$:
$$\begin{bmatrix} x_{room} \\ y_{room} \end{bmatrix} = \begin{bmatrix} \cos(\psi_{mount}) & -\sin(\psi_{mount}) \\ \sin(\psi_{mount}) & \cos(\psi_{mount}) \end{bmatrix} \begin{bmatrix} x_s \\ y_s \end{bmatrix} + \begin{bmatrix} t_x \\ t_y \end{bmatrix}$$
$$z_{room} = t_z$$

### 2.3 Room Frame to Robot Ego Frame
Given robot position $(x_{rob}, y_{rob})$ and heading $\theta_{rob}$ in the Room frame:
$$\begin{bmatrix} x_{ego} \\ y_{ego} \end{bmatrix} = \begin{bmatrix} \cos(\theta_{rob}) & \sin(\theta_{rob}) \\ -\sin(\theta_{rob}) & \cos(\theta_{rob}) \end{bmatrix} \begin{bmatrix} x_{room} - x_{rob} \\ y_{room} - y_{rob} \end{bmatrix}$$

---

## 3. Calibration Procedure

1. **Mounting Measurement:**
   - Measure the distance from the room reference center $(0,0)$ to the LiDAR base.
   - Enter values in `config/sensors.yaml` under `lidar.mounting.x` and `lidar.mounting.y`.
   - Measure mounting height above floor and set `lidar.mounting.z`.
2. **Orientation Alignment:**
   - Align the LiDAR $0^\circ$ index mark with the room's positive X axis. If angled, record the offset in `lidar.mounting.yaw_deg`.
3. **Verification with Ground-Truth Target:**
   - Place a known test cylinder (e.g. 0.20m radius) at a known surveyed position $(1.00\text{ m}, 0.00\text{ m})$.
   - Run the inspection or unit tests:
     ```bash
     python3 tests/test_transforms.py
     ```
   - Verify that the estimated centroid reported by the digital twin matches the ground-truth target within $\pm 0.05$ m.
