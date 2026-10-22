# DHRUV AUTONOMOUS NAVIGATION & SLAM (ENCODER-FREE)

An integrated, multi-sensor SLAM and autonomous navigation stack designed for **Dhruv Robot** on the **NVIDIA Jetson Orin Nano (JetPack 6.x / ROS 2 Humble)** without requiring motor/wheel encoders.

---

## 1. System Architecture

```
                    ┌──────────────┐
                    │ 2D RPLiDAR   │ ──> /scan ───────────> [RF2O Laser Odometry] ──> /rf2o/odom (Twist)
                    └──────────────┘                                                          │
                    ┌──────────────┐                                                          ▼
                    │  6-Axis IMU  │ ──> [Madgwick AHRS] ─> /imu/data (Yaw Rate + Ori) ───> [robot_localization EKF]
                    └──────────────┘                                                          ▲   (Publishes odom -> base_link TF
                    ┌──────────────┐                                                          │    & /odometry/filtered)
                    │  Mono Camera │ ──> /camera/image_raw ─> [Monocular VO] ──> /vo/odom ────┘
                    └──────────────┘
                                                                │
                                                                ▼
                     ┌───────────────────────────────────────────────────────────────────────┐
                     │                           SLAM & NAVIGATION                           │
                     ├───────────────────────────────────┬───────────────────────────────────┤
                     │ Phase A: Dynamic Room Mapping     │ Phase B: Static Map Navigation    │
                     │ [slam_toolbox (Online Async)]     │ [nav2_amcl Localization]          │
                     │  - Subscribes to /scan + EKF      │  - Subscribes to /map + /scan     │
                     │  - Computes Ceres loop closures   │  - Particle-filter localization   │
                     │  - Publishes map -> odom TF       │  - Publishes map -> odom TF       │
                     └───────────────────────────────────┴───────────────────────────────────┘
                                                         │
                                                         ▼
                                                [Nav2 Navigation]
                                     - Smac / NavFn Global Planner
                                     - Dynamic Obstacle Raytrace Clearing (Local Costmap)
                                     - DWB / MPPI Mecanum Local Controller
                                                         │
                                                         ▼
                                            [/cmd_vel & cmd_vel_bridge]
                                                         │
                                                         ▼
                                           [Arduino Mecanum Drive Motors]
```

---

## 2. Hardware Connections & Port Mapping

| Hardware Sensor | Default Port | Mount Position | Topic |
| :--- | :--- | :--- | :--- |
| **RPLiDAR A1** | `/dev/ttyUSB0` | $x=0.0\text{m}, y=0.0\text{m}, z=1.60\text{m}$ | `/scan` |
| **6-Axis IMU** (MPU6050/WT901/BNO) | `/dev/ttyUSB1` | $x=0.0\text{m}, y=0.0\text{m}, z=0.10\text{m}$ | `/imu/data` |
| **Monocular Camera** (USB / CSI) | `/dev/video0` | $x=0.20\text{m}, y=0.0\text{m}, z=0.85\text{m}$ | `/camera/image_raw` & `/vo/odom` |
| **Arduino Motor Controller** | `/dev/ttyACM0` | Robot chassis | `<MIX, vx, vy, w, 0>` |

---

## 3. Quick Start (Single Docker Command)

### Step 1: Clone or navigate to the `nav` directory
```bash
cd ~/DhruvBrain/nav
```

### Step 2: Build and start the container
```bash
sudo docker compose up --build
```
All static transforms, LiDAR, IMU, Monocular VO, RF2O, EKF, SLAM Toolbox, Bridge, and Nav2 will initialize automatically in sequence.

### Step 3: Access Live Web Dashboard & Telemetry
Open your browser on any laptop or phone connected to the same Wi-Fi:
```
http://<JETSON_IP>:8000
```
- Real-time occupancy grid map.
- Live robot pose and path trail.
- Sensor health monitor: LiDAR, IMU, VO, EKF, and SLAM freshness.
- Buttons for: **Status**, **Set Checkpoint**, **Save Map**, **Navigate To**, and **Emergency Stop**.

### Step 4: Attach to Interactive Terminal CLI
```bash
sudo docker attach dhruv-nav
```
*(To detach safely without stopping the container, press `Ctrl-P` followed by `Ctrl-Q`)*

---

## 4. Operational Workflows

### Mode A: Mapping a New Room / Floor
1. Open the Web UI at `http://<JETSON_IP>:8000` (or use the interactive CLI).
2. Teleoperate or nudge the robot around the room.
3. Observe the occupancy grid expanding in real-time.
4. When at an important location (e.g., *lab_desk*, *entrance*, *charging_dock*):
   * Click **Set Checkpoint** and name it.
5. When the room is mapped, click **Save Map** and enter a name (e.g. `office_floor1`).
   * Saves: `office_floor1_map.pgm`, `office_floor1_map.yaml`, and `office_floor1_checkpoints.json` in `/root/`.

### Mode B: Autonomous Navigation with Dynamic Obstacle Avoidance
1. From the Web UI, select the saved map and click **Load Map**.
2. The system switches from `slam_toolbox` to `nav2_amcl` (Monte Carlo Particle Localization).
3. Select any saved checkpoint from the dropdown and click **Go!**.
4. Nav2 plans the global path. If people or dynamic obstacles step into the path:
   * The 2D LiDAR raytracing costmap layer marks the obstacle immediately.
   * When the obstacle moves away, `clearing: True` flushes the occupied cells within 0.2 seconds.
   * The local controller safely steers around the obstacle or halts if blocked.

---

## 5. Sensor Diagnostics & Calibration

To test all sensors in isolation without launching the full SLAM stack, run the diagnostic script:

```bash
docker exec -it dhruv-nav python3 /workspace/test_sensors.py
```
This tests:
* RPLiDAR scan rate (Hz) and distance verification.
* 6-Axis IMU yaw rate response ($\omega_z$) when spinning the robot.
* Monocular Visual Odometry tracking quality.
* EKF `/odometry/filtered` consistency and transform tree integrity (`map -> odom -> base_link`).

### Calibrating Camera Height for Visual Odometry
In [monocular_vo_node.py](file:///d:/Dhruv/DhruvBrain/nav/monocular_vo_node.py), ensure `camera_height_m` matches the actual ground clearance of your camera lens in meters:
```python
self.declare_parameter('camera_height_m', 0.85)
```
This ensures scale invariance on ground-plane optical flow without needing expensive stereo hardware.
