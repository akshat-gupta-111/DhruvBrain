#!/bin/bash
# =============================================================================
#  DHRUV NAVIGATION & SLAM — Unified Multi-Sensor Entrypoint
#  Hardware: NVIDIA Jetson Orin Nano | ROS 2 Humble
#  Sensors: RPLiDAR A1 (1.6m) + 6-Axis IMU + Monocular Camera (VO) + RF2O
#  Fusion: robot_localization EKF (NO wheel encoders needed)
# =============================================================================

source /opt/ros/humble/setup.bash
source /workspace/rf2o_ws/install/setup.bash 2>/dev/null || true

echo ""
echo "============================================================"
echo "  DHRUV AUTONOMOUS NAVIGATION & SLAM STARTING UP..."
echo "  Multi-Sensor EKF Fusion Stack (LiDAR + IMU + Monocular VO)"
echo "============================================================"
echo ""

# Graceful shutdown handler
cleanup() {
    echo ""
    echo "[SHUTDOWN] Stopping all navigation and SLAM processes..."
    kill "$NAV2_PID" "$SLAM_PID" "$AMCL_PID" "$MAP_SERVER_PID" "$BRIDGE_PID" \
         "$EKF_PID" "$RF2O_PID" "$VO_PID" "$IMU_PID" "$LIDAR_PID" \
         "$TF_LASER_PID" "$TF_IMU_PID" "$TF_CAM_PID" "$TF_FOOTPRINT_PID" 2>/dev/null || true
    wait 2>/dev/null || true
    echo "[SHUTDOWN] Clean exit complete."
}
trap cleanup EXIT INT TERM

# -----------------------------------------------------------------------------
# STEP 1 — Static TF Publishers
# -----------------------------------------------------------------------------
echo "[1/8] Broadcasting robot static transforms..."

# base_link -> laser (LiDAR mounted 1.6m above ground center, PHYSICALLY FACING BACKWARDS)
# Confirmed by scan test: hand in front of robot appeared at 162 deg -> 180 deg yaw correction.
# ROS 2 quaternion format: x y z  qx qy qz qw  parent child
# 180 deg yaw = quaternion (qx=0, qy=0, qz=1.0, qw=0.0)
ros2 run tf2_ros static_transform_publisher 0 0 1.6 0 0 1.0 0.0 base_link laser >/dev/null 2>&1 &
TF_LASER_PID=$!

# base_link -> imu_link (IMU mounted near center 10cm up, identity orientation)
ros2 run tf2_ros static_transform_publisher 0 0 0.1 0 0 0 1.0 base_link imu_link >/dev/null 2>&1 &
TF_IMU_PID=$!

# base_link -> camera_link (Camera mounted 20cm forward, 85cm up, identity orientation)
ros2 run tf2_ros static_transform_publisher 0.2 0 0.85 0 0 0 1.0 base_link camera_link >/dev/null 2>&1 &
TF_CAM_PID=$!

# base_link -> base_footprint (Ground plane projection, identity orientation)
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 1.0 base_link base_footprint >/dev/null 2>&1 &
TF_FOOTPRINT_PID=$!

sleep 1

# -----------------------------------------------------------------------------
# STEP 2 — RPLidar A1 (2D LiDAR on /scan)
# -----------------------------------------------------------------------------
echo "[2/8] Initializing RPLiDAR A1..."

LIDAR_PORT=""
for p in /dev/ttyUSB0 /dev/ttyUSB1 /dev/ttyUSB2; do
    if [ -e "$p" ]; then
        LIDAR_PORT="$p"
        break
    fi
done

if [ -z "$LIDAR_PORT" ] && [ -d "/dev/serial/by-id" ]; then
    BY_ID=$(ls /dev/serial/by-id/*Silicon_Labs* /dev/serial/by-id/*CP210* 2>/dev/null | head -n 1 || true)
    if [ -n "$BY_ID" ]; then
        LIDAR_PORT=$(realpath "$BY_ID" 2>/dev/null || readlink -f "$BY_ID" || echo "$BY_ID")
    fi
fi
LIDAR_PORT=${LIDAR_PORT:-"/dev/ttyUSB0"}

chmod 666 "$LIDAR_PORT" 2>/dev/null || true
chmod 666 /dev/ttyACM* 2>/dev/null || true
chmod 666 /dev/video* 2>/dev/null || true

# Soft-reset RPLidar via serial command
python3 -c "
import serial, time
try:
    s = serial.Serial('$LIDAR_PORT', 115200, timeout=0.5)
    s.write(b'\xa5\x25')
    s.flush()
    time.sleep(0.3)
    s.reset_input_buffer()
    s.close()
except:
    pass
" 2>/dev/null || true

echo "      LiDAR serial port: $LIDAR_PORT"
ros2 launch rplidar_ros rplidar_a1_launch.py \
    serial_port:="$LIDAR_PORT" \
    serial_baudrate:=115200 \
    scan_mode:=Standard >/tmp/lidar.log 2>&1 &
LIDAR_PID=$!

# Wait for /scan topic
echo "      Waiting for /scan topic..."
MAX_WAIT=20
WAITED=0
until ros2 topic list 2>/dev/null | grep -q "^/scan$"; do
    sleep 1
    WAITED=$((WAITED + 1))
    if [ "$WAITED" -ge "$MAX_WAIT" ]; then
        echo "      [WARN] /scan delayed; continuing startup..."
        break
    fi
done
[ "$WAITED" -lt "$MAX_WAIT" ] && echo "      /scan is active."

# -----------------------------------------------------------------------------
# STEP 3 — 6-Axis IMU Driver & Attitude Filter
# -----------------------------------------------------------------------------
echo "[3/8] Starting 6-Axis IMU Driver (/imu/data)..."
IMU_SCRIPT="/workspace/imu_node.py"
[ -f /workspace/src_live/imu_node.py ] && IMU_SCRIPT="/workspace/src_live/imu_node.py"

python3 "$IMU_SCRIPT" >/tmp/imu.log 2>&1 &
IMU_PID=$!

# -----------------------------------------------------------------------------
# STEP 4 — Monocular Camera & Visual Odometry Node
# -----------------------------------------------------------------------------
echo "[4/8] Starting Monocular Visual Odometry (/vo/odom)..."
VO_SCRIPT="/workspace/monocular_vo_node.py"
[ -f /workspace/src_live/monocular_vo_node.py ] && VO_SCRIPT="/workspace/monocular_vo_node.py"

# If a video device is present (/dev/video0), start VO directly with device capture
if [ -e /dev/video0 ]; then
    echo "      Found camera device at /dev/video0"
    python3 "$VO_SCRIPT" --ros-args -p use_device_capture:=True -p camera_device:=/dev/video0 >/tmp/vo.log 2>&1 &
else
    echo "      No /dev/video0 detected — VO waiting on /camera/image_raw topic"
    python3 "$VO_SCRIPT" --ros-args -p use_device_capture:=False >/tmp/vo.log 2>&1 &
fi
VO_PID=$!

# -----------------------------------------------------------------------------
# STEP 5 — RF2O Laser Odometry (/scan -> /rf2o/odom, publish_tf:=false)
# -----------------------------------------------------------------------------
echo "[5/8] Starting RF2O Laser Odometry..."
RF2O_LAUNCH="/workspace/rf2o.launch.py"
[ -f /workspace/src_live/rf2o.launch.py ] && RF2O_LAUNCH="/workspace/src_live/rf2o.launch.py"

ros2 launch "$RF2O_LAUNCH" >/tmp/rf2o.log 2>&1 &
RF2O_PID=$!

sleep 3

# -----------------------------------------------------------------------------
# STEP 6 — Extended Kalman Filter (robot_localization EKF -> odom TF)
# -----------------------------------------------------------------------------
echo "[6/8] Starting Extended Kalman Filter (odom -> base_link TF & /odometry/filtered)..."
EKF_CONFIG="/workspace/ekf.yaml"
[ -f /workspace/src_live/ekf.yaml ] && EKF_CONFIG="/workspace/src_live/ekf.yaml"

ros2 run robot_localization ekf_node --ros-args --params-file "$EKF_CONFIG" >/tmp/ekf.log 2>&1 &
EKF_PID=$!

sleep 3

# -----------------------------------------------------------------------------
# STEP 7 — SLAM Toolbox OR Map Localization
# -----------------------------------------------------------------------------
NAV_MODE="mapping"
ACTIVE_MAP=""
if [ -f "/root/default_mode.txt" ]; then
    source "/root/default_mode.txt" 2>/dev/null || true
fi

if [ "$NAV_MODE" = "localization" ] && [ -n "$ACTIVE_MAP" ] && [ -f "/root/${ACTIVE_MAP}_map.yaml" ]; then
    echo "[7/8] Starting AMCL Localization on map: $ACTIVE_MAP..."
    MAP_YAML="/root/${ACTIVE_MAP}_map.yaml"
    NAV2_PARAMS="/workspace/nav2_params.yaml"
    [ -f /workspace/src_live/nav2_params.yaml ] && NAV2_PARAMS="/workspace/src_live/nav2_params.yaml"

    ros2 run nav2_map_server map_server --ros-args -p yaml_filename:="$MAP_YAML" -p use_sim_time:=false >/tmp/map_server.log 2>&1 &
    MAP_SERVER_PID=$!
    sleep 2
    ros2 run nav2_amcl amcl --ros-args --params-file "$NAV2_PARAMS" -p use_sim_time:=false >/tmp/amcl.log 2>&1 &
    AMCL_PID=$!
    sleep 1
    ros2 lifecycle set /map_server configure >/dev/null 2>&1 || true
    ros2 lifecycle set /map_server activate >/dev/null 2>&1 || true
    ros2 lifecycle set /amcl configure >/dev/null 2>&1 || true
    ros2 lifecycle set /amcl activate >/dev/null 2>&1 || true
else
    echo "[7/8] Starting SLAM Toolbox (Dynamic Online Async SLAM)..."
    SLAM_PARAMS="/workspace/slam_toolbox_params.yaml"
    [ -f /workspace/src_live/slam_toolbox_params.yaml ] && SLAM_PARAMS="/workspace/slam_toolbox_params.yaml"
    ros2 launch slam_toolbox online_async_launch.py params_file:="$SLAM_PARAMS" use_sim_time:=false >/tmp/slam.log 2>&1 &
    SLAM_PID=$!
fi

sleep 4

# -----------------------------------------------------------------------------
# STEP 8 — ROS-Arduino Bridge & Nav2 Stack
# -----------------------------------------------------------------------------
echo "[8/8] Starting ROS-Arduino Bridge and Nav2..."
BRIDGE_SCRIPT="/workspace/cmd_vel_bridge.py"
[ -f /workspace/src_live/cmd_vel_bridge.py ] && BRIDGE_SCRIPT="/workspace/cmd_vel_bridge.py"
python3 "$BRIDGE_SCRIPT" >/tmp/bridge.log 2>&1 &
BRIDGE_PID=$!

NAV2_PARAMS="/workspace/nav2_params.yaml"
[ -f /workspace/src_live/nav2_params.yaml ] && NAV2_PARAMS="/workspace/nav2_params.yaml"
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=false params_file:="$NAV2_PARAMS" >/tmp/nav2.log 2>&1 &
NAV2_PID=$!

echo ""
echo "============================================================"
echo "  All Nodes Initialized."
echo "  Web UI & Map Stream: http://<JETSON_IP>:8000"
echo "  System Health Check: http://<JETSON_IP>:8000/status"
echo "============================================================"
echo ""

# Launch Interactive Robot Manager in foreground
MGR_SCRIPT="/workspace/robot_manager.py"
[ -f /workspace/src_live/robot_manager.py ] && MGR_SCRIPT="/workspace/src_live/robot_manager.py"
exec python3 "$MGR_SCRIPT"
