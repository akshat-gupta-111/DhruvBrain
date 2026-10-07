#!/bin/bash
# =============================================================================
#  DHRUV SLAM — Unified Entrypoint
#  Launches all ROS 2 nodes in the correct startup order inside one container.
#  Replaces the old "4-terminal" manual workflow.
# =============================================================================

# Do NOT use set -e here — a transient node hiccup must not kill the container.

# Source ROS 2 and RF2O workspaces
source /opt/ros/humble/setup.bash
source /workspace/rf2o_ws/install/setup.bash

echo ""
echo "============================================================"
echo "  DHRUV SLAM — Starting up..."
echo "  Hardware: NVIDIA Jetson Orin Nano | RPLidar A1 | ROS 2 Humble"
echo "============================================================"
echo ""

# -----------------------------------------------------------------------------
# Graceful shutdown: kill all background children when this script exits
# -----------------------------------------------------------------------------
cleanup() {
    echo ""
    echo "[SHUTDOWN] Stopping all SLAM nodes..."
    # Kill by PID if we have them, fall back to killall for safety
    kill "$SLAM_PID" "$RF2O_PID" "$LIDAR_PID" "$TF_LASER_PID" "$TF_FOOTPRINT_PID" 2>/dev/null || true
    wait 2>/dev/null || true
    echo "[SHUTDOWN] All nodes stopped. Goodbye."
}
trap cleanup EXIT INT TERM

# -----------------------------------------------------------------------------
# STEP 1 — Static TF Publishers (no dependencies)
# -----------------------------------------------------------------------------
echo "[1/4] Publishing static TF frames..."

# base_link → laser  (LiDAR mounted 1.6 m above robot centre)
ros2 run tf2_ros static_transform_publisher 0 0 1.6 0 0 0 base_link laser >/dev/null 2>&1 &
TF_LASER_PID=$!

# base_link → base_footprint  (prevents "Failed to compute odom pose" in SLAM)
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link base_footprint >/dev/null 2>&1 &
TF_FOOTPRINT_PID=$!

sleep 2

# -----------------------------------------------------------------------------
# STEP 2 — RPLidar A1  (must be first — everything subscribes to /scan)
# -----------------------------------------------------------------------------
echo "[2/4] Starting RPLidar A1 node..."
ros2 launch rplidar_ros rplidar_a1_launch.py >/dev/null 2>&1 &
LIDAR_PID=$!

# /scan is the only HARD dependency — no LiDAR means nothing works.
echo "      Waiting for /scan topic (up to 30 s)..."
MAX_WAIT=30
WAITED=0
until ros2 topic list 2>/dev/null | grep -q "^/scan$"; do
    sleep 1
    WAITED=$((WAITED + 1))
    if [ "$WAITED" -ge "$MAX_WAIT" ]; then
        echo ""
        echo "[ERROR] /scan did not appear after ${MAX_WAIT}s."
        echo "        Is the RPLidar plugged in? Check: ls /dev/ttyUSB*"
        echo "        Container will keep running so you can debug."
        # Drop straight into robot_manager — user can investigate
        break
    fi
done
[ "$WAITED" -lt "$MAX_WAIT" ] && echo "      /scan is live."

# Let LiDAR stabilise before RF2O tries to read it
sleep 3

# -----------------------------------------------------------------------------
# STEP 3 — RF2O Laser Odometry  (/scan → /odom)
# -----------------------------------------------------------------------------
echo "[3/4] Starting RF2O odometry (/scan → /odom)..."
ros2 launch rf2o_laser_odometry rf2o_laser_odometry.launch.py \
    laser_scan_topic:=/scan \
    odom_topic:=/odom \
    publish_tf:=true \
    base_frame_id:=base_link \
    odom_frame_id:=odom >/tmp/rf2o.log 2>&1 &
RF2O_PID=$!

# RF2O needs a few seconds to read scan data and start publishing odom.
# Fixed delay is more reliable than topic polling for this node.
echo "      Allowing RF2O 8 s to initialise..."
sleep 8

# -----------------------------------------------------------------------------
# STEP 4 — SLAM Toolbox  (needs /scan + /odom)
# -----------------------------------------------------------------------------
echo "[4/4] Starting SLAM Toolbox (online async)..."
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=false >/tmp/slam.log 2>&1 &
SLAM_PID=$!

# SLAM toolbox takes ~5-10 s to initialise Ceres solver
echo "      Allowing SLAM Toolbox 10 s to initialise..."
sleep 10

# -----------------------------------------------------------------------------
# Ready — hand over to the interactive Robot Manager
# -----------------------------------------------------------------------------
echo ""
echo "============================================================"
echo "  All SLAM nodes launched."
echo "  Live map browser: http://<JETSON_IP>:8000"
echo "  Diagnostics:      http://<JETSON_IP>:8000/status"
echo ""
echo "  If /map never appears, check logs:"
echo "    RF2O log:  tail -f /tmp/rf2o.log"
echo "    SLAM log:  tail -f /tmp/slam.log"
echo "============================================================"
echo ""

# exec replaces the shell with Python so signals are forwarded correctly
if [ -f /workspace/src_live/robot_manager.py ]; then
    exec python3 /workspace/src_live/robot_manager.py
else
    exec python3 /workspace/robot_manager.py
fi
