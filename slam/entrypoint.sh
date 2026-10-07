#!/bin/bash
# =============================================================================
#  DHRUV SLAM — Unified Entrypoint
#  Launches all ROS 2 nodes in the correct startup order inside one container.
#  Replaces the old "4-terminal" manual workflow.
# =============================================================================

set -e

# Source ROS 2 and RF2O workspaces on every shell spawned from this script
source /opt/ros/humble/setup.bash
source /workspace/rf2o_ws/install/setup.bash

echo ""
echo "============================================================"
echo "  DHRUV SLAM — Unified Startup"
echo "  Hardware: NVIDIA Jetson Orin Nano | RPLidar A1 | ROS 2 Humble"
echo "============================================================"
echo ""

# -----------------------------------------------------------------------------
# STEP 1: Static TF Publishers (no dependencies, start immediately)
# -----------------------------------------------------------------------------
# Link 'base_link' → 'laser'  (LiDAR mounted 1.6 m above robot centre)
echo "[TF] Publishing base_link → laser (height: 1.6 m)"
ros2 run tf2_ros static_transform_publisher 0 0 1.6 0 0 0 base_link laser >/dev/null 2>&1 &
TF_LASER_PID=$!

# Link 'base_link' → 'base_footprint'  (prevents "Failed to compute odom pose")
echo "[TF] Publishing base_link → base_footprint"
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link base_footprint >/dev/null 2>&1 &
TF_FOOTPRINT_PID=$!

sleep 1

# -----------------------------------------------------------------------------
# STEP 2: RPLidar A1 Node  (must be first — everyone subscribes to /scan)
# -----------------------------------------------------------------------------
echo "[LIDAR] Starting RPLidar A1 node..."
ros2 launch rplidar_ros rplidar_a1_launch.py >/dev/null 2>&1 &
LIDAR_PID=$!

# Wait until /scan is actively publishing before moving on
echo "[LIDAR] Waiting for /scan topic to become active..."
MAX_WAIT=30
WAITED=0
until ros2 topic list 2>/dev/null | grep -q "^/scan$"; do
    sleep 1
    WAITED=$((WAITED + 1))
    if [ "$WAITED" -ge "$MAX_WAIT" ]; then
        echo "[ERROR] /scan topic did not appear after ${MAX_WAIT}s. Is the RPLidar connected?"
        exit 1
    fi
done
echo "[LIDAR] / /scan is live."

# Give LiDAR a moment to warm up and publish stable data
sleep 2

# -----------------------------------------------------------------------------
# STEP 3: RF2O Laser Odometry  (/scan → /odom)
# -----------------------------------------------------------------------------
echo "[RF2O] Starting RF2O laser odometry (/scan → /odom)..."
ros2 launch rf2o_laser_odometry rf2o_laser_odometry.launch.py \
    laser_scan_topic:=/scan \
    odom_topic:=/odom \
    publish_tf:=true \
    base_frame_id:=base_link \
    odom_frame_id:=odom >/dev/null 2>&1 &
RF2O_PID=$!

# Wait until /odom is publishing before starting SLAM
echo "[RF2O] Waiting for /odom topic to become active..."
WAITED=0
until ros2 topic list 2>/dev/null | grep -q "^/odom$"; do
    sleep 1
    WAITED=$((WAITED + 1))
    if [ "$WAITED" -ge "$MAX_WAIT" ]; then
        echo "[ERROR] /odom topic did not appear after ${MAX_WAIT}s."
        exit 1
    fi
done
echo "[RF2O] /odom is live."

sleep 1

# -----------------------------------------------------------------------------
# STEP 4: SLAM Toolbox  (needs /scan + /odom)
# -----------------------------------------------------------------------------
echo "[SLAM] Starting slam_toolbox (online async mode)..."
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=false >/dev/null 2>&1 &
SLAM_PID=$!

# Wait until /map starts publishing
echo "[SLAM] Waiting for /map topic to become active..."
WAITED=0
until ros2 topic list 2>/dev/null | grep -q "^/map$"; do
    sleep 1
    WAITED=$((WAITED + 1))
    if [ "$WAITED" -ge 60 ]; then
        echo "[WARN] /map not yet published after 60s — SLAM toolbox may still be initialising."
        echo "[WARN] Continuing anyway. The web viewer will show data once the map appears."
        break
    fi
done
[ "$WAITED" -lt 60 ] && echo "[SLAM] /map is live."

# -----------------------------------------------------------------------------
# STEP 5: Robot Manager + Web Server  (interactive menu + http://<IP>:8000)
# -----------------------------------------------------------------------------
echo ""
echo "============================================================"
echo "  All background ROS nodes are running."
echo "  Open the live map in your browser: http://<JETSON_IP>:8000"
echo "============================================================"
echo ""

# Graceful shutdown: if robot_manager exits (user types 5), kill all children
cleanup() {
    echo ""
    echo "[SHUTDOWN] Stopping all SLAM nodes..."
    kill "$SLAM_PID" "$RF2O_PID" "$LIDAR_PID" "$TF_LASER_PID" "$TF_FOOTPRINT_PID" 2>/dev/null || true
    wait 2>/dev/null || true
    echo "[SHUTDOWN] All nodes stopped. Goodbye."
}
trap cleanup EXIT INT TERM

# Run robot_manager in the foreground so the container stays alive and the
# interactive menu is available via `docker attach` or `docker compose exec`.
exec python3 /workspace/robot_manager.py
