#!/bin/bash
# =============================================================================
#  DHRUV SLAM — Navigation & Motor Diagnostics Tool
#  Run this inside the container or host to isolate why the robot won't move.
#  Usage:
#     sudo docker exec -it dhruv-slam bash /workspace/src_live/diagnose_navigation.sh
# =============================================================================

source /opt/ros/humble/setup.bash 2>/dev/null || true

echo "============================================================"
echo "  DHRUV NAVIGATION DIAGNOSTICS"
echo "============================================================"

echo ""
echo "--- [1/5] SERIAL & ARDUINO HARDWARE CHECK ---"
echo "Available serial devices:"
ls -l /dev/ttyACM* /dev/ttyUSB* 2>/dev/null || echo "No /dev/ttyACM* or /dev/ttyUSB* found!"

echo ""
echo "--- [2/5] CMD_VEL_BRIDGE LOGS (/tmp/bridge.log) ---"
if [ -f /tmp/bridge.log ]; then
    tail -n 15 /tmp/bridge.log
else
    echo "No /tmp/bridge.log found."
fi

echo ""
echo "--- [3/5] NAV2 LOGS (/tmp/nav2.log - last errors/warnings) ---"
if [ -f /tmp/nav2.log ]; then
    grep -E "WARN|ERROR|abort|out of map bounds|Costmap" /tmp/nav2.log | tail -n 25
else
    echo "No /tmp/nav2.log found."
fi

echo ""
echo "--- [4/5] ROS 2 TOPICS & PUBLISHER/SUBSCRIBER WIRING ---"
echo "Active Topics:"
ros2 topic list 2>/dev/null | grep -E "scan|odom|map|cmd_vel"

echo ""
echo "Topic details:"
echo -n "  /cmd_vel publishers: "
ros2 topic info /cmd_vel 2>/dev/null | grep "Publisher count" || echo "not found"
echo -n "  /cmd_vel subscribers: "
ros2 topic info /cmd_vel 2>/dev/null | grep "Subscription count" || echo "not found"
echo -n "  /cmd_vel_nav publishers: "
ros2 topic info /cmd_vel_nav 2>/dev/null | grep "Publisher count" || echo "not found"

echo ""
echo "--- [5/5] MOTOR TEST (SAFE DUAL-MODE) ---"
BRIDGE_PID=$(pgrep -f "cmd_vel_bridge" || true)

if [ -n "$BRIDGE_PID" ]; then
    echo "cmd_vel_bridge is ACTIVE (PID $BRIDGE_PID, holding /dev/ttyACM0 open)."
    echo "Testing movement via ROS 2 topic (/cmd_vel) to avoid serial port conflict..."
    ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.25, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" >/dev/null 2>&1 &
    sleep 0.8
    echo "Bridge log response:"
    tail -n 3 /tmp/bridge.log 2>/dev/null || true
    echo "--> Did the wheels nudge forward during this test?"
else
    echo "cmd_vel_bridge is NOT running. Testing direct serial access..."
    ARDUINO_PORT=""
    for p in /dev/ttyACM0 /dev/ttyACM1 /dev/ttyACM2; do
        if [ -e "$p" ]; then
            ARDUINO_PORT="$p"
            break
        fi
    done

    if [ -n "$ARDUINO_PORT" ]; then
        echo "Testing direct serial write to $ARDUINO_PORT at 115200 baud..."
        python3 -c "
import serial, time
try:
    s = serial.Serial('$ARDUINO_PORT', 115200, timeout=1.0)
    time.sleep(1.0)
    s.write(b'MODE:AUTO\n')
    s.flush()
    time.sleep(0.1)
    s.write(b'<FWD,120,500>\n')
    s.flush()
    print('  [SUCCESS] Sent <FWD,120,500> to $ARDUINO_PORT.')
    s.close()
except Exception as e:
    print('  [FAILED] Direct test error:', e)
"
    else
        echo "  [ERROR] No Arduino port detected!"
    fi
fi

echo ""
echo "--- [BONUS] KERNEL USB HEALTH (Check for USB Disconnects / Brownouts) ---"
dmesg -T 2>/dev/null | grep -iE "ttyACM|ttyUSB|disconnect|reset high-speed|under-voltage" | tail -n 8 || echo "No dmesg access inside container (run 'dmesg | tail -n 20' on host if needed)."

echo ""
echo "============================================================"
echo "Diagnostics complete."
echo "============================================================"
