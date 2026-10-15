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
    tail -n 20 /tmp/bridge.log
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
echo "--- [4/5] ROS 2 TOPICS & TF CHECK ---"
echo "Active Topics:"
ros2 topic list 2>/dev/null | grep -E "scan|odom|map|cmd_vel"

echo ""
echo "--- [5/5] DIRECT MOTOR HARDWARE TEST ---"
echo "Attempting to send a 0.5s test command directly to Arduino..."
# Find the active Arduino port
ARDUINO_PORT=""
for p in /dev/ttyACM0 /dev/ttyACM1 /dev/ttyACM2 /dev/ttyUSB1 /dev/ttyUSB2; do
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
    print('  [SUCCESS] Sent <FWD,120,500> to $ARDUINO_PORT. Did the wheels nudge?')
    s.close()
except Exception as e:
    print('  [FAILED] Direct test error:', e)
"
else
    echo "  [ERROR] No Arduino port (/dev/ttyACM* or /dev/ttyUSB1+) detected!"
fi

echo ""
echo "============================================================"
echo "Diagnostics complete."
echo "============================================================"
