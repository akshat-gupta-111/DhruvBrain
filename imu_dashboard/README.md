# Jetson Orin Robot Digital Twin Dashboard

A dark real-time dashboard served on **port 6769**. It shows a transparent cylinder representing the robot (66 cm tall, 22 cm diameter), a red IMU marker centered on the cylinder at 38 cm above the ground, live sensor values, orientation, and a terminal log stream.

This version targets the **MPU6050** on I²C. If your IMU is a different model, the register map and scaling must be changed.

## Important behavior

- No Kalman filter, complementary filter, or smoothing is used.
- Roll and pitch are calculated directly from accelerometer readings.
- Yaw is integrated from raw gyroscope Z-rate. It will drift over time because the MPU6050 has no absolute heading reference.
- The MPU6050 is mounted upside down. The server corrects that mounting by inverting the Y and Z accelerometer and gyroscope axes before calculating orientation.
- The visual follows the corrected IMU-derived roll, pitch, and yaw in real time.
- The dashboard is read-only and does not control robot motors.
- The browser loads Three.js and Socket.IO client from public CDNs, so the dashboard browser needs internet access unless those libraries are hosted locally.

## 1. Check Jetson I²C

Connect SDA, SCL, GND, and the correct sensor supply. Use the voltage level required by your breakout board. On Jetson, check which I²C bus is enabled and available:

```bash
ls /dev/i2c-*
sudo i2cdetect -y 1
```

The MPU6050 commonly appears at `0x68` (sometimes `0x69` if AD0 is high). Do not assume bus 1 is correct; check the device tree and wiring on your particular Jetson Orin carrier board.

## 2. Install and run

```bash
sudo apt update
sudo apt install -y python3-pip python3-venv i2c-tools
mkdir -p ~/robot-dashboard
cd ~/robot-dashboard
```

Copy this project into that directory, then:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
python app.py
```

Open on the Jetson itself at `http://localhost:6769`, or from another device on the same network at `http://JETSON_IP:6769`.

If I²C permission is denied, test once with `sudo` to confirm the cause, then configure the appropriate I²C device permissions for your system rather than running the web server as root permanently.

## 3. Configure bus/address

Defaults are I²C bus `7`, address `0x68`, and 50 samples/second. Override them if needed:

```bash
IMU_I2C_BUS=7 IMU_I2C_ADDRESS=0x69 IMU_SAMPLE_HZ=50 IMU_DASHBOARD_PORT=6769 python app.py
```

Use the actual bus number discovered on your Jetson. The address must match the result from `i2cdetect`.

## 4. Test the UI without hardware

```bash
IMU_SIMULATE=1 python app.py
```

Simulation is for UI testing only. Stop the process and run without `IMU_SIMULATE` to use the physical sensor.

## 5. Verify

On startup, the terminal should show an I²C bus/address and `WHO_AM_I` response, followed by `IMU connection ready`. If it reports an error, check the bus, address, sensor power, SDA/SCL, and I²C permissions.

## MPU6050 scaling assumptions

The code wakes the MPU6050 and assumes its default accelerometer range of ±2 g and gyroscope range of ±250 degrees/second. If other code changes these range registers, update the scaling factors in `app.py`.

The current orientation is intentionally simple:
- Upside-down mounting correction: `ay = -ay`, `az = -az`, `gy = -gy`, `gz = -gz`
- `roll = atan2(ay, az)`
- `pitch = atan2(-ax, sqrt(ay² + az²))`
- `yaw += gyro_z_rate × elapsed_time`

Accelerometer-derived roll/pitch are not reliable during strong linear acceleration. Gyroscope-integrated yaw accumulates bias and drift. These are limitations of raw IMU-only orientation, not dashboard synchronization.
