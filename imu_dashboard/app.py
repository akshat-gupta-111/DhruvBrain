import glob
import math
import os
import threading
import time
from datetime import datetime

from flask import Flask, render_template
from flask_socketio import SocketIO

try:
    from smbus2 import SMBus
except ImportError:
    SMBus = None

HOST = "0.0.0.0"
PORT = int(os.getenv("IMU_DASHBOARD_PORT", "6769"))
# Jetson 40-pin header: physical pin 3 = SDA, physical pin 5 = SCL.
# The Jetson kernel exposes that header bus to userspace as /dev/i2c-7.
JETSON_SDA_PIN = 3
JETSON_SCL_PIN = 5
JETSON_I2C_BUS = 7
I2C_BUS_PREFERENCE = os.getenv(
    "IMU_I2C_BUS", str(JETSON_I2C_BUS)
).strip().lower()
IMU_ADDRESS = int(os.getenv("IMU_I2C_ADDRESS", "0x68"), 0)
SAMPLE_HZ = max(1.0, float(os.getenv("IMU_SAMPLE_HZ", "50")))
SIMULATE = os.getenv("IMU_SIMULATE", "0").lower() in (
    "1", "true", "yes"
)

app = Flask(__name__)
app.config["SECRET_KEY"] = os.getenv(
    "DASHBOARD_SECRET", "robot-dashboard-local"
)

socketio = SocketIO(
    app,
    cors_allowed_origins="*",
    async_mode="threading"
)

stop_event = threading.Event()
sensor_thread_started = False
sensor_thread_lock = threading.Lock()
logs_lock = threading.Lock()

latest = {
    "connected": False,
    "source": "simulation" if SIMULATE else "I2C",
    "roll": 0.0,
    "pitch": 0.0,
    "yaw": 0.0,
    "ax": 0.0,
    "ay": 0.0,
    "az": 0.0,
    "gx": 0.0,
    "gy": 0.0,
    "gz": 0.0,
    "timestamp": None
}

logs = []
MAX_LOGS = 250


def log(message, level="INFO"):
    line = (
        f"[{datetime.now().strftime('%H:%M:%S.%f')[:-3]}] "
        f"[{level}] {message}"
    )

    with logs_lock:
        logs.append(line)
        del logs[:-MAX_LOGS]

    socketio.emit("log", {"line": line})


def signed_word(bus, register):
    high = bus.read_byte_data(IMU_ADDRESS, register)
    low = bus.read_byte_data(IMU_ADDRESS, register + 1)

    value = (high << 8) | low

    if value >= 32768:
        value -= 65536

    return value


def read_mpu6050(bus):
    raw_ax = signed_word(bus, 0x3B)
    raw_ay = signed_word(bus, 0x3D)
    raw_az = signed_word(bus, 0x3F)

    raw_gx = signed_word(bus, 0x43)
    raw_gy = signed_word(bus, 0x45)
    raw_gz = signed_word(bus, 0x47)

    ax = raw_ax / 16384.0
    ay = -(raw_ay / 16384.0)
    az = -(raw_az / 16384.0)

    gx = raw_gx / 131.0
    gy = -(raw_gy / 131.0)
    gz = -(raw_gz / 131.0)

    return ax, ay, az, gx, gy, gz


def discover_mpu6050_bus():
    if SMBus is None:
        raise RuntimeError(
            "smbus2 is not installed. Run: pip install smbus2"
        )

    if I2C_BUS_PREFERENCE == "auto":
        preferred_buses = [JETSON_I2C_BUS]
    else:
        try:
            preferred_buses = [
                int(I2C_BUS_PREFERENCE, 0)
            ]
        except ValueError as exc:
            raise RuntimeError(
                "IMU_I2C_BUS must be a bus number or 'auto'."
            ) from exc

    available_buses = []

    for path in glob.glob("/dev/i2c-*"):
        name = path.rsplit("-", 1)[-1]

        if name.isdigit():
            available_buses.append(int(name))

    buses_to_check = preferred_buses

    if I2C_BUS_PREFERENCE == "auto":
        buses_to_check += sorted(
            number
            for number in set(available_buses)
            if number not in preferred_buses
        )

    errors = []

    for bus_number in buses_to_check:
        bus = None

        try:
            bus = SMBus(bus_number)

            who_am_i = bus.read_byte_data(
                IMU_ADDRESS, 0x75
            )

            # Accept 0x68 (standard MPU6050), 0x69 (AD0=1), 0x70 (MPU9250),
            # 0x71 (MPU9250 alt), 0x72 (clone variant seen on Jetson i2c-7).
            # Any non-zero response means the device is alive and communicating.
            if who_am_i == 0x00:
                errors.append(
                    f"Bus {bus_number}: WHO_AM_I returned 0x00 — sensor may be in reset"
                )
                bus.close()
                continue

            bus.write_byte_data(
                IMU_ADDRESS, 0x6B, 0x00
            )

            time.sleep(0.1)

            if bus_number == 7:
                log(
                    f"MPU6050 detected on I2C bus 7 "
                    f"at address 0x{IMU_ADDRESS:02X}. "
                    "This commonly maps to physical pins "
                    "3=SDA and 5=SCL on the Orin Nano "
                    "Developer Kit."
                )
            else:
                log(
                    f"MPU6050 detected on I2C bus "
                    f"{bus_number} at address "
                    f"0x{IMU_ADDRESS:02X}. "
                    "Verify this bus maps to physical "
                    "pins 3 and 5 on your carrier board.",
                    "WARN"
                )

            return bus, bus_number, who_am_i

        except Exception as exc:
            errors.append(
                f"Bus {bus_number}: {exc}"
            )

            if bus is not None:
                try:
                    bus.close()
                except Exception:
                    pass

    checked = (
        ", ".join(map(str, buses_to_check))
        if buses_to_check
        else "no I2C devices found"
    )

    details = "; ".join(errors[-5:])

    raise RuntimeError(
        f"Could not detect MPU6050 at "
        f"0x{IMU_ADDRESS:02X}. "
        f"Checked buses: {checked}. "
        f"Check I2C permissions, wiring, bus mapping "
        f"and sensor address. {details}"
    )


def sensor_loop():
    global latest

    bus = None
    yaw = 0.0
    previous = time.monotonic()
    last_telemetry_log = 0.0

    log(
        "IMU worker starting. "
        f"Jetson header pins: SDA={JETSON_SDA_PIN}, "
        f"SCL={JETSON_SCL_PIN}; "
        f"I2C bus: {I2C_BUS_PREFERENCE}"
    )

    if SIMULATE:
        log(
            "Simulation mode is active. "
            "No physical sensor data is being read.",
            "WARN"
        )

    while not stop_event.is_set():
        try:
            if bus is None and not SIMULATE:
                bus, bus_number, who_am_i = discover_mpu6050_bus()
                latest["source"] = f"I2C bus {bus_number}"
                log(
                    f"I2C connection ready: "
                    f"bus={bus_number}, "
                    f"address=0x{IMU_ADDRESS:02X}, "
                    f"WHO_AM_I=0x{who_am_i:02X}"
                )

            if not latest["connected"]:
                latest["connected"] = True
                socketio.emit(
                    "status",
                    {
                        "connected": True,
                        "source": latest["source"]
                    }
                )
                log("IMU connection ready.")

            now = time.monotonic()
            dt = max(
                0.0,
                min(now - previous, 0.2)
            )
            previous = now

            if SIMULATE:
                elapsed = time.time()

                roll = 18.0 * math.sin(elapsed * 0.7)
                pitch = 12.0 * math.sin(elapsed * 0.43)
                yaw = (yaw + 22.0 * dt) % 360.0

                ax = math.sin(math.radians(pitch))
                ay = -math.sin(math.radians(roll))

                az = math.sqrt(
                    max(
                        0.0,
                        1.0 - ax * ax - ay * ay
                    )
                )

                gx = 0.0
                gy = 0.0
                gz = 22.0

            else:
                ax, ay, az, gx, gy, gz = (
                    read_mpu6050(bus)
                )

                roll = math.degrees(
                    math.atan2(ay, az)
                )

                pitch = math.degrees(
                    math.atan2(
                        -ax,
                        math.sqrt(
                            ay * ay + az * az
                        )
                    )
                )

                yaw = (
                    yaw + gz * dt
                ) % 360.0

            packet = {
                "connected": True,
                "source": latest["source"],
                "roll": round(roll, 2),
                "pitch": round(pitch, 2),
                "yaw": round(yaw, 2),
                "ax": round(ax, 4),
                "ay": round(ay, 4),
                "az": round(az, 4),
                "gx": round(gx, 2),
                "gy": round(gy, 2),
                "gz": round(gz, 2),
                "timestamp": datetime.now().isoformat(
                    timespec="milliseconds"
                )
            }

            latest = packet

            socketio.emit("imu", packet)

            if now - last_telemetry_log >= 2.0:  # Log at most once per 2s
                log(
                    f"IMU ax={ax:+.3f}g "
                    f"ay={ay:+.3f}g "
                    f"az={az:+.3f}g | "
                    f"gx={gx:+.2f} "
                    f"gy={gy:+.2f} "
                    f"gz={gz:+.2f} deg/s | "
                    f"roll={roll:+.2f} "
                    f"pitch={pitch:+.2f} "
                    f"yaw={yaw:+.2f} deg"
                )

                last_telemetry_log = now

            time.sleep(1.0 / SAMPLE_HZ)

        except Exception as exc:
            if bus is not None:
                try:
                    bus.close()
                except Exception:
                    pass
                bus = None

            was_connected = latest["connected"]
            latest["connected"] = False
            socketio.emit(
                "status",
                {
                    "connected": False,
                    "source": latest["source"]
                }
            )

            log(
                f"IMU {'disconnected' if was_connected else 'not ready'}: "
                f"{exc}",
                "ERROR"
            )
            log(
                "Retrying the Jetson pin 3/5 I2C connection in 3 seconds.",
                "WARN"
            )
            stop_event.wait(3.0)

    if bus is not None:
        try:
            bus.close()
        except Exception:
            pass

    log("IMU worker stopped.")


@app.route("/")
def index():
    with logs_lock:
        initial_logs = list(logs)

    return render_template(
        "index.html",
        initial_logs=initial_logs,
        dashboard_port=PORT
    )


@socketio.on("connect")
def on_connect():
    socketio.emit(
        "status",
        {
            "connected": latest["connected"],
            "source": latest["source"]
        }
    )

    socketio.emit("imu", latest)

    with logs_lock:
        recent_logs = list(logs[-80:])

    for line in recent_logs:
        socketio.emit("log", {"line": line})


def start_sensor_worker():
    global sensor_thread_started

    with sensor_thread_lock:
        if not sensor_thread_started:
            sensor_thread_started = True

            thread = threading.Thread(
                target=sensor_loop,
                daemon=True
            )

            thread.start()



# Start the IMU sensor worker here so it runs regardless of whether
# the app is launched as `python app.py` or via gunicorn/Docker CMD.
start_sensor_worker()

if __name__ == "__main__":
    start_sensor_worker()

    log(
        f"Dashboard starting at "
        f"http://0.0.0.0:{PORT}"
    )

    socketio.run(
        app,
        host=HOST,
        port=PORT,
        debug=False,
        use_reloader=False,
        allow_unsafe_werkzeug=True
    )