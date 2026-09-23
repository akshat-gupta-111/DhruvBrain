import json
import queue
import sys
import threading
import time
import os
import glob
import cv2
import serial
import subprocess

# ==========================================
# 1. Jetson Camera (Thread-Safe Singleton)
# ==========================================
class JetsonCamera:
    _instance = None
    _lock = threading.Lock()

    def __new__(cls, *args, **kwargs):
        with cls._lock:
            if cls._instance is None:
                cls._instance = super(JetsonCamera, cls).__new__(cls)
                cls._instance._initialized = False
            return cls._instance

    def __init__(self, camera_index: int = 0):
        if self._initialized: return
        self.camera_index = camera_index
        self.cap = None
        self.latest_frame = None
        self.running = False
        self._initialized = True

    def start_capture_thread(self):
        if self.running: return
        # V4L2 is the standard Linux video driver used by Jetson
        self.cap = cv2.VideoCapture(self.camera_index, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise RuntimeError("Could not open Jetson webcam (/dev/video0).")
            
        self.running = True
        self.thread = threading.Thread(target=self._update, daemon=True)
        self.thread.start()
        print("[HAL Camera] Jetson background capture thread started.")

    def _update(self):
        while self.running:
            if self.cap and self.cap.isOpened():
                ret, frame = self.cap.read()
                if ret:
                    self.latest_frame = frame

    def capture_frame_bytes(self, quality: int = 85) -> bytes:
        if self.latest_frame is None:
            raise RuntimeError("No frame available.")
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), quality]
        success, buffer = cv2.imencode(".jpg", self.latest_frame, encode_param)
        return buffer.tobytes()

    def capture_frame(self):
        if self.latest_frame is None:
            raise RuntimeError("No frame available.")
        return self.latest_frame

    def close(self):
        self.running = False
        if self.thread and self.thread.is_alive():
            self.thread.join()
        if self.cap:
            self.cap.release()

# ==========================================
# 2. Arduino Motor Controller (USB Cable Link)
# ==========================================
SERIAL_BAUDRATE = 115200
DEFAULT_SERIAL_PORT = "/dev/ttyACM0"

class MotorController:
    def __init__(self, port: str = None, baudrate: int = SERIAL_BAUDRATE):
        self.cmd_queue = queue.Queue()
        self.port = port or self._find_arduino_port()
        self.baudrate = baudrate
        self.serial = None
        self.connected = False
        self.running = True
        self._last_move_duration_ms = 0  # Duration of the last timed motor command (ms)

        # Start the background Serial worker thread
        self.serial_thread = threading.Thread(target=self._serial_worker, daemon=True)
        self.serial_thread.start()

    def _find_arduino_port(self) -> str:
        """Auto-detects Arduino UNO R4 CDC port across Linux and Windows."""
        for candidate in ["/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyUSB0", "/dev/ttyUSB1"]:
            if os.path.exists(candidate):
                return candidate
        try:
            import serial.tools.list_ports
            for p in serial.tools.list_ports.comports():
                desc = (p.description or "").lower()
                hwid = (p.hwid or "").lower()
                if "arduino" in desc or "uno" in desc or "2341:" in hwid:
                    return p.device
            ports = list(serial.tools.list_ports.comports())
            if ports:
                return ports[0].device
        except Exception:
            pass
        return DEFAULT_SERIAL_PORT

    def _serial_worker(self):
        """Background thread handling connection, queue dispatch, and ACK handshake."""
        while self.running:
            try:
                if not self.connected:
                    active_port = self.port if os.path.exists(self.port) else self._find_arduino_port()
                    print(f"[HAL Motor] Connecting to Arduino on {active_port} @ {self.baudrate} baud...")
                    self.serial = serial.Serial(
                        port=active_port,
                        baudrate=self.baudrate,
                        timeout=1.0,
                        write_timeout=1.0
                    )
                    time.sleep(1.8) # Allow Arduino boot reset recovery
                    self.serial.reset_input_buffer()
                    self.serial.reset_output_buffer()
                    
                    # Force Arduino to respect Jetson's raw durations (disable 1800ms safeguard overrides)
                    self.serial.write(b"DURATION:RAW\n")
                    self.serial.flush()
                    
                    self.connected = True
                    self.port = active_port
                    print(f"[HAL Motor] USB Serial Connected to Arduino on {self.port}!\n")

                # 1. Process outgoing command from queue
                try:
                    payload = self.cmd_queue.get(timeout=0.05)
                    if not payload.endswith("\n"):
                        payload += "\n"

                    # Parse duration from the payload just before executing to avoid race conditions!
                    import re
                    m = re.search(r'<[^,]+,\d+,(\d+)>', payload)
                    move_duration = int(m.group(1)) if m else 0

                    self.serial.write(payload.encode("utf-8"))
                    self.serial.flush()
                    send_time = time.time()

                    # Wait for ACK response from Arduino (max 1.0s)
                    start_wait = time.time()
                    while time.time() - start_wait < 1.0:
                        if self.serial.in_waiting > 0:
                            line = self.serial.readline().decode("utf-8", errors="replace").strip()
                            if line == "ACK":
                                break
                            elif line:
                                print(f"[ARDUINO] {line}")
                                if "MODE: Switched to MANUAL" in line:
                                    self.clear_queue()
                        time.sleep(0.01)

                    # KEY FIX: If this was a timed movement command, wait for the
                    # full duration before allowing the next command to be sent.
                    # Without this, the next command (even just an LED update) will
                    # interrupt the motor mid-movement.
                    if move_duration > 0:
                        elapsed_ms = (time.time() - send_time) * 1000
                        remaining_ms = move_duration - elapsed_ms
                        if remaining_ms > 50:  # Only sleep if more than 50ms left
                            time.sleep(remaining_ms / 1000.0)

                except queue.Empty:
                    pass

                # 2. Process asynchronous incoming telemetry from Arduino
                if self.connected and self.serial and self.serial.in_waiting > 0:
                    line = self.serial.readline().decode("utf-8", errors="replace").strip()
                    if line:
                        print(f"[ARDUINO] {line}")
                        # If Arduino switched to Manual mode, drain queued commands
                        if "MODE: Switched to MANUAL" in line:
                            self.clear_queue()

            except Exception as e:
                print(f"[HAL Motor] Serial Error: {e}. Reconnecting in 2s...")
                self.connected = False
                if self.serial:
                    try:
                        self.serial.close()
                    except Exception:
                        pass
                time.sleep(2.0)

    def clear_queue(self):
        """Purges any pending commands from the queue immediately."""
        with self.cmd_queue.mutex:
            self.cmd_queue.queue.clear()

    def execute(self, action: str, led_mood: str = "IDLE_WHITE"):
        # Reduced durations (600ms) and speeds (150) to prevent crashing into walls
        # Reduced durations (600ms) and speeds (150) to prevent crashing into walls
        action_map = {
            "APPROACH_0.5M": ("⬆️  APPROACH (+0.5m)", "<REV,150,600>"),
            "BACKUP_0.5M":   ("⬇️  BACKUP   (-0.5m)", "<FWD,150,600>"),
            "STRAFE_LEFT":   ("⬅️  STRAFE   (L)",     "<STRAFE_R,150,600>"),
            "STRAFE_RIGHT":  ("➡️  STRAFE   (R)",     "<STRAFE_L,150,600>"),
            "DIAGONAL_FL":   ("↖️  DIAG     (FL)",    "<DIAG_BR,150,600>"),
            "DIAGONAL_FR":   ("↗️  DIAG     (FR)",    "<DIAG_BL,150,600>"),
            "PIVOT_LEFT_30": ("🔄  PIVOT    (-30°)",  "<PIVOT_R,150,600>"),
            "PIVOT_RIGHT_30":("🔄  PIVOT    (+30°)",  "<PIVOT_L,150,600>"),
            
            # Continuous streaming commands for smooth Find Exit navigation (duration=0)
            "CONT_FWD":      ("⬆️  CONT FWD",         "<REV,150,0>"),
            "CONT_REV":      ("⬇️  CONT REV",         "<FWD,150,0>"),
            "CONT_DIAG_FL":  ("↖️  CONT DIAG (FL)",   "<DIAG_BR,150,0>"),
            "CONT_DIAG_FR":  ("↗️  CONT DIAG (FR)",   "<DIAG_BL,150,0>"),
            "CONT_PIVOT_L":  ("🔄  CONT PIVOT (L)",   "<PIVOT_R,150,0>"),
            "CONT_PIVOT_R":  ("🔄  CONT PIVOT (R)",   "<PIVOT_L,150,0>"),
            
            # LiDAR-specific commands: The LiDAR is facing the true physical front, so these 
            # bypass the camera-inversions. They also use a 300ms watchdog duration instead of 0 
            # so the Arduino safely auto-stops if the Jetson crashes or gets stopped!
            "LIDAR_FWD":     ("⬆️  LIDAR FWD",       "<FWD,150,300>"),
            "LIDAR_DIAG_FL": ("↖️  LIDAR DIAG (FL)", "<DIAG_FL,150,300>"),
            "LIDAR_DIAG_FR": ("↗️  LIDAR DIAG (FR)", "<DIAG_FR,150,300>"),
            "LIDAR_PIVOT_L": ("🔄  LIDAR PIVOT (L)", "<PIVOT_L,150,300>"),
            "LIDAR_PIVOT_R": ("🔄  LIDAR PIVOT (R)", "<PIVOT_R,150,300>"),

            "HALT":          ("🛑 HALT",             "<STOP>"),
            "CONTINUE_WANDER":("🔄 WANDER",          "<WANDER>")
        }
        display_text, serial_cmd = action_map.get(action, (f"❓ UNKNOWN ({action})", "<STOP>"))
        print(f"[CHASSIS ACTION] {display_text} | 💡 LED: {led_mood}")

        full_payload = f"{serial_cmd}|<LED,{led_mood}>\n"
        # Push to background thread so AI loop never freezes
        self.cmd_queue.put(full_payload)

    def send_raw(self, raw_cmd: str):
        """Sends raw command string (e.g. MODE:MANUAL, MODE:AUTO, or custom token)."""
        clean = raw_cmd.strip()
        if clean:
            self.cmd_queue.put(clean + "\n")

    def close(self):
        self.running = False
        if self.serial and self.serial.is_open:
            try:
                self.send_raw("<STOP>")
                time.sleep(0.1)
                self.serial.close()
            except Exception:
                pass

# ==========================================
# 3. Speaker & Mic — HTTP Audio Sidecar
# ==========================================
# All audio runs on the Jetson HOST via audio_server.py (outside Docker).
# audio_server.py must be running on the host before docker compose up.

AUDIO_SERVER = os.getenv("AUDIO_SERVER_URL", "http://localhost:5555")

class Speaker:
    """Delegates TTS to audio_server.py running on the Jetson host.
    POST /speak → host streams edge-tts to the physical speaker.
    Blocks until playback is complete (same interface as before).
    """

    def __init__(self):
        print(f"[HAL Speaker] Audio sidecar connected → {AUDIO_SERVER}")

    def speak(self, text: str):
        if not text.strip():
            return
        print(f'[SPEAKER 🎙️] "{text}"')
        try:
            import urllib.request
            body = json.dumps({"text": text}).encode()
            req  = urllib.request.Request(
                f"{AUDIO_SERVER}/speak",
                data=body,
                headers={"Content-Type": "application/json"},
                method="POST"
            )
            with urllib.request.urlopen(req, timeout=60) as resp:
                resp.read()
        except Exception as e:
            print(f"[HAL Speaker] sidecar error: {e}")

    def play_loop(self, filename: str):
        try:
            import urllib.request
            body = json.dumps({"file": filename}).encode()
            req  = urllib.request.Request(
                f"{AUDIO_SERVER}/play_loop",
                data=body,
                headers={"Content-Type": "application/json"},
                method="POST"
            )
            with urllib.request.urlopen(req, timeout=5) as resp:
                resp.read()
        except Exception as e:
            print(f"[HAL Speaker] play_loop error: {e}")

    def stop_loop(self):
        try:
            import urllib.request
            req  = urllib.request.Request(
                f"{AUDIO_SERVER}/stop_loop",
                data=b"{}",
                headers={"Content-Type": "application/json"},
                method="POST"
            )
            with urllib.request.urlopen(req, timeout=5) as resp:
                resp.read()
        except Exception as e:
            print(f"[HAL Speaker] stop_loop error: {e}")


class Microphone:
    """Delegates STT to audio_server.py running on the Jetson host.
    GET  /hear   → returns latest recognised text (non-blocking, "" if none)
    POST /mute   → tells the server to discard mic input
    POST /unmute → tells the server to start recognising again
    """
    _instance = None
    _lock     = threading.Lock()

    def __new__(cls, *args, **kwargs):
        with cls._lock:
            if cls._instance is None:
                cls._instance = super(Microphone, cls).__new__(cls)
                cls._instance._initialized = False
            return cls._instance

    def __init__(self):
        if self._initialized:
            return
        self.is_muted = True
        try:
            import urllib.request
            with urllib.request.urlopen(f"{AUDIO_SERVER}/health", timeout=3) as r:
                r.read()
            print(f"[HAL Mic] 🎤 Audio sidecar connected → {AUDIO_SERVER}")
        except Exception as e:
            print(f"[HAL Mic] ⚠️  Audio sidecar not reachable at {AUDIO_SERVER}: {e}")
            print("[HAL Mic]    → Start audio_server.py on the Jetson before launching Docker.")
        self._initialized = True

    def get_speech(self) -> str:
        try:
            import urllib.request
            with urllib.request.urlopen(f"{AUDIO_SERVER}/hear", timeout=2) as r:
                data = json.loads(r.read())
            return data.get("text", "")
        except Exception:
            return ""

    def _post(self, path: str):
        try:
            import urllib.request
            req = urllib.request.Request(
                f"{AUDIO_SERVER}{path}",
                data=b"{}",
                headers={"Content-Type": "application/json"},
                method="POST"
            )
            with urllib.request.urlopen(req, timeout=3) as r:
                r.read()
        except Exception as e:
            print(f"[HAL Mic] sidecar error on {path}: {e}")

    def mute(self):
        self.is_muted = True
        self._post("/mute")

    def unmute(self):
        self.is_muted = False
        self._post("/unmute")


# ==========================================
# 4. LiDAR (Autonomous Trigger)
# ==========================================
def get_sector(angle):
    if angle >= 337.5 or angle < 22.5:   return 'N'  
    elif 22.5 <= angle < 67.5:           return 'NE' 
    elif 67.5 <= angle < 112.5:          return 'E'  
    elif 112.5 <= angle < 157.5:         return 'SE' 
    elif 157.5 <= angle < 202.5:         return 'S'  
    elif 202.5 <= angle < 247.5:         return 'SW' 
    elif 247.5 <= angle < 292.5:         return 'W'  
    elif 292.5 <= angle < 337.5:         return 'NW'

class JetsonLiDAR:
    _instance = None
    _lock = threading.Lock()

    def __new__(cls, *args, **kwargs):
        with cls._lock:
            if cls._instance is None:
                cls._instance = super(JetsonLiDAR, cls).__new__(cls)
                cls._instance._initialized = False
            return cls._instance

    def __init__(self):
        if self._initialized: return
        self.port = '/dev/ttyUSB0'
        self.running = False
        self._thread = None
        self.latest_safest_direction = ""
        self.latest_obstacles = ""
        self.latest_closest_obstacles = {}
        self._initialized = True

    def prompt_user(self) -> str:
        time.sleep(5)
        return "SCAN"

    def start(self):
        if self.running: return
        self.running = True
        self._thread = threading.Thread(target=self._lidar_loop, daemon=True)
        self._thread.start()

    def stop(self):
        self.running = False
        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None

    def _lidar_loop(self):
        try:
            from rplidar import RPLidar
            lidar = RPLidar(self.port, baudrate=115200, timeout=3)
            try:
                lidar.stop()
                lidar.stop_motor()
                lidar.clean_input()
            except:
                pass
            time.sleep(0.5)
            
            for scan in lidar.iter_scans():
                if not self.running:
                    break
                sector_data = { 'N': [], 'NE': [], 'E': [], 'SE': [], 'S': [], 'SW': [], 'W': [], 'NW': [] }
                for (_, angle, distance) in scan:
                    # Ignore anything less than 200mm (chassis/wire reflections!)
                    if distance > 200:
                        # Electronically rotate the LiDAR 90 degrees clockwise to align 
                        # its software "Front" (0 deg) with the robot's true physical Front.
                        corrected_angle = (angle + 90.0) % 360.0
                        sector = get_sector(corrected_angle)
                        if sector:
                            sector_data[sector].append(distance)
                        
                closest_obstacles = {}
                for sector, distances in sector_data.items():
                    # If the distances array is empty, it means the ENTIRE sector was either 
                    # blocked (< 200mm) or invalid (0). So it is highly DANGEROUS (0m), not safe!
                    closest_obstacles[sector] = min(distances) if distances else 0
                    
                self.latest_closest_obstacles = closest_obstacles
                self.latest_safest_direction = max(closest_obstacles, key=closest_obstacles.get)
                self.latest_obstacles = (f"Front:{closest_obstacles['N']/1000:.1f}m, "
                                         f"Back:{closest_obstacles['S']/1000:.1f}m, "
                                         f"Left:{closest_obstacles['W']/1000:.1f}m, "
                                         f"Right:{closest_obstacles['E']/1000:.1f}m")
                
            lidar.stop()
            lidar.stop_motor()
            lidar.disconnect()
        except Exception as e:
            print(f"[HAL LiDAR] Thread error: {e}")

    def get_safest_direction(self) -> str:
        if not self.running:
            return ""
        if self.latest_obstacles:
            print(f"[HAL LiDAR] Obstacles -> {self.latest_obstacles}")
        if self.latest_safest_direction:
            print(f"[HAL LiDAR] Safest free-space direction: {self.latest_safest_direction}")
        return self.latest_safest_direction

    def get_obstacles(self) -> dict:
        return self.latest_closest_obstacles


# ==========================================
# 5. Interactive Standalone Execution
# ==========================================
if __name__ == "__main__":
    port_arg = sys.argv[1] if len(sys.argv) > 1 else None
    controller = MotorController(port=port_arg)
    time.sleep(2.0)

    print("-" * 65)
    print("   DHRUVBRAIN — JETSON SERIAL COMMAND CONSOLE")
    print("-" * 65)
    print("Quick Shortcuts:")
    print("  [1] Forward:  APPROACH_0.5M  (FWD 180, 1000ms)")
    print("  [2] Backward: BACKUP_0.5M    (REV 180, 1000ms)")
    print("  [3] Strafe L: STRAFE_LEFT    (STRAFE_L 200, 1000ms)")
    print("  [4] Strafe R: STRAFE_RIGHT   (STRAFE_R 200, 1000ms)")
    print("  [5] Pivot L:  PIVOT_LEFT_30  (PIVOT_L 150, 600ms)")
    print("  [6] Pivot R:  PIVOT_RIGHT_30 (PIVOT_R 150, 600ms)")
    print("  [0] Halt:     HALT (<STOP>)")
    print("Modes: 'auto', 'manual', 'status', 'exit'")
    print("-" * 65)

    try:
        while True:
            cmd = input("\n[Jetson Input] > ").strip()
            if not cmd:
                continue

            lowered = cmd.lower()
            if lowered in ["exit", "quit", "q"]:
                controller.send_raw("<STOP>")
                break
            elif lowered in ["auto", "mode:auto"]:
                controller.clear_queue()
                controller.send_raw("MODE:AUTO")
            elif lowered in ["manual", "mode:manual"]:
                controller.clear_queue()
                controller.send_raw("MODE:MANUAL")
            elif lowered in ["status", "mode?"]:
                controller.send_raw("MODE?")
            elif cmd == "1":
                controller.execute("APPROACH_0.5M", "IDLE_WHITE")
            elif cmd == "2":
                controller.execute("BACKUP_0.5M", "IDLE_WHITE")
            elif cmd == "3":
                controller.execute("STRAFE_LEFT", "IDLE_WHITE")
            elif cmd == "4":
                controller.execute("STRAFE_RIGHT", "IDLE_WHITE")
            elif cmd == "5":
                controller.execute("PIVOT_LEFT_30", "CURIOSITY_GREEN")
            elif cmd == "6":
                controller.execute("PIVOT_RIGHT_30", "CURIOSITY_GREEN")
            elif cmd == "0":
                controller.execute("HALT", "ALERT_RED")
            else:
                controller.send_raw(cmd)

    except KeyboardInterrupt:
        controller.send_raw("<STOP>")
    finally:
        controller.close()
