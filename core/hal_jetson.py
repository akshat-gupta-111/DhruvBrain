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

    def close(self):
        self.running = False
        if self.thread and self.thread.is_alive():
            self.thread.join()
        if self.cap:
            self.cap.release()

# ==========================================
# 2. Arduino Motor Controller (Linux USB)
# ==========================================
class MotorController:
    def __init__(self, baud_rate: int = 115200):
        self.serial_conn = None
        self._init_serial(baud_rate)

    def _init_serial(self, baud: int):
        # Jetson usually mounts Arduino Uno as /dev/ttyACM0 or ACM1
        ports = glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*")
        if ports:
            try:
                self.serial_conn = serial.Serial(ports[0], baud, timeout=1)
                time.sleep(1.5)
                print(f"[HAL Motor] Connected to Arduino on {ports[0]}")
            except Exception as e:
                print(f"[HAL Motor] Failed to open serial port {ports[0]}: {e}")
        else:
            print("[HAL Motor] No Arduino found on Jetson USB.")

    def execute(self, action: str, led_mood: str = "IDLE_WHITE"):
        action_map = {
            "APPROACH_0.5M": ("⬆️  APPROACH (+0.5m)", "<FWD,50,500>"),
            "BACKUP_0.5M":   ("⬇️  BACKUP   (-0.5m)", "<BWD,50,500>"),
            "PIVOT_LEFT_30": ("⬅️  PIVOT    (-30°)",  "<LEFT,40,300>"),
            "PIVOT_RIGHT_30":("➡️  PIVOT    (+30°)",  "<RIGHT,40,300>"),
            "HALT":          ("🛑 HALT",             "<STOP>"),
            "CONTINUE_WANDER":("🔄 WANDER",          "<WANDER>")
        }
        display_text, serial_cmd = action_map.get(action, (f"❓ UNKNOWN ({action})", "<STOP>"))
        print(f"[CHASSIS ACTION] {display_text} | 💡 LED: {led_mood}")

        if self.serial_conn and self.serial_conn.is_open:
            try:
                full_payload = f"{serial_cmd}|<LED,{led_mood}>\n"
                self.serial_conn.write(full_payload.encode('utf-8'))
            except Exception as e:
                print(f"[HAL Motor] Serial write error: {e}")

# ==========================================
# 3. Speaker & Mic — HTTP Audio Sidecar
# ==========================================
# All audio runs on the Jetson HOST via audio_server.py (outside Docker).
# This container is a thin HTTP client — no sounddevice, no ALSA issues.
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


class Microphone:
    """Delegates STT to audio_server.py running on the Jetson host.
    GET  /hear   → returns latest recognised text (non-blocking, "" if none)
    POST /mute   → tells the server to discard mic input
    POST /unmute → tells the server to start recognising again
    Same get_speech() / mute() / unmute() interface — main.py unchanged.
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
        # Verify audio server is reachable
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
class JetsonLiDAR:
    def prompt_user(self) -> str:
        time.sleep(5)
        return "SCAN"


