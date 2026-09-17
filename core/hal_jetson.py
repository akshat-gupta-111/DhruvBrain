import json
import queue
import sys
import threading
import queue
import asyncio
from bleak import BleakClient, BleakScanner
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
# 2. Arduino Motor Controller (BLE)
# ==========================================
BLE_SERVICE_UUID = "19b10000-e8f2-537e-4f6c-d104768a1214"
BLE_CHAR_UUID = "19b10001-e8f2-537e-4f6c-d104768a1214"
BLE_DEVICE_NAME = "Dhruv_Arduino"

class MotorController:
    def __init__(self):
        self.cmd_queue = queue.Queue()
        self.client = None
        self.connected = False
        
        # Start the background BLE worker thread
        self.ble_thread = threading.Thread(target=self._run_ble_loop, daemon=True)
        self.ble_thread.start()

    def _run_ble_loop(self):
        """Runs the asyncio event loop for BLE in a background thread."""
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        loop.run_until_complete(self._ble_worker())

    async def _ble_worker(self):
        print(f"[HAL Motor] Scanning for BLE device: {BLE_DEVICE_NAME}...")
        while True:
            try:
                if not self.connected:
                    device = await BleakScanner.find_device_by_name(BLE_DEVICE_NAME, timeout=5.0)
                    if device:
                        print(f"[HAL Motor] Found {BLE_DEVICE_NAME}. Connecting...")
                        self.client = BleakClient(device)
                        await self.client.connect()
                        self.connected = True
                        print(f"[HAL Motor] BLE Connected!")
                    else:
                        await asyncio.sleep(2)
                        continue

                # Check queue for commands (non-blocking)
                try:
                    payload = self.cmd_queue.get_nowait()
                    if self.client and self.connected:
                        await self.client.write_gatt_char(BLE_CHAR_UUID, payload.encode('utf-8'))
                except queue.Empty:
                    await asyncio.sleep(0.05) # Small sleep to prevent CPU spin
                    continue

            except Exception as e:
                print(f"[HAL Motor] BLE Error: {e}. Reconnecting...")
                self.connected = False
                if self.client:
                    try:
                        await self.client.disconnect()
                    except:
                        pass
                await asyncio.sleep(2)

    def execute(self, action: str, led_mood: str = "IDLE_WHITE"):
        action_map = {
            "APPROACH_0.5M": ("⬆️  APPROACH (+0.5m)", "<FWD,50,500>"),
            "BACKUP_0.5M":   ("⬇️  BACKUP   (-0.5m)", "<REV,50,500>"),
            "STRAFE_LEFT":   ("⬅️  STRAFE   (L)",     "<STRAFE_L,50,500>"),
            "STRAFE_RIGHT":  ("➡️  STRAFE   (R)",     "<STRAFE_R,50,500>"),
            "DIAGONAL_FL":   ("↖️  DIAG     (FL)",    "<DIAG_FL,50,500>"),
            "DIAGONAL_FR":   ("↗️  DIAG     (FR)",    "<DIAG_FR,50,500>"),
            "PIVOT_LEFT_30": ("🔄  PIVOT    (-30°)",  "<PIVOT_L,40,300>"),
            "PIVOT_RIGHT_30":("🔄  PIVOT    (+30°)",  "<PIVOT_R,40,300>"),
            "HALT":          ("🛑 HALT",             "<STOP>"),
            "CONTINUE_WANDER":("🔄 WANDER",          "<WANDER>")
        }
        display_text, serial_cmd = action_map.get(action, (f"❓ UNKNOWN ({action})", "<STOP>"))
        print(f"[CHASSIS ACTION] {display_text} | 💡 LED: {led_mood}")

        full_payload = f"{serial_cmd}|<LED,{led_mood}>\n"
        # Push to background thread so AI loop never freezes on BLE drop
        self.cmd_queue.put(full_payload)

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
    def __init__(self):
        self.port = '/dev/ttyUSB0'

    def prompt_user(self) -> str:
        # Kept for backward compatibility if called
        time.sleep(5)
        return "SCAN"

    def get_safest_direction(self) -> str:
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
            
            safest_direction = ""
            for i, scan in enumerate(lidar.iter_scans()):
                sector_data = { 'N': [], 'NE': [], 'E': [], 'SE': [], 'S': [], 'SW': [], 'W': [], 'NW': [] }
                for (_, angle, distance) in scan:
                    if distance > 0:
                        sector = get_sector(angle)
                        if sector:
                            sector_data[sector].append(distance)
                        
                closest_obstacles = {}
                for sector, distances in sector_data.items():
                    closest_obstacles[sector] = min(distances) if distances else 12000
                    
                safest_direction = max(closest_obstacles, key=closest_obstacles.get)
                break # Just read one full 360-degree rotation!
                
            lidar.stop()
            lidar.stop_motor()
            lidar.disconnect()
            
            print(f"[HAL LiDAR] Safest free-space direction: {safest_direction}")
            return safest_direction
        except Exception as e:
            print(f"[HAL LiDAR] Error reading LiDAR: {e}")
            return ""


