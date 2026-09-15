import asyncio
import os
import cv2
import sys
import select
import queue
import threading
from typing import Optional
import time
import subprocess
import glob
import speech_recognition as sr
import edge_tts

# Optional serial import for when Arduino is plugged in
try:
    import serial
except ImportError:
    serial = None


# ==========================================
# 1. Mac Camera Abstraction
# ==========================================


# ==========================================
# 1. Mac Camera Abstraction (Thread-Safe Singleton)
# ==========================================
class MacCamera:
    _instance = None
    _lock = threading.Lock()

    def __new__(cls, *args, **kwargs):
        # Singleton pattern: Ensure only one camera object exists globally
        with cls._lock:
            if cls._instance is None:
                cls._instance = super(MacCamera, cls).__new__(cls)
                cls._instance._initialized = False
            return cls._instance

    def __init__(self, camera_index: int = 0):
        if self._initialized:
            return
            
        self.camera_index = camera_index
        self.cap: Optional[cv2.VideoCapture] = None
        self.latest_frame = None
        self.running = False
        self._initialized = True

    def start_capture_thread(self):
        """Spawns a background thread that constantly reads the camera."""
        if self.running:
            return
            
        # cv2.CAP_AVFOUNDATION ensures clean access to macOS camera
        self.cap = cv2.VideoCapture(self.camera_index, cv2.CAP_AVFOUNDATION)
        if not self.cap.isOpened():
            raise RuntimeError("Could not open Mac webcam. Check permissions.")
            
        self.running = True
        self.thread = threading.Thread(target=self._update, daemon=True)
        self.thread.start()
        print("[HAL Camera] Background capture thread started.")

    def _update(self):
        """Continuously pulls frames into memory."""
        while self.running:
            if self.cap and self.cap.isOpened():
                ret, frame = self.cap.read()
                if ret:
                    self.latest_frame = frame

    def capture_frame_bytes(self, quality: int = 85) -> bytes:
        """Called by LangGraph to get the current frame for Moondream/OCR."""
        if self.latest_frame is None:
            raise RuntimeError("No frame available yet. Is the camera covered or unplugged?")

        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), quality]
        success, buffer = cv2.imencode(".jpg", self.latest_frame, encode_param)
        if not success:
            raise RuntimeError("Failed to encode frame to JPEG.")
            
        return buffer.tobytes()

    def close(self):
        self.running = False
        if self.thread and self.thread.is_alive():
            self.thread.join()
        if self.cap and self.cap.isOpened():
            self.cap.release()
            self.cap = None
        print("[HAL Camera] Safely shut down.")
# ==========================================
# 2. Motor Controller (Simulated + Serial)
# ==========================================
class MotorController:
    def __init__(self, baud_rate: int = 115200):
        self.serial_conn = None
        self._init_serial(baud_rate)

    def _init_serial(self, baud: int):
        """Auto-detects Arduino on macOS USB ports."""
        if serial is None:
            print("[HAL Motor] pyserial not installed. Operating in simulation-only mode.")
            return

        ports = glob.glob("/dev/cu.usbmodem*") + glob.glob("/dev/cu.usbserial*")
        if ports:
            try:
                self.serial_conn = serial.Serial(ports[0], baud, timeout=1)
                time.sleep(1.5)
                print(f"[HAL Motor] Connected to Arduino on {ports[0]}")
            except Exception as e:
                print(f"[HAL Motor] Failed to open serial port {ports[0]}: {e}")
        else:
            print("[HAL Motor] No Arduino found on USB. Operating in simulation-only mode.")

    def execute(self, action: str, led_mood: str = "IDLE_WHITE"):
        """Prints visual representation, simulation LED state, and sends raw command to Arduino if connected."""
        action_map = {
            "APPROACH_0.5M": ("⬆️  APPROACH (+0.5m)", "<FWD,50,500>"),
            "BACKUP_0.5M":   ("⬇️  BACKUP   (-0.5m)", "<BWD,50,500>"),
            "PIVOT_LEFT_30": ("⬅️  PIVOT    (-30°)",  "<LEFT,40,300>"),
            "PIVOT_RIGHT_30":("➡️  PIVOT    (+30°)",  "<RIGHT,40,300>"),
            "HALT":          ("🛑 HALT",             "<STOP>"),
            "CONTINUE_WANDER":("🔄 WANDER",          "<WANDER>")
        }

        display_text, serial_cmd = action_map.get(action, (f"❓ UNKNOWN ({action})", "<STOP>"))
        print(f"\n[CHASSIS ACTION] {display_text} | 💡 LED State: {led_mood}")

        if self.serial_conn and self.serial_conn.is_open:
            try:
                full_payload = f"{serial_cmd}|<LED,{led_mood}>\n"
                self.serial_conn.write(full_payload.encode('utf-8'))
            except Exception as e:
                print(f"[HAL Motor] Serial write error: {e}")


# ==========================================
# 3. Audio Output (Local Mac Speech)
# ==========================================

            # Ava is extremely natural, conversational, and energetic


class Speaker:
    """TTS via edge-tts on Mac. Falls back to macOS `say` command."""
    VOICE = "en-US-GuyNeural"

    def __init__(self):
        print("[HAL Speaker] Edge TTS initialized.")

    def speak(self, text: str):
        if not text.strip():
            return
        print(f'[SPEAKER 🎙️] "{text}"')
        try:
            loop = asyncio.new_event_loop()
            loop.run_until_complete(self._synthesize_and_play(text))
            loop.close()
        except Exception as e:
            print(f"[HAL Speaker] Edge TTS failed: {e} — falling back to say")
            self._fallback_speak(text)

    async def _synthesize_and_play(self, text: str):
        communicate = edge_tts.Communicate(text, self.VOICE)
        await communicate.save("/tmp/speak.mp3")
        subprocess.run(["afplay", "/tmp/speak.mp3"], stderr=subprocess.DEVNULL)

    def _fallback_speak(self, text: str):
        sanitized = text.replace('"', '\\"')
        subprocess.run(["say", "-v", "Samantha", sanitized])


# ==========================================
# 4. Mock LiDAR Trigger
# ==========================================



# ... (Keep MacCamera, MotorController, Speaker exactly the same) ...

class MockLiDAR:
    def prompt_user(self) -> str:
        # No more [Enter]! It simulates the robot wandering to a new spot every 5 seconds.
        time.sleep(5) 
        return "SCAN"

class Microphone:
    """Continuous STT via SpeechRecognition + Google STT on Mac default mic."""
    _instance = None
    _lock = threading.Lock()

    def __new__(cls, *args, **kwargs):
        with cls._lock:
            if cls._instance is None:
                cls._instance = super(Microphone, cls).__new__(cls)
                cls._instance._initialized = False
            return cls._instance

    def __init__(self):
        if self._initialized:
            return

        self.speech_queue = queue.Queue()
        self.is_muted = True
        self._stop_fn = None

        try:
            self._recognizer = sr.Recognizer()
            self._recognizer.dynamic_energy_threshold = True
            mic = sr.Microphone()  # Uses Mac default mic
            with mic as source:
                self._recognizer.adjust_for_ambient_noise(source, duration=1)
            self._stop_fn = self._recognizer.listen_in_background(
                mic, self._recognized_cb, phrase_time_limit=6
            )
            print("[HAL Mic] 🎤 Background Queue Listener active.")
        except Exception as e:
            print(f"[HAL Mic] ⚠️  Failed to start microphone: {e}")

        self._initialized = True

    def _recognized_cb(self, recognizer: sr.Recognizer, audio: sr.AudioData):
        if self.is_muted:
            return
        try:
            text = recognizer.recognize_google(audio).lower().strip()
            if text:
                print(f"\n[MIC 🎤] Heard: '{text}'")
                self.speech_queue.put(text)
        except sr.UnknownValueError:
            pass
        except sr.RequestError as e:
            print(f"[HAL Mic] STT request error: {e}")

    def get_speech(self) -> str:
        try:
            return self.speech_queue.get_nowait()
        except queue.Empty:
            return ""

    def mute(self):
        self.is_muted = True
        with self.speech_queue.mutex:
            self.speech_queue.queue.clear()

    def unmute(self):
        with self.speech_queue.mutex:
            self.speech_queue.queue.clear()
        self.is_muted = False