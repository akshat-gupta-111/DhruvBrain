import asyncio
import queue
import sys
import threading
import time
import os
import glob
import cv2
import serial
import speech_recognition as sr
import edge_tts
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
# 3. Speaker & Mic (Singleton Queue)
# ==========================================
# ALSA device for playback — override via ALSA_DEVICE in .env or docker-compose.yml
# Run `aplay -l` on the Jetson to find your card index/name
ALSA_DEVICE = os.getenv("ALSA_DEVICE", "plughw:2,0")

# PyAudio device index for the ReSpeaker microphone
# Run `python -c "import speech_recognition as sr; print(sr.Microphone.list_microphone_names())"` to find it
MIC_DEVICE_INDEX = int(os.getenv("MIC_DEVICE_INDEX", "0"))


class Speaker:
    """TTS via edge-tts (Microsoft Neural voices, no API key needed).
    Synthesizes MP3 to /tmp/speak.mp3, plays via mpv on the ALSA device.
    Falls back to espeak if edge-tts or mpv fails.
    """
    VOICE = "en-US-GuyNeural"

    def __init__(self):
        self.alsa_device = ALSA_DEVICE
        print(f"[HAL Speaker] Edge TTS initialized (mpv via {self.alsa_device}).")

    def speak(self, text: str):
        if not text.strip():
            return
        print(f'[SPEAKER 🎙️] "{text}"')
        try:
            # Run async edge-tts in a fresh event loop (safe from any thread)
            loop = asyncio.new_event_loop()
            loop.run_until_complete(self._synthesize_and_play(text))
            loop.close()
        except Exception as e:
            print(f"[HAL Speaker] Edge TTS failed: {e} — falling back to espeak")
            self._fallback_speak(text)

    async def _synthesize_and_play(self, text: str):
        """Generates MP3 via edge-tts then plays it through mpv on the ALSA device."""
        communicate = edge_tts.Communicate(text, self.VOICE)
        await communicate.save("/tmp/speak.mp3")
        # mpv audio-device format: alsa/plughw:CARD,DEV
        alsa_mpv = f"alsa/{self.alsa_device}"
        result = subprocess.run(
            ["mpv", "--no-terminal", f"--audio-device={alsa_mpv}", "/tmp/speak.mp3"],
            stderr=subprocess.PIPE
        )
        if result.returncode != 0:
            # mpv failed — try without device spec (use ALSA default)
            subprocess.run(["mpv", "--no-terminal", "/tmp/speak.mp3"], stderr=subprocess.DEVNULL)

    def _fallback_speak(self, text: str):
        sanitized = text.replace('"', '\\"')
        subprocess.run(["espeak", "-ven+f3", "-s150", sanitized])


class Microphone:
    """Continuous STT via SpeechRecognition + PyAudio + Google STT.
    Runs listen_in_background() in a daemon thread, same queue-based
    interface as before — main.py needs zero changes.
    """
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
        self.is_muted = True  # Start muted until Orchestrator is ready
        self._stop_fn = None

        try:
            self._recognizer = sr.Recognizer()
            # Slightly more aggressive energy threshold for robot env noise
            self._recognizer.dynamic_energy_threshold = True
            self._recognizer.energy_threshold = 400

            mic = sr.Microphone(device_index=MIC_DEVICE_INDEX)
            with mic as source:
                # Calibrate for ambient noise once at startup
                self._recognizer.adjust_for_ambient_noise(source, duration=1)

            # listen_in_background returns a stop function; phrase_time_limit
            # prevents the thread from blocking forever on one phrase
            self._stop_fn = self._recognizer.listen_in_background(
                mic, self._recognized_cb, phrase_time_limit=6
            )
            print("[HAL Mic] 🎤 SpeechRecognition Background Queue Listener active.")
        except Exception as e:
            print(f"[HAL Mic] ⚠️  Failed to start microphone: {e}")

        self._initialized = True

    def _recognized_cb(self, recognizer: sr.Recognizer, audio: sr.AudioData):
        """Called in background thread for every detected phrase."""
        if self.is_muted:
            return
        try:
            text = recognizer.recognize_google(audio).lower().strip()
            if text:
                print(f"\n[MIC 🎤] Heard: '{text}'")
                self.speech_queue.put(text)
        except sr.UnknownValueError:
            pass  # Inaudible / silence
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


# ==========================================
# 4. LiDAR (Autonomous Trigger)
# ==========================================
class JetsonLiDAR:
    def prompt_user(self) -> str:
        time.sleep(5)
        return "SCAN"
