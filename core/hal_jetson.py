import queue
import sys
import threading
import time
import os
import glob
import cv2
import serial
import azure.cognitiveservices.speech as speechsdk

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
class Speaker:
    def __init__(self):
        self.speech_key = os.getenv("AZURE_SPEECH_KEY")
        self.speech_region = os.getenv("AZURE_SPEECH_REGION")
        self.synthesizer = None
        if self.speech_key and self.speech_region:
            speech_config = speechsdk.SpeechConfig(subscription=self.speech_key, region=self.speech_region)
            speech_config.speech_synthesis_voice_name = "en-US-GuyNeural"
            # Jetson will route this natively through ALSA to /dev/snd
            self.synthesizer = speechsdk.SpeechSynthesizer(speech_config=speech_config)
            print("[HAL Speaker] Azure Neural Voice initialized (ALSA).")

    def speak(self, text: str):
        print(f"[SPEAKER 🎙️] \"{text}\"")
        if self.synthesizer:
            self.synthesizer.speak_text_async(text).get()

class Microphone:
    _instance = None
    _lock = threading.Lock()

    def __new__(cls, *args, **kwargs):
        with cls._lock:
            if cls._instance is None:
                cls._instance = super(Microphone, cls).__new__(cls)
                cls._instance._initialized = False
            return cls._instance

    def __init__(self):
        if self._initialized: return
        self.speech_key = os.getenv("AZURE_SPEECH_KEY")
        self.speech_region = os.getenv("AZURE_SPEECH_REGION")
        self.speech_queue = queue.Queue()
        self.is_muted = True 
        
        if self.speech_key and self.speech_region:
            speech_config = speechsdk.SpeechConfig(subscription=self.speech_key, region=self.speech_region)
            audio_config = speechsdk.audio.AudioConfig(use_default_microphone=True)
            self.recognizer = speechsdk.SpeechRecognizer(speech_config=speech_config, audio_config=audio_config)
            
            self.recognizer.recognized.connect(self._recognized_cb)
            self.recognizer.start_continuous_recognition_async()
            print("[HAL Mic] 🎤 Jetson ALSA Background Queue Listener active.")
            
        self._initialized = True

    def _recognized_cb(self, evt):
        if self.is_muted: return 
        text = evt.result.text.lower().strip()
        if text:
            print(f"\n[MIC 🎤] Heard: '{text}'")
            self.speech_queue.put(text)

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