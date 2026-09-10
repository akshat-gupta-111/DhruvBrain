import os
import cv2
import time
import glob
import subprocess
from typing import Optional
import serial
import azure.cognitiveservices.speech as speechsdk

# ==========================================
# 1. Jetson Camera (V4L2 / GStreamer)
# ==========================================

import threading

class JetsonCamera:
    _instance = None
    _lock = threading.Lock()

    def __new__(cls, *args, **kwargs):
        # Singleton pattern: Ensure only one camera object exists
        with cls._lock:
            if cls._instance is None:
                cls._instance = super(JetsonCamera, cls).__new__(cls)
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
            
        self.cap = cv2.VideoCapture(self.camera_index, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        
        self.running = True
        self.thread = threading.Thread(target=self._update, daemon=True)
        self.thread.start()

    def _update(self):
        while self.running:
            if self.cap.isOpened():
                ret, frame = self.cap.read()
                if ret:
                    self.latest_frame = frame

    def capture_frame_bytes(self, quality: int = 85) -> bytes:
        """Called by LangGraph to get the current frame for Moondream."""
        if self.latest_frame is None:
            raise RuntimeError("No frame available.")

        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), quality]
        success, buffer = cv2.imencode(".jpg", self.latest_frame, encode_param)
        return buffer.tobytes()

# =========================================
# 2. Motor & LED Controller (Arduino R4)
# ==========================================
class MotorController:
    def __init__(self, baud_rate: int = 115200):
        self.serial_conn = None
        self._init_serial(baud_rate)

    def _init_serial(self, baud: int):
        """Auto-detects Arduino R4 on Jetson USB ports."""
        # Uno R4 typically mounts as /dev/ttyACM0 or ACM1 on Linux
        ports = glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*")
        if ports:
            try:
                self.serial_conn = serial.Serial(ports[0], baud, timeout=1)
                time.sleep(1.5)  # Allow Arduino bootloader to initialize
                print(f"[HAL Motor] Connected to Arduino on {ports[0]}")
            except Exception as e:
                print(f"[HAL Motor] Failed to open {ports[0]}. Run: sudo chmod a+rw {ports[0]}")
        else:
            print("[HAL Motor] FATAL: No Arduino found on USB.")

    def execute(self, action: str, led_mood: str = "IDLE_WHITE"):
        """Sends a combined motor and LED command over PySerial."""
        action_map = {
            "APPROACH_0.5M":  "<FWD,50,500>",
            "BACKUP_0.5M":    "<BWD,50,500>",
            "PIVOT_LEFT_30":  "<LEFT,40,300>",
            "PIVOT_RIGHT_30": "<RIGHT,40,300>",
            "HALT":           "<STOP>",
            "CONTINUE_WANDER":"<WANDER>"
        }

        motor_cmd = action_map.get(action, "<STOP>")
        # We send both commands separated by a pipe: e.g., <FWD,50,500>|<LED,FLIRT_PINK>\n
        full_payload = f"{motor_cmd}|<LED,{led_mood}>\n"
        
        print(f"[CHASSIS SERIAL OUT] {full_payload.strip()}")

        if self.serial_conn and self.serial_conn.is_open:
            try:
                self.serial_conn.write(full_payload.encode('utf-8'))
                self.serial_conn.flush()
            except Exception as e:
                print(f"[HAL Motor] Serial write error: {e}")

# ==========================================
# 3. Audio Output (Linux ALSA / Azure)
# ==========================================
class Speaker:
    def __init__(self):
        self.speech_key = os.getenv("AZURE_SPEECH_KEY")
        self.speech_region = os.getenv("AZURE_SPEECH_REGION")
        self.synthesizer = None

        if self.speech_key and self.speech_region:
            speech_config = speechsdk.SpeechConfig(subscription=self.speech_key, region=self.speech_region)
            self.synthesizer = speechsdk.SpeechSynthesizer(speech_config=speech_config)
            print("[HAL Speaker] Azure Neural Voice initialized via ALSA.")
        else:
            print("[HAL Speaker] Azure keys missing. Using espeak fallback.")

    def speak(self, text: str):
        if not text.strip(): return
        print(f"[SPEAKER 🎙️] \"{text}\"")

        if self.synthesizer:
            ssml = f"""
            <speak version="1.0" xmlns="http://www.w3.org/2001/10/synthesis" xmlns:mstts="https://www.w3.org/2001/mstts" xml:lang="en-US">
                <voice name="en-US-SaraNeural">
                    <mstts:express-as style="cheerful" styledegree="1.2">
                        <prosody rate="+5%" pitch="+2%">
                            {text}
                        </prosody>
                    </mstts:express-as>
                </voice>
            </speak>
            """
            result = self.synthesizer.speak_ssml_async(ssml).get()
            if result.reason != speechsdk.ResultReason.SynthesizingAudioCompleted:
                self._fallback_speak(text)
        else:
            self._fallback_speak(text)

    def _fallback_speak(self, text: str):
        """Linux fallback using espeak (must apt-get install espeak)."""
        sanitized = text.replace('"', '\\"')
        subprocess.run(["espeak", "-ven+f3", "-s150", sanitized])

# ==========================================
# 4. LiDAR Bridge (ROS 2 / Terminal)
# ==========================================
class JetsonLiDAR:
    @staticmethod
    def wait_for_trigger() -> str:
        """
        In production, this will subscribe to a ROS 2 Nav topic.
        For terminal testing over SSH, it blocks for user input.
        """
        print("\n[LiDAR Node] Scanning environment... (Press Enter to trigger point of interest, 'q' to quit)")
        return input().strip().lower()