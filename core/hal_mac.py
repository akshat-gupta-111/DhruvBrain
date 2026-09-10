import os
import cv2
# import cv2
import threading
from typing import Optional
import time
import subprocess
import glob
# from typing import Optional
# pyrefly: ignore [missing-import]
import azure.cognitiveservices.speech as speechsdk

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
    def __init__(self):
        self.speech_key = os.getenv("AZURE_SPEECH_KEY")
        self.speech_region = os.getenv("AZURE_SPEECH_REGION")
        self.synthesizer = None

        if self.speech_key and self.speech_region:
            speech_config = speechsdk.SpeechConfig(subscription=self.speech_key, region=self.speech_region)
            # We leave voice blank here because SSML will override it
            self.synthesizer = speechsdk.SpeechSynthesizer(speech_config=speech_config)
            print("[HAL Speaker] Azure Neural Voice initialized.")
        else:
            print("[HAL Speaker] Azure keys missing! Falling back to Mac robot voice.")

    def speak(self, text: str):
        if not text.strip(): return
        print(f"[SPEAKER 🎙️] \"{text}\"")

        if self.synthesizer:
            # SSML injects emotion, speeds up the talking rate by 5%, and raises pitch slightly
            ssml = f"""
            <speak version="1.0" xmlns="http://www.w3.org/2001/10/synthesis" xmlns:mstts="https://www.w3.org/2001/mstts" xml:lang="en-US">
                <voice name="en-US-DavisNeural">
                    <mstts:express-as style="cheerful" styledegree="1.5">
                        <prosody rate="+5%" pitch="+0%">
                            {text}
                        </prosody>
                    </mstts:express-as>
                </voice>
            </speak>
            """
            result = self.synthesizer.speak_ssml_async(ssml).get()
            
            if result.reason != speechsdk.ResultReason.SynthesizingAudioCompleted:
                print(f"[HAL Speaker] Azure TTS failed. Falling back.")
                self._fallback_speak(text)
        else:
            self._fallback_speak(text)

    def _fallback_speak(self, text: str):
        sanitized = text.replace('"', '\\"')
        subprocess.run(["say", "-v", "Samantha", sanitized])


# ==========================================
# 4. Mock LiDAR Trigger
# ==========================================
class MockLiDAR:
    @staticmethod
    def prompt_user() -> str:
        """Blocks for a keypress to simulate spatial triggers."""
        print("\n" + "=" * 50)
        print("SIMULATION CONTROLS:")
        print(" [Enter] -> Take snapshot and run Perception Loop")
        print(" [q]     -> Quit")
        print("=" * 50)
        return input("Press [Enter] to scan... ").strip().lower()